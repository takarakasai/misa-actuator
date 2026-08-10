//! Quasi-static characterization over the [`Actuator`] trait.
//!
//! Where [`crate::runner`] identifies *dynamics* (a chirp gives a Bode plot),
//! this module measures *statics*: what the rig costs to hold, what it costs to
//! start moving, how fast it heats, and how torque relates to current.
//!
//! All four runs are motor-agnostic — they speak only the `Actuator` trait — and
//! all four execute inside a [`SafetyLimits`] envelope, because unlike a chirp
//! they intentionally hold torque against a loaded or stalled shaft. Each run
//! returns its samples *plus* an `Option<AbortReason>` rather than an `Err`, so
//! an aborted run still yields the data collected up to that point; that partial
//! data is usually the interesting part.
//!
//! Every run ends by commanding zero torque before disabling. `disable()` alone
//! is not a latching safe state on at least the DAMIAO family — the next control
//! frame re-energizes the motor — so the zero-torque command is what actually
//! leaves the rig quiet.
//!
//! # Which measurement answers what
//!
//! - [`run_load_map`] — holding torque vs position. Finds the equilibrium point
//!   and separates gravity/spring load from friction. Run this first: it tells
//!   you the safe position window and torque budget for the others.
//! - [`run_breakaway`] — torque at which motion starts. Quantifies stiction,
//!   cogging and gearbox drag; asymmetry between directions is meaningful.
//! - [`run_thermal`] — temperature rise while holding a fixed torque. Turns
//!   "this got hot quickly" into a continuous-rating number.
//! - [`run_kt`] — torque vs current, fitted to a torque constant. Needs a driver
//!   that reports current (RobStride, LKMotor); on DAMIAO it can only validate
//!   the motor's own `KT_Value` register.

use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use misa_actuator::realtime::sleep_precise;
use misa_actuator::{Actuator, MotorFeedback, Result, RunMode};

use crate::limits::{AbortReason, Guard, SafetyLimits};

/// Which way a directional measurement pushes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Increasing position / positive torque.
    Positive,
    /// Decreasing position / negative torque.
    Negative,
}

impl Direction {
    /// `+1.0` or `-1.0`, for scaling a magnitude into a signed command.
    pub const fn sign(self) -> f32 {
        match self {
            Direction::Positive => 1.0,
            Direction::Negative => -1.0,
        }
    }

    /// The opposite direction.
    pub const fn flipped(self) -> Self {
        match self {
            Direction::Positive => Direction::Negative,
            Direction::Negative => Direction::Positive,
        }
    }
}

/// One quasi-static observation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    /// Time since the run started (s).
    pub t_s: f32,
    /// What was commanded — a position (rad) or a torque (N·m) depending on run.
    pub cmd: f32,
    pub position_rad: f32,
    pub velocity_rad_per_s: f32,
    pub torque_nm: f32,
    /// `NaN` on drivers that do not report current.
    pub current_a: f32,
    /// `NaN` on drivers that do not report temperature.
    pub temperature_c: f32,
}

impl Point {
    fn from(t_s: f32, cmd: f32, fb: &MotorFeedback) -> Self {
        Self {
            t_s,
            cmd,
            position_rad: fb.position_rad,
            velocity_rad_per_s: fb.velocity_rad_per_s,
            torque_nm: fb.torque_nm,
            current_a: fb.current_a,
            temperature_c: fb.temperature_c,
        }
    }
}

/// The CSV header row emitted by [`write_points_csv`].
pub const POINTS_CSV_HEADER: &str =
    "t_s,cmd,position_rad,velocity_rad_per_s,torque_nm,current_a,temperature_c";

/// Write points as CSV: the header when `write_header`, then one row per point.
///
/// `write_header` exists for runs that append to a writer another run already
/// used — a two-direction breakaway is one file with two ramps in it, and
/// re-emitting the header between them puts a text row in the middle of the
/// data.
pub fn write_points_csv(
    points: &[Point],
    w: &mut dyn Write,
    write_header: bool,
) -> io::Result<()> {
    if write_header {
        writeln!(w, "{POINTS_CSV_HEADER}")?;
    }
    for p in points {
        writeln!(
            w,
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.3}",
            p.t_s,
            p.cmd,
            p.position_rad,
            p.velocity_rad_per_s,
            p.torque_nm,
            p.current_a,
            p.temperature_c
        )?;
    }
    Ok(())
}

// ---------------------------------------------------------------- load map

/// How to sweep position while recording holding torque.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoadMapSpec {
    /// Sweep start, relative to the position at enable (rad).
    pub from_rad: f32,
    /// Sweep end, relative to the position at enable (rad).
    pub to_rad: f32,
    /// Number of positions visited, including both ends (>= 2).
    pub steps: usize,
    /// How long to dwell at each position *after arriving* before recording, so
    /// the reading is the holding torque and not the acceleration transient (s).
    pub settle_s: f32,
    /// How close counts as arrived (rad).
    ///
    /// The dwell only starts once the shaft is this close to the target.
    /// Without an arrival check the first point of a sweep records a travel
    /// transient: reaching `from_rad` can be a long move, and at
    /// [`Self::max_speed_rad_s`] it may take much longer than `settle_s` — seen
    /// on a DM-J4310, where the first point of a ±0.12 rad sweep was logged at
    /// -0.011 rad while still travelling toward -0.05.
    pub arrive_tolerance_rad: f32,
    /// Give up waiting to arrive after this long and record anyway (s).
    ///
    /// A shaft blocked by an end stop never arrives; recording the stalled
    /// torque is the informative outcome, so this bounds the wait rather than
    /// hanging. The [`SafetyLimits`] time budget still applies on top.
    pub travel_timeout_s: f32,
    /// Position-controller speed cap while moving between points (rad/s).
    pub max_speed_rad_s: f32,
    /// Also sweep back to the start, which exposes hysteresis: the difference
    /// between the two passes at the same position is friction, while their
    /// mean is the conservative (gravity/spring) load.
    pub return_sweep: bool,
}

impl LoadMapSpec {
    /// A symmetric sweep of `+/- half_span_rad` around the start position.
    pub fn symmetric(half_span_rad: f32, steps: usize) -> Self {
        Self {
            from_rad: -half_span_rad,
            to_rad: half_span_rad,
            steps: steps.max(2),
            settle_s: 0.3,
            arrive_tolerance_rad: 0.005,
            travel_timeout_s: 3.0,
            max_speed_rad_s: 0.5,
            return_sweep: true,
        }
    }

    /// The commanded offsets, in visit order.
    fn offsets(&self) -> Vec<f32> {
        let n = self.steps.max(2);
        let mut v: Vec<f32> = (0..n)
            .map(|i| self.from_rad + (self.to_rad - self.from_rad) * i as f32 / (n - 1) as f32)
            .collect();
        if self.return_sweep {
            let mut back = v.clone();
            back.pop(); // don't re-measure the turning point
            back.reverse();
            v.extend(back);
        }
        v
    }
}

/// Holding torque as a function of position.
#[derive(Debug, Clone)]
pub struct LoadMap {
    /// One point per visited position, in visit order. Each is the **mean** of
    /// its dwell, not a single sample — see [`Self::dwell_torque_spreads_nm`].
    pub points: Vec<Point>,
    /// Peak-to-peak torque seen during each point's dwell (N·m), parallel to
    /// [`Self::points`].
    ///
    /// Large relative to the torque itself means the shaft was hunting rather
    /// than holding, so that point averages a limit cycle instead of measuring
    /// a steady load. [`Self::worst_dwell_spread_nm`] summarises it.
    pub dwell_torque_spreads_nm: Vec<f32>,
    /// Position at enable, which every `cmd` is relative to (rad).
    pub start_position_rad: f32,
    pub spec: LoadMapSpec,
    pub abort: Option<AbortReason>,
}

impl LoadMap {
    /// Outbound/return point pairs that visited the **same commanded offset**.
    ///
    /// Paired by sweep index rather than by nearest measured position: the visit
    /// order is deterministic, so `back[j]` is the same commanded offset as
    /// `out[n - 2 - j]`. Matching on measured position instead would pair points
    /// at genuinely different offsets whenever one of them is missing, and read
    /// the resulting spring-torque difference as friction.
    ///
    /// The **first** outbound point is excluded, and with it the last return
    /// point that would have paired with it: the move that reaches `from_rad`
    /// travels opposite to the outbound sweep, so friction there acts in the
    /// return direction and pairing it would cancel the very spread being
    /// measured. Both remain in [`Self::points`] — only this split drops them.
    fn pairs(&self) -> Vec<(&Point, &Point)> {
        if !self.spec.return_sweep {
            return Vec::new();
        }
        let n = self.spec.steps.max(2);
        let (Some(out), Some(back)) = (self.points.get(..n), self.points.get(n..)) else {
            return Vec::new();
        };
        let mut pairs = Vec::with_capacity(back.len());
        for (j, b) in back.iter().enumerate() {
            // back[j] revisits offset index n - 2 - j; index 0 has no usable
            // outbound partner (see above).
            let Some(i) = (n - 2).checked_sub(j).filter(|&i| i >= 1) else {
                continue;
            };
            if let Some(o) = out.get(i) {
                pairs.push((o, b));
            }
        }
        pairs
    }

    /// Friction-compensated load curve: `(position, static_load, friction)` per
    /// matched outbound/return pair.
    ///
    /// At a given position the outbound and return passes differ only in which
    /// way friction acts, so their **mean** is the conservative (gravity/spring)
    /// load and **half their difference** is the Coulomb friction. Separating
    /// them is the point of the return sweep: on a DM-J4310 the raw holding
    /// torque read +0.17 N·m outbound and -0.15 N·m on the way back across the
    /// whole span, which is friction of ~0.16 N·m over a static load of
    /// essentially zero — indistinguishable without this pairing.
    ///
    /// Empty unless [`LoadMapSpec::return_sweep`] produced a second pass.
    pub fn static_load_curve(&self) -> Vec<(f32, f32, f32)> {
        let mut curve: Vec<(f32, f32, f32)> = self
            .pairs()
            .into_iter()
            .filter(|(o, b)| o.torque_nm.is_finite() && b.torque_nm.is_finite())
            .map(|(o, b)| {
                (
                    0.5 * (o.position_rad + b.position_rad),
                    0.5 * (o.torque_nm + b.torque_nm),
                    0.5 * (o.torque_nm - b.torque_nm).abs(),
                )
            })
            .collect();
        curve.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(core::cmp::Ordering::Equal));
        curve
    }

    /// Equilibrium taken from the friction-compensated curve — where the
    /// *static* load crosses zero.
    ///
    /// Prefer this over [`Self::equilibrium_position_rad`]: that one minimizes
    /// raw |holding torque|, which on a friction-dominated rig just picks
    /// whichever sample happened to sit lowest in the friction band and moves
    /// around between runs. `None` without a return sweep.
    pub fn static_equilibrium_position_rad(&self) -> Option<f32> {
        let curve = self.static_load_curve();
        curve
            .iter()
            .min_by(|a, b| {
                a.1.abs()
                    .partial_cmp(&b.1.abs())
                    .unwrap_or(core::cmp::Ordering::Equal)
            })
            .map(|&(pos, _, _)| pos)
    }

    /// Mean Coulomb friction over the friction-compensated curve (N·m).
    /// `None` without a return sweep.
    pub fn mean_friction_nm(&self) -> Option<f32> {
        let curve = self.static_load_curve();
        (!curve.is_empty())
            .then(|| curve.iter().map(|&(_, _, f)| f).sum::<f32>() / curve.len() as f32)
    }

    /// Largest |static load| after removing friction (N·m). `None` without a
    /// return sweep.
    ///
    /// Compare against [`Self::mean_friction_nm`]: a peak well below the
    /// friction means the rig has no meaningful gravity/spring term over this
    /// span, so a `load-map` reading is measuring friction and nothing else.
    pub fn peak_static_load_nm(&self) -> Option<f32> {
        self.static_load_curve()
            .iter()
            .map(|&(_, load, _)| load.abs())
            .max_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal))
    }

    /// The position whose **raw** holding torque is smallest in magnitude.
    ///
    /// Only meaningful when the static load dominates friction; otherwise this
    /// is friction noise — see [`Self::static_equilibrium_position_rad`], which
    /// removes friction first.
    pub fn equilibrium_position_rad(&self) -> Option<f32> {
        self.points
            .iter()
            .filter(|p| p.torque_nm.is_finite())
            .min_by(|a, b| {
                a.torque_nm
                    .abs()
                    .partial_cmp(&b.torque_nm.abs())
                    .unwrap_or(core::cmp::Ordering::Equal)
            })
            .map(|p| p.position_rad)
    }

    /// Largest peak-to-peak torque swing seen inside any single dwell (N·m).
    ///
    /// Compare against [`Self::peak_holding_torque_nm`]: a spread of the same
    /// order or larger means the shaft was hunting, so the load figures average
    /// a limit cycle and should not be read as a steady-state load. Seen on an
    /// RS04, whose 0.054 N·m of friction damps almost nothing against a
    /// `loc_kp` of 80.
    pub fn worst_dwell_spread_nm(&self) -> Option<f32> {
        self.dwell_torque_spreads_nm
            .iter()
            .copied()
            .filter(|s| s.is_finite())
            .max_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal))
    }

    /// Largest |holding torque| seen, i.e. what the rig costs at its worst.
    pub fn peak_holding_torque_nm(&self) -> Option<f32> {
        self.points
            .iter()
            .map(|p| p.torque_nm.abs())
            .filter(|t| t.is_finite())
            .max_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal))
    }

    /// Peak hysteresis: the largest torque gap between the outbound and return
    /// passes at (approximately) the same position. Twice the Coulomb friction.
    ///
    /// `None` unless [`LoadMapSpec::return_sweep`] produced a second pass.
    pub fn peak_hysteresis_nm(&self) -> Option<f32> {
        self.pairs()
            .into_iter()
            .map(|(o, b)| (o.torque_nm - b.torque_nm).abs())
            .filter(|g| g.is_finite())
            .max_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal))
    }
}

/// Step through positions and record the torque needed to hold each one.
///
/// Run this before the other measurements: its equilibrium point is where
/// [`run_thermal`] and [`run_kt`] should sit, and its peak holding torque sizes
/// their torque budget.
pub fn run_load_map(
    act: &mut dyn Actuator,
    spec: &LoadMapSpec,
    limits: SafetyLimits,
    abort: &AtomicBool,
) -> Result<LoadMap> {
    act.set_run_mode(RunMode::Position)?;
    let start = act.enable()?.position_rad;
    let mut guard = Guard::new(limits, start);

    let mut points = Vec::with_capacity(spec.offsets().len());
    let mut spreads = Vec::with_capacity(spec.offsets().len());
    let mut stop = None;
    let t0 = Instant::now();

    for offset in spec.offsets() {
        if abort.load(Ordering::Relaxed) {
            stop = Some(AbortReason::Cancelled);
            break;
        }
        let target = start + offset;
        let mut fb = act.set_position(target, spec.max_speed_rad_s)?;

        // Phase 1: travel. Wait until the shaft is actually near the target,
        // otherwise the recorded "holding" torque is a travel transient — the
        // first point of a sweep is the worst case, since reaching `from_rad`
        // can be a long move. A shaft that never arrives (end stop) falls
        // through on the timeout, which is itself the informative reading.
        let travel_deadline = Instant::now() + Duration::from_secs_f32(spec.travel_timeout_s.max(0.0));
        while (fb.position_rad - target).abs() > spec.arrive_tolerance_rad {
            if Instant::now() >= travel_deadline {
                break;
            }
            if abort.load(Ordering::Relaxed) {
                stop = Some(AbortReason::Cancelled);
                break;
            }
            fb = act.set_position(target, spec.max_speed_rad_s)?;
            if let Some(r) = guard.check(t0.elapsed().as_secs_f32(), &fb) {
                stop = Some(r);
                break;
            }
            sleep_precise(Duration::from_millis(5));
        }
        if stop.is_some() {
            points.push(Point::from(t0.elapsed().as_secs_f32(), target, &fb));
            break;
        }

        // Phase 2: settle, averaging the dwell rather than taking its last
        // sample. A stiff position loop against a low-friction load hunts
        // instead of settling — on an RS04 the current alternated sign every
        // sample at ~0.45 A, so a single sample recorded the phase of a limit
        // cycle and called it a holding torque. The mean cancels the cycle; the
        // spread records that it was there.
        let settle = Duration::from_secs_f32(spec.settle_s.max(0.0));
        let dwell_start = Instant::now();
        let mut acc = DwellAccumulator::default();
        acc.push(&fb);
        while dwell_start.elapsed() < settle {
            if abort.load(Ordering::Relaxed) {
                stop = Some(AbortReason::Cancelled);
                break;
            }
            fb = act.set_position(target, spec.max_speed_rad_s)?;
            acc.push(&fb);
            if let Some(r) = guard.check(t0.elapsed().as_secs_f32(), &fb) {
                stop = Some(r);
                break;
            }
            sleep_precise(Duration::from_millis(5));
        }
        spreads.push(acc.torque_spread());
        points.push(acc.into_point(t0.elapsed().as_secs_f32(), target));
        if stop.is_some() {
            break;
        }
    }

    // Return to the start before releasing, so the rig is left where it began.
    let _ = act.set_position(start, spec.max_speed_rad_s);
    quiet(act);

    Ok(LoadMap {
        points,
        dwell_torque_spreads_nm: spreads,
        start_position_rad: start,
        spec: *spec,
        abort: stop,
    })
}

/// Accumulates one dwell so the recorded point is its mean, and the spread it
/// hid is still reportable.
#[derive(Default)]
struct DwellAccumulator {
    n: u32,
    position: f32,
    velocity: f32,
    torque: f32,
    current: f32,
    temperature: f32,
    torque_min: f32,
    torque_max: f32,
}

impl DwellAccumulator {
    fn push(&mut self, fb: &MotorFeedback) {
        // Only finite readings contribute, so a driver that does not report a
        // channel yields NaN for it rather than poisoning the others.
        self.n += 1;
        self.position += fb.position_rad;
        self.velocity += fb.velocity_rad_per_s;
        self.torque += fb.torque_nm;
        self.current += fb.current_a;
        self.temperature += fb.temperature_c;
        if fb.torque_nm.is_finite() {
            if self.torque_min.is_nan() || self.n == 1 {
                self.torque_min = fb.torque_nm;
                self.torque_max = fb.torque_nm;
            } else {
                self.torque_min = self.torque_min.min(fb.torque_nm);
                self.torque_max = self.torque_max.max(fb.torque_nm);
            }
        }
    }

    fn torque_spread(&self) -> f32 {
        if self.n == 0 || !self.torque_min.is_finite() {
            return f32::NAN;
        }
        self.torque_max - self.torque_min
    }

    fn into_point(self, t_s: f32, cmd: f32) -> Point {
        let n = self.n.max(1) as f32;
        Point {
            t_s,
            cmd,
            position_rad: self.position / n,
            velocity_rad_per_s: self.velocity / n,
            torque_nm: self.torque / n,
            current_a: self.current / n,
            temperature_c: self.temperature / n,
        }
    }
}

// --------------------------------------------------------------- breakaway

/// How to ramp torque until the shaft breaks away.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BreakawaySpec {
    /// Torque ramp rate (N·m/s). Slow enough that inertia does not contribute —
    /// this is a *static* friction measurement, so err on the slow side.
    pub ramp_nm_per_s: f32,
    /// Ceiling for the ramp (N·m). Reaching it without motion is a valid,
    /// informative outcome, reported as `breakaway_torque_nm: None`.
    pub max_torque_nm: f32,
    /// Speed above which the shaft counts as moving (rad/s). Must clear the
    /// velocity noise floor — bench DM-J4310 idles around 0.02 rad/s.
    pub motion_threshold_rad_per_s: f32,
    /// Displacement from the start that also counts as motion (rad), as a
    /// backstop for a shaft that creeps below the speed threshold.
    pub motion_threshold_rad: f32,
    /// Which way to push.
    pub direction: Direction,
    /// Command rate (Hz).
    pub rate_hz: f32,
    /// Require the shaft to hold still for this long, at zero torque, before
    /// the ramp starts (s).
    ///
    /// Without this the measurement is worthless whenever the shaft is already
    /// moving: the motion test trips on the very first sample and reports a
    /// breakaway torque of zero. Seen on a DM-J4310 running `--both`, where the
    /// second ramp began while the shaft was still coasting from the first and
    /// duly reported -0.0000 N·m.
    pub rest_window_s: f32,
    /// How long motion must persist before it counts as breakaway (s).
    ///
    /// A single threshold crossing is not breakaway. A rig that judders creeps
    /// back and forth at low torque, and the velocity threshold catches one of
    /// those spikes long before the shaft can actually be driven — measured on
    /// an RS04, where velocity crossed +/-0.08 rad/s in *both* directions while
    /// the command was only 0.12 A, and the reported figure came out three times
    /// below the torque a constant-speed traverse actually needed. Requiring the
    /// motion to hold rejects judder; `0.0` restores the single-crossing
    /// behaviour.
    ///
    /// Keep it short relative to the ramp's own duration
    /// (`max_torque_nm / ramp_nm_per_s`): a hold longer than the ramp means the
    /// ceiling arrives first and no breakaway is ever confirmed.
    pub motion_hold_s: f32,
    /// Give up waiting for stillness after this long and ramp anyway (s).
    ///
    /// A shaft that never settles — a live load, or a rig being touched — still
    /// gets measured, but [`Breakaway::rested`] records that the precondition
    /// did not hold so the number can be discounted.
    pub rest_timeout_s: f32,
}

impl BreakawaySpec {
    /// A slow ramp to `max_torque_nm` in the given direction: 0.2 N·m/s,
    /// 0.05 rad/s and 0.02 rad motion thresholds, 200 Hz.
    pub fn slow(max_torque_nm: f32, direction: Direction) -> Self {
        Self {
            ramp_nm_per_s: 0.2,
            max_torque_nm,
            motion_threshold_rad_per_s: 0.05,
            motion_threshold_rad: 0.02,
            direction,
            rate_hz: 200.0,
            motion_hold_s: 0.1,
            rest_window_s: 0.3,
            rest_timeout_s: 3.0,
        }
    }
}

/// Result of one breakaway ramp.
#[derive(Debug, Clone)]
pub struct Breakaway {
    /// Commanded torque when motion was first detected (N·m, signed). `None`
    /// means the ramp reached its ceiling — or aborted — without the shaft
    /// moving, which bounds the friction from below rather than measuring it.
    pub breakaway_torque_nm: Option<f32>,
    /// Position at which motion started (rad).
    pub breakaway_position_rad: Option<f32>,
    /// The whole ramp, so the torque/velocity curve can be inspected.
    pub points: Vec<Point>,
    /// Whether the shaft actually came to rest before the ramp started. `false`
    /// means [`BreakawaySpec::rest_timeout_s`] expired first, so the breakaway
    /// torque may be an artefact of pre-existing motion.
    pub rested: bool,
    pub spec: BreakawaySpec,
    pub abort: Option<AbortReason>,
}

/// Ramp torque from zero until the shaft starts moving, and report the torque it
/// took.
///
/// Measures stiction plus whatever static load opposes `spec.direction`, so it
/// is most informative run from the equilibrium position found by
/// [`run_load_map`] — elsewhere the gravity/spring term dominates the friction
/// term. Running it in both directions and differencing separates the two.
pub fn run_breakaway(
    act: &mut dyn Actuator,
    spec: &BreakawaySpec,
    limits: SafetyLimits,
    abort: &AtomicBool,
) -> Result<Breakaway> {
    act.set_run_mode(RunMode::Torque)?;
    act.enable()?;

    // Settle first: the motion test that ends the ramp would otherwise trip on
    // motion that was already there, reporting a breakaway torque of zero.
    // Commanding zero torque while waiting also lets a loaded shaft relax to
    // where it actually rests, which is where the measurement belongs.
    let period = Duration::from_secs_f32(1.0 / spec.rate_hz.max(1.0));
    let rest_deadline = Instant::now() + Duration::from_secs_f32(spec.rest_timeout_s.max(0.0));
    let rest_window = Duration::from_secs_f32(spec.rest_window_s.max(0.0));
    let mut still_since: Option<Instant> = None;
    let mut fb = act.set_torque(0.0)?;
    let rested = loop {
        if fb.velocity_rad_per_s.abs() <= spec.motion_threshold_rad_per_s {
            let since = *still_since.get_or_insert_with(Instant::now);
            if since.elapsed() >= rest_window {
                break true;
            }
        } else {
            still_since = None;
        }
        if Instant::now() >= rest_deadline || abort.load(Ordering::Relaxed) {
            break false;
        }
        sleep_precise(period);
        fb = act.set_torque(0.0)?;
    };

    let start = fb.position_rad;
    let mut guard = Guard::new(limits, start);

    let ceiling = spec.max_torque_nm.min(limits.max_torque_nm);
    let mut points = Vec::new();
    let mut stop = None;
    let (mut torque_at_motion, mut position_at_motion) = (None, None);
    // When the current run of motion began: (instant, commanded torque, position).
    let mut motion_since: Option<(Instant, f32, f32)> = None;

    let t0 = Instant::now();
    loop {
        let t = t0.elapsed().as_secs_f32();
        let magnitude = spec.ramp_nm_per_s * t;
        if magnitude > ceiling {
            break;
        }
        if abort.load(Ordering::Relaxed) {
            stop = Some(AbortReason::Cancelled);
            break;
        }
        let iter_start = Instant::now();

        let cmd = magnitude * spec.direction.sign();
        let fb = act.set_torque(cmd)?;
        points.push(Point::from(t, cmd, &fb));

        // Motion must persist: a lone threshold crossing is judder, not
        // breakaway. The torque recorded is the one from when motion *started*,
        // not from when it was confirmed, so the hold does not inflate it.
        let moving = fb.velocity_rad_per_s.abs() > spec.motion_threshold_rad_per_s
            || (fb.position_rad - start).abs() > spec.motion_threshold_rad;
        if moving {
            let (since, at_cmd, at_pos) =
                *motion_since.get_or_insert((Instant::now(), cmd, fb.position_rad));
            if since.elapsed() >= Duration::from_secs_f32(spec.motion_hold_s.max(0.0)) {
                torque_at_motion = Some(at_cmd);
                position_at_motion = Some(at_pos);
                break;
            }
        } else {
            motion_since = None;
        }
        if let Some(r) = guard.check(t, &fb) {
            stop = Some(r);
            break;
        }
        if let Some(rem) = period.checked_sub(iter_start.elapsed()) {
            sleep_precise(rem);
        }
    }

    quiet(act);

    Ok(Breakaway {
        breakaway_torque_nm: torque_at_motion,
        breakaway_position_rad: position_at_motion,
        points,
        rested,
        spec: *spec,
        abort: stop,
    })
}

// ----------------------------------------------------------------- thermal

/// How to hold torque while watching temperature.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalSpec {
    /// Torque to hold (N·m, signed).
    pub hold_torque_nm: f32,
    /// How long to hold (s). The [`SafetyLimits`] time budget still applies.
    pub duration_s: f32,
    /// Sample rate (Hz). A few Hz is plenty — motor temperature sensors are
    /// slow and typically quantized to 1 °C.
    pub rate_hz: f32,
    /// Position "leash" stiffness (N·m/rad) that stops the shaft running away.
    /// `0.0` commands open-loop torque instead.
    ///
    /// **The leash prevents runaway; it does not make a free shaft hold
    /// torque.** At steady state the shaft settles at `err = -tau_ff / kp`,
    /// where the motor produces `kp * err + tau_ff = 0` — the leash cancels the
    /// very feed-forward you asked it to hold. Measured on an RS04 (friction
    /// 0.054 N·m): commanding 2.0 N·m delivered 0.141 N·m, and the run would
    /// otherwise have reported "sustainable" about a torque the motor never
    /// produced.
    ///
    /// So a fixed-torque thermal test needs the output shaft **mechanically
    /// restrained**, or a rig whose own load restrains it — a DM-J4310 against
    /// its end stop did deliver 1.218 of a commanded 1.5 N·m and warmed at
    /// 0.116 °C/s. [`Thermal::delivered_fraction`] reports whether the torque
    /// actually arrived, so a leash-cancelled run is visible rather than
    /// mistaken for a thermal result.
    ///
    /// Needs native MIT control (DAMIAO / RobStride). On LKMotor the gains are
    /// ignored, so the shaft must be restrained regardless.
    pub leash_kp: f32,
    /// Leash damping (N·m·s/rad). Pairs with [`Self::leash_kp`].
    pub leash_kd: f32,
}

/// Temperature response to a held torque.
#[derive(Debug, Clone)]
pub struct Thermal {
    pub points: Vec<Point>,
    pub spec: ThermalSpec,
    pub abort: Option<AbortReason>,
}

impl Thermal {
    /// Least-squares temperature rise rate over the whole hold (°C/s).
    ///
    /// `None` if the driver reports no temperature, or the hold was too short
    /// to fit a slope. Extrapolating it to a temperature limit gives the
    /// allowable hold time at this torque; comparing rates across torques gives
    /// the continuous-rating curve.
    pub fn rise_rate_c_per_s(&self) -> Option<f32> {
        let pts: Vec<(f32, f32)> = self
            .points
            .iter()
            .filter(|p| p.temperature_c.is_finite())
            .map(|p| (p.t_s, p.temperature_c))
            .collect();
        least_squares_slope(&pts)
    }

    /// Mean torque actually delivered during the hold (N·m), which can fall
    /// short of the command once the motor saturates or faults.
    pub fn mean_torque_nm(&self) -> Option<f32> {
        let vals: Vec<f32> = self
            .points
            .iter()
            .map(|p| p.torque_nm)
            .filter(|t| t.is_finite())
            .collect();
        (!vals.is_empty()).then(|| vals.iter().sum::<f32>() / vals.len() as f32)
    }

    /// Fraction of the commanded torque the motor actually delivered.
    ///
    /// `1.0` means the hold was real. Well below 1 means the shaft was free
    /// enough that the leash cancelled the feed-forward (see
    /// [`ThermalSpec::leash_kp`]), so the temperature result describes some
    /// smaller torque — or none — and must not be read as a rating for the
    /// torque that was asked for. `None` if nothing was measured or the command
    /// was zero.
    pub fn delivered_fraction(&self) -> Option<f32> {
        let mean = self.mean_torque_nm()?;
        (self.spec.hold_torque_nm.abs() > 1e-6)
            .then(|| (mean / self.spec.hold_torque_nm).abs())
    }

    /// Seconds until `limit_c` at the measured rise rate, from the last reading.
    /// `None` if the temperature is not rising (or is unknown) — i.e. this
    /// torque is thermally sustainable.
    pub fn seconds_to_limit(&self, limit_c: f32) -> Option<f32> {
        let rate = self.rise_rate_c_per_s()?;
        if rate <= 0.0 {
            return None;
        }
        let last = self
            .points
            .iter()
            .rev()
            .find(|p| p.temperature_c.is_finite())?
            .temperature_c;
        Some(((limit_c - last) / rate).max(0.0))
    }
}

/// Hold a fixed torque and log the temperature rise.
///
/// The guard's rise-rate check is the real protection here; the absolute
/// temperature limit trips too late to be useful on a motor that heats at
/// several °C/s.
pub fn run_thermal(
    act: &mut dyn Actuator,
    spec: &ThermalSpec,
    limits: SafetyLimits,
    abort: &AtomicBool,
) -> Result<Thermal> {
    let leashed = spec.leash_kp > 0.0 || spec.leash_kd > 0.0;
    act.set_run_mode(if leashed {
        RunMode::Mit
    } else {
        RunMode::Torque
    })?;
    let start = act.enable()?.position_rad;
    let mut guard = Guard::new(limits, start);

    let period = Duration::from_secs_f32(1.0 / spec.rate_hz.max(0.1));
    let mut points = Vec::new();
    let mut stop = None;

    let t0 = Instant::now();
    loop {
        let t = t0.elapsed().as_secs_f32();
        if t >= spec.duration_s {
            break;
        }
        if abort.load(Ordering::Relaxed) {
            stop = Some(AbortReason::Cancelled);
            break;
        }
        let iter_start = Instant::now();

        let fb = if leashed {
            act.mit_control(start, 0.0, spec.leash_kp, spec.leash_kd, spec.hold_torque_nm)?
        } else {
            act.set_torque(spec.hold_torque_nm)?
        };
        points.push(Point::from(t, spec.hold_torque_nm, &fb));
        if let Some(r) = guard.check(t, &fb) {
            stop = Some(r);
            break;
        }
        if let Some(rem) = period.checked_sub(iter_start.elapsed()) {
            sleep_precise(rem);
        }
    }

    quiet(act);

    Ok(Thermal {
        points,
        spec: *spec,
        abort: stop,
    })
}

// ---------------------------------------------------------------------- Kt

/// How to sweep torque while recording current.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KtSpec {
    /// Largest |torque| commanded (N·m).
    pub max_torque_nm: f32,
    /// Number of levels from `-max` to `+max`, including zero (>= 3).
    pub steps: usize,
    /// Dwell per level (s), so the current reading is steady-state.
    pub settle_s: f32,
    /// Command rate while dwelling (Hz).
    pub rate_hz: f32,
    /// Position "leash" stiffness (N·m/rad) holding the shaft near where it
    /// started. `0.0` commands open-loop torque.
    ///
    /// Same requirement as [`ThermalSpec::leash_kp`], for the same reason: a
    /// free shaft cannot be held at a torque level long enough to read a
    /// steady-state current. On an RS04 an open-loop -1.5 N·m level left the
    /// +/-0.2 rad window after a single sample.
    pub leash_kp: f32,
    /// Leash damping (N·m·s/rad). Pairs with [`Self::leash_kp`].
    pub leash_kd: f32,
}

impl KtSpec {
    /// A bipolar sweep with 9 levels and a 0.4 s dwell.
    pub fn bipolar(max_torque_nm: f32) -> Self {
        Self {
            max_torque_nm,
            steps: 9,
            settle_s: 0.4,
            rate_hz: 100.0,
            leash_kp: 8.0,
            leash_kd: 0.5,
        }
    }

    fn levels(&self) -> Vec<f32> {
        let n = self.steps.max(3);
        (0..n)
            .map(|i| -self.max_torque_nm + 2.0 * self.max_torque_nm * i as f32 / (n - 1) as f32)
            .collect()
    }
}

/// Torque-vs-current sweep, fitted to a torque constant.
#[derive(Debug, Clone)]
pub struct KtSweep {
    /// One point per level, taken at the end of its dwell.
    pub points: Vec<Point>,
    pub spec: KtSpec,
    pub abort: Option<AbortReason>,
}

impl KtSweep {
    /// Torque constant from a least-squares fit of torque against current
    /// (N·m/A).
    ///
    /// `None` when the driver reports no current — DAMIAO's feedback frame has
    /// no current field, so there it stays `None` and the motor's own
    /// `KT_Value` register is the only source. Where current *is* reported
    /// (RobStride, LKMotor), this is an independent measurement of it.
    pub fn kt_nm_per_a(&self) -> Option<f32> {
        let pts: Vec<(f32, f32)> = self
            .points
            .iter()
            .filter(|p| p.current_a.is_finite() && p.torque_nm.is_finite())
            .map(|p| (p.current_a, p.torque_nm))
            .collect();
        least_squares_slope(&pts)
    }

    /// How linear the torque/current relation was, as `R^2` in `0.0..=1.0`.
    /// Well below 1 means saturation, a fault, or too few usable points — treat
    /// [`Self::kt_nm_per_a`] with suspicion.
    pub fn r_squared(&self) -> Option<f32> {
        let pts: Vec<(f32, f32)> = self
            .points
            .iter()
            .filter(|p| p.current_a.is_finite() && p.torque_nm.is_finite())
            .map(|p| (p.current_a, p.torque_nm))
            .collect();
        let slope = least_squares_slope(&pts)?;
        let n = pts.len() as f32;
        let (mx, my) = (
            pts.iter().map(|p| p.0).sum::<f32>() / n,
            pts.iter().map(|p| p.1).sum::<f32>() / n,
        );
        let intercept = my - slope * mx;
        let (mut ss_res, mut ss_tot) = (0.0f32, 0.0f32);
        for (x, y) in pts {
            ss_res += (y - (slope * x + intercept)).powi(2);
            ss_tot += (y - my).powi(2);
        }
        (ss_tot > 0.0).then(|| (1.0 - ss_res / ss_tot).clamp(0.0, 1.0))
    }

    /// Mean |torque / current| across the non-zero levels (N·m/A) — a
    /// cross-check on the fitted slope that does not assume a zero intercept.
    pub fn pointwise_kt_nm_per_a(&self) -> Option<f32> {
        let ratios: Vec<f32> = self
            .points
            .iter()
            .filter(|p| p.current_a.is_finite() && p.current_a.abs() > 1e-3)
            .map(|p| p.torque_nm / p.current_a)
            .filter(|r| r.is_finite())
            .collect();
        (!ratios.is_empty()).then(|| ratios.iter().sum::<f32>() / ratios.len() as f32)
    }
}

/// Sweep torque through a set of levels and record the resulting current.
///
/// Each level is held for `spec.settle_s` and sampled at the end, so the
/// recorded current is steady-state rather than the inrush.
pub fn run_kt(
    act: &mut dyn Actuator,
    spec: &KtSpec,
    limits: SafetyLimits,
    abort: &AtomicBool,
) -> Result<KtSweep> {
    let leashed = spec.leash_kp > 0.0 || spec.leash_kd > 0.0;
    act.set_run_mode(if leashed {
        RunMode::Mit
    } else {
        RunMode::Torque
    })?;
    let start = act.enable()?.position_rad;
    let mut guard = Guard::new(limits, start);

    let period = Duration::from_secs_f32(1.0 / spec.rate_hz.max(1.0));
    let mut points = Vec::new();
    let mut stop = None;
    let t0 = Instant::now();
    // Hold near `start` while each level is applied; an unleashed shaft simply
    // accelerates away and no level ever reaches steady state.
    let drive = |act: &mut dyn Actuator, level: f32| -> Result<MotorFeedback> {
        if leashed {
            act.mit_control(start, 0.0, spec.leash_kp, spec.leash_kd, level)
        } else {
            act.set_torque(level)
        }
    };

    for level in spec.levels() {
        if abort.load(Ordering::Relaxed) {
            stop = Some(AbortReason::Cancelled);
            break;
        }
        let dwell_start = Instant::now();
        let settle = Duration::from_secs_f32(spec.settle_s.max(0.0));
        let mut fb = drive(act, level)?;
        while dwell_start.elapsed() < settle {
            if abort.load(Ordering::Relaxed) {
                stop = Some(AbortReason::Cancelled);
                break;
            }
            fb = drive(act, level)?;
            if let Some(r) = guard.check(t0.elapsed().as_secs_f32(), &fb) {
                stop = Some(r);
                break;
            }
            sleep_precise(period);
        }
        points.push(Point::from(t0.elapsed().as_secs_f32(), level, &fb));
        if stop.is_some() {
            break;
        }
    }

    quiet(act);

    Ok(KtSweep {
        points,
        spec: *spec,
        abort: stop,
    })
}

// ------------------------------------------------------- breakaway map (C)

/// How to run a breakaway ramp at each of several positions.
///
/// This is the load map done entirely under **torque** control. `load-map`
/// measures the torque a *position loop* emits to hold a setpoint, so the
/// reading is the controller's own output and inherits its dynamics — on a rig
/// that damps poorly the loop hunts and the measurement becomes the phase of a
/// limit cycle (seen on an RS04: `loc_kp` 80 against 0.054 N·m of friction).
///
/// Here the position loop is only a *means of travel*; whatever it does while
/// getting there is discarded. The measurement itself is a torque ramp judged by
/// velocity, which needs no torque feedback at all — so it also works on
/// firmware whose `MeasuredTorque` is broken.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BreakawayMapSpec {
    /// First position, relative to the position at enable (rad).
    pub from_rad: f32,
    /// Last position, relative to the position at enable (rad).
    pub to_rad: f32,
    /// Positions visited (>= 1).
    pub steps: usize,
    /// Speed cap while travelling between positions (rad/s).
    pub travel_speed_rad_s: f32,
    /// How close counts as arrived (rad).
    pub arrive_tolerance_rad: f32,
    /// Give up travelling after this long and ramp anyway (s).
    pub travel_timeout_s: f32,
    /// The ramp performed at each position. Its `direction` is overridden per
    /// pass when [`Self::both_directions`] is set.
    pub ramp: BreakawaySpec,
    /// Ramp both ways at each position, which is what separates the static load
    /// (their mean) from the stiction (half their spread).
    pub both_directions: bool,
}

impl BreakawayMapSpec {
    /// A symmetric span of `+/- half_span_rad` around the start position.
    pub fn symmetric(half_span_rad: f32, steps: usize, ramp: BreakawaySpec) -> Self {
        Self {
            from_rad: -half_span_rad,
            to_rad: half_span_rad,
            steps: steps.max(1),
            travel_speed_rad_s: 0.3,
            arrive_tolerance_rad: 0.01,
            travel_timeout_s: 3.0,
            ramp,
            both_directions: true,
        }
    }

    fn offsets(&self) -> Vec<f32> {
        let n = self.steps.max(1);
        if n == 1 {
            return vec![self.from_rad];
        }
        (0..n)
            .map(|i| self.from_rad + (self.to_rad - self.from_rad) * i as f32 / (n - 1) as f32)
            .collect()
    }
}

/// One position's worth of breakaway measurements.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BreakawayMapPoint {
    /// Position the ramps started from, as measured (rad).
    pub position_rad: f32,
    /// Breakaway torque pushing positive (N·m). `None` if it never moved.
    pub positive_nm: Option<f32>,
    /// Breakaway torque pushing negative (N·m). `None` if it never moved, or if
    /// only one direction was requested.
    pub negative_nm: Option<f32>,
    /// Whether every ramp at this position started from rest. `false` makes the
    /// figures suspect — see [`BreakawaySpec::rest_window_s`].
    pub rested: bool,
}

impl BreakawayMapPoint {
    /// Conservative (gravity/spring) load: the mean of the two directions, in
    /// which stiction cancels. `None` without both directions.
    pub fn static_load_nm(&self) -> Option<f32> {
        Some((self.positive_nm? + self.negative_nm?) / 2.0)
    }

    /// Stiction: half the spread between the directions, in which the static
    /// load cancels. `None` without both directions.
    pub fn stiction_nm(&self) -> Option<f32> {
        Some((self.positive_nm? - self.negative_nm?).abs() / 2.0)
    }
}

/// Load and stiction against position, measured under torque control.
#[derive(Debug, Clone)]
pub struct BreakawayMap {
    pub points: Vec<BreakawayMapPoint>,
    /// Every ramp sample, concatenated in visit order, for the raw log.
    pub samples: Vec<Point>,
    pub start_position_rad: f32,
    pub spec: BreakawayMapSpec,
    pub abort: Option<AbortReason>,
}

impl BreakawayMap {
    /// Mean stiction across the positions that yielded one (N·m).
    pub fn mean_stiction_nm(&self) -> Option<f32> {
        let v: Vec<f32> = self.points.iter().filter_map(|p| p.stiction_nm()).collect();
        (!v.is_empty()).then(|| v.iter().sum::<f32>() / v.len() as f32)
    }

    /// Largest |static load| across the positions (N·m).
    ///
    /// Compare against [`Self::mean_stiction_nm`]: well below it means this span
    /// has no meaningful gravity/spring term, so the rig is friction-dominated.
    pub fn peak_static_load_nm(&self) -> Option<f32> {
        self.points
            .iter()
            .filter_map(|p| p.static_load_nm())
            .map(f32::abs)
            .max_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal))
    }

    /// Position whose |static load| is smallest — the load's equilibrium.
    pub fn equilibrium_position_rad(&self) -> Option<f32> {
        self.points
            .iter()
            .filter(|p| p.static_load_nm().is_some())
            .min_by(|a, b| {
                a.static_load_nm()
                    .unwrap()
                    .abs()
                    .partial_cmp(&b.static_load_nm().unwrap().abs())
                    .unwrap_or(core::cmp::Ordering::Equal)
            })
            .map(|p| p.position_rad)
    }

    /// Positions where a ramp did not start from rest, so their figures are
    /// suspect.
    pub fn unrested_positions(&self) -> Vec<f32> {
        self.points
            .iter()
            .filter(|p| !p.rested)
            .map(|p| p.position_rad)
            .collect()
    }
}

/// Ramp torque to breakaway at each of several positions.
///
/// Prefer this over [`run_load_map`] when the position loop cannot hold still —
/// it needs neither a settled dwell nor working torque feedback. The cost is
/// time: every position pays a full ramp per direction.
pub fn run_breakaway_map(
    act: &mut dyn Actuator,
    spec: &BreakawayMapSpec,
    limits: SafetyLimits,
    abort: &AtomicBool,
) -> Result<BreakawayMap> {
    let start = {
        act.set_run_mode(RunMode::Position)?;
        act.enable()?.position_rad
    };
    let mut points = Vec::with_capacity(spec.steps);
    let mut samples = Vec::new();
    let mut stop = None;

    for offset in spec.offsets() {
        if abort.load(Ordering::Relaxed) {
            stop = Some(AbortReason::Cancelled);
            break;
        }
        let target = start + offset;

        // Travel under position control. Hunting here is irrelevant: nothing is
        // recorded until the ramp begins, and the ramp is torque-controlled.
        //
        // Re-enable first: each `run_breakaway` ends by commanding zero torque
        // and disabling, so without this the second position onward would issue
        // position commands to a de-energised motor and never travel. Measured
        // on an RS04, where all five positions came back within 0.003 rad of the
        // first.
        act.set_run_mode(RunMode::Position)?;
        act.enable()?;
        let travel_deadline =
            Instant::now() + Duration::from_secs_f32(spec.travel_timeout_s.max(0.0));
        let mut fb = act.set_position(target, spec.travel_speed_rad_s)?;
        while (fb.position_rad - target).abs() > spec.arrive_tolerance_rad {
            if Instant::now() >= travel_deadline || abort.load(Ordering::Relaxed) {
                break;
            }
            fb = act.set_position(target, spec.travel_speed_rad_s)?;
            sleep_precise(Duration::from_millis(5));
        }

        let dirs: Vec<Direction> = if spec.both_directions {
            vec![spec.ramp.direction, spec.ramp.direction.flipped()]
        } else {
            vec![spec.ramp.direction]
        };

        let mut positive_nm = None;
        let mut negative_nm = None;
        let mut rested = true;
        let mut measured_at = fb.position_rad;

        for d in dirs {
            let ramp_spec = BreakawaySpec {
                direction: d,
                ..spec.ramp
            };
            let r = run_breakaway(act, &ramp_spec, limits, abort)?;
            rested &= r.rested;
            if let Some(p) = r.points.first() {
                measured_at = p.position_rad;
            }
            samples.extend(r.points);
            match d {
                Direction::Positive => positive_nm = r.breakaway_torque_nm,
                Direction::Negative => negative_nm = r.breakaway_torque_nm,
            }
            if let Some(reason) = r.abort {
                stop = Some(reason);
                break;
            }
        }

        points.push(BreakawayMapPoint {
            position_rad: measured_at,
            positive_nm,
            negative_nm,
            rested,
        });
        if stop.is_some() {
            break;
        }
    }

    // Leave the rig where it began.
    let _ = act.set_run_mode(RunMode::Position);
    let _ = act.set_position(start, spec.travel_speed_rad_s);
    quiet(act);

    Ok(BreakawayMap {
        points,
        samples,
        start_position_rad: start,
        spec: *spec,
        abort: stop,
    })
}

// ------------------------------------------------------- velocity sweep (B)

/// How to traverse at constant speed while logging torque.
///
/// The other way to keep a hunting position loop out of the measurement: rather
/// than holding setpoints, keep the shaft *moving* slowly and record torque
/// continuously. A velocity loop is naturally damped, and in steady motion
///
/// ```text
/// moving + : tau = load(theta) + kinetic friction
/// moving - : tau = load(theta) - kinetic friction
/// ```
///
/// so the same mean/half-difference decomposition applies, densely and without
/// any settling. This measures **kinetic** friction, where
/// [`run_breakaway_map`] measures **static** — running both is how the two are
/// compared (a DM-J4310 showed 0.126 against 0.175).
///
/// Needs working torque feedback, unlike the breakaway map.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocitySweepSpec {
    /// Traverse speed (rad/s). Slow enough that inertia does not contribute.
    pub speed_rad_s: f32,
    /// Sweep `start +/- this` (rad).
    pub half_span_rad: f32,
    /// Log rate (Hz).
    pub rate_hz: f32,
    /// Also sweep back, which is what makes the friction separable.
    pub return_sweep: bool,
    /// Travel to discard at the start of each measured leg (rad).
    ///
    /// Each leg begins by commanding full speed *against* the direction the
    /// shaft is already moving, so it has to decelerate, stop and reverse before
    /// anything it reports is steady motion. Logging that transient put it in
    /// the position bins at the ends of the span — and at opposite ends for the
    /// two legs, since each reverses where the other finished.
    ///
    /// The damage was not noise. In the lowest bins the `+` leg is a transient
    /// while the `−` leg is steady, so the half-difference subtracted one from
    /// the other and read friction low; the half-sum kept the residue, with
    /// opposite signs at the two ends. On an RS-04 that showed as friction
    /// sagging from 0.60 to 0.49 N·m in the end bins and a load curve running
    /// −0.08 to +0.115 N·m — which reads exactly like a spring term and is not
    /// one (2026-08-07).
    ///
    /// Judged by **position**, not by reported velocity: some RobStride firmware
    /// reports velocity as an exact zero in most frames, so "has it reached
    /// speed" is not a question its feedback can answer.
    ///
    /// Costs coverage at the ends of the span — those bins genuinely were not
    /// measured in steady motion, so losing them is the correction.
    pub lead_in_rad: f32,
}

impl VelocitySweepSpec {
    /// Fraction of the half-span discarded at the start of each measured leg.
    ///
    /// A fraction rather than a distance because the reversal distance depends
    /// on an inertia nobody here knows; this at least scales with the sweep and
    /// stays bounded. Generous enough to cover a reversal at these speeds while
    /// leaving most of the span measured.
    pub const LEAD_IN_FRACTION: f32 = 0.15;

    /// The default lead-in for a given half-span (rad).
    pub fn lead_in_for(half_span_rad: f32) -> f32 {
        half_span_rad.abs() * Self::LEAD_IN_FRACTION
    }

    /// A slow symmetric traverse: 0.05 rad/s at 200 Hz, both directions.
    pub fn slow(half_span_rad: f32) -> Self {
        Self {
            speed_rad_s: 0.05,
            half_span_rad,
            rate_hz: 200.0,
            return_sweep: true,
            lead_in_rad: Self::lead_in_for(half_span_rad),
        }
    }
}

/// Continuous torque-vs-position log from a constant-speed traverse.
#[derive(Debug, Clone)]
pub struct VelocitySweep {
    /// Samples in order. `cmd` is the commanded velocity, so its sign says which
    /// pass a sample belongs to.
    pub points: Vec<Point>,
    pub start_position_rad: f32,
    pub spec: VelocitySweepSpec,
    pub abort: Option<AbortReason>,
}

impl VelocitySweep {
    /// Load and kinetic friction against position, from binning the two passes.
    ///
    /// Returns `(position, static_load, kinetic_friction)` per bin that both
    /// passes visited. Bins are uniform over the swept range; `bins` below 2 is
    /// treated as 2.
    pub fn friction_curve(&self, bins: usize) -> Vec<(f32, f32, f32)> {
        let n = bins.max(2);
        let usable: Vec<&Point> = self
            .points
            .iter()
            .filter(|p| p.position_rad.is_finite() && p.torque_nm.is_finite() && p.cmd != 0.0)
            .collect();
        let (Some(lo), Some(hi)) = (
            usable
                .iter()
                .map(|p| p.position_rad)
                .min_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal)),
            usable
                .iter()
                .map(|p| p.position_rad)
                .max_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal)),
        ) else {
            return Vec::new();
        };
        // A degenerate range means the shaft never traversed, so there is
        // nothing to bin. `<=` rather than `!(>)` so NaN also bails out.
        if hi <= lo || !(hi - lo).is_finite() {
            return Vec::new();
        }
        // (sum, count) per direction per bin.
        let mut fwd = vec![(0.0f32, 0u32); n];
        let mut rev = vec![(0.0f32, 0u32); n];
        for p in &usable {
            let t = ((p.position_rad - lo) / (hi - lo) * n as f32) as usize;
            let idx = t.min(n - 1);
            let slot = if p.cmd > 0.0 { &mut fwd } else { &mut rev };
            slot[idx].0 += p.torque_nm;
            slot[idx].1 += 1;
        }
        (0..n)
            .filter_map(|i| {
                let (fs, fc) = fwd[i];
                let (rs, rc) = rev[i];
                if fc == 0 || rc == 0 {
                    return None;
                }
                let (f, r) = (fs / fc as f32, rs / rc as f32);
                let pos = lo + (hi - lo) * (i as f32 + 0.5) / n as f32;
                Some((pos, (f + r) / 2.0, (f - r).abs() / 2.0))
            })
            .collect()
    }

    /// Mean kinetic friction over the curve (N·m).
    pub fn mean_kinetic_friction_nm(&self, bins: usize) -> Option<f32> {
        let c = self.friction_curve(bins);
        (!c.is_empty()).then(|| c.iter().map(|&(_, _, f)| f).sum::<f32>() / c.len() as f32)
    }

    /// Largest |static load| over the curve (N·m).
    pub fn peak_static_load_nm(&self, bins: usize) -> Option<f32> {
        self.friction_curve(bins)
            .iter()
            .map(|&(_, l, _)| l.abs())
            .max_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal))
    }

    /// Mean of the speed the motor **reported**, while commanded to move (rad/s).
    ///
    /// Not the same thing as the speed it went — see
    /// [`Self::traversed_speed_rad_s`], and prefer that for judging a stall.
    /// Measured on an RS-03 (2026-08-06): this reads 0.0085 while the shaft
    /// travels at the commanded 0.05, because the firmware reports velocity as an
    /// exact zero in most frames. Kept because a disagreement between the two is
    /// worth reporting — it says the velocity field cannot be trusted.
    pub fn mean_speed_rad_s(&self) -> Option<f32> {
        let v: Vec<f32> = self
            .points
            .iter()
            .filter(|p| p.cmd != 0.0 && p.velocity_rad_per_s.is_finite())
            .map(|p| p.velocity_rad_per_s.abs())
            .collect();
        (!v.is_empty()).then(|| v.iter().sum::<f32>() / v.len() as f32)
    }

    /// Speed from the positions actually visited (rad/s).
    ///
    /// The distance travelled — summed over consecutive samples, so a there-and-back
    /// traverse counts both legs — divided by the time spent under command.
    ///
    /// **This is what a stall check should use.** Position is the one field this
    /// measurement can rely on; the reported velocity is a constant zero on some
    /// firmware, and judging a stall by it condemns every run on such a motor
    /// while the shaft is visibly turning at the commanded speed.
    ///
    /// Far below [`VelocitySweepSpec::speed_rad_s`] means the traverse really did
    /// stall, so the log is not steady motion and the decomposition does not hold.
    pub fn traversed_speed_rad_s(&self) -> Option<f32> {
        let moving: Vec<&Point> = self
            .points
            .iter()
            .filter(|p| p.cmd != 0.0 && p.position_rad.is_finite() && p.t_s.is_finite())
            .collect();
        if moving.len() < 2 {
            return None;
        }
        let distance: f32 = moving
            .windows(2)
            .map(|w| (w[1].position_rad - w[0].position_rad).abs())
            .sum();
        let elapsed = moving[moving.len() - 1].t_s - moving[0].t_s;
        (elapsed > 0.0).then_some(distance / elapsed)
    }
}

/// Traverse at constant speed in each direction, logging torque throughout.
pub fn run_velocity_sweep(
    act: &mut dyn Actuator,
    spec: &VelocitySweepSpec,
    limits: SafetyLimits,
    abort: &AtomicBool,
) -> Result<VelocitySweep> {
    act.set_run_mode(RunMode::Velocity)?;
    let start = act.enable()?.position_rad;
    let mut guard = Guard::new(limits, start);

    let period = Duration::from_secs_f32(1.0 / spec.rate_hz.max(1.0));
    let speed = spec.speed_rad_s.abs().max(1e-4);
    let (lo, hi) = (start - spec.half_span_rad, start + spec.half_span_rad);
    let mut points = Vec::new();
    let t0 = Instant::now();

    // Each leg drives until the bound is crossed. `log` marks whether the leg's
    // samples are part of the measurement or just positioning.
    let lead_in = spec.lead_in_rad.max(0.0);
    let leg = |act: &mut dyn Actuator,
                   points: &mut Vec<Point>,
                   guard: &mut Guard,
                   vel: f32,
                   done: &dyn Fn(f32) -> bool,
                   log: bool|
     -> Result<Option<AbortReason>> {
        // Where this leg began, so the lead-in can be measured from it. Taken
        // from the first feedback rather than from the bound the previous leg
        // aimed at, which it overshoots by however far one period carries it.
        let mut origin: Option<f32> = None;
        loop {
            let t = t0.elapsed().as_secs_f32();
            if abort.load(Ordering::Relaxed) {
                return Ok(Some(AbortReason::Cancelled));
            }
            let iter_start = Instant::now();
            let fb = act.set_velocity(vel)?;
            if fb.position_rad.is_finite() {
                origin.get_or_insert(fb.position_rad);
            }
            // Discard the reversal: this leg is fighting the momentum the last
            // one left, and nothing it reports is steady motion until the shaft
            // has actually travelled. A rig whose position never becomes finite
            // logs nothing, which is correct — there is no measurement there.
            let past_lead_in = origin.is_some_and(|o| {
                fb.position_rad.is_finite() && (fb.position_rad - o).abs() >= lead_in
            });
            if log && past_lead_in {
                points.push(Point::from(t, vel, &fb));
            }
            if let Some(r) = guard.check(t, &fb) {
                return Ok(Some(r));
            }
            if done(fb.position_rad) {
                return Ok(None);
            }
            if let Some(rem) = period.checked_sub(iter_start.elapsed()) {
                sleep_precise(rem);
            }
        }
    };

    // Positioning leg to the low end (not logged), then the two measured legs so
    // both directions cover the same range.
    let mut stop = leg(act, &mut points, &mut guard, -speed, &|p| p <= lo, false)?;
    if stop.is_none() {
        stop = leg(act, &mut points, &mut guard, speed, &|p| p >= hi, true)?;
    }
    if stop.is_none() && spec.return_sweep {
        stop = leg(act, &mut points, &mut guard, -speed, &|p| p <= lo, true)?;
    }

    // Return to where the sweep began, unlogged. Without this each run ends at
    // `lo` and a repeated series walks one span per repetition in that
    // direction — a DM-J4310 marched from -0.42 to roughly -1.17 rad over three
    // repetitions and hit its end stop, aborting the series on torque.
    let back_toward_start: &dyn Fn(f32) -> bool = if start >= lo {
        &|p| p >= start
    } else {
        &|p| p <= start
    };
    let toward = if start >= lo { speed } else { -speed };
    if stop.is_none() {
        stop = leg(
            act,
            &mut points,
            &mut guard,
            toward,
            back_toward_start,
            false,
        )?;
    }

    let _ = act.set_velocity(0.0);
    quiet(act);

    Ok(VelocitySweep {
        points,
        start_position_rad: start,
        spec: *spec,
        abort: stop,
    })
}

// ------------------------------------------------------------------ shared

/// Command zero torque, then disable.
///
/// The zero-torque command is the part that matters: `disable()` is not a
/// latching safe state on the DAMIAO family (the next control frame
/// re-energizes the motor), so leaving a non-zero command as the last thing on
/// the wire would arm the rig for whatever runs next.
fn quiet(act: &mut dyn Actuator) {
    let _ = act.set_torque(0.0);
    let _ = act.disable();
}

/// Least-squares slope of `y` on `x`. `None` for fewer than two points or no
/// spread in `x`.
fn least_squares_slope(pts: &[(f32, f32)]) -> Option<f32> {
    if pts.len() < 2 {
        return None;
    }
    let n = pts.len() as f32;
    let mx = pts.iter().map(|p| p.0).sum::<f32>() / n;
    let my = pts.iter().map(|p| p.1).sum::<f32>() / n;
    let mut num = 0.0f32;
    let mut den = 0.0f32;
    for &(x, y) in pts {
        num += (x - mx) * (y - my);
        den += (x - mx) * (x - mx);
    }
    (den > 0.0).then_some(num / den)
}

#[cfg(test)]
mod tests {
    use super::*;
    use misa_actuator::{Error, MotorStatus};

    /// A rig with the characteristics these runs are meant to recover, so each
    /// measurement can be checked against planted ground truth:
    ///
    /// - a spring load `k * (pos - eq)` pulling toward `eq`,
    /// - Coulomb friction `mu` that must be overcome before anything moves,
    /// - a torque constant `kt` mapping commanded torque to reported current,
    /// - heating proportional to commanded torque.
    ///
    /// Position control is modelled as an ideal servo that reaches the target
    /// when the commanded torque can overcome the spring and friction, and
    /// reports the torque it is holding.
    struct Rig {
        eq: f32,
        k: f32,
        mu: f32,
        kt: f32,
        heat_c_per_nm_s: f32,
        pos: f32,
        vel: f32,
        temp: f32,
        last_torque: f32,
        /// Wall-clock-free heating: each command advances this much.
        dt: f32,
    }

    impl Rig {
        fn new(eq: f32, k: f32, mu: f32, kt: f32) -> Self {
            Self {
                eq,
                k,
                mu,
                kt,
                heat_c_per_nm_s: 0.0,
                pos: eq,
                vel: 0.0,
                temp: 30.0,
                last_torque: 0.0,
                dt: 0.005,
            }
        }
        fn heating(mut self, c_per_nm_s: f32) -> Self {
            self.heat_c_per_nm_s = c_per_nm_s;
            self
        }
        /// Spring torque opposing displacement from equilibrium.
        fn spring(&self) -> f32 {
            -self.k * (self.pos - self.eq)
        }
        fn fb(&self) -> MotorFeedback {
            MotorFeedback {
                position_rad: self.pos,
                velocity_rad_per_s: self.vel,
                torque_nm: self.last_torque,
                current_a: if self.kt > 0.0 {
                    self.last_torque / self.kt
                } else {
                    f32::NAN
                },
                temperature_c: self.temp,
            }
        }
        fn advance_heat(&mut self, torque: f32) {
            self.temp += self.heat_c_per_nm_s * torque.abs() * self.dt;
        }
    }

    impl Actuator for Rig {
        fn motor_id(&self) -> u8 {
            1
        }
        fn enable(&mut self) -> Result<MotorFeedback> {
            Ok(self.fb())
        }
        fn disable(&mut self) -> Result<()> {
            Ok(())
        }
        fn set_zero(&mut self) -> Result<()> {
            Ok(())
        }
        fn set_run_mode(&mut self, _m: RunMode) -> Result<()> {
            Ok(())
        }
        fn set_position(&mut self, pos_rad: f32, _v: f32) -> Result<MotorFeedback> {
            // Ideal servo: it gets there, and holds against the spring.
            self.pos = pos_rad;
            self.vel = 0.0;
            self.last_torque = -self.spring();
            self.advance_heat(self.last_torque);
            Ok(self.fb())
        }
        fn set_velocity(&mut self, _v: f32) -> Result<MotorFeedback> {
            Err(Error::Unsupported("rig: velocity"))
        }
        fn set_torque(&mut self, torque_nm: f32) -> Result<MotorFeedback> {
            self.last_torque = torque_nm;
            // Net of the spring; motion only once friction is overcome.
            let net = torque_nm + self.spring();
            if net.abs() > self.mu {
                self.vel = (net.abs() - self.mu) * net.signum();
                self.pos += self.vel * self.dt;
            } else {
                self.vel = 0.0;
            }
            self.advance_heat(torque_nm);
            Ok(self.fb())
        }
        fn mit_control(
            &mut self,
            p: f32,
            _v: f32,
            kp: f32,
            _kd: f32,
            tau: f32,
        ) -> Result<MotorFeedback> {
            self.set_torque(kp * (p - self.pos) + tau)
        }
        fn measure(&mut self) -> Result<MotorFeedback> {
            Ok(self.fb())
        }
        fn read_status(&mut self) -> Result<MotorStatus> {
            Ok(MotorStatus {
                voltage_v: 24.0,
                temperature_c: self.temp,
                error: Default::default(),
            })
        }
    }

    fn no_abort() -> AtomicBool {
        AtomicBool::new(false)
    }

    fn roomy() -> SafetyLimits {
        SafetyLimits {
            max_torque_nm: 50.0,
            max_temperature_c: 200.0,
            max_temperature_rise_c_per_s: 1000.0,
            position_window_rad: 10.0,
            max_duration_s: 60.0,
        }
    }

    /// The load map must find the planted equilibrium and report a holding
    /// torque that grows with displacement (`k * offset`).
    #[test]
    fn load_map_recovers_the_spring_equilibrium_and_stiffness() {
        // eq = 0 relative to start, k = 4 N·m/rad, no friction.
        let mut rig = Rig::new(0.0, 4.0, 0.0, 0.5);
        let spec = LoadMapSpec {
            settle_s: 0.0,
            return_sweep: false,
            ..LoadMapSpec::symmetric(0.5, 5)
        };
        let map = run_load_map(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        assert_eq!(map.abort, None);
        assert_eq!(map.points.len(), 5);

        // Equilibrium is where |holding torque| is least.
        let eq = map.equilibrium_position_rad().unwrap();
        assert!(eq.abs() < 1e-3, "equilibrium at {eq}");

        // Holding torque = k * offset, so the peak is at the sweep end.
        let peak = map.peak_holding_torque_nm().unwrap();
        assert!((peak - 4.0 * 0.5).abs() < 1e-3, "peak {peak}");

        // And it must be monotonic in |offset|.
        for p in &map.points {
            let expected = 4.0 * p.position_rad;
            assert!(
                (p.torque_nm - expected).abs() < 1e-3,
                "at {}: {} vs {}",
                p.position_rad,
                p.torque_nm,
                expected
            );
        }
    }

    /// With `return_sweep`, the outbound and return passes are both recorded so
    /// hysteresis can be computed. A frictionless rig must show none.
    #[test]
    fn load_map_return_sweep_reports_no_hysteresis_without_friction() {
        let mut rig = Rig::new(0.0, 4.0, 0.0, 0.5);
        let spec = LoadMapSpec {
            settle_s: 0.0,
            ..LoadMapSpec::symmetric(0.3, 4)
        };
        let map = run_load_map(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        // 4 out + 3 back (turning point not re-measured).
        assert_eq!(map.points.len(), 7);
        let h = map.peak_hysteresis_nm().unwrap();
        assert!(h < 1e-3, "unexpected hysteresis {h}");
    }

    /// Breakaway must report the friction level: starting at equilibrium the
    /// spring contributes nothing, so motion begins just past `mu`.
    #[test]
    fn breakaway_recovers_the_planted_friction() {
        let mu = 0.4;
        let mut rig = Rig::new(0.0, 1.0, mu, 0.5);
        let spec = BreakawaySpec {
            ramp_nm_per_s: 20.0, // fast, to keep the test short
            rate_hz: 2000.0,
            // This rig breaks away cleanly, so the sustained-motion hold is not
            // what is under test — and at 20 N·m/s the ramp would end before the
            // default hold elapsed.
            motion_hold_s: 0.0,
            ..BreakawaySpec::slow(2.0, Direction::Positive)
        };
        let r = run_breakaway(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        assert_eq!(r.abort, None);
        let t = r.breakaway_torque_nm.expect("should have moved");
        assert!(t > 0.0, "positive direction must give positive torque");
        assert!(
            (t - mu).abs() < 0.15,
            "breakaway {t} should be near friction {mu}"
        );
    }

    /// The negative direction must break away at a mirrored torque.
    #[test]
    fn breakaway_is_signed_by_direction() {
        let mut rig = Rig::new(0.0, 1.0, 0.4, 0.5);
        let spec = BreakawaySpec {
            ramp_nm_per_s: 20.0,
            rate_hz: 2000.0,
            motion_hold_s: 0.0, // clean breakaway; see the positive-direction test
            ..BreakawaySpec::slow(2.0, Direction::Negative)
        };
        let r = run_breakaway(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        let t = r.breakaway_torque_nm.expect("should have moved");
        assert!(t < 0.0, "negative direction must give negative torque");
    }

    /// A shaft that never moves is a valid outcome, not an error: the ramp hits
    /// its ceiling and reports no breakaway torque.
    #[test]
    fn breakaway_reports_none_when_the_shaft_never_moves() {
        // Friction far above the ramp ceiling.
        let mut rig = Rig::new(0.0, 1.0, 100.0, 0.5);
        let spec = BreakawaySpec {
            ramp_nm_per_s: 50.0,
            rate_hz: 2000.0,
            ..BreakawaySpec::slow(1.0, Direction::Positive)
        };
        let r = run_breakaway(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        assert_eq!(r.breakaway_torque_nm, None);
        assert!(!r.points.is_empty(), "the ramp itself must still be logged");
    }

    /// The ramp ceiling is the tighter of the spec and the safety envelope, so
    /// a generous spec cannot exceed the guard.
    #[test]
    fn breakaway_ceiling_respects_the_safety_envelope() {
        let mut rig = Rig::new(0.0, 1.0, 100.0, 0.5);
        let limits = SafetyLimits {
            max_torque_nm: 0.3,
            ..roomy()
        };
        let spec = BreakawaySpec {
            ramp_nm_per_s: 50.0,
            rate_hz: 2000.0,
            ..BreakawaySpec::slow(10.0, Direction::Positive) // spec allows 10
        };
        let r = run_breakaway(&mut rig, &spec, limits, &no_abort()).unwrap();
        let peak = r.points.iter().map(|p| p.cmd.abs()).fold(0.0, f32::max);
        assert!(peak <= 0.3 + 1e-3, "commanded {peak}, limit was 0.3");
    }

    /// The thermal run must recover the planted heating slope, and its
    /// extrapolation to a limit must be finite while the rig is warming.
    #[test]
    fn thermal_recovers_the_heating_slope() {
        // 2 °C per (N·m·s); holding 1 N·m => 2 °C/s.
        let mut rig = Rig::new(0.0, 0.0, 0.0, 0.5).heating(2.0);
        let spec = ThermalSpec {
            hold_torque_nm: 1.0,
            duration_s: 0.4,
            rate_hz: 500.0,
            // Open-loop: these mock rigs implement set_torque, not MIT.
            leash_kp: 0.0,
            leash_kd: 0.0,
        };
        let t = run_thermal(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        assert_eq!(t.abort, None);
        let rate = t.rise_rate_c_per_s().expect("slope");
        // The rig advances 0.005 s of heat per command, not wall clock, so the
        // slope is checked for sign and order of magnitude rather than exactly.
        assert!(rate > 0.0, "should be warming, got {rate}");
        assert!(t.mean_torque_nm().unwrap() > 0.9);
        assert!(t.seconds_to_limit(1_000.0).unwrap() > 0.0);
    }

    /// A rig that does not heat is thermally sustainable, which the API reports
    /// as "no time-to-limit" rather than a bogus number.
    #[test]
    fn thermal_reports_no_time_to_limit_when_not_warming() {
        let mut rig = Rig::new(0.0, 0.0, 0.0, 0.5); // heating disabled
        let spec = ThermalSpec {
            hold_torque_nm: 1.0,
            duration_s: 0.1,
            rate_hz: 500.0,
            // Open-loop: these mock rigs implement set_torque, not MIT.
            leash_kp: 0.0,
            leash_kd: 0.0,
        };
        let t = run_thermal(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        assert_eq!(t.seconds_to_limit(100.0), None);
    }

    /// The guard must stop a thermal hold that runs away, and the partial log
    /// must survive — that data is the whole point of aborting gracefully.
    #[test]
    fn thermal_aborts_on_runaway_and_keeps_its_samples() {
        let mut rig = Rig::new(0.0, 0.0, 0.0, 0.5).heating(500.0);
        let limits = SafetyLimits {
            max_temperature_c: 45.0,
            ..roomy()
        };
        let spec = ThermalSpec {
            hold_torque_nm: 1.0,
            duration_s: 10.0,
            rate_hz: 1000.0,
            // Open-loop: these mock rigs implement set_torque, not MIT.
            leash_kp: 0.0,
            leash_kd: 0.0,
        };
        let t = run_thermal(&mut rig, &spec, limits, &no_abort()).unwrap();
        assert!(
            matches!(
                t.abort,
                Some(AbortReason::TemperatureLimit) | Some(AbortReason::TemperatureRiseRate)
            ),
            "expected a thermal abort, got {:?}",
            t.abort
        );
        assert!(!t.points.is_empty(), "partial log must be returned");
    }

    /// The Kt sweep must recover the planted torque constant, both from the
    /// least-squares slope and from the pointwise ratio, with a clean fit.
    #[test]
    fn kt_sweep_recovers_the_planted_torque_constant() {
        let kt = 0.83;
        let mut rig = Rig::new(0.0, 0.0, 0.0, kt);
        let spec = KtSpec {
            settle_s: 0.0,
            rate_hz: 1000.0,
            ..KtSpec::bipolar(1.0)
        };
        let s = run_kt(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        assert_eq!(s.abort, None);
        assert_eq!(s.points.len(), 9);

        let fitted = s.kt_nm_per_a().expect("slope");
        assert!((fitted - kt).abs() < 1e-3, "fitted {fitted} vs {kt}");
        let pointwise = s.pointwise_kt_nm_per_a().expect("ratio");
        assert!((pointwise - kt).abs() < 1e-3, "pointwise {pointwise}");
        assert!(s.r_squared().unwrap() > 0.99, "fit should be clean");
    }

    /// On a driver that reports no current — DAMIAO's feedback frame has no
    /// current field — Kt must come back `None` rather than a fabricated value.
    #[test]
    fn kt_sweep_is_none_without_current_reporting() {
        let mut rig = Rig::new(0.0, 0.0, 0.0, 0.0); // kt = 0 => current is NaN
        let spec = KtSpec {
            settle_s: 0.0,
            rate_hz: 1000.0,
            ..KtSpec::bipolar(1.0)
        };
        let s = run_kt(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        assert_eq!(s.kt_nm_per_a(), None);
        assert_eq!(s.pointwise_kt_nm_per_a(), None);
        assert_eq!(s.r_squared(), None);
    }

    /// A pre-set abort flag must stop every run immediately and be reported.
    #[test]
    fn a_preset_abort_flag_stops_every_run() {
        let abort = AtomicBool::new(true);
        let limits = roomy();

        let mut rig = Rig::new(0.0, 1.0, 0.0, 0.5);
        let m = run_load_map(
            &mut rig,
            &LoadMapSpec::symmetric(0.2, 3),
            limits,
            &abort,
        )
        .unwrap();
        assert_eq!(m.abort, Some(AbortReason::Cancelled));

        let b = run_breakaway(
            &mut rig,
            &BreakawaySpec::slow(1.0, Direction::Positive),
            limits,
            &abort,
        )
        .unwrap();
        assert_eq!(b.abort, Some(AbortReason::Cancelled));

        let t = run_thermal(
            &mut rig,
            &ThermalSpec {
                hold_torque_nm: 0.1,
                duration_s: 1.0,
                rate_hz: 100.0,
                // Open-loop: these mock rigs implement set_torque, not MIT.
                leash_kp: 0.0,
                leash_kd: 0.0,
            },
            limits,
            &abort,
        )
        .unwrap();
        assert_eq!(t.abort, Some(AbortReason::Cancelled));

        let k = run_kt(&mut rig, &KtSpec::bipolar(0.5), limits, &abort).unwrap();
        assert_eq!(k.abort, Some(AbortReason::Cancelled));
    }

    /// CSV must carry a header plus one row per point.
    #[test]
    fn csv_has_a_header_and_one_row_per_point() {
        let mut rig = Rig::new(0.0, 2.0, 0.0, 0.5);
        let spec = LoadMapSpec {
            settle_s: 0.0,
            return_sweep: false,
            ..LoadMapSpec::symmetric(0.2, 3)
        };
        let map = run_load_map(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        let mut out = Vec::new();
        write_points_csv(&map.points, &mut out, true).unwrap();
        let text = String::from_utf8(out).unwrap();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 1 + map.points.len());
        assert!(lines[0].starts_with("t_s,cmd,position_rad"));
        assert!(lines[0].ends_with("temperature_c"));
    }

    /// A friction-dominated rig: the two passes differ by 2*mu while the static
    /// load is zero. Recovering that is the point of the return sweep, and was
    /// the situation found on the DM-J4310 bench rig.
    #[test]
    fn load_map_separates_friction_from_a_zero_static_load() {
        // No spring (k = 0), friction 0.16 N·m, reported as a direction-
        // dependent holding torque by the stub below.
        struct Frictional {
            pos: f32,
            mu: f32,
            last_dir: f32,
        }
        impl Actuator for Frictional {
            fn motor_id(&self) -> u8 {
                1
            }
            fn enable(&mut self) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn disable(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_zero(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_run_mode(&mut self, _m: RunMode) -> Result<()> {
                Ok(())
            }
            fn set_position(&mut self, p: f32, _v: f32) -> Result<MotorFeedback> {
                if (p - self.pos).abs() > 1e-6 {
                    self.last_dir = (p - self.pos).signum();
                }
                self.pos = p;
                Ok(self.fb())
            }
            fn set_velocity(&mut self, _v: f32) -> Result<MotorFeedback> {
                Err(misa_actuator::Error::Unsupported("no vel"))
            }
            fn set_torque(&mut self, _t: f32) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn mit_control(
                &mut self,
                _p: f32,
                _v: f32,
                _kp: f32,
                _kd: f32,
                _t: f32,
            ) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn measure(&mut self) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn read_status(&mut self) -> Result<misa_actuator::MotorStatus> {
                Ok(misa_actuator::MotorStatus {
                    voltage_v: 24.0,
                    temperature_c: 30.0,
                    error: Default::default(),
                })
            }
        }
        impl Frictional {
            fn fb(&self) -> MotorFeedback {
                MotorFeedback {
                    position_rad: self.pos,
                    velocity_rad_per_s: 0.0,
                    // Holding torque opposes the direction of last travel: pure
                    // Coulomb friction, no position term at all.
                    torque_nm: self.mu * self.last_dir,
                    current_a: f32::NAN,
                    temperature_c: 30.0,
                }
            }
        }

        let mu = 0.16;
        let mut rig = Frictional {
            pos: 0.0,
            mu,
            last_dir: 1.0,
        };
        let spec = LoadMapSpec {
            settle_s: 0.0,
            travel_timeout_s: 0.0,
            ..LoadMapSpec::symmetric(0.1, 5)
        };
        let map = run_load_map(&mut rig, &spec, roomy(), &no_abort()).unwrap();

        // Raw peak holding torque is just the friction level.
        assert!((map.peak_holding_torque_nm().unwrap() - mu).abs() < 1e-3);
        // Friction is recovered, and the static load is ~zero.
        assert!((map.mean_friction_nm().unwrap() - mu).abs() < 1e-3);
        assert!(
            map.peak_static_load_nm().unwrap() < 1e-3,
            "static load should vanish, got {:?}",
            map.peak_static_load_nm()
        );
        // Which is exactly when the raw equilibrium is not to be trusted.
        assert!(map.peak_static_load_nm().unwrap() < map.mean_friction_nm().unwrap());
    }

    /// With a spring and no friction, the de-frictioned equilibrium must agree
    /// with the planted one and the friction estimate must vanish.
    #[test]
    fn static_equilibrium_matches_the_spring_with_no_friction() {
        let mut rig = Rig::new(0.0, 4.0, 0.0, 0.5);
        let spec = LoadMapSpec {
            settle_s: 0.0,
            travel_timeout_s: 0.0,
            ..LoadMapSpec::symmetric(0.4, 5)
        };
        let map = run_load_map(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        let eq = map.static_equilibrium_position_rad().unwrap();
        assert!(eq.abs() < 0.15, "de-frictioned equilibrium at {eq}");
        assert!(map.mean_friction_nm().unwrap() < 1e-3);
        // k = 4 N·m/rad over offsets +/-0.4 in 5 steps, but the +/-0.4 extremes
        // are excluded from the pairing by design, so the widest paired offset
        // is 0.2 rad => 0.8 N·m.
        let peak = map.peak_static_load_nm().unwrap();
        assert!((peak - 0.8).abs() < 1e-3, "spring load {peak}, expected 0.8");
    }

    /// Without a return pass there is nothing to pair, so the friction-based
    /// views report nothing rather than guessing.
    #[test]
    fn friction_views_need_a_return_sweep() {
        let mut rig = Rig::new(0.0, 4.0, 0.0, 0.5);
        let spec = LoadMapSpec {
            settle_s: 0.0,
            travel_timeout_s: 0.0,
            return_sweep: false,
            ..LoadMapSpec::symmetric(0.2, 4)
        };
        let map = run_load_map(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        assert!(map.static_load_curve().is_empty());
        assert_eq!(map.static_equilibrium_position_rad(), None);
        assert_eq!(map.mean_friction_nm(), None);
        assert_eq!(map.peak_static_load_nm(), None);
    }

    /// The arrival wait must not hang on a shaft that cannot reach the target:
    /// it falls through on the travel timeout and records the stalled reading.
    #[test]
    fn load_map_times_out_travel_instead_of_hanging() {
        /// A shaft clamped at 0.0 — it never arrives anywhere.
        struct Stuck;
        impl Actuator for Stuck {
            fn motor_id(&self) -> u8 {
                1
            }
            fn enable(&mut self) -> Result<MotorFeedback> {
                self.measure()
            }
            fn disable(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_zero(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_run_mode(&mut self, _m: RunMode) -> Result<()> {
                Ok(())
            }
            fn set_position(&mut self, _p: f32, _v: f32) -> Result<MotorFeedback> {
                self.measure()
            }
            fn set_velocity(&mut self, _v: f32) -> Result<MotorFeedback> {
                self.measure()
            }
            fn set_torque(&mut self, _t: f32) -> Result<MotorFeedback> {
                self.measure()
            }
            fn mit_control(
                &mut self,
                _p: f32,
                _v: f32,
                _kp: f32,
                _kd: f32,
                _t: f32,
            ) -> Result<MotorFeedback> {
                self.measure()
            }
            fn measure(&mut self) -> Result<MotorFeedback> {
                Ok(MotorFeedback {
                    position_rad: 0.0,
                    velocity_rad_per_s: 0.0,
                    torque_nm: 0.5,
                    current_a: f32::NAN,
                    temperature_c: 30.0,
                })
            }
            fn read_status(&mut self) -> Result<misa_actuator::MotorStatus> {
                Ok(misa_actuator::MotorStatus {
                    voltage_v: 24.0,
                    temperature_c: 30.0,
                    error: Default::default(),
                })
            }
        }
        let spec = LoadMapSpec {
            settle_s: 0.0,
            travel_timeout_s: 0.02, // short, so the test stays fast
            ..LoadMapSpec::symmetric(0.5, 3)
        };
        let map = run_load_map(&mut Stuck, &spec, roomy(), &no_abort()).unwrap();
        // Every commanded point still produced a reading.
        assert_eq!(map.points.len(), 5);
        assert_eq!(map.abort, None);
    }

    /// A shaft that is already moving must not be read as an instant breakaway
    /// at zero torque. This is what `--both` hit on real hardware: the second
    /// ramp began while the shaft still coasted from the first and reported
    /// -0.0000 N·m.
    #[test]
    fn breakaway_waits_for_rest_before_ramping() {
        /// Coasts at 0.5 rad/s for the first few commands, then stops.
        struct Coasting {
            commands: u32,
            settle_after: u32,
            pos: f32,
        }
        impl Coasting {
            fn fb(&self) -> MotorFeedback {
                let moving = self.commands < self.settle_after;
                MotorFeedback {
                    position_rad: self.pos,
                    velocity_rad_per_s: if moving { 0.5 } else { 0.0 },
                    torque_nm: 0.0,
                    current_a: f32::NAN,
                    temperature_c: 30.0,
                }
            }
        }
        impl Actuator for Coasting {
            fn motor_id(&self) -> u8 {
                1
            }
            fn enable(&mut self) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn disable(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_zero(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_run_mode(&mut self, _m: RunMode) -> Result<()> {
                Ok(())
            }
            fn set_position(&mut self, _p: f32, _v: f32) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn set_velocity(&mut self, _v: f32) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn set_torque(&mut self, _t: f32) -> Result<MotorFeedback> {
                self.commands += 1;
                Ok(self.fb())
            }
            fn mit_control(
                &mut self,
                _p: f32,
                _v: f32,
                _kp: f32,
                _kd: f32,
                _t: f32,
            ) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn measure(&mut self) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn read_status(&mut self) -> Result<misa_actuator::MotorStatus> {
                Ok(misa_actuator::MotorStatus {
                    voltage_v: 24.0,
                    temperature_c: 30.0,
                    error: Default::default(),
                })
            }
        }

        let mut rig = Coasting {
            commands: 0,
            settle_after: 3,
            pos: 0.0,
        };
        let spec = BreakawaySpec {
            ramp_nm_per_s: 20.0,
            rate_hz: 2000.0,
            rest_window_s: 0.0, // one still sample is enough for the test
            rest_timeout_s: 1.0,
            ..BreakawaySpec::slow(1.0, Direction::Positive)
        };
        let r = run_breakaway(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        assert!(r.rested, "should have waited for stillness");
        // Having waited, the shaft is stopped, so this rig never breaks away and
        // the ramp runs to its ceiling — rather than reporting a bogus 0 N·m.
        assert_eq!(r.breakaway_torque_nm, None);
    }

    /// A shaft that never settles is still measured, but flagged so the number
    /// can be discounted rather than silently trusted.
    #[test]
    fn breakaway_flags_a_shaft_that_never_rests() {
        /// Always moving.
        struct Spinning;
        impl Actuator for Spinning {
            fn motor_id(&self) -> u8 {
                1
            }
            fn enable(&mut self) -> Result<MotorFeedback> {
                self.measure()
            }
            fn disable(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_zero(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_run_mode(&mut self, _m: RunMode) -> Result<()> {
                Ok(())
            }
            fn set_position(&mut self, _p: f32, _v: f32) -> Result<MotorFeedback> {
                self.measure()
            }
            fn set_velocity(&mut self, _v: f32) -> Result<MotorFeedback> {
                self.measure()
            }
            fn set_torque(&mut self, _t: f32) -> Result<MotorFeedback> {
                self.measure()
            }
            fn mit_control(
                &mut self,
                _p: f32,
                _v: f32,
                _kp: f32,
                _kd: f32,
                _t: f32,
            ) -> Result<MotorFeedback> {
                self.measure()
            }
            fn measure(&mut self) -> Result<MotorFeedback> {
                Ok(MotorFeedback {
                    position_rad: 0.0,
                    velocity_rad_per_s: 1.0,
                    torque_nm: 0.0,
                    current_a: f32::NAN,
                    temperature_c: 30.0,
                })
            }
            fn read_status(&mut self) -> Result<misa_actuator::MotorStatus> {
                Ok(misa_actuator::MotorStatus {
                    voltage_v: 24.0,
                    temperature_c: 30.0,
                    error: Default::default(),
                })
            }
        }
        let spec = BreakawaySpec {
            ramp_nm_per_s: 20.0,
            rate_hz: 2000.0,
            rest_window_s: 0.05,
            rest_timeout_s: 0.05, // gives up almost immediately
            ..BreakawaySpec::slow(1.0, Direction::Positive)
        };
        let r = run_breakaway(&mut Spinning, &spec, roomy(), &no_abort()).unwrap();
        assert!(!r.rested, "must record that stillness was never reached");
    }

    /// With a leash the run must drive `mit_control`, not `set_torque`: that is
    /// what keeps a free shaft from accelerating out of the safety window before
    /// any thermal data is collected.
    #[test]
    fn thermal_uses_mit_when_leashed_and_torque_when_not() {
        /// Records which control path was exercised.
        struct Counting {
            mit: u32,
            torque: u32,
        }
        impl Counting {
            fn fb(&self) -> MotorFeedback {
                MotorFeedback {
                    position_rad: 0.0,
                    velocity_rad_per_s: 0.0,
                    torque_nm: 0.1,
                    current_a: f32::NAN,
                    temperature_c: 30.0,
                }
            }
        }
        impl Actuator for Counting {
            fn motor_id(&self) -> u8 {
                1
            }
            fn enable(&mut self) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn disable(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_zero(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_run_mode(&mut self, _m: RunMode) -> Result<()> {
                Ok(())
            }
            fn set_position(&mut self, _p: f32, _v: f32) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn set_velocity(&mut self, _v: f32) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn set_torque(&mut self, _t: f32) -> Result<MotorFeedback> {
                self.torque += 1;
                Ok(self.fb())
            }
            fn mit_control(
                &mut self,
                _p: f32,
                _v: f32,
                _kp: f32,
                _kd: f32,
                _t: f32,
            ) -> Result<MotorFeedback> {
                self.mit += 1;
                Ok(self.fb())
            }
            fn measure(&mut self) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn read_status(&mut self) -> Result<misa_actuator::MotorStatus> {
                Ok(misa_actuator::MotorStatus {
                    voltage_v: 24.0,
                    temperature_c: 30.0,
                    error: Default::default(),
                })
            }
        }

        let base = ThermalSpec {
            hold_torque_nm: 0.2,
            duration_s: 0.05,
            rate_hz: 500.0,
            leash_kp: 0.0,
            leash_kd: 0.0,
        };

        let mut open = Counting { mit: 0, torque: 0 };
        run_thermal(&mut open, &base, roomy(), &no_abort()).unwrap();
        assert!(open.torque > 0, "open-loop must use set_torque");
        assert_eq!(open.mit, 0, "open-loop must not use mit_control");

        let mut leashed = Counting { mit: 0, torque: 0 };
        let spec = ThermalSpec {
            leash_kp: 8.0,
            leash_kd: 0.5,
            ..base
        };
        run_thermal(&mut leashed, &spec, roomy(), &no_abort()).unwrap();
        assert!(leashed.mit > 0, "leashed must use mit_control");
        // The final safe-stop still commands zero torque, so allow exactly that.
        assert!(
            leashed.torque <= 1,
            "leashed must not drive open-loop torque during the hold, got {}",
            leashed.torque
        );
    }

    /// Two runs sharing one file must yield one header, not one per run: a
    /// two-direction breakaway writes both ramps to the same CSV, and a second
    /// header lands as a text row in the middle of the data.
    #[test]
    fn appending_a_second_run_does_not_repeat_the_header() {
        let mut rig = Rig::new(0.0, 2.0, 0.0, 0.5);
        let spec = LoadMapSpec {
            settle_s: 0.0,
            travel_timeout_s: 0.0,
            return_sweep: false,
            ..LoadMapSpec::symmetric(0.1, 3)
        };
        let map = run_load_map(&mut rig, &spec, roomy(), &no_abort()).unwrap();

        let mut out = Vec::new();
        write_points_csv(&map.points, &mut out, true).unwrap();
        write_points_csv(&map.points, &mut out, false).unwrap();
        let text = String::from_utf8(out).unwrap();
        let headers = text.lines().filter(|l| *l == POINTS_CSV_HEADER).count();
        assert_eq!(headers, 1, "expected exactly one header row");
        assert_eq!(text.lines().count(), 1 + 2 * map.points.len());
    }

    /// A hold that never delivered its commanded torque must be visible as
    /// such. Measured on a free-shafted RS04: commanding 2.0 N·m delivered
    /// 0.14, and without this the run reported "sustainable" about a torque the
    /// motor never produced.
    #[test]
    fn thermal_reports_how_much_torque_actually_arrived() {
        // Rig reports a fixed small torque regardless of command — the shape a
        // leash-cancelled hold produces.
        let mut rig = Rig::new(0.0, 0.0, 0.0, 0.5);
        let spec = ThermalSpec {
            hold_torque_nm: 2.0,
            duration_s: 0.05,
            rate_hz: 500.0,
            leash_kp: 0.0,
            leash_kd: 0.0,
        };
        let t = run_thermal(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        // This rig echoes the command, so the fraction is ~1: the honest case.
        let f = t.delivered_fraction().expect("fraction");
        assert!((f - 1.0).abs() < 1e-3, "echoing rig should deliver ~100%, got {f}");

        // Now a rig that swallows most of the command.
        struct Weak;
        impl Actuator for Weak {
            fn motor_id(&self) -> u8 {
                1
            }
            fn enable(&mut self) -> Result<MotorFeedback> {
                self.measure()
            }
            fn disable(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_zero(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_run_mode(&mut self, _m: RunMode) -> Result<()> {
                Ok(())
            }
            fn set_position(&mut self, _p: f32, _v: f32) -> Result<MotorFeedback> {
                self.measure()
            }
            fn set_velocity(&mut self, _v: f32) -> Result<MotorFeedback> {
                self.measure()
            }
            fn set_torque(&mut self, _t: f32) -> Result<MotorFeedback> {
                self.measure()
            }
            fn mit_control(
                &mut self,
                _p: f32,
                _v: f32,
                _kp: f32,
                _kd: f32,
                _t: f32,
            ) -> Result<MotorFeedback> {
                self.measure()
            }
            fn measure(&mut self) -> Result<MotorFeedback> {
                Ok(MotorFeedback {
                    position_rad: 0.0,
                    velocity_rad_per_s: 0.0,
                    torque_nm: 0.25, // regardless of the 2.0 asked for
                    current_a: f32::NAN,
                    temperature_c: 30.0,
                })
            }
            fn read_status(&mut self) -> Result<misa_actuator::MotorStatus> {
                Ok(misa_actuator::MotorStatus {
                    voltage_v: 24.0,
                    temperature_c: 30.0,
                    error: Default::default(),
                })
            }
        }
        let t = run_thermal(&mut Weak, &spec, roomy(), &no_abort()).unwrap();
        let f = t.delivered_fraction().expect("fraction");
        assert!((f - 0.125).abs() < 1e-3, "expected ~12.5%, got {f}");
        assert!(f < 0.5, "must fall below the CLI's validity threshold");
    }

    /// The breakaway map must recover the planted friction at every position,
    /// and — crucially — do so **without depending on torque feedback**, since
    /// that is why it exists.
    #[test]
    fn breakaway_map_recovers_friction_without_torque_feedback() {
        /// Reports NaN torque throughout, like RobStride firmware whose
        /// `MeasuredTorque` is broken. Motion still gates on velocity.
        struct NoTorque {
            mu: f32,
            pos: f32,
            vel: f32,
            cmd: f32,
        }
        impl NoTorque {
            fn fb(&self) -> MotorFeedback {
                MotorFeedback {
                    position_rad: self.pos,
                    velocity_rad_per_s: self.vel,
                    torque_nm: f32::NAN, // the whole point
                    current_a: f32::NAN,
                    temperature_c: f32::NAN,
                }
            }
        }
        impl Actuator for NoTorque {
            fn motor_id(&self) -> u8 {
                1
            }
            fn enable(&mut self) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn disable(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_zero(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_run_mode(&mut self, _m: RunMode) -> Result<()> {
                Ok(())
            }
            fn set_position(&mut self, p: f32, _v: f32) -> Result<MotorFeedback> {
                self.pos = p;
                self.vel = 0.0;
                self.cmd = 0.0;
                Ok(self.fb())
            }
            fn set_velocity(&mut self, _v: f32) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn set_torque(&mut self, t: f32) -> Result<MotorFeedback> {
                self.cmd = t;
                // Moves only once the command clears friction.
                self.vel = if t.abs() > self.mu { t.signum() } else { 0.0 };
                Ok(self.fb())
            }
            fn mit_control(
                &mut self,
                _p: f32,
                _v: f32,
                _kp: f32,
                _kd: f32,
                t: f32,
            ) -> Result<MotorFeedback> {
                self.set_torque(t)
            }
            fn measure(&mut self) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn read_status(&mut self) -> Result<misa_actuator::MotorStatus> {
                Ok(misa_actuator::MotorStatus {
                    voltage_v: 24.0,
                    temperature_c: f32::NAN,
                    error: Default::default(),
                })
            }
        }

        let mu = 0.3;
        let mut rig = NoTorque {
            mu,
            pos: 0.0,
            vel: 0.0,
            cmd: 0.0,
        };
        let ramp = BreakawaySpec {
            ramp_nm_per_s: 20.0,
            rate_hz: 2000.0,
            rest_window_s: 0.0,
            rest_timeout_s: 0.2,
            motion_threshold_rad: 1e9, // gate on velocity only
            motion_hold_s: 0.0,        // clean breakaway, and the ramp is short
            ..BreakawaySpec::slow(2.0, Direction::Positive)
        };
        let spec = BreakawayMapSpec {
            travel_timeout_s: 0.0,
            ..BreakawayMapSpec::symmetric(0.2, 3, ramp)
        };
        let m = run_breakaway_map(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        assert_eq!(m.points.len(), 3);
        assert_eq!(m.abort, None);

        // Friction recovered at every position, static load ~0.
        let stic = m.mean_stiction_nm().expect("stiction");
        assert!((stic - mu).abs() < 0.1, "stiction {stic} vs planted {mu}");
        assert!(
            m.peak_static_load_nm().unwrap() < 0.1,
            "static load should vanish, got {:?}",
            m.peak_static_load_nm()
        );
        // And it worked despite torque never being reported.
        assert!(m.samples.iter().all(|p| p.torque_nm.is_nan()));
    }

    /// A leg that starts by reversing must not log the reversal.
    ///
    /// The rig only reaches the commanded speed after a delay, and reports a
    /// wildly wrong torque until it does. That transient used to land in the
    /// position bins at the ends of the span — at opposite ends for the two legs,
    /// since each reverses where the other finished — so the end bins compared a
    /// transient in one direction against steady motion in the other.
    #[test]
    fn a_legs_reversal_is_not_part_of_the_measurement() {
        /// Takes `lag` samples to pick up the commanded speed, and reports a
        /// large bogus torque throughout that window.
        struct Laggy {
            pos: f32,
            vel: f32,
            cmd: f32,
            since_change: u32,
            lag: u32,
            dt: f32,
        }
        impl Actuator for Laggy {
            fn motor_id(&self) -> u8 {
                1
            }
            fn enable(&mut self) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn disable(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_zero(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_run_mode(&mut self, _m: RunMode) -> Result<()> {
                Ok(())
            }
            fn set_position(&mut self, _p: f32, _s: f32) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn set_velocity(&mut self, v: f32) -> Result<MotorFeedback> {
                if v != self.cmd {
                    self.cmd = v;
                    self.since_change = 0;
                }
                self.since_change += 1;
                // Keep the old velocity through the lag window, so the shaft
                // carries on the way it was going while the new command is
                // already in force — which is what a real reversal does.
                if self.since_change > self.lag {
                    self.vel = v;
                }
                self.pos += self.vel * self.dt;
                Ok(self.fb())
            }
            fn set_torque(&mut self, _t: f32) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn mit_control(
                &mut self,
                _p: f32,
                _v: f32,
                _kp: f32,
                _kd: f32,
                _tff: f32,
            ) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn measure(&mut self) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn read_status(&mut self) -> Result<misa_actuator::MotorStatus> {
                Ok(misa_actuator::MotorStatus {
                    voltage_v: 24.0,
                    temperature_c: 30.0,
                    error: Default::default(),
                })
            }
        }
        impl Laggy {
            fn fb(&self) -> MotorFeedback {
                // 10 N·m while catching up, 0.5·direction once at speed.
                let torque = if self.since_change <= self.lag {
                    10.0
                } else {
                    0.5 * self.vel.signum()
                };
                MotorFeedback {
                    position_rad: self.pos,
                    velocity_rad_per_s: self.vel,
                    torque_nm: torque,
                    current_a: f32::NAN,
                    temperature_c: 30.0,
                }
            }
        }

        let spec = VelocitySweepSpec {
            speed_rad_s: 1.0,
            half_span_rad: 0.5,
            rate_hz: 1000.0,
            return_sweep: true,
            lead_in_rad: VelocitySweepSpec::lead_in_for(0.5),
        };
        let mut rig = Laggy {
            pos: 0.0,
            vel: -1.0,
            cmd: -1.0,
            since_change: 999,
            lag: 20,
            dt: 0.001,
        };
        let sweep = run_velocity_sweep(&mut rig, &spec, roomy(), &no_abort()).unwrap();

        // The bogus 10 N·m must be nowhere in the measurement.
        assert!(
            sweep.points.iter().all(|p| p.torque_nm < 5.0),
            "the reversal transient was logged: {:?}",
            sweep.points.iter().map(|p| p.torque_nm).fold(0.0, f32::max)
        );
        // And what is left is the planted friction, with no load.
        let friction = sweep.mean_kinetic_friction_nm(8).expect("a curve");
        assert!(
            (friction - 0.5).abs() < 0.05,
            "friction {friction}, expected 0.5"
        );
        let load = sweep.peak_static_load_nm(8).expect("a curve");
        assert!(load < 0.05, "load should vanish, got {load}");
    }

    /// The velocity sweep must split a planted spring load from kinetic friction
    /// by binning the two passes.
    #[test]
    fn velocity_sweep_separates_a_spring_load_from_friction() {
        /// Moves at the commanded speed and reports `k*pos + mu*dir` — a spring
        /// plus direction-dependent friction, which is exactly what the
        /// decomposition should pull apart.
        struct Traverse {
            k: f32,
            mu: f32,
            pos: f32,
            vel: f32,
            dt: f32,
        }
        impl Traverse {
            fn fb(&self) -> MotorFeedback {
                MotorFeedback {
                    position_rad: self.pos,
                    velocity_rad_per_s: self.vel,
                    torque_nm: self.k * self.pos + self.mu * self.vel.signum(),
                    current_a: f32::NAN,
                    temperature_c: 30.0,
                }
            }
        }
        impl Actuator for Traverse {
            fn motor_id(&self) -> u8 {
                1
            }
            fn enable(&mut self) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn disable(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_zero(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_run_mode(&mut self, _m: RunMode) -> Result<()> {
                Ok(())
            }
            fn set_position(&mut self, _p: f32, _v: f32) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn set_velocity(&mut self, v: f32) -> Result<MotorFeedback> {
                self.vel = v;
                self.pos += v * self.dt;
                Ok(self.fb())
            }
            fn set_torque(&mut self, _t: f32) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn mit_control(
                &mut self,
                _p: f32,
                _v: f32,
                _kp: f32,
                _kd: f32,
                _t: f32,
            ) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn measure(&mut self) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn read_status(&mut self) -> Result<misa_actuator::MotorStatus> {
                Ok(misa_actuator::MotorStatus {
                    voltage_v: 24.0,
                    temperature_c: 30.0,
                    error: Default::default(),
                })
            }
        }

        let (k, mu) = (2.0f32, 0.15f32);
        let mut rig = Traverse {
            k,
            mu,
            pos: 0.0,
            vel: 0.0,
            dt: 0.01,
        };
        let spec = VelocitySweepSpec {
            speed_rad_s: 1.0,
            half_span_rad: 0.3,
            rate_hz: 5000.0,
            return_sweep: true,
            // No lead-in: these rigs have no momentum to reverse, so it would
            // only drop samples the test is counting.
            lead_in_rad: 0.0,
        };
        let sweep = run_velocity_sweep(&mut rig, &spec, roomy(), &no_abort()).unwrap();
        assert_eq!(sweep.abort, None);

        let fric = sweep.mean_kinetic_friction_nm(10).expect("friction");
        assert!((fric - mu).abs() < 0.02, "friction {fric} vs planted {mu}");

        // The spring shows up as a static load rising with |position|.
        let curve = sweep.friction_curve(10);
        assert!(curve.len() >= 5, "expected a populated curve");
        for (pos, load, _) in &curve {
            assert!(
                (load - k * pos).abs() < 0.05,
                "at {pos}: load {load} vs expected {}",
                k * pos
            );
        }
    }

    /// A traverse that never moves must be reported as stalled rather than
    /// yielding a bogus split.
    #[test]
    fn velocity_sweep_flags_a_stalled_traverse() {
        struct Stalled;
        impl Actuator for Stalled {
            fn motor_id(&self) -> u8 {
                1
            }
            fn enable(&mut self) -> Result<MotorFeedback> {
                self.measure()
            }
            fn disable(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_zero(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_run_mode(&mut self, _m: RunMode) -> Result<()> {
                Ok(())
            }
            fn set_position(&mut self, _p: f32, _v: f32) -> Result<MotorFeedback> {
                self.measure()
            }
            fn set_velocity(&mut self, _v: f32) -> Result<MotorFeedback> {
                self.measure()
            }
            fn set_torque(&mut self, _t: f32) -> Result<MotorFeedback> {
                self.measure()
            }
            fn mit_control(
                &mut self,
                _p: f32,
                _v: f32,
                _kp: f32,
                _kd: f32,
                _t: f32,
            ) -> Result<MotorFeedback> {
                self.measure()
            }
            fn measure(&mut self) -> Result<MotorFeedback> {
                Ok(MotorFeedback {
                    position_rad: 0.0, // never advances
                    velocity_rad_per_s: 0.0,
                    torque_nm: 0.1,
                    current_a: f32::NAN,
                    temperature_c: 30.0,
                })
            }
            fn read_status(&mut self) -> Result<misa_actuator::MotorStatus> {
                Ok(misa_actuator::MotorStatus {
                    voltage_v: 24.0,
                    temperature_c: 30.0,
                    error: Default::default(),
                })
            }
        }
        let limits = SafetyLimits {
            max_duration_s: 0.05, // the run can only end on the time budget
            ..roomy()
        };
        let spec = VelocitySweepSpec {
            speed_rad_s: 0.5,
            half_span_rad: 0.3,
            rate_hz: 2000.0,
            return_sweep: true,
            // No lead-in: these rigs have no momentum to reverse, so it would
            // only drop samples the test is counting.
            lead_in_rad: 0.0,
        };
        let sweep = run_velocity_sweep(&mut Stalled, &spec, limits, &no_abort()).unwrap();
        // The unlogged positioning leg never reaches its bound, so the time
        // budget ends the run before any measured leg starts. Zero samples plus a
        // Timeout is the honest report — better than a curve built from a shaft
        // that never moved.
        assert_eq!(sweep.abort, Some(AbortReason::Timeout));
        assert!(sweep.points.is_empty(), "nothing should have been measured");
        assert_eq!(sweep.mean_speed_rad_s(), None);
        assert!(sweep.friction_curve(10).is_empty());
    }

    /// Judder must not register as breakaway. A rig that crosses the velocity
    /// threshold in brief spikes at low torque, then sticks again, previously
    /// reported the torque of the first spike — three times below what a
    /// constant-speed traverse of the same RS04 actually needed.
    #[test]
    fn breakaway_ignores_judder_and_waits_for_sustained_motion() {
        /// Spikes above the threshold every other command until `sustain_above`,
        /// then moves continuously.
        struct Juddering {
            n: u32,
            sustain_above: f32,
            cmd: f32,
        }
        impl Juddering {
            fn fb(&self) -> MotorFeedback {
                let sustained = self.cmd.abs() >= self.sustain_above;
                // Judder: alternate between a threshold-clearing spike and rest.
                let v = if sustained {
                    1.0
                } else if self.n.is_multiple_of(2) {
                    0.5
                } else {
                    0.0
                };
                MotorFeedback {
                    position_rad: 0.0,
                    velocity_rad_per_s: v,
                    torque_nm: f32::NAN,
                    current_a: f32::NAN,
                    temperature_c: f32::NAN,
                }
            }
        }
        impl Actuator for Juddering {
            fn motor_id(&self) -> u8 {
                1
            }
            fn enable(&mut self) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn disable(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_zero(&mut self) -> Result<()> {
                Ok(())
            }
            fn set_run_mode(&mut self, _m: RunMode) -> Result<()> {
                Ok(())
            }
            fn set_position(&mut self, _p: f32, _v: f32) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn set_velocity(&mut self, _v: f32) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn set_torque(&mut self, t: f32) -> Result<MotorFeedback> {
                self.n += 1;
                self.cmd = t;
                Ok(self.fb())
            }
            fn mit_control(
                &mut self,
                _p: f32,
                _v: f32,
                _kp: f32,
                _kd: f32,
                t: f32,
            ) -> Result<MotorFeedback> {
                self.set_torque(t)
            }
            fn measure(&mut self) -> Result<MotorFeedback> {
                Ok(self.fb())
            }
            fn read_status(&mut self) -> Result<misa_actuator::MotorStatus> {
                Ok(misa_actuator::MotorStatus {
                    voltage_v: 24.0,
                    temperature_c: f32::NAN,
                    error: Default::default(),
                })
            }
        }

        let sustain_above = 0.5;
        let base = BreakawaySpec {
            ramp_nm_per_s: 5.0,
            rate_hz: 2000.0,
            motion_threshold_rad_per_s: 0.2, // both the spike and the sustain clear it
            motion_threshold_rad: 1e9,
            rest_window_s: 0.0,
            rest_timeout_s: 0.2,
            ..BreakawaySpec::slow(2.0, Direction::Positive)
        };

        // Single-crossing: fires on the first judder spike, far too early.
        let mut rig = Juddering {
            n: 0,
            sustain_above,
            cmd: 0.0,
        };
        let lax = run_breakaway(
            &mut rig,
            &BreakawaySpec {
                motion_hold_s: 0.0,
                ..base
            },
            roomy(),
            &no_abort(),
        )
        .unwrap();
        let early = lax.breakaway_torque_nm.expect("should trigger");
        assert!(
            early < sustain_above,
            "single-crossing should fire before {sustain_above}, got {early}"
        );

        // Requiring the motion to hold rejects the spikes and waits for real
        // motion.
        let mut rig = Juddering {
            n: 0,
            sustain_above,
            cmd: 0.0,
        };
        let strict = run_breakaway(
            &mut rig,
            &BreakawaySpec {
                motion_hold_s: 0.02,
                ..base
            },
            roomy(),
            &no_abort(),
        )
        .unwrap();
        let real = strict.breakaway_torque_nm.expect("should still trigger");
        // Allow a couple of ramp steps of slack. The figure reported is the
        // command from when motion *started*, and the last judder spike can land
        // on the sample immediately before the ramp crosses `sustain_above` — so
        // a correct run can come in one step under it. One step here is
        // 5.0 N·m/s / 2000 Hz = 0.0025 N·m; without this the test fails roughly
        // one run in ten.
        let step = 5.0 / 2000.0;
        assert!(
            real >= sustain_above - 2.0 * step,
            "with a hold it must wait for sustained motion (~{sustain_above}), got {real}"
        );
        assert!(real > early, "the hold must raise the estimate");
    }

    /// A sweep must finish where it started, or a repeated series walks one span
    /// per repetition until it hits something. A DM-J4310 did exactly that.
    #[test]
    fn velocity_sweep_returns_to_its_starting_position() {
        let mut rig = Traverse2 {
            pos: 0.0,
            vel: 0.0,
            dt: 0.002,
        };
        let spec = VelocitySweepSpec {
            speed_rad_s: 1.0,
            half_span_rad: 0.2,
            rate_hz: 5000.0,
            return_sweep: true,
            // No lead-in: these rigs have no momentum to reverse, so it would
            // only drop samples the test is counting.
            lead_in_rad: 0.0,
        };
        let start = 0.0;
        for rep in 0..3 {
            let sweep = run_velocity_sweep(&mut rig, &spec, roomy(), &no_abort()).unwrap();
            assert_eq!(sweep.abort, None, "rep {rep}");
            assert!(
                (rig.pos - start).abs() < 0.05,
                "after rep {rep} the shaft sits at {} instead of near {start}",
                rig.pos
            );
        }
    }

    /// A stall must be judged by where the shaft went, not by what it said.
    ///
    /// Measured on an RS-03 (2026-08-06): while traversing at the commanded
    /// 0.05 rad/s, the firmware reported velocity as an exact zero in most frames,
    /// so the mean of the reported values came out at 0.0085 — under a fifth of
    /// the command. Judging by that condemned every run on that motor as stalled
    /// while the position column showed a clean traverse.
    #[test]
    fn a_stall_is_judged_by_position_not_by_the_reported_velocity() {
        // 100 samples at 100 Hz travelling 0.01 rad each: 1.0 rad/s, and the
        // motor claims zero for two out of every three frames.
        let points: Vec<Point> = (0..100)
            .map(|i| Point {
                t_s: i as f32 * 0.01,
                position_rad: i as f32 * 0.01,
                velocity_rad_per_s: if i % 3 == 0 { 1.0 } else { 0.0 },
                torque_nm: 0.2,
                current_a: f32::NAN,
                temperature_c: 30.0,
                cmd: 1.0,
            })
            .collect();
        let sweep = VelocitySweep {
            points,
            start_position_rad: 0.0,
            spec: VelocitySweepSpec {
                speed_rad_s: 1.0,
                half_span_rad: 0.5,
                rate_hz: 100.0,
                return_sweep: true,
                lead_in_rad: 0.0,
            },
            abort: None,
        };

        let travelled = sweep.traversed_speed_rad_s().expect("distance over time");
        assert!(
            (travelled - 1.0).abs() < 0.02,
            "position says 1.0 rad/s, got {travelled}"
        );

        let reported = sweep.mean_speed_rad_s().expect("reported mean");
        assert!(
            reported < 0.5,
            "the reported mean should be dragged down by the zeros, got {reported}"
        );
        // The two disagreeing is the signature of an unreliable velocity field,
        // which is worth reporting rather than resolving silently either way.
        assert!(travelled > reported * 2.0);
    }

    /// And a real stall still has to be caught: no movement, whatever is reported.
    #[test]
    fn a_shaft_that_did_not_move_is_still_a_stall() {
        let points: Vec<Point> = (0..50)
            .map(|i| Point {
                t_s: i as f32 * 0.01,
                position_rad: 0.25,
                velocity_rad_per_s: 1.0,
                torque_nm: 0.2,
                current_a: f32::NAN,
                temperature_c: 30.0,
                cmd: 1.0,
            })
            .collect();
        let sweep = VelocitySweep {
            points,
            start_position_rad: 0.25,
            spec: VelocitySweepSpec {
                speed_rad_s: 1.0,
                half_span_rad: 0.5,
                rate_hz: 100.0,
                return_sweep: true,
                lead_in_rad: 0.0,
            },
            abort: None,
        };
        assert_eq!(sweep.traversed_speed_rad_s(), Some(0.0));
    }

    /// Minimal traverse rig for the walk test: moves at the commanded speed.
    struct Traverse2 {
        pos: f32,
        vel: f32,
        dt: f32,
    }
    impl Traverse2 {
        fn fb(&self) -> MotorFeedback {
            MotorFeedback {
                position_rad: self.pos,
                velocity_rad_per_s: self.vel,
                torque_nm: 0.1 * self.vel.signum(),
                current_a: f32::NAN,
                temperature_c: 30.0,
            }
        }
    }
    impl Actuator for Traverse2 {
        fn motor_id(&self) -> u8 {
            1
        }
        fn enable(&mut self) -> Result<MotorFeedback> {
            Ok(self.fb())
        }
        fn disable(&mut self) -> Result<()> {
            Ok(())
        }
        fn set_zero(&mut self) -> Result<()> {
            Ok(())
        }
        fn set_run_mode(&mut self, _m: RunMode) -> Result<()> {
            Ok(())
        }
        fn set_position(&mut self, _p: f32, _v: f32) -> Result<MotorFeedback> {
            Ok(self.fb())
        }
        fn set_velocity(&mut self, v: f32) -> Result<MotorFeedback> {
            self.vel = v;
            self.pos += v * self.dt;
            Ok(self.fb())
        }
        fn set_torque(&mut self, _t: f32) -> Result<MotorFeedback> {
            Ok(self.fb())
        }
        fn mit_control(
            &mut self,
            _p: f32,
            _v: f32,
            _kp: f32,
            _kd: f32,
            _t: f32,
        ) -> Result<MotorFeedback> {
            Ok(self.fb())
        }
        fn measure(&mut self) -> Result<MotorFeedback> {
            Ok(self.fb())
        }
        fn read_status(&mut self) -> Result<misa_actuator::MotorStatus> {
            Ok(misa_actuator::MotorStatus {
                voltage_v: 24.0,
                temperature_c: 30.0,
                error: Default::default(),
            })
        }
    }
}
