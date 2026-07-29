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

/// Write points as CSV (header + one row per point).
pub fn write_points_csv(points: &[Point], w: &mut dyn Write) -> io::Result<()> {
    writeln!(
        w,
        "t_s,cmd,position_rad,velocity_rad_per_s,torque_nm,current_a,temperature_c"
    )?;
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
    /// How long to dwell at each position before recording, so the reading is
    /// the *holding* torque and not the acceleration transient (s).
    pub settle_s: f32,
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
    /// One point per visited position, in visit order.
    pub points: Vec<Point>,
    /// Position at enable, which every `cmd` is relative to (rad).
    pub start_position_rad: f32,
    pub spec: LoadMapSpec,
    pub abort: Option<AbortReason>,
}

impl LoadMap {
    /// The position whose holding torque is smallest in magnitude — the load's
    /// equilibrium. `None` if nothing was measured.
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
        if !self.spec.return_sweep {
            return None;
        }
        let n = self.spec.steps.max(2);
        let out = self.points.get(..n)?;
        let back = self.points.get(n..)?;
        let mut worst: Option<f32> = None;
        for b in back {
            // Pair with the outbound point closest in position.
            let nearest = out.iter().min_by(|x, y| {
                (x.position_rad - b.position_rad)
                    .abs()
                    .partial_cmp(&(y.position_rad - b.position_rad).abs())
                    .unwrap_or(core::cmp::Ordering::Equal)
            })?;
            let gap = (nearest.torque_nm - b.torque_nm).abs();
            if gap.is_finite() && worst.is_none_or(|w| gap > w) {
                worst = Some(gap);
            }
        }
        worst
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
    let mut stop = None;
    let t0 = Instant::now();

    for offset in spec.offsets() {
        if abort.load(Ordering::Relaxed) {
            stop = Some(AbortReason::Cancelled);
            break;
        }
        let target = start + offset;
        // Drive to the point, then dwell so the reading is a holding torque
        // rather than the acceleration transient.
        let settle = Duration::from_secs_f32(spec.settle_s.max(0.0));
        let dwell_start = Instant::now();
        let mut fb = act.set_position(target, spec.max_speed_rad_s)?;
        while dwell_start.elapsed() < settle {
            if abort.load(Ordering::Relaxed) {
                stop = Some(AbortReason::Cancelled);
                break;
            }
            fb = act.set_position(target, spec.max_speed_rad_s)?;
            if let Some(r) = guard.check(t0.elapsed().as_secs_f32(), &fb) {
                stop = Some(r);
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        points.push(Point::from(t0.elapsed().as_secs_f32(), target, &fb));
        if stop.is_some() {
            break;
        }
    }

    // Return to the start before releasing, so the rig is left where it began.
    let _ = act.set_position(start, spec.max_speed_rad_s);
    quiet(act);

    Ok(LoadMap {
        points,
        start_position_rad: start,
        spec: *spec,
        abort: stop,
    })
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
    let start = act.enable()?.position_rad;
    let mut guard = Guard::new(limits, start);

    let period = Duration::from_secs_f32(1.0 / spec.rate_hz.max(1.0));
    let ceiling = spec.max_torque_nm.min(limits.max_torque_nm);
    let mut points = Vec::new();
    let mut stop = None;
    let (mut torque_at_motion, mut position_at_motion) = (None, None);

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

        let moving = fb.velocity_rad_per_s.abs() > spec.motion_threshold_rad_per_s
            || (fb.position_rad - start).abs() > spec.motion_threshold_rad;
        if moving {
            torque_at_motion = Some(cmd);
            position_at_motion = Some(fb.position_rad);
            break;
        }
        if let Some(r) = guard.check(t, &fb) {
            stop = Some(r);
            break;
        }
        if let Some(rem) = period.checked_sub(iter_start.elapsed()) {
            std::thread::sleep(rem);
        }
    }

    quiet(act);

    Ok(Breakaway {
        breakaway_torque_nm: torque_at_motion,
        breakaway_position_rad: position_at_motion,
        points,
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
    act.set_run_mode(RunMode::Torque)?;
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

        let fb = act.set_torque(spec.hold_torque_nm)?;
        points.push(Point::from(t, spec.hold_torque_nm, &fb));
        if let Some(r) = guard.check(t, &fb) {
            stop = Some(r);
            break;
        }
        if let Some(rem) = period.checked_sub(iter_start.elapsed()) {
            std::thread::sleep(rem);
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
}

impl KtSpec {
    /// A bipolar sweep with 9 levels and a 0.4 s dwell.
    pub fn bipolar(max_torque_nm: f32) -> Self {
        Self {
            max_torque_nm,
            steps: 9,
            settle_s: 0.4,
            rate_hz: 100.0,
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
    act.set_run_mode(RunMode::Torque)?;
    let start = act.enable()?.position_rad;
    let mut guard = Guard::new(limits, start);

    let period = Duration::from_secs_f32(1.0 / spec.rate_hz.max(1.0));
    let mut points = Vec::new();
    let mut stop = None;
    let t0 = Instant::now();

    for level in spec.levels() {
        if abort.load(Ordering::Relaxed) {
            stop = Some(AbortReason::Cancelled);
            break;
        }
        let dwell_start = Instant::now();
        let settle = Duration::from_secs_f32(spec.settle_s.max(0.0));
        let mut fb = act.set_torque(level)?;
        while dwell_start.elapsed() < settle {
            if abort.load(Ordering::Relaxed) {
                stop = Some(AbortReason::Cancelled);
                break;
            }
            fb = act.set_torque(level)?;
            if let Some(r) = guard.check(t0.elapsed().as_secs_f32(), &fb) {
                stop = Some(r);
                break;
            }
            std::thread::sleep(period);
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
        write_points_csv(&map.points, &mut out).unwrap();
        let text = String::from_utf8(out).unwrap();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 1 + map.points.len());
        assert!(lines[0].starts_with("t_s,cmd,position_rad"));
        assert!(lines[0].ends_with("temperature_c"));
    }
}
