//! What crosses the boundary between a UI and the motor I/O thread.
//!
//! The split into [`Setpoint`] and [`Command`] is the load-bearing decision
//! here, and it is not stylistic.
//!
//! A slider being dragged wants to update the commanded position every frame.
//! Each bus transaction costs about a millisecond, so at 60 Hz of UI events
//! against a bus that can absorb maybe 1000 commands a second, a queue would
//! *mostly* keep up — right up until it does not, and then it grows without
//! bound and the motor keeps executing commands for seconds after the user
//! let go of the mouse. That is not a latency problem, it is a safety one.
//!
//! So continuous quantities live in a [`Setpoint`] the UI **overwrites** and
//! the worker samples at its own rate, and only genuinely discrete events —
//! enable, zero, start a scan — travel as queued [`Command`]s.

use std::time::Duration;

use misa_actuator::RunMode;
use serde::{Deserialize, Serialize};

/// Which control law the worker should be driving.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ControlMode {
    Position,
    Velocity,
    Torque,
    Mit,
}

impl From<ControlMode> for RunMode {
    fn from(m: ControlMode) -> Self {
        match m {
            ControlMode::Position => RunMode::Position,
            ControlMode::Velocity => RunMode::Velocity,
            ControlMode::Torque => RunMode::Torque,
            ControlMode::Mit => RunMode::Mit,
        }
    }
}

/// The continuously-updated command state.
///
/// Overwritten by the UI whenever a control changes, read by the worker once
/// per tick. Never queued — see the module docs.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Setpoint {
    pub mode: ControlMode,
    /// Position reference (rad), for `Position` and `Mit`.
    pub position_rad: f32,
    /// Velocity reference (rad/s), for `Velocity` and `Mit`.
    pub velocity_rad_s: f32,
    /// Torque command (N·m), for `Torque`.
    pub torque_nm: f32,
    /// MIT stiffness (N·m/rad).
    pub kp: f32,
    /// MIT damping (N·m·s/rad).
    pub kd: f32,
    /// MIT feed-forward torque (N·m).
    pub torque_ff_nm: f32,
    /// Speed cap for `Position` (rad/s).
    pub max_speed_rad_s: f32,
}

impl Default for Setpoint {
    fn default() -> Self {
        Self {
            mode: ControlMode::Position,
            position_rad: 0.0,
            velocity_rad_s: 0.0,
            torque_nm: 0.0,
            kp: 10.0,
            kd: 0.5,
            torque_ff_nm: 0.0,
            max_speed_rad_s: 2.0,
        }
    }
}

impl Setpoint {
    /// The value a command-vs-response plot should show as "commanded" for the
    /// current mode.
    pub fn primary(&self) -> f32 {
        match self.mode {
            ControlMode::Position | ControlMode::Mit => self.position_rad,
            ControlMode::Velocity => self.velocity_rad_s,
            ControlMode::Torque => self.torque_nm,
        }
    }
}

/// Which channel a chirp drives, mirroring [`misa_sysid::Excitation`].
///
/// Kept as its own type because the sysid one is not `serde`-shaped and
/// carries its gains inline; here they live on [`ChirpJob`] so a UI can bind
/// each field to a control.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ExcitationKind {
    /// Position reference. The chirp rides on the start position, so motion
    /// stays bounded — the safe default.
    Position,
    Velocity,
    /// Open-loop torque. Position can drift; short runs and small amplitudes.
    Torque,
    /// MIT position reference with the configured `kp`/`kd`.
    MitPosition,
    /// MIT torque feed-forward with a `kp`/`kd` leash holding the shaft near
    /// its start position. The safe way to identify the open-loop plant.
    MitTorque,
}

impl ExcitationKind {
    /// Whether this channel is open-loop enough to run away if the amplitude
    /// is wrong. A UI should say so before starting one.
    pub fn is_open_loop(self) -> bool {
        matches!(self, ExcitationKind::Velocity | ExcitationKind::Torque)
    }
}

/// A swept-sine identification run.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ChirpJob {
    pub excitation: ExcitationKind,
    pub f_start_hz: f32,
    pub f_end_hz: f32,
    pub duration_s: f32,
    /// Amplitude in the channel's own units — rad, rad/s or N·m.
    pub amplitude: f32,
    /// Logarithmic sweep. Spends more time at low frequencies, which is
    /// usually where the interesting dynamics are.
    pub log_sweep: bool,
    pub rate_hz: f32,
    /// MIT stiffness, for the two MIT channels.
    pub kp: f32,
    /// MIT damping, for the two MIT channels.
    pub kd: f32,
    /// Speed cap for the position channel (rad/s).
    pub max_speed_rad_s: f32,
}

impl Default for ChirpJob {
    fn default() -> Self {
        Self {
            excitation: ExcitationKind::Position,
            f_start_hz: 0.5,
            f_end_hz: 30.0,
            duration_s: 10.0,
            amplitude: 0.15,
            log_sweep: true,
            rate_hz: 500.0,
            kp: 10.0,
            kd: 0.5,
            max_speed_rad_s: 5.0,
        }
    }
}

/// A quasi-static characterization run.
///
/// These share the chirp's lifecycle — the worker owns the loop, `STOP` cuts
/// them short — but not its result shape, so they report through
/// [`Event::CharacterizeFinished`] rather than [`Event::JobFinished`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case", rename_all_fields = "camelCase", tag = "run")]
pub enum CharacterizeJob {
    /// Step through positions and record the holding torque at each: the load
    /// the motor is working against, as a function of where it is.
    LoadMap {
        from_rad: f32,
        to_rad: f32,
        steps: u32,
        settle_s: f32,
        max_speed_rad_s: f32,
        /// Sweep back as well, so hysteresis shows up as a gap between the
        /// two directions instead of hiding inside one curve.
        return_sweep: bool,
    },
    /// Traverse at a constant slow speed and split the torque by direction:
    /// **kinetic** friction, as opposed to the static friction
    /// [`Self::Breakaway`] measures.
    ///
    /// Sweeping both ways is what makes the split possible. In steady motion the
    /// torque is load ± friction, so the half-difference between the two
    /// directions is friction and the half-sum is load. One direction alone
    /// cannot separate them, which is why `return_sweep` defaults on and turning
    /// it off is reported rather than assumed.
    ///
    /// **Needs working torque feedback.** Some RobStride firmware reports
    /// `MeasuredTorque` as a constant 0 (`doc/handover.md` §4); there the run
    /// completes and every reading is zero, so the result is checked and
    /// reported as "no torque was reported" rather than as a frictionless joint.
    VelocitySweep {
        /// Traverse ± this far around the current position.
        half_span_rad: f32,
        /// Slow enough that inertia does not contribute.
        speed_rad_s: f32,
        rate_hz: f32,
        return_sweep: bool,
        /// Speed bins the friction curve is averaged into.
        bins: u32,
    },
    /// Ramp torque until the shaft breaks loose. The torque it goes at is the
    /// stiction the motor has to overcome before it can move at all.
    Breakaway {
        ramp_nm_per_s: f32,
        max_torque_nm: f32,
        positive: bool,
        rate_hz: f32,
    },
    /// Step through torque levels and fit torque against current: the **torque
    /// constant**, N·m/A.
    ///
    /// Exists mostly to feed the other runs. Firmware that reports a constant
    /// zero torque makes every torque-based measurement read zero, and the way
    /// out is to derive torque from current — which needs a Kt that until now
    /// the operator had to find elsewhere and type in. This run measures it on
    /// the motor that is already connected, so the number stops being
    /// something to look up.
    ///
    /// Needs a **leash**: a free shaft cannot be held at a torque level long
    /// enough to read a steady-state current, so the levels are commanded in
    /// MIT mode against a position hold. `leash_kp`/`leash_kd` of zero fall
    /// back to plain torque control, which only works on a shaft that is
    /// already restrained.
    Kt {
        /// Largest level in the sweep, N·m. Levels run from `-max` to `+max`,
        /// so the fit sees both directions and any offset shows up as an
        /// intercept rather than skewing the slope.
        max_torque_nm: f32,
        steps: u32,
        /// Dwell at each level. Long enough that current has settled — this is
        /// also what makes reading current affordable on families that need a
        /// second transaction for it.
        settle_s: f32,
        rate_hz: f32,
        leash_kp: f32,
        leash_kd: f32,
    },
}

impl CharacterizeJob {
    /// A label for the run, for logs and the UI.
    pub fn name(&self) -> &'static str {
        match self {
            CharacterizeJob::LoadMap { .. } => "load map",
            CharacterizeJob::VelocitySweep { .. } => "velocity sweep",
            CharacterizeJob::Breakaway { .. } => "breakaway",
            CharacterizeJob::Kt { .. } => "kt",
        }
    }

    /// Roughly how long the run will take, and whether that is a ceiling.
    ///
    /// Returns `(duration, is_upper_bound)`. There is no progress callback out
    /// of `misa_sysid` — the runs are blocking calls — so a progress bar has to
    /// divide real elapsed time by a figure derived from the spec. Being
    /// derived, it is an estimate, and a UI that draws it as certainty will
    /// eventually draw a bar that sits at 100% while the run continues.
    ///
    /// `is_upper_bound` is true for [`Self::Breakaway`], which stops as soon as
    /// the shaft moves and so normally finishes well before its ramp completes.
    /// Every other run dwells for a fixed schedule.
    pub fn expected_duration(&self) -> (Duration, bool) {
        let secs = match *self {
            CharacterizeJob::LoadMap {
                from_rad,
                to_rad,
                steps,
                settle_s,
                max_speed_rad_s,
                return_sweep,
            } => {
                let steps = steps.max(2) as f32;
                // Each step dwells, and before dwelling it travels one gap at
                // the speed limit. Ignoring the travel would under-estimate a
                // wide slow sweep by more than the dwells themselves.
                let gap = (to_rad - from_rad).abs() / (steps - 1.0).max(1.0);
                let travel = if max_speed_rad_s > 0.0 {
                    gap / max_speed_rad_s
                } else {
                    0.0
                };
                steps * (settle_s.max(0.0) + travel) * if return_sweep { 2.0 } else { 1.0 }
            }
            CharacterizeJob::VelocitySweep {
                half_span_rad,
                speed_rad_s,
                return_sweep,
                ..
            } => {
                if speed_rad_s > 0.0 {
                    2.0 * half_span_rad.abs() / speed_rad_s * if return_sweep { 2.0 } else { 1.0 }
                } else {
                    0.0
                }
            }
            CharacterizeJob::Breakaway {
                ramp_nm_per_s,
                max_torque_nm,
                ..
            } => {
                if ramp_nm_per_s > 0.0 {
                    max_torque_nm.abs() / ramp_nm_per_s
                } else {
                    0.0
                }
            }
            CharacterizeJob::Kt {
                steps, settle_s, ..
            } => steps.max(2) as f32 * settle_s.max(0.0),
        };
        // A non-finite or negative estimate would divide a progress bar into
        // nonsense; zero is the agreed "no estimate" value.
        let secs = if secs.is_finite() && secs > 0.0 { secs } else { 0.0 };
        (
            Duration::from_secs_f32(secs),
            matches!(self, CharacterizeJob::Breakaway { .. }),
        )
    }

    /// Whether the run reads motor current, and so needs current reporting
    /// switched on for its duration.
    ///
    /// Only the Kt fit does. The others read torque, and on families where
    /// current costs an extra transaction per sample, paying for it would
    /// halve their sample rate for nothing.
    pub fn needs_current(&self) -> bool {
        matches!(self, CharacterizeJob::Kt { .. })
    }
}

/// The limits and constants a characterization run is given, per run.
///
/// Per run rather than per connection, and this was not the first shape. Both
/// values started as connect-time options, which meant reconnecting to change
/// either one and put two fields in the connection bar that nothing else on
/// screen used — the bar is about reaching a motor, and neither of these is.
/// They belong to the measurement: the ceiling is how hard the operator is
/// willing to push this rig for *this* run, and the Kt may be a figure a Kt
/// run has only just produced.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RunEnvelope {
    /// Torque ceiling for the run, N·m. `0.0` keeps
    /// [`misa_sysid::SafetyLimits::gentle`]'s 1 N·m, which is sized for the
    /// smallest motor in the workspace and is below what a geared joint needs
    /// just to move.
    pub max_torque_nm: f32,
    /// Torque constant, N·m/A. `0.0` means "report the torque the motor
    /// reports". Set it for firmware that reports a constant zero torque; the
    /// driver then derives torque from current for the run's duration and puts
    /// it back afterwards.
    pub kt: f32,
}

/// What a characterization run produced.
///
/// Deliberately generic: one x/y series plus named findings. The runs measure
/// different things, but every one of them is "a curve and a few numbers", and
/// a per-run result type would make the UI carry a branch for each without
/// showing the operator anything more.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CharacterizeData {
    pub x_label: String,
    pub y_label: String,
    pub x: Vec<f32>,
    pub y: Vec<f32>,
    /// What the first series is. Named per run rather than fixed, because
    /// "outbound / return" and "commanded / measured" are both two series on
    /// shared axes but mean entirely different things.
    pub series: String,
    /// A second series on the same axes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub x2: Option<Vec<f32>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub y2: Option<Vec<f32>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub series2: Option<String>,
    /// Named findings, e.g. `("breakaway torque", "0.902 N·m")`.
    pub summary: Vec<(String, String)>,
}

/// A long-running measurement the worker owns for its whole duration.
///
/// Unlike a [`Command`], a job blocks the worker's loop — a chirp is a control
/// loop, and interleaving it with anything else on the same bus would corrupt
/// it. Stopping one therefore goes through [`crate::Session::stop`], not
/// through the command queue.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case", tag = "job")]
pub enum JobSpec {
    Chirp(ChirpJob),
    Characterize {
        /// Flattened, so the run's own tag stays at the top level and the
        /// wire shape is `{"job":"characterize","run":"breakaway",…}` rather
        /// than nesting a `run` object inside a `run` field.
        #[serde(flatten)]
        run: CharacterizeJob,
        #[serde(default)]
        envelope: RunEnvelope,
    },
}

/// A Bode estimate, as [`misa_sysid::FreqResponse`] but serde-shaped.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BodeData {
    pub freqs_hz: Vec<f32>,
    pub magnitude_db: Vec<f32>,
    pub phase_deg: Vec<f32>,
    /// Magnitude-squared coherence in `[0, 1]`. Points where this is low are
    /// not measurements — the excitation did not reach that frequency, or
    /// noise dominated it — and a UI should de-emphasise them rather than
    /// drawing them as confidently as the rest.
    pub coherence: Vec<f32>,
}

impl From<misa_sysid::FreqResponse> for BodeData {
    fn from(f: misa_sysid::FreqResponse) -> Self {
        Self {
            freqs_hz: f.freqs_hz,
            magnitude_db: f.magnitude_db,
            phase_deg: f.phase_deg,
            coherence: f.coherence,
        }
    }
}

/// A one-off instruction to the worker.
///
/// Note both rename attributes: `rename_all` covers variant *names*,
/// `rename_all_fields` covers the fields inside them. Without the second one a
/// field like `timeout_ms` crosses the wire in snake_case while everything
/// else the UI sees is camelCase — and the mismatch stays invisible until
/// somebody adds the first multi-word field, then shows up as an undefined
/// value rather than as a parse error.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case", rename_all_fields = "camelCase", tag = "kind")]
pub enum Command {
    Enable,
    Disable,
    SetZero,
    /// Pre-configure the run mode without issuing a control command.
    SetRunMode { mode: ControlMode },
    /// One read, outside the polling cadence.
    Measure,
    ReadStatus,
    /// Read every setting the motor reports. Read-only; there is no write
    /// counterpart, deliberately — see `Actuator::read_parameters`.
    ///
    /// `deep` opts into reverse-engineered address spaces. It stops streaming
    /// first: this is dozens of round trips and interleaving them with control
    /// frames would disturb both.
    ReadParameters { deep: bool },
    /// Begin an incremental bus scan. Progress arrives as
    /// [`Event::ScanProgress`]; the worker probes one id per tick so the UI
    /// keeps updating through it.
    StartScan { from: u8, to: u8, timeout_ms: u64 },
    CancelScan,
    /// Start or stop pushing the [`Setpoint`] to the motor every tick.
    ///
    /// While streaming, the watchdog is armed: a UI that stops sending
    /// heartbeats gets its motor disabled.
    SetStreaming { on: bool },
    /// Worker tick rate (Hz). Bounded by the worker to something sane.
    SetRate { hz: f32 },
    /// Start a long-running measurement. Cancel it with
    /// [`crate::Session::stop`] — a job owns the loop, so a queued cancel
    /// would not be read until it had already finished.
    StartJob { spec: JobSpec },
    /// Disconnect and end the worker thread.
    Shutdown,
}

/// Why the worker stopped driving the motor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum StopReason {
    /// Someone pressed stop.
    UserRequest,
    /// The UI stopped sending heartbeats — hung renderer, crashed webview, or
    /// a frozen laptop lid.
    Watchdog,
    /// The bus failed in a way the worker could not continue through.
    Fault,
    /// Ordinary shutdown.
    Shutdown,
}

impl StopReason {
    /// Whether this stop indicates something went wrong, as opposed to being
    /// asked for.
    pub fn is_unexpected(self) -> bool {
        matches!(self, StopReason::Watchdog | StopReason::Fault)
    }
}

/// One motor setting, as [`misa_actuator::Parameter`] but serde-shaped.
///
/// The value is flattened to a string rather than kept as a tagged union.
/// These are read-only and exist to be looked at and compared, so a UI never
/// needs to do arithmetic on one — and a union would make the common case (a
/// table, a TOML dump, a diff against last time) harder for nothing.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ParameterRow {
    pub group: String,
    pub name: String,
    /// Formatted value, or empty when the read failed.
    pub value: String,
    /// Set instead of `value` when the motor would not answer, carrying why.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub unavailable: Option<String>,
    pub unit: String,
    /// Driver-native address, e.g. `"RID 21"`, `"0x2007"`, `"0xC0[0x3E]"`.
    pub address: String,
    /// From a reverse-engineered address space rather than a documented one.
    pub undocumented: bool,
}

/// Six decimals, with the trailing zeros taken back off.
///
/// Both halves matter. Six decimals keeps a gain of `0.00453008` from
/// rounding to `0.0045`; trimming keeps a limit of `225` from rendering as
/// `225.000000`, which buries the numbers that do carry precision in a column
/// of noise. Non-finite values are printed as-is — a `hard_knee` of `inf` is
/// a real configuration meaning "no hard stop", not a formatting failure.
fn format_float(v: f32) -> String {
    if !v.is_finite() {
        return format!("{v}");
    }
    let s = format!("{v:.6}");
    let s = s.trim_end_matches('0');
    s.trim_end_matches('.').to_string()
}

impl From<misa_actuator::Parameter> for ParameterRow {
    fn from(p: misa_actuator::Parameter) -> Self {
        use misa_actuator::ParamValue as V;
        let (value, unavailable) = match p.value {
            V::Float(v) => (format_float(v), None),
            V::Int(v) => (v.to_string(), None),
            V::Bool(v) => (v.to_string(), None),
            V::Text(v) => (v, None),
            V::Unavailable(why) => (String::new(), Some(why)),
        };
        Self {
            group: p.group.to_string(),
            name: p.name,
            value,
            unavailable,
            unit: p.unit.to_string(),
            address: p.address,
            undocumented: p.undocumented,
        }
    }
}

/// Where the session is in its life cycle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(
    rename_all = "kebab-case",
    rename_all_fields = "camelCase",
    tag = "state"
)]
pub enum ConnectionState {
    Connected {
        /// What the transport reports it opened, e.g. `PCAN_USBBUS1 (CAN, ...)`.
        description: String,
        motor_id: u8,
        /// What the motor said about itself, when it can say anything — the
        /// MIT ranges a DAMIAO reports, or the limits a RobStride reports for
        /// cross-checking the selected model. Empty when the driver has no way
        /// to ask.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        identity: Option<String>,
        /// Set when what the motor reported contradicts how it was opened.
        /// Worth a coloured line rather than a log entry: the consequence is
        /// silently mis-scaled torque, not a visible failure.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        identity_warning: Option<String>,
    },
    Disconnected,
}

/// One synchronized command + response sample.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Sample {
    /// Seconds since the session started.
    pub t_s: f64,
    /// The setpoint's primary value at the time of the command.
    pub commanded: f32,
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    pub torque_nm: f32,
    pub current_a: f32,
    pub temperature_c: f32,
}

/// Slow-changing motor health, polled well below the control rate.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct StatusSnapshot {
    pub voltage_v: f32,
    pub temperature_c: f32,
    /// Common fault bits, as `misa_actuator::ErrorFlags::bits`.
    pub error_bits: u32,
    /// Driver-native fault word, for diagnostics.
    pub error_raw: u32,
    pub enabled: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum LogLevel {
    Info,
    Warn,
    Error,
}

/// Everything the worker reports back, other than bulk telemetry.
///
/// Telemetry travels on its own bounded channel because it is the one thing
/// here that is safe to drop: a UI that falls behind should lose plot samples,
/// never a state change or an error.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case", rename_all_fields = "camelCase", tag = "event")]
pub enum Event {
    State(ConnectionState),
    Status(StatusSnapshot),
    /// A single out-of-band reading, from [`Command::Measure`].
    Reading(Sample),
    ScanProgress {
        done: u32,
        total: u32,
        current: u8,
        found: Vec<u8>,
    },
    ScanFinished {
        found: Vec<u8>,
        cancelled: bool,
    },
    Stopped {
        reason: StopReason,
    },
    /// The answer to [`Command::ReadParameters`].
    ///
    /// Rows arrive in the order the driver emitted them, which is its own
    /// grouping — the families do not share a taxonomy and imposing one here
    /// would only misfile things.
    Parameters {
        rows: Vec<ParameterRow>,
        /// Whether reverse-engineered spaces were included, so a UI can say
        /// which kind of dump it is showing rather than leaving the reader to
        /// infer it from the row count.
        deep: bool,
    },
    JobStarted {
        spec: JobSpec,
    },
    /// The answer to a [`JobSpec::Characterize`] run.
    ///
    /// `data` is `None` when the run was cut short before it had measured
    /// anything — distinct from an empty series, which would plot as a real
    /// measurement of nothing.
    CharacterizeFinished {
        run: CharacterizeJob,
        data: Option<CharacterizeData>,
        aborted: bool,
        /// The torque constant a [`CharacterizeJob::Kt`] run fitted, N·m/A.
        ///
        /// Typed rather than left in `data.summary`, because a UI acts on it:
        /// it fills in the Kt the other runs need. A number a client has to
        /// scrape back out of a display string is a number it will
        /// occasionally scrape wrong.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        fitted_kt: Option<f32>,
        /// A ceiling worth retrying at, N·m, when the torque limit is what
        /// stopped the run.
        ///
        /// Not a measurement of what the rig needs — the guard trips as soon
        /// as torque exceeds the ceiling, so every observed sample sits just
        /// above it and says nothing about how much more would be enough.
        /// It is a stated multiple of the ceiling that was in force, offered
        /// so the operator can retry in one click instead of guessing at a
        /// field. Silently raising the limit would be the wrong favour.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        suggested_torque_limit_nm: Option<f32>,
    },
    JobProgress {
        elapsed_s: f32,
        duration_s: f32,
        samples: u32,
    },
    JobFinished {
        /// `None` when the run was cut short before there was enough data to
        /// estimate anything.
        bode: Option<BodeData>,
        achieved_rate_hz: f32,
        samples: u32,
        /// Peak-to-peak of the excitation, in the channel's units.
        command_pp: f32,
        /// Peak-to-peak of whatever the FRF treats as the output.
        ///
        /// Reported so a UI can say "the shaft did not move" rather than
        /// drawing the −240 dB line that a zero response produces. A Bode plot
        /// of nothing looks like a measurement; it is not one.
        response_pp: f32,
        /// Whether the run was stopped rather than completing its sweep. The
        /// partial log is still returned — a chirp that aborted at 7 of 10
        /// seconds identified the band it did reach.
        aborted: bool,
    },
    JobFailed {
        message: String,
    },
    Log {
        level: LogLevel,
        message: String,
    },
}

/// A batch of samples, emitted at UI frame rate rather than per sample.
///
/// At a 1 kHz control rate, one message per sample would mean 1000 IPC
/// crossings a second for data the UI redraws 60 times a second. Batching
/// makes the cost of the boundary irrelevant.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TelemetryBatch {
    pub samples: Vec<Sample>,
    /// Samples the worker had to discard because the consumer was not keeping
    /// up. Non-zero means the plot has gaps, and the UI should say so rather
    /// than silently drawing a smooth line through them.
    pub dropped: u64,
    /// Control-loop rate actually achieved since the last batch (Hz).
    pub achieved_rate_hz: f32,
}

/// Worker tick rates the session will accept.
pub const MIN_RATE_HZ: f32 = 1.0;
pub const MAX_RATE_HZ: f32 = 2_000.0;
pub const DEFAULT_RATE_HZ: f32 = 200.0;

/// How often status is polled, relative to the control loop.
pub const STATUS_INTERVAL: Duration = Duration::from_millis(500);

/// How often telemetry batches are flushed to the consumer.
pub const TELEMETRY_INTERVAL: Duration = Duration::from_millis(16);

/// How long the worker will drive the motor without hearing from the UI.
///
/// Only enforced while streaming — an idle session that is merely polling
/// cannot run away, so nagging it would be noise.
pub const DEFAULT_WATCHDOG: Duration = Duration::from_millis(750);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_primary_value_follows_the_mode() {
        let sp = Setpoint {
            mode: ControlMode::Torque,
            position_rad: 1.0,
            velocity_rad_s: 2.0,
            torque_nm: 3.0,
            ..Setpoint::default()
        };
        assert_eq!(sp.primary(), 3.0);
        assert_eq!(
            Setpoint {
                mode: ControlMode::Velocity,
                ..sp
            }
            .primary(),
            2.0
        );
        assert_eq!(
            Setpoint {
                mode: ControlMode::Position,
                ..sp
            }
            .primary(),
            1.0
        );
        // MIT tracks a position reference, so it plots against position.
        assert_eq!(
            Setpoint {
                mode: ControlMode::Mit,
                ..sp
            }
            .primary(),
            1.0
        );
    }

    #[test]
    fn control_modes_map_onto_the_driver_run_modes() {
        assert_eq!(RunMode::from(ControlMode::Position), RunMode::Position);
        assert_eq!(RunMode::from(ControlMode::Velocity), RunMode::Velocity);
        assert_eq!(RunMode::from(ControlMode::Torque), RunMode::Torque);
        assert_eq!(RunMode::from(ControlMode::Mit), RunMode::Mit);
    }

    #[test]
    fn parameter_values_keep_precision_without_padding_it() {
        use misa_actuator::{ParamValue, Parameter};

        let row = |v: f32| ParameterRow::from(Parameter::float("g", "n", v, "", "a")).value;
        // A round limit should not read as `225.000000`.
        assert_eq!(row(225.0), "225");
        assert_eq!(row(0.5), "0.5");
        // ...but a gain must not round to `0.0045`.
        assert_eq!(row(0.004_530_08), "0.00453");
        assert_eq!(row(12.5), "12.5");
        // `inf` is a real configuration ("no hard stop"), not a failure.
        assert_eq!(row(f32::INFINITY), "inf");

        // An unanswered read carries its reason and leaves the value empty,
        // so a UI can tell "nothing there" from "zero".
        let missing = ParameterRow::from(Parameter::new(
            "g",
            "n",
            ParamValue::Unavailable("timeout".into()),
            "",
            "a",
        ));
        assert_eq!(missing.value, "");
        assert_eq!(missing.unavailable.as_deref(), Some("timeout"));
    }

    #[test]
    fn only_watchdog_and_fault_count_as_unexpected() {
        assert!(StopReason::Watchdog.is_unexpected());
        assert!(StopReason::Fault.is_unexpected());
        assert!(!StopReason::UserRequest.is_unexpected());
        assert!(!StopReason::Shutdown.is_unexpected());
    }

    #[test]
    fn the_wire_format_is_stable_enough_to_hand_to_a_ui() {
        // The GUI's TypeScript side depends on these shapes, so a rename here
        // should fail a test rather than a running app.
        let json = serde_json::to_string(&Command::StartScan {
            from: 1,
            to: 32,
            timeout_ms: 50,
        })
        .unwrap();
        assert!(json.contains("\"kind\":\"start-scan\""), "{json}");
        assert!(json.contains("\"timeoutMs\":50"), "{json}");

        // `ConnectionState` is the first event a UI ever sees, so a field it
        // cannot read leaves the app looking connected to nothing.
        let json = serde_json::to_string(&Event::State(ConnectionState::Connected {
            description: "robstride RS-04 on pcan:usb1".into(),
            motor_id: 1,
            identity: Some("motor reports limit_torque 17.000 N·m".into()),
            identity_warning: None,
        }))
        .unwrap();
        assert!(json.contains("\"state\":\"connected\""), "{json}");
        assert!(json.contains("\"motorId\":1"), "{json}");
        assert!(json.contains("\"identity\":\"motor reports"), "{json}");
        // Absent, not null: the frontend treats the field as optional and a
        // literal null would render as the string "null" in a banner.
        assert!(!json.contains("identityWarning"), "{json}");

        let json = serde_json::to_string(&Event::Stopped {
            reason: StopReason::Watchdog,
        })
        .unwrap();
        assert!(json.contains("\"event\":\"stopped\""), "{json}");
        assert!(json.contains("\"reason\":\"watchdog\""), "{json}");

        // Job events are the newest and the most nested — `JobSpec` is an
        // internally-tagged enum inside an internally-tagged enum — so their
        // shape is the most likely to drift away from what the UI reads.
        let json = serde_json::to_string(&Event::JobStarted {
            spec: JobSpec::Chirp(ChirpJob::default()),
        })
        .unwrap();
        assert!(json.contains("\"event\":\"job-started\""), "{json}");
        assert!(json.contains("\"job\":\"chirp\""), "{json}");
        assert!(json.contains("\"durationS\":10.0"), "{json}");
        assert!(json.contains("\"excitation\":\"position\""), "{json}");

        // A characterization run is the same nesting again, and the two friction
        // runs are the pair most easily confused: one measures static friction,
        // the other kinetic, and they differ only by the tag.
        let json = serde_json::to_string(&Event::JobStarted {
            spec: JobSpec::Characterize {
                run: CharacterizeJob::Breakaway {
                    ramp_nm_per_s: 0.2,
                    max_torque_nm: 1.0,
                    positive: true,
                    rate_hz: 200.0,
                },
                envelope: RunEnvelope {
                    max_torque_nm: 3.0,
                    kt: 1.5872,
                },
            },
        })
        .unwrap();
        assert!(json.contains("\"job\":\"characterize\""), "{json}");
        assert!(json.contains("\"run\":\"breakaway\""), "{json}");
        assert!(json.contains("\"rampNmPerS\":0.2"), "{json}");
        // The envelope is a sibling of the run's own fields, not a wrapper
        // around them: flattening the run is what keeps the tag at the top
        // level where every client already looks for it.
        assert!(json.contains("\"envelope\":{"), "{json}");
        assert!(json.contains("\"kt\":1.5872"), "{json}");

        let json = serde_json::to_string(&Event::JobStarted {
            spec: JobSpec::Characterize {
                run: CharacterizeJob::VelocitySweep {
                    half_span_rad: 0.2,
                    speed_rad_s: 0.05,
                    rate_hz: 200.0,
                    return_sweep: true,
                    bins: 12,
                },
                envelope: RunEnvelope::default(),
            },
        })
        .unwrap();
        assert!(json.contains("\"run\":\"velocity-sweep\""), "{json}");
        assert!(json.contains("\"halfSpanRad\":0.2"), "{json}");
        assert!(json.contains("\"speedRadS\":0.05"), "{json}");
        assert!(json.contains("\"returnSweep\":true"), "{json}");

        // And it survives the round trip, which is the half that flatten can
        // break: serde has to pull the run's tag back out of a buffered map.
        let spec = JobSpec::Characterize {
            run: CharacterizeJob::Kt {
                max_torque_nm: 1.2,
                steps: 9,
                settle_s: 0.4,
                rate_hz: 100.0,
                leash_kp: 8.0,
                leash_kd: 0.5,
            },
            envelope: RunEnvelope {
                max_torque_nm: 3.0,
                kt: 0.0,
            },
        };
        let json = serde_json::to_string(&spec).unwrap();
        assert!(json.contains("\"run\":\"kt\""), "{json}");
        assert_eq!(serde_json::from_str::<JobSpec>(&json).unwrap(), spec);

        let json = serde_json::to_string(&Event::JobProgress {
            elapsed_s: 1.5,
            duration_s: 10.0,
            samples: 750,
        })
        .unwrap();
        assert!(json.contains("\"elapsedS\":1.5"), "{json}");
        assert!(json.contains("\"durationS\":10.0"), "{json}");

        let json = serde_json::to_string(&Event::JobFinished {
            bode: Some(BodeData {
                freqs_hz: vec![1.0],
                magnitude_db: vec![-3.0],
                phase_deg: vec![-45.0],
                coherence: vec![0.99],
            }),
            achieved_rate_hz: 498.0,
            samples: 5000,
            command_pp: 0.3,
            response_pp: 0.28,
            aborted: false,
        })
        .unwrap();
        assert!(json.contains("\"event\":\"job-finished\""), "{json}");
        assert!(json.contains("\"freqsHz\":[1.0]"), "{json}");
        assert!(json.contains("\"achievedRateHz\":498.0"), "{json}");

        // The command the Run button sends.
        let cmd: Command = serde_json::from_str(
            r#"{"kind":"start-job","spec":{"job":"chirp","excitation":"mit-torque",
                "fStartHz":0.5,"fEndHz":30.0,"durationS":10.0,"amplitude":0.15,
                "logSweep":true,"rateHz":500.0,"kp":10.0,"kd":0.5,"maxSpeedRadS":5.0}}"#,
        )
        .expect("start-job should parse");
        let Command::StartJob {
            spec: JobSpec::Chirp(job),
        } = cmd
        else {
            panic!("wrong command");
        };
        assert_eq!(job.excitation, ExcitationKind::MitTorque);
        assert_eq!(job.duration_s, 10.0);

        let json = serde_json::to_string(&Sample {
            t_s: 1.0,
            commanded: 0.5,
            position_rad: 0.4,
            velocity_rad_s: 0.1,
            torque_nm: 0.2,
            current_a: 0.3,
            temperature_c: 30.0,
        })
        .unwrap();
        assert!(json.contains("\"positionRad\""), "{json}");
    }

    /// The progress bar divides real elapsed time by these, so an estimate that
    /// is wrong by a factor draws a bar that is wrong by a factor.
    #[test]
    fn a_runs_estimated_duration_matches_its_schedule() {
        // 13 dwells of 0.3 s, plus travel of one 0.05 rad gap at 0.5 rad/s
        // (0.1 s) before each, both ways: 13 × 0.4 × 2 = 10.4 s.
        let (d, bound) = CharacterizeJob::LoadMap {
            from_rad: -0.3,
            to_rad: 0.3,
            steps: 13,
            settle_s: 0.3,
            max_speed_rad_s: 0.5,
            return_sweep: true,
        }
        .expected_duration();
        assert!((d.as_secs_f32() - 10.4).abs() < 0.1, "{d:?}");
        assert!(!bound, "a load map dwells to a fixed schedule");

        // ±0.2 rad at 0.05 rad/s is 8 s one way, 16 s both.
        let (d, _) = CharacterizeJob::VelocitySweep {
            half_span_rad: 0.2,
            speed_rad_s: 0.05,
            rate_hz: 200.0,
            return_sweep: true,
            bins: 12,
        }
        .expected_duration();
        assert!((d.as_secs_f32() - 16.0).abs() < 0.1, "{d:?}");

        // 9 levels × 0.4 s.
        let (d, _) = CharacterizeJob::Kt {
            max_torque_nm: 1.2,
            steps: 9,
            settle_s: 0.4,
            rate_hz: 100.0,
            leash_kp: 8.0,
            leash_kd: 0.5,
        }
        .expected_duration();
        assert!((d.as_secs_f32() - 3.6).abs() < 0.05, "{d:?}");

        // 1 N·m at 0.2 N·m/s is 5 s of ramp — but only if the shaft never
        // breaks loose, which is the whole point of the run, so it is a
        // ceiling and must say so.
        let (d, bound) = CharacterizeJob::Breakaway {
            ramp_nm_per_s: 0.2,
            max_torque_nm: 1.0,
            positive: true,
            rate_hz: 200.0,
        }
        .expected_duration();
        assert!((d.as_secs_f32() - 5.0).abs() < 0.05, "{d:?}");
        assert!(bound, "a breakaway ramp normally ends early");
    }

    /// A zero or absurd parameter must yield "no estimate" rather than an
    /// infinity that a progress bar would divide by.
    #[test]
    fn a_run_that_cannot_be_estimated_says_zero() {
        let (d, _) = CharacterizeJob::VelocitySweep {
            half_span_rad: 0.2,
            speed_rad_s: 0.0,
            rate_hz: 200.0,
            return_sweep: true,
            bins: 12,
        }
        .expected_duration();
        assert_eq!(d, Duration::ZERO);

        let (d, _) = CharacterizeJob::Breakaway {
            ramp_nm_per_s: 0.0,
            max_torque_nm: 1.0,
            positive: true,
            rate_hz: 200.0,
        }
        .expected_duration();
        assert_eq!(d, Duration::ZERO);

        // NaN in, zero out — not a panic from `Duration::from_secs_f32`.
        let (d, _) = CharacterizeJob::Kt {
            max_torque_nm: 1.2,
            steps: 9,
            settle_s: f32::NAN,
            rate_hz: 100.0,
            leash_kp: 8.0,
            leash_kd: 0.5,
        }
        .expected_duration();
        assert_eq!(d, Duration::ZERO);
    }
}
