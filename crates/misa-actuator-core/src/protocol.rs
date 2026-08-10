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
    JobStarted {
        spec: JobSpec,
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
}
