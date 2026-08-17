//! The `Actuator` trait — the bus-independent SI-unit motor-control API.

use core::ops::RangeInclusive;
use core::time::Duration;

use crate::error::{Error, Result};
use crate::feedback::{MotorFeedback, MotorStatus, RunMode};

/// High-level motor-control interface.
///
/// Implementations live in each driver crate (e.g. `lkmotor-driver`,
/// `robstride-driver`). The trait is dyn-compatible so that an application
/// can hold `Box<dyn Actuator>` and switch backends at runtime.
///
/// # Threading
///
/// Methods take `&mut self` because the underlying bus is single-owner —
/// concurrent issuance of commands on the same bus is a recipe for cross-
/// talk. To talk to multiple motors on one bus, share the bus inside an
/// `Arc<Mutex<...>>` at the driver level.
///
/// # Units
///
/// All quantities are output-frame SI (rad, rad/s, N·m). See
/// [`crate::feedback::MotorFeedback`] for details.
pub trait Actuator {
    /// Identifier of the underlying motor (CAN id / bus address). Returned
    /// only for logging / diagnostics — does not affect routing.
    fn motor_id(&self) -> u8;

    /// Enable closed-loop control. Most drivers also send a status frame in
    /// response, which is returned here for convenience; if a driver cannot
    /// produce a feedback frame at enable time it should return
    /// [`MotorFeedback::zero()`].
    fn enable(&mut self) -> Result<MotorFeedback>;

    /// Disable closed-loop control. The motor coasts.
    fn disable(&mut self) -> Result<()>;

    /// Set the **current physical position** as the new zero reference.
    ///
    /// On drivers that have no native command for this (lkmotor V3 case)
    /// the implementation may rezero by reading the absolute angle and
    /// caching an offset internally.
    fn set_zero(&mut self) -> Result<()>;

    /// Pre-configure the high-level control mode.
    ///
    /// Robstride distinguishes Position/Velocity/Torque/MIT modes via a
    /// `run_mode` parameter; lkmotor V3 doesn't (it uses a different
    /// command code per mode). Drivers that don't need this are free to
    /// no-op. Calling `set_run_mode` while enabled is allowed.
    fn set_run_mode(&mut self, mode: RunMode) -> Result<()>;

    /// Closed-loop position control. `pos_rad` is in the output frame,
    /// relative to the last `set_zero()` anchor. `max_speed_rad_s` caps
    /// motion speed.
    ///
    /// On drivers where position and speed limit are separate parameter
    /// writes (e.g. Robstride's `LocRef` + `LimitSpd`), implementations
    /// should cache `max_speed_rad_s` and only push when it changes — the
    /// caller is allowed to repeat the same value every tick.
    fn set_position(&mut self, pos_rad: f32, max_speed_rad_s: f32) -> Result<MotorFeedback>;

    /// Closed-loop velocity control.
    fn set_velocity(&mut self, vel_rad_s: f32) -> Result<MotorFeedback>;

    /// Closed-loop torque control. Units are output-frame N·m.
    fn set_torque(&mut self, torque_nm: f32) -> Result<MotorFeedback>;

    /// MIT-mode joint command: `tau = kp·(pos − q) + kd·(vel − dq) + tau_ff`.
    /// All gains and references are output-frame.
    fn mit_control(
        &mut self,
        pos_rad: f32,
        vel_rad_s: f32,
        kp_nm_per_rad: f32,
        kd_nm_per_rad_s: f32,
        torque_ff_nm: f32,
    ) -> Result<MotorFeedback>;

    /// Read the current state without sending a control command (where
    /// possible). On Robstride this requires a zero-MIT poll; on lkmotor
    /// V3 this is a single State2 read.
    fn measure(&mut self) -> Result<MotorFeedback>;

    /// Slow-changing status (bus voltage, fault flags, temperature).
    fn read_status(&mut self) -> Result<MotorStatus>;

    /// The driver's *tracked* current run mode, if the family models one.
    /// Returns `None` for families that have no explicit run-mode parameter
    /// (lkmotor V3 picks the controller per-command, so it returns `None`).
    ///
    /// This is the value the driver believes the firmware is in, based on
    /// the last `set_run_mode` call — not a fresh query against the wire.
    fn current_run_mode_hint(&self) -> Option<RunMode> {
        None
    }

    /// The angle, in the motor's own frame, that this driver is currently
    /// reporting as `position_rad = 0` (rad).
    ///
    /// Add it to any reported position to get the motor's own angle. That is
    /// what makes two runs comparable: `position_rad` alone is measured from a
    /// soft zero the driver placed, so identical numbers from two runs can be
    /// different physical angles. Recording this turns a run's positions back
    /// into something another run can be laid over.
    ///
    /// `Some(0.0)` means the reported frame *is* the motor's — nothing to add
    /// (RobStride, DAMIAO). `None` means the driver cannot say, which is not
    /// the same answer and must not be recorded as zero: a file that cannot
    /// distinguish "no offset" from "not asked" lies about what it contains.
    ///
    /// **How absolute the motor's own frame is, is a per-family question.** On
    /// an RMD-X4 it is the output shaft and survives a power cycle only modulo
    /// one turn — the encoder ROM offset persists but the turn count does not
    /// (measured 2026-08-08).
    fn position_zero_in_motor_frame_rad(&self) -> Option<f32> {
        None
    }

    /// The motor's **own** persistent origin, formatted so the value carries
    /// its own meaning. `None` when the family has none, or cannot be asked.
    ///
    /// Distinct from [`Self::position_zero_in_motor_frame_rad`], which is the
    /// host-side offset. This is the one stored in the motor, which a
    /// documented command moves: `robstride-cli set-zero`, `myactuator-cli zero
    /// --rom`, `damiao-cli zero --nvm`. When it moves, every position recorded
    /// before is in a frame that no longer exists — and nothing else in a saved
    /// run would show it, because the host-side offset can be unchanged while
    /// the motor's zero has shifted underneath it (measured 2026-08-08: an
    /// RS-04's `MechOffset` went 0.428 → 5.559 while the driver reported the
    /// same frame throughout).
    ///
    /// Recorded for **detection, not arithmetic** — the families store it in
    /// different units and frames, so the string is compared, not converted.
    /// Costs one transaction; called once per saved run, not per sample.
    fn motor_origin(&mut self) -> Option<String> {
        None
    }

    /// Whether the driver thinks the motor is currently enabled. Same
    /// caveat as [`Self::current_run_mode_hint`]: this is a tracked flag,
    /// not a fresh query.
    fn is_enabled_hint(&self) -> bool {
        false
    }

    /// Ask the driver to include motor current in [`Self::measure`], if it can.
    ///
    /// Off by default because on some families it is not free. RobStride's
    /// feedback frame carries no current field, so its driver spends a second
    /// transaction per sample and the achievable loop rate roughly halves —
    /// which would quietly shrink the usable band of a chirp. Turn it on for
    /// the span of a measurement that dwells anyway, then turn it off.
    ///
    /// Default does nothing, which is right for families whose feedback
    /// already carries current and for the simulator.
    fn set_report_current(&mut self, _on: bool) {}

    /// Supply the torque constant (N·m/A) used to synthesize torque from
    /// current, or `0.0` to go back to reporting what the motor reports.
    ///
    /// Only meaningful for drivers that *derive* torque this way, which they
    /// need to do when the firmware reports a constant zero torque. Requires
    /// [`Self::set_report_current`] — without a current reading there is
    /// nothing to scale.
    ///
    /// This exists as a method rather than only a connect-time option so a
    /// run can apply a constant it has just measured without reopening the
    /// bus. Default does nothing.
    fn set_torque_constant(&mut self, _kt_nm_per_a: f32) {}

    /// Ask for the finest position the driver can report, if that costs extra.
    ///
    /// Off by default because for at least one family it is a second
    /// transaction per command, which halves the achievable loop rate. The
    /// trade is the same shape as [`Self::set_report_current`]: streaming and
    /// chirps want the rate, quasi-static runs want the resolution and dwell
    /// long enough not to notice the cost.
    ///
    /// The difference it makes is not cosmetic. A MyActuator's control replies
    /// inline a 1°/LSB angle — about 0.017 rad — which is coarser than the
    /// tolerances the position-visiting runs are written against.
    ///
    /// Default does nothing, which is right for families whose ordinary
    /// feedback is already at full resolution.
    fn set_fine_position(&mut self, _on: bool) {}

    /// Probe the underlying bus for responding motors in `id_range`.
    ///
    /// Returns the list of motor IDs that responded. Implementations
    /// share the actuator's open bus (no second `open` of the
    /// transport), so this works even on RS485 ports that the OS only
    /// allows one process to hold.
    ///
    /// Default returns [`Error::Unsupported`]; drivers that can scan
    /// their bus override this.
    fn scan_bus(
        &mut self,
        id_range: RangeInclusive<u8>,
        timeout_per_id: Duration,
    ) -> Result<Vec<u8>> {
        let _ = (id_range, timeout_per_id);
        Err(Error::Unsupported("scan_bus"))
    }

    /// Probe a single motor id and report whether it responded.
    ///
    /// This is the building block used by progress-reporting scan UIs:
    /// the caller drives the loop one id at a time and renders progress
    /// between probes. Default implementation falls back to
    /// [`Self::scan_bus`] over a single-element range.
    fn probe_motor(&mut self, motor_id: u8, timeout: Duration) -> Result<bool> {
        let found = self.scan_bus(motor_id..=motor_id, timeout)?;
        Ok(found.iter().any(|&id| id == motor_id))
    }

    /// Read every setting the motor will report about itself.
    ///
    /// **Read-only by design.** There is no companion `write_parameter`, and
    /// that is deliberate rather than unfinished: a real RS-04 in this project
    /// vanished from its bus for a day when a value went to the address the
    /// manual named `motor_baud` and the firmware actually used for `CAN_ID`.
    /// A write path has to be built on per-driver whitelists of addresses
    /// somebody has confirmed on hardware, not on a generic address poke.
    ///
    /// `deep` opts into address spaces that were reverse-engineered rather
    /// than documented — RobStride's bulk parameter table, MyActuator's `0xC0`
    /// block. They hold parameters available nowhere else, and the RobStride
    /// one has twice left a motor unresponsive until power-cycled, so the
    /// caller decides.
    ///
    /// Individual reads that fail are reported as [`Parameter::unavailable`]
    /// rather than failing the whole call: one absent register should not cost
    /// you the other eighty. Default returns [`Error::Unsupported`].
    fn read_parameters(&mut self, deep: bool) -> Result<Vec<crate::Parameter>> {
        let _ = deep;
        Err(Error::Unsupported("read_parameters"))
    }
}
