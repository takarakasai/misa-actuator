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

    /// Whether the driver thinks the motor is currently enabled. Same
    /// caveat as [`Self::current_run_mode_hint`]: this is a tracked flag,
    /// not a fresh query.
    fn is_enabled_hint(&self) -> bool {
        false
    }

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
}
