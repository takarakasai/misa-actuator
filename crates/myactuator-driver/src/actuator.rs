//! `misa_actuator::Actuator` implementation for [`MyActuatorMotor<B>`].
//!
//! ## Mapping notes
//!
//! - **No run-mode parameter** — like lkmotor V3, each command picks its own
//!   controller, so `set_run_mode` is a no-op and the mode hint is `None`.
//! - **`enable`** — V3 has no servo-on command; motion commands activate the
//!   controller directly. `enable` therefore anchors zero and engages the
//!   position controller at the current location (holding torque), matching
//!   the lkmotor driver's behaviour.
//! - **`disable`** — motor stop (`0x81`), which halts output but keeps the
//!   closed loop armed. Use [`MyActuatorMotor::shutdown`] for a full output-off.
//! - **`mit_control`** — native motion mode (`0x400 + ID`), real on-motor PD.
//!   Requires motion-mode-capable firmware (RMD-X V3); a timeout usually means
//!   the firmware lacks the mode. **The returned feedback is unverified and
//!   should not be trusted** — see [`myactuator_protocol::motion`] for the
//!   real-hardware evidence (constant velocity with unchanging position is
//!   physically impossible, so the reply decode is known wrong). The command
//!   itself does drive the motor; only the parsed feedback is suspect.
//! - **`set_torque`** — `0xA1` **latches** on the firmware: the motor keeps
//!   applying the torque until a new command arrives, so re-issue it from a
//!   control loop rather than calling it once and walking away.

use std::ops::RangeInclusive;
use std::time::Duration;

use misa_actuator::{
    Actuator, ErrorFlags as MisaErrorFlags, MotorFeedback as MisaFeedback,
    MotorStatus as MisaStatus, Result as MisaResult, RunMode,
};
use myactuator_protocol::ErrorState;

use crate::bus::MyActuatorBus;
use crate::driver::{MotorFeedback, MotorStatus, MyActuatorMotor};
use crate::error::Error;
use crate::scan::scan_bus_on;

/// Default max-speed (rad/s) used when `enable` engages the position hold.
const DEFAULT_HOLD_MAX_SPEED: f32 = 1.0;

fn to_misa_feedback(fb: MotorFeedback) -> MisaFeedback {
    MisaFeedback {
        position_rad: fb.position_rad,
        velocity_rad_per_s: fb.velocity_rad_per_s,
        torque_nm: fb.torque_nm,
        current_a: fb.current_a,
        temperature_c: fb.temperature_c as f32,
    }
}

fn to_misa_status(s: MotorStatus) -> MisaStatus {
    MisaStatus {
        voltage_v: s.voltage_v,
        temperature_c: s.temperature_c as f32,
        error: to_misa_error(s.error),
    }
}

fn to_misa_error(e: ErrorState) -> MisaErrorFlags {
    let mut bits = 0u32;
    if e.stall()                     { bits |= MisaErrorFlags::STALL; }
    if e.under_voltage()             { bits |= MisaErrorFlags::UNDER_VOLTAGE; }
    if e.over_voltage()              { bits |= MisaErrorFlags::OVER_VOLTAGE; }
    if e.over_current()              { bits |= MisaErrorFlags::OVER_CURRENT; }
    if e.motor_over_temperature()    { bits |= MisaErrorFlags::MOTOR_OVERHEAT; }
    if e.encoder_calibration_error() { bits |= MisaErrorFlags::ENCODER_FAULT; }
    MisaErrorFlags::new(bits, e.raw() as u32)
}

impl<B: MyActuatorBus> Actuator for MyActuatorMotor<B> {
    fn motor_id(&self) -> u8 {
        MyActuatorMotor::motor_id(self)
    }

    fn enable(&mut self) -> MisaResult<MisaFeedback> {
        // Anchor zero at the current physical position, then engage the
        // position controller there — V3 has no bare "servo on", so holding
        // the current position is what gives the expected enable feel.
        self.rezero()?;
        let fb = MyActuatorMotor::set_position(self, 0.0, DEFAULT_HOLD_MAX_SPEED)?;
        self.enabled = true;
        Ok(to_misa_feedback(fb))
    }

    fn disable(&mut self) -> MisaResult<()> {
        self.stop()?;
        Ok(())
    }

    fn set_zero(&mut self) -> MisaResult<()> {
        self.rezero()?;
        Ok(())
    }

    // This family is why the trait method exists: its control replies inline a
    // 1°/LSB angle, and the fine one costs a second transaction.
    fn set_fine_position(&mut self, on: bool) {
        MyActuatorMotor::set_fine_position(self, on);
    }

    fn set_run_mode(&mut self, _mode: RunMode) -> MisaResult<()> {
        // Each V3 command picks its own controller; nothing to configure.
        Ok(())
    }

    fn set_position(&mut self, pos_rad: f32, max_speed_rad_s: f32) -> MisaResult<MisaFeedback> {
        match MyActuatorMotor::set_position(self, pos_rad, max_speed_rad_s) {
            Ok(fb) => Ok(to_misa_feedback(fb)),
            // Auto-anchor on first use (same policy as the lkmotor driver's
            // `PositionAnchor::OnFirstUse`) so the TUI can command positions
            // without an explicit set_zero.
            Err(Error::PositionNotAnchored { .. }) => {
                self.rezero()?;
                Ok(to_misa_feedback(MyActuatorMotor::set_position(
                    self,
                    pos_rad,
                    max_speed_rad_s,
                )?))
            }
            Err(e) => Err(e.into()),
        }
    }

    fn set_velocity(&mut self, vel_rad_s: f32) -> MisaResult<MisaFeedback> {
        Ok(to_misa_feedback(MyActuatorMotor::set_velocity(
            self, vel_rad_s,
        )?))
    }

    fn set_torque(&mut self, torque_nm: f32) -> MisaResult<MisaFeedback> {
        Ok(to_misa_feedback(MyActuatorMotor::set_torque(
            self, torque_nm,
        )?))
    }

    fn mit_control(
        &mut self,
        pos_rad: f32,
        vel_rad_s: f32,
        kp_nm_per_rad: f32,
        kd_nm_per_rad_s: f32,
        torque_ff_nm: f32,
    ) -> MisaResult<MisaFeedback> {
        let kt = self.config().torque_constant_nm_per_a;
        let fb = self.motion_control(pos_rad, vel_rad_s, kp_nm_per_rad, kd_nm_per_rad_s, torque_ff_nm)?;
        Ok(MisaFeedback {
            position_rad: fb.position_rad,
            velocity_rad_per_s: fb.velocity_rad_per_s,
            torque_nm: fb.torque_nm,
            current_a: if kt > 0.0 { fb.torque_nm / kt } else { f32::NAN },
            // The motion-mode reply carries no temperature.
            temperature_c: f32::NAN,
        })
    }

    fn measure(&mut self) -> MisaResult<MisaFeedback> {
        Ok(to_misa_feedback(MyActuatorMotor::measure(self)?))
    }

    fn read_status(&mut self) -> MisaResult<MisaStatus> {
        Ok(to_misa_status(MyActuatorMotor::read_status(self)?))
    }

    fn current_run_mode_hint(&self) -> Option<RunMode> {
        None
    }

    fn is_enabled_hint(&self) -> bool {
        self.enabled
    }

    fn scan_bus(
        &mut self,
        id_range: RangeInclusive<u8>,
        timeout_per_id: Duration,
    ) -> MisaResult<Vec<u8>> {
        Ok(scan_bus_on(self.bus(), id_range, timeout_per_id, None)?)
    }

    fn probe_motor(&mut self, motor_id: u8, timeout: Duration) -> MisaResult<bool> {
        Ok(crate::scan::probe_one(self.bus(), motor_id, timeout)?)
    }

    fn read_parameters(&mut self, deep: bool) -> MisaResult<Vec<misa_actuator::Parameter>> {
        Ok(MyActuatorMotor::read_parameters(self, deep))
    }
}

impl<B: MyActuatorBus> Drop for MyActuatorMotor<B> {
    fn drop(&mut self) {
        // Leave the motor stopped (not shut down) so it stays in a safe,
        // recoverable state. Best-effort — the bus may already be gone.
        let _ = self.stop();
    }
}
