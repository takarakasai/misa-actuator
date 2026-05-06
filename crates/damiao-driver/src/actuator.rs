//! `misa_actuator::Actuator` implementation for [`DamiaoMotor<B>`].
//!
//! ## Mode mapping
//!
//! The unified [`misa_actuator::RunMode`] maps onto DAMIAO's `CTRL_MODE`
//! register as follows:
//!
//! | misa `RunMode` | DAMIAO `ControlMode` | command channel |
//! |----------------|----------------------|-----------------|
//! | `Mit`          | `Mit`                | `CAN_ID`        |
//! | `Position`     | `PosVel`             | `0x100 + CAN_ID`|
//! | `Velocity`     | `Vel`                | `0x200 + CAN_ID`|
//! | `Torque`       | `Mit` (kp=kd=0)      | `CAN_ID`        |
//!
//! DAMIAO has no dedicated torque mode, so `set_torque` runs in MIT mode with
//! zero gains and the torque as feed-forward.
//!
//! ## Mode-mismatch errors
//!
//! `set_position` / `set_velocity` / `set_torque` / `mit_control` send on a
//! mode-specific CAN id; issued in the wrong mode they do nothing on the wire.
//! We return an explicit error instead of silently no-op'ing — call
//! `set_run_mode` first.

use std::ops::RangeInclusive;
use std::time::Duration;

use misa_actuator::{
    Actuator, Error as MisaError, ErrorFlags, MotorFeedback as MisaFeedback, MotorStatus,
    Result as MisaResult, RunMode as MisaRunMode,
};
use damiao_protocol::{ControlMode, ErrorCode, Feedback};

use crate::bus::DamiaoBus;
use crate::driver::DamiaoMotor;
use crate::scan::scan_bus_on;

fn dm_to_misa_feedback(fb: Feedback) -> MisaFeedback {
    MisaFeedback {
        position_rad: fb.position,
        velocity_rad_per_s: fb.velocity,
        torque_nm: fb.torque,
        // DAMIAO does not report motor-frame iq current in its feedback frame.
        current_a: f32::NAN,
        // Rotor/coil temperature is the closest to "motor temperature".
        temperature_c: fb.t_rotor,
    }
}

fn dm_err_to_misa(err: ErrorCode) -> ErrorFlags {
    let mut bits = 0u32;
    match err {
        ErrorCode::OverVoltage => bits |= ErrorFlags::OVER_VOLTAGE,
        ErrorCode::UnderVoltage => bits |= ErrorFlags::UNDER_VOLTAGE,
        ErrorCode::OverCurrent => bits |= ErrorFlags::OVER_CURRENT,
        ErrorCode::MosOverTemp => bits |= ErrorFlags::DRIVER_OVERHEAT,
        ErrorCode::CoilOverTemp => bits |= ErrorFlags::MOTOR_OVERHEAT,
        ErrorCode::CommLost => bits |= ErrorFlags::SIGNAL_TIMEOUT,
        ErrorCode::Overload => bits |= ErrorFlags::STALL,
        ErrorCode::Disabled | ErrorCode::Enabled | ErrorCode::Unknown(_) => {}
    }
    ErrorFlags::new(bits, err.raw() as u32)
}

fn map_misa_run_mode(m: MisaRunMode) -> ControlMode {
    match m {
        MisaRunMode::Mit => ControlMode::Mit,
        MisaRunMode::Position => ControlMode::PosVel,
        MisaRunMode::Velocity => ControlMode::Vel,
        // No dedicated torque mode — torque is commanded via MIT feed-forward.
        MisaRunMode::Torque => ControlMode::Mit,
    }
}

fn map_dm_run_mode(m: ControlMode) -> MisaRunMode {
    match m {
        ControlMode::Mit => MisaRunMode::Mit,
        ControlMode::PosVel => MisaRunMode::Position,
        ControlMode::Vel => MisaRunMode::Velocity,
        ControlMode::ForcePos => MisaRunMode::Position,
    }
}

fn require_mode<B: DamiaoBus>(
    motor: &DamiaoMotor<B>,
    expected: ControlMode,
    op: &str,
) -> MisaResult<()> {
    if motor.mode() != expected {
        return Err(MisaError::Other(format!(
            "{op} requires {expected:?} control mode (currently {:?}); call set_run_mode first",
            motor.mode()
        )));
    }
    Ok(())
}

impl<B: DamiaoBus> Actuator for DamiaoMotor<B> {
    fn motor_id(&self) -> u8 {
        DamiaoMotor::can_id(self)
    }

    fn enable(&mut self) -> MisaResult<MisaFeedback> {
        let fb = DamiaoMotor::enable(self)?;
        Ok(fb.map(dm_to_misa_feedback).unwrap_or(MisaFeedback::zero()))
    }

    fn disable(&mut self) -> MisaResult<()> {
        DamiaoMotor::disable(self)?;
        Ok(())
    }

    fn set_zero(&mut self) -> MisaResult<()> {
        DamiaoMotor::set_zero(self)?;
        Ok(())
    }

    fn set_run_mode(&mut self, mode: MisaRunMode) -> MisaResult<()> {
        // Switch the mode register with the motor disabled, then restore the
        // previous enable state — changing CTRL_MODE while running is unsafe.
        let was_enabled = self.is_enabled();
        if was_enabled {
            DamiaoMotor::disable(self)?;
        }
        DamiaoMotor::switch_mode(self, map_misa_run_mode(mode))?;
        if was_enabled {
            DamiaoMotor::enable(self)?;
        }
        Ok(())
    }

    fn set_position(&mut self, pos_rad: f32, max_speed_rad_s: f32) -> MisaResult<MisaFeedback> {
        require_mode(self, ControlMode::PosVel, "set_position")?;
        Ok(dm_to_misa_feedback(DamiaoMotor::set_pos_vel(
            self,
            pos_rad,
            max_speed_rad_s,
        )?))
    }

    fn set_velocity(&mut self, vel_rad_s: f32) -> MisaResult<MisaFeedback> {
        require_mode(self, ControlMode::Vel, "set_velocity")?;
        Ok(dm_to_misa_feedback(DamiaoMotor::set_vel(self, vel_rad_s)?))
    }

    fn set_torque(&mut self, torque_nm: f32) -> MisaResult<MisaFeedback> {
        // DAMIAO torque control = MIT with zero gains and torque feed-forward.
        require_mode(self, ControlMode::Mit, "set_torque")?;
        Ok(dm_to_misa_feedback(DamiaoMotor::mit_control(
            self, 0.0, 0.0, 0.0, 0.0, torque_nm,
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
        require_mode(self, ControlMode::Mit, "mit_control")?;
        Ok(dm_to_misa_feedback(DamiaoMotor::mit_control(
            self,
            pos_rad,
            vel_rad_s,
            kp_nm_per_rad,
            kd_nm_per_rad_s,
            torque_ff_nm,
        )?))
    }

    fn measure(&mut self) -> MisaResult<MisaFeedback> {
        Ok(dm_to_misa_feedback(DamiaoMotor::measure(self)?))
    }

    fn read_status(&mut self) -> MisaResult<MotorStatus> {
        let fb = DamiaoMotor::measure(self)?;
        Ok(MotorStatus {
            // DAMIAO feedback frames carry no DC bus voltage.
            voltage_v: f32::NAN,
            temperature_c: fb.t_rotor,
            error: dm_err_to_misa(fb.err),
        })
    }

    fn current_run_mode_hint(&self) -> Option<MisaRunMode> {
        Some(map_dm_run_mode(self.mode()))
    }

    fn is_enabled_hint(&self) -> bool {
        self.is_enabled()
    }

    fn scan_bus(
        &mut self,
        id_range: RangeInclusive<u8>,
        timeout_per_id: Duration,
    ) -> MisaResult<Vec<u8>> {
        Ok(scan_bus_on(self.bus(), id_range, timeout_per_id, None)?)
    }

    fn probe_motor(&mut self, motor_id: u8, timeout: Duration) -> MisaResult<bool> {
        let found = scan_bus_on(self.bus(), motor_id..=motor_id, timeout, None)?;
        Ok(found.contains(&motor_id))
    }
}
