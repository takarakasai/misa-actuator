//! `misa_actuator::Actuator` implementation for [`crate::Motor<B>`].
//!
//! ## Run-mode-safe feedback
//!
//! The Robstride status frame (`OperationStatus`) is only emitted in
//! response to a MIT-mode `OperationControl` frame. Asking for status while
//! the motor is in Position/Velocity/Torque mode would require sending a
//! zero-MIT control frame — which silently switches the firmware *into*
//! MIT mode (with zero gains, i.e. no holding torque) for one cycle. The
//! visible symptom is that the motor appears to lose servo and goes limp.
//!
//! To avoid that, every method that needs feedback while the motor might
//! be in a non-MIT mode dispatches through [`Motor::measure_safe`], which
//! falls back to per-parameter reads (`MechPos`, `MechVel`, `MeasuredTorque`).
//!
//! ## Per-mode safe enable
//!
//! The Robstride firmware needs the active mode's reference written to a
//! sane value *before* (and, for Position, also right after) `Enable`,
//! otherwise a freshly-enabled motor in Position mode tracks a stale
//! `LocRef` (jumps / faults / no holding torque). [`Self::enable`] handles
//! this automatically:
//! - **Velocity**: write `SpdRef = 0` before enable
//! - **Torque**:   write `IqRef  = 0` before enable
//! - **Position**: write `LimitSpd = 5.0` before enable, then read `MechPos`
//!   and write `LocRef = MechPos` after enable so the motor holds in place
//! - **MIT**: nothing
//!
//! ## Mode-mismatch errors
//!
//! `set_position` / `set_velocity` / `set_torque` write parameter-mode
//! references and only have an effect in the matching run mode. Calling
//! them in the wrong mode silently does nothing on the wire — confusing.
//! We return an explicit error instead.

use misa_actuator::{
    Actuator, Error as MisaError, ErrorFlags, MotorFeedback as MisaFeedback, MotorStatus,
    Result as MisaResult, RunMode as MisaRunMode,
};
use robstride_protocol::{MotorFeedback as RsFeedback, MotorStatusBits, ParamIndex, RunMode};

use crate::bus::RobstrideBus;
use crate::driver::Motor;

/// Default `LimitSpd` (rad/s) written before enabling in Position mode so
/// that a freshly-enabled motor doesn't run away at maximum speed.
const POSITION_DEFAULT_LIMIT_SPD: f32 = 5.0;

fn rs_to_misa_feedback(fb: RsFeedback, current_a: f32) -> MisaFeedback {
    MisaFeedback {
        position_rad: fb.position,
        velocity_rad_per_s: fb.velocity,
        torque_nm: fb.torque,
        current_a,
        temperature_c: fb.temperature,
    }
}

fn rs_status_to_misa_error(s: &MotorStatusBits, raw_extra: u16) -> ErrorFlags {
    let mut bits = 0u32;
    if s.undervoltage           { bits |= ErrorFlags::UNDER_VOLTAGE; }
    if s.overcurrent            { bits |= ErrorFlags::OVER_CURRENT; }
    if s.overtemperature        { bits |= ErrorFlags::MOTOR_OVERHEAT; }
    if s.stall                  { bits |= ErrorFlags::STALL; }
    if s.magnetic_encoder_fault { bits |= ErrorFlags::ENCODER_FAULT; }
    if s.uncalibrated           { bits |= ErrorFlags::UNCALIBRATED; }
    ErrorFlags::new(bits, raw_extra as u32)
}

fn map_misa_run_mode(m: MisaRunMode) -> RunMode {
    match m {
        MisaRunMode::Mit      => RunMode::Mit,
        MisaRunMode::Position => RunMode::Position,
        MisaRunMode::Velocity => RunMode::Velocity,
        MisaRunMode::Torque   => RunMode::Torque,
    }
}

fn map_rs_run_mode(m: RunMode) -> MisaRunMode {
    match m {
        RunMode::Mit      => MisaRunMode::Mit,
        RunMode::Position => MisaRunMode::Position,
        RunMode::Velocity => MisaRunMode::Velocity,
        RunMode::Torque   => MisaRunMode::Torque,
    }
}

fn require_mode<B: RobstrideBus>(motor: &Motor<B>, expected: RunMode, op: &str) -> MisaResult<()> {
    if motor.current_run_mode() != expected {
        return Err(MisaError::Other(format!(
            "{} requires {:?} run mode (currently {:?}); call set_run_mode({:?}) first",
            op, expected, motor.current_run_mode(), expected
        )));
    }
    Ok(())
}

impl<B: RobstrideBus> Actuator for Motor<B> {
    fn motor_id(&self) -> u8 {
        Motor::motor_id(self)
    }

    fn enable(&mut self) -> MisaResult<MisaFeedback> {
        // Pre-enable: write a safe per-mode reference so that as soon as
        // the firmware turns on, it has something sensible to track.
        match self.current_run_mode() {
            RunMode::Mit => { /* no parameter reference — MIT cmds drive it */ }
            RunMode::Velocity => {
                self.write_param_f32(ParamIndex::SpdRef, 0.0)?;
            }
            RunMode::Torque => {
                self.write_param_f32(ParamIndex::IqRef, 0.0)?;
            }
            RunMode::Position => {
                self.set_position_speed_limit(POSITION_DEFAULT_LIMIT_SPD)?;
                // Note: LocRef writes that arrive while disabled are ignored
                // by firmware. We re-write LocRef *after* enable below.
            }
        }

        let fb = Motor::enable(self)?;

        // Post-enable: in Position mode, write LocRef = current MechPos so
        // the motor holds in place instead of slewing toward an old/zero
        // LocRef value. (For other modes the pre-enable write is enough.)
        if self.current_run_mode() == RunMode::Position {
            if let Ok(pos) = self.read_position() {
                let _ = Motor::set_position_with_speed(self, pos, POSITION_DEFAULT_LIMIT_SPD);
            }
        }

        Ok(rs_to_misa_feedback(fb, f32::NAN))
    }

    fn disable(&mut self) -> MisaResult<()> {
        Motor::disable(self)?;
        Ok(())
    }

    fn set_zero(&mut self) -> MisaResult<()> {
        Motor::set_zero(self)?;
        Ok(())
    }

    fn set_run_mode(&mut self, mode: MisaRunMode) -> MisaResult<()> {
        // Robstride firmware silently ignores `RunMode` writes while the
        // motor is enabled — the CLI works around this by always issuing
        // `disable → set_run_mode → enable`. We do the same here, AND we
        // route the re-enable through our own `Actuator::enable` so the
        // per-mode safe references get written.
        let was_enabled = self.is_enabled();
        if was_enabled {
            let _ = Motor::disable(self);
        }
        Motor::set_run_mode(self, map_misa_run_mode(mode))?;
        if was_enabled {
            self.enable()?;
        }
        Ok(())
    }

    fn set_position(&mut self, pos_rad: f32, max_speed_rad_s: f32) -> MisaResult<MisaFeedback> {
        require_mode(self, RunMode::Position, "set_position")?;
        Motor::set_position_with_speed(self, pos_rad, max_speed_rad_s)?;
        Ok(rs_to_misa_feedback(Motor::measure_safe(self)?, f32::NAN))
    }

    fn set_velocity(&mut self, vel_rad_s: f32) -> MisaResult<MisaFeedback> {
        require_mode(self, RunMode::Velocity, "set_velocity")?;
        Motor::set_velocity(self, vel_rad_s)?;
        Ok(rs_to_misa_feedback(Motor::measure_safe(self)?, f32::NAN))
    }

    fn set_torque(&mut self, torque_nm: f32) -> MisaResult<MisaFeedback> {
        require_mode(self, RunMode::Torque, "set_torque")?;
        // Robstride's torque-mode parameter is `IqRef` — quadrature current.
        // Without a Kt-aware mapping this is approximate; for accurate Nm
        // command use `set_run_mode(Mit)` + `mit_control` with `torque_ff`.
        Motor::set_torque(self, torque_nm)?;
        Ok(rs_to_misa_feedback(Motor::measure_safe(self)?, f32::NAN))
    }

    fn mit_control(
        &mut self,
        pos_rad: f32,
        vel_rad_s: f32,
        kp_nm_per_rad: f32,
        kd_nm_per_rad_s: f32,
        torque_ff_nm: f32,
    ) -> MisaResult<MisaFeedback> {
        require_mode(self, RunMode::Mit, "mit_control")?;
        // mit_control is itself a MIT frame, so the firmware reply IS the
        // status frame — no extra read needed and no mode disturbance.
        Ok(rs_to_misa_feedback(
            Motor::mit_control(self, pos_rad, vel_rad_s, kp_nm_per_rad, kd_nm_per_rad_s, torque_ff_nm)?,
            f32::NAN,
        ))
    }

    fn measure(&mut self) -> MisaResult<MisaFeedback> {
        Ok(rs_to_misa_feedback(Motor::measure_safe(self)?, f32::NAN))
    }

    fn read_status(&mut self) -> MisaResult<MotorStatus> {
        // Use measure_safe so we don't tear down the active control mode
        // just because the user asked for a status snapshot.
        let fb = Motor::measure_safe(self)?;
        let voltage_v = Motor::read_vbus(self).unwrap_or(f32::NAN);
        let raw_extra = ((fb.status.mode as u16) << 14)
            | ((fb.status.uncalibrated as u16) << 13)
            | ((fb.status.stall as u16) << 12)
            | ((fb.status.magnetic_encoder_fault as u16) << 11)
            | ((fb.status.overtemperature as u16) << 10)
            | ((fb.status.overcurrent as u16) << 9)
            | ((fb.status.undervoltage as u16) << 8)
            | (fb.status.device_id as u16);
        Ok(MotorStatus {
            voltage_v,
            temperature_c: fb.temperature,
            error: rs_status_to_misa_error(&fb.status, raw_extra),
        })
    }

    fn current_run_mode_hint(&self) -> Option<MisaRunMode> {
        Some(map_rs_run_mode(self.current_run_mode()))
    }

    fn is_enabled_hint(&self) -> bool {
        self.is_enabled()
    }
}
