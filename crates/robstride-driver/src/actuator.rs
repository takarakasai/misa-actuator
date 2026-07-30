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

use std::ops::RangeInclusive;
use std::time::Duration;

use misa_actuator::{
    Actuator, Error as MisaError, ErrorFlags, MotorFeedback as MisaFeedback, MotorStatus,
    Result as MisaResult, RunMode as MisaRunMode,
};
use robstride_protocol::{MotorFeedback as RsFeedback, MotorStatusBits, ParamIndex, RunMode};

use crate::bus::RobstrideBus;
use crate::driver::Motor;
use crate::scan::scan_bus_on;

/// Default `LimitSpd` (rad/s) written before enabling in Position mode so
/// that a freshly-enabled motor doesn't run away at maximum speed.
const POSITION_DEFAULT_LIMIT_SPD: f32 = 5.0;

/// Fetch current only when the caller opted in, since it costs an extra bus
/// round-trip. `NaN` otherwise — RobStride's feedback frame has no current
/// field, so reporting a zero would be a fabrication.
fn optional_current<B: RobstrideBus>(motor: &mut Motor<B>) -> f32 {
    if motor.reports_current() {
        Motor::read_current(motor).unwrap_or(f32::NAN)
    } else {
        f32::NAN
    }
}

/// Torque to report, substituting `current * Kt` when the motor's own reading is
/// unusable.
///
/// Substitution is deliberately narrow: only when a torque constant was
/// supplied, a current reading exists, and the reported torque is either
/// non-finite or **exactly** zero. Firmware whose `MeasuredTorque` is broken
/// returns a hard 0 (see `Motor::set_torque_constant`), while a genuine reading
/// essentially never lands on 0.0 with current flowing — and when it does, the
/// substituted value is also ~0, so the swap is harmless. A real non-zero
/// reading is always preferred over the derived one.
fn effective_torque(reported_nm: f32, current_a: f32, kt_nm_per_a: Option<f32>) -> f32 {
    match kt_nm_per_a {
        Some(kt) if current_a.is_finite() && (!reported_nm.is_finite() || reported_nm == 0.0) => {
            current_a * kt
        }
        _ => reported_nm,
    }
}

fn rs_to_misa_feedback(fb: RsFeedback, current_a: f32, kt: Option<f32>) -> MisaFeedback {
    MisaFeedback {
        position_rad: fb.position,
        velocity_rad_per_s: fb.velocity,
        torque_nm: effective_torque(fb.torque, current_a, kt),
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

        Ok(rs_to_misa_feedback(fb, f32::NAN, None))
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
        let fb = Motor::measure_safe(self)?;
        let current = optional_current(self);
        let kt = self.torque_constant();
        Ok(rs_to_misa_feedback(fb, current, kt))
    }

    fn set_velocity(&mut self, vel_rad_s: f32) -> MisaResult<MisaFeedback> {
        require_mode(self, RunMode::Velocity, "set_velocity")?;
        Motor::set_velocity(self, vel_rad_s)?;
        let fb = Motor::measure_safe(self)?;
        let current = optional_current(self);
        let kt = self.torque_constant();
        Ok(rs_to_misa_feedback(fb, current, kt))
    }

    fn set_torque(&mut self, torque_nm: f32) -> MisaResult<MisaFeedback> {
        require_mode(self, RunMode::Torque, "set_torque")?;
        // Robstride's torque-mode parameter is `IqRef` — a quadrature *current*,
        // in amps. So the trait's N·m must be divided by the torque constant;
        // passing it through raw silently commands amps and understates the
        // torque by a factor of Kt (1.5093 on an RS04, i.e. a third of what was
        // asked for). Verified on hardware: commanded IqRef tracks measured
        // current to within 0.0001 A, so this conversion is the whole story.
        //
        // Without a Kt there is nothing to convert with, so the old raw
        // behaviour stands — see `Motor::set_torque_constant`. For an exact N·m
        // command regardless, use MIT mode with `torque_ff`.
        let iq = match self.torque_constant() {
            Some(kt) => torque_nm / kt,
            None => torque_nm,
        };
        Motor::set_torque(self, iq)?;
        let fb = Motor::measure_safe(self)?;
        let current = optional_current(self);
        let kt = self.torque_constant();
        Ok(rs_to_misa_feedback(fb, current, kt))
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
        // status frame — no extra read needed and no mode disturbance. Current
        // is still absent from that frame, so it needs the opt-in read like
        // every other path; without this the leashed quasi-static runs, which
        // drive MIT, report no current at all.
        let fb =
            Motor::mit_control(self, pos_rad, vel_rad_s, kp_nm_per_rad, kd_nm_per_rad_s, torque_ff_nm)?;
        let current = optional_current(self);
        let kt = self.torque_constant();
        Ok(rs_to_misa_feedback(fb, current, kt))
    }

    fn measure(&mut self) -> MisaResult<MisaFeedback> {
        let fb = Motor::measure_safe(self)?;
        let current = optional_current(self);
        let kt = self.torque_constant();
        Ok(rs_to_misa_feedback(fb, current, kt))
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

    fn scan_bus(
        &mut self,
        id_range: RangeInclusive<u8>,
        timeout_per_id: Duration,
    ) -> MisaResult<Vec<u8>> {
        let host_id = self.host_id();
        // scan_bus_on borrows the bus mutably. Since we're inside &mut self
        // we can hand it our own bus.
        let results =
            scan_bus_on(self.bus(), host_id, id_range, timeout_per_id, None)?;
        Ok(results.into_iter().map(|r| r.motor_id).collect())
    }

    fn probe_motor(&mut self, motor_id: u8, timeout: Duration) -> MisaResult<bool> {
        let host_id = self.host_id();
        let results =
            scan_bus_on(self.bus(), host_id, motor_id..=motor_id, timeout, None)?;
        Ok(results.iter().any(|r| r.motor_id == motor_id))
    }
}

#[cfg(test)]
mod tests {
    use super::effective_torque;

    /// A real non-zero reading always wins: passing a Kt on healthy firmware
    /// must not change anything.
    #[test]
    fn a_real_torque_reading_is_never_overridden() {
        assert_eq!(effective_torque(0.86, 0.57, Some(1.5093)), 0.86);
        assert_eq!(effective_torque(-0.31, -0.21, Some(1.5093)), -0.31);
    }

    /// The broken-firmware case: a hard 0 with current flowing is replaced by
    /// `current * Kt`. Numbers from an RS04 Position-mode sample.
    #[test]
    fn a_hard_zero_with_current_is_replaced() {
        let t = effective_torque(0.0, 0.446717, Some(1.5093));
        assert!((t - 0.446717 * 1.5093).abs() < 1e-6, "got {t}");
        // Sign is carried through, which is what makes the friction hysteresis
        // visible at all.
        assert!(effective_torque(0.0, -0.085677, Some(1.5093)) < 0.0);
    }

    /// Without a Kt there is nothing to scale by, so the reading stands as-is —
    /// including a zero.
    #[test]
    fn without_a_torque_constant_nothing_is_substituted() {
        assert_eq!(effective_torque(0.0, 0.45, None), 0.0);
        assert!(effective_torque(f32::NAN, 0.45, None).is_nan());
    }

    /// Without a current reading there is nothing to derive from. `--kt` implies
    /// `--report-current` for this reason, but the guard belongs here too.
    #[test]
    fn without_a_current_reading_nothing_is_substituted() {
        assert_eq!(effective_torque(0.0, f32::NAN, Some(1.5093)), 0.0);
    }

    /// A non-finite reading is replaced when it can be: NaN torque is worse than
    /// a derived one.
    #[test]
    fn a_non_finite_reading_is_replaced_when_possible() {
        let t = effective_torque(f32::NAN, 0.2, Some(2.0));
        assert!((t - 0.4).abs() < 1e-6, "got {t}");
    }

    /// At genuine rest both sources agree on ~0, so the substitution is harmless
    /// in the ambiguous case that motivates the "exactly 0.0" trigger.
    #[test]
    fn at_rest_the_substitution_is_harmless() {
        let t = effective_torque(0.0, 0.014557, Some(1.5093));
        assert!(t.abs() < 0.03, "should stay near zero, got {t}");
    }
}
