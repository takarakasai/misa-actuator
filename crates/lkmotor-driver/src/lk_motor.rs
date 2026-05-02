//! [`LkMotor`] — owns a [`LkBus`] and a [`Motor`], implements
//! [`misa_actuator::Actuator`].
//!
//! This is the high-level type that downstream applications and the
//! `misa-actuator-tui` debug TUI consume. It hides the bus type behind a
//! generic parameter so the same `LkMotor::set_position`, `enable`, etc.
//! work over RS485, CAN, or any future transport.
//!
//! ## lkmotor V3 quirks vs Robstride
//!
//! - **No "Enable" command that turns on the servo** — `MotorRun` (`0x88`)
//!   only resumes from `MotorStop`. To get holding torque the motor must
//!   be told *what* to control. [`Self::enable`] therefore: sends
//!   `MotorRun`, reads the absolute angle to anchor zero, and issues a
//!   position-control command at the current location to engage the
//!   position controller (= holding torque).
//! - **No native MIT mode** — [`Motor::mit_control`] is a host-side PD
//!   emulation that sends a `torque_control` (`0xA1`) frame. `0xA1`
//!   *latches* on the firmware: the motor keeps applying that torque
//!   until a new command arrives, so a one-shot host-side mit_control
//!   would spin the motor forever. For the polymorphic `Actuator` API,
//!   [`Self::mit_control`] therefore translates to a safe one-shot
//!   `position_control_with_speed` and **ignores `kp` / `kd` /
//!   `torque_ff`**. For real PD-loop MIT control, use
//!   [`Motor::mit_control`] directly inside your own control loop.
//! - **No run-mode parameter** — each motion command picks its own
//!   controller, so `Actuator::set_run_mode` is a no-op.

use std::time::Duration;

use misa_actuator::{
    Actuator, ErrorFlags as MisaErrorFlags, MotorFeedback as MisaFeedback,
    MotorStatus as MisaStatus, Result as MisaResult, RunMode,
};

use crate::bus::LkBus;
use crate::driver::Rs485Driver;
use crate::error::Result as LkResult;
use crate::motor::{ErrorFlags as LkErrorFlags, Motor, MotorConfig, MotorFeedback as LkFeedback,
    MotorStatus as LkStatus};
use crate::motor_id::MotorId;

/// Default max-speed (rad/s, output frame) used by `Actuator::enable` when
/// engaging the position-hold controller, and as a fallback by
/// `Actuator::mit_control` when `vel_rad_s == 0`.
const DEFAULT_HOLD_MAX_SPEED: f32 = 1.0;

/// Position-anchoring policy for the first `set_position` call.
///
/// `set_position` requires an absolute zero anchor (lkmotor V3 has no
/// hardware "set zero" command — the firmware always reports motor-frame
/// absolute angle). [`PositionAnchor::OnFirstUse`] auto-rezeros on the
/// first call; [`PositionAnchor::Manual`] requires the caller to invoke
/// [`Actuator::set_zero`] explicitly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionAnchor {
    /// Auto-rezero on the first `set_position`. Convenient for the TUI.
    OnFirstUse,
    /// Caller must explicitly invoke `set_zero()` before `set_position`.
    Manual,
}

/// `Actuator`-implementing wrapper around an [`LkBus`].
///
/// Owns the bus and a [`Motor`] (encoder turn tracker, gear ratio,
/// torque-constant, position anchor). The bus is generic so the same type
/// can be backed by RS485 (`LkMotor<Rs485Driver>`) or future CAN
/// transports.
pub struct LkMotor<B: LkBus> {
    bus: B,
    motor: Motor,
    anchor: PositionAnchor,
    /// Tracked enable state (set by `Actuator::enable` / `Actuator::disable`).
    /// Surfaced to the TUI through [`Actuator::is_enabled_hint`].
    enabled: bool,
}

impl<B: LkBus> LkMotor<B> {
    /// Build a new `LkMotor` over an already-open bus.
    pub fn new(bus: B, id: MotorId, config: MotorConfig) -> Self {
        Self {
            bus,
            motor: Motor::new(id, config),
            anchor: PositionAnchor::OnFirstUse,
            enabled: false,
        }
    }

    /// Override the position-anchoring policy. Default is [`PositionAnchor::OnFirstUse`].
    pub fn with_position_anchor(mut self, anchor: PositionAnchor) -> Self {
        self.anchor = anchor;
        self
    }

    /// Borrow the underlying bus (for low-level diagnostic access).
    pub fn bus(&mut self) -> &mut B {
        &mut self.bus
    }

    /// Borrow the underlying [`Motor`] (for state inspection).
    pub fn motor(&self) -> &Motor {
        &self.motor
    }
}

impl LkMotor<Rs485Driver> {
    /// Convenience constructor: open an RS485 port and wrap it.
    pub fn open_rs485(
        device: &str,
        baud: u32,
        id: MotorId,
        config: MotorConfig,
        response_timeout: Duration,
    ) -> LkResult<Self> {
        let bus = Rs485Driver::open(device, baud, response_timeout)?;
        Ok(Self::new(bus, id, config))
    }
}

fn lk_to_misa_feedback(fb: LkFeedback) -> MisaFeedback {
    MisaFeedback {
        position_rad: fb.position_rad,
        velocity_rad_per_s: fb.velocity_rad_per_s,
        torque_nm: fb.torque_nm,
        current_a: fb.current_a,
        temperature_c: fb.temperature_c as f32,
    }
}

fn lk_to_misa_status(s: LkStatus) -> MisaStatus {
    MisaStatus {
        voltage_v: s.voltage_v,
        temperature_c: s.temperature_c as f32,
        error: lk_to_misa_error(s.error),
    }
}

fn lk_to_misa_error(f: LkErrorFlags) -> MisaErrorFlags {
    let raw = f.raw() as u32;
    let mut bits = 0u32;
    if f.under_voltage()    { bits |= MisaErrorFlags::UNDER_VOLTAGE; }
    if f.over_voltage()     { bits |= MisaErrorFlags::OVER_VOLTAGE; }
    if f.over_current()     { bits |= MisaErrorFlags::OVER_CURRENT; }
    if f.motor_overheat()   { bits |= MisaErrorFlags::MOTOR_OVERHEAT; }
    if f.driver_overheat()  { bits |= MisaErrorFlags::DRIVER_OVERHEAT; }
    if f.stalled()          { bits |= MisaErrorFlags::STALL; }
    if f.motor_short()      { bits |= MisaErrorFlags::MOTOR_SHORT; }
    if f.signal_timeout()   { bits |= MisaErrorFlags::SIGNAL_TIMEOUT; }
    MisaErrorFlags::new(bits, raw)
}

impl<B: LkBus> Actuator for LkMotor<B> {
    fn motor_id(&self) -> u8 {
        self.motor.id().get()
    }

    fn enable(&mut self) -> MisaResult<MisaFeedback> {
        // Step 1: wake the motor from a possible MotorStop (0x81) state.
        self.motor.enable(&mut self.bus)?;

        // Step 2: anchor zero at the current physical position. Required
        // for set_position (lkmotor V3 firmware always reports motor-frame
        // absolute angle, so a software-side anchor is needed for relative
        // `set_position(0.0, …) → "stay where you are"` semantics).
        self.motor.rezero(&mut self.bus)?;

        // Step 3: engage the position controller at the current location.
        // `MotorRun` alone doesn't activate any closed-loop controller —
        // sending a position-control command (0xA4) at the anchored zero
        // makes the motor hold in place (= "servo on" feel that the user
        // expects from Enable).
        let fb = self
            .motor
            .set_position(&mut self.bus, 0.0, DEFAULT_HOLD_MAX_SPEED)?;

        self.enabled = true;
        Ok(lk_to_misa_feedback(fb))
    }

    fn disable(&mut self) -> MisaResult<()> {
        self.motor.disable(&mut self.bus)?;
        self.enabled = false;
        Ok(())
    }

    fn set_zero(&mut self) -> MisaResult<()> {
        self.motor.rezero(&mut self.bus)?;
        Ok(())
    }

    fn set_run_mode(&mut self, _mode: RunMode) -> MisaResult<()> {
        // lkmotor V3 has no run-mode parameter — each motion command picks
        // its own controller. Treat as no-op so generic callers don't need
        // to special-case the family.
        Ok(())
    }

    fn set_position(
        &mut self,
        pos_rad: f32,
        max_speed_rad_s: f32,
    ) -> MisaResult<MisaFeedback> {
        let result = self.motor.set_position(&mut self.bus, pos_rad, max_speed_rad_s);
        match result {
            Ok(fb) => Ok(lk_to_misa_feedback(fb)),
            Err(crate::error::Error::PositionNotAnchored { .. })
                if matches!(self.anchor, PositionAnchor::OnFirstUse) =>
            {
                self.motor.rezero(&mut self.bus)?;
                Ok(lk_to_misa_feedback(
                    self.motor.set_position(&mut self.bus, pos_rad, max_speed_rad_s)?,
                ))
            }
            Err(e) => Err(e.into()),
        }
    }

    fn set_velocity(&mut self, vel_rad_s: f32) -> MisaResult<MisaFeedback> {
        Ok(lk_to_misa_feedback(
            self.motor.set_velocity(&mut self.bus, vel_rad_s)?,
        ))
    }

    fn set_torque(&mut self, torque_nm: f32) -> MisaResult<MisaFeedback> {
        // NOTE: torque_control (0xA1) latches on lkmotor V3 — the motor
        // keeps applying the commanded torque until a new command arrives.
        // For one-shot use through this trait this can spin the motor
        // indefinitely if the load doesn't balance the torque. Callers
        // that want torque control should re-issue this in a tight loop
        // (or use a velocity / position controller for safer one-shot use).
        Ok(lk_to_misa_feedback(
            self.motor.set_torque(&mut self.bus, torque_nm)?,
        ))
    }

    fn mit_control(
        &mut self,
        pos_rad: f32,
        vel_rad_s: f32,
        _kp_nm_per_rad: f32,
        _kd_nm_per_rad_s: f32,
        _torque_ff_nm: f32,
    ) -> MisaResult<MisaFeedback> {
        // SAFETY: the underlying `Motor::mit_control` does host-side PD
        // and emits a `torque_control` (0xA1) frame, which *latches* on
        // the firmware. A one-shot call would compute torque from the
        // current position error and the motor would then spin forever
        // applying that torque — clearly the wrong behaviour through the
        // misa::Actuator trait, which is meant to be safe to call once.
        //
        // We translate one-shot mit_control into a safe
        // `position_control_with_speed`: target = `pos_rad`, max_speed
        // derived from |vel_rad_s| (or a default). This loses the kp/kd/
        // tau_ff semantics, but the motor reaches the target safely and
        // holds there. For real PD-loop MIT, drive `Motor::mit_control`
        // from your own control loop.
        let max_speed = vel_rad_s.abs().max(DEFAULT_HOLD_MAX_SPEED);
        self.set_position(pos_rad, max_speed)
    }

    fn measure(&mut self) -> MisaResult<MisaFeedback> {
        Ok(lk_to_misa_feedback(self.motor.measure(&mut self.bus)?))
    }

    fn read_status(&mut self) -> MisaResult<MisaStatus> {
        Ok(lk_to_misa_status(self.motor.read_status(&mut self.bus)?))
    }

    fn current_run_mode_hint(&self) -> Option<RunMode> {
        // lkmotor has no run-mode concept, so leave this as None.
        None
    }

    fn is_enabled_hint(&self) -> bool {
        self.enabled
    }
}

impl<B: LkBus> Drop for LkMotor<B> {
    fn drop(&mut self) {
        let _ = self.motor.disable(&mut self.bus);
    }
}
