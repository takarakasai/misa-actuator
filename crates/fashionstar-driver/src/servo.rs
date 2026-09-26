//! [`FashionStarServo`] — one servo on an [`FsBus`], implementing
//! [`misa_actuator::Actuator`].
//!
//! ## How the trait maps onto a hobby-class bus servo
//!
//! - **enable / disable** → `STOP_ON_CONTROL` with [`StopMode::Hold`] /
//!   [`StopMode::Release`]. There is no separate "torque on" command; hold
//!   is the servo locking at its current angle, release makes it limp.
//! - **set_position** → `SET_SERVO_ANGLE_MTURN` (code 13), *not* the
//!   single-turn `SET_SERVO_ANGLE`: `measure` reports the multi-turn
//!   angle, and commanding in the single-turn frame would disagree with it
//!   by 360° whenever the turn counter is non-zero — and could not reach
//!   the 0..270° gripper range at all past 180°. The move time comes from
//!   the distance and `max_speed_rad_s`.
//! - **measure / read_status** → `QUERY_SERVO_MONITOR`. The servo reports
//!   neither velocity nor torque; both are returned as `0.0`.
//! - **set_zero** → `SET_ORIGIN_POINT`, which **writes the servo's
//!   non-volatile memory** — see [`Actuator::set_zero`] below.
//! - velocity / torque / MIT → [`misa_actuator::Error::Unsupported`]. (The
//!   servo has a wheel mode, but it is a different operating mode, not a
//!   velocity loop at the joint.)
//!
//! For a whole arm, one [`crate::FsCommands::sync_monitor`] per cycle is
//! seven times cheaper than seven `measure` calls; this type exists so a
//! FashionStar joint can also be driven through the common interface
//! (TUI, sysid tools).

use std::ops::RangeInclusive;
use std::time::Duration;

use fashionstar_protocol::{status_bits, StopMode};
use misa_actuator::{
    Actuator, Error as MisaError, ErrorFlags, MotorFeedback, MotorStatus, Result as MisaResult,
    RunMode,
};

use crate::bus::FsBus;
use crate::commands::{FsCommands, Monitor};

/// `Actuator` over a (usually [`misa_actuator::Shared`]) FashionStar bus.
pub struct FashionStarServo<B: FsBus> {
    bus: B,
    id: u8,
    enabled: bool,
    hold_power_mw: u16,
}

impl<B: FsBus> FashionStarServo<B> {
    /// Servo `id` on `bus`. Pass a [`misa_actuator::Shared`] clone to put
    /// several servos on one port.
    pub fn new(bus: B, id: u8) -> Self {
        Self {
            bus,
            id,
            enabled: false,
            hold_power_mw: 0,
        }
    }

    /// Power argument sent with [`StopMode::Hold`] on `enable` (mW). `0`
    /// (default) is what the SDK sends.
    pub fn with_hold_power(mut self, power_mw: u16) -> Self {
        self.hold_power_mw = power_mw;
        self
    }

    /// The bus, for typed commands the trait does not cover.
    pub fn bus_mut(&mut self) -> &mut B {
        &mut self.bus
    }

    /// Servo id.
    pub fn id(&self) -> u8 {
        self.id
    }

    fn feedback(m: &Monitor) -> MotorFeedback {
        MotorFeedback {
            position_rad: m.angle_rad,
            velocity_rad_per_s: 0.0,
            torque_nm: 0.0,
            current_a: m.current_a,
            temperature_c: m.temperature_c,
        }
    }
}

/// Map the status byte onto the common flags; the raw byte is kept.
fn error_flags(status: u8) -> ErrorFlags {
    let map = [
        (status_bits::STALL, ErrorFlags::STALL),
        (status_bits::OVER_VOLTAGE, ErrorFlags::OVER_VOLTAGE),
        (status_bits::UNDER_VOLTAGE, ErrorFlags::UNDER_VOLTAGE),
        (status_bits::CURRENT_ERROR, ErrorFlags::OVER_CURRENT),
        (status_bits::TEMPERATURE_ERROR, ErrorFlags::MOTOR_OVERHEAT),
        // EXECUTING, COMMAND_ERROR and POWER_ERROR have no common bit; they
        // stay visible through `raw()`.
    ];
    let bits = map
        .iter()
        .filter(|(s, _)| status & s != 0)
        .fold(0, |acc, (_, c)| acc | c);
    ErrorFlags::new(bits, status as u32)
}

impl<B: FsBus> Actuator for FashionStarServo<B> {
    fn motor_id(&self) -> u8 {
        self.id
    }

    /// Lock the joint where it is (`STOP_ON_CONTROL` hold).
    fn enable(&mut self) -> MisaResult<MotorFeedback> {
        self.bus
            .stop_with_power(self.id, StopMode::Hold, self.hold_power_mw)?;
        self.enabled = true;
        self.measure()
    }

    /// Release the joint (`STOP_ON_CONTROL` release) — it goes limp.
    fn disable(&mut self) -> MisaResult<()> {
        self.bus.stop(self.id, StopMode::Release)?;
        self.enabled = false;
        Ok(())
    }

    /// **Writes the servo's non-volatile memory** (`SET_ORIGIN_POINT`).
    ///
    /// Unlike the soft zero most drivers in this workspace keep on the host,
    /// this moves the servo's own origin permanently: it survives power
    /// cycles and silently re-bases every angle recorded before it. On a
    /// calibrated arm that invalidates the calibration. Call it only as a
    /// deliberate calibration step, never per session or per loop.
    fn set_zero(&mut self) -> MisaResult<()> {
        self.bus.set_origin_point(self.id)?;
        Ok(())
    }

    fn set_run_mode(&mut self, mode: RunMode) -> MisaResult<()> {
        match mode {
            RunMode::Position => Ok(()),
            RunMode::Velocity => Err(MisaError::Unsupported("velocity mode")),
            RunMode::Torque => Err(MisaError::Unsupported("torque mode")),
            RunMode::Mit => Err(MisaError::Unsupported("MIT mode")),
        }
    }

    /// Multi-turn move whose duration is `|target − current| / max_speed`.
    ///
    /// The command has no reply, so the monitor read that supplies the
    /// current angle is also what is returned (i.e. the pre-move state).
    fn set_position(&mut self, pos_rad: f32, max_speed_rad_s: f32) -> MisaResult<MotorFeedback> {
        if !pos_rad.is_finite() {
            return Err(MisaError::Other("set_position: target is not finite".into()));
        }
        // Written so NaN fails too.
        if max_speed_rad_s.is_nan() || max_speed_rad_s <= 0.0 {
            return Err(MisaError::Other(
                "set_position: max_speed_rad_s must be > 0".into(),
            ));
        }
        let now = self.bus.monitor(self.id)?;
        let secs = ((pos_rad - now.angle_rad).abs() / max_speed_rad_s) as f64;
        let interval = Duration::from_secs_f64(secs.min(4096.0));
        self.bus.set_angle_multiturn(self.id, pos_rad, interval)?;
        Ok(Self::feedback(&now))
    }

    fn set_velocity(&mut self, _vel_rad_s: f32) -> MisaResult<MotorFeedback> {
        Err(MisaError::Unsupported("set_velocity"))
    }

    fn set_torque(&mut self, _torque_nm: f32) -> MisaResult<MotorFeedback> {
        Err(MisaError::Unsupported("set_torque"))
    }

    fn mit_control(
        &mut self,
        _pos_rad: f32,
        _vel_rad_s: f32,
        _kp: f32,
        _kd: f32,
        _torque_ff_nm: f32,
    ) -> MisaResult<MotorFeedback> {
        Err(MisaError::Unsupported("mit_control"))
    }

    fn measure(&mut self) -> MisaResult<MotorFeedback> {
        let m = self.bus.monitor(self.id)?;
        Ok(Self::feedback(&m))
    }

    fn read_status(&mut self) -> MisaResult<MotorStatus> {
        let m = self.bus.monitor(self.id)?;
        Ok(MotorStatus {
            voltage_v: m.voltage_v,
            temperature_c: m.temperature_c,
            error: error_flags(m.status),
        })
    }

    /// The reported angle is the servo's own; there is no host-side offset.
    fn position_zero_in_motor_frame_rad(&self) -> Option<f32> {
        Some(0.0)
    }

    fn is_enabled_hint(&self) -> bool {
        self.enabled
    }

    fn scan_bus(
        &mut self,
        id_range: RangeInclusive<u8>,
        timeout_per_id: Duration,
    ) -> MisaResult<Vec<u8>> {
        let saved = self.bus.timeout();
        self.bus.set_timeout(timeout_per_id);
        let mut found = Vec::new();
        let mut result = Ok(());
        for id in id_range {
            match self.bus.ping(id) {
                Ok(true) => found.push(id),
                Ok(false) => {}
                Err(e) => {
                    result = Err(e);
                    break;
                }
            }
        }
        // Restore before propagating, so a failed scan does not leave the
        // shared bus on the probe timeout.
        self.bus.set_timeout(saved);
        result?;
        Ok(found)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_mapping() {
        let f = error_flags(status_bits::STALL | status_bits::EXECUTING);
        assert!(f.stall());
        assert!(!f.over_voltage());
        assert_eq!(f.raw(), 0b101);
        assert!(!error_flags(status_bits::EXECUTING).any());
    }
}
