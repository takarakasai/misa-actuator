//! Bus abstraction and typed-command extension trait for the LK Motor V3 family.
//!
//! The transport layer (RS485, CAN, ...) only needs to provide the basic
//! request/response primitive [`LkBus::transact`]. All higher-level command
//! helpers are provided as default methods on [`LkCommands`], which is
//! blanket-implemented for any [`LkBus`] type. New transports only need to
//! wire up the wire I/O — every typed helper comes for free.

use std::time::Duration;

use lkmotor_protocol::command::{Command, ControlParamId, SettingParamId};
use lkmotor_protocol::response::{
    ControlParamValue, LegacyPids, MotorState1, MotorState2, PidTriple, SettingParamValue,
    parse_brake_state, parse_control_param, parse_legacy_accel, parse_legacy_pids,
    parse_clear_error, parse_control_param_write, parse_multi_turn_angle, parse_setting_param, parse_single_turn_angle,
    parse_state1, parse_state2, parse_state2_payload,
};

use misa_actuator::Shared;

use crate::error::Result;
use crate::motor_id::MotorId;

/// An owned response frame returned from any [`LkBus`] transaction.
#[derive(Debug, Clone)]
pub struct Response {
    /// V3 command echo byte from the response frame.
    pub command: u8,
    /// Source motor id from the response frame.
    pub motor_id: u8,
    /// Response payload (already validated by the bus).
    pub data: Vec<u8>,
}

/// Application-level request/response transport for an LK Motor V3 bus.
///
/// `transact` takes the **command byte and raw payload** (not wire-encoded
/// bytes) — the bus implementation is responsible for the on-the-wire
/// framing (V3 RS485 checksum frame, CAN extended frame, ...). This keeps
/// the typed helpers in [`LkCommands`] transport-agnostic.
pub trait LkBus {
    /// Send a request and return the next response addressed to `motor_id`.
    fn transact(&mut self, command: u8, motor_id: MotorId, data: &[u8]) -> Result<Response>;

    /// Send a request that the motor does not reply to (e.g.
    /// [`Command::MotorRestart`]) — fire-and-forget, no wait for a response.
    fn send_only(&mut self, command: u8, motor_id: MotorId, data: &[u8]) -> Result<()>;

    /// Discard any buffered/in-flight bytes on the wire.
    fn flush_rx(&mut self) -> Result<()>;

    /// Listen for a *further* frame addressed to `motor_id` without sending
    /// anything, and return it if one arrives inside `window`.
    ///
    /// One request should produce one reply. A second one means two devices
    /// hold the same id — see [`crate::lk_motor::ProbeReport`] for why that
    /// matters and how badly a plain "did anyone answer" probe misreads it.
    ///
    /// The default returns `Ok(None)`, so a transport that cannot listen
    /// passively simply never reports a duplicate. Pair it with
    /// [`Self::can_detect_duplicate_ids`] to tell "no duplicate" apart from
    /// "cannot tell".
    fn recv_extra(&mut self, motor_id: MotorId, window: Duration) -> Result<Option<Response>> {
        let _ = (motor_id, window);
        Ok(None)
    }

    /// Whether [`Self::recv_extra`] really listens on this transport.
    fn can_detect_duplicate_ids(&self) -> bool {
        false
    }
}

/// Share one RS485 bus across several [`crate::LkMotor`] handles (a multi-drop
/// V3 bus): wrap an opened bus in [`misa_actuator::Shared`] and hand each motor
/// a clone. Each `transact` is serialized by the mutex — drive the motors from
/// a single control loop per bus.
impl<B: LkBus> LkBus for Shared<B> {
    fn transact(&mut self, command: u8, motor_id: MotorId, data: &[u8]) -> Result<Response> {
        self.lock().transact(command, motor_id, data)
    }

    fn send_only(&mut self, command: u8, motor_id: MotorId, data: &[u8]) -> Result<()> {
        self.lock().send_only(command, motor_id, data)
    }

    fn flush_rx(&mut self) -> Result<()> {
        self.lock().flush_rx()
    }

    fn recv_extra(&mut self, motor_id: MotorId, window: Duration) -> Result<Option<Response>> {
        self.lock().recv_extra(motor_id, window)
    }

    fn can_detect_duplicate_ids(&self) -> bool {
        self.lock().can_detect_duplicate_ids()
    }
}

/// Decode the State2 payload from any motion-control reply
/// (`0xA1` / `0xA2` / `0xA3` / `0xA4` / ...) — bypasses the strict command-
/// code check that [`parse_state2`] performs.
pub fn parse_state2_from_response(resp: &Response) -> Result<MotorState2> {
    Ok(parse_state2_payload(&resp.data)?)
}

const SCALE_AMPS_TO_RAW: f32 = 2048.0 / 33.0;

fn current_amps_to_raw(current_a: f32) -> i16 {
    let scaled = current_a * SCALE_AMPS_TO_RAW;
    let bias = if scaled >= 0.0 { 0.5 } else { -0.5 };
    (scaled + bias).clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

/// Typed command helpers, available on any [`LkBus`].
///
/// Bring it into scope (`use lkmotor_driver::LkCommands;`) to call the
/// typed helpers directly on a bus value.
pub trait LkCommands: LkBus {
    /// Power off the motor (`0x80`). **Bench note:** on V2 firmware this
    /// hangs the bus until power-cycle — prefer [`Self::motor_stop`].
    fn motor_off(&mut self, motor_id: MotorId) -> Result<()> {
        self.transact(Command::MotorOff.code(), motor_id, &[])?;
        Ok(())
    }

    /// Stop the motor while keeping the run flag (`0x81`).
    fn motor_stop(&mut self, motor_id: MotorId) -> Result<()> {
        self.transact(Command::MotorStop.code(), motor_id, &[])?;
        Ok(())
    }

    /// Resume motor operation (`0x88`).
    fn motor_run(&mut self, motor_id: MotorId) -> Result<()> {
        self.transact(Command::MotorRun.code(), motor_id, &[])?;
        Ok(())
    }

    /// Read motor state 1 (temperature, voltage, error flags).
    fn read_state1(&mut self, motor_id: MotorId) -> Result<MotorState1> {
        let resp = self.transact(Command::ReadMotorState1.code(), motor_id, &[])?;
        Ok(parse_state1(resp.command, &resp.data)?)
    }

    /// Read motor state 2 (temperature, current, speed, encoder position).
    fn read_state2(&mut self, motor_id: MotorId) -> Result<MotorState2> {
        let resp = self.transact(Command::ReadMotorState2.code(), motor_id, &[])?;
        Ok(parse_state2(resp.command, &resp.data)?)
    }

    /// Read all three PID loops through the legacy `ReadPid` (`0x30`) command.
    ///
    /// Use when [`Self::read_position_pid`] and friends return an error: which
    /// of the two parameter interfaces answers is a property of the driver
    /// board, so a tool that supports both boards needs this fallback. Note
    /// the legacy response carries no `kd` — see [`LegacyPids`].
    fn read_legacy_pids(&mut self, motor_id: MotorId) -> Result<LegacyPids> {
        let resp = self.transact(Command::ReadPid.code(), motor_id, &[])?;
        Ok(parse_legacy_pids(resp.command, &resp.data)?)
    }

    /// Read the acceleration setting through the legacy `ReadAccel` (`0x33`)
    /// command, in 1 dps/s. Same board-dependent availability as
    /// [`Self::read_legacy_pids`].
    fn read_legacy_accel(&mut self, motor_id: MotorId) -> Result<i32> {
        let resp = self.transact(Command::ReadAccel.code(), motor_id, &[])?;
        Ok(parse_legacy_accel(resp.command, &resp.data)?)
    }

    /// Read one control parameter (`0xC0`). Returns the typed value.
    fn read_control_param(
        &mut self,
        motor_id: MotorId,
        param: ControlParamId,
    ) -> Result<ControlParamValue> {
        let mut data = [0u8; 7];
        data[0] = param.code();
        let resp = self.transact(Command::ReadControlParam.code(), motor_id, &data)?;
        Ok(parse_control_param(resp.command, &resp.data, param)?)
    }

    /// Convenience wrapper for the position-loop PID (`paramID = 0x0A`).
    fn read_position_pid(&mut self, motor_id: MotorId) -> Result<PidTriple> {
        match self.read_control_param(motor_id, ControlParamId::PositionLoopPid)? {
            ControlParamValue::Pid(p) => Ok(p),
            _ => unreachable!("parse_control_param returned wrong variant for PositionLoopPid"),
        }
    }

    /// Convenience wrapper for the speed-loop PID (`paramID = 0x0B`).
    fn read_speed_pid(&mut self, motor_id: MotorId) -> Result<PidTriple> {
        match self.read_control_param(motor_id, ControlParamId::SpeedLoopPid)? {
            ControlParamValue::Pid(p) => Ok(p),
            _ => unreachable!("parse_control_param returned wrong variant for SpeedLoopPid"),
        }
    }

    /// Convenience wrapper for the current-loop PID (`paramID = 0x0C`).
    fn read_current_pid(&mut self, motor_id: MotorId) -> Result<PidTriple> {
        match self.read_control_param(motor_id, ControlParamId::CurrentLoopPid)? {
            ControlParamValue::Pid(p) => Ok(p),
            _ => unreachable!("parse_control_param returned wrong variant for CurrentLoopPid"),
        }
    }

    /// Convenience wrapper for the torque/current limit (`paramID = 0x1E`).
    fn read_torque_limit(&mut self, motor_id: MotorId) -> Result<i16> {
        match self.read_control_param(motor_id, ControlParamId::TorqueLimit)? {
            ControlParamValue::TorqueLimit(v) => Ok(v),
            _ => unreachable!("parse_control_param returned wrong variant for TorqueLimit"),
        }
    }

    /// Convenience wrapper for the speed limit (`paramID = 0x20`, 0.01 deg/s units).
    fn read_speed_limit(&mut self, motor_id: MotorId) -> Result<i32> {
        match self.read_control_param(motor_id, ControlParamId::SpeedLimit)? {
            ControlParamValue::SpeedLimit(v) => Ok(v),
            _ => unreachable!("parse_control_param returned wrong variant for SpeedLimit"),
        }
    }

    /// Convenience wrapper for the angle limit (`paramID = 0x22`, 0.01 deg units).
    fn read_angle_limit(&mut self, motor_id: MotorId) -> Result<i32> {
        match self.read_control_param(motor_id, ControlParamId::AngleLimit)? {
            ControlParamValue::AngleLimit(v) => Ok(v),
            _ => unreachable!("parse_control_param returned wrong variant for AngleLimit"),
        }
    }

    /// Convenience wrapper for the current ramp (`paramID = 0x24`).
    fn read_current_ramp(&mut self, motor_id: MotorId) -> Result<i32> {
        match self.read_control_param(motor_id, ControlParamId::CurrentRamp)? {
            ControlParamValue::CurrentRamp(v) => Ok(v),
            _ => unreachable!("parse_control_param returned wrong variant for CurrentRamp"),
        }
    }

    /// Convenience wrapper for the speed ramp (`paramID = 0x26`, 1 dps/s units).
    fn read_speed_ramp(&mut self, motor_id: MotorId) -> Result<i32> {
        match self.read_control_param(motor_id, ControlParamId::SpeedRamp)? {
            ControlParamValue::SpeedRamp(v) => Ok(v),
            _ => unreachable!("parse_control_param returned wrong variant for SpeedRamp"),
        }
    }

    /// Read a setting parameter (`0x40`, "Setting Parameter Table" — a
    /// *different* parameter space from [`Self::read_control_param`], see
    /// [`SettingParamId`]'s doc comment).
    fn read_setting_param(
        &mut self,
        motor_id: MotorId,
        param: SettingParamId,
    ) -> Result<SettingParamValue> {
        let data = [0x05u8, param.code(), 0, 0, 0, 0, 0];
        let resp = self.transact(Command::ReadSettingParam.code(), motor_id, &data)?;
        Ok(parse_setting_param(resp.command, &resp.data, param)?)
    }

    /// Convenience wrapper for the driver ID setting.
    fn read_driver_id(&mut self, motor_id: MotorId) -> Result<u8> {
        match self.read_setting_param(motor_id, SettingParamId::DriverId)? {
            SettingParamValue::DriverId(v) => Ok(v),
            _ => unreachable!("parse_setting_param returned wrong variant for DriverId"),
        }
    }

    /// Convenience wrapper for the bus-type setting (0=None, 1=RS485, 2=CAN).
    fn read_bus_type(&mut self, motor_id: MotorId) -> Result<u8> {
        match self.read_setting_param(motor_id, SettingParamId::BusType)? {
            SettingParamValue::BusType(v) => Ok(v),
            _ => unreachable!("parse_setting_param returned wrong variant for BusType"),
        }
    }

    /// Convenience wrapper for the RS485 baud-rate code setting.
    fn read_rs485_baudrate(&mut self, motor_id: MotorId) -> Result<u8> {
        match self.read_setting_param(motor_id, SettingParamId::Rs485Baudrate)? {
            SettingParamValue::Rs485Baudrate(v) => Ok(v),
            _ => unreachable!("parse_setting_param returned wrong variant for Rs485Baudrate"),
        }
    }

    /// Convenience wrapper for the CAN baud-rate code setting.
    fn read_can_baudrate(&mut self, motor_id: MotorId) -> Result<u8> {
        match self.read_setting_param(motor_id, SettingParamId::CanBaudrate)? {
            SettingParamValue::CanBaudrate(v) => Ok(v),
            _ => unreachable!("parse_setting_param returned wrong variant for CanBaudrate"),
        }
    }

    /// Convenience wrapper for the max-power setting.
    fn read_max_power(&mut self, motor_id: MotorId) -> Result<i16> {
        match self.read_setting_param(motor_id, SettingParamId::MaxPower)? {
            SettingParamValue::MaxPower(v) => Ok(v),
            _ => unreachable!("parse_setting_param returned wrong variant for MaxPower"),
        }
    }

    /// Convenience wrapper for the max-speed setting (0.01 deg/s units).
    fn read_max_speed_setting(&mut self, motor_id: MotorId) -> Result<i32> {
        match self.read_setting_param(motor_id, SettingParamId::MaxSpeed)? {
            SettingParamValue::MaxSpeed(v) => Ok(v),
            _ => unreachable!("parse_setting_param returned wrong variant for MaxSpeed"),
        }
    }

    /// Convenience wrapper for the max-angle setting (0.01 deg units).
    fn read_max_angle_setting(&mut self, motor_id: MotorId) -> Result<i32> {
        match self.read_setting_param(motor_id, SettingParamId::MaxAngle)? {
            SettingParamValue::MaxAngle(v) => Ok(v),
            _ => unreachable!("parse_setting_param returned wrong variant for MaxAngle"),
        }
    }

    /// Convenience wrapper for the (persisted) current-ramp setting. **Note**:
    /// this is `int16` per the Setting Parameter Table, unlike
    /// [`Self::read_current_ramp`]'s `int32` — see [`SettingParamId::CurrentRamp`].
    fn read_setting_current_ramp(&mut self, motor_id: MotorId) -> Result<i16> {
        match self.read_setting_param(motor_id, SettingParamId::CurrentRamp)? {
            SettingParamValue::CurrentRamp(v) => Ok(v),
            _ => unreachable!("parse_setting_param returned wrong variant for CurrentRamp"),
        }
    }

    /// Convenience wrapper for the (persisted) speed-ramp setting (1 dps/s units).
    fn read_setting_speed_ramp(&mut self, motor_id: MotorId) -> Result<i32> {
        match self.read_setting_param(motor_id, SettingParamId::SpeedRamp)? {
            SettingParamValue::SpeedRamp(v) => Ok(v),
            _ => unreachable!("parse_setting_param returned wrong variant for SpeedRamp"),
        }
    }

    /// Write an 8-bit setting parameter (`0x42`) to RAM — see
    /// [`Self::save_setting_params`] to persist it. Only
    /// [`SettingParamId::DriverId`], [`SettingParamId::BusType`],
    /// [`SettingParamId::Rs485Baudrate`], and [`SettingParamId::CanBaudrate`]
    /// take a `u8` value; passing another [`SettingParamId`] here would
    /// encode a nonsensical write, so callers should stick to those four.
    fn write_setting_param_u8(
        &mut self,
        motor_id: MotorId,
        param: SettingParamId,
        value: u8,
    ) -> Result<SettingParamValue> {
        let data = [0x05u8, param.code(), value, 0, 0, 0, 0];
        let resp = self.transact(Command::WriteSettingParam.code(), motor_id, &data)?;
        Ok(parse_setting_param(resp.command, &resp.data, param)?)
    }

    /// Write a 16-bit setting parameter (`0x42`) to RAM —
    /// [`SettingParamId::MaxPower`] or [`SettingParamId::CurrentRamp`].
    fn write_setting_param_i16(
        &mut self,
        motor_id: MotorId,
        param: SettingParamId,
        value: i16,
    ) -> Result<SettingParamValue> {
        let mut data = [0x05u8, param.code(), 0, 0, 0, 0, 0];
        data[2..4].copy_from_slice(&value.to_le_bytes());
        let resp = self.transact(Command::WriteSettingParam.code(), motor_id, &data)?;
        Ok(parse_setting_param(resp.command, &resp.data, param)?)
    }

    /// Write a 32-bit setting parameter (`0x42`) to RAM —
    /// [`SettingParamId::MaxSpeed`], [`SettingParamId::MaxAngle`], or
    /// [`SettingParamId::SpeedRamp`].
    fn write_setting_param_i32(
        &mut self,
        motor_id: MotorId,
        param: SettingParamId,
        value: i32,
    ) -> Result<SettingParamValue> {
        let mut data = [0x05u8, param.code(), 0, 0, 0, 0, 0];
        data[2..6].copy_from_slice(&value.to_le_bytes());
        let resp = self.transact(Command::WriteSettingParam.code(), motor_id, &data)?;
        Ok(parse_setting_param(resp.command, &resp.data, param)?)
    }

    /// Commit all `write_setting_param_*` RAM writes to ROM (`0x44`). Takes
    /// effect after [`Self::motor_restart`] or a power cycle.
    fn save_setting_params(&mut self, motor_id: MotorId) -> Result<()> {
        let data = [0x05u8, 0xFA, 0, 0, 0, 0, 0];
        self.transact(Command::SaveSettingParam.code(), motor_id, &data)?;
        Ok(())
    }

    /// Restart the motor (`0x07`), equivalent to a power cycle. The motor
    /// sends no reply, so this uses [`LkBus::send_only`] rather than
    /// [`LkBus::transact`] (which would otherwise time out waiting for a
    /// response that never arrives).
    ///
    /// # Not in the RS485 manual
    ///
    /// Documented only for CAN (§29 "Motor restart"). Whether the RS485
    /// firmware honours it is unverified.
    ///
    /// **The CAN manual cannot tell you the RS485 framing.** On CAN every
    /// command pads `DATA[1..7]` with nulls, so `0x92` and `0x95` look
    /// identical there — yet over RS485 one carries `CMD[3] = 0x00` and the
    /// other `CMD[3] = 0x07` with seven zero bytes. Guessing that wrong on
    /// `0x95` left a multi-turn counter at an arbitrary value twice
    /// (measured on MG4005, 2026-08-21).
    ///
    /// The empty payload used here (`CMD[3] = 0x00`) is the right bet
    /// because **every argument-less command in the RS485 manual uses it** —
    /// motor off/run/stop, read status, clear error, read multi-turn, read
    /// single-turn. §24 (`0x95`) is the sole exception in the document.
    ///
    /// # It resets the multi-turn origin
    ///
    /// A power cycle is exactly what it says: the multi-turn frame restarts
    /// at whatever position the shaft is in, so any calibration expressed in
    /// the old frame is gone. Prefer [`Motor::restart`], which clears the
    /// host-side tracker to match.
    fn motor_restart(&mut self, motor_id: MotorId) -> Result<()> {
        self.send_only(Command::MotorRestart.code(), motor_id, &[])
    }

    /// Closed-loop torque/current control (`0xA1`).
    fn torque_control(&mut self, motor_id: MotorId, current_a: f32) -> Result<Response> {
        let raw = current_amps_to_raw(current_a);
        self.transact(Command::TorqueClosedLoop.code(), motor_id, &raw.to_le_bytes())
    }

    /// Closed-loop speed control (`0xA2`). `centideg_per_s` = signed `0.01 deg/s`.
    fn speed_control(&mut self, motor_id: MotorId, centideg_per_s: i32) -> Result<Response> {
        self.transact(
            Command::SpeedClosedLoop.code(),
            motor_id,
            &centideg_per_s.to_le_bytes(),
        )
    }

    /// Closed-loop multi-turn position control (`0xA3`). `centideg` = signed `0.01 deg`.
    fn position_control(&mut self, motor_id: MotorId, centideg: i64) -> Result<Response> {
        self.transact(
            Command::PositionClosedLoop1.code(),
            motor_id,
            &centideg.to_le_bytes(),
        )
    }

    /// Closed-loop multi-turn position control with max-speed cap (`0xA4`).
    fn position_control_with_speed(
        &mut self,
        motor_id: MotorId,
        position_centideg: i64,
        max_speed_centideg_per_s: u32,
    ) -> Result<Response> {
        let mut data = [0u8; 12];
        data[0..8].copy_from_slice(&position_centideg.to_le_bytes());
        data[8..12].copy_from_slice(&max_speed_centideg_per_s.to_le_bytes());
        self.transact(Command::PositionClosedLoop2.code(), motor_id, &data)
    }

    /// Read the motor's multi-turn absolute angle (`0x92`).
    fn read_multi_turn_angle(&mut self, motor_id: MotorId) -> Result<i64> {
        let resp = self.transact(Command::ReadMultiTurnAngle.code(), motor_id, &[])?;
        Ok(parse_multi_turn_angle(resp.command, &resp.data)?)
    }

    /// Write a PID triple into **RAM** (`0xC1`, manual §19).
    ///
    /// **RAM only: the value takes effect immediately and is lost at the next
    /// power-down.** That is the point — gains can be tried on the bench and
    /// undone by cutting power. There is a ROM variant in the manual; it is
    /// deliberately not wired here.
    ///
    /// Each gain is `u16`, range 0..=2000 per the parameter table. The drive
    /// echoes the write back in the same shape as [`Self::read_control_param`],
    /// so the caller can confirm what actually landed rather than assuming.
    ///
    /// **Not every drive answers this.** Firmware generations differ: on one
    /// MG4005 fleet, 5 of 12 axes answered only the undocumented `0x30`
    /// family and 7 answered only `0xC0`/`0xC1` (measured 2026-08-22).
    /// Pair this with the interface that the *read* succeeded on.
    fn write_control_pid_ram(
        &mut self,
        motor_id: MotorId,
        param: ControlParamId,
        pid: PidTriple,
    ) -> Result<ControlParamValue> {
        let mut data = [0u8; 7];
        data[0] = param.code();
        data[1..3].copy_from_slice(&pid.kp.to_le_bytes());
        data[3..5].copy_from_slice(&pid.ki.to_le_bytes());
        data[5..7].copy_from_slice(&pid.kd.to_le_bytes());
        let resp = self.transact(Command::WriteControlParamRam.code(), motor_id, &data)?;
        Ok(parse_control_param_write(resp.command, &resp.data, param)?)
    }

    /// Clear the drive's latched error flags (`0x9B`) and return the status
    /// it answers with.
    ///
    /// **The reply carries the flags as they stand after the attempt.**
    /// Manual §2: "the error flags cannot be cleared while the motor state has
    /// not yet returned to normal" — a drive still under its low-voltage
    /// threshold answers with the flag still set. Callers must look at
    /// `error_state` instead of treating `Ok(_)` as success.
    fn clear_error(&mut self, motor_id: MotorId) -> Result<MotorState1> {
        let resp = self.transact(Command::ClearError.code(), motor_id, &[])?;
        Ok(parse_clear_error(resp.command, &resp.data)?)
    }

    /// Read the motor's single-turn absolute angle (`0x94`), 0.01°/LSB,
    /// `0..=35999`.
    ///
    /// Referenced to the encoder zero stored in the drive's ROM, so **this
    /// value reproduces across a power cycle** — unlike
    /// [`Self::read_multi_turn_angle`], which restarts at 0 every power-up.
    /// It only pins the shaft down within one motor revolution.
    fn read_single_turn_angle(&mut self, motor_id: MotorId) -> Result<u32> {
        let resp = self.transact(Command::ReadSingleTurnAngle.code(), motor_id, &[])?;
        Ok(parse_single_turn_angle(resp.command, &resp.data)?)
    }

    /// Set the current position as the multi-turn zero point (`0x95`, **RAM only**).
    ///
    /// Manual §24. The effect on the multi-turn frame is exactly that of a
    /// power cycle — the origin lands under the shaft where it is now — without
    /// cutting power. Lost at the next real power-up.
    ///
    /// Not to be confused with `0x19` (`WriteCurrentPosAsZero`), which writes
    /// ROM and costs flash cycles. This one writes nothing persistent.
    ///
    /// **The frame carries 7 zero data bytes and the drive replies** with the
    /// same 8 bytes (manual §24). Both matter: sending an empty payload was
    /// measured on MG4005 to leave the multi-turn counter at a large arbitrary
    /// value (+293 rad on a joint that had been near zero, 2026-08-21).
    ///
    /// Callers holding a host-side frame offset (see `Motor::read_absolute_angle`)
    /// must re-establish it afterwards; the motor's frame moved under them.
    fn set_current_position_as_zero_ram(&mut self, motor_id: MotorId) -> Result<()> {
        self.transact(
            Command::SetCurrentPositionAsZeroRam.code(),
            motor_id,
            &[0u8; 7],
        )?;
        Ok(())
    }

    /// Single-turn position control 1 (`0xA5`). `counterclockwise`: `false`=CW,
    /// `true`=CCW. `angle_centideg` is unsigned `0.01 deg/LSB`. Reply is
    /// State2-shaped — decode with [`parse_state2_from_response`].
    fn position_control_singleturn(
        &mut self,
        motor_id: MotorId,
        counterclockwise: bool,
        angle_centideg: u32,
    ) -> Result<Response> {
        let mut data = [0u8; 7];
        data[0] = counterclockwise as u8;
        data[3..7].copy_from_slice(&angle_centideg.to_le_bytes());
        self.transact(Command::PositionClosedLoop3.code(), motor_id, &data)
    }

    /// Single-turn position control 2 (`0xA6`) — as
    /// [`Self::position_control_singleturn`] plus a max-speed cap (1 dps/LSB).
    fn position_control_singleturn_with_speed(
        &mut self,
        motor_id: MotorId,
        counterclockwise: bool,
        max_speed_dps: u16,
        angle_centideg: u32,
    ) -> Result<Response> {
        let mut data = [0u8; 7];
        data[0] = counterclockwise as u8;
        data[1..3].copy_from_slice(&max_speed_dps.to_le_bytes());
        data[3..7].copy_from_slice(&angle_centideg.to_le_bytes());
        self.transact(Command::PositionClosedLoop4.code(), motor_id, &data)
    }

    /// Incremental position control 1 (`0xA7`). `angle_increment_centideg` is
    /// signed `0.01 deg/LSB`; the sign selects the direction of motion.
    fn position_control_incremental(
        &mut self,
        motor_id: MotorId,
        angle_increment_centideg: i32,
    ) -> Result<Response> {
        let mut data = [0u8; 7];
        data[3..7].copy_from_slice(&angle_increment_centideg.to_le_bytes());
        self.transact(Command::IncrementalPosition1.code(), motor_id, &data)
    }

    /// Incremental position control 2 (`0xA8`) — as
    /// [`Self::position_control_incremental`] plus a max-speed cap. **Note**:
    /// wire field is 2 bytes (`u16`) despite the manual's prose claiming
    /// `uint32_t` — see `lkmotor_protocol::request::encode_position_incremental_with_speed`.
    fn position_control_incremental_with_speed(
        &mut self,
        motor_id: MotorId,
        max_speed_dps: u16,
        angle_increment_centideg: i32,
    ) -> Result<Response> {
        let mut data = [0u8; 7];
        data[1..3].copy_from_slice(&max_speed_dps.to_le_bytes());
        data[3..7].copy_from_slice(&angle_increment_centideg.to_le_bytes());
        self.transact(Command::IncrementalPosition2.code(), motor_id, &data)
    }

    /// Engage or release the brake (`0x8C`). `released=false` holds the
    /// output (brake engaged); `released=true` frees it.
    fn set_brake(&mut self, motor_id: MotorId, released: bool) -> Result<bool> {
        let mut data = [0u8; 7];
        data[0] = released as u8;
        let resp = self.transact(Command::BrakeControl.code(), motor_id, &data)?;
        Ok(parse_brake_state(resp.command, &resp.data)?)
    }

    /// Read the current brake state (`0x8C`, read-marker `0x10`). Returns
    /// `true` if released (free), `false` if engaged (holding).
    fn read_brake_state(&mut self, motor_id: MotorId) -> Result<bool> {
        let mut data = [0u8; 7];
        data[0] = 0x10;
        let resp = self.transact(Command::BrakeControl.code(), motor_id, &data)?;
        Ok(parse_brake_state(resp.command, &resp.data)?)
    }
}

impl<B: LkBus + ?Sized> LkCommands for B {}
