//! Bus abstraction and typed-command extension trait for the LK Motor V3 family.
//!
//! The transport layer (RS485, CAN, ...) only needs to provide the basic
//! request/response primitive [`LkBus::transact`]. All higher-level command
//! helpers are provided as default methods on [`LkCommands`], which is
//! blanket-implemented for any [`LkBus`] type. New transports only need to
//! wire up the wire I/O — every typed helper comes for free.

use lkmotor_protocol::command::{Command, ControlParamId};
use lkmotor_protocol::response::{
    ControlParamValue, MotorState1, MotorState2, PidTriple, parse_control_param,
    parse_multi_turn_angle, parse_state1, parse_state2, parse_state2_payload,
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

    /// Discard any buffered/in-flight bytes on the wire.
    fn flush_rx(&mut self) -> Result<()>;
}

/// Share one RS485 bus across several [`crate::LkMotor`] handles (a multi-drop
/// V3 bus): wrap an opened bus in [`misa_actuator::Shared`] and hand each motor
/// a clone. Each `transact` is serialized by the mutex — drive the motors from
/// a single control loop per bus.
impl<B: LkBus> LkBus for Shared<B> {
    fn transact(&mut self, command: u8, motor_id: MotorId, data: &[u8]) -> Result<Response> {
        self.lock().transact(command, motor_id, data)
    }

    fn flush_rx(&mut self) -> Result<()> {
        self.lock().flush_rx()
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
}

impl<B: LkBus + ?Sized> LkCommands for B {}
