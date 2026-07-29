//! Builders for the most common MG4005 request frames.
//!
//! Each helper writes into a caller-supplied buffer and returns the number of
//! bytes used, so the same APIs work in `no_std` contexts. Refer to the LK-Tech
//! RS485 V3 manual for payload semantics — the comments below capture what is
//! widely documented but should be treated as a starting point, not a contract.

use crate::command::{Command, ControlParamId, SettingParamId};
use crate::frame::{EncodeError, encode};

/// Encode a no-payload command (motor on/off, state reads, etc.).
pub fn encode_simple(
    command: Command,
    motor_id: u8,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    encode(command.code(), motor_id, &[], out)
}

/// Power off the motor (`0x80`).
pub fn encode_motor_off(motor_id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_simple(Command::MotorOff, motor_id, out)
}

/// Stop the motor while keeping the run flag (`0x81`).
pub fn encode_motor_stop(motor_id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_simple(Command::MotorStop, motor_id, out)
}

/// Resume motor operation (`0x88`).
pub fn encode_motor_run(motor_id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_simple(Command::MotorRun, motor_id, out)
}

/// Read motor state 1 (`0x9A`) — temperature, voltage, error flags.
pub fn encode_read_state1(motor_id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_simple(Command::ReadMotorState1, motor_id, out)
}

/// Read motor state 2 (`0x9C`) — temperature, current, speed, encoder position.
pub fn encode_read_state2(motor_id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_simple(Command::ReadMotorState2, motor_id, out)
}

/// Read motor state 3 (`0x9D`) — phase currents.
pub fn encode_read_state3(motor_id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_simple(Command::ReadMotorState3, motor_id, out)
}

/// Closed-loop torque control (`0xA1`).
///
/// `iq_control` is the target torque current in raw units — typically signed
/// int16 little-endian where `±2048` maps to roughly `±33 A` on the MG4005.
/// Convert from amps with [`encode_torque_current`].
pub fn encode_torque_raw(
    motor_id: u8,
    iq_control: i16,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let bytes = iq_control.to_le_bytes();
    encode(Command::TorqueClosedLoop.code(), motor_id, &bytes, out)
}

/// Convenience wrapper for [`encode_torque_raw`] that converts amps to the
/// raw int16 unit (`±33 A` → `±2048`). Saturates at the int16 limits.
pub fn encode_torque_current(
    motor_id: u8,
    current_amps: f32,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    const SCALE: f32 = 2048.0 / 33.0;
    // Manual round-half-away-from-zero: `core::f32` has no `round` (that's in `std`).
    let scaled = current_amps * SCALE;
    let bias = if scaled >= 0.0 { 0.5 } else { -0.5 };
    let rounded = scaled + bias;
    let clamped = rounded.clamp(i16::MIN as f32, i16::MAX as f32) as i16;
    encode_torque_raw(motor_id, clamped, out)
}

/// Closed-loop speed control (`0xA2`).
///
/// `speed_centideg_per_s` is the target speed in `0.01 deg/s` units (signed)
/// on the **motor shaft** (pre-gearbox). For geared motors, multiply the
/// desired output speed by the gear ratio before encoding.
pub fn encode_speed(
    motor_id: u8,
    speed_centideg_per_s: i32,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let bytes = speed_centideg_per_s.to_le_bytes();
    encode(Command::SpeedClosedLoop.code(), motor_id, &bytes, out)
}

/// Closed-loop multi-turn position (`0xA3`).
///
/// `position_centideg` is the target absolute angle in `0.01 deg` units (signed).
pub fn encode_position_multiturn(
    motor_id: u8,
    position_centideg: i64,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let bytes = position_centideg.to_le_bytes();
    encode(Command::PositionClosedLoop1.code(), motor_id, &bytes, out)
}

/// Closed-loop multi-turn position with max speed (`0xA4`).
///
/// `max_speed_centideg_per_s` is unsigned — the controller picks the sign of
/// motion from the difference between current and target position.
pub fn encode_position_multiturn_with_speed(
    motor_id: u8,
    position_centideg: i64,
    max_speed_centideg_per_s: u32,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let mut data = [0u8; 12];
    data[0..8].copy_from_slice(&position_centideg.to_le_bytes());
    data[8..12].copy_from_slice(&max_speed_centideg_per_s.to_le_bytes());
    encode(Command::PositionClosedLoop2.code(), motor_id, &data, out)
}

/// Single-turn position control 1 (`0xA5`). `counterclockwise`: `false`=CW,
/// `true`=CCW (per the manual's `spinDirection` byte). `angle_centideg` is
/// unsigned `0.01 deg/LSB` (0..=35999 for one revolution).
///
/// Reply data is shaped like [`crate::response::parse_state2_payload`]
/// (only the echoed command byte differs).
pub fn encode_position_singleturn(
    motor_id: u8,
    counterclockwise: bool,
    angle_centideg: u32,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let mut data = [0u8; 7];
    data[0] = counterclockwise as u8;
    data[3..7].copy_from_slice(&angle_centideg.to_le_bytes());
    encode(Command::PositionClosedLoop3.code(), motor_id, &data, out)
}

/// Single-turn position control 2 (`0xA6`) — as
/// [`encode_position_singleturn`] plus a max-speed cap (`1 dps/LSB`).
pub fn encode_position_singleturn_with_speed(
    motor_id: u8,
    counterclockwise: bool,
    max_speed_dps: u16,
    angle_centideg: u32,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let mut data = [0u8; 7];
    data[0] = counterclockwise as u8;
    data[1..3].copy_from_slice(&max_speed_dps.to_le_bytes());
    data[3..7].copy_from_slice(&angle_centideg.to_le_bytes());
    encode(Command::PositionClosedLoop4.code(), motor_id, &data, out)
}

/// Incremental position control 1 (`0xA7`). `angle_increment_centideg` is
/// signed `0.01 deg/LSB`; the sign selects the direction of motion.
pub fn encode_position_incremental(
    motor_id: u8,
    angle_increment_centideg: i32,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let mut data = [0u8; 7];
    data[3..7].copy_from_slice(&angle_increment_centideg.to_le_bytes());
    encode(Command::IncrementalPosition1.code(), motor_id, &data, out)
}

/// Incremental position control 2 (`0xA8`) — as [`encode_position_incremental`]
/// plus a max-speed cap.
///
/// **Note**: the manual's prose declares `maxSpeed` as `uint32_t`, but its own
/// frame layout only reserves 2 bytes (`DATA[2]`/`DATA[3]`) for it — an
/// internal inconsistency in the source (see
/// `doc/can-rs485-manual-analysis.md`). This encodes the effective 2-byte
/// wire field (`u16`), matching the documented byte layout over the prose.
pub fn encode_position_incremental_with_speed(
    motor_id: u8,
    max_speed_dps: u16,
    angle_increment_centideg: i32,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let mut data = [0u8; 7];
    data[1..3].copy_from_slice(&max_speed_dps.to_le_bytes());
    data[3..7].copy_from_slice(&angle_increment_centideg.to_le_bytes());
    encode(Command::IncrementalPosition2.code(), motor_id, &data, out)
}

/// Read multi-turn absolute angle (`0x92`). No payload.
pub fn encode_read_multi_turn_angle(motor_id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_simple(Command::ReadMultiTurnAngle, motor_id, out)
}

/// Read a control parameter (`0xC0`).
///
/// The request payload is `[param_id, 0, 0, 0, 0, 0, 0]` (7 bytes); the motor
/// echoes the same command code back with the value populated in `DATA[1..7]`.
pub fn encode_read_control_param(
    motor_id: u8,
    param_id: ControlParamId,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let mut data = [0u8; 7];
    data[0] = param_id.code();
    encode(Command::ReadControlParam.code(), motor_id, &data, out)
}

/// Read a setting parameter (`0x40`, "One Parameter Command" — see
/// [`SettingParamId`]).
///
/// The request payload is `[0x05, param_id, 0, 0, 0, 0, 0]` (7 bytes) — the
/// leading `0x05` is a fixed marker byte per the manual, present on every
/// row of the Setting Parameter Table. The motor echoes it back with the
/// value populated in `DATA[2..7]`.
pub fn encode_read_setting_param(
    motor_id: u8,
    param_id: SettingParamId,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let mut data = [0u8; 7];
    data[0] = 0x05;
    data[1] = param_id.code();
    encode(Command::ReadSettingParam.code(), motor_id, &data, out)
}

/// Write an 8-bit setting parameter (`0x42`) — [`SettingParamId::DriverId`],
/// [`SettingParamId::BusType`], [`SettingParamId::Rs485Baudrate`], or
/// [`SettingParamId::CanBaudrate`].
///
/// Takes effect in RAM immediately; call [`encode_save_setting_params`]
/// afterward for the change to survive a power cycle.
pub fn encode_write_setting_param_u8(
    motor_id: u8,
    param_id: SettingParamId,
    value: u8,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let mut data = [0u8; 7];
    data[0] = 0x05;
    data[1] = param_id.code();
    data[2] = value;
    encode(Command::WriteSettingParam.code(), motor_id, &data, out)
}

/// Write a 16-bit setting parameter (`0x42`) —
/// [`SettingParamId::MaxPower`] or [`SettingParamId::CurrentRamp`].
pub fn encode_write_setting_param_i16(
    motor_id: u8,
    param_id: SettingParamId,
    value: i16,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let mut data = [0u8; 7];
    data[0] = 0x05;
    data[1] = param_id.code();
    data[2..4].copy_from_slice(&value.to_le_bytes());
    encode(Command::WriteSettingParam.code(), motor_id, &data, out)
}

/// Write a 32-bit setting parameter (`0x42`) — [`SettingParamId::MaxSpeed`],
/// [`SettingParamId::MaxAngle`], or [`SettingParamId::SpeedRamp`].
pub fn encode_write_setting_param_i32(
    motor_id: u8,
    param_id: SettingParamId,
    value: i32,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let mut data = [0u8; 7];
    data[0] = 0x05;
    data[1] = param_id.code();
    data[2..6].copy_from_slice(&value.to_le_bytes());
    encode(Command::WriteSettingParam.code(), motor_id, &data, out)
}

/// Commit all `WriteSettingParam` RAM writes to ROM (`0x44`). Per the
/// manual: takes effect after [`encode_motor_restart`] or a power cycle.
pub fn encode_save_setting_params(motor_id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    let data = [0x05u8, 0xFA, 0, 0, 0, 0, 0];
    encode(Command::SaveSettingParam.code(), motor_id, &data, out)
}

/// Restart the motor (`0x07`), equivalent to a power cycle. No payload; the
/// motor does not reply.
pub fn encode_motor_restart(motor_id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_simple(Command::MotorRestart, motor_id, out)
}

/// Engage or release the brake (`0x8C`). `released=false` energizes the
/// brake circuit's de-energized state (brake engaged/holding);
/// `released=true` energizes it (brake released/free).
pub fn encode_brake_control(
    motor_id: u8,
    released: bool,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let mut data = [0u8; 7];
    data[0] = released as u8;
    encode(Command::BrakeControl.code(), motor_id, &data, out)
}

/// Read the current brake state (`0x8C` with the read-marker byte `0x10`).
pub fn encode_read_brake_state(motor_id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    let mut data = [0u8; 7];
    data[0] = 0x10;
    encode(Command::BrakeControl.code(), motor_id, &data, out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{MAX_FRAME, try_decode};

    #[test]
    fn motor_off_round_trip() {
        let mut buf = [0u8; MAX_FRAME];
        let n = encode_motor_off(0x01, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.command, Command::MotorOff.code());
        assert_eq!(frame.motor_id, 0x01);
        assert_eq!(frame.data, &[][..]);
    }

    #[test]
    fn speed_round_trip() {
        let mut buf = [0u8; MAX_FRAME];
        let n = encode_speed(0x02, -54_321, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.command, Command::SpeedClosedLoop.code());
        assert_eq!(frame.motor_id, 0x02);
        let recovered = i32::from_le_bytes(frame.data.try_into().unwrap());
        assert_eq!(recovered, -54_321);
    }

    #[test]
    fn position_multiturn_payload_layout() {
        let mut buf = [0u8; MAX_FRAME];
        let n = encode_position_multiturn(0x03, 12345, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.data.len(), 8);
        assert_eq!(i64::from_le_bytes(frame.data.try_into().unwrap()), 12345);
    }

    #[test]
    fn position_with_speed_payload_layout() {
        let mut buf = [0u8; MAX_FRAME];
        let n = encode_position_multiturn_with_speed(0x03, -100, 36000, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.data.len(), 12);
        assert_eq!(i64::from_le_bytes(frame.data[0..8].try_into().unwrap()), -100);
        assert_eq!(u32::from_le_bytes(frame.data[8..12].try_into().unwrap()), 36000);
    }

    #[test]
    fn read_control_param_payload() {
        let mut buf = [0u8; MAX_FRAME];
        let n =
            encode_read_control_param(0x01, ControlParamId::PositionLoopPid, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.command, Command::ReadControlParam.code());
        assert_eq!(frame.data.len(), 7);
        assert_eq!(frame.data[0], 0x0A);
        assert!(frame.data[1..7].iter().all(|&b| b == 0));
    }

    #[test]
    fn torque_current_clamps() {
        let mut buf = [0u8; MAX_FRAME];
        // Way over the saturation limit — should clamp to i16::MAX
        let n = encode_torque_current(0x01, 1_000.0, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        let raw = i16::from_le_bytes(frame.data.try_into().unwrap());
        assert_eq!(raw, i16::MAX);
    }

    #[test]
    fn read_setting_param_payload() {
        let mut buf = [0u8; MAX_FRAME];
        let n = encode_read_setting_param(0x01, SettingParamId::DriverId, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.command, Command::ReadSettingParam.code());
        assert_eq!(frame.data.len(), 7);
        assert_eq!(frame.data[0], 0x05);
        assert_eq!(frame.data[1], 0x0A);
        assert!(frame.data[2..7].iter().all(|&b| b == 0));
    }

    #[test]
    fn write_setting_param_u8_payload() {
        let mut buf = [0u8; MAX_FRAME];
        let n =
            encode_write_setting_param_u8(0x01, SettingParamId::DriverId, 7, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.command, Command::WriteSettingParam.code());
        assert_eq!(frame.data[0], 0x05);
        assert_eq!(frame.data[1], SettingParamId::DriverId.code());
        assert_eq!(frame.data[2], 7);
    }

    #[test]
    fn write_setting_param_i32_payload() {
        let mut buf = [0u8; MAX_FRAME];
        let n = encode_write_setting_param_i32(0x01, SettingParamId::MaxSpeed, 360_000, &mut buf)
            .unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.data[0], 0x05);
        assert_eq!(frame.data[1], SettingParamId::MaxSpeed.code());
        assert_eq!(i32::from_le_bytes(frame.data[2..6].try_into().unwrap()), 360_000);
    }

    #[test]
    fn save_setting_params_payload() {
        let mut buf = [0u8; MAX_FRAME];
        let n = encode_save_setting_params(0x01, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.command, Command::SaveSettingParam.code());
        assert_eq!(frame.data[0], 0x05);
        assert_eq!(frame.data[1], 0xFA);
    }

    #[test]
    fn position_singleturn_payload_layout() {
        let mut buf = [0u8; MAX_FRAME];
        let n = encode_position_singleturn(0x01, true, 18000, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.command, Command::PositionClosedLoop3.code());
        assert_eq!(frame.data[0], 1);
        assert_eq!(u32::from_le_bytes(frame.data[3..7].try_into().unwrap()), 18000);
    }

    #[test]
    fn position_singleturn_with_speed_payload_layout() {
        let mut buf = [0u8; MAX_FRAME];
        let n =
            encode_position_singleturn_with_speed(0x01, false, 360, 9000, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.command, Command::PositionClosedLoop4.code());
        assert_eq!(frame.data[0], 0);
        assert_eq!(u16::from_le_bytes(frame.data[1..3].try_into().unwrap()), 360);
        assert_eq!(u32::from_le_bytes(frame.data[3..7].try_into().unwrap()), 9000);
    }

    #[test]
    fn position_incremental_payload_layout() {
        let mut buf = [0u8; MAX_FRAME];
        let n = encode_position_incremental(0x01, -4500, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.command, Command::IncrementalPosition1.code());
        assert_eq!(i32::from_le_bytes(frame.data[3..7].try_into().unwrap()), -4500);
    }

    #[test]
    fn position_incremental_with_speed_payload_layout() {
        let mut buf = [0u8; MAX_FRAME];
        let n =
            encode_position_incremental_with_speed(0x01, 720, 4500, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.command, Command::IncrementalPosition2.code());
        assert_eq!(u16::from_le_bytes(frame.data[1..3].try_into().unwrap()), 720);
        assert_eq!(i32::from_le_bytes(frame.data[3..7].try_into().unwrap()), 4500);
    }

    #[test]
    fn motor_restart_round_trip() {
        let mut buf = [0u8; MAX_FRAME];
        let n = encode_motor_restart(0x01, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.command, Command::MotorRestart.code());
        assert_eq!(frame.data, &[][..]);
    }

    #[test]
    fn brake_control_payload() {
        let mut buf = [0u8; MAX_FRAME];
        let n = encode_brake_control(0x01, true, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.command, Command::BrakeControl.code());
        assert_eq!(frame.data[0], 0x01);
        assert!(frame.data[1..].iter().all(|&b| b == 0));

        let n = encode_brake_control(0x01, false, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.data[0], 0x00);
    }

    #[test]
    fn read_brake_state_payload() {
        let mut buf = [0u8; MAX_FRAME];
        let n = encode_read_brake_state(0x01, &mut buf).unwrap();
        let (frame, _) = try_decode(&buf[..n]).unwrap();
        assert_eq!(frame.command, Command::BrakeControl.code());
        assert_eq!(frame.data[0], 0x10);
    }
}
