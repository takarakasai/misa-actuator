//! Parsers for the most common MG4005 response payloads.
//!
//! Each parser takes the **payload** of a decoded [`crate::Frame`] (the
//! `data` field, with checksums already validated) and returns a typed view.
//! Verify scaling against your firmware manual before relying on the SI
//! conversions — they follow the widely published V3 protocol.

use crate::command::{Command, ControlParamId};

/// Errors returned from the response parsers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    /// Payload is shorter than the response layout demands.
    TooShort { expected: usize, got: usize },
    /// Frame `command` does not match the expected response code.
    CommandMismatch { expected: u8, found: u8 },
    /// Response `paramID` does not match the one we asked for.
    ParamIdMismatch { expected: u8, found: u8 },
}

/// Decoded payload of a `0x9A` "motor state 1" response.
///
/// Wire layout (7 bytes, observed on MG/MS V3 firmware):
/// `temp[i8] | voltage_lo | voltage_hi | reserved[3] | error_state[u8]`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorState1 {
    /// Motor temperature in degrees Celsius.
    pub temperature_c: i8,
    /// Voltage in 0.01 V units (e.g. `2400` = 24.00 V) on current MG/MS V3
    /// firmware. Some legacy MG manuals list 0.1 V LSB — verify against the
    /// firmware revision if you see values 10× too large.
    pub voltage_centivolt: u16,
    /// Error flag bitfield (firmware-specific).
    pub error_state: u8,
}

impl MotorState1 {
    /// Voltage in volts.
    #[inline]
    pub fn voltage_v(self) -> f32 {
        f32::from(self.voltage_centivolt) * 0.01
    }
}

/// Parse the payload of a `ReadMotorState1` (`0x9A`) response.
pub fn parse_state1(command: u8, data: &[u8]) -> Result<MotorState1, ParseError> {
    expect_cmd(Command::ReadMotorState1, command)?;
    expect_len(data, 7)?;
    Ok(MotorState1 {
        temperature_c: data[0] as i8,
        voltage_centivolt: u16::from_le_bytes([data[1], data[2]]),
        // bytes 3-5: reserved
        error_state: data[6],
    })
}

/// Encoder period for State2 / ReadEncoder responses. The single-turn encoder
/// reading wraps at this value; physical encoders may have fewer effective
/// bits (e.g. 14) but the wire field spans the full `u16` range and rolls
/// over at `2^16`.
pub const ENCODER_PERIOD: u32 = 65_536;

/// Decoded payload of a `0x9C` "motor state 2" response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorState2 {
    /// Motor temperature in degrees Celsius.
    pub temperature_c: i8,
    /// Torque current as raw int16 (`±2048` ≈ `±33 A`).
    pub iq_raw: i16,
    /// Speed in degrees per second (signed) on the **motor shaft** (pre-gearbox).
    /// Divide by the gear ratio for output-shaft speed.
    pub speed_deg_per_s: i16,
    /// Raw single-turn encoder position on the motor shaft, wrapping at
    /// [`ENCODER_PERIOD`] (= 65536). Increment with wrap correction to track
    /// multi-turn motion.
    pub encoder_raw: u16,
}

impl MotorState2 {
    /// Approximate torque current in amps (`iq_raw * 33 / 2048`).
    #[inline]
    pub fn current_amps(self) -> f32 {
        f32::from(self.iq_raw) * (33.0 / 2048.0)
    }

    /// Encoder position as a fraction of one motor-shaft revolution (`0.0..1.0`).
    #[inline]
    pub fn encoder_fraction(self) -> f32 {
        f32::from(self.encoder_raw) / ENCODER_PERIOD as f32
    }
}

/// Parse the 7-byte State2 payload without checking the command code.
///
/// State2 layout is shared by the explicit `0x9C` read **and** by the reply
/// data of every motion-control command (`0xA1`/`0xA2`/`0xA3`/`0xA4`/...);
/// only the wrapping command byte differs. Use this when you need to recover
/// (iq, speed, encoder) from a position-control reply, for example.
pub fn parse_state2_payload(data: &[u8]) -> Result<MotorState2, ParseError> {
    expect_len(data, 7)?;
    Ok(MotorState2 {
        temperature_c: data[0] as i8,
        iq_raw: i16::from_le_bytes([data[1], data[2]]),
        speed_deg_per_s: i16::from_le_bytes([data[3], data[4]]),
        encoder_raw: u16::from_le_bytes([data[5], data[6]]),
    })
}

/// Parse the payload of a `ReadMotorState2` (`0x9C`) response.
pub fn parse_state2(command: u8, data: &[u8]) -> Result<MotorState2, ParseError> {
    expect_cmd(Command::ReadMotorState2, command)?;
    parse_state2_payload(data)
}

/// Parse the payload of a `ReadMultiTurnAngle` (`0x92`) response.
///
/// Returns the multi-turn absolute angle in 0.01°/LSB (motor-frame).
pub fn parse_multi_turn_angle(command: u8, data: &[u8]) -> Result<i64, ParseError> {
    expect_cmd(Command::ReadMultiTurnAngle, command)?;
    expect_len(data, 8)?;
    let bytes: [u8; 8] = data[0..8].try_into().unwrap();
    Ok(i64::from_le_bytes(bytes))
}

/// PID triple as exposed by the `0x0A`/`0x0B`/`0x0C` control parameters
/// (raw integer units, 0..=2000 per the spec).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PidTriple {
    pub kp: u16,
    pub ki: u16,
    pub kd: u16,
}

/// Parsed value of a `ReadControlParam` (`0xC0`) response. Variant is selected
/// by the [`ControlParamId`] echoed in `DATA[0]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlParamValue {
    /// `0x0A` / `0x0B` / `0x0C` PID triples.
    Pid(PidTriple),
    /// `0x1E` torque/current limit (raw int16).
    TorqueLimit(i16),
    /// `0x20` speed limit, 0.01 deg/s units.
    SpeedLimit(i32),
    /// `0x22` angle limit, 0.01 deg units.
    AngleLimit(i32),
    /// `0x24` current ramp.
    CurrentRamp(i32),
    /// `0x26` speed ramp, 1 dps/s units.
    SpeedRamp(i32),
}

/// Parse the payload of a `ReadControlParam` (`0xC0`) response.
///
/// `expected` is the param ID we asked for — used to validate that the motor
/// echoed the correct selector and to pick the correct decoding for `data[1..7]`.
pub fn parse_control_param(
    command: u8,
    data: &[u8],
    expected: ControlParamId,
) -> Result<ControlParamValue, ParseError> {
    expect_cmd(Command::ReadControlParam, command)?;
    expect_len(data, 7)?;
    if data[0] != expected.code() {
        return Err(ParseError::ParamIdMismatch {
            expected: expected.code(),
            found: data[0],
        });
    }
    let v = &data[1..7];
    Ok(match expected {
        ControlParamId::PositionLoopPid
        | ControlParamId::SpeedLoopPid
        | ControlParamId::CurrentLoopPid => ControlParamValue::Pid(PidTriple {
            kp: u16::from_le_bytes([v[0], v[1]]),
            ki: u16::from_le_bytes([v[2], v[3]]),
            kd: u16::from_le_bytes([v[4], v[5]]),
        }),
        // Spec layout for these is `[0, 0, b0, b1, ...]` — value lives at offset 2.
        ControlParamId::TorqueLimit => {
            ControlParamValue::TorqueLimit(i16::from_le_bytes([v[2], v[3]]))
        }
        ControlParamId::SpeedLimit => {
            ControlParamValue::SpeedLimit(i32::from_le_bytes([v[2], v[3], v[4], v[5]]))
        }
        ControlParamId::AngleLimit => {
            ControlParamValue::AngleLimit(i32::from_le_bytes([v[2], v[3], v[4], v[5]]))
        }
        ControlParamId::CurrentRamp => {
            ControlParamValue::CurrentRamp(i32::from_le_bytes([v[2], v[3], v[4], v[5]]))
        }
        ControlParamId::SpeedRamp => {
            ControlParamValue::SpeedRamp(i32::from_le_bytes([v[2], v[3], v[4], v[5]]))
        }
    })
}

#[inline]
fn expect_cmd(expected: Command, found: u8) -> Result<(), ParseError> {
    if expected.code() == found {
        Ok(())
    } else {
        Err(ParseError::CommandMismatch {
            expected: expected.code(),
            found,
        })
    }
}

#[inline]
fn expect_len(data: &[u8], expected: usize) -> Result<(), ParseError> {
    if data.len() >= expected {
        Ok(())
    } else {
        Err(ParseError::TooShort {
            expected,
            got: data.len(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state1_parses() {
        // temp=25°C, voltage=24.00V (2400 = 0x0960), error=0x07
        let payload = [25, 0x60, 0x09, 0, 0, 0, 0x07];
        let s = parse_state1(Command::ReadMotorState1.code(), &payload).unwrap();
        assert_eq!(s.temperature_c, 25);
        assert_eq!(s.voltage_centivolt, 2400);
        assert!((s.voltage_v() - 24.0).abs() < 1e-3);
        assert_eq!(s.error_state, 0x07);
    }

    #[test]
    fn state2_parses() {
        // temp=30, iq=1024 (≈16.5A), speed=-360 deg/s, encoder=8192
        let mut payload = [0u8; 7];
        payload[0] = 30;
        payload[1..3].copy_from_slice(&1024i16.to_le_bytes());
        payload[3..5].copy_from_slice(&(-360i16).to_le_bytes());
        payload[5..7].copy_from_slice(&8192u16.to_le_bytes());

        let s = parse_state2(Command::ReadMotorState2.code(), &payload).unwrap();
        assert_eq!(s.temperature_c, 30);
        assert_eq!(s.iq_raw, 1024);
        assert!((s.current_amps() - 16.5).abs() < 0.05);
        assert_eq!(s.speed_deg_per_s, -360);
        assert_eq!(s.encoder_raw, 8192);
        assert!((s.encoder_fraction() - 8192.0 / 65536.0).abs() < 1e-4);
    }

    #[test]
    fn state2_command_mismatch() {
        let payload = [0u8; 7];
        let err = parse_state2(0x9A, &payload).unwrap_err();
        assert!(matches!(err, ParseError::CommandMismatch { .. }));
    }

    #[test]
    fn state2_too_short() {
        let err = parse_state2(0x9C, &[0u8; 4]).unwrap_err();
        assert_eq!(err, ParseError::TooShort { expected: 7, got: 4 });
    }

    #[test]
    fn control_param_pid_parses() {
        // paramID=0x0A, kp=100 (0x0064), ki=20 (0x0014), kd=5 (0x0005)
        let payload = [0x0A, 0x64, 0x00, 0x14, 0x00, 0x05, 0x00];
        let v = parse_control_param(
            Command::ReadControlParam.code(),
            &payload,
            ControlParamId::PositionLoopPid,
        )
        .unwrap();
        assert_eq!(
            v,
            ControlParamValue::Pid(PidTriple {
                kp: 100,
                ki: 20,
                kd: 5,
            })
        );
    }

    #[test]
    fn control_param_speed_limit_parses() {
        // paramID=0x20, value=360000 = 0x57E40 (0x40, 0x7E, 0x05, 0x00)
        let payload = [0x20, 0x00, 0x00, 0x40, 0x7E, 0x05, 0x00];
        let v = parse_control_param(
            Command::ReadControlParam.code(),
            &payload,
            ControlParamId::SpeedLimit,
        )
        .unwrap();
        assert_eq!(v, ControlParamValue::SpeedLimit(360_000));
    }

    #[test]
    fn control_param_id_mismatch() {
        let payload = [0x0B, 0, 0, 0, 0, 0, 0];
        let err = parse_control_param(
            Command::ReadControlParam.code(),
            &payload,
            ControlParamId::PositionLoopPid,
        )
        .unwrap_err();
        assert!(matches!(err, ParseError::ParamIdMismatch { .. }));
    }
}
