//! Response parsers.
//!
//! Each parser takes the frame's `code` and `params` (checksum already
//! validated) and returns a typed view in raw wire units. Layouts are the
//! `struct.unpack` format strings of the matching `response_*` handler in
//! `uservo.py`, quoted on each type.

use crate::command::Code;

/// Errors returned by the parsers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    /// Frame code is not the one this parser handles.
    CodeMismatch { expected: u8, found: u8 },
    /// Params have the wrong length for the layout.
    BadLength { expected: usize, got: usize },
}

fn expect(code: Code, found: u8) -> Result<(), ParseError> {
    if code.code() == found {
        Ok(())
    } else {
        Err(ParseError::CodeMismatch {
            expected: code.code(),
            found,
        })
    }
}

fn expect_len(params: &[u8], expected: usize) -> Result<(), ParseError> {
    // Exact, not "at least": `struct.unpack` in the SDK raises on any other
    // length, so a longer payload is not a reply the SDK would accept either.
    if params.len() == expected {
        Ok(())
    } else {
        Err(ParseError::BadLength {
            expected,
            got: params.len(),
        })
    }
}

/// `PING` reply (`'<B'`): the responding id.
pub fn parse_ping(code: u8, params: &[u8]) -> Result<u8, ParseError> {
    expect(Code::Ping, code)?;
    expect_len(params, 1)?;
    Ok(params[0])
}

/// `QUERY_SERVO_ANGLE` reply (`'<Bh'`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AngleReading {
    pub id: u8,
    /// Single-turn angle, 0.1° per LSB (`angle /= 10` in the SDK).
    pub angle: i16,
}

/// Parse a `QUERY_SERVO_ANGLE` reply.
pub fn parse_angle(code: u8, params: &[u8]) -> Result<AngleReading, ParseError> {
    expect(Code::QueryAngle, code)?;
    expect_len(params, 3)?;
    Ok(AngleReading {
        id: params[0],
        angle: i16::from_le_bytes([params[1], params[2]]),
    })
}

/// `QUERY_SERVO_ANGLE_MTURN` reply (`'<Bih'`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MultiTurnAngle {
    pub id: u8,
    /// Multi-turn angle, 0.1° per LSB. This is the **whole** angle, turns
    /// included: the SDK uses it alone (`angle /= 10.0`) and never adds
    /// `turns * 360`, and the vendor teleop loop feeds it straight into a
    /// multi-turn set command.
    pub angle: i32,
    /// Turn counter. Informational — already folded into [`Self::angle`].
    pub turns: i16,
}

/// Parse a `QUERY_SERVO_ANGLE_MTURN` reply.
pub fn parse_angle_mturn(code: u8, params: &[u8]) -> Result<MultiTurnAngle, ParseError> {
    expect(Code::QueryAngleMturn, code)?;
    expect_len(params, 7)?;
    Ok(MultiTurnAngle {
        id: params[0],
        angle: i32::from_le_bytes([params[1], params[2], params[3], params[4]]),
        turns: i16::from_le_bytes([params[5], params[6]]),
    })
}

/// Params length of a `QUERY_SERVO_MONITOR` reply (`'<BHHHHBih'`); the full
/// frame is `MONITOR_PARAMS_LEN + 5 = 21` bytes — the constant the SDK's
/// `monitor_update` reads in (`bytes_waiting >= 21`).
pub const MONITOR_PARAMS_LEN: usize = 16;

/// Angle value the SDK treats as "no reading" when it comes with
/// `turns == 0` (`if angle == -235929599 and turn == 0: pass`).
///
/// As bytes it is `01 00 F0 F1` (`0xF1F0_0001`). Its origin is not
/// documented; the SDK discards such a frame instead of updating its cache,
/// and `monitor_update` does not count it as a response.
pub const MONITOR_INVALID_ANGLE: i32 = -235_929_599;

/// `QUERY_SERVO_MONITOR` reply (`'<BHHHHBih'`, handler
/// `response_query_servo_monitor`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Monitor {
    pub id: u8,
    /// Bus voltage, mV.
    pub voltage_mv: u16,
    /// Current, mA. Unsigned on the wire (`H`), so it carries no direction.
    pub current_ma: u16,
    /// Power, mW.
    pub power_mw: u16,
    /// Raw 12-bit NTC ADC count — **not** °C. The driver converts it with
    /// the SDK's formula (10 kΩ NTC, B = 3435).
    pub temp_raw: u16,
    /// Status byte, see [`crate::status_bits`].
    pub status: u8,
    /// Multi-turn angle, 0.1° per LSB, turns included (see
    /// [`MultiTurnAngle::angle`]).
    pub angle: i32,
    /// Turn counter, informational.
    pub turns: i16,
}

impl Monitor {
    /// `false` for the [`MONITOR_INVALID_ANGLE`] sentinel, which the SDK
    /// drops. The other fields of such a frame are not trustworthy either.
    #[inline]
    pub const fn is_valid(&self) -> bool {
        !(self.angle == MONITOR_INVALID_ANGLE && self.turns == 0)
    }
}

/// Parse a `QUERY_SERVO_MONITOR` reply. The sentinel is returned as-is; check
/// [`Monitor::is_valid`].
pub fn parse_monitor(code: u8, params: &[u8]) -> Result<Monitor, ParseError> {
    expect(Code::QueryMonitor, code)?;
    expect_len(params, MONITOR_PARAMS_LEN)?;
    let u16_at = |i: usize| u16::from_le_bytes([params[i], params[i + 1]]);
    Ok(Monitor {
        id: params[0],
        voltage_mv: u16_at(1),
        current_ma: u16_at(3),
        power_mw: u16_at(5),
        temp_raw: u16_at(7),
        status: params[9],
        angle: i32::from_le_bytes([params[10], params[11], params[12], params[13]]),
        turns: i16::from_le_bytes([params[14], params[15]]),
    })
}

/// `READ_DATA` reply (`'<BB'` + content).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataRead<'a> {
    pub id: u8,
    pub address: u8,
    /// Raw content; its layout depends on the address.
    pub content: &'a [u8],
}

/// Parse a `READ_DATA` reply.
pub fn parse_read_data(code: u8, params: &[u8]) -> Result<DataRead<'_>, ParseError> {
    expect(Code::ReadData, code)?;
    if params.len() < 2 {
        return Err(ParseError::BadLength {
            expected: 2,
            got: params.len(),
        });
    }
    Ok(DataRead {
        id: params[0],
        address: params[1],
        content: &params[2..],
    })
}

/// `WRITE_DATA` reply (`'<BBB'`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WriteAck {
    pub id: u8,
    pub address: u8,
    /// `result == 1` in the SDK.
    pub ok: bool,
}

/// Parse a `WRITE_DATA` reply.
pub fn parse_write_data(code: u8, params: &[u8]) -> Result<WriteAck, ParseError> {
    expect(Code::WriteData, code)?;
    expect_len(params, 3)?;
    Ok(WriteAck {
        id: params[0],
        address: params[1],
        ok: params[2] == 1,
    })
}

/// `RESET_USER_DATA` reply (`'<BB'`): `(id, result)`.
pub fn parse_reset_user_data(code: u8, params: &[u8]) -> Result<(u8, u8), ParseError> {
    expect(Code::ResetUserData, code)?;
    expect_len(params, 2)?;
    Ok((params[0], params[1]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{try_decode, HEADER_RESPONSE};

    #[test]
    fn angle_reply() {
        // pack('<Bh', 3, -1234): 03 | -1234 = 0xFB2E -> 2E FB
        //   05 1C 0A 03 03 2E FB | sum 05+1C+0A+03+03+2E+FB = 0x15A -> 0x5A
        let wire = [0x05, 0x1C, 0x0A, 0x03, 0x03, 0x2E, 0xFB, 0x5A];
        let (f, _) = try_decode(HEADER_RESPONSE, &wire).unwrap();
        let a = parse_angle(f.code, f.params).unwrap();
        assert_eq!(a, AngleReading { id: 3, angle: -1234 });
    }

    #[test]
    fn mturn_reply() {
        // pack('<Bih', 3, -36123, -1):
        //   03 | -36123 = 0xFFFF72E5 -> E5 72 FF FF | -1 -> FF FF ; checksum 0x8E
        let wire = [0x05, 0x1C, 0x10, 0x07, 0x03, 0xE5, 0x72, 0xFF, 0xFF, 0xFF, 0xFF, 0x8E];
        let (f, _) = try_decode(HEADER_RESPONSE, &wire).unwrap();
        let a = parse_angle_mturn(f.code, f.params).unwrap();
        assert_eq!(a, MultiTurnAngle { id: 3, angle: -36123, turns: -1 });
    }

    #[test]
    fn monitor_reply() {
        // pack('<BHHHHBih', 2, 12000, 150, 1800, 2048, 1, 1234, 0):
        //   02 | E0 2E | 96 00 | 08 07 | 00 08 | 01 | D2 04 00 00 | 00 00
        //   (12000=0x2EE0, 150=0x96, 1800=0x708, 2048=0x800, 1234=0x4D2)
        //   sum = 05+1C+16+10 + 02+E0+2E+96+08+07+08+01+D2+04 = 0x3DB -> 0xDB
        let wire = [
            0x05, 0x1C, 0x16, 0x10, 0x02, 0xE0, 0x2E, 0x96, 0x00, 0x08, 0x07, 0x00, 0x08, 0x01,
            0xD2, 0x04, 0x00, 0x00, 0x00, 0x00, 0xDB,
        ];
        assert_eq!(wire.len(), 21);
        let (f, used) = try_decode(HEADER_RESPONSE, &wire).unwrap();
        assert_eq!(used, 21);
        let m = parse_monitor(f.code, f.params).unwrap();
        assert_eq!(
            m,
            Monitor {
                id: 2,
                voltage_mv: 12000,
                current_ma: 150,
                power_mw: 1800,
                temp_raw: 2048,
                status: 1,
                angle: 1234,
                turns: 0,
            }
        );
        assert!(m.is_valid());
    }

    #[test]
    fn monitor_invalid_sentinel() {
        // angle -235929599 = 0xF1F00001 -> 01 00 F0 F1, turns 0 ; checksum 0xE8
        let wire = [
            0x05, 0x1C, 0x16, 0x10, 0x04, 0xE0, 0x2E, 0x96, 0x00, 0x08, 0x07, 0x00, 0x08, 0x00,
            0x01, 0x00, 0xF0, 0xF1, 0x00, 0x00, 0xE8,
        ];
        let (f, _) = try_decode(HEADER_RESPONSE, &wire).unwrap();
        let m = parse_monitor(f.code, f.params).unwrap();
        assert_eq!(m.angle, MONITOR_INVALID_ANGLE);
        assert!(!m.is_valid());
        // The sentinel only counts together with turns == 0.
        let m2 = Monitor { turns: 1, ..m };
        assert!(m2.is_valid());
    }

    #[test]
    fn data_replies() {
        // read_data voltage reply: 01 01 | E0 2E ; checksum 0x38
        let wire = [0x05, 0x1C, 0x03, 0x04, 0x01, 0x01, 0xE0, 0x2E, 0x38];
        let (f, _) = try_decode(HEADER_RESPONSE, &wire).unwrap();
        let r = parse_read_data(f.code, f.params).unwrap();
        assert_eq!((r.id, r.address, r.content), (1, 1, &[0xE0, 0x2E][..]));
        // write_data ack: pack('<BBB', 1, 33, 1) ; checksum 0x4B
        let wire = [0x05, 0x1C, 0x04, 0x03, 0x01, 0x21, 0x01, 0x4B];
        let (f, _) = try_decode(HEADER_RESPONSE, &wire).unwrap();
        assert_eq!(
            parse_write_data(f.code, f.params).unwrap(),
            WriteAck { id: 1, address: 33, ok: true }
        );
        assert_eq!(parse_reset_user_data(2, &[1, 1]).unwrap(), (1, 1));
    }

    #[test]
    fn mismatches() {
        assert_eq!(
            parse_ping(10, &[1]),
            Err(ParseError::CodeMismatch { expected: 1, found: 10 })
        );
        assert_eq!(
            parse_monitor(22, &[0; 15]),
            Err(ParseError::BadLength { expected: 16, got: 15 })
        );
        assert_eq!(parse_ping(1, &[5]), Ok(5));
    }
}
