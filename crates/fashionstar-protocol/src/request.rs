//! Request encoders.
//!
//! Every helper writes a complete frame into a caller-supplied buffer and
//! returns the number of bytes used, so the same API works without `alloc`.
//! Arguments are raw wire units; the layout of each one is the `struct.pack`
//! format string quoted in its doc comment (from `uservo.py`).
//!
//! The SDK clamps some arguments on the host before packing (angles to
//! ±180° / ±1024 turns, accel/decel times to ≥ 20 ms, ...). This layer does
//! **not** — it encodes what it is given, so a test can put any value on the
//! wire. The driver applies the SDK's clamps.

use crate::command::{Code, StopMode, WheelMode};
use crate::frame::{
    checksum, encode, encoded_size, EncodeError, HEADER_REQUEST, MAX_PARAMS,
};

/// Encode a command whose only parameter is the servo id (`'<B'`).
fn encode_id_only(code: Code, id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode(code.code(), &[id], out)
}

/// `PING` (`'<B'`: id). The servo replies with its id.
pub fn encode_ping(id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_id_only(Code::Ping, id, out)
}

/// `RESET_USER_DATA` (`'<B'`: id) — resets the user table to factory
/// defaults. Reply: `[id, result]`.
pub fn encode_reset_user_data(id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_id_only(Code::ResetUserData, id, out)
}

/// `READ_DATA` (`'<BB'`: id, address).
pub fn encode_read_data(id: u8, address: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode(Code::ReadData.code(), &[id, address], out)
}

/// `WRITE_DATA` (`'<BB'` + content: id, address, raw content bytes).
pub fn encode_write_data(
    id: u8,
    address: u8,
    content: &[u8],
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let mut params = [0u8; MAX_PARAMS];
    let len = 2 + content.len();
    if len > MAX_PARAMS {
        return Err(EncodeError::ParamsTooLong { got: len });
    }
    params[0] = id;
    params[1] = address;
    params[2..len].copy_from_slice(content);
    encode(Code::WriteData.code(), &params[..len], out)
}

/// `SET_SPIN` (`'<BBHH'`: id, method, speed dps, value) — wheel mode.
///
/// `method = mode | 0x80` when clockwise (`set_wheel`). `value` is turns for
/// [`WheelMode::Turns`], ms for [`WheelMode::Timed`], ignored otherwise.
pub fn encode_set_wheel(
    id: u8,
    mode: WheelMode,
    clockwise: bool,
    speed_dps: u16,
    value: u16,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let method = if clockwise { mode as u8 | 0x80 } else { mode as u8 };
    let s = speed_dps.to_le_bytes();
    let v = value.to_le_bytes();
    encode(Code::SetSpin.code(), &[id, method, s[0], s[1], v[0], v[1]], out)
}

/// `SET_DAMPING` (`'<BH'`: id, power mW).
pub fn encode_set_damping(id: u8, power_mw: u16, out: &mut [u8]) -> Result<usize, EncodeError> {
    let p = power_mw.to_le_bytes();
    encode(Code::SetDamping.code(), &[id, p[0], p[1]], out)
}

/// `QUERY_SERVO_ANGLE` (`'<B'`: id). Reply: single-turn angle.
pub fn encode_query_angle(id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_id_only(Code::QueryAngle, id, out)
}

/// `QUERY_SERVO_ANGLE_MTURN` (`'<B'`: id). Reply: multi-turn angle + turns.
pub fn encode_query_angle_mturn(id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_id_only(Code::QueryAngleMturn, id, out)
}

/// `RESET_MULTI_TURN_ANGLE` (`'<B'`: id) — clears the turn counter so the
/// multi-turn angle folds back into a single turn. No reply.
pub fn encode_reset_multi_turn(id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_id_only(Code::ResetMultiTurn, id, out)
}

/// `BEGIN_ASYNC` (no params).
pub fn encode_begin_async(out: &mut [u8]) -> Result<usize, EncodeError> {
    encode(Code::BeginAsync.code(), &[], out)
}

/// `END_ASYNC` (`'<B'`: cancel) — `0` executes the buffered commands, `1`
/// discards them.
pub fn encode_end_async(cancel: bool, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode(Code::EndAsync.code(), &[cancel as u8], out)
}

/// `QUERY_SERVO_MONITOR` (`'<B'`: id).
pub fn encode_query_monitor(id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_id_only(Code::QueryMonitor, id, out)
}

/// `SET_ORIGIN_POINT` (`'<BB'`: id, 0).
///
/// **Writes non-volatile state**: the current position becomes the servo's
/// zero and survives a power cycle. Every angle recorded before it is in a
/// frame that no longer exists. The meaning of the trailing `0` is not
/// documented; the SDK always sends it.
pub fn encode_set_origin_point(id: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    encode(Code::SetOriginPoint.code(), &[id, 0], out)
}

/// `SET_STOP_ON_CONTROL` (`'<BBH'`: id, mode, power mW).
///
/// `power_mw` is the holding power for [`StopMode::Hold`] /
/// [`StopMode::Damping`]; the vendor script sends `0` with
/// [`StopMode::Release`]. `id` may be [`crate::BROADCAST_ID`].
pub fn encode_stop_on_control(
    id: u8,
    mode: StopMode,
    power_mw: u16,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let p = power_mw.to_le_bytes();
    encode(Code::StopOnControl.code(), &[id, mode.code(), p[0], p[1]], out)
}

/// `SYNC_COMMAND` carrying `QUERY_SERVO_MONITOR` for every id in `ids`.
///
/// Layout (`send_sync_servo_monitor`): `'<BBB'` = (22, 1, N) followed by one
/// byte per id — the "1" is the per-servo record length, here just the id.
///
/// Each listed servo then answers with its **own ordinary** 21-byte
/// `QUERY_SERVO_MONITOR` reply, so N ids produce N back-to-back frames.
pub fn encode_sync_monitor(ids: &[u8], out: &mut [u8]) -> Result<usize, EncodeError> {
    let plen = 3 + ids.len();
    if plen > MAX_PARAMS {
        return Err(EncodeError::ParamsTooLong { got: plen });
    }
    let mut params = [0u8; MAX_PARAMS];
    params[0] = Code::QueryMonitor.code();
    params[1] = 1;
    params[2] = ids.len() as u8;
    params[3..plen].copy_from_slice(ids);
    encode(Code::SyncCommand.code(), &params[..plen], out)
}

/// A position command that exists both as a single-servo frame and as a
/// per-servo record inside `SYNC_COMMAND`.
///
/// The record layout is identical in both cases — the SDK builds sync
/// records with the same `struct.pack` strings as the single commands (see
/// `SyncPositionControl_EX` and `stararm102_ro.py`), and the sync header
/// declares the record length (`send_sync_*`'s hard-coded
/// `total_data_length`), which must equal [`Self::PARAMS_LEN`].
pub trait AngleCommand {
    /// Code of the single-servo command.
    const CODE: Code;
    /// Bytes per record, servo id included.
    const PARAMS_LEN: usize;
    /// Write exactly [`Self::PARAMS_LEN`] bytes into `out`.
    fn write_params(&self, out: &mut [u8]);
}

/// Encode a single-servo position command.
pub fn encode_command<C: AngleCommand>(cmd: &C, out: &mut [u8]) -> Result<usize, EncodeError> {
    let mut params = [0u8; 16];
    cmd.write_params(&mut params[..C::PARAMS_LEN]);
    encode(C::CODE.code(), &params[..C::PARAMS_LEN], out)
}

/// Encode a `SYNC_COMMAND` carrying `cmds`, one record per servo.
///
/// Layout: `'<BBB'` = (C::CODE, C::PARAMS_LEN, N), then the N records.
/// Written straight into `out` rather than through a params buffer so a long
/// sync needs no second copy.
pub fn encode_sync<C: AngleCommand>(cmds: &[C], out: &mut [u8]) -> Result<usize, EncodeError> {
    let plen = 3 + cmds.len() * C::PARAMS_LEN;
    if plen > MAX_PARAMS {
        return Err(EncodeError::ParamsTooLong { got: plen });
    }
    let needed = encoded_size(plen);
    if out.len() < needed {
        return Err(EncodeError::BufferTooSmall {
            needed,
            got: out.len(),
        });
    }
    out[0] = HEADER_REQUEST[0];
    out[1] = HEADER_REQUEST[1];
    out[2] = Code::SyncCommand.code();
    out[3] = plen as u8;
    out[4] = C::CODE.code();
    out[5] = C::PARAMS_LEN as u8;
    out[6] = cmds.len() as u8;
    let mut at = 7;
    for c in cmds {
        c.write_params(&mut out[at..at + C::PARAMS_LEN]);
        at += C::PARAMS_LEN;
    }
    out[at] = checksum(&out[..at]);
    Ok(needed)
}

/// `SET_SERVO_ANGLE` (`'<BhHH'`: id, angle 0.1°, interval ms, power mW).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SetAngle {
    pub id: u8,
    pub angle: i16,
    pub interval_ms: u16,
    pub power_mw: u16,
}

impl AngleCommand for SetAngle {
    const CODE: Code = Code::SetAngle;
    const PARAMS_LEN: usize = 7;
    fn write_params(&self, out: &mut [u8]) {
        out[0] = self.id;
        out[1..3].copy_from_slice(&self.angle.to_le_bytes());
        out[3..5].copy_from_slice(&self.interval_ms.to_le_bytes());
        out[5..7].copy_from_slice(&self.power_mw.to_le_bytes());
    }
}

/// `SET_SERVO_ANGLE_BY_INTERVAL` (`'<BhHHHH'`: id, angle 0.1°, interval ms,
/// accel ms, decel ms, power mW).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AngleByInterval {
    pub id: u8,
    pub angle: i16,
    pub interval_ms: u16,
    pub t_acc_ms: u16,
    pub t_dec_ms: u16,
    pub power_mw: u16,
}

impl AngleCommand for AngleByInterval {
    const CODE: Code = Code::SetAngleByInterval;
    const PARAMS_LEN: usize = 11;
    fn write_params(&self, out: &mut [u8]) {
        out[0] = self.id;
        out[1..3].copy_from_slice(&self.angle.to_le_bytes());
        out[3..5].copy_from_slice(&self.interval_ms.to_le_bytes());
        out[5..7].copy_from_slice(&self.t_acc_ms.to_le_bytes());
        out[7..9].copy_from_slice(&self.t_dec_ms.to_le_bytes());
        out[9..11].copy_from_slice(&self.power_mw.to_le_bytes());
    }
}

/// `SET_SERVO_ANGLE_BY_VELOCITY` (`'<BhHHHH'`: id, angle 0.1°, velocity
/// 0.1 dps, accel ms, decel ms, power mW).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AngleByVelocity {
    pub id: u8,
    pub angle: i16,
    pub velocity_ddps: u16,
    pub t_acc_ms: u16,
    pub t_dec_ms: u16,
    pub power_mw: u16,
}

impl AngleCommand for AngleByVelocity {
    const CODE: Code = Code::SetAngleByVelocity;
    const PARAMS_LEN: usize = 11;
    fn write_params(&self, out: &mut [u8]) {
        out[0] = self.id;
        out[1..3].copy_from_slice(&self.angle.to_le_bytes());
        out[3..5].copy_from_slice(&self.velocity_ddps.to_le_bytes());
        out[5..7].copy_from_slice(&self.t_acc_ms.to_le_bytes());
        out[7..9].copy_from_slice(&self.t_dec_ms.to_le_bytes());
        out[9..11].copy_from_slice(&self.power_mw.to_le_bytes());
    }
}

/// `SET_SERVO_ANGLE_MTURN` (`'<BiIH'`: id, angle 0.1°, interval ms, power mW).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MturnAngle {
    pub id: u8,
    pub angle: i32,
    pub interval_ms: u32,
    pub power_mw: u16,
}

impl AngleCommand for MturnAngle {
    const CODE: Code = Code::SetAngleMturn;
    const PARAMS_LEN: usize = 11;
    fn write_params(&self, out: &mut [u8]) {
        out[0] = self.id;
        out[1..5].copy_from_slice(&self.angle.to_le_bytes());
        out[5..9].copy_from_slice(&self.interval_ms.to_le_bytes());
        out[9..11].copy_from_slice(&self.power_mw.to_le_bytes());
    }
}

/// `SET_SERVO_ANGLE_MTURN_BY_INTERVAL` (`'<BiIHHH'`: id, angle 0.1°,
/// interval ms, accel ms, decel ms, power mW).
///
/// The vendor teleop loop drives the follower with exactly this, in a sync
/// frame: `struct.pack("<BlLHHH", i, angle*10, 100, 50, 50, 0)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MturnAngleByInterval {
    pub id: u8,
    pub angle: i32,
    pub interval_ms: u32,
    pub t_acc_ms: u16,
    pub t_dec_ms: u16,
    pub power_mw: u16,
}

impl AngleCommand for MturnAngleByInterval {
    const CODE: Code = Code::SetAngleMturnByInterval;
    const PARAMS_LEN: usize = 15;
    fn write_params(&self, out: &mut [u8]) {
        out[0] = self.id;
        out[1..5].copy_from_slice(&self.angle.to_le_bytes());
        out[5..9].copy_from_slice(&self.interval_ms.to_le_bytes());
        out[9..11].copy_from_slice(&self.t_acc_ms.to_le_bytes());
        out[11..13].copy_from_slice(&self.t_dec_ms.to_le_bytes());
        out[13..15].copy_from_slice(&self.power_mw.to_le_bytes());
    }
}

/// `SET_SERVO_ANGLE_MTURN_BY_VELOCITY` (`'<BiHHHH'`: id, angle 0.1°,
/// velocity 0.1 dps, accel ms, decel ms, power mW).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MturnAngleByVelocity {
    pub id: u8,
    pub angle: i32,
    pub velocity_ddps: u16,
    pub t_acc_ms: u16,
    pub t_dec_ms: u16,
    pub power_mw: u16,
}

impl AngleCommand for MturnAngleByVelocity {
    const CODE: Code = Code::SetAngleMturnByVelocity;
    const PARAMS_LEN: usize = 13;
    fn write_params(&self, out: &mut [u8]) {
        out[0] = self.id;
        out[1..5].copy_from_slice(&self.angle.to_le_bytes());
        out[5..7].copy_from_slice(&self.velocity_ddps.to_le_bytes());
        out[7..9].copy_from_slice(&self.t_acc_ms.to_le_bytes());
        out[9..11].copy_from_slice(&self.t_dec_ms.to_le_bytes());
        out[11..13].copy_from_slice(&self.power_mw.to_le_bytes());
    }
}

#[cfg(test)]
mod tests {
    //! Every expected vector below was derived by hand from the SDK's
    //! `struct.pack` call (quoted), then confirmed by running the SDK's own
    //! `Packet.pack(code, params)` on the same arguments.
    use super::*;
    use crate::frame::MAX_FRAME;

    fn enc(f: impl FnOnce(&mut [u8]) -> Result<usize, EncodeError>) -> ([u8; MAX_FRAME], usize) {
        let mut buf = [0u8; MAX_FRAME];
        let n = f(&mut buf).unwrap();
        (buf, n)
    }

    #[test]
    fn ping_id6() {
        // pack('<B', 6) = 06; sum 12+4C+01+01+06 = 0x66
        let (b, n) = enc(|o| encode_ping(6, o));
        assert_eq!(&b[..n], &[0x12, 0x4C, 0x01, 0x01, 0x06, 0x66]);
    }

    #[test]
    fn query_angle_and_mturn() {
        // code 10: 12+4C+0A+01+03 = 0x6C
        let (b, n) = enc(|o| encode_query_angle(3, o));
        assert_eq!(&b[..n], &[0x12, 0x4C, 0x0A, 0x01, 0x03, 0x6C]);
        // code 16: 12+4C+10+01+03 = 0x72
        let (b, n) = enc(|o| encode_query_angle_mturn(3, o));
        assert_eq!(&b[..n], &[0x12, 0x4C, 0x10, 0x01, 0x03, 0x72]);
    }

    #[test]
    fn query_monitor() {
        // code 22 = 0x16: 12+4C+16+01+02 = 0x77
        let (b, n) = enc(|o| encode_query_monitor(2, o));
        assert_eq!(&b[..n], &[0x12, 0x4C, 0x16, 0x01, 0x02, 0x77]);
    }

    #[test]
    fn sync_monitor_seven() {
        // send_sync_servo_monitor([0..=6]):
        //   pack('<BBB', 22, 1, 7) = 16 01 07, then 00 01 02 03 04 05 06
        //   len = 3 + 7 = 10 = 0x0A
        //   sum = 12+4C+19+0A + 16+01+07 + (0+1+..+6 = 0x15) = 0xB4
        let (b, n) = enc(|o| encode_sync_monitor(&[0, 1, 2, 3, 4, 5, 6], o));
        assert_eq!(
            &b[..n],
            &[
                0x12, 0x4C, 0x19, 0x0A, 0x16, 0x01, 0x07, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
                0x06, 0xB4
            ]
        );
        assert_eq!(n, 15);
    }

    #[test]
    fn stop_on_control() {
        // stararm102_ro.py: stop_on_control_mode(0xff, 0x10, 0x00)
        //   pack('<BBH', 0xFF, 0x10, 0) = FF 10 00 00
        //   sum = 12+4C+18+04+FF+10 = 0x189 -> 0x89
        let (b, n) = enc(|o| encode_stop_on_control(0xFF, StopMode::Release, 0, o));
        assert_eq!(&b[..n], &[0x12, 0x4C, 0x18, 0x04, 0xFF, 0x10, 0x00, 0x00, 0x89]);
        // pack('<BBH', 1, 0x11, 500) = 01 11 F4 01
        //   sum = 12+4C+18+04+01+11+F4+01 = 0x181 -> 0x81
        let (b, n) = enc(|o| encode_stop_on_control(1, StopMode::Hold, 500, o));
        assert_eq!(&b[..n], &[0x12, 0x4C, 0x18, 0x04, 0x01, 0x11, 0xF4, 0x01, 0x81]);
    }

    #[test]
    fn set_angle() {
        // pack('<BhHH', 0, -900, 1000, 0):
        //   -900 = 0xFC7C -> 7C FC; 1000 = 0x03E8 -> E8 03; 0 -> 00 00
        //   sum = 12+4C+08+07+00+7C+FC+E8+03 = 0x2D0 -> 0xD0
        let cmd = SetAngle { id: 0, angle: -900, interval_ms: 1000, power_mw: 0 };
        let (b, n) = enc(|o| encode_command(&cmd, o));
        assert_eq!(
            &b[..n],
            &[0x12, 0x4C, 0x08, 0x07, 0x00, 0x7C, 0xFC, 0xE8, 0x03, 0x00, 0x00, 0xD0]
        );
    }

    #[test]
    fn set_angle_by_interval() {
        // pack('<BhHHHH', 1, 450, 500, 20, 20, 0):
        //   01 | C2 01 | F4 01 | 14 00 | 14 00 | 00 00
        //   sum = 12+4C+0B+0B+01+C2+01+F4+01+14+14 = 0x255 -> 0x55
        let cmd = AngleByInterval {
            id: 1,
            angle: 450,
            interval_ms: 500,
            t_acc_ms: 20,
            t_dec_ms: 20,
            power_mw: 0,
        };
        let (b, n) = enc(|o| encode_command(&cmd, o));
        assert_eq!(
            &b[..n],
            &[
                0x12, 0x4C, 0x0B, 0x0B, 0x01, 0xC2, 0x01, 0xF4, 0x01, 0x14, 0x00, 0x14, 0x00,
                0x00, 0x00, 0x55
            ]
        );
    }

    #[test]
    fn set_angle_by_velocity() {
        // pack('<BhHHHH', 1, 450, 1000, 20, 20, 0) with velocity in 0.1 dps
        //   01 | C2 01 | E8 03 | 14 00 | 14 00 | 00 00 ; checksum 0x4C
        let cmd = AngleByVelocity {
            id: 1,
            angle: 450,
            velocity_ddps: 1000,
            t_acc_ms: 20,
            t_dec_ms: 20,
            power_mw: 0,
        };
        let (b, n) = enc(|o| encode_command(&cmd, o));
        assert_eq!(
            &b[..n],
            &[
                0x12, 0x4C, 0x0C, 0x0B, 0x01, 0xC2, 0x01, 0xE8, 0x03, 0x14, 0x00, 0x14, 0x00,
                0x00, 0x00, 0x4C
            ]
        );
    }

    #[test]
    fn mturn_variants() {
        // pack('<BiIH', 2, -36000, 2000, 0):
        //   -36000 = 0xFFFF7360 -> 60 73 FF FF; 2000 = 0x7D0 -> D0 07 00 00
        //   sum = 12+4C+0D+0B+02+60+73+FF+FF+D0+07 = 0x420 -> 0x20
        let cmd = MturnAngle { id: 2, angle: -36000, interval_ms: 2000, power_mw: 0 };
        let (b, n) = enc(|o| encode_command(&cmd, o));
        assert_eq!(
            &b[..n],
            &[
                0x12, 0x4C, 0x0D, 0x0B, 0x02, 0x60, 0x73, 0xFF, 0xFF, 0xD0, 0x07, 0x00, 0x00,
                0x00, 0x00, 0x20
            ]
        );

        // pack('<BiIHHH', 2, 3600, 100, 50, 50, 0):
        //   02 | 10 0E 00 00 | 64 00 00 00 | 32 00 | 32 00 | 00 00
        //   sum = 12+4C+0E+0F+02+10+0E+64+32+32 = 0x163 -> 0x63
        let cmd = MturnAngleByInterval {
            id: 2,
            angle: 3600,
            interval_ms: 100,
            t_acc_ms: 50,
            t_dec_ms: 50,
            power_mw: 0,
        };
        let (b, n) = enc(|o| encode_command(&cmd, o));
        assert_eq!(
            &b[..n],
            &[
                0x12, 0x4C, 0x0E, 0x0F, 0x02, 0x10, 0x0E, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00,
                0x32, 0x00, 0x32, 0x00, 0x00, 0x00, 0x63
            ]
        );

        // pack('<BiHHHH', 2, 3600, 1000, 20, 20, 0) ; checksum 0xAD
        let cmd = MturnAngleByVelocity {
            id: 2,
            angle: 3600,
            velocity_ddps: 1000,
            t_acc_ms: 20,
            t_dec_ms: 20,
            power_mw: 0,
        };
        let (b, n) = enc(|o| encode_command(&cmd, o));
        assert_eq!(
            &b[..n],
            &[
                0x12, 0x4C, 0x0F, 0x0D, 0x02, 0x10, 0x0E, 0x00, 0x00, 0xE8, 0x03, 0x14, 0x00,
                0x14, 0x00, 0x00, 0x00, 0xAD
            ]
        );
    }

    #[test]
    fn sync_mturn_by_interval() {
        // send_sync_multiturnanglebyinterval(14, 2, [
        //     pack("<BlLHHH", 0,  100, 100, 50, 50, 0),
        //     pack("<BlLHHH", 1, -100, 100, 50, 50, 0)])
        //   header record: pack('<BBB', 14, 15, 2) = 0E 0F 02
        //   len = 3 + 2*15 = 33 = 0x21 ; SDK-computed checksum 0x45
        let cmds = [
            MturnAngleByInterval { id: 0, angle: 100, interval_ms: 100, t_acc_ms: 50, t_dec_ms: 50, power_mw: 0 },
            MturnAngleByInterval { id: 1, angle: -100, interval_ms: 100, t_acc_ms: 50, t_dec_ms: 50, power_mw: 0 },
        ];
        let (b, n) = enc(|o| encode_sync(&cmds, o));
        assert_eq!(
            &b[..n],
            &[
                0x12, 0x4C, 0x19, 0x21, 0x0E, 0x0F, 0x02, //
                0x00, 0x64, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x32, 0x00, 0x32, 0x00,
                0x00, 0x00, //
                0x01, 0x9C, 0xFF, 0xFF, 0xFF, 0x64, 0x00, 0x00, 0x00, 0x32, 0x00, 0x32, 0x00,
                0x00, 0x00, //
                0x45
            ]
        );
    }

    #[test]
    fn sync_record_lengths_match_sdk() {
        // The SDK hard-codes these in send_sync_*: 7, 11, 11, 11, 15, 13.
        assert_eq!(SetAngle::PARAMS_LEN, 7);
        assert_eq!(AngleByInterval::PARAMS_LEN, 11);
        assert_eq!(AngleByVelocity::PARAMS_LEN, 11);
        assert_eq!(MturnAngle::PARAMS_LEN, 11);
        assert_eq!(MturnAngleByInterval::PARAMS_LEN, 15);
        assert_eq!(MturnAngleByVelocity::PARAMS_LEN, 13);
    }

    #[test]
    fn sync_too_long() {
        let cmds = [MturnAngleByInterval { id: 0, angle: 0, interval_ms: 0, t_acc_ms: 0, t_dec_ms: 0, power_mw: 0 }; 17];
        let mut buf = [0u8; 512];
        // 3 + 17*15 = 258 > 255
        assert_eq!(encode_sync(&cmds, &mut buf), Err(EncodeError::ParamsTooLong { got: 258 }));
    }

    #[test]
    fn misc_commands() {
        // reset_multi_turn_angle(4): code 17=0x11; 12+4C+11+01+04 = 0x74
        let (b, n) = enc(|o| encode_reset_multi_turn(4, o));
        assert_eq!(&b[..n], &[0x12, 0x4C, 0x11, 0x01, 0x04, 0x74]);
        // set_origin_point(5): pack('<BB', 5, 0); 12+4C+17+02+05 = 0x7C
        let (b, n) = enc(|o| encode_set_origin_point(5, o));
        assert_eq!(&b[..n], &[0x12, 0x4C, 0x17, 0x02, 0x05, 0x00, 0x7C]);
        // read_data(1, 1): pack('<BB', 1, 1); 12+4C+03+02+01+01 = 0x65
        let (b, n) = enc(|o| encode_read_data(1, 1, o));
        assert_eq!(&b[..n], &[0x12, 0x4C, 0x03, 0x02, 0x01, 0x01, 0x65]);
        // write_data(1, 33, b'\x01'); 12+4C+04+03+01+21+01 = 0x88
        let (b, n) = enc(|o| encode_write_data(1, 33, &[1], o));
        assert_eq!(&b[..n], &[0x12, 0x4C, 0x04, 0x03, 0x01, 0x21, 0x01, 0x88]);
        // set_damping(1, 1000): pack('<BH') = 01 E8 03; checksum 0x56
        let (b, n) = enc(|o| encode_set_damping(1, 1000, o));
        assert_eq!(&b[..n], &[0x12, 0x4C, 0x09, 0x03, 0x01, 0xE8, 0x03, 0x56]);
        // set_wheel(1, NORMAL, 0, is_cw=True, 100): pack('<BBHH', 1, 0x81, 100, 0)
        let (b, n) = enc(|o| encode_set_wheel(1, WheelMode::Normal, true, 100, 0, o));
        assert_eq!(&b[..n], &[0x12, 0x4C, 0x07, 0x06, 0x01, 0x81, 0x64, 0x00, 0x00, 0x00, 0x51]);
        // reset_user_data(1): 12+4C+02+01+01 = 0x62
        let (b, n) = enc(|o| encode_reset_user_data(1, o));
        assert_eq!(&b[..n], &[0x12, 0x4C, 0x02, 0x01, 0x01, 0x62]);
        // end_async(0): 12+4C+13+01+00 = 0x72
        let (b, n) = enc(|o| encode_end_async(false, o));
        assert_eq!(&b[..n], &[0x12, 0x4C, 0x13, 0x01, 0x00, 0x72]);
        let (b, n) = enc(encode_begin_async);
        assert_eq!(&b[..n], &[0x12, 0x4C, 0x12, 0x00, 0x70]);
    }
}
