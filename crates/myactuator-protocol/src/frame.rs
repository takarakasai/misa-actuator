//! Single-motor command builders and reply parsers (`0x140 + ID` channel).
//!
//! Every builder returns the 8-byte payload; the caller pairs it with
//! [`crate::can_id::command_id`]. Replies echo the command byte in `data[0]`.

use crate::feedback::{
    ErrorState, PidIndex, RunMode, SingleTurnEncoder, Status1, Status2, Status3,
};
use crate::param::ParamIndex;

/// All V3 frames carry exactly 8 data bytes.
pub const DATA_LEN: usize = 8;

/// Which acceleration/deceleration value `0x42`/`0x43` reads/writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AccelIndex {
    /// Position planning: acceleration from initial to max velocity.
    PositionAccel = 0x00,
    /// Position planning: deceleration from max velocity to standstill.
    PositionDecel = 0x01,
    /// Speed planning: acceleration to the target speed.
    SpeedAccel = 0x02,
    /// Speed planning: deceleration to the target speed.
    SpeedDecel = 0x03,
}

/// Command bytes used by this crate (subset of the V3.9 manual).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Cmd {
    /// Read one current/speed/position-loop PID gain, selected by [`PidIndex`].
    ReadPid = 0x30,
    /// Read a position/speed-planning acceleration or deceleration value.
    ReadAcceleration = 0x42,
    /// Function control (compound; e.g. clear multi-turn value).
    FunctionControl = 0x20,
    /// Read the multi-turn encoder position (zero offset applied), pulses.
    ReadMultiTurnEncoder = 0x60,
    /// Read the raw multi-turn encoder position (no zero offset), pulses.
    ReadMultiTurnEncoderRaw = 0x61,
    /// Read the multi-turn encoder's zero-offset value, pulses.
    ReadMultiTurnZeroOffset = 0x62,
    /// Write current multi-turn position to ROM as zero (effective after reset).
    SetZeroRom = 0x64,
    /// Read the current run mode (current/speed/position loop).
    ReadRunMode = 0x70,
    /// Read the motor's instantaneous output power.
    ReadMotorPower = 0x71,
    /// System reset (restart the motor firmware).
    SystemReset = 0x76,
    /// Release the holding brake.
    BrakeRelease = 0x77,
    /// Lock the holding brake.
    BrakeLock = 0x78,
    /// Motor shutdown — turn off output and clear the running state.
    Shutdown = 0x80,
    /// Motor stop — halt motion but stay in closed-loop mode.
    Stop = 0x81,
    /// Read the single-turn encoder position (direct-drive models).
    ReadSingleTurnEncoder = 0x90,
    /// Read multi-turn absolute angle (0.01 °/LSB).
    ReadMultiTurnAngle = 0x92,
    /// Read the single-turn angle (0.01 °/LSB, wraps 0-359.99°).
    ReadSingleTurnAngle = 0x94,
    /// Read Status1 (temperature / voltage / error flags).
    ReadStatus1 = 0x9A,
    /// Read Status2 (temperature / iq / speed / angle).
    ReadStatus2 = 0x9C,
    /// Read Status3 (temperature / per-phase current).
    ReadStatus3 = 0x9D,
    /// Torque (current) closed-loop control.
    TorqueControl = 0xA1,
    /// Speed closed-loop control.
    SpeedControl = 0xA2,
    /// Absolute (multi-turn) position closed-loop control.
    PositionControl = 0xA4,
    /// Read system uptime since last reset/reboot, in ms.
    ReadUptime = 0xB1,
    /// Read the system software version date (YYYYMMDD).
    ReadVersionDate = 0xB2,
    /// Read the motor model name (ASCII).
    ReadMotorModel = 0xB5,
    /// Generic indexed parameter read/write, selected by [`crate::ParamIndex`]
    /// (undocumented — reverse-engineered from Setup Software V4.0 traffic,
    /// see `doc/setup-software-c0-param-protocol.md`).
    ReadWriteParam = 0xC0,
    /// Commit all `ReadWriteParam` RAM writes to flash (undocumented, same
    /// source as `ReadWriteParam`).
    CommitParams = 0xC1,
}

/// A command byte followed by 7 zero bytes — the shape of every "plain" frame.
const fn plain(cmd: Cmd) -> [u8; DATA_LEN] {
    [cmd as u8, 0, 0, 0, 0, 0, 0, 0]
}

/// `0x9A` — read temperature, voltage and error flags.
pub const fn build_read_status1() -> [u8; DATA_LEN] {
    plain(Cmd::ReadStatus1)
}

/// `0x9C` — read temperature, iq, output-shaft speed and angle.
pub const fn build_read_status2() -> [u8; DATA_LEN] {
    plain(Cmd::ReadStatus2)
}

/// `0x92` — read the multi-turn absolute angle (0.01 °/LSB).
pub const fn build_read_multi_turn_angle() -> [u8; DATA_LEN] {
    plain(Cmd::ReadMultiTurnAngle)
}

/// `0x30` — read one PID gain, selected by `index` (V4.2+ indexed protocol).
pub const fn build_read_pid(index: PidIndex) -> [u8; DATA_LEN] {
    [Cmd::ReadPid as u8, index as u8, 0, 0, 0, 0, 0, 0]
}

/// `0x60` — read the multi-turn encoder position (zero offset applied), pulses.
pub const fn build_read_multi_turn_encoder() -> [u8; DATA_LEN] {
    plain(Cmd::ReadMultiTurnEncoder)
}

/// `0x61` — read the raw multi-turn encoder position (no zero offset), pulses.
pub const fn build_read_multi_turn_encoder_raw() -> [u8; DATA_LEN] {
    plain(Cmd::ReadMultiTurnEncoderRaw)
}

/// `0x62` — read the multi-turn encoder's zero-offset value, pulses.
pub const fn build_read_multi_turn_zero_offset() -> [u8; DATA_LEN] {
    plain(Cmd::ReadMultiTurnZeroOffset)
}

/// `0x70` — read the current run mode.
pub const fn build_read_run_mode() -> [u8; DATA_LEN] {
    plain(Cmd::ReadRunMode)
}

/// `0x71` — read the motor's instantaneous output power (0.1 W/LSB).
pub const fn build_read_motor_power() -> [u8; DATA_LEN] {
    plain(Cmd::ReadMotorPower)
}

/// `0x90` — read the single-turn encoder position (direct-drive models).
pub const fn build_read_single_turn_encoder() -> [u8; DATA_LEN] {
    plain(Cmd::ReadSingleTurnEncoder)
}

/// `0x94` — read the single-turn angle (0.01 °/LSB, wraps 0-359.99°).
pub const fn build_read_single_turn_angle() -> [u8; DATA_LEN] {
    plain(Cmd::ReadSingleTurnAngle)
}

/// `0x9D` — read Status3 (temperature + per-phase current).
pub const fn build_read_status3() -> [u8; DATA_LEN] {
    plain(Cmd::ReadStatus3)
}

/// `0xB1` — read system uptime since last reset/reboot, in ms.
pub const fn build_read_uptime() -> [u8; DATA_LEN] {
    plain(Cmd::ReadUptime)
}

/// `0x42` — read one acceleration/deceleration value (1 dps/s, range
/// 100-60000 per the X4-36 V4.3 manual — the V3.9 manual's own text
/// disagrees with itself, citing 50 in one section and 100 in another).
pub const fn build_read_acceleration(index: AccelIndex) -> [u8; DATA_LEN] {
    [Cmd::ReadAcceleration as u8, index as u8, 0, 0, 0, 0, 0, 0]
}

/// `0xB2` — read the system software version date (YYYYMMDD, e.g. `20211126`).
pub const fn build_read_version_date() -> [u8; DATA_LEN] {
    plain(Cmd::ReadVersionDate)
}

/// `0xB5` — read the motor model name (ASCII, up to 7 characters).
pub const fn build_read_motor_model() -> [u8; DATA_LEN] {
    plain(Cmd::ReadMotorModel)
}

/// `0x80` — motor shutdown (output off, running state cleared).
pub const fn build_shutdown() -> [u8; DATA_LEN] {
    plain(Cmd::Shutdown)
}

/// `0x81` — motor stop (halt motion, stay in closed loop).
pub const fn build_stop() -> [u8; DATA_LEN] {
    plain(Cmd::Stop)
}

/// `0x76` — system reset. The motor drops off the bus while rebooting.
pub const fn build_system_reset() -> [u8; DATA_LEN] {
    plain(Cmd::SystemReset)
}

/// `0x77` — release the holding brake (brake-equipped models).
pub const fn build_brake_release() -> [u8; DATA_LEN] {
    plain(Cmd::BrakeRelease)
}

/// `0x78` — lock the holding brake (brake-equipped models).
pub const fn build_brake_lock() -> [u8; DATA_LEN] {
    plain(Cmd::BrakeLock)
}

/// `0x64` — write the current multi-turn position to ROM as the motor zero.
/// Takes effect only after a restart ([`build_system_reset`]); wears flash,
/// so prefer a driver-side soft zero for routine re-zeroing.
pub const fn build_set_zero_rom() -> [u8; DATA_LEN] {
    plain(Cmd::SetZeroRom)
}

/// `0x20` — function control (`index` selects the function, `value` is its
/// argument; e.g. index 1 = clear multi-turn value).
pub const fn build_function_control(index: u8, value: i32) -> [u8; DATA_LEN] {
    let v = value.to_le_bytes();
    [Cmd::FunctionControl as u8, index, 0, 0, v[0], v[1], v[2], v[3]]
}

/// `0xC0` (undocumented) — read one parameter, selected by [`ParamIndex`].
/// Wire layout: `[0xC0, 0x00, index, 0x01, 0, 0, 0, 0]` — byte 3 is a flag
/// (`0x01` = read, `0x00` = write, see [`build_write_param`]); the request's
/// value bytes are ignored on a read. See
/// `doc/setup-software-c0-param-protocol.md` §1.
pub const fn build_read_param(index: ParamIndex) -> [u8; DATA_LEN] {
    [Cmd::ReadWriteParam as u8, 0x00, index.code(), 0x01, 0, 0, 0, 0]
}

/// `0xC0` (undocumented) — write one parameter to RAM, selected by
/// [`ParamIndex`]. Takes effect immediately but is lost on power-cycle
/// unless followed by [`build_commit_params`]. See
/// `doc/setup-software-c0-param-protocol.md` §1, §1.5 for a caveat on
/// [`ParamIndex::EnableCanFilter`] specifically (write observed not to take
/// effect via this path).
pub fn build_write_param(index: ParamIndex, value: f32) -> [u8; DATA_LEN] {
    let v = value.to_le_bytes();
    [Cmd::ReadWriteParam as u8, 0x00, index.code(), 0x00, v[0], v[1], v[2], v[3]]
}

/// `0xC1` (undocumented) — commit all `ReadWriteParam` RAM writes to flash.
/// No payload. See `doc/setup-software-c0-param-protocol.md` §2.
pub const fn build_commit_params() -> [u8; DATA_LEN] {
    plain(Cmd::CommitParams)
}

/// `0xA1` — torque closed-loop control. `iq_centi_amps` is the target torque
/// current in 0.01 A/LSB. **Latches**: the motor keeps applying the current
/// until another command arrives.
pub const fn build_torque_control(iq_centi_amps: i16) -> [u8; DATA_LEN] {
    let iq = iq_centi_amps.to_le_bytes();
    [Cmd::TorqueControl as u8, 0, 0, 0, iq[0], iq[1], 0, 0]
}

/// `0xA2` — speed closed-loop control. `centi_dps` is the target output-shaft
/// speed in 0.01 dps/LSB.
pub const fn build_speed_control(centi_dps: i32) -> [u8; DATA_LEN] {
    let s = centi_dps.to_le_bytes();
    [Cmd::SpeedControl as u8, 0, 0, 0, s[0], s[1], s[2], s[3]]
}

/// `0xA4` — absolute (multi-turn) position closed-loop control.
/// `max_speed_dps` caps the output-shaft speed (1 dps/LSB, `0` = PI output
/// only); `centi_deg` is the target angle in 0.01 °/LSB.
pub const fn build_position_control(max_speed_dps: u16, centi_deg: i32) -> [u8; DATA_LEN] {
    let sp = max_speed_dps.to_le_bytes();
    let a = centi_deg.to_le_bytes();
    [
        Cmd::PositionControl as u8,
        0,
        sp[0],
        sp[1],
        a[0],
        a[1],
        a[2],
        a[3],
    ]
}

/// Parse a Status1 reply (`0x9A`). Returns `None` on wrong command byte or
/// short payload.
pub fn parse_status1(data: &[u8]) -> Option<Status1> {
    if data.len() < DATA_LEN || data[0] != Cmd::ReadStatus1 as u8 {
        return None;
    }
    Some(Status1 {
        temperature_c: data[1] as i8,
        mos_temperature_c: data[2] as i8,
        brake_released: data[3] != 0,
        voltage_dv: u16::from_le_bytes([data[4], data[5]]),
        error: ErrorState(u16::from_le_bytes([data[6], data[7]])),
    })
}

/// Parse a Status2-shaped reply. `0x9C` and every closed-loop control command
/// (`0xA1`/`0xA2`/`0xA4`) reply with the identical layout, so `data[0]` may be
/// any of those command bytes.
pub fn parse_status2(data: &[u8]) -> Option<Status2> {
    if data.len() < DATA_LEN {
        return None;
    }
    match data[0] {
        x if x == Cmd::ReadStatus2 as u8
            || x == Cmd::TorqueControl as u8
            || x == Cmd::SpeedControl as u8
            || x == Cmd::PositionControl as u8 => {}
        _ => return None,
    }
    Some(Status2 {
        temperature_c: data[1] as i8,
        iq_centi_amps: i16::from_le_bytes([data[2], data[3]]),
        speed_dps: i16::from_le_bytes([data[4], data[5]]),
        angle_deg: i16::from_le_bytes([data[6], data[7]]),
    })
}

/// Parse a multi-turn angle reply (`0x92`) into 0.01 °/LSB.
pub fn parse_multi_turn_angle(data: &[u8]) -> Option<i32> {
    if data.len() < DATA_LEN || data[0] != Cmd::ReadMultiTurnAngle as u8 {
        return None;
    }
    Some(i32::from_le_bytes([data[4], data[5], data[6], data[7]]))
}

/// Shared layout for `0x60`/`0x61`/`0x62`/`0xB1`: command echo, `data[1..4]`
/// `NULL`, a 32-bit little-endian value at `data[4..8]`.
fn parse_i32_at_4(data: &[u8], cmd: Cmd) -> Option<i32> {
    if data.len() < DATA_LEN || data[0] != cmd as u8 {
        return None;
    }
    Some(i32::from_le_bytes([data[4], data[5], data[6], data[7]]))
}

/// Parse a multi-turn encoder reply (`0x60`) into pulses (zero offset applied).
pub fn parse_multi_turn_encoder(data: &[u8]) -> Option<i32> {
    parse_i32_at_4(data, Cmd::ReadMultiTurnEncoder)
}

/// Parse a raw multi-turn encoder reply (`0x61`) into pulses (no zero offset).
pub fn parse_multi_turn_encoder_raw(data: &[u8]) -> Option<i32> {
    parse_i32_at_4(data, Cmd::ReadMultiTurnEncoderRaw)
}

/// Parse a multi-turn zero-offset reply (`0x62`) into pulses.
pub fn parse_multi_turn_zero_offset(data: &[u8]) -> Option<i32> {
    parse_i32_at_4(data, Cmd::ReadMultiTurnZeroOffset)
}

/// Parse a system-uptime reply (`0xB1`) into milliseconds since last reset.
pub fn parse_uptime(data: &[u8]) -> Option<u32> {
    parse_i32_at_4(data, Cmd::ReadUptime).map(|v| v as u32)
}

/// Parse a run-mode reply (`0x70`).
pub fn parse_run_mode(data: &[u8]) -> Option<RunMode> {
    if data.len() < DATA_LEN || data[0] != Cmd::ReadRunMode as u8 {
        return None;
    }
    Some(RunMode::from_raw(data[7]))
}

/// Parse a motor-power reply (`0x71`) into watts.
pub fn parse_motor_power(data: &[u8]) -> Option<f32> {
    if data.len() < DATA_LEN || data[0] != Cmd::ReadMotorPower as u8 {
        return None;
    }
    Some(u16::from_le_bytes([data[6], data[7]]) as f32 * 0.1)
}

/// Parse a single-turn encoder reply (`0x90`).
pub fn parse_single_turn_encoder(data: &[u8]) -> Option<SingleTurnEncoder> {
    if data.len() < DATA_LEN || data[0] != Cmd::ReadSingleTurnEncoder as u8 {
        return None;
    }
    Some(SingleTurnEncoder {
        encoder: i16::from_le_bytes([data[2], data[3]]),
        encoder_raw: i16::from_le_bytes([data[4], data[5]]),
        encoder_offset: i16::from_le_bytes([data[6], data[7]]),
    })
}

/// Parse a single-turn angle reply (`0x94`) into 0.01°/LSB (0-35999). The
/// manual labels this field `int16_t` but documents a 0-35999 range, which
/// overflows `i16` — decoded as `u16` per the stated range, not the stated type.
pub fn parse_single_turn_angle(data: &[u8]) -> Option<u16> {
    if data.len() < DATA_LEN || data[0] != Cmd::ReadSingleTurnAngle as u8 {
        return None;
    }
    Some(u16::from_le_bytes([data[6], data[7]]))
}

/// Parse a Status3 reply (`0x9D`): temperature + per-phase current.
pub fn parse_status3(data: &[u8]) -> Option<Status3> {
    if data.len() < DATA_LEN || data[0] != Cmd::ReadStatus3 as u8 {
        return None;
    }
    Some(Status3 {
        temperature_c: data[1] as i8,
        phase_a_centi_amps: i16::from_le_bytes([data[2], data[3]]),
        phase_b_centi_amps: i16::from_le_bytes([data[4], data[5]]),
        phase_c_centi_amps: i16::from_le_bytes([data[6], data[7]]),
    })
}

/// Parse a single PID-gain reply (`0x30`) into its raw `Float` value.
/// `data[1]` echoes the requested [`PidIndex`] but isn't checked here —
/// callers issuing concurrent requests for different indices should verify
/// it matches.
pub fn parse_pid_value(data: &[u8]) -> Option<f32> {
    if data.len() < DATA_LEN || data[0] != Cmd::ReadPid as u8 {
        return None;
    }
    Some(f32::from_le_bytes([data[4], data[5], data[6], data[7]]))
}

/// Parse an acceleration reply (`0x42`) into 1 dps/s units. `data[1]` echoes
/// the requested [`AccelIndex`] on real X4-36 firmware (confirmed on
/// hardware), though the manual's own worked example shows it as always
/// `0x00` — not checked here either way, so callers issuing concurrent
/// requests for different indices should verify it matches if they rely on
/// manual-only firmware.
pub fn parse_acceleration(data: &[u8]) -> Option<i32> {
    if data.len() < DATA_LEN || data[0] != Cmd::ReadAcceleration as u8 {
        return None;
    }
    Some(i32::from_le_bytes([data[4], data[5], data[6], data[7]]))
}

/// Parse a version-date reply (`0xB2`) into a `YYYYMMDD` integer (e.g.
/// `20211126`).
pub fn parse_version_date(data: &[u8]) -> Option<u32> {
    if data.len() < DATA_LEN || data[0] != Cmd::ReadVersionDate as u8 {
        return None;
    }
    Some(u32::from_le_bytes([data[4], data[5], data[6], data[7]]))
}

/// Parse a `ReadWriteParam` reply (`0xC0`, undocumented) into its `f32`
/// value. `data[2]` echoes the requested [`ParamIndex`] but isn't checked
/// here — callers issuing concurrent requests for different indices should
/// verify it matches. Used for both read replies (value = current setting)
/// and write replies (value = an ack whose meaning is not understood, see
/// `doc/setup-software-c0-param-protocol.md` §1 — don't rely on it to judge
/// write success).
pub fn parse_param_value(data: &[u8]) -> Option<f32> {
    if data.len() < DATA_LEN || data[0] != Cmd::ReadWriteParam as u8 {
        return None;
    }
    Some(f32::from_le_bytes([data[4], data[5], data[6], data[7]]))
}

/// Parse a motor-model reply (`0xB5`) into its raw ASCII bytes (`data[1..8]`,
/// up to 7 characters; unused trailing bytes are typically `0x00`).
pub fn parse_motor_model(data: &[u8]) -> Option<[u8; 7]> {
    if data.len() < DATA_LEN || data[0] != Cmd::ReadMotorModel as u8 {
        return None;
    }
    let mut model = [0u8; 7];
    model.copy_from_slice(&data[1..8]);
    Some(model)
}

#[cfg(test)]
mod new_command_tests {
    use super::*;

    #[test]
    fn multi_turn_encoder_reply_matches_manual_example() {
        // X4-36 V4.3 §2.6 Example 1: reply 60 00 00 00 10 27 00 00 → 10000 pulses.
        let data = [0x60, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00];
        assert_eq!(parse_multi_turn_encoder(&data), Some(10_000));
        assert_eq!(parse_multi_turn_encoder_raw(&data), None);
    }

    #[test]
    fn multi_turn_encoder_raw_reply_matches_manual_example() {
        // X4-36 V4.3 §2.7 Example 1: reply 61 00 00 00 10 27 00 00 → 10000 pulses.
        let data = [0x61, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00];
        assert_eq!(parse_multi_turn_encoder_raw(&data), Some(10_000));
    }

    #[test]
    fn multi_turn_zero_offset_reply_matches_manual_example() {
        // X4-36 V4.3 §2.8 Example 1: reply 62 00 00 00 10 27 00 00 → 10000 pulses.
        let data = [0x62, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00];
        assert_eq!(parse_multi_turn_zero_offset(&data), Some(10_000));
    }

    #[test]
    fn uptime_reply_matches_manual_example() {
        // X4-36 V4.3 §2.29 Example 1: reply B1 00 00 00 00 00 00 10 → 268435456 ms.
        let data = [0xB1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10];
        assert_eq!(parse_uptime(&data), Some(268_435_456));
    }

    #[test]
    fn run_mode_reply_matches_manual_example() {
        // X4-36 V4.3 §2.25 Example 1: reply 70 00 00 00 00 00 00 03 → position loop.
        let data = [0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03];
        assert_eq!(parse_run_mode(&data), Some(RunMode::PositionLoop));
    }

    #[test]
    fn run_mode_unknown_value_is_preserved() {
        let data = [0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09];
        assert_eq!(parse_run_mode(&data), Some(RunMode::Unknown(0x09)));
    }

    #[test]
    fn motor_power_reply_matches_manual_example() {
        // V3.9 §2.25.4 Example 1: reply 71 .. D0 07 → 0x07D0=2000 → 200.0 W.
        let data = [0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0x07];
        assert_eq!(parse_motor_power(&data), Some(200.0));
    }

    #[test]
    fn single_turn_encoder_reply_matches_manual_example() {
        // X4-36 V4.3 §2.11 Example 1: reply 90 00 33 08 BE 2C 8B 24.
        let data = [0x90, 0x00, 0x33, 0x08, 0xBE, 0x2C, 0x8B, 0x24];
        let e = parse_single_turn_encoder(&data).unwrap();
        assert_eq!(e.encoder, 2099);
        assert_eq!(e.encoder_raw, 11454);
        assert_eq!(e.encoder_offset, 9355);
    }

    #[test]
    fn single_turn_angle_reply_matches_manual_example() {
        // X4-36 V4.3 §2.13 Example 1: reply 94 00 00 00 00 00 10 27 → 10000 → 100.00°.
        let data = [0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27];
        assert_eq!(parse_single_turn_angle(&data), Some(10_000));
    }

    #[test]
    fn status3_reply_matches_manual_example() {
        // X4-36 V4.3 §2.16 Example 1: reply 9D 32 C2 0B 10 FA C0 F9.
        let data = [0x9D, 0x32, 0xC2, 0x0B, 0x10, 0xFA, 0xC0, 0xF9];
        let s = parse_status3(&data).unwrap();
        assert_eq!(s.temperature_c, 50);
        assert_eq!(s.phase_a_centi_amps, 3010);
        assert_eq!(s.phase_b_centi_amps, -1520);
        assert_eq!(s.phase_c_centi_amps, -1600);
    }

    #[test]
    fn status1_decodes_mos_temperature() {
        // X4-36 V4.3 §2.14 Example 1: reply 9A 32 00 01 E5 01 04 00.
        let data = [0x9A, 0x32, 0x00, 0x01, 0xE5, 0x01, 0x04, 0x00];
        let s = parse_status1(&data).unwrap();
        assert_eq!(s.temperature_c, 50);
        assert_eq!(s.mos_temperature_c, 0);
    }

    // `0xC0`/`0xC1` (undocumented, no manual — reverse-engineered from a live
    // capture of Setup Software V4.0 against a real X4-36, FW `2026042402`).
    // See `doc/setup-software-c0-param-protocol.md`. Byte arrays below are
    // taken verbatim from that capture.

    #[test]
    fn read_param_request_layout_matches_capture() {
        // Over Voltage read request: C0 00 13 01 00 00 00 00.
        assert_eq!(
            build_read_param(ParamIndex::OverVoltage),
            [0xC0, 0x00, 0x13, 0x01, 0x00, 0x00, 0x00, 0x00]
        );
    }

    #[test]
    fn read_param_reply_matches_capture() {
        // Over Voltage read reply: C0 00 13 01 00 00 5C 42 -> 55.0.
        let data = [0xC0, 0x00, 0x13, 0x01, 0x00, 0x00, 0x5C, 0x42];
        assert_eq!(parse_param_value(&data), Some(55.0));
    }

    #[test]
    fn write_param_request_layout_matches_capture() {
        // Brake Mode write (Resistor = 1.0): C0 00 19 00 00 00 80 3F.
        assert_eq!(
            build_write_param(ParamIndex::BrakeMode, 1.0),
            [0xC0, 0x00, 0x19, 0x00, 0x00, 0x00, 0x80, 0x3F]
        );
        // ... and back to E-Brake = 0.0: C0 00 19 00 00 00 00 00.
        assert_eq!(
            build_write_param(ParamIndex::BrakeMode, 0.0),
            [0xC0, 0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00]
        );
    }

    #[test]
    fn commit_params_request_matches_capture() {
        // C1 00 00 00 00 00 00 00.
        assert_eq!(build_commit_params(), [0xC1, 0, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn param_value_wrong_command_byte_is_rejected() {
        let data = [0xB2, 0x00, 0x13, 0x01, 0x00, 0x00, 0x5C, 0x42];
        assert_eq!(parse_param_value(&data), None);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn torque_frame_matches_manual_example() {
        // Manual 2.19.4: iqControl = 100 (1 A) → A1 00 00 00 64 00 00 00.
        assert_eq!(
            build_torque_control(100),
            [0xA1, 0, 0, 0, 0x64, 0x00, 0, 0]
        );
    }

    #[test]
    fn speed_frame_matches_manual_example() {
        // Manual 2.20.4: speedControl = 10000 (100 dps) → A2 .. 10 27 00 00.
        assert_eq!(
            build_speed_control(10_000),
            [0xA2, 0, 0, 0, 0x10, 0x27, 0x00, 0x00]
        );
    }

    #[test]
    fn read_pid_frame_carries_index() {
        // X4-36 V4.3 manual §2.1.4 Example 1: index 0x01 (current KP).
        assert_eq!(
            build_read_pid(PidIndex::CurrentKp),
            [0x30, 0x01, 0, 0, 0, 0, 0, 0]
        );
    }

    #[test]
    fn pid_value_reply_matches_manual_example() {
        // X4-36 V4.3 manual §2.1.4 Example 1: reply 30 01 00 00 00 00 80 3F
        // → float 0x3F800000 = 1.0 (current loop KP).
        let value = parse_pid_value(&[0x30, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F]).unwrap();
        assert!((value - 1.0).abs() < 1e-6);
    }

    #[test]
    fn read_acceleration_frame_carries_index() {
        assert_eq!(
            build_read_acceleration(AccelIndex::PositionAccel),
            [0x42, 0x00, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            build_read_acceleration(AccelIndex::PositionDecel),
            [0x42, 0x01, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            build_read_acceleration(AccelIndex::SpeedAccel),
            [0x42, 0x02, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            build_read_acceleration(AccelIndex::SpeedDecel),
            [0x42, 0x03, 0, 0, 0, 0, 0, 0]
        );
    }

    #[test]
    fn acceleration_reply_matches_manual_example() {
        // Manual 2.4.5 Example 1: position-accel reply 42 00 00 00 10 27 00 00
        // → 0x00002710 = 10000 dps/s.
        let accel = parse_acceleration(&[0x42, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00]).unwrap();
        assert_eq!(accel, 10_000);
    }

    #[test]
    fn position_frame_matches_manual_example() {
        // Manual 2.21.4: maxSpeed = 500 dps, angleControl = 36000 (360°)
        // → A4 00 F4 01 A0 8C 00 00.
        assert_eq!(
            build_position_control(500, 36_000),
            [0xA4, 0x00, 0xF4, 0x01, 0xA0, 0x8C, 0x00, 0x00]
        );
    }

    #[test]
    fn status2_round_trip() {
        // temp 25 °C, iq 1.00 A, speed -90 dps, angle 180°.
        let mut data = [0u8; DATA_LEN];
        data[0] = 0x9C;
        data[1] = 25;
        data[2..4].copy_from_slice(&100i16.to_le_bytes());
        data[4..6].copy_from_slice(&(-90i16).to_le_bytes());
        data[6..8].copy_from_slice(&180i16.to_le_bytes());
        let s = parse_status2(&data).unwrap();
        assert_eq!(s.temperature_c, 25);
        assert_eq!(s.iq_centi_amps, 100);
        assert!((s.current_a() - 1.0).abs() < 1e-6);
        assert_eq!(s.speed_dps, -90);
        assert_eq!(s.angle_deg, 180);
    }

    #[test]
    fn status2_accepts_control_reply_command_bytes() {
        for cmd in [0x9C, 0xA1, 0xA2, 0xA4] {
            let mut data = [0u8; DATA_LEN];
            data[0] = cmd;
            assert!(parse_status2(&data).is_some(), "cmd 0x{cmd:02X}");
        }
        let mut data = [0u8; DATA_LEN];
        data[0] = 0x92;
        assert!(parse_status2(&data).is_none());
    }

    #[test]
    fn status1_decodes_voltage_and_errors() {
        // 24.0 V = 240 dv, error = stall | overtemp.
        let mut data = [0u8; DATA_LEN];
        data[0] = 0x9A;
        data[1] = 40;
        data[3] = 1;
        data[4..6].copy_from_slice(&240u16.to_le_bytes());
        data[6..8].copy_from_slice(&0x1002u16.to_le_bytes());
        let s = parse_status1(&data).unwrap();
        assert_eq!(s.temperature_c, 40);
        assert!(s.brake_released);
        assert!((s.voltage_v() - 24.0).abs() < 1e-5);
        assert!(s.error.stall());
        assert!(s.error.motor_over_temperature());
        assert!(!s.error.over_current());
    }

    #[test]
    fn multi_turn_angle_negative() {
        // -180.00° = -18000 centideg.
        let mut data = [0u8; DATA_LEN];
        data[0] = 0x92;
        data[4..8].copy_from_slice(&(-18_000i32).to_le_bytes());
        assert_eq!(parse_multi_turn_angle(&data), Some(-18_000));
    }

    #[test]
    fn version_date_round_trip() {
        let mut data = [0u8; DATA_LEN];
        data[0] = 0xB2;
        data[4..8].copy_from_slice(&20_211_126u32.to_le_bytes());
        assert_eq!(parse_version_date(&data), Some(20_211_126));
    }

    #[test]
    fn motor_model_extracts_ascii_bytes() {
        let mut data = [0u8; DATA_LEN];
        data[0] = 0xB5;
        data[1..8].copy_from_slice(b"RMD-X4-");
        assert_eq!(parse_motor_model(&data), Some(*b"RMD-X4-"));
    }
}
