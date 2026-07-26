//! Single-motor command builders and reply parsers (`0x140 + ID` channel).
//!
//! Every builder returns the 8-byte payload; the caller pairs it with
//! [`crate::can_id::command_id`]. Replies echo the command byte in `data[0]`.

use crate::feedback::{ErrorState, PidGains, Status1, Status2};

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
    /// Read current/speed/position-loop PID gains (0-255 normalized units).
    ReadPid = 0x30,
    /// Read a position/speed-planning acceleration or deceleration value.
    ReadAcceleration = 0x42,
    /// Function control (compound; e.g. clear multi-turn value).
    FunctionControl = 0x20,
    /// Write current multi-turn position to ROM as zero (effective after reset).
    SetZeroRom = 0x64,
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
    /// Read multi-turn absolute angle (0.01 °/LSB).
    ReadMultiTurnAngle = 0x92,
    /// Read Status1 (temperature / voltage / error flags).
    ReadStatus1 = 0x9A,
    /// Read Status2 (temperature / iq / speed / angle).
    ReadStatus2 = 0x9C,
    /// Torque (current) closed-loop control.
    TorqueControl = 0xA1,
    /// Speed closed-loop control.
    SpeedControl = 0xA2,
    /// Absolute (multi-turn) position closed-loop control.
    PositionControl = 0xA4,
    /// Read the system software version date (YYYYMMDD).
    ReadVersionDate = 0xB2,
    /// Read the motor model name (ASCII).
    ReadMotorModel = 0xB5,
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

/// `0x30` — read current/speed/position-loop PID gains (all six at once).
pub const fn build_read_pid() -> [u8; DATA_LEN] {
    plain(Cmd::ReadPid)
}

/// `0x42` — read one acceleration/deceleration value (1 dps/s, range
/// 50-60000 per the manual).
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

/// Parse a PID-gains reply (`0x30`).
pub fn parse_pid_gains(data: &[u8]) -> Option<PidGains> {
    if data.len() < DATA_LEN || data[0] != Cmd::ReadPid as u8 {
        return None;
    }
    Some(PidGains {
        current_kp: data[2],
        current_ki: data[3],
        speed_kp: data[4],
        speed_ki: data[5],
        position_kp: data[6],
        position_ki: data[7],
    })
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
    fn read_pid_frame_is_plain() {
        assert_eq!(build_read_pid(), [0x30, 0, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn pid_gains_reply_matches_manual_example() {
        // Manual 2.1.4 Example 1: reply 30 00 55 19 55 19 55 19.
        let gains = parse_pid_gains(&[0x30, 0x00, 0x55, 0x19, 0x55, 0x19, 0x55, 0x19]).unwrap();
        assert_eq!(
            gains,
            PidGains {
                current_kp: 0x55,
                current_ki: 0x19,
                speed_kp: 0x55,
                speed_ki: 0x19,
                position_kp: 0x55,
                position_ki: 0x19,
            }
        );
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
