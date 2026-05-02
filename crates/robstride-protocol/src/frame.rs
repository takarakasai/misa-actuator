//! Frame builders for outgoing Robstride commands.
//!
//! Every Robstride control frame is an extended (29-bit) CAN frame with an
//! 8-byte data payload. Each builder returns `(can_id, payload_bytes)` where
//! `payload_bytes` is a `[u8; 8]` ready to be written to the wire.

use crate::can_id::build_can_id;
use crate::comm_type::{CommType, RunMode};
use crate::mit::{encode_mit_signed, encode_mit_unsigned};
use crate::model::MitScales;
use crate::param::ParamIndex;

/// Length of the data payload for every Robstride frame.
pub const DATA_LEN: usize = 8;

/// Build a `GET_DEVICE_ID` (ping) frame. Used to probe whether a motor is
/// present on the bus.
pub fn build_ping_frame(host_id: u8, device_id: u8) -> (u32, [u8; DATA_LEN]) {
    let can_id = build_can_id(CommType::GetDeviceId, host_id as u16, device_id);
    (can_id, [0u8; DATA_LEN])
}

/// Build an `ENABLE` frame.
pub fn build_enable_frame(host_id: u8, device_id: u8) -> (u32, [u8; DATA_LEN]) {
    let can_id = build_can_id(CommType::Enable, host_id as u16, device_id);
    (can_id, [0u8; DATA_LEN])
}

/// Build a `DISABLE` frame.
pub fn build_disable_frame(host_id: u8, device_id: u8) -> (u32, [u8; DATA_LEN]) {
    let can_id = build_can_id(CommType::Disable, host_id as u16, device_id);
    (can_id, [0u8; DATA_LEN])
}

/// Build a `SET_ZERO_POSITION` frame. The first payload byte is `1` per spec.
pub fn build_set_zero_frame(host_id: u8, device_id: u8) -> (u32, [u8; DATA_LEN]) {
    let can_id = build_can_id(CommType::SetZeroPosition, host_id as u16, device_id);
    let mut data = [0u8; DATA_LEN];
    data[0] = 1;
    (can_id, data)
}

/// Build a MIT-mode `OPERATION_CONTROL` frame.
///
/// The torque feedforward rides in the 16-bit `extra_data` field of the CAN
/// ID, while the data payload carries `[pos, vel, kp, kd]` as big-endian u16
/// words.
pub fn build_mit_frame(
    device_id: u8,
    scales: &MitScales,
    position: f32,
    velocity: f32,
    kp: f32,
    kd: f32,
    torque: f32,
) -> (u32, [u8; DATA_LEN]) {
    let pos = encode_mit_signed(position, scales.position);
    let vel = encode_mit_signed(velocity, scales.velocity);
    let kp_u = encode_mit_unsigned(kp, scales.kp);
    let kd_u = encode_mit_unsigned(kd, scales.kd);
    let torque_u = encode_mit_signed(torque, scales.torque);

    let can_id = build_can_id(CommType::OperationControl, torque_u, device_id);
    let data = [
        (pos >> 8) as u8,
        (pos & 0xFF) as u8,
        (vel >> 8) as u8,
        (vel & 0xFF) as u8,
        (kp_u >> 8) as u8,
        (kp_u & 0xFF) as u8,
        (kd_u >> 8) as u8,
        (kd_u & 0xFF) as u8,
    ];
    (can_id, data)
}

/// Build a `READ_PARAMETER` frame.
pub fn build_read_param_frame(
    host_id: u8,
    device_id: u8,
    param: ParamIndex,
) -> (u32, [u8; DATA_LEN]) {
    let can_id = build_can_id(CommType::ReadParameter, host_id as u16, device_id);
    let idx = param.raw();
    let mut data = [0u8; DATA_LEN];
    data[0] = (idx & 0xFF) as u8;
    data[1] = (idx >> 8) as u8;
    (can_id, data)
}

/// Build a `WRITE_PARAMETER` frame with an `f32` value.
///
/// The value occupies bytes `[4..8]` little-endian; bytes `[2..4]` are reserved.
pub fn build_write_param_f32_frame(
    host_id: u8,
    device_id: u8,
    param: ParamIndex,
    value: f32,
) -> (u32, [u8; DATA_LEN]) {
    let can_id = build_can_id(CommType::WriteParameter, host_id as u16, device_id);
    let idx = param.raw();
    let val = value.to_le_bytes();
    let mut data = [0u8; DATA_LEN];
    data[0] = (idx & 0xFF) as u8;
    data[1] = (idx >> 8) as u8;
    data[4] = val[0];
    data[5] = val[1];
    data[6] = val[2];
    data[7] = val[3];
    (can_id, data)
}

/// Build a `WRITE_PARAMETER` frame with a single `i8` value (used for
/// `RunMode` and a few other one-byte parameters).
pub fn build_write_param_i8_frame(
    host_id: u8,
    device_id: u8,
    param: ParamIndex,
    value: i8,
) -> (u32, [u8; DATA_LEN]) {
    let can_id = build_can_id(CommType::WriteParameter, host_id as u16, device_id);
    let idx = param.raw();
    let mut data = [0u8; DATA_LEN];
    data[0] = (idx & 0xFF) as u8;
    data[1] = (idx >> 8) as u8;
    data[4] = value as u8;
    (can_id, data)
}

/// Convenience wrapper for selecting a [`RunMode`].
pub fn build_run_mode_frame(
    host_id: u8,
    device_id: u8,
    mode: RunMode,
) -> (u32, [u8; DATA_LEN]) {
    build_write_param_i8_frame(host_id, device_id, ParamIndex::RunMode, mode as i8)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::MotorModel;

    #[test]
    fn ping_payload_zeroed() {
        let (_id, data) = build_ping_frame(0xFD, 0x01);
        assert_eq!(data, [0u8; 8]);
    }

    #[test]
    fn set_zero_first_byte_is_one() {
        let (_id, data) = build_set_zero_frame(0xFD, 0x01);
        assert_eq!(data[0], 1);
        assert_eq!(&data[1..], &[0u8; 7]);
    }

    #[test]
    fn mit_zero_command_at_zero_point() {
        let scales = MitScales::for_model(MotorModel::Rs05);
        let (_id, data) = build_mit_frame(0x01, &scales, 0.0, 0.0, 0.0, 0.0, 0.0);
        // pos and vel both encode to 0x7FFF; kp/kd encode to 0
        assert_eq!(&data[0..2], &[0x7F, 0xFF]);
        assert_eq!(&data[2..4], &[0x7F, 0xFF]);
        assert_eq!(&data[4..6], &[0, 0]);
        assert_eq!(&data[6..8], &[0, 0]);
    }

    #[test]
    fn write_param_f32_layout() {
        let (_id, data) = build_write_param_f32_frame(0xFD, 0x01, ParamIndex::LocRef, 1.0);
        assert_eq!(data[0], (ParamIndex::LocRef as u16 & 0xFF) as u8);
        assert_eq!(data[1], (ParamIndex::LocRef as u16 >> 8) as u8);
        assert_eq!(&data[4..8], &1.0f32.to_le_bytes());
    }
}
