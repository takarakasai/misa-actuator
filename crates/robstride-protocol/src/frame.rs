//! Frame builders for outgoing Robstride commands.
//!
//! Every Robstride control frame is an extended (29-bit) CAN frame with an
//! 8-byte data payload. Each builder returns `(can_id, payload_bytes)` where
//! `payload_bytes` is a `[u8; 8]` ready to be written to the wire.

use crate::can_id::{build_can_id, parse_can_id};
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

/// Payload prefix that marks a version-read request (manual: communication
/// type 26). Also what tells such a request apart from a `DISABLE`.
pub const VERSION_REQUEST_PREFIX: [u8; 2] = [0x00, 0xC4];

/// Payload prefix on a version-read reply, which is otherwise shaped exactly
/// like an ordinary feedback frame.
pub const VERSION_REPLY_PREFIX: [u8; 3] = [0x00, 0xC4, 0x56];

/// Build a version-read frame (manual: communication type 26).
///
/// The comm-type field carries **4**, not 26 — the same value as `DISABLE`.
/// That reads like a documentation error and was noted as one here until a
/// capture of the vendor tool's "Detection Devices" on a real RS-04
/// (2026-08-02) showed `ID=0x0400FD01  00 C4 00 00 00 00 00 00` on the wire
/// and the motor answering. The manual is right; only the payload prefix
/// separates this from a disable.
pub fn build_version_read_frame(host_id: u8, device_id: u8) -> (u32, [u8; DATA_LEN]) {
    let can_id = build_can_id(CommType::Disable, host_id as u16, device_id);
    let mut data = [0u8; DATA_LEN];
    data[..VERSION_REQUEST_PREFIX.len()].copy_from_slice(&VERSION_REQUEST_PREFIX);
    (can_id, data)
}

/// Whether a frame is a version-read *request* rather than a `DISABLE`.
///
/// They share a comm type, so anything displaying captured traffic has to
/// check the payload or it will label a version query as a command that stops
/// the motor.
pub fn is_version_request(can_id: u32, data: &[u8]) -> bool {
    let (comm_type, _, _) = parse_can_id(can_id);
    comm_type == CommType::Disable as u8
        && data.len() >= VERSION_REQUEST_PREFIX.len()
        && data[..VERSION_REQUEST_PREFIX.len()] == VERSION_REQUEST_PREFIX
}

/// A motor's reported firmware version, from [`parse_version_reply`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VersionReply {
    /// The motor that answered.
    pub motor_id: u8,
    /// Version components, most significant first — `[0, 4, 1, 32]` renders as
    /// `0.4.1.32`, which is what the vendor tool shows for `AppCodeVersion`.
    pub version: [u8; 4],
    /// Byte 7. Undocumented, and **`0x07` on both an RS-04 and an EduLite05**
    /// — two motors whose MIT scales differ 21-fold — so it does not identify
    /// a model. Seven is also exactly the number of meaningful bytes in this
    /// reply (`00 C4 56` plus four version bytes), which would make it a
    /// length marker rather than data. Kept because it is the last
    /// unexplained field in the exchange the vendor tool uses.
    pub trailing: u8,
}

impl core::fmt::Display for VersionReply {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let [a, b, c, d] = self.version;
        write!(f, "{a}.{b}.{c}.{d}")
    }
}

/// Decode a version-read reply.
///
/// Returns `None` for anything else — including ordinary feedback, which
/// arrives under the *same* comm type (2) and is told apart only by the
/// [`VERSION_REPLY_PREFIX`]. Feedback parsers must reject these frames for the
/// same reason: decoded as feedback, a version reply is plausible-looking
/// nonsense rather than an obvious error.
pub fn parse_version_reply(can_id: u32, data: &[u8]) -> Option<VersionReply> {
    let (comm_type, extra, _target) = parse_can_id(can_id);
    if comm_type != CommType::OperationStatus as u8 {
        return None;
    }
    if data.len() < 8 || data[..3] != VERSION_REPLY_PREFIX {
        return None;
    }
    Some(VersionReply {
        motor_id: (extra & 0xFF) as u8,
        version: [data[3], data[4], data[5], data[6]],
        trailing: data[7],
    })
}

/// Build a `SET_ZERO_POSITION` frame. The first payload byte is `1` per spec.
pub fn build_set_zero_frame(host_id: u8, device_id: u8) -> (u32, [u8; DATA_LEN]) {
    let can_id = build_can_id(CommType::SetZeroPosition, host_id as u16, device_id);
    let mut data = [0u8; DATA_LEN];
    data[0] = 1;
    (can_id, data)
}

/// Build a `SET_CAN_ID` frame: reassigns `device_id`'s address to `new_id`.
///
/// The request is addressed to the motor's **current** id (`device_id`); the
/// new id rides in the 16-bit `extra_data` field. Takes effect immediately
/// (no restart / explicit flash-save needed) and persists across power
/// cycles, per the reference implementation.
pub fn build_set_device_id_frame(device_id: u8, new_id: u8) -> (u32, [u8; DATA_LEN]) {
    let can_id = build_can_id(CommType::SetDeviceId, new_id as u16, device_id);
    (can_id, [0u8; DATA_LEN])
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

    // Bytes below are a live capture of motorstudio's "Detection Devices"
    // against a real RS-04 on PCAN (2026-08-02), taken with
    // `robstride-cli dump`.

    #[test]
    fn version_request_matches_the_captured_frame() {
        let (id, data) = build_version_read_frame(0xFD, 0x01);
        assert_eq!(id, 0x0400FD01, "captured ID was 0x0400FD01");
        assert_eq!(data, [0x00, 0xC4, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn version_reply_decodes_to_the_version_the_vendor_tool_shows() {
        // 0x020001FD  00 C4 56 00 04 01 20 07  → motorstudio displayed
        // AppCodeVersion "0.4.1.32" for this unit.
        let reply = parse_version_reply(0x020001FD, &[0x00, 0xC4, 0x56, 0x00, 0x04, 0x01, 0x20, 0x07])
            .expect("a version reply");
        assert_eq!(reply.motor_id, 1);
        assert_eq!(reply.version, [0, 4, 1, 32]);
        assert_eq!(reply.trailing, 0x07);
    }

    /// A version reply arrives under comm type 2, exactly like feedback. Only
    /// the `00 C4 56` prefix separates them, so the parser must not accept an
    /// ordinary feedback frame as a version — it would report a fabricated
    /// firmware version rather than fail.
    #[test]
    fn ordinary_feedback_is_not_mistaken_for_a_version() {
        assert!(parse_version_reply(0x020001FD, &[0x7F, 0xFF, 0x80, 0x00, 0x80, 0x00, 0x01, 0x2C]).is_none());
        // Right prefix, wrong comm type.
        assert!(parse_version_reply(0x000001FE, &[0x00, 0xC4, 0x56, 0, 0, 0, 0, 0]).is_none());
        // Truncated.
        assert!(parse_version_reply(0x020001FD, &[0x00, 0xC4, 0x56]).is_none());
    }

    /// Captured from the same exchange against a real EduLite05.
    ///
    /// The point of this test is byte 7: it is `0x07` here *and* on the RS-04,
    /// two motors with 21-fold different MIT scales. Whatever it is, it is not
    /// a model code — which was the leading hypothesis for how the vendor tool
    /// tells `RS04` from `EL05` until this capture ruled it out.
    #[test]
    fn the_edulite_version_reply_has_the_same_trailing_byte() {
        let rs04 = parse_version_reply(0x020001FD, &[0x00, 0xC4, 0x56, 0x00, 0x04, 0x01, 0x20, 0x07])
            .expect("rs04 version");
        let el05 = parse_version_reply(0x020001FD, &[0x00, 0xC4, 0x56, 0x0A, 0x05, 0x00, 0x01, 0x07])
            .expect("el05 version");

        assert_eq!(el05.version, [10, 5, 0, 1]);
        assert_eq!(
            el05.trailing, rs04.trailing,
            "byte 7 is constant across models, so it cannot identify one"
        );
    }

    /// The request shares its comm-type field with `DISABLE`; the payload is
    /// the only thing that distinguishes them.
    #[test]
    fn a_version_request_is_a_disable_id_with_a_marked_payload() {
        let (version_id, version_data) = build_version_read_frame(0xFD, 0x01);
        let (disable_id, disable_data) = build_disable_frame(0xFD, 0x01);
        assert_eq!(version_id, disable_id);
        assert_ne!(version_data, disable_data);
    }

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
    fn set_device_id_layout() {
        // Addressed to the current id (127); new id (1) rides in extra_data.
        let (can_id, data) = build_set_device_id_frame(127, 1);
        assert_eq!(can_id, (CommType::SetDeviceId as u32) << 24 | (1u32 << 8) | 127);
        assert_eq!(data, [0u8; 8]);
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
