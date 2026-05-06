//! DAMIAO command/feedback frame encoding and decoding.
//!
//! Three command families share the 8-byte payload but use different layouts:
//!
//! - **MIT** (`mit_id`): big-endian bit-packed pos(16) / vel(12) / kp(12) /
//!   kd(12) / tau(12), matching the SDK's `controlMIT` byte order.
//! - **POS_VEL / VEL** (`pos_vel_id` / `vel_id`): little-endian IEEE-754
//!   `f32` fields.
//! - **Special commands** (enable / disable / set-zero / clear-error):
//!   `FF FF FF FF FF FF FF <cmd>` on the motor's MIT id.

use crate::can_id::{mit_id, pos_vel_id, vel_id};
use crate::feedback::{ErrorCode, Feedback};
use crate::limits::Limits;
use crate::pack::{float_to_uint, uint_to_float};

/// CAN payload length used by every DAMIAO frame.
pub const DATA_LEN: usize = 8;

/// Special command byte for "enable motor" (`FF*7 FC`).
const CMD_ENABLE: u8 = 0xFC;
/// Special command byte for "disable motor" (`FF*7 FD`).
const CMD_DISABLE: u8 = 0xFD;
/// Special command byte for "save current position as zero" (`FF*7 FE`).
const CMD_SET_ZERO: u8 = 0xFE;
/// Special command byte for "clear error" (`FF*7 FB`).
///
/// Note: the official protocol doc lists `0xFB`, but one SDK mirror uses
/// `0xFF`. Verify against your firmware with `candump`; recovering via a
/// disable→enable toggle is a robust alternative.
const CMD_CLEAR_ERROR: u8 = 0xFB;

/// Build a MIT-mode control frame.
///
/// Field widths (matching the SDK): position 16-bit, velocity/kp/kd/torque
/// 12-bit. Values are clamped to the model `limits` before quantization.
/// Returns the (standard) CAN id and the 8-byte payload.
pub fn build_mit_frame(
    can_id: u8,
    limits: &Limits,
    position: f32,
    velocity: f32,
    kp: f32,
    kd: f32,
    torque: f32,
) -> (u16, [u8; DATA_LEN]) {
    let p = float_to_uint(position, -limits.p_max, limits.p_max, 16);
    let v = float_to_uint(velocity, -limits.v_max, limits.v_max, 12);
    let kp_u = float_to_uint(kp, 0.0, limits.kp_max, 12);
    let kd_u = float_to_uint(kd, 0.0, limits.kd_max, 12);
    let t = float_to_uint(torque, -limits.t_max, limits.t_max, 12);

    let data = [
        (p >> 8) as u8,
        (p & 0xFF) as u8,
        (v >> 4) as u8,
        (((v & 0x0F) << 4) | ((kp_u >> 8) & 0x0F)) as u8,
        (kp_u & 0xFF) as u8,
        (kd_u >> 4) as u8,
        (((kd_u & 0x0F) << 4) | ((t >> 8) & 0x0F)) as u8,
        (t & 0xFF) as u8,
    ];
    (mit_id(can_id), data)
}

/// Build a Position-Velocity frame (`0x100 + can_id`): two little-endian
/// `f32`s — target position (rad) and the max profile speed (rad/s).
pub fn build_pos_vel_frame(can_id: u8, position: f32, max_speed: f32) -> (u16, [u8; DATA_LEN]) {
    let p = position.to_le_bytes();
    let v = max_speed.to_le_bytes();
    let data = [p[0], p[1], p[2], p[3], v[0], v[1], v[2], v[3]];
    (pos_vel_id(can_id), data)
}

/// Build a Velocity frame (`0x200 + can_id`): one little-endian `f32` (rad/s),
/// the remaining bytes zero-padded.
pub fn build_vel_frame(can_id: u8, velocity: f32) -> (u16, [u8; DATA_LEN]) {
    let v = velocity.to_le_bytes();
    let data = [v[0], v[1], v[2], v[3], 0, 0, 0, 0];
    (vel_id(can_id), data)
}

/// `FF FF FF FF FF FF FF <cmd>` on the motor's MIT id.
fn special_frame(can_id: u8, cmd: u8) -> (u16, [u8; DATA_LEN]) {
    (mit_id(can_id), [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd])
}

/// Build the "enable motor" special frame.
pub fn build_enable_frame(can_id: u8) -> (u16, [u8; DATA_LEN]) {
    special_frame(can_id, CMD_ENABLE)
}

/// Build the "disable motor" special frame.
pub fn build_disable_frame(can_id: u8) -> (u16, [u8; DATA_LEN]) {
    special_frame(can_id, CMD_DISABLE)
}

/// Build the "set current position as zero" special frame.
pub fn build_set_zero_frame(can_id: u8) -> (u16, [u8; DATA_LEN]) {
    special_frame(can_id, CMD_SET_ZERO)
}

/// Build the "clear error" special frame.
pub fn build_clear_error_frame(can_id: u8) -> (u16, [u8; DATA_LEN]) {
    special_frame(can_id, CMD_CLEAR_ERROR)
}

/// Parse a feedback frame.
///
/// The `can_id` argument is the frame's CAN id — feedback arrives on the
/// motor's Master ID, so the driver is responsible for matching it; this
/// function only decodes the 8-byte payload. Returns `None` if `data` is
/// shorter than 8 bytes.
pub fn parse_feedback(data: &[u8], limits: &Limits) -> Option<Feedback> {
    if data.len() < DATA_LEN {
        return None;
    }
    let motor_id = data[0] & 0x0F;
    let err = ErrorCode::from_nibble(data[0] >> 4);

    let p_int = ((data[1] as u16) << 8) | data[2] as u16;
    let v_int = ((data[3] as u16) << 4) | ((data[4] as u16) >> 4);
    let t_int = (((data[4] & 0x0F) as u16) << 8) | data[5] as u16;

    Some(Feedback {
        motor_id,
        position: uint_to_float(p_int, -limits.p_max, limits.p_max, 16),
        velocity: uint_to_float(v_int, -limits.v_max, limits.v_max, 12),
        torque: uint_to_float(t_int, -limits.t_max, limits.t_max, 12),
        t_mos: data[6] as f32,
        t_rotor: data[7] as f32,
        err,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::limits::MotorModel;

    fn dm4310() -> Limits {
        MotorModel::Dm4310.limits()
    }

    #[test]
    fn special_frames_match_known_bytes() {
        assert_eq!(
            build_enable_frame(1),
            (0x001, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        );
        assert_eq!(build_disable_frame(1).1[7], 0xFD);
        assert_eq!(build_set_zero_frame(1).1[7], 0xFE);
        assert_eq!(build_clear_error_frame(1).1[7], 0xFB);
    }

    #[test]
    fn mit_frame_id_and_zero_command() {
        // Zero pos/vel/torque over symmetric ranges → ~mid-scale codes.
        let (id, data) = build_mit_frame(1, &dm4310(), 0.0, 0.0, 0.0, 0.0, 0.0);
        assert_eq!(id, 0x001);
        // position field = bytes 0..2 ≈ 0x7FFF
        let p = ((data[0] as u16) << 8) | data[1] as u16;
        assert!((p as i32 - 0x7FFF).abs() <= 1);
        // kp=kd=0 → those code bits are zero
        assert_eq!(data[4], 0x00); // kp low byte
    }

    #[test]
    fn feedback_decodes_packed_values() {
        let limits = dm4310();
        let fb_frame = make_feedback_fixture(1, ErrorCode::Enabled, 1.5, -4.0, 2.0, &limits);
        let fb = parse_feedback(&fb_frame, &limits).unwrap();
        assert_eq!(fb.motor_id, 1);
        assert_eq!(fb.err, ErrorCode::Enabled);
        assert!((fb.position - 1.5).abs() < 0.01, "pos {}", fb.position);
        assert!((fb.velocity - (-4.0)).abs() < 0.05, "vel {}", fb.velocity);
        assert!((fb.torque - 2.0).abs() < 0.05, "tau {}", fb.torque);
    }

    #[test]
    fn feedback_temps_are_raw_celsius() {
        let limits = dm4310();
        let mut frame = make_feedback_fixture(2, ErrorCode::Enabled, 0.0, 0.0, 0.0, &limits);
        frame[6] = 41; // T_MOS
        frame[7] = 55; // T_Rotor
        let fb = parse_feedback(&frame, &limits).unwrap();
        assert_eq!(fb.t_mos, 41.0);
        assert_eq!(fb.t_rotor, 55.0);
    }

    #[test]
    fn pos_vel_and_vel_are_le_floats() {
        let (id, data) = build_pos_vel_frame(1, 3.14, 5.0);
        assert_eq!(id, 0x101);
        assert_eq!(f32::from_le_bytes([data[0], data[1], data[2], data[3]]), 3.14);
        assert_eq!(f32::from_le_bytes([data[4], data[5], data[6], data[7]]), 5.0);

        let (id, data) = build_vel_frame(1, -2.5);
        assert_eq!(id, 0x201);
        assert_eq!(f32::from_le_bytes([data[0], data[1], data[2], data[3]]), -2.5);
        assert_eq!(&data[4..8], &[0, 0, 0, 0]);
    }

    // --- test helpers ---

    /// Build a synthetic feedback payload with the given decoded values, so we
    /// can verify `parse_feedback` independently of a real motor.
    fn make_feedback_fixture(
        id: u8,
        err: ErrorCode,
        pos: f32,
        vel: f32,
        tau: f32,
        limits: &Limits,
    ) -> [u8; 8] {
        let p = float_to_uint(pos, -limits.p_max, limits.p_max, 16);
        let v = float_to_uint(vel, -limits.v_max, limits.v_max, 12);
        let t = float_to_uint(tau, -limits.t_max, limits.t_max, 12);
        [
            (err.raw() << 4) | (id & 0x0F),
            (p >> 8) as u8,
            (p & 0xFF) as u8,
            (v >> 4) as u8,
            (((v & 0x0F) << 4) | ((t >> 8) & 0x0F)) as u8,
            (t & 0xFF) as u8,
            0,
            0,
        ]
    }
}
