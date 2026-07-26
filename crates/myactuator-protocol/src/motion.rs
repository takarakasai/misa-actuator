//! Motion-mode (MIT) control — `0x400 + ID` command / `0x500 + ID` reply.
//!
//! The command packs five parameters MIT-style into 8 bytes:
//!
//! | field | bits | range                  |
//! |-------|------|------------------------|
//! | p_des | 16   | ±[`P_MAX`] rad         |
//! | v_des | 12   | ±[`V_MAX`] rad/s       |
//! | kp    | 12   | 0..=[`KP_MAX`]         |
//! | kd    | 12   | 0..=[`KD_MAX`]         |
//! | t_ff  | 12   | ±[`T_MAX`] N·m         |
//!
//! `IqRef = [kp·(p_des − p) + kd·(v_des − v) + t_ff] · Kt`. Unlike the
//! Robstride / DAMIAO MIT tables these ranges are fixed by the protocol, not
//! per-model.
//!
//! The reply carries the actual position (16 bit), velocity (12 bit) and
//! torque (12 bit) over the same ranges — but **not** at the same byte
//! offsets as the command. Per the official "Servo Motor Control Protocol"
//! V4.3 manual §5.3, the reply's `DATA[0]` is the replying device's CAN
//! address (echoed back), so the p/v/t fields are shifted one byte later
//! than in the command frame:
//!
//! | byte | command (§5.2)      | reply (§5.3)        |
//! |------|----------------------|----------------------|
//! | 0    | p_des[8:15]          | CAN address           |
//! | 1    | p_des[0:7]           | p_des[8:15]           |
//! | 2    | v_des[4:11]          | p_des[0:7]            |
//! | 3    | v_des[0:3] / kp[8:11]| v_des[4:11]           |
//! | 4    | kp[0:7]              | v_des[0:3] / t_ff[8:11]|
//! | 5    | kd[4:11]             | t_ff[0:7]              |
//! | 6    | kd[0:3] / t_ff[8:11] | NULL                   |
//! | 7    | t_ff[0:7]            | NULL                   |
//!
//! (The reply has no kp/kd fields — only p/v/t feedback.) A prior revision
//! of this parser assumed the reply shared the command's byte layout, which
//! misread the leading CAN-address byte as part of `p_des`; that produced
//! the "stuck" feedback (constant `pos`/`vel`/`tau` regardless of actual
//! motion) observed on X4-36 hardware on 2026-07-26. Fixed here against the
//! manual's own worked example in §5.4.

/// Position range: `[-P_MAX, +P_MAX]` rad.
pub const P_MAX: f32 = 12.5;
/// Velocity range: `[-V_MAX, +V_MAX]` rad/s.
pub const V_MAX: f32 = 45.0;
/// Kp range: `[0, KP_MAX]`.
pub const KP_MAX: f32 = 500.0;
/// Kd range: `[0, KD_MAX]`.
pub const KD_MAX: f32 = 5.0;
/// Feed-forward torque range: `[-T_MAX, +T_MAX]` N·m.
pub const T_MAX: f32 = 24.0;

/// Quantize `x` into a `bits`-wide unsigned code over `[x_min, x_max]`,
/// clamping first. Denominator is `2^bits − 1`, matching the manual's
/// `(58981/65535)*25 + (-12.5)` example arithmetic.
fn float_to_uint(x: f32, x_min: f32, x_max: f32, bits: u8) -> u16 {
    let span = x_max - x_min;
    let clamped = x.clamp(x_min, x_max);
    let max_code = ((1u32 << bits) - 1) as f32;
    ((clamped - x_min) / span * max_code) as u16
}

/// Inverse of [`float_to_uint`].
fn uint_to_float(code: u16, x_min: f32, x_max: f32, bits: u8) -> f32 {
    let span = x_max - x_min;
    let max_code = ((1u32 << bits) - 1) as f32;
    (code as f32) / max_code * span + x_min
}

/// Build a motion-mode command payload for the `0x400 + ID` channel.
pub fn build_motion_control(p_des: f32, v_des: f32, kp: f32, kd: f32, t_ff: f32) -> [u8; 8] {
    let p = float_to_uint(p_des, -P_MAX, P_MAX, 16);
    let v = float_to_uint(v_des, -V_MAX, V_MAX, 12);
    let kp = float_to_uint(kp, 0.0, KP_MAX, 12);
    let kd = float_to_uint(kd, 0.0, KD_MAX, 12);
    let t = float_to_uint(t_ff, -T_MAX, T_MAX, 12);
    [
        (p >> 8) as u8,
        p as u8,
        (v >> 4) as u8,
        (((v & 0xF) << 4) | (kp >> 8)) as u8,
        kp as u8,
        (kd >> 4) as u8,
        (((kd & 0xF) << 4) | (t >> 8)) as u8,
        t as u8,
    ]
}

/// Feedback decoded from a `0x500 + ID` motion-mode reply.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionFeedback {
    /// Output-shaft position in rad (±[`P_MAX`]).
    pub position_rad: f32,
    /// Output-shaft velocity in rad/s (±[`V_MAX`]).
    pub velocity_rad_per_s: f32,
    /// Output torque in N·m (±[`T_MAX`]).
    pub torque_nm: f32,
}

/// Decode a motion-mode reply: `data[0]` is the replying device's CAN
/// address (ignored here — the caller already knows which motor it asked),
/// followed by position (16 bit), velocity (12 bit) and torque (12 bit);
/// the remaining bytes are padding.
pub fn parse_motion_reply(data: &[u8]) -> Option<MotionFeedback> {
    if data.len() < 6 {
        return None;
    }
    let p = ((data[1] as u16) << 8) | data[2] as u16;
    let v = ((data[3] as u16) << 4) | (data[4] >> 4) as u16;
    let t = (((data[4] & 0xF) as u16) << 8) | data[5] as u16;
    Some(MotionFeedback {
        position_rad: uint_to_float(p, -P_MAX, P_MAX, 16),
        velocity_rad_per_s: uint_to_float(v, -V_MAX, V_MAX, 12),
        torque_nm: uint_to_float(t, -T_MAX, T_MAX, 12),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pull the five packed codes back out of a command payload.
    fn unpack(frame: &[u8; 8]) -> (u16, u16, u16, u16, u16) {
        let p = ((frame[0] as u16) << 8) | frame[1] as u16;
        let v = ((frame[2] as u16) << 4) | (frame[3] >> 4) as u16;
        let kp = (((frame[3] & 0xF) as u16) << 8) | frame[4] as u16;
        let kd = ((frame[5] as u16) << 4) | (frame[6] >> 4) as u16;
        let t = (((frame[6] & 0xF) as u16) << 8) | frame[7] as u16;
        (p, v, kp, kd, t)
    }

    #[test]
    fn packing_matches_manual_example() {
        // Manual 5.4 example payload E6 66 82 E0 52 33 3B 55 decodes to
        // p=0xE666=58982 (≈10.0 rad), v=0x82E (≈1.022 rad/s), kp=0x052
        // (≈10.01), kd=0x333 (=1.0), t_ff=0xB55 (≈10.004 N·m). Re-encode
        // those values and allow ±1 code of truncation slack.
        let frame = build_motion_control(9.99981, 1.02198, 10.0122, 1.0, 10.0044);
        let (p, v, kp, kd, t) = unpack(&frame);
        assert!((p as i32 - 0xE666).abs() <= 1, "p={p:#06X}");
        assert!((v as i32 - 0x82E).abs() <= 1, "v={v:#05X}");
        assert!((kp as i32 - 0x052).abs() <= 1, "kp={kp:#05X}");
        assert!((kd as i32 - 0x333).abs() <= 1, "kd={kd:#05X}");
        assert!((t as i32 - 0xB55).abs() <= 1, "t={t:#05X}");
    }

    #[test]
    fn reply_parses_manual_worked_example() {
        // Manual §5.4 reply example for ID 0x501: 01 E6 66 82 EB 55 00 00.
        // DATA[0]=0x01 is the replying CAN address (motor 1); p=0xE666=58982
        // (≈10.00 rad, matches the manual's own decimal), v=0x82E
        // (≈1.021 rad/s), t_ff=0xB55 (≈10.004 N·m).
        let data = [0x01, 0xE6, 0x66, 0x82, 0xEB, 0x55, 0x00, 0x00];
        let fb = parse_motion_reply(&data).unwrap();
        assert!((fb.position_rad - 10.00).abs() < 0.01, "pos={}", fb.position_rad);
        assert!(
            (fb.velocity_rad_per_s - 1.021).abs() < 0.01,
            "vel={}",
            fb.velocity_rad_per_s
        );
        assert!((fb.torque_nm - 10.004).abs() < 0.01, "t={}", fb.torque_nm);
    }

    #[test]
    fn reply_extremes_decode_to_range_edges() {
        // data[0] (CAN address) is ignored by the decoder; all-zero payload
        // bytes decode every field to its negative range edge.
        let fb = parse_motion_reply(&[0x01, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        assert!((fb.position_rad - -P_MAX).abs() < 1e-3);
        assert!((fb.velocity_rad_per_s - -V_MAX).abs() < 1e-3);
        assert!((fb.torque_nm - -T_MAX).abs() < 1e-3);
    }

    #[test]
    fn clamping_at_range_edges() {
        let frame = build_motion_control(100.0, -100.0, 1e6, -1.0, 0.0);
        // p_des clamps to +12.5 → 0xFFFF.
        assert_eq!(frame[0], 0xFF);
        assert_eq!(frame[1], 0xFF);
        // v_des clamps to -45 → code 0.
        assert_eq!(frame[2], 0x00);
        assert_eq!(frame[3] >> 4, 0x0);
        // kp clamps to max (0xFFF), kd to 0.
        assert_eq!(frame[3] & 0xF, 0xF);
        assert_eq!(frame[4], 0xFF);
        assert_eq!(frame[5], 0x00);
    }

    #[test]
    fn zero_command_is_midscale() {
        let frame = build_motion_control(0.0, 0.0, 0.0, 0.0, 0.0);
        let p = ((frame[0] as u16) << 8) | frame[1] as u16;
        assert!((p as i32 - 0x7FFF).abs() <= 1);
    }
}
