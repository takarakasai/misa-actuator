//! MIT-mode value encoding/decoding.
//!
//! Signed quantities (position, velocity, torque) map `[-scale, +scale]` onto
//! `[0x0000, 0xFFFF]` using `0x7FFF` as the zero-point. Unsigned quantities
//! (kp, kd) map `[0, scale]` onto `[0x0000, 0xFFFF]`.

const SIGNED_ZERO: u32 = 0x7FFF;
const SIGNED_FULL: u32 = 0xFFFF;

/// `((value / scale) + 1.0) * 0x7FFF`, clamped to `[0, 0xFFFF]`.
pub fn encode_mit_signed(value: f32, scale: f32) -> u16 {
    let clamped = value.max(-scale).min(scale);
    let u = ((clamped / scale) + 1.0) * SIGNED_ZERO as f32;
    if u <= 0.0 {
        0
    } else if u >= SIGNED_FULL as f32 {
        SIGNED_FULL as u16
    } else {
        u as u16
    }
}

/// Inverse of [`encode_mit_signed`].
pub fn decode_mit_signed(raw: u16, scale: f32) -> f32 {
    ((raw as f32) / SIGNED_ZERO as f32 - 1.0) * scale
}

/// `(value / scale) * 0xFFFF`, clamped to `[0, 0xFFFF]`.
pub fn encode_mit_unsigned(value: f32, scale: f32) -> u16 {
    let clamped = value.max(0.0).min(scale);
    ((clamped / scale) * SIGNED_FULL as f32) as u16
}

/// Inverse of [`encode_mit_unsigned`].
pub fn decode_mit_unsigned(raw: u16, scale: f32) -> f32 {
    (raw as f32 / SIGNED_FULL as f32) * scale
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    #[test]
    fn signed_zero() {
        assert_eq!(encode_mit_signed(0.0, 4.0 * PI), 0x7FFF);
    }

    #[test]
    fn signed_round_trip() {
        let scale = 4.0 * PI;
        let v = 1.5f32;
        let encoded = encode_mit_signed(v, scale);
        let decoded = decode_mit_signed(encoded, scale);
        assert!((v - decoded).abs() < 0.01);
    }

    #[test]
    fn signed_boundaries() {
        let scale = 50.0;
        assert_eq!(encode_mit_signed(-60.0, scale), 0);
        assert!(encode_mit_signed(60.0, scale) >= 0xFFFE);
    }

    #[test]
    fn unsigned_round_trip() {
        let scale = 500.0;
        let v = 100.0f32;
        let encoded = encode_mit_unsigned(v, scale);
        let decoded = decode_mit_unsigned(encoded, scale);
        assert!((v - decoded).abs() < 0.01);
    }
}
