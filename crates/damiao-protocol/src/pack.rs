//! Fixed-point float ⇄ uint conversion used by the DAMIAO MIT protocol.
//!
//! DAMIAO packs `position` / `velocity` / `kp` / `kd` / `torque` as fixed-width
//! unsigned integers spanning a configured `[min, max]` range. The exact
//! quantization must match the firmware bit-for-bit, so two details matter:
//!
//! 1. the value is **clamped** to `[min, max]` *before* quantizing, and
//! 2. the scale denominator is **`2^bits − 1`** (e.g. `65535` for the 16-bit
//!    position field, `4095` for the 12-bit fields) — *not* `2^bits`.
//!
//! These match the official DAMIAO SDK (`DM_CAN.py` / `damiao.h`):
//!
//! ```text
//! u = (uint) ( (clamp(x, x_min, x_max) - x_min) / (x_max - x_min) * ((1<<bits)-1) )
//! x = (float) u / ((1<<bits)-1) * (x_max - x_min) + x_min
//! ```

/// Quantize `x` into a `bits`-wide unsigned integer over `[x_min, x_max]`.
///
/// `x` is clamped to the range first. The result fits in `bits` bits
/// (caller passes `bits <= 16`).
pub fn float_to_uint(x: f32, x_min: f32, x_max: f32, bits: u8) -> u16 {
    let span = x_max - x_min;
    let clamped = x.clamp(x_min, x_max);
    let norm = (clamped - x_min) / span;
    let max_code = ((1u32 << bits) - 1) as f32;
    // `as u16` truncates toward zero and saturates, matching the C `(uint)` cast
    // for the non-negative `norm * max_code` produced here.
    (norm * max_code) as u16
}

/// Inverse of [`float_to_uint`]: expand a `bits`-wide code back to a float in
/// `[x_min, x_max]`.
pub fn uint_to_float(code: u16, x_min: f32, x_max: f32, bits: u8) -> f32 {
    let span = x_max - x_min;
    let max_code = ((1u32 << bits) - 1) as f32;
    (code as f32) / max_code * span + x_min
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn endpoints_map_to_code_extremes() {
        // min → 0, max → all-ones
        assert_eq!(float_to_uint(-12.5, -12.5, 12.5, 16), 0);
        assert_eq!(float_to_uint(12.5, -12.5, 12.5, 16), 0xFFFF);
        assert_eq!(float_to_uint(0.0, 0.0, 500.0, 12), 0);
        assert_eq!(float_to_uint(500.0, 0.0, 500.0, 12), 0x0FFF);
    }

    #[test]
    fn midpoint_is_half_scale() {
        // 0.0 over a symmetric range → ~half of 0xFFFF
        let code = float_to_uint(0.0, -12.5, 12.5, 16);
        assert!((code as i32 - 0x7FFF).abs() <= 1, "got {code:#06X}");
    }

    #[test]
    fn out_of_range_is_clamped_not_wrapped() {
        assert_eq!(float_to_uint(100.0, -12.5, 12.5, 16), 0xFFFF);
        assert_eq!(float_to_uint(-100.0, -12.5, 12.5, 16), 0);
        // kp/kd are unsigned ranges; negative input clamps to 0.
        assert_eq!(float_to_uint(-1.0, 0.0, 5.0, 12), 0);
    }

    #[test]
    fn round_trip_is_close() {
        for &x in &[-10.0_f32, -3.3, 0.0, 1.234, 9.9] {
            let code = float_to_uint(x, -12.5, 12.5, 16);
            let back = uint_to_float(code, -12.5, 12.5, 16);
            assert!((back - x).abs() < 1e-3, "x={x} back={back}");
        }
    }
}
