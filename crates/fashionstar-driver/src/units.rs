//! Conversions between wire units and SI.

use std::f32::consts::{PI, TAU};

/// One wire angle LSB (0.1°) in radians.
pub const RAD_PER_LSB: f32 = PI / 1800.0;

/// Wire angle (0.1°) → rad.
#[inline]
pub fn raw_to_rad(raw: i32) -> f32 {
    // Via f64: a multi-turn raw angle can reach ±3.7 M, past f32's exact
    // integer range once multiplied.
    (raw as f64 * (std::f64::consts::PI / 1800.0)) as f32
}

/// rad → wire angle (0.1°), rounded to nearest. Non-finite input maps to 0
/// rather than to a saturated extreme that would slam the servo.
#[inline]
pub fn rad_to_raw(rad: f32) -> i32 {
    if !rad.is_finite() {
        return 0;
    }
    let v = (rad as f64 * (1800.0 / std::f64::consts::PI)).round();
    v.clamp(i32::MIN as f64, i32::MAX as f64) as i32
}

/// Raw NTC ADC count → °C, the SDK's formula
/// (`response_query_servo_monitor` / `query_temperature`):
///
/// ```text
/// T = 1 / (ln(raw / (4096 - raw)) / 3435 + 1 / 298.15) - 273.15
/// ```
///
/// i.e. a 12-bit ADC across a 10 kΩ NTC (B = 3435 K, 25 °C reference) and a
/// 10 kΩ fixed resistor; `raw = 2048` is exactly 25 °C and a *higher* count
/// means *colder*. The SDK returns `0` when the log argument is not
/// positive; this returns NaN, which cannot be mistaken for a reading.
pub fn ntc_to_celsius(raw: u16) -> f32 {
    if raw == 0 || raw >= 4096 {
        return f32::NAN;
    }
    let r = raw as f64 / (4096.0 - raw as f64);
    (1.0 / (r.ln() / 3435.0 + 1.0 / (273.15 + 25.0)) - 273.15) as f32
}

/// Fold a multi-turn angle into the 2π-wide window centred on `center_rad`.
///
/// The multi-turn angle can carry whole extra turns that are not joint
/// motion — the vendor scripts clear the turn counter at start-up
/// ([`crate::FsCommands::reset_multi_turn`]) precisely because it is not
/// zero on its own, a reset that did not take (Seeed found the broadcast
/// form ineffective) leaves it set, and a joint whose range crosses the
/// servo's ±180° seam reads a turn off after a reset. Seeed's
/// reBot Arm 102 teleoperator (`_round_to_valid_range`) therefore unwraps
/// every reading into a window centred on the joint's range midpoint; this is
/// the closed form of its bidirectional search.
///
/// Only valid for joints whose mechanical range is under one turn, which is
/// every joint of the Star Arm 102.
pub fn unwrap_to_window(angle_rad: f32, center_rad: f32) -> f32 {
    let k = ((angle_rad - center_rad) / TAU).round();
    angle_rad - k * TAU
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angle_round_trip() {
        assert_eq!(rad_to_raw(PI), 1800);
        assert_eq!(rad_to_raw(-PI / 2.0), -900);
        assert!((raw_to_rad(1234) - 123.4f32.to_radians()).abs() < 1e-6);
        assert_eq!(rad_to_raw(raw_to_rad(3_686_400)), 3_686_400);
        assert_eq!(rad_to_raw(f32::NAN), 0);
    }

    #[test]
    fn temperature_matches_sdk() {
        // Values from running the SDK expression in Python.
        assert!((ntc_to_celsius(2048) - 25.0).abs() < 1e-4);
        assert!((ntc_to_celsius(1000) - 57.426_59).abs() < 1e-3);
        assert!((ntc_to_celsius(3000) - 1.036_009).abs() < 1e-3);
        assert!(ntc_to_celsius(0).is_nan());
        assert!(ntc_to_celsius(4096).is_nan());
    }

    #[test]
    fn unwrap() {
        let d = |x: f32| x.to_radians();
        // Gripper range 0..270 → centre 135°: a reading of -100° is really 260°.
        assert!((unwrap_to_window(d(-100.0), d(135.0)) - d(260.0)).abs() < 1e-5);
        // Two extra turns on the shoulder (centre 0).
        assert!((unwrap_to_window(d(30.0 + 720.0), 0.0) - d(30.0)).abs() < 1e-4);
        assert!((unwrap_to_window(d(-30.0 - 360.0), 0.0) - d(-30.0)).abs() < 1e-4);
        // Elbow range -200..1 → centre -99.5°.
        assert!((unwrap_to_window(d(170.0), d(-99.5)) - d(-190.0)).abs() < 1e-4);
    }
}
