//! Per-model quantization limits (the DAMIAO SDK `Limit_Param` table).
//!
//! The MIT protocol quantizes each field over a symmetric range
//! `[-max, +max]` (position/velocity/torque) or `[0, max]` (kp/kd). The
//! position range is `±12.5 rad` and `kp ∈ [0,500]`, `kd ∈ [0,5]` for every
//! DAMIAO model; only the velocity and torque maxima vary per model.

/// A DAMIAO motor model. Add more variants from the SDK `Limit_Param` table as
/// needed (Kp/Kd ranges are global; only P/V/T differ per model).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorModel {
    /// DM-J4310-2EC — `Limit_Param = {12.5, 30, 10}`.
    Dm4310,
    /// DM-J3507-2EC — `Limit_Param = {12.566, 50, 5}` (small low-inertia joint).
    Dm3507,
}

impl MotorModel {
    /// Parse a model name. Accepts the canonical name, the full part number, or
    /// just the digits, case-insensitively — e.g. `"DM4310"`, `"dm-j4310-2ec"`,
    /// `"4310"`, `"DM3507"`, `"dm-j3507-2ec"`, `"3507"`.
    pub fn from_name(s: &str) -> Option<Self> {
        let key = LowerBuf::new(s);
        if key.contains("4310") {
            Some(MotorModel::Dm4310)
        } else if key.contains("3507") {
            Some(MotorModel::Dm3507)
        } else {
            None
        }
    }

    /// Canonical short name.
    pub const fn name(&self) -> &'static str {
        match self {
            MotorModel::Dm4310 => "DM4310",
            MotorModel::Dm3507 => "DM3507",
        }
    }

    /// Whether this model honours the NVM "set zero" magic frame (`FF..FE`),
    /// which writes the zero offset to the motor's flash.
    ///
    /// Drivers gate `set_zero_nvm` on this so that issuing it to a motor that
    /// does not support it fails with a clear "unsupported" error rather than
    /// silently doing nothing.
    pub const fn supports_nvm_zero(&self) -> bool {
        match self {
            MotorModel::Dm4310 => true,
            MotorModel::Dm3507 => true,
        }
    }

    /// Quantization limits for this model.
    pub const fn limits(&self) -> Limits {
        Limits::for_model(*self)
    }
}

/// Symmetric/one-sided quantization ranges for the MIT fields.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    /// Position half-range: field spans `[-p_max, +p_max]` rad.
    pub p_max: f32,
    /// Velocity half-range: field spans `[-v_max, +v_max]` rad/s.
    pub v_max: f32,
    /// Torque half-range: field spans `[-t_max, +t_max]` N·m.
    pub t_max: f32,
    /// Kp range: `[0, kp_max]` N·m/rad.
    pub kp_max: f32,
    /// Kd range: `[0, kd_max]` N·m·s/rad.
    pub kd_max: f32,
}

impl Limits {
    /// The `Limit_Param` entry for `model`.
    pub const fn for_model(model: MotorModel) -> Self {
        match model {
            MotorModel::Dm4310 => Self {
                p_max: 12.5,
                v_max: 30.0,
                t_max: 10.0,
                kp_max: 500.0,
                kd_max: 5.0,
            },
            // Kp/Kd ranges are global across DAMIAO models (0..500 / 0..5);
            // only P/V/T differ. P_MAX is 4π (≈12.566), not 12.5.
            MotorModel::Dm3507 => Self {
                p_max: 12.566,
                v_max: 50.0,
                t_max: 5.0,
                kp_max: 500.0,
                kd_max: 5.0,
            },
        }
    }
}

/// Tiny no_std/no_alloc case-insensitive substring matcher so `from_name`
/// works without `alloc`. Lower-cases on the fly into a fixed buffer.
struct LowerBuf {
    buf: [u8; 32],
    len: usize,
}

impl LowerBuf {
    fn new(s: &str) -> Self {
        let mut buf = [0u8; 32];
        let mut len = 0;
        for &b in s.as_bytes() {
            if len >= buf.len() {
                break;
            }
            buf[len] = b.to_ascii_lowercase();
            len += 1;
        }
        Self { buf, len }
    }

    fn contains(&self, needle: &str) -> bool {
        let hay = &self.buf[..self.len];
        let n = needle.as_bytes();
        if n.is_empty() || n.len() > hay.len() {
            return false;
        }
        hay.windows(n.len()).any(|w| w == n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_names() {
        assert_eq!(MotorModel::from_name("DM4310"), Some(MotorModel::Dm4310));
        assert_eq!(MotorModel::from_name("dm-j4310-2ec"), Some(MotorModel::Dm4310));
        assert_eq!(MotorModel::from_name("4310"), Some(MotorModel::Dm4310));
        assert_eq!(MotorModel::from_name("DM3507"), Some(MotorModel::Dm3507));
        assert_eq!(MotorModel::from_name("dm-j3507-2ec"), Some(MotorModel::Dm3507));
        assert_eq!(MotorModel::from_name("3507"), Some(MotorModel::Dm3507));
        assert_eq!(MotorModel::from_name("rs05"), None);
    }

    #[test]
    fn dm3507_limits() {
        let l = MotorModel::Dm3507.limits();
        assert_eq!(l.p_max, 12.566);
        assert_eq!(l.v_max, 50.0);
        assert_eq!(l.t_max, 5.0);
        assert_eq!(l.kp_max, 500.0); // global, same as DM4310
        assert_eq!(l.kd_max, 5.0);
        assert!(MotorModel::Dm3507.supports_nvm_zero());
    }

    #[test]
    fn dm4310_supports_nvm_zero() {
        assert!(MotorModel::Dm4310.supports_nvm_zero());
    }

    #[test]
    fn dm4310_limits() {
        let l = MotorModel::Dm4310.limits();
        assert_eq!(l.p_max, 12.5);
        assert_eq!(l.v_max, 30.0);
        assert_eq!(l.t_max, 10.0);
        assert_eq!(l.kp_max, 500.0);
        assert_eq!(l.kd_max, 5.0);
    }
}
