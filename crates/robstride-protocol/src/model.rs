//! Motor model identifiers and per-model MIT scaling tables.
//!
//! The Edulite series uses the underlying RS-XX motors:
//! `Edulite01`/`02`/`05` ↔ `Rs01`/`Rs02`/`Rs05`.

use core::f32::consts::PI;

/// Robstride motor model identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorModel {
    Rs00,
    Rs01,
    Rs02,
    Rs03,
    Rs04,
    Rs05,
    Rs06,
}

impl MotorModel {
    /// Parse a model name (case-insensitive). Accepts both `RS-05` style and
    /// `Edulite05` aliases for the supported variants.
    pub fn from_name(s: &str) -> Option<Self> {
        let lower_buf = ascii_lower::<32>(s);
        let lower = lower_buf.as_str();
        Some(match lower {
            "rs-00" | "rs00" => Self::Rs00,
            "rs-01" | "rs01" | "edulite01" => Self::Rs01,
            "rs-02" | "rs02" | "edulite02" => Self::Rs02,
            "rs-03" | "rs03" => Self::Rs03,
            "rs-04" | "rs04" => Self::Rs04,
            "rs-05" | "rs05" | "edulite05" => Self::Rs05,
            "rs-06" | "rs06" => Self::Rs06,
            _ => return None,
        })
    }

    /// Canonical short name (e.g. `"RS-05"`).
    pub fn name(&self) -> &'static str {
        match self {
            Self::Rs00 => "RS-00",
            Self::Rs01 => "RS-01",
            Self::Rs02 => "RS-02",
            Self::Rs03 => "RS-03",
            Self::Rs04 => "RS-04",
            Self::Rs05 => "RS-05",
            Self::Rs06 => "RS-06",
        }
    }
}

impl core::fmt::Display for MotorModel {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.name())
    }
}

/// MIT-mode scaling parameters for a specific motor model.
///
/// Signed quantities (position, velocity, torque) span `[-scale, +scale]`;
/// unsigned ones (kp, kd) span `[0, scale]`. Values are derived from the
/// official Robstride manual — confirm against your firmware revision.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MitScales {
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    pub kp: f32,
    pub kd: f32,
}

impl MitScales {
    pub const fn for_model(model: MotorModel) -> Self {
        match model {
            MotorModel::Rs00 => Self {
                position: 4.0 * PI,
                velocity: 50.0,
                torque: 17.0,
                kp: 500.0,
                kd: 5.0,
            },
            MotorModel::Rs01 => Self {
                position: 4.0 * PI,
                velocity: 44.0,
                torque: 17.0,
                kp: 500.0,
                kd: 5.0,
            },
            MotorModel::Rs02 => Self {
                position: 4.0 * PI,
                velocity: 44.0,
                torque: 17.0,
                kp: 500.0,
                kd: 5.0,
            },
            MotorModel::Rs03 => Self {
                position: 4.0 * PI,
                velocity: 50.0,
                torque: 60.0,
                kp: 5000.0,
                kd: 100.0,
            },
            MotorModel::Rs04 => Self {
                position: 4.0 * PI,
                velocity: 15.0,
                torque: 120.0,
                kp: 5000.0,
                kd: 100.0,
            },
            MotorModel::Rs05 => Self {
                position: 4.0 * PI,
                velocity: 33.0,
                torque: 17.0,
                kp: 500.0,
                kd: 5.0,
            },
            MotorModel::Rs06 => Self {
                position: 4.0 * PI,
                velocity: 20.0,
                torque: 60.0,
                kp: 5000.0,
                kd: 100.0,
            },
        }
    }
}

/// Tiny no_std-friendly buffer used by [`MotorModel::from_name`] to lowercase
/// a candidate string without allocating.
struct AsciiLower<const N: usize> {
    buf: [u8; N],
    len: usize,
}

impl<const N: usize> AsciiLower<N> {
    fn as_str(&self) -> &str {
        // SAFETY: we only push ASCII bytes (lowercased) into `buf`.
        unsafe { core::str::from_utf8_unchecked(&self.buf[..self.len]) }
    }
}

fn ascii_lower<const N: usize>(s: &str) -> AsciiLower<N> {
    let mut out = AsciiLower {
        buf: [0u8; N],
        len: 0,
    };
    for &b in s.as_bytes() {
        if out.len >= N {
            break;
        }
        out.buf[out.len] = b.to_ascii_lowercase();
        out.len += 1;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_aliases() {
        assert_eq!(MotorModel::from_name("rs05"), Some(MotorModel::Rs05));
        assert_eq!(MotorModel::from_name("RS-05"), Some(MotorModel::Rs05));
        assert_eq!(MotorModel::from_name("Edulite05"), Some(MotorModel::Rs05));
        assert_eq!(MotorModel::from_name("rs-04"), Some(MotorModel::Rs04));
        assert_eq!(MotorModel::from_name("nope"), None);
    }

    #[test]
    fn rs05_scales() {
        let s = MitScales::for_model(MotorModel::Rs05);
        assert!((s.velocity - 33.0).abs() < f32::EPSILON);
        assert!((s.torque - 17.0).abs() < f32::EPSILON);
    }
}
