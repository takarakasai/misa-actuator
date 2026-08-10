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
    /// Parse a model name (case-insensitive). Accepts `RS-05` style,
    /// `Robstride05` / `Robstride-05` style, and `Edulite05` aliases for the
    /// supported variants.
    pub fn from_name(s: &str) -> Option<Self> {
        let lower_buf = ascii_lower::<32>(s);
        let lower = lower_buf.as_str();
        // `RobStride04` / `EduLite-05` and friends: a family prefix followed
        // by the series number. EduLite is a rebadge, not a separate motor —
        // `EduLite05` *is* an RS-05 and shares its MIT scales, confirmed
        // against both the EL05 and RS05 manuals (2025-11-12 rev).
        for prefix in ["robstride", "edulite"] {
            if let Some(rest) = lower.strip_prefix(prefix) {
                let rest = rest.strip_prefix(['-', '_', ' ']).unwrap_or(rest);
                return match rest {
                    "00" => Some(Self::Rs00),
                    "01" => Some(Self::Rs01),
                    "02" => Some(Self::Rs02),
                    "03" => Some(Self::Rs03),
                    "04" => Some(Self::Rs04),
                    "05" => Some(Self::Rs05),
                    "06" => Some(Self::Rs06),
                    _ => None,
                };
            }
        }
        Some(match lower {
            "rs-00" | "rs00" => Self::Rs00,
            "rs-01" | "rs01" => Self::Rs01,
            "rs-02" | "rs02" => Self::Rs02,
            "rs-03" | "rs03" => Self::Rs03,
            "rs-04" | "rs04" => Self::Rs04,
            "rs-05" | "rs05" => Self::Rs05,
            "rs-06" | "rs06" => Self::Rs06,
            _ => return None,
        })
    }

    /// Map the two-digit series number, as it appears inside a name string.
    pub fn from_number(n: u8) -> Option<Self> {
        Some(match n {
            0 => Self::Rs00,
            1 => Self::Rs01,
            2 => Self::Rs02,
            3 => Self::Rs03,
            4 => Self::Rs04,
            5 => Self::Rs05,
            6 => Self::Rs06,
            _ => return None,
        })
    }

    /// Find a model designation anywhere inside a longer string.
    ///
    /// [`Self::from_name`] wants the whole string to be the name. The strings
    /// a motor reports about itself — `Name`, `BarCode`, `AppCodeName` — are
    /// not that: they are serial numbers, build tags and free text that may
    /// merely *contain* the model. Since RobStride publishes no model
    /// register, one of these is the only place on the bus a motor can
    /// actually state what it is, so it is worth reading them loosely.
    ///
    /// Matches `RS04`, `RS-04`, `RS_04`, `RobStride04` and `Edulite04`,
    /// case-insensitively, followed by exactly two digits.
    pub fn find_in(text: &str) -> Option<Self> {
        const PREFIXES: &[&[u8]] = &[b"robstride", b"edulite", b"rs"];
        let b = text.as_bytes();
        for i in 0..b.len() {
            for prefix in PREFIXES {
                let Some(mut j) = match_ci(b, i, prefix) else {
                    continue;
                };
                if matches!(b.get(j), Some(b'-') | Some(b'_') | Some(b' ')) {
                    j += 1;
                }
                let (Some(&hi), Some(&lo)) = (b.get(j), b.get(j + 1)) else {
                    continue;
                };
                if !hi.is_ascii_digit() || !lo.is_ascii_digit() {
                    continue;
                }
                if let Some(m) = Self::from_number((hi - b'0') * 10 + (lo - b'0')) {
                    return Some(m);
                }
            }
        }
        None
    }

    /// Canonical short name (e.g. `"RS-05"`).
    /// Every model, so a UI can offer them as a list instead of asking the
    /// operator to type one.
    ///
    /// Getting this wrong is not cosmetic: the MIT scales differ by more than
    /// twenty-fold between models (RS-04 quantises torque over ±120 N·m,
    /// RS-05 over ±5.5), so a mis-picked model silently mis-scales every
    /// torque command and reading.
    pub const ALL: &'static [MotorModel] = &[
        MotorModel::Rs00,
        MotorModel::Rs01,
        MotorModel::Rs02,
        MotorModel::Rs03,
        MotorModel::Rs04,
        MotorModel::Rs05,
        MotorModel::Rs06,
    ];

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

    /// Every name a motor in this range is actually sold under, paired with
    /// the model it resolves to.
    ///
    /// [`Self::ALL`] lists the seven distinct motors; this lists the labels a
    /// person reads off the casing. The EduLite series is the same hardware
    /// rebadged for education — `EduLite05` is an RS-05 and shares its MIT
    /// scales exactly — but someone holding one has no reason to know that,
    /// and a picker that only offers `RS-05` either sends them to the wrong
    /// entry or stops them connecting at all.
    ///
    /// Use this to build a selector; use [`Self::ALL`] when enumerating
    /// distinct hardware.
    pub const CATALOGUE: &'static [(&'static str, MotorModel)] = &[
        ("RS-00", MotorModel::Rs00),
        ("RS-01", MotorModel::Rs01),
        ("RS-02", MotorModel::Rs02),
        ("RS-03", MotorModel::Rs03),
        ("RS-04", MotorModel::Rs04),
        ("RS-05", MotorModel::Rs05),
        ("RS-06", MotorModel::Rs06),
        ("EduLite01", MotorModel::Rs01),
        ("EduLite02", MotorModel::Rs02),
        ("EduLite05", MotorModel::Rs05),
    ];

    /// Derive the model from the firmware version a motor reports over
    /// communication type 26.
    ///
    /// **The minor component is the model number.** RobStride's own firmware
    /// release assets spell the mapping out:
    ///
    /// | asset | version |
    /// |---|---|
    /// | `rs00-0.0.3.32.bin` | 0.**0**.3.32 |
    /// | `rs01-0.1.3.18.bin` | 0.**1**.3.18 |
    /// | `rs02-0.2.3.34.bin` | 0.**2**.3.34 |
    /// | `rs03-0.3.1.42.bin` | 0.**3**.1.42 |
    /// | `rs04-0.4.1.32.bin` | 0.**4**.1.32 |
    /// | `rs05_0.5.0.14.bin` | 0.**5**.0.14 |
    /// | `rs06_0.6.0.12.bin` | 0.**6**.0.12 |
    ///
    /// A live RS-04 reports `0.4.1.32` — the release asset's version exactly.
    /// A live EduLite05 reports `10.5.0.1`: same rule, with major `10` marking
    /// the EduLite line instead of `0` for RS. That is what lets the vendor
    /// tool print `RS04` and `EL05` from a version read alone, which is all it
    /// asks a motor for before naming it.
    ///
    /// The major only selects the product line, not the scaling — an EduLite05
    /// *is* an RS-05 and shares its MIT ranges, confirmed against both
    /// manuals. So the returned [`MotorModel`] is correct for either line.
    ///
    /// Returns `None` for an unrecognised line or model number rather than
    /// guessing. Evidence is uneven and worth knowing: the RS mapping rests on
    /// seven published assets plus a live match, the EduLite mapping on one
    /// motor following the same structure.
    pub fn from_firmware_version(version: [u8; 4]) -> Option<(Self, ProductLine)> {
        let line = ProductLine::from_major(version[0])?;
        Some((Self::from_number(version[1])?, line))
    }

    /// What a motor's `AppCodeName` (`0x1007`) says about which models it
    /// could be.
    ///
    /// This is the firmware build's own name for itself, and it is the one
    /// self-describing row that is actually populated: a live EduLite05
    /// reports `"EL_motor"` where `Name` and `BarCode` are both erased flash
    /// (`FF FF …`) on every unit measured so far.
    ///
    /// It names the **product line, not the size within it** — `"EL_motor"`
    /// does not say `05`. So this narrows and never identifies. The vendor
    /// tool does print `EL05`, which means it holds a mapping we have not
    /// reproduced; until that is confirmed against more than one unit,
    /// promoting a line name to a model would be inventing the digits.
    ///
    /// Returns `None` for a build name we have not observed, which is not the
    /// same as "no models match".
    pub fn classify_build_name(app_code_name: &str) -> Option<BuildName> {
        let lower = ascii_lower::<32>(app_code_name);
        match lower.as_str().trim() {
            // Measured on a real EduLite05: AppCodeVersion 1.0.5.0.1,
            // AppBuildDate Oct 23 2025.
            "el_motor" => Some(BuildName::Narrows(&[
                MotorModel::Rs01,
                MotorModel::Rs02,
                MotorModel::Rs05,
            ])),
            // Measured on a real RS-04: AppCodeVersion 0.4.1.32, AppBuildDate
            // May 14 2026. Just `"motor"` — the RS line does not put its size
            // in the build name, so this says nothing beyond "not an EduLite
            // build". Recorded so it reads as *checked and useless* rather
            // than as unrecognised.
            "motor" => Some(BuildName::Uninformative),
            _ => None,
        }
    }

    /// Models a build name is compatible with, or `None` when it narrows
    /// nothing.
    pub fn family_from_build_name(app_code_name: &str) -> Option<&'static [MotorModel]> {
        match Self::classify_build_name(app_code_name)? {
            BuildName::Narrows(models) => Some(models),
            BuildName::Uninformative => None,
        }
    }

    /// The catalogue name given, rendered so a rebadge shows what it really
    /// is: `"EduLite05 (RS-05)"`, but plain `"RS-05"` for a canonical name.
    ///
    /// A session opened as `EduLite05` should keep saying so — the operator
    /// picked that — while still making the electrical identity visible,
    /// since that is what sets the MIT scaling.
    pub fn describe_selection(selected_name: &str) -> Option<CatalogueName<'_>> {
        let model = Self::from_name(selected_name)?;
        Some(CatalogueName {
            given: selected_name,
            model,
        })
    }
}

/// Which product line a firmware version's major component marks.
///
/// The line is a badge, not a different motor: an EduLite05 and an RS-05 are
/// the same hardware with the same MIT scales. It matters for what to *call*
/// the motor, not for how to talk to it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProductLine {
    /// Major `0` — the RS-00..RS-06 range, named `RS-04` and so on.
    Rs,
    /// Major `10` — the EduLite education range, named `EduLite05`.
    EduLite,
}

impl ProductLine {
    pub fn from_major(major: u8) -> Option<Self> {
        match major {
            0 => Some(Self::Rs),
            10 => Some(Self::EduLite),
            _ => None,
        }
    }

    /// The catalogue name this line uses for `model` — `"RS-05"` or
    /// `"EduLite05"`, matching what the picker offers.
    pub fn catalogue_name(self, model: MotorModel) -> &'static str {
        match self {
            Self::Rs => model.name(),
            Self::EduLite => match model {
                MotorModel::Rs01 => "EduLite01",
                MotorModel::Rs02 => "EduLite02",
                MotorModel::Rs05 => "EduLite05",
                // An EduLite badge we have never seen; the model is still
                // right, so name it by the motor rather than inventing a
                // product name for it.
                other => other.name(),
            },
        }
    }
}

/// What a firmware build name is worth, from [`MotorModel::classify_build_name`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BuildName {
    /// Compatible only with these models.
    Narrows(&'static [MotorModel]),
    /// Observed on real hardware and carries no model information — the RS
    /// line reports a bare `"motor"`. Distinct from an unrecognised name,
    /// because "we know this one tells us nothing" and "we have never seen
    /// this one" should not read the same to whoever is debugging a
    /// mis-scaled motor.
    Uninformative,
}

/// A name the operator chose, plus the model it resolves to. See
/// [`MotorModel::describe_selection`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CatalogueName<'a> {
    pub given: &'a str,
    pub model: MotorModel,
}

impl core::fmt::Display for CatalogueName<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if self.given.eq_ignore_ascii_case(self.model.name()) {
            f.write_str(self.model.name())
        } else {
            write!(f, "{} ({})", self.given, self.model.name())
        }
    }
}

impl core::fmt::Display for MotorModel {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.name())
    }
}

/// How much a self-reported limit may exceed the model's MIT scale before the
/// pair is treated as inconsistent.
///
/// Not tight, deliberately. The manuals round their own numbers — the EL05
/// lists `limit_torque` as `0..6` N·m against a 5.5 MIT scale, and the RS-04
/// lists `limit_spd` as `0..20` rad/s against a 15 velocity scale. Slack has
/// to cover that without swallowing the failure this exists to catch, which is
/// an RS-04 driven as an RS-05: a 21.8-fold error, not a 1.3-fold one.
const TORQUE_SLACK: f32 = 1.25;
const VELOCITY_SLACK: f32 = 1.5;

/// Limits a motor reports about itself, for cross-checking the selected model.
///
/// Fields are optional because a read can fail or a firmware build can omit a
/// parameter, and a missing reading must not read as a passing check.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ReportedLimits {
    /// `limit_torque` (`0x700B`), N·m.
    pub limit_torque: Option<f32>,
    /// `limit_spd` (`0x7017`), rad/s.
    pub limit_spd: Option<f32>,
}

impl ReportedLimits {
    pub fn is_empty(&self) -> bool {
        self.limit_torque.is_none() && self.limit_spd.is_none()
    }

    /// Whether `model`'s MIT scales can express these limits.
    ///
    /// This is a refutation test, not an identification. RobStride motors
    /// carry no model register, so nothing on the bus states what they are —
    /// but a motor that reports a 17 N·m torque limit cannot be a model whose
    /// MIT range stops at ±5.5, because the firmware would be honouring a
    /// limit its own command encoding has no way to ask for. That is enough to
    /// catch the mis-selection that matters.
    pub fn consistent_with(&self, model: MotorModel) -> bool {
        let scales = MitScales::for_model(model);
        let ok = |reported: Option<f32>, scale: f32, slack: f32| {
            reported.map_or(true, |v| !v.is_finite() || v <= scale * slack)
        };
        ok(self.limit_torque, scales.torque, TORQUE_SLACK)
            && ok(self.limit_spd, scales.velocity, VELOCITY_SLACK)
    }

    /// Every model whose MIT scales can express these limits, in `ALL` order.
    ///
    /// Usually several — the test only rules models out from below, so any
    /// model with wide enough ranges survives. One survivor is an
    /// identification; none means the readings fit nothing known.
    pub fn candidates(&self) -> impl Iterator<Item = MotorModel> + '_ {
        MotorModel::ALL
            .iter()
            .copied()
            .filter(move |&m| self.consistent_with(m))
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
                velocity: 50.0,
                torque: 5.5,
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

/// Case-insensitive literal match of `needle` at `at`; returns the index just
/// past it.
fn match_ci(haystack: &[u8], at: usize, needle: &[u8]) -> Option<usize> {
    let end = at.checked_add(needle.len())?;
    if end > haystack.len() {
        return None;
    }
    for (h, n) in haystack[at..end].iter().zip(needle) {
        if !h.eq_ignore_ascii_case(n) {
            return None;
        }
    }
    Some(end)
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
    // This crate is `no_std`; `alloc` is pulled in under `cfg(test)` at the
    // crate root under this alias.
    use crate::_alloc_for_tests::string::ToString;

    #[test]
    fn parse_aliases() {
        assert_eq!(MotorModel::from_name("rs05"), Some(MotorModel::Rs05));
        assert_eq!(MotorModel::from_name("RS-05"), Some(MotorModel::Rs05));
        assert_eq!(MotorModel::from_name("Edulite05"), Some(MotorModel::Rs05));
        assert_eq!(MotorModel::from_name("rs-04"), Some(MotorModel::Rs04));
        assert_eq!(MotorModel::from_name("Robstride04"), Some(MotorModel::Rs04));
        assert_eq!(MotorModel::from_name("robstride-04"), Some(MotorModel::Rs04));
        assert_eq!(MotorModel::from_name("robstride07"), None);
        assert_eq!(MotorModel::from_name("nope"), None);
    }

    #[test]
    fn edulite_is_spelled_the_several_ways_it_appears() {
        for s in ["EduLite05", "edulite05", "EduLite-05", "EDULITE_05", "EduLite 05"] {
            assert_eq!(MotorModel::from_name(s), Some(MotorModel::Rs05), "{s}");
        }
        assert_eq!(MotorModel::from_name("EduLite01"), Some(MotorModel::Rs01));
        assert_eq!(MotorModel::from_name("EduLite02"), Some(MotorModel::Rs02));
        // No such rebadge, and inventing one would pick MIT scales at random.
        assert_eq!(MotorModel::from_name("EduLite09"), None);
        assert_eq!(MotorModel::from_name("EduLite"), None);
    }

    /// An `EduLite05` is an `RS-05`. If these ever diverge, every MIT torque
    /// command through an EduLite is silently mis-scaled — which is exactly
    /// the failure the model picker exists to prevent.
    #[test]
    fn edulite_shares_the_scales_of_the_motor_it_rebadges() {
        assert_eq!(
            MitScales::for_model(MotorModel::from_name("EduLite05").unwrap()),
            MitScales::for_model(MotorModel::Rs05)
        );
    }

    #[test]
    fn every_catalogue_name_parses_back_to_its_own_model() {
        for &(name, model) in MotorModel::CATALOGUE {
            assert_eq!(MotorModel::from_name(name), Some(model), "{name}");
        }
        // Every distinct motor is reachable from the catalogue, so a picker
        // built from it cannot strand one.
        for &m in MotorModel::ALL {
            assert!(
                MotorModel::CATALOGUE.iter().any(|&(_, c)| c == m),
                "{m} is absent from the catalogue"
            );
        }
    }

    #[test]
    fn a_rebadge_is_displayed_with_the_model_it_resolves_to() {
        let c = MotorModel::describe_selection("EduLite05").unwrap();
        assert_eq!(c.model, MotorModel::Rs05);
        assert_eq!(c.to_string(), "EduLite05 (RS-05)");

        // A canonical name is not annotated with itself.
        assert_eq!(
            MotorModel::describe_selection("RS-04").unwrap().to_string(),
            "RS-04"
        );
        // Case differences are not a rebadge either.
        assert_eq!(
            MotorModel::describe_selection("rs04").unwrap().to_string(),
            "rs04 (RS-04)"
        );
        assert!(MotorModel::describe_selection("nonsense").is_none());
    }

    #[test]
    fn a_model_is_found_inside_a_longer_self_reported_string() {
        // The shapes a Name / BarCode / AppCodeName field plausibly carries.
        assert_eq!(MotorModel::find_in("RS04"), Some(MotorModel::Rs04));
        assert_eq!(MotorModel::find_in("RS-04_v1.2"), Some(MotorModel::Rs04));
        assert_eq!(
            MotorModel::find_in("RobStride04 joint 3"),
            Some(MotorModel::Rs04)
        );
        assert_eq!(
            MotorModel::find_in("Edulite 05 rev B"),
            Some(MotorModel::Rs05)
        );
        assert_eq!(
            MotorModel::find_in("SN:240815RS03-0091"),
            Some(MotorModel::Rs03)
        );
    }

    #[test]
    fn find_in_does_not_invent_a_model_from_noise() {
        // A firmware version is not a model, and neither is a bare prefix or
        // an out-of-range series number. Guessing here would be worse than
        // saying nothing: the operator would trust a fabricated answer.
        assert_eq!(MotorModel::find_in("0.4.1.32"), None);
        assert_eq!(MotorModel::find_in(""), None);
        assert_eq!(MotorModel::find_in("rs"), None);
        assert_eq!(MotorModel::find_in("rs-"), None);
        assert_eq!(MotorModel::find_in("rs4"), None);
        assert_eq!(MotorModel::find_in("RS-99"), None);
        assert_eq!(MotorModel::find_in("May 14 2026"), None);
        // Mojibake from an unwritten field must not match anything.
        assert_eq!(MotorModel::find_in("\u{fffd}\u{fffd}\u{fffd}"), None);
    }

    /// Every published firmware asset from RobStride's own release page must
    /// map to the model its filename names. This is the whole basis for
    /// version-derived identification, so it is pinned exhaustively.
    #[test]
    fn every_published_firmware_version_maps_to_its_own_model() {
        // (asset filename, version, expected model)
        let assets = [
            ("rs00-0.0.3.32.bin", [0, 0, 3, 32], MotorModel::Rs00),
            ("rs01-0.1.3.18.bin", [0, 1, 3, 18], MotorModel::Rs01),
            ("rs02-0.2.3.34.bin", [0, 2, 3, 34], MotorModel::Rs02),
            ("rs03-0.3.1.42.bin", [0, 3, 1, 42], MotorModel::Rs03),
            ("rs04-0.4.1.32.bin", [0, 4, 1, 32], MotorModel::Rs04),
            ("rs05_0.5.0.14.bin", [0, 5, 0, 14], MotorModel::Rs05),
            ("rs06_0.6.0.12.bin", [0, 6, 0, 12], MotorModel::Rs06),
        ];
        for (asset, version, expected) in assets {
            assert_eq!(
                MotorModel::from_firmware_version(version),
                Some((expected, ProductLine::Rs)),
                "{asset}"
            );
        }
    }

    /// The two live readings, via communication type 26.
    #[test]
    fn the_measured_firmware_versions_name_the_right_motors() {
        // RS-04 on the bench — identical to the published rs04 asset.
        let (model, line) = MotorModel::from_firmware_version([0, 4, 1, 32]).unwrap();
        assert_eq!(model, MotorModel::Rs04);
        assert_eq!(line.catalogue_name(model), "RS-04");

        // EduLite05 on the bench: major 10 marks the line, minor 5 the size.
        // The vendor tool prints "EL05" for exactly this.
        let (model, line) = MotorModel::from_firmware_version([10, 5, 0, 1]).unwrap();
        assert_eq!(model, MotorModel::Rs05);
        assert_eq!(line, ProductLine::EduLite);
        assert_eq!(line.catalogue_name(model), "EduLite05");
        // Whatever it is called, it scales like an RS-05.
        assert_eq!(
            MitScales::for_model(model),
            MitScales::for_model(MotorModel::Rs05)
        );
    }

    #[test]
    fn an_unknown_line_or_size_is_not_guessed_at() {
        // Major 7 is no line we have seen.
        assert_eq!(MotorModel::from_firmware_version([7, 4, 1, 32]), None);
        // Minor 9 is no model in the family.
        assert_eq!(MotorModel::from_firmware_version([0, 9, 0, 0]), None);
    }

    /// Both values were read off real hardware through the vendor tool's
    /// parameter table (2026-08-01/02).
    #[test]
    fn the_edulite_build_name_narrows_to_its_line() {
        let family = MotorModel::family_from_build_name("EL_motor").expect("known build name");
        assert!(family.contains(&MotorModel::Rs05));
        // It must NOT resolve to a single model: `"EL_motor"` carries no size.
        // Collapsing a line name to one model is how you pick MIT scales by
        // wishful thinking.
        assert!(family.len() > 1, "a line name must not identify one model");
        assert!(!family.contains(&MotorModel::Rs04));

        assert_eq!(MotorModel::family_from_build_name("el_motor"), family.into());
    }

    /// The RS line reports a bare `"motor"` — measured on an RS-04
    /// (AppCodeVersion 0.4.1.32). It has to classify as *known and useless*
    /// rather than as unrecognised, so nobody re-runs this investigation a
    /// fourth time.
    #[test]
    fn the_rs_build_name_is_known_to_be_worthless() {
        assert_eq!(
            MotorModel::classify_build_name("motor"),
            Some(BuildName::Uninformative)
        );
        assert_eq!(MotorModel::family_from_build_name("motor"), None);

        // Never seen: distinct from the above, and also narrows nothing.
        assert_eq!(MotorModel::classify_build_name("RS_motor"), None);
        assert_eq!(MotorModel::classify_build_name(""), None);
    }

    /// The build name is not a model designation, and must not be read as one.
    #[test]
    fn find_in_does_not_turn_a_build_name_into_a_model() {
        assert_eq!(MotorModel::find_in("EL_motor"), None);
    }

    /// The case this whole check exists for, with the values a real RS-04
    /// actually reports (`limit_torque` 115 N·m, `limit_spd` 1 rad/s, read
    /// 2026-08-02) against the RS-05 that used to be the silent default.
    ///
    /// 115 N·m is decisive rather than merely suggestive: only the RS-04
    /// quantises torque far enough to express it, so the refutation test
    /// happens to leave exactly one survivor.
    #[test]
    fn the_measured_rs04_limits_identify_it_uniquely() {
        let seen = ReportedLimits {
            limit_torque: Some(115.0),
            limit_spd: Some(1.0),
        };
        assert!(!seen.consistent_with(MotorModel::Rs05));
        assert!(seen.consistent_with(MotorModel::Rs04));
        let mut only = seen.candidates();
        assert_eq!(only.next(), Some(MotorModel::Rs04));
        assert_eq!(only.next(), None, "115 N-m should fit only the RS-04");
    }

    /// And the EduLite05's measured `limit_torque` of 6 N·m must not trip the
    /// same check against its own model — the manual rounds 5.5 up to 6.
    #[test]
    fn the_measured_edulite_limits_do_not_refute_it() {
        let seen = ReportedLimits {
            limit_torque: Some(6.0),
            limit_spd: Some(50.0),
        };
        assert!(seen.consistent_with(MotorModel::Rs05));
    }

    /// Manual rounding must not produce false alarms: the RS-04 lists
    /// `limit_spd` up to 20 rad/s against a 15 rad/s MIT scale, and the EL05
    /// lists `limit_torque` up to 6 N·m against 5.5.
    #[test]
    fn manual_rounding_does_not_refute_the_matching_model() {
        assert!(ReportedLimits {
            limit_torque: Some(120.0),
            limit_spd: Some(20.0),
        }
        .consistent_with(MotorModel::Rs04));
        assert!(ReportedLimits {
            limit_torque: Some(6.0),
            limit_spd: Some(50.0),
        }
        .consistent_with(MotorModel::Rs05));
    }

    #[test]
    fn a_reading_that_is_missing_refutes_nothing() {
        let none = ReportedLimits::default();
        assert!(none.is_empty());
        for &m in MotorModel::ALL {
            assert!(none.consistent_with(m), "{m} refuted by no evidence");
        }
        assert_eq!(none.candidates().count(), MotorModel::ALL.len());
    }

    #[test]
    fn candidates_narrow_as_the_limits_rise() {
        // Only RS-04 quantises torque past 60 N·m.
        let big = ReportedLimits {
            limit_torque: Some(100.0),
            limit_spd: None,
        };
        let mut only = big.candidates();
        assert_eq!(only.next(), Some(MotorModel::Rs04));
        assert_eq!(only.next(), None);

        // Nothing reaches 500 N·m, so the readings fit no known model rather
        // than silently picking the largest.
        let absurd = ReportedLimits {
            limit_torque: Some(500.0),
            limit_spd: None,
        };
        assert_eq!(absurd.candidates().count(), 0);
    }

    #[test]
    fn rs05_scales() {
        // Confirmed against the official RS05-EN and EL05-EN manuals
        // (2025-11-12 rev): V_MAX=50.0 rad/s, T_MAX=5.5 N·m.
        let s = MitScales::for_model(MotorModel::Rs05);
        assert!((s.velocity - 50.0).abs() < f32::EPSILON);
        assert!((s.torque - 5.5).abs() < f32::EPSILON);
    }
}
