//! Per-model quantization limits (the DAMIAO SDK `Limit_Param` table).
//!
//! The MIT protocol quantizes each field over a symmetric range
//! `[-max, +max]` (position/velocity/torque) or `[0, max]` (kp/kd). The
//! position range is `±12.5 rad` and `kp ∈ [0,500]`, `kd ∈ [0,5]` for every
//! DAMIAO model; only the velocity and torque maxima vary per model.

/// Which register-map layout a model uses above RID `0x25`.
///
/// The common block `0x00`–`0x24` is **byte-for-byte identical across all nine
/// documented DM-J models**, but above it the maps split into three distinct
/// layouts. Grouping models by layout (rather than listing each model's
/// addresses separately) is what keeps [`crate::ModelReg`] tractable: adding a
/// new model is a one-line mapping to an existing layout.
///
/// The split does **not** follow motor size — DM-J8009 (20 N·m rated) shares a
/// layout with DM-J3507 (the smallest), while the smaller DM-J4340 sits in the
/// other group — so it appears to track driver-board hardware rather than the
/// motor itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegisterLayout {
    /// DM-J4310 / 4310P / 4340 / 4340P.
    ///
    /// Has `Boot_ver` (`0x25`), `m_off` at `0x38`, and the diagnostics block
    /// `Imax`/`VBus`/`Tpcb`/`Tmtr`/`I_{U,V,W}_OFF` at `0x3B`–`0x41`. Has no
    /// `u_off`/`v_off`/`k1`/`k2`.
    J4310,
    /// DM-J3507 / 6248P / 8009 / 8009P.
    ///
    /// Has `u_off`/`v_off`/`k1`/`k2` at `0x32`–`0x35` and `m_off` at `0x36`
    /// (**not** `0x38`). Has no `Boot_ver` and no `0x3B`–`0x41` diagnostics
    /// block.
    J3507,
    /// DM-J10422P — [`Self::J4310`] plus `x_off` (`0x36`, output-shaft angle
    /// offset), and with the `0x3B`–`0x3E` diagnostics renamed: `IBase`
    /// ("phase current base peak value", a different quantity from `Imax`),
    /// `TMOS`, `T_CEL`.
    ///
    /// Note `0x36` is `x_off` here but `m_off` under [`Self::J3507`] — the
    /// same address means different things across layouts.
    J10422,
}

/// Where a model's [`Limits`] values came from.
///
/// The official manuals define `PMAX`/`VMAX`/`TMAX` only as *writable
/// registers* with range `(0.0, fmax]` — **no manual states concrete MIT
/// mapping ranges for any model**. The vendor SDK's `Limit_Param` table is the
/// only published source, and it contradicts itself across its own four
/// copies (see `doc/dm-j-series-cross-model-reference.md`).
///
/// Treat every value here as a *default*. The authoritative ranges live in the
/// motor's own `PMAX`/`VMAX`/`TMAX` registers — read them with
/// `DamiaoMotor::refresh_limits_from_registers()`, which is what the vendor
/// SDK itself does via its `changeMotorLimit` hook.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimitsSource {
    /// Taken from the vendor SDK's `Limit_Param` table, with all four SDK
    /// copies agreeing.
    Sdk,
    /// Taken from the SDK entry for the sibling model without the `P` suffix.
    /// The `P` variants are absent from every SDK copy, but their manuals list
    /// **identical** rated/peak torque, rated/no-load speed and reduction
    /// ratio to the non-`P` sibling, so the sibling's ranges apply.
    SdkSiblingIdenticalSpecs,
    /// No SDK entry and no sibling: derived from the manual's specification
    /// table as a floor (`t_max` = peak torque, `v_max` = max no-load speed,
    /// `p_max` = 12.5 as used by every other model). **Most likely wrong** —
    /// the SDK's `TMAX` values sit well above peak torque on models where both
    /// are known, so read the registers before relying on MIT scaling.
    ManualSpecsFloor,
}

/// A DAMIAO motor model, one variant per official DM-J series manual in
/// `ref/`.
///
/// Kp/Kd quantization ranges are global across the family (`[0,500]` /
/// `[0,5]`); only P/V/T differ per model — and those are register-backed, so
/// see [`LimitsSource`] before trusting [`Self::limits`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorModel {
    /// DM-J4310-2EC V1.2.
    Dm4310,
    /// DM-J4310P-2EC V1.1 — same specs as [`Self::Dm4310`].
    Dm4310P,
    /// DM-J4340-2EC V1.1 — 14 N·m rated / 40 N·m peak, 1:40.
    Dm4340,
    /// DM-J4340P-2EC V1.1 — same specs as [`Self::Dm4340`].
    Dm4340P,
    /// DM-J3507-2EC (small low-inertia joint).
    Dm3507,
    /// DM-J6248P-2EC — 30 N·m rated / 97 N·m peak, 48:1.
    Dm6248P,
    /// DM-J8009-2EC — 20 N·m rated / 40 N·m peak, 9:1.
    Dm8009,
    /// DM-J8009P-2EC — same specs as [`Self::Dm8009`].
    Dm8009P,
    /// DM-J10422P-2EC — 100 N·m rated / 400 N·m peak, 1:22, 48 V.
    Dm10422P,
}

impl MotorModel {
    /// Parse a model name. Accepts the canonical name, the full part number,
    /// or just the digits, case-insensitively — e.g. `"DM4310"`,
    /// `"dm-j4310-2ec"`, `"4310"`, `"DM4310P"`, `"dm-j10422p-2ec"`.
    ///
    /// `P` variants are matched before their non-`P` siblings, since
    /// `"dm-j4310p-2ec"` also contains `"4310"`.
    pub fn from_name(s: &str) -> Option<Self> {
        let key = LowerBuf::new(s);
        // Longest/most-specific keys first: "4310p" must win over "4310".
        if key.contains("10422") {
            Some(MotorModel::Dm10422P)
        } else if key.contains("4310p") {
            Some(MotorModel::Dm4310P)
        } else if key.contains("4340p") {
            Some(MotorModel::Dm4340P)
        } else if key.contains("8009p") {
            Some(MotorModel::Dm8009P)
        } else if key.contains("4310") {
            Some(MotorModel::Dm4310)
        } else if key.contains("4340") {
            Some(MotorModel::Dm4340)
        } else if key.contains("8009") {
            Some(MotorModel::Dm8009)
        } else if key.contains("6248") {
            Some(MotorModel::Dm6248P)
        } else if key.contains("3507") {
            Some(MotorModel::Dm3507)
        } else {
            None
        }
    }

    /// Every model, for CLI help text and exhaustive dumps.
    pub const ALL: &'static [MotorModel] = &[
        MotorModel::Dm4310,
        MotorModel::Dm4310P,
        MotorModel::Dm4340,
        MotorModel::Dm4340P,
        MotorModel::Dm3507,
        MotorModel::Dm6248P,
        MotorModel::Dm8009,
        MotorModel::Dm8009P,
        MotorModel::Dm10422P,
    ];

    /// Canonical short name.
    pub const fn name(&self) -> &'static str {
        match self {
            MotorModel::Dm4310 => "DM4310",
            MotorModel::Dm4310P => "DM4310P",
            MotorModel::Dm4340 => "DM4340",
            MotorModel::Dm4340P => "DM4340P",
            MotorModel::Dm3507 => "DM3507",
            MotorModel::Dm6248P => "DM6248P",
            MotorModel::Dm8009 => "DM8009",
            MotorModel::Dm8009P => "DM8009P",
            MotorModel::Dm10422P => "DM10422P",
        }
    }

    /// The reduction ratio, which is also the last digits of the part number.
    ///
    /// From the manuals' specification tables (see
    /// `doc/dm-j-series-cross-model-reference.md`). `P` variants match their
    /// siblings exactly.
    pub const fn gear_ratio(&self) -> u16 {
        match self {
            MotorModel::Dm3507 => 7,
            MotorModel::Dm4310 | MotorModel::Dm4310P => 10,
            MotorModel::Dm4340 | MotorModel::Dm4340P => 40,
            MotorModel::Dm6248P => 48,
            MotorModel::Dm8009 | MotorModel::Dm8009P => 9,
            MotorModel::Dm10422P => 22,
        }
    }

    /// Identify the model from the reduction ratio the motor reports in `GR`
    /// (RID 20).
    ///
    /// **This is model identification over the bus, and it is exact.** Every
    /// distinct DM-J motor has a distinct reduction ratio, and the ratio is a
    /// read-only constant the firmware reports about itself — not a setting
    /// that can drift.
    ///
    /// The one ambiguity is the `P` suffix, which shares a ratio with its
    /// sibling. That is not a real ambiguity for anything the driver does: the
    /// manuals list identical torque, speed and ratio for each pair, and both
    /// members share a register layout, so this returns the non-`P` variant
    /// and every derived value is correct either way.
    ///
    /// Rounds, because `GR` comes back as an `f32` — a real DM4310 reports
    /// exactly `10`, but a ratio is an integer and float equality is the wrong
    /// test for one.
    pub fn from_gear_ratio(gr: f32) -> Option<Self> {
        if !gr.is_finite() || gr <= 0.0 {
            return None;
        }
        let want = (gr + 0.5) as u16;
        // Prefer the non-`P` sibling when there is one, but fall back to a `P`
        // variant — `DM6248P` and `DM10422P` have no non-`P` counterpart, and
        // skipping them outright would leave two real motors unidentifiable.
        let mut fallback = None;
        let mut i = 0;
        while i < Self::ALL.len() {
            let m = Self::ALL[i];
            if m.gear_ratio() == want {
                if !m.name().ends_with('P') {
                    return Some(m);
                }
                if fallback.is_none() {
                    fallback = Some(m);
                }
            }
            i += 1;
        }
        fallback
    }

    /// Which register-map layout this model uses above RID `0x25`.
    pub const fn register_layout(&self) -> RegisterLayout {
        match self {
            MotorModel::Dm4310
            | MotorModel::Dm4310P
            | MotorModel::Dm4340
            | MotorModel::Dm4340P => RegisterLayout::J4310,
            MotorModel::Dm3507
            | MotorModel::Dm6248P
            | MotorModel::Dm8009
            | MotorModel::Dm8009P => RegisterLayout::J3507,
            MotorModel::Dm10422P => RegisterLayout::J10422,
        }
    }

    /// Whether this model's register map documents `Boot_ver` (RID `0x25`).
    ///
    /// The only register in the otherwise-common `0x00`–`0x25` block that is
    /// not universal: the J3507-layout models' maps jump straight from `0x24`
    /// to `0x32`. Reading [`crate::Rid::BOOT_VER`] on those models targets an
    /// undocumented address.
    pub const fn has_boot_ver(&self) -> bool {
        match self.register_layout() {
            RegisterLayout::J4310 | RegisterLayout::J10422 => true,
            RegisterLayout::J3507 => false,
        }
    }

    /// Whether this model honours the NVM "set zero" magic frame (`FF..FE`),
    /// which writes the zero offset to the motor's flash.
    ///
    /// `true` for every model: all nine official DM-J manuals document the
    /// frame identically. (Previously this was an SDK-derived assumption; the
    /// manuals now confirm it.) Kept as a method so a future model that drops
    /// the frame can be expressed without touching call sites.
    ///
    /// Drivers gate `set_zero_nvm` on this so that issuing it to a motor that
    /// does not support it fails with a clear "unsupported" error rather than
    /// silently doing nothing.
    pub const fn supports_nvm_zero(&self) -> bool {
        match self {
            MotorModel::Dm4310
            | MotorModel::Dm4310P
            | MotorModel::Dm4340
            | MotorModel::Dm4340P
            | MotorModel::Dm3507
            | MotorModel::Dm6248P
            | MotorModel::Dm8009
            | MotorModel::Dm8009P
            | MotorModel::Dm10422P => true,
        }
    }

    /// Default quantization limits for this model — see [`LimitsSource`] for
    /// how much to trust them, and prefer reading the motor's own
    /// `PMAX`/`VMAX`/`TMAX` registers.
    pub const fn limits(&self) -> Limits {
        Limits::for_model(*self)
    }

    /// Provenance of [`Self::limits`] for this model.
    pub const fn limits_source(&self) -> LimitsSource {
        match self {
            MotorModel::Dm4310
            | MotorModel::Dm4340
            | MotorModel::Dm3507
            | MotorModel::Dm6248P
            | MotorModel::Dm8009 => LimitsSource::Sdk,
            MotorModel::Dm4310P | MotorModel::Dm4340P | MotorModel::Dm8009P => {
                LimitsSource::SdkSiblingIdenticalSpecs
            }
            MotorModel::Dm10422P => LimitsSource::ManualSpecsFloor,
        }
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
    /// Kp/Kd quantization ranges, global across every DAMIAO model.
    const KP_MAX: f32 = 500.0;
    const KD_MAX: f32 = 5.0;

    /// Default P/V/T ranges for `model`. See [`LimitsSource`] — these are
    /// defaults, not ground truth; the motor's `PMAX`/`VMAX`/`TMAX` registers
    /// are authoritative.
    pub const fn for_model(model: MotorModel) -> Self {
        // (p_max, v_max, t_max)
        let (p_max, v_max, t_max) = match model {
            // SDK `Limit_Param`, all four copies agreeing.
            MotorModel::Dm4310 => (12.5, 30.0, 10.0),
            // 14 N·m rated / 40 N·m peak. **Note**: the SDK's own copies
            // disagree on v_max here (u2can C++ says 8, the other three say
            // 10); 10 is the majority and matches both Python copies.
            MotorModel::Dm4340 => (12.5, 10.0, 28.0),
            // P_MAX is 4π (≈12.566) rather than 12.5 on this model.
            MotorModel::Dm3507 => (12.566, 50.0, 5.0),
            MotorModel::Dm6248P => (12.566, 20.0, 120.0),
            MotorModel::Dm8009 => (12.5, 45.0, 54.0),

            // `P` variants: absent from every SDK copy, but their manuals
            // list identical specs to the non-`P` sibling, so the sibling's
            // ranges carry over.
            MotorModel::Dm4310P => (12.5, 30.0, 10.0),
            MotorModel::Dm4340P => (12.5, 10.0, 28.0),
            MotorModel::Dm8009P => (12.5, 45.0, 54.0),

            // No SDK entry and no sibling. Floor derived from the manual's
            // spec table: 400 N·m peak torque, 120 rpm max no-load speed
            // (= 4π rad/s ≈ 12.57). Almost certainly not the firmware's
            // actual TMAX/VMAX — read the registers.
            MotorModel::Dm10422P => (12.5, 12.6, 400.0),
        };
        Self {
            p_max,
            v_max,
            t_max,
            kp_max: Self::KP_MAX,
            kd_max: Self::KD_MAX,
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

    /// The reduction ratio is what makes over-the-bus identification work, so
    /// it has to stay unique across distinct motors. If a future model
    /// duplicates one, `from_gear_ratio` silently starts answering with the
    /// wrong motor.
    #[test]
    fn every_distinct_model_has_its_own_gear_ratio() {
        for &a in MotorModel::ALL {
            for &b in MotorModel::ALL {
                if a == b || a.gear_ratio() != b.gear_ratio() {
                    continue;
                }
                // The only permitted collision is a P/non-P pair, which the
                // manuals give identical specs and a shared register layout.
                assert_eq!(
                    a.name().trim_end_matches('P'),
                    b.name().trim_end_matches('P'),
                    "{} and {} share a ratio but are different motors",
                    a.name(),
                    b.name()
                );
                assert_eq!(a.limits(), b.limits());
                assert_eq!(a.register_layout(), b.register_layout());
            }
        }
    }

    #[test]
    fn a_reported_gear_ratio_identifies_the_model() {
        // Measured on a real DM-J4310-2EC over CAN-FD: GR (RID 20) = 10.
        assert_eq!(MotorModel::from_gear_ratio(10.0), Some(MotorModel::Dm4310));

        // Every model must be reachable, including the two that exist only as
        // `P` variants (DM6248P, DM10422P) and so have no sibling to fall back
        // to.
        for &m in MotorModel::ALL {
            let found = MotorModel::from_gear_ratio(m.gear_ratio() as f32)
                .unwrap_or_else(|| panic!("{} has no match", m.name()));
            // A `P` motor may resolve to its sibling, which is the same machine.
            assert_eq!(found.limits(), m.limits(), "{}", m.name());
            assert_eq!(found.register_layout(), m.register_layout(), "{}", m.name());
        }
        assert_eq!(
            MotorModel::from_gear_ratio(48.0),
            Some(MotorModel::Dm6248P),
            "a P-only model must still be identifiable"
        );
        assert_eq!(MotorModel::from_gear_ratio(22.0), Some(MotorModel::Dm10422P));
    }

    #[test]
    fn an_implausible_gear_ratio_is_not_forced_onto_a_model() {
        assert_eq!(MotorModel::from_gear_ratio(0.0), None);
        assert_eq!(MotorModel::from_gear_ratio(-1.0), None);
        assert_eq!(MotorModel::from_gear_ratio(f32::NAN), None);
        assert_eq!(MotorModel::from_gear_ratio(100.0), None);
    }

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

    /// A `P` part number contains its non-`P` sibling's digits, so ordering in
    /// `from_name` decides the answer. Pin it.
    #[test]
    fn parse_names_prefers_p_variants_over_their_siblings() {
        assert_eq!(MotorModel::from_name("DM4310P"), Some(MotorModel::Dm4310P));
        assert_eq!(
            MotorModel::from_name("dm-j4310p-2ec"),
            Some(MotorModel::Dm4310P)
        );
        assert_eq!(MotorModel::from_name("DM4340P"), Some(MotorModel::Dm4340P));
        assert_eq!(MotorModel::from_name("DM8009P"), Some(MotorModel::Dm8009P));
        // Non-P siblings must still resolve to the non-P variant.
        assert_eq!(MotorModel::from_name("dm-j4310-2ec"), Some(MotorModel::Dm4310));
        assert_eq!(MotorModel::from_name("dm-j4340-2ec"), Some(MotorModel::Dm4340));
        assert_eq!(MotorModel::from_name("dm-j8009-2ec"), Some(MotorModel::Dm8009));
    }

    /// Every model in `ALL` must round-trip through its own canonical name,
    /// so a new variant cannot be added without a matching `from_name` arm.
    #[test]
    fn every_model_round_trips_through_its_canonical_name() {
        for &model in MotorModel::ALL {
            assert_eq!(
                MotorModel::from_name(model.name()),
                Some(model),
                "{} does not round-trip",
                model.name()
            );
        }
        // `ALL` must be complete and duplicate-free.
        assert_eq!(MotorModel::ALL.len(), 9);
        for (i, a) in MotorModel::ALL.iter().enumerate() {
            for b in &MotorModel::ALL[i + 1..] {
                assert_ne!(a, b, "duplicate entry in ALL");
            }
        }
    }

    /// The full part numbers as printed on the nine manuals must all parse.
    #[test]
    fn parse_full_part_numbers_from_the_manuals() {
        let cases = [
            ("DM-J4310-2EC", MotorModel::Dm4310),
            ("DM-J4310P-2EC", MotorModel::Dm4310P),
            ("DM-J4340-2EC", MotorModel::Dm4340),
            ("DM-J4340P-2EC", MotorModel::Dm4340P),
            ("DM-J3507-2EC", MotorModel::Dm3507),
            ("DM-J6248P-2EC", MotorModel::Dm6248P),
            ("DM-J8009-2EC", MotorModel::Dm8009),
            ("DM-J8009P-2EC", MotorModel::Dm8009P),
            ("DM-J10422P-2EC", MotorModel::Dm10422P),
        ];
        for (part_number, expected) in cases {
            assert_eq!(
                MotorModel::from_name(part_number),
                Some(expected),
                "{part_number}"
            );
        }
    }

    /// Kp/Kd are global across the family; only P/V/T vary.
    #[test]
    fn kp_kd_ranges_are_global() {
        for &model in MotorModel::ALL {
            let l = model.limits();
            assert_eq!(l.kp_max, 500.0, "{}", model.name());
            assert_eq!(l.kd_max, 5.0, "{}", model.name());
            assert!(l.p_max > 0.0 && l.v_max > 0.0 && l.t_max > 0.0);
        }
    }

    /// `P` variants inherit their sibling's ranges, which their manuals
    /// justify by listing identical specs.
    #[test]
    fn p_variants_share_their_siblings_limits() {
        for (p, sibling) in [
            (MotorModel::Dm4310P, MotorModel::Dm4310),
            (MotorModel::Dm4340P, MotorModel::Dm4340),
            (MotorModel::Dm8009P, MotorModel::Dm8009),
        ] {
            assert_eq!(p.limits(), sibling.limits(), "{}", p.name());
            assert_eq!(p.limits_source(), LimitsSource::SdkSiblingIdenticalSpecs);
            assert_eq!(sibling.limits_source(), LimitsSource::Sdk);
        }
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
