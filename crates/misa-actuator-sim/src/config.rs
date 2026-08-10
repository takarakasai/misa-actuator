//! Simulator configuration, with presets calibrated against real bench runs.
//!
//! The numbers in [`SimConfig::dm4310`] and [`SimConfig::rs04`] come from
//! `doc/bench-measurements-2026-07-30.md` — the same measurements that drove
//! 16 bug fixes in `misa-sysid`. Reproducing them here means a characterization
//! run against the simulator has a *known right answer*, which is something no
//! real motor can offer.
//!
//! Values that were measured are marked as such. Values that were not are
//! plausible estimates and say so — do not treat them as data.

use std::time::Duration;

/// How the shaft is loaded, beyond the motor's own rotor.
#[derive(Debug, Clone, Copy)]
pub struct Load {
    /// Linear restoring stiffness about [`Self::equilibrium_rad`] (N·m/rad).
    pub spring_nm_per_rad: f32,
    /// Where the linear spring is relaxed (rad, absolute shaft frame).
    pub equilibrium_rad: f32,
    /// Pendulum-style load: torque is `-gravity_nm * sin(pos)`.
    pub gravity_nm: f32,
    /// Beyond `hard_knee_rad` from equilibrium the mechanism starts binding,
    /// adding `hard_stiffness * (excess)^2`. The DM-J4310 rig did exactly this
    /// past ~0.14 rad, and that binding is what made its thermal test possible.
    pub hard_knee_rad: f32,
    pub hard_stiffness_nm_per_rad2: f32,
}

impl Load {
    /// A free shaft: nothing but the motor's own inertia and friction.
    pub const FREE: Load = Load {
        spring_nm_per_rad: 0.0,
        equilibrium_rad: 0.0,
        gravity_nm: 0.0,
        hard_knee_rad: f32::INFINITY,
        hard_stiffness_nm_per_rad2: 0.0,
    };

    /// Torque the load applies to the shaft at position `pos_rad`. Negative
    /// means it opposes positive motion.
    pub fn torque_at(&self, pos_rad: f32) -> f32 {
        let delta = pos_rad - self.equilibrium_rad;
        let mut tau = -self.spring_nm_per_rad * delta - self.gravity_nm * pos_rad.sin();
        let excess = delta.abs() - self.hard_knee_rad;
        if excess > 0.0 {
            tau -= self.hard_stiffness_nm_per_rad2 * excess * excess * delta.signum();
        }
        tau
    }
}

/// Coulomb + viscous friction with a distinct breakaway (static) level.
///
/// The static > kinetic ordering is what `misa-sysid`'s `breakaway` measures;
/// getting it wrong in the simulator would make the tool untestable.
#[derive(Debug, Clone, Copy)]
pub struct Friction {
    /// Torque needed to start a stationary shaft moving (N·m).
    pub stiction_nm: f32,
    /// Torque opposing an already-moving shaft (N·m), sign-opposed to velocity.
    pub kinetic_nm: f32,
    /// Velocity-proportional damping (N·m·s/rad).
    pub viscous_nm_s_per_rad: f32,
}

/// Lumped thermal model: `dT/dt = heating * τ² − (T − ambient) / cooling_tau`.
///
/// Parameterising on torque rather than I²R avoids committing to a copper-loss
/// convention, and torque is what the measurement actually recorded.
#[derive(Debug, Clone, Copy)]
pub struct Thermal {
    pub ambient_c: f32,
    /// °C/s per (N·m)². Calibrate from one measured (torque, rise-rate) point.
    pub heating_c_per_s_per_nm2: f32,
    /// Newton cooling time constant (s).
    pub cooling_tau_s: f32,
}

impl Thermal {
    /// Derive the heating coefficient from a measured operating point.
    ///
    /// `doc/bench-measurements-2026-07-30.md` §1.7: the DM-J4310 delivered
    /// 1.218 N·m and warmed at 0.116 °C/s.
    pub fn from_measured_point(torque_nm: f32, rise_c_per_s: f32) -> Self {
        Self {
            ambient_c: 25.0,
            heating_c_per_s_per_nm2: rise_c_per_s / (torque_nm * torque_nm),
            cooling_tau_s: 1_000.0,
        }
    }
}

/// The motor firmware's own control loops, which the simulator emulates so
/// that `set_position` / `set_velocity` behave like a real servo rather than
/// teleporting the shaft.
///
/// These run at the plant's substep rate, not at the rate the host issues
/// commands — see [`crate::plant::Plant::step`].
#[derive(Debug, Clone, Copy)]
pub struct Gains {
    /// Outer position loop: position error → velocity reference (1/s).
    pub position_p: f32,
    /// Inner velocity loop: velocity error → torque (N·m·s/rad).
    pub velocity_p: f32,
    /// Integral term on the velocity loop (N·m/rad), needed to hold against
    /// friction without steady-state droop.
    pub velocity_i: f32,
}

/// Velocity-loop bandwidth the presets are tuned to (rad/s).
///
/// 300 rad/s ≈ 48 Hz, which is what the bench DM-J4310 actually reported for
/// its own velocity loop (`V_BW` = 50, §1.3). The position gains that fall out
/// of it — 30 by the `ω/10` rule — land in the same range as the two motors'
/// recorded values (`KP_APR` 54 on the DM-J4310, `loc_kp` 80 on the RS04),
/// which is a reassuring independent check on the rule.
///
/// Well inside the integrator's stability limit: at the plant's 0.5 ms
/// substep, `ω·h` is 0.15.
pub const DEFAULT_VELOCITY_BANDWIDTH: f32 = 300.0;

impl Gains {
    /// Gains sized for a given rotor and velocity-loop bandwidth.
    ///
    /// Hand-picking these per preset invites the mistake of pairing a light
    /// rotor with gains meant for a heavy one, which shows up as a control
    /// loop that oscillates rather than as an obviously wrong number. Deriving
    /// them from `J` keeps every preset consistently damped:
    ///
    /// - `velocity_p = J·ω` puts the velocity loop's pole at ω
    /// - `velocity_i = J·ω²/4` keeps it (over)damped rather than ringing
    /// - `position_p = ω/10` leaves the outer loop a decade below the inner
    ///   one, the usual cascade separation
    ///
    /// The bandwidth has to be chosen against the *friction*, not just the
    /// rotor. A loop tuned for an unloaded inertia develops torque far too
    /// slowly to break a stiction band: at 30 rad/s the RS04 preset needed
    /// most of a second to wind up to its 0.9 N·m breakaway, so a position
    /// chirp never moved the shaft at all — while the real unit tracks a
    /// ±0.6 rad sweep to ±0.0004 rad (bench notes §2.4). See
    /// [`DEFAULT_VELOCITY_BANDWIDTH`].
    pub fn for_inertia(inertia_kg_m2: f32, velocity_bandwidth_rad_s: f32) -> Self {
        let w = velocity_bandwidth_rad_s;
        Self {
            position_p: w / 10.0,
            velocity_p: inertia_kg_m2 * w,
            velocity_i: inertia_kg_m2 * w * w / 4.0,
        }
    }
}

/// Sensing imperfections, so a GUI is developed against realistic-looking
/// signals rather than mathematically clean ones.
#[derive(Debug, Clone, Copy)]
pub struct Sensing {
    /// Encoder step (rad). 0 disables quantization.
    pub position_step_rad: f32,
    /// Peak velocity noise (rad/s). The bench DM-J4310 idled near 0.02 rad/s,
    /// which is why `breakaway --motion-speed` defaults above it.
    pub velocity_noise_rad_s: f32,
    /// Peak torque-reading noise (N·m).
    pub torque_noise_nm: f32,
    /// If `false`, `current_a` reads NaN — matching RobStride, whose feedback
    /// frame carries no current at all (§2.6 of the bench notes).
    pub reports_current: bool,
    /// If `false`, `torque_nm` reads 0 outside MIT mode — reproducing the RS04
    /// unit whose `MeasuredTorque` register was stuck at zero (§2.3).
    pub reports_torque_outside_mit: bool,
}

impl Default for Sensing {
    fn default() -> Self {
        Self {
            position_step_rad: 0.0,
            velocity_noise_rad_s: 0.0,
            torque_noise_nm: 0.0,
            reports_current: true,
            reports_torque_outside_mit: true,
        }
    }
}

/// Deliberate misbehaviour, for exercising error paths without unplugging
/// anything.
#[derive(Debug, Clone, Copy, Default)]
pub struct Faults {
    /// Every Nth bus transaction times out. 0 disables.
    pub timeout_every_n: u32,
    /// Report an over-temperature fault above this (°C). `None` disables.
    pub over_temperature_c: Option<f32>,
    /// Refuse every command — as if the motor were unpowered.
    pub dead: bool,
}

#[derive(Debug, Clone)]
pub struct SimConfig {
    /// Human-readable name, surfaced in logs and the GUI.
    pub name: String,
    /// Output-shaft inertia (kg·m²).
    pub inertia_kg_m2: f32,
    /// Output-shaft torque constant (N·m/A), used to report current.
    pub torque_constant_nm_per_a: f32,
    pub friction: Friction,
    pub load: Load,
    pub thermal: Thermal,
    pub gains: Gains,
    pub sensing: Sensing,
    pub faults: Faults,
    /// Firmware torque ceiling (N·m).
    pub torque_limit_nm: f32,
    /// Firmware speed ceiling (rad/s).
    pub velocity_limit_rad_s: f32,
    /// Absolute travel limits (rad). `None` = unlimited.
    pub position_limit_rad: Option<(f32, f32)>,
    /// Bus supply voltage reported by `read_status`.
    pub bus_voltage_v: f32,
    /// Simulated round-trip time of one bus transaction. Set to zero in tests.
    pub bus_latency: Duration,
    /// Motor ids `scan_bus` should report as present, so scan UIs have
    /// something to find.
    pub present_ids: Vec<u8>,
}

impl SimConfig {
    /// A frictionless, unloaded, instant-bus plant. The right baseline for
    /// unit tests: every deviation from ideal is something the test asked for.
    pub fn ideal() -> Self {
        Self {
            name: "ideal".to_string(),
            inertia_kg_m2: 1.0e-3,
            torque_constant_nm_per_a: 1.0,
            friction: Friction {
                stiction_nm: 0.0,
                kinetic_nm: 0.0,
                viscous_nm_s_per_rad: 0.01,
            },
            load: Load::FREE,
            thermal: Thermal {
                ambient_c: 25.0,
                heating_c_per_s_per_nm2: 0.0,
                cooling_tau_s: 1_000.0,
            },
            // 50 rad/s velocity loop on a 1e-3 kg·m² rotor.
            gains: Gains::for_inertia(1.0e-3, DEFAULT_VELOCITY_BANDWIDTH),
            sensing: Sensing::default(),
            faults: Faults::default(),
            torque_limit_nm: 10.0,
            velocity_limit_rad_s: 30.0,
            position_limit_rad: None,
            bus_voltage_v: 24.0,
            bus_latency: Duration::ZERO,
            present_ids: vec![1],
        }
    }

    /// DAMIAO DM-J4310-2EC on the 2026-07-30 bench rig.
    ///
    /// Measured (bench notes §1.1, §1.6, §1.7):
    /// - `Kt` 0.94324 N·m/A output-shaft, from `1.5·Npp·ψf·GR·GREF`
    /// - rotor inertia 1.7915e-5 and `Damp` 5.2233e-4, both motor-side; the
    ///   1:10 gearbox scales each by `Gr²` to reach the output shaft
    /// - stiction 0.175 N·m, kinetic 0.126 N·m (ratio 1.39)
    /// - `TMAX` 10 N·m, `VMAX` 30 rad/s, `PMAX` 12.5 rad, bus 24.10 V
    /// - warmed 0.116 °C/s at 1.218 N·m delivered
    ///
    /// Estimated: the binding past ±0.14 rad is fitted as a quadratic to the
    /// two recorded points (0.945 N·m at 0.138 rad, 6.7 N·m at 0.201 rad) and
    /// lands within ~12 % of both — enough to reproduce the *behaviour* that
    /// made the thermal run hold torque, not a validated stiffness curve.
    pub fn dm4310() -> Self {
        const GEAR: f32 = 10.0;
        Self {
            name: "DM-J4310-2EC (sim)".to_string(),
            inertia_kg_m2: 1.7915e-5 * GEAR * GEAR,
            torque_constant_nm_per_a: 0.94324,
            friction: Friction {
                stiction_nm: 0.175,
                kinetic_nm: 0.126,
                viscous_nm_s_per_rad: 5.2233e-4 * GEAR * GEAR,
            },
            load: Load {
                // Near equilibrium the rig was friction-dominated: breakaway
                // put the static load at only 0.009 N·m.
                spring_nm_per_rad: 0.075,
                equilibrium_rad: 0.0,
                gravity_nm: 0.0,
                hard_knee_rad: 0.10,
                hard_stiffness_nm_per_rad2: 600.0,
            },
            thermal: Thermal::from_measured_point(1.218, 0.116),
            gains: Gains::for_inertia(1.7915e-5 * GEAR * GEAR, DEFAULT_VELOCITY_BANDWIDTH),
            sensing: Sensing {
                // 16-bit over ±PMAX.
                position_step_rad: 2.0 * 12.5 / 65_536.0,
                velocity_noise_rad_s: 0.02,
                torque_noise_nm: 0.004,
                reports_current: true,
                reports_torque_outside_mit: true,
            },
            faults: Faults {
                // `OT` register read back as 100 °C.
                over_temperature_c: Some(100.0),
                ..Faults::default()
            },
            torque_limit_nm: 10.0,
            velocity_limit_rad_s: 30.0,
            position_limit_rad: Some((-12.5, 12.5)),
            bus_voltage_v: 24.10,
            bus_latency: Duration::from_millis(1),
            present_ids: vec![16],
        }
    }

    /// RobStride RS04 on the 2026-07-30 bench, shaft free.
    ///
    /// Measured (bench notes §2.1, §2.2, §7.4):
    /// - `Kt` 1.5093 N·m/A by least squares (R² 0.998) — the one genuinely
    ///   independent Kt measurement of the four families
    /// - stiction 0.902 N·m, kinetic 0.530 N·m (ratio 1.70), after both the
    ///   unit-conversion and judder bugs were fixed
    /// - friction was velocity-independent across 0.08…1.0 rad/s, so viscous
    ///   damping is essentially zero
    /// - `limit_torque` 115 N·m, `vel_max` 10 rad/s, bus 23.91 V, motor 31.0 °C
    ///
    /// Estimated: inertia (never measured) and the thermal coefficients — §2.5
    /// records that the thermal run *failed* on this motor precisely because
    /// the shaft was free. The free-shaft load here reproduces that failure,
    /// which is the useful part.
    pub fn rs04() -> Self {
        Self {
            name: "RobStride RS04 (sim)".to_string(),
            inertia_kg_m2: 1.0e-2,
            torque_constant_nm_per_a: 1.5093,
            friction: Friction {
                stiction_nm: 0.902,
                kinetic_nm: 0.530,
                viscous_nm_s_per_rad: 0.005,
            },
            load: Load::FREE,
            thermal: Thermal {
                ambient_c: 31.0,
                heating_c_per_s_per_nm2: 0.01,
                cooling_tau_s: 1_500.0,
            },
            // A 30 rad/s loop, slower than the lighter motors. The real
            // unit ran `loc_kp` 80 and hunted against its 0.9 N·m of friction
            // (§2.3); this is a usable simulator, not a claim about that
            // firmware's tuning.
            gains: Gains::for_inertia(1.0e-2, DEFAULT_VELOCITY_BANDWIDTH),
            sensing: Sensing {
                position_step_rad: 0.0,
                velocity_noise_rad_s: 0.01,
                torque_noise_nm: 0.02,
                // RobStride's feedback frame carries no current (§2.6).
                reports_current: false,
                // This unit's `MeasuredTorque` was stuck at 0 outside MIT (§2.3).
                reports_torque_outside_mit: false,
            },
            faults: Faults::default(),
            torque_limit_nm: 115.0,
            velocity_limit_rad_s: 10.0,
            position_limit_rad: None,
            bus_voltage_v: 23.91,
            bus_latency: Duration::from_millis(1),
            present_ids: vec![1],
        }
    }

    /// Look a preset up by name, for `--driver sim --model <name>`.
    pub fn from_name(name: &str) -> Option<Self> {
        match name.to_ascii_lowercase().replace(['-', '_'], "").as_str() {
            "ideal" | "default" => Some(Self::ideal()),
            "dm4310" | "dmj4310" | "damiao" => Some(Self::dm4310()),
            "rs04" | "robstride04" | "robstride" => Some(Self::rs04()),
            _ => None,
        }
    }

    /// Every preset name `from_name` accepts, for help text and error messages.
    pub const PRESETS: &'static [&'static str] = &["ideal", "dm4310", "rs04"];

    /// Drop the simulated bus latency — tests should not spend real time in
    /// `sleep`.
    pub fn instant(mut self) -> Self {
        self.bus_latency = Duration::ZERO;
        self
    }

    /// Replace the load, e.g. to hang a pendulum off a preset motor.
    pub fn with_load(mut self, load: Load) -> Self {
        self.load = load;
        self
    }

    /// Replace the friction model.
    pub fn with_friction(mut self, friction: Friction) -> Self {
        self.friction = friction;
        self
    }

    /// Set which ids `scan_bus` reports.
    pub fn with_present_ids(mut self, ids: impl Into<Vec<u8>>) -> Self {
        self.present_ids = ids.into();
        self
    }
}

impl Default for SimConfig {
    fn default() -> Self {
        Self::ideal()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_resolve_by_their_aliases() {
        assert!(SimConfig::from_name("dm4310").is_some());
        assert!(SimConfig::from_name("DM-J4310").is_some());
        assert!(SimConfig::from_name("rs04").is_some());
        assert!(SimConfig::from_name("RobStride").is_some());
        assert!(SimConfig::from_name("ideal").is_some());
        assert!(SimConfig::from_name("nonsense").is_none());
    }

    #[test]
    fn every_advertised_preset_actually_resolves() {
        for name in SimConfig::PRESETS {
            assert!(SimConfig::from_name(name).is_some(), "{name}");
        }
    }

    #[test]
    fn measured_friction_keeps_the_static_over_kinetic_ordering() {
        // Getting this backwards is exactly the bug the bench run found
        // (§7.4); the simulator must not be able to reproduce it silently.
        for cfg in [SimConfig::dm4310(), SimConfig::rs04()] {
            assert!(
                cfg.friction.stiction_nm >= cfg.friction.kinetic_nm,
                "{}: stiction {} < kinetic {}",
                cfg.name,
                cfg.friction.stiction_nm,
                cfg.friction.kinetic_nm
            );
        }
    }

    #[test]
    fn dm4310_thermal_reproduces_the_measured_rise_rate() {
        let t = SimConfig::dm4310().thermal;
        let rise = t.heating_c_per_s_per_nm2 * 1.218 * 1.218;
        assert!((rise - 0.116).abs() < 1e-3, "rise = {rise}");
    }

    #[test]
    fn dm4310_binding_matches_the_recorded_hardening_points() {
        let load = SimConfig::dm4310().load;
        // 0.945 N·m at +0.138 rad and 6.7 N·m at +0.201 rad, less the ~0.17
        // N·m friction plateau the holding torque also had to overcome.
        let at = |d: f32| -load.torque_at(d);
        assert!(
            (at(0.138) - 0.775).abs() / 0.775 < 0.15,
            "0.138 rad → {}",
            at(0.138)
        );
        assert!(
            (at(0.201) - 6.53).abs() / 6.53 < 0.15,
            "0.201 rad → {}",
            at(0.201)
        );
    }

    #[test]
    fn a_free_load_pulls_nowhere() {
        for d in [-1.0f32, -0.1, 0.0, 0.1, 5.0] {
            assert_eq!(Load::FREE.torque_at(d), 0.0);
        }
    }

    #[test]
    fn the_ideal_preset_is_actually_ideal() {
        let c = SimConfig::ideal();
        assert_eq!(c.friction.stiction_nm, 0.0);
        assert_eq!(c.friction.kinetic_nm, 0.0);
        assert_eq!(c.bus_latency, Duration::ZERO);
        assert_eq!(c.thermal.heating_c_per_s_per_nm2, 0.0);
    }
}
