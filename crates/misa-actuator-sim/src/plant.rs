//! The physical model: one rotational degree of freedom with stick-slip
//! friction, a configurable load, and a lumped thermal state.
//!
//! `J·dω/dt = τ_motor + τ_load(θ) + τ_friction(ω)`
//!
//! The friction term is the interesting one. A plain `−μ·sign(ω)` never lets
//! the shaft actually stop, which would make `misa-sysid`'s breakaway
//! measurement meaningless — the whole point of that test is that a stationary
//! shaft resists more than a moving one. So the model carries an explicit
//! stuck/sliding state and only releases when the drive torque exceeds
//! stiction.

use crate::config::SimConfig;

/// Longest step the integrator will take in one go. A GUI that polls at 60 Hz
/// hands us 16 ms at a time; integrating that as a single Euler step with a
/// stiff spring would be unstable, so it is subdivided.
const MAX_SUBSTEP_S: f32 = 5.0e-4;

/// Below this the shaft counts as stationary and stiction applies.
const STICK_VEL_RAD_S: f32 = 1.0e-4;

/// Deterministic noise source. Inline so the crate needs no `rand`.
#[derive(Debug, Clone)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        // Any non-zero seed works for xorshift64*.
        Self(seed | 1)
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in `-1.0..=1.0`.
    pub fn symmetric(&mut self) -> f32 {
        let bits = (self.next_u64() >> 40) as f32; // 24 bits
        bits / 8_388_608.0 - 1.0
    }
}

/// Instantaneous state of the simulated shaft.
#[derive(Debug, Clone, Copy)]
pub struct PlantState {
    /// Absolute shaft angle (rad) — the simulator's "encoder", before any
    /// soft-zero offset the driver layer applies.
    pub pos_rad: f32,
    pub vel_rad_s: f32,
    pub temp_c: f32,
    /// Motor torque actually applied last step, after clamping (N·m).
    pub applied_torque_nm: f32,
    /// Whether friction is currently holding the shaft still.
    pub stuck: bool,
}

pub struct Plant {
    pub state: PlantState,
    rng: Rng,
}

impl Plant {
    pub fn new(cfg: &SimConfig) -> Self {
        Self {
            state: PlantState {
                pos_rad: 0.0,
                vel_rad_s: 0.0,
                temp_c: cfg.thermal.ambient_c,
                applied_torque_nm: 0.0,
                stuck: true,
            },
            rng: Rng::new(0x5EED_1234_ABCD_0001),
        }
    }

    /// Advance the plant by `dt` seconds, asking `torque_fn` for the motor
    /// torque at every substep.
    ///
    /// The torque comes from a callback rather than being passed by value
    /// because a real motor's control loops do **not** run at the rate its
    /// host sends commands — the firmware closes current and velocity loops
    /// in the kilohertz while CAN only moves the setpoint. Recomputing the
    /// law once per `Actuator` call and holding it flat both misrepresents
    /// that and is numerically unstable: a light rotor commanded at 200 Hz
    /// diverges outright.
    ///
    /// `dt` is whatever wall-clock time elapsed since the last call, so it is
    /// subdivided internally rather than trusted as a single integration step.
    pub fn step(
        &mut self,
        cfg: &SimConfig,
        dt_s: f32,
        mut torque_fn: impl FnMut(&PlantState, f32) -> f32,
    ) {
        if !(dt_s > 0.0) || !dt_s.is_finite() {
            return;
        }
        // A caller that was blocked for a long time (breakpoint, laptop sleep)
        // should not make the simulator grind through minutes of physics.
        let dt_s = dt_s.min(0.5);

        let steps = (dt_s / MAX_SUBSTEP_S).ceil().max(1.0) as u32;
        let h = dt_s / steps as f32;
        for _ in 0..steps {
            let tau_motor = clamp_sym(torque_fn(&self.state, h), cfg.torque_limit_nm);
            self.state.applied_torque_nm = tau_motor;
            self.substep(cfg, h, tau_motor);
        }
    }

    /// Advance under a torque that does not depend on the state — open-loop
    /// torque commands, and most tests.
    pub fn step_constant(&mut self, cfg: &SimConfig, dt_s: f32, motor_torque_nm: f32) {
        self.step(cfg, dt_s, |_, _| motor_torque_nm);
    }

    fn substep(&mut self, cfg: &SimConfig, h: f32, tau_motor: f32) {
        let s = &mut self.state;
        let f = &cfg.friction;

        // Everything driving the shaft before friction gets a say.
        let tau_drive = tau_motor + cfg.load.torque_at(s.pos_rad);

        if s.stuck {
            if tau_drive.abs() <= f.stiction_nm {
                // Static friction absorbs the whole drive torque: no motion,
                // but the thermal state still tracks the current being pushed.
                s.vel_rad_s = 0.0;
                Self::integrate_thermal(s, cfg, h, tau_motor);
                return;
            }
            s.stuck = false;
        }

        let friction = -(f.kinetic_nm * sign_of(s.vel_rad_s, tau_drive)
            + f.viscous_nm_s_per_rad * s.vel_rad_s);
        let accel = (tau_drive + friction) / cfg.inertia_kg_m2;

        let new_vel = s.vel_rad_s + accel * h;

        // Coming to rest: if the shaft crosses (or reaches) zero velocity and
        // the drive can no longer beat stiction, it sticks rather than
        // chattering around zero.
        let crossed = s.vel_rad_s != 0.0 && new_vel * s.vel_rad_s < 0.0;
        if (crossed || new_vel.abs() < STICK_VEL_RAD_S) && tau_drive.abs() <= f.stiction_nm {
            s.vel_rad_s = 0.0;
            s.stuck = true;
            Self::integrate_thermal(s, cfg, h, tau_motor);
            return;
        }

        let new_vel = clamp_sym(new_vel, cfg.velocity_limit_rad_s);
        // Trapezoidal position update — noticeably less drift than Euler when
        // the caller hands us coarse steps.
        s.pos_rad += 0.5 * (s.vel_rad_s + new_vel) * h;
        s.vel_rad_s = new_vel;

        if let Some((lo, hi)) = cfg.position_limit_rad {
            if s.pos_rad <= lo {
                s.pos_rad = lo;
                s.vel_rad_s = s.vel_rad_s.max(0.0);
            } else if s.pos_rad >= hi {
                s.pos_rad = hi;
                s.vel_rad_s = s.vel_rad_s.min(0.0);
            }
        }

        Self::integrate_thermal(s, cfg, h, tau_motor);
    }

    /// Associated rather than a method so it can be called while the caller
    /// still holds `&mut self.state`.
    fn integrate_thermal(state: &mut PlantState, cfg: &SimConfig, h: f32, tau_motor: f32) {
        let t = &cfg.thermal;
        let heating = t.heating_c_per_s_per_nm2 * tau_motor * tau_motor;
        let cooling = (state.temp_c - t.ambient_c) / t.cooling_tau_s;
        state.temp_c += (heating - cooling) * h;
    }

    /// Shaft angle as a sensor would report it, with quantization applied.
    pub fn sensed_position(&self, cfg: &SimConfig) -> f32 {
        let step = cfg.sensing.position_step_rad;
        if step > 0.0 {
            (self.state.pos_rad / step).round() * step
        } else {
            self.state.pos_rad
        }
    }

    /// Velocity as a sensor would report it, with noise.
    pub fn sensed_velocity(&mut self, cfg: &SimConfig) -> f32 {
        let n = cfg.sensing.velocity_noise_rad_s;
        if n > 0.0 {
            self.state.vel_rad_s + n * self.rng.symmetric()
        } else {
            self.state.vel_rad_s
        }
    }

    /// Applied torque as a sensor would report it, with noise.
    pub fn sensed_torque(&mut self, cfg: &SimConfig) -> f32 {
        let n = cfg.sensing.torque_noise_nm;
        if n > 0.0 {
            self.state.applied_torque_nm + n * self.rng.symmetric()
        } else {
            self.state.applied_torque_nm
        }
    }
}

/// Sign to use for kinetic friction. A shaft at exactly zero velocity that is
/// being driven takes its direction from the drive torque, so friction opposes
/// the motion that is about to happen rather than vanishing.
fn sign_of(vel: f32, tau_drive: f32) -> f32 {
    if vel.abs() > STICK_VEL_RAD_S {
        vel.signum()
    } else if tau_drive != 0.0 {
        tau_drive.signum()
    } else {
        0.0
    }
}

fn clamp_sym(v: f32, limit: f32) -> f32 {
    if limit <= 0.0 {
        v
    } else {
        v.clamp(-limit, limit)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Friction, Load, SimConfig};

    fn frictionless() -> SimConfig {
        let mut c = SimConfig::ideal();
        c.friction = Friction {
            stiction_nm: 0.0,
            kinetic_nm: 0.0,
            viscous_nm_s_per_rad: 0.0,
        };
        c
    }

    /// Run the plant for `seconds` at a fixed torque, 1 ms at a time.
    fn run(plant: &mut Plant, cfg: &SimConfig, seconds: f32, torque: f32) {
        let n = (seconds / 1.0e-3).round() as u32;
        for _ in 0..n {
            plant.step_constant(cfg, 1.0e-3, torque);
        }
    }

    #[test]
    fn a_frictionless_shaft_obeys_alpha_equals_tau_over_j() {
        let cfg = frictionless();
        let mut p = Plant::new(&cfg);
        // ω = τ/J · t, θ = ½·τ/J·t²
        run(&mut p, &cfg, 1.0, 0.01);
        let alpha = 0.01 / cfg.inertia_kg_m2;
        assert!(
            (p.state.vel_rad_s - alpha).abs() / alpha < 1e-3,
            "vel {} vs {alpha}",
            p.state.vel_rad_s
        );
        let expected_pos = 0.5 * alpha;
        assert!(
            (p.state.pos_rad - expected_pos).abs() / expected_pos < 1e-2,
            "pos {} vs {expected_pos}",
            p.state.pos_rad
        );
    }

    #[test]
    fn stiction_holds_the_shaft_until_it_is_exceeded() {
        let cfg = SimConfig::rs04().instant();
        let mut p = Plant::new(&cfg);

        // Just under the measured 0.902 N·m: nothing moves, at all.
        run(&mut p, &cfg, 1.0, 0.90);
        assert!(p.state.stuck);
        assert_eq!(p.state.vel_rad_s, 0.0);
        assert_eq!(p.state.pos_rad, 0.0);

        // Just over: it breaks away.
        run(&mut p, &cfg, 0.2, 0.95);
        assert!(!p.state.stuck);
        assert!(p.state.vel_rad_s > 0.0, "vel {}", p.state.vel_rad_s);
    }

    #[test]
    fn breakaway_torque_matches_the_configured_stiction() {
        // Ramp torque the way `misa-sysid breakaway` does and record where the
        // shaft lets go. The free RS04 load means this should land on the
        // stiction value itself.
        let cfg = SimConfig::rs04().instant();
        let mut p = Plant::new(&cfg);
        let ramp_nm_per_s = 0.5;
        let dt = 1.0e-3;
        let mut tau = 0.0;
        let mut broke_at = None;
        for _ in 0..10_000 {
            tau += ramp_nm_per_s * dt;
            p.step_constant(&cfg, dt, tau);
            if !p.state.stuck && broke_at.is_none() {
                broke_at = Some(tau);
                break;
            }
        }
        let broke_at = broke_at.expect("never broke away");
        assert!(
            (broke_at - cfg.friction.stiction_nm).abs() < 0.01,
            "broke at {broke_at}, stiction {}",
            cfg.friction.stiction_nm
        );
    }

    #[test]
    fn a_moving_shaft_settles_where_kinetic_friction_balances_the_drive() {
        // Above breakaway, terminal velocity satisfies τ = kinetic + viscous·ω.
        // Purpose-built damping, because both measured presets have so little
        // viscous friction that anything past breakaway runs into the speed
        // limit instead (see the next test).
        let mut cfg = frictionless();
        cfg.friction = Friction {
            stiction_nm: 0.10,
            kinetic_nm: 0.10,
            viscous_nm_s_per_rad: 0.05,
        };
        let mut p = Plant::new(&cfg);
        let tau = 0.5;
        run(&mut p, &cfg, 30.0, tau);
        let expected = (tau - cfg.friction.kinetic_nm) / cfg.friction.viscous_nm_s_per_rad;
        assert!(
            (p.state.vel_rad_s - expected).abs() / expected < 0.02,
            "vel {} vs {expected}",
            p.state.vel_rad_s
        );
    }

    #[test]
    fn the_speed_ceiling_is_enforced() {
        // RS04's measured friction is almost purely Coulomb, so any torque
        // that breaks stiction accelerates it into the firmware speed limit.
        let cfg = SimConfig::rs04().instant();
        let mut p = Plant::new(&cfg);
        run(&mut p, &cfg, 20.0, 1.5);
        assert!(
            (p.state.vel_rad_s - cfg.velocity_limit_rad_s).abs() < 1e-3,
            "vel {} vs limit {}",
            p.state.vel_rad_s,
            cfg.velocity_limit_rad_s
        );
    }

    #[test]
    fn a_spring_load_pulls_the_shaft_back_to_equilibrium() {
        let mut cfg = frictionless();
        cfg.load = Load {
            spring_nm_per_rad: 1.0,
            equilibrium_rad: 0.3,
            gravity_nm: 0.0,
            hard_knee_rad: f32::INFINITY,
            hard_stiffness_nm_per_rad2: 0.0,
        };
        cfg.friction.viscous_nm_s_per_rad = 0.1; // so it settles rather than rings
        let mut p = Plant::new(&cfg);
        run(&mut p, &cfg, 30.0, 0.0);
        assert!(
            (p.state.pos_rad - 0.3).abs() < 1e-2,
            "settled at {}",
            p.state.pos_rad
        );
    }

    #[test]
    fn holding_torque_warms_the_motor_at_the_measured_rate() {
        // DM-J4310 §1.7: 1.218 N·m produced +0.116 °C/s. The shaft is held by
        // the rig's binding, so this is pure heating with no motion.
        let mut cfg = SimConfig::dm4310().instant();
        cfg.friction.stiction_nm = 100.0; // clamp the shaft, as the rig did
        let mut p = Plant::new(&cfg);
        let t0 = p.state.temp_c;
        run(&mut p, &cfg, 10.0, 1.218);
        let rate = (p.state.temp_c - t0) / 10.0;
        assert!((rate - 0.116).abs() < 5e-3, "rate {rate}");
    }

    #[test]
    fn position_limits_stop_the_shaft_without_letting_it_escape() {
        let mut cfg = frictionless();
        cfg.position_limit_rad = Some((-0.5, 0.5));
        let mut p = Plant::new(&cfg);
        run(&mut p, &cfg, 5.0, 0.05);
        assert!(p.state.pos_rad <= 0.5 + 1e-6, "pos {}", p.state.pos_rad);
        assert!(p.state.vel_rad_s <= 0.0);
    }

    #[test]
    fn a_long_stall_does_not_make_the_simulator_integrate_forever() {
        let cfg = frictionless();
        let mut p = Plant::new(&cfg);
        // 10 minutes of "elapsed" time in one call gets clipped.
        p.step_constant(&cfg, 600.0, 0.01);
        let alpha = 0.01 / cfg.inertia_kg_m2;
        assert!(p.state.vel_rad_s <= alpha * 0.5 + 1e-3);
    }

    #[test]
    fn zero_and_negative_steps_are_ignored() {
        let cfg = frictionless();
        let mut p = Plant::new(&cfg);
        p.step_constant(&cfg, 0.0, 1.0);
        p.step_constant(&cfg, -1.0, 1.0);
        p.step_constant(&cfg, f32::NAN, 1.0);
        assert_eq!(p.state.pos_rad, 0.0);
        assert_eq!(p.state.vel_rad_s, 0.0);
    }

    #[test]
    fn noise_is_deterministic_for_a_given_seed() {
        let mut a = Rng::new(42);
        let mut b = Rng::new(42);
        for _ in 0..100 {
            assert_eq!(a.symmetric(), b.symmetric());
        }
    }

    #[test]
    fn noise_stays_inside_its_band() {
        let mut r = Rng::new(7);
        for _ in 0..10_000 {
            let v = r.symmetric();
            assert!((-1.0..=1.0).contains(&v), "{v}");
        }
    }
}
