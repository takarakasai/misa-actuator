//! Chirp-excitation system identification for any [`misa_actuator::Actuator`].
//!
//! Drive a motor (and its load) with a swept-sine [`Chirp`] on a chosen
//! [`Excitation`] channel, log the synchronized command + response, and
//! estimate the [`FreqResponse`] (Bode) — a non-parametric model. The whole
//! pipeline is motor-agnostic: it speaks only the `Actuator` trait, so it works
//! identically over Robstride / DAMIAO / LK and any future driver.
//!
//! ```no_run
//! use std::sync::atomic::AtomicBool;
//! use misa_actuator::Actuator;
//! use misa_sysid::{run_chirp, Chirp, Excitation};
//!
//! # fn demo(act: &mut dyn Actuator) -> misa_actuator::Result<()> {
//! let chirp = Chirp::linear(0.5, 30.0, 10.0, 0.15); // 0.5→30 Hz, 10 s, 0.15 rad
//! let abort = AtomicBool::new(false);
//! let log = run_chirp(act, &chirp, Excitation::Position { max_speed_rad_s: 5.0 }, 500.0, &abort)?;
//! let bode = log.frf();
//! println!("identified {} frequency points", bode.freqs_hz.len());
//! # Ok(()) }
//! ```
//!
//! # Safety
//!
//! `Excitation::Position` / `MitPosition` ride the chirp on the start position,
//! so motion stays bounded — prefer them. `Velocity` / `Torque` are open-loop
//! and can drift; use small amplitudes and short runs. `run_chirp` commands a
//! safe stop and disables on exit, and aborts immediately when the supplied
//! `AtomicBool` is set (wire it to Ctrl-C).
//!
//! # Bandwidth
//!
//! Each `Actuator` call blocks on the bus (~1 ms), so the sustainable loop rate
//! is ~500 Hz–1 kHz and the usable identification band is ~50–100 Hz. Check
//! [`ChirpLog::achieved_rate_hz`] against your target.

#[cfg(feature = "cli")]
pub mod cli;
pub mod chirp;
pub mod frf;
pub mod limits;
pub mod quasistatic;
pub mod report;
pub mod runner;
pub mod series;

pub use chirp::{Chirp, Sweep};
pub use frf::{estimate_frf, FreqResponse, FrfOptions};
pub use limits::{AbortReason, Guard, SafetyLimits};
pub use series::{
    run_breakaway_series, run_velocity_sweep_series, FrictionKind, FrictionSample, FrictionSeries,
    Stats,
};
#[cfg(feature = "cli")]
pub use cli::{run_characterize, CharacterizeCmd};
pub use report::{
    run_breakaway_map_to_csv, run_velocity_sweep_to_csv, BreakawayMapReport, VelocitySweepReport,
};
pub use report::{
    run_breakaway_to_csv, run_kt_to_csv, run_load_map_to_csv, run_thermal_to_csv, BreakawayReport,
    KtReport, LoadMapReport, ThermalReport,
};
pub use quasistatic::{
    run_breakaway_map, run_velocity_sweep, BreakawayMap, BreakawayMapPoint, BreakawayMapSpec,
    VelocitySweep, VelocitySweepSpec,
};
pub use quasistatic::{
    run_breakaway, run_kt, run_load_map, run_thermal, Breakaway, BreakawaySpec, Direction, KtSpec,
    KtSweep, LoadMap, LoadMapSpec, Point, Thermal, ThermalSpec, POINTS_CSV_HEADER,
    write_points_csv,
};
pub use runner::{
    run_chirp, run_chirp_to_csv, run_chirp_with, ChirpLog, Excitation, RunReport, Sample,
};

#[cfg(test)]
mod tests {
    use core::f32::consts::TAU;
    use std::sync::atomic::AtomicBool;

    use misa_actuator::{Actuator, Error, MotorFeedback, MotorStatus, Result, RunMode};

    use crate::runner::Sample;
    use crate::{run_chirp, Chirp, ChirpLog, Excitation};

    /// A simulated 1st-order rotational plant `J·dω/dt = τ − b·ω` driven through
    /// the `Actuator::set_torque` channel, so `run_chirp` can be exercised
    /// end-to-end without hardware. Each command advances the sim by `dt`.
    struct MockPlant {
        j: f32,
        b: f32,
        dt: f32,
        pos: f32,
        vel: f32,
    }

    impl MockPlant {
        fn new(j: f32, b: f32, rate_hz: f32) -> Self {
            Self {
                j,
                b,
                dt: 1.0 / rate_hz,
                pos: 0.0,
                vel: 0.0,
            }
        }
        fn step(&mut self, torque: f32) {
            // forward Euler
            let acc = (torque - self.b * self.vel) / self.j;
            self.vel += acc * self.dt;
            self.pos += self.vel * self.dt;
        }
        fn fb(&self) -> MotorFeedback {
            MotorFeedback {
                position_rad: self.pos,
                velocity_rad_per_s: self.vel,
                torque_nm: self.b * self.vel, // reaction torque
                current_a: f32::NAN,
                temperature_c: f32::NAN,
            }
        }
    }

    impl Actuator for MockPlant {
        fn motor_id(&self) -> u8 {
            1
        }
        fn enable(&mut self) -> Result<MotorFeedback> {
            Ok(self.fb())
        }
        fn disable(&mut self) -> Result<()> {
            Ok(())
        }
        fn set_zero(&mut self) -> Result<()> {
            Ok(())
        }
        fn set_run_mode(&mut self, _m: RunMode) -> Result<()> {
            Ok(())
        }
        fn set_position(&mut self, _p: f32, _v: f32) -> Result<MotorFeedback> {
            Err(Error::Unsupported("mock: position"))
        }
        fn set_velocity(&mut self, _v: f32) -> Result<MotorFeedback> {
            Err(Error::Unsupported("mock: velocity"))
        }
        fn set_torque(&mut self, torque_nm: f32) -> Result<MotorFeedback> {
            self.step(torque_nm);
            Ok(self.fb())
        }
        fn mit_control(
            &mut self,
            p_des: f32,
            v_des: f32,
            kp: f32,
            kd: f32,
            tau_ff: f32,
        ) -> Result<MotorFeedback> {
            // MIT law: τ = kp(p_des−p) + kd(v_des−v) + tau_ff.
            let tau = kp * (p_des - self.pos) + kd * (v_des - self.vel) + tau_ff;
            self.step(tau);
            Ok(self.fb())
        }
        fn measure(&mut self) -> Result<MotorFeedback> {
            Ok(self.fb())
        }
        fn read_status(&mut self) -> Result<MotorStatus> {
            Ok(MotorStatus {
                voltage_v: f32::NAN,
                temperature_c: f32::NAN,
                error: Default::default(),
            })
        }
    }

    /// Deterministic (no real-time) end-to-end: simulate a torque chirp through
    /// `J·dω/dt = τ − b·ω`, build a `ChirpLog`, and check `ChirpLog::frf`
    /// recovers the analytic plant `G(jω) = 1/(b + jωJ)`.
    #[test]
    fn chirp_log_frf_recovers_first_order_plant() {
        let (j, b) = (0.02f32, 0.5f32);
        let rate = 1000.0f32;
        let dt = 1.0 / rate;
        let chirp = Chirp::linear(1.0, 120.0, 8.0, 0.5);
        let n = (chirp.duration_s * rate) as usize;

        let mut plant = MockPlant::new(j, b, rate);
        let mut samples = Vec::with_capacity(n);
        for k in 0..n {
            let t = k as f32 * dt;
            let u = chirp.value(t); // torque command
            plant.step(u);
            let fb = plant.fb();
            samples.push(Sample {
                t_s: t,
                cmd: u,
                position_rad: fb.position_rad,
                velocity_rad_per_s: fb.velocity_rad_per_s,
                torque_nm: fb.torque_nm,
                current_a: f32::NAN,
            });
        }
        let log = ChirpLog {
            samples,
            excitation: Excitation::Torque,
            chirp,
            achieved_rate_hz: rate,
        };

        let fr = log.frf();
        assert!(!fr.freqs_hz.is_empty());
        let mut checked = 0;
        for i in 0..fr.freqs_hz.len() {
            let f = fr.freqs_hz[i];
            if !(2.0..=15.0).contains(&f) || fr.coherence[i] < 0.9 {
                continue;
            }
            let w = TAU * f;
            let g_db = 20.0 * (1.0 / (b * b + (w * j) * (w * j)).sqrt()).log10();
            assert!(
                (fr.magnitude_db[i] - g_db).abs() < 3.0,
                "f={f}: est={:.2} dB, exp={:.2} dB",
                fr.magnitude_db[i],
                g_db
            );
            checked += 1;
        }
        assert!(checked >= 5, "too few well-excited frequencies checked");
    }

    /// Short real-time run to exercise `run_chirp`'s actuator-driving loop.
    #[test]
    fn run_chirp_drives_actuator_and_logs() {
        let mut plant = MockPlant::new(0.02, 0.5, 500.0);
        let chirp = Chirp::linear(1.0, 50.0, 0.2, 0.3);
        let abort = AtomicBool::new(false);
        let log = run_chirp(&mut plant, &chirp, Excitation::Torque, 500.0, &abort).unwrap();
        assert!(log.samples.len() >= 16, "got {} samples", log.samples.len());
        assert!(log.achieved_rate_hz > 0.0);
        // first command ≈ chirp.value(0) ≈ 0
        assert!(log.samples[0].cmd.abs() < 0.1);
        let _ = log.frf(); // must not panic
    }

    /// MIT torque-feed-forward excitation with a small leash drives the plant
    /// via `mit_control`; the chirp value is the torque FF (the FRF input).
    #[test]
    fn run_chirp_mit_torque_excites_via_feedforward() {
        let mut plant = MockPlant::new(0.02, 0.5, 500.0);
        let chirp = Chirp::linear(2.0, 50.0, 0.2, 0.3);
        let abort = AtomicBool::new(false);
        let log = run_chirp(
            &mut plant,
            &chirp,
            Excitation::MitTorque { kp: 1.0, kd: 0.05 },
            500.0,
            &abort,
        )
        .unwrap();
        assert!(log.samples.len() >= 16);
        // cmd is the torque feed-forward (≈ chirp value), not a position.
        assert!(log.samples[0].cmd.abs() < 0.1);
        let _ = log.frf();
    }
}
