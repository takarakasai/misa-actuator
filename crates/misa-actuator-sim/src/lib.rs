//! A simulated [`Actuator`] — a motor, its load, and its firmware, with no
//! hardware attached.
//!
//! Two jobs:
//!
//! 1. **Develop without a bench.** Every tool in this workspace speaks
//!    `Actuator`, so a simulator is a drop-in for all of them:
//!    `misa-actuator-tui --driver sim`, the GUI, the vendor CLIs.
//! 2. **Give `misa-sysid` a known answer.** A real motor never tells you its
//!    true friction or inertia, so a characterization run can only ever be
//!    *plausible*. Here the answer is in the config, and the presets are
//!    calibrated from `doc/bench-measurements-2026-07-30.md` — so a run can be
//!    checked, not just eyeballed.
//!
//! ```
//! use misa_actuator::Actuator;
//! use misa_actuator_sim::{SimActuator, SimConfig};
//!
//! // A frictionless plant with no simulated bus delay.
//! let mut motor = SimActuator::new(SimConfig::ideal());
//! motor.enable().unwrap();
//! let fb = motor.set_torque(0.05).unwrap();
//! println!("{:+.3} rad", fb.position_rad);
//! ```
//!
//! # Time
//!
//! The plant integrates against elapsed wall-clock time, so the shaft keeps
//! moving between calls exactly as a real one would. Tests that need exact,
//! instant, repeatable time build with [`SimActuator::with_clock`] and a
//! [`ManualClock`].

pub mod clock;
pub mod config;
pub mod plant;

use std::ops::RangeInclusive;
use std::sync::Arc;
use std::time::Duration;

use misa_actuator::{Actuator, Error, ErrorFlags, MotorFeedback, MotorStatus, Result, RunMode};

pub use clock::{Clock, ManualClock, RealClock};
pub use config::{Faults, Friction, Gains, Load, Sensing, SimConfig, Thermal};
pub use plant::{Plant, PlantState};

/// What the simulated firmware is currently doing.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Active {
    /// Not energised — the shaft coasts.
    Idle,
    Position {
        target_rad: f32,
        max_speed_rad_s: f32,
    },
    Velocity {
        target_rad_s: f32,
    },
    Torque {
        target_nm: f32,
    },
    Mit {
        pos_rad: f32,
        vel_rad_s: f32,
        kp: f32,
        kd: f32,
        torque_ff_nm: f32,
    },
}

/// One evaluation of the simulated firmware's control law.
///
/// Free-standing rather than a method so it can run inside the plant's
/// integration loop without borrowing the whole [`SimActuator`].
#[allow(clippy::too_many_arguments)]
fn firmware_torque(
    enabled: bool,
    active: Active,
    gains: Gains,
    velocity_limit: f32,
    torque_limit: f32,
    state: &PlantState,
    h: f32,
    vel_integral: &mut f32,
) -> f32 {
    if !enabled {
        *vel_integral = 0.0;
        return 0.0;
    }
    let pos = state.pos_rad;
    let vel = state.vel_rad_s;

    match active {
        Active::Idle => {
            *vel_integral = 0.0;
            0.0
        }
        Active::Torque { target_nm } => target_nm,
        Active::Mit {
            pos_rad,
            vel_rad_s,
            kp,
            kd,
            torque_ff_nm,
        } => kp * (pos_rad - pos) + kd * (vel_rad_s - vel) + torque_ff_nm,
        Active::Velocity { target_rad_s } => {
            velocity_loop(target_rad_s, vel, h, gains, torque_limit, vel_integral)
        }
        Active::Position {
            target_rad,
            max_speed_rad_s,
        } => {
            // Cascaded P (position) → PI (velocity), which is how these
            // firmwares are actually built and what makes `max_speed`
            // meaningful rather than decorative.
            let cap = max_speed_rad_s.abs().min(velocity_limit);
            let v_ref = (gains.position_p * (target_rad - pos)).clamp(-cap, cap);
            velocity_loop(v_ref, vel, h, gains, torque_limit, vel_integral)
        }
    }
}

fn velocity_loop(
    target: f32,
    vel: f32,
    h: f32,
    gains: Gains,
    torque_limit: f32,
    integral: &mut f32,
) -> f32 {
    let err = target - vel;
    *integral += err * h;
    // Anti-windup: bound the integral by what it alone could command.
    if gains.velocity_i > 0.0 {
        let i_limit = torque_limit / gains.velocity_i;
        *integral = integral.clamp(-i_limit, i_limit);
    } else {
        *integral = 0.0;
    }
    gains.velocity_p * err + gains.velocity_i * *integral
}

/// A simulated motor that implements [`Actuator`].
pub struct SimActuator {
    cfg: SimConfig,
    clock: Arc<dyn Clock>,
    plant: Plant,
    motor_id: u8,

    active: Active,
    /// The mode `set_run_mode` selected, mirroring the drivers that track one.
    run_mode: RunMode,
    enabled: bool,
    /// Soft zero, as every driver in this workspace implements it: reported
    /// positions subtract it, commanded ones add it.
    zero_offset_rad: Option<f32>,
    /// Velocity-loop integrator state.
    vel_integral: f32,

    last_update_s: f64,
    /// Counts bus transactions, so fault injection can fire every Nth.
    transactions: u32,
}

impl SimActuator {
    /// Build a simulator on wall-clock time.
    pub fn new(cfg: SimConfig) -> Self {
        Self::with_clock(cfg, Arc::new(RealClock::new()))
    }

    /// Build a simulator on a caller-supplied clock — pass a [`ManualClock`]
    /// for deterministic tests.
    pub fn with_clock(cfg: SimConfig, clock: Arc<dyn Clock>) -> Self {
        let motor_id = cfg.present_ids.first().copied().unwrap_or(1);
        let plant = Plant::new(&cfg);
        let last_update_s = clock.now_s();
        Self {
            cfg,
            clock,
            plant,
            motor_id,
            active: Active::Idle,
            run_mode: RunMode::Mit,
            enabled: false,
            zero_offset_rad: None,
            vel_integral: 0.0,
            last_update_s,
            transactions: 0,
        }
    }

    /// Bind to a specific motor id (the ids `scan_bus` reports come from the
    /// config's `present_ids`).
    pub fn with_motor_id(mut self, motor_id: u8) -> Self {
        self.motor_id = motor_id;
        self
    }

    /// The configuration in force, including the true friction and inertia a
    /// characterization run is trying to recover.
    pub fn config(&self) -> &SimConfig {
        &self.cfg
    }

    /// The raw plant state, for tests that want ground truth rather than what
    /// the sensors report.
    pub fn truth(&self) -> PlantState {
        self.plant.state
    }

    /// Simulate one bus transaction: burn the configured latency and decide
    /// whether this one is going to fail.
    fn transact(&mut self) -> Result<()> {
        self.clock.sleep(self.cfg.bus_latency);
        self.transactions = self.transactions.wrapping_add(1);

        if self.cfg.faults.dead {
            return Err(Error::Timeout {
                motor_id: self.motor_id,
            });
        }
        let every = self.cfg.faults.timeout_every_n;
        if every > 0 && self.transactions % every == 0 {
            log::debug!("sim: injected timeout on transaction {}", self.transactions);
            return Err(Error::Timeout {
                motor_id: self.motor_id,
            });
        }
        Ok(())
    }

    /// Integrate the plant forward to "now" under the active command.
    ///
    /// The control law is handed to the plant as a callback so it is evaluated
    /// at every substep, the way a motor's firmware closes its loops in the
    /// kilohertz regardless of how often the host talks to it.
    fn advance(&mut self) {
        let now = self.clock.now_s();
        let dt = (now - self.last_update_s) as f32;
        self.last_update_s = now;
        if dt <= 0.0 {
            return;
        }

        // Copied out so the closure borrows neither `self` nor `self.cfg`,
        // both of which the plant already has.
        let enabled = self.enabled;
        let active = self.active;
        let gains = self.cfg.gains;
        let velocity_limit = self.cfg.velocity_limit_rad_s;
        let torque_limit = self.cfg.torque_limit_nm;
        let integral = &mut self.vel_integral;

        self.plant.step(&self.cfg, dt, |state, h| {
            firmware_torque(
                enabled,
                active,
                gains,
                velocity_limit,
                torque_limit,
                state,
                h,
                integral,
            )
        });
    }

    /// Build the feedback a driver would hand back, applying the soft zero and
    /// the configured sensing limitations.
    fn feedback(&mut self, from_mit: bool) -> MotorFeedback {
        let zero = self.zero_offset_rad.unwrap_or(0.0);
        let position_rad = self.plant.sensed_position(&self.cfg) - zero;
        let velocity_rad_per_s = self.plant.sensed_velocity(&self.cfg);
        let torque = self.plant.sensed_torque(&self.cfg);

        // Some units only report torque through the MIT feedback frame — the
        // bench RS04's `MeasuredTorque` register was stuck at zero everywhere
        // else, and that quirk is worth being able to reproduce.
        let torque_nm = if from_mit || self.cfg.sensing.reports_torque_outside_mit {
            torque
        } else {
            0.0
        };
        let current_a = if self.cfg.sensing.reports_current {
            torque / self.cfg.torque_constant_nm_per_a
        } else {
            f32::NAN
        };

        MotorFeedback {
            position_rad,
            velocity_rad_per_s,
            torque_nm,
            current_a,
            temperature_c: self.plant.state.temp_c,
        }
    }

    /// Turn a command in the zeroed frame into an absolute shaft angle.
    fn absolute(&self, pos_rad: f32) -> f32 {
        pos_rad + self.zero_offset_rad.unwrap_or(0.0)
    }

    /// Shared prologue: run the bus transaction, then bring the plant up to
    /// date under whatever command was already active.
    fn begin(&mut self) -> Result<()> {
        self.transact()?;
        self.advance();
        Ok(())
    }
}

impl Actuator for SimActuator {
    fn motor_id(&self) -> u8 {
        self.motor_id
    }

    fn enable(&mut self) -> Result<MotorFeedback> {
        self.begin()?;
        self.enabled = true;
        self.vel_integral = 0.0;
        // Hold where we are, so enabling does not itself command motion.
        let here = self.plant.state.pos_rad;
        self.active = Active::Position {
            target_rad: here,
            max_speed_rad_s: self.cfg.velocity_limit_rad_s,
        };
        Ok(self.feedback(false))
    }

    fn disable(&mut self) -> Result<()> {
        self.begin()?;
        self.enabled = false;
        self.active = Active::Idle;
        self.vel_integral = 0.0;
        Ok(())
    }

    fn set_zero(&mut self) -> Result<()> {
        self.begin()?;
        self.zero_offset_rad = Some(self.plant.state.pos_rad);
        // A position hold has to follow the anchor, or the shaft jumps by the
        // offset the moment the zero moves.
        if let Active::Position { target_rad, .. } = &mut self.active {
            *target_rad = self.plant.state.pos_rad;
        }
        Ok(())
    }

    fn set_run_mode(&mut self, mode: RunMode) -> Result<()> {
        self.begin()?;
        self.run_mode = mode;
        Ok(())
    }

    fn set_position(&mut self, pos_rad: f32, max_speed_rad_s: f32) -> Result<MotorFeedback> {
        self.begin()?;
        if !self.enabled {
            return Err(Error::NotEnabled {
                motor_id: self.motor_id,
            });
        }
        self.run_mode = RunMode::Position;
        self.active = Active::Position {
            target_rad: self.absolute(pos_rad),
            max_speed_rad_s,
        };
        Ok(self.feedback(false))
    }

    fn set_velocity(&mut self, vel_rad_s: f32) -> Result<MotorFeedback> {
        self.begin()?;
        if !self.enabled {
            return Err(Error::NotEnabled {
                motor_id: self.motor_id,
            });
        }
        self.run_mode = RunMode::Velocity;
        self.active = Active::Velocity {
            target_rad_s: vel_rad_s,
        };
        Ok(self.feedback(false))
    }

    fn set_torque(&mut self, torque_nm: f32) -> Result<MotorFeedback> {
        self.begin()?;
        if !self.enabled {
            return Err(Error::NotEnabled {
                motor_id: self.motor_id,
            });
        }
        self.run_mode = RunMode::Torque;
        self.active = Active::Torque {
            target_nm: torque_nm,
        };
        Ok(self.feedback(false))
    }

    fn mit_control(
        &mut self,
        pos_rad: f32,
        vel_rad_s: f32,
        kp_nm_per_rad: f32,
        kd_nm_per_rad_s: f32,
        torque_ff_nm: f32,
    ) -> Result<MotorFeedback> {
        self.begin()?;
        if !self.enabled {
            return Err(Error::NotEnabled {
                motor_id: self.motor_id,
            });
        }
        self.run_mode = RunMode::Mit;
        self.active = Active::Mit {
            pos_rad: self.absolute(pos_rad),
            vel_rad_s,
            kp: kp_nm_per_rad,
            kd: kd_nm_per_rad_s,
            torque_ff_nm,
        };
        Ok(self.feedback(true))
    }

    fn measure(&mut self) -> Result<MotorFeedback> {
        self.begin()?;
        // A read must not change what the motor is doing.
        Ok(self.feedback(matches!(self.active, Active::Mit { .. })))
    }

    fn read_status(&mut self) -> Result<MotorStatus> {
        self.begin()?;
        let overheated = self
            .cfg
            .faults
            .over_temperature_c
            .is_some_and(|limit| self.plant.state.temp_c > limit);
        let error = if overheated {
            // `raw` mirrors the common bit: the simulator has no vendor-native
            // fault word to report underneath it.
            ErrorFlags::new(ErrorFlags::MOTOR_OVERHEAT, ErrorFlags::MOTOR_OVERHEAT)
        } else {
            ErrorFlags::default()
        };
        Ok(MotorStatus {
            voltage_v: self.cfg.bus_voltage_v,
            temperature_c: self.plant.state.temp_c,
            error,
        })
    }

    fn current_run_mode_hint(&self) -> Option<RunMode> {
        Some(self.run_mode)
    }

    fn is_enabled_hint(&self) -> bool {
        self.enabled
    }

    fn scan_bus(
        &mut self,
        id_range: RangeInclusive<u8>,
        timeout_per_id: Duration,
    ) -> Result<Vec<u8>> {
        let mut found = Vec::new();
        for id in id_range {
            if self.probe_motor(id, timeout_per_id)? {
                found.push(id);
            }
        }
        Ok(found)
    }

    fn probe_motor(&mut self, motor_id: u8, timeout: Duration) -> Result<bool> {
        // A probe costs a round trip whether or not anyone answers, and a
        // silent id costs the full timeout — which is what makes scan
        // progress UIs worth building in the first place.
        let present = self.cfg.present_ids.contains(&motor_id);
        if present {
            self.clock.sleep(self.cfg.bus_latency);
        } else {
            self.clock.sleep(timeout);
        }
        self.advance();
        Ok(present)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A simulator on virtual time: no sleeping, exactly reproducible.
    fn manual(cfg: SimConfig) -> (SimActuator, Arc<ManualClock>) {
        let clock = Arc::new(ManualClock::new());
        let act = SimActuator::with_clock(cfg.instant(), clock.clone());
        (act, clock)
    }

    /// Hold a command for `seconds`, re-issuing it at `hz` the way a control
    /// loop would.
    fn hold(act: &mut SimActuator, clock: &ManualClock, seconds: f32, hz: f32, mut tick: impl FnMut(&mut SimActuator)) {
        let dt = 1.0 / hz;
        let n = (seconds * hz).round() as u32;
        for _ in 0..n {
            clock.advance_s(dt as f64);
            tick(act);
        }
    }

    #[test]
    fn a_fresh_simulator_is_disabled_and_still() {
        let (mut act, _c) = manual(SimConfig::ideal());
        assert!(!act.is_enabled_hint());
        let fb = act.measure().unwrap();
        assert_eq!(fb.position_rad, 0.0);
        assert_eq!(fb.velocity_rad_per_s, 0.0);
    }

    #[test]
    fn control_commands_are_refused_until_enabled() {
        let (mut act, _c) = manual(SimConfig::ideal());
        assert!(matches!(
            act.set_torque(1.0),
            Err(Error::NotEnabled { .. })
        ));
        assert!(matches!(
            act.set_velocity(1.0),
            Err(Error::NotEnabled { .. })
        ));
        assert!(matches!(
            act.mit_control(0.0, 0.0, 10.0, 1.0, 0.0),
            Err(Error::NotEnabled { .. })
        ));
    }

    #[test]
    fn enabling_holds_position_rather_than_commanding_motion() {
        let (mut act, clock) = manual(SimConfig::ideal());
        act.enable().unwrap();
        hold(&mut act, &clock, 2.0, 100.0, |a| {
            a.measure().unwrap();
        });
        assert!(act.truth().pos_rad.abs() < 1e-3, "drifted to {}", act.truth().pos_rad);
    }

    #[test]
    fn position_control_reaches_its_target() {
        let (mut act, clock) = manual(SimConfig::ideal());
        act.enable().unwrap();
        hold(&mut act, &clock, 5.0, 200.0, |a| {
            a.set_position(1.0, 5.0).unwrap();
        });
        let fb = act.measure().unwrap();
        assert!((fb.position_rad - 1.0).abs() < 0.02, "at {}", fb.position_rad);
    }

    #[test]
    fn position_control_respects_the_speed_cap() {
        let (mut act, clock) = manual(SimConfig::ideal());
        act.enable().unwrap();
        let mut peak: f32 = 0.0;
        hold(&mut act, &clock, 3.0, 500.0, |a| {
            let fb = a.set_position(10.0, 1.0).unwrap();
            peak = peak.max(fb.velocity_rad_per_s.abs());
        });
        // Some overshoot is expected from the inner loop; a cap that did
        // nothing would let it run to the 30 rad/s device limit.
        assert!(peak < 1.5, "peaked at {peak} rad/s against a 1.0 cap");
    }

    #[test]
    fn velocity_control_settles_on_the_commanded_speed() {
        let (mut act, clock) = manual(SimConfig::ideal());
        act.enable().unwrap();
        hold(&mut act, &clock, 5.0, 200.0, |a| {
            a.set_velocity(2.0).unwrap();
        });
        let fb = act.measure().unwrap();
        assert!(
            (fb.velocity_rad_per_s - 2.0).abs() < 0.05,
            "at {}",
            fb.velocity_rad_per_s
        );
    }

    #[test]
    fn mit_torque_follows_kp_times_the_position_error() {
        // The relationship the bench verified on the real DM-J4310 (§1.4).
        let (mut act, clock) = manual(SimConfig::ideal());
        act.enable().unwrap();
        // Hold the shaft still by making it immovable, so the error persists.
        act.cfg.friction.stiction_nm = 1000.0;
        let kp = 5.0;
        hold(&mut act, &clock, 0.5, 200.0, |a| {
            a.mit_control(0.2, 0.0, kp, 0.0, 0.0).unwrap();
        });
        let fb = act.measure().unwrap();
        let expected = kp * 0.2;
        assert!(
            (fb.torque_nm - expected).abs() < 0.01,
            "τ {} vs {expected}",
            fb.torque_nm
        );
    }

    #[test]
    fn set_zero_reanchors_the_reported_position() {
        let (mut act, clock) = manual(SimConfig::ideal());
        act.enable().unwrap();
        hold(&mut act, &clock, 3.0, 200.0, |a| {
            a.set_position(0.5, 5.0).unwrap();
        });
        assert!((act.measure().unwrap().position_rad - 0.5).abs() < 0.02);

        act.set_zero().unwrap();
        let fb = act.measure().unwrap();
        assert!(fb.position_rad.abs() < 0.02, "after zero: {}", fb.position_rad);
        // ...and the shaft did not lurch when the anchor moved.
        hold(&mut act, &clock, 1.0, 200.0, |a| {
            a.measure().unwrap();
        });
        assert!((act.truth().pos_rad - 0.5).abs() < 0.05);
    }

    #[test]
    fn disable_lets_the_shaft_coast_instead_of_holding() {
        let (mut act, clock) = manual(SimConfig::ideal());
        act.enable().unwrap();
        hold(&mut act, &clock, 2.0, 200.0, |a| {
            a.set_velocity(3.0).unwrap();
        });
        assert!(act.truth().vel_rad_s > 2.0);

        act.disable().unwrap();
        hold(&mut act, &clock, 5.0, 200.0, |a| {
            a.measure().unwrap();
        });
        // Only viscous drag remains, so it slows but is no longer driven.
        assert!(act.truth().vel_rad_s < 1.0, "still at {}", act.truth().vel_rad_s);
        assert!(!act.is_enabled_hint());
    }

    #[test]
    fn scan_reports_exactly_the_configured_ids() {
        let cfg = SimConfig::ideal().with_present_ids(vec![1, 4, 7]);
        let (mut act, _c) = manual(cfg);
        let found = act.scan_bus(1..=10, Duration::from_millis(5)).unwrap();
        assert_eq!(found, vec![1, 4, 7]);
    }

    #[test]
    fn scanning_a_silent_id_costs_the_full_timeout() {
        // Which is exactly why the scan UI needs incremental progress.
        let cfg = SimConfig::ideal().with_present_ids(vec![1]);
        let (mut act, clock) = manual(cfg);
        let t0 = clock.now_s();
        act.scan_bus(1..=5, Duration::from_millis(100)).unwrap();
        let elapsed = clock.now_s() - t0;
        // Four silent ids at 100 ms each.
        assert!(elapsed >= 0.4, "only {elapsed} s of virtual time");
    }

    #[test]
    fn injected_timeouts_surface_as_timeout_errors() {
        let mut cfg = SimConfig::ideal();
        cfg.faults.timeout_every_n = 3;
        let (mut act, _c) = manual(cfg);
        let mut timeouts = 0;
        for _ in 0..9 {
            if matches!(act.measure(), Err(Error::Timeout { .. })) {
                timeouts += 1;
            }
        }
        assert_eq!(timeouts, 3);
    }

    #[test]
    fn a_dead_motor_never_answers() {
        let mut cfg = SimConfig::ideal();
        cfg.faults.dead = true;
        let (mut act, _c) = manual(cfg);
        assert!(act.measure().is_err());
        assert!(act.enable().is_err());
    }

    #[test]
    fn rs04_withholds_current_and_non_mit_torque_like_the_real_unit() {
        let (mut act, clock) = manual(SimConfig::rs04());
        act.enable().unwrap();
        hold(&mut act, &clock, 0.5, 200.0, |a| {
            a.set_torque(2.0).unwrap();
        });
        let fb = act.measure().unwrap();
        assert!(fb.current_a.is_nan(), "RS04 reports no current in feedback");
        assert_eq!(fb.torque_nm, 0.0, "MeasuredTorque was stuck at zero");

        // ...but the MIT path does report it.
        let fb = act.mit_control(0.0, 0.0, 0.0, 0.0, 1.0).unwrap();
        assert_ne!(fb.torque_nm, 0.0);
    }

    #[test]
    fn dm4310_reports_current_derived_from_its_torque_constant() {
        let (mut act, clock) = manual(SimConfig::dm4310());
        act.enable().unwrap();
        hold(&mut act, &clock, 0.2, 500.0, |a| {
            a.set_torque(0.5).unwrap();
        });
        let fb = act.measure().unwrap();
        let expected = fb.torque_nm / 0.94324;
        assert!(
            (fb.current_a - expected).abs() < 1e-3,
            "{} vs {expected}",
            fb.current_a
        );
    }

    #[test]
    fn over_temperature_raises_a_fault_flag() {
        let mut cfg = SimConfig::dm4310();
        cfg.faults.over_temperature_c = Some(30.0);
        cfg.friction.stiction_nm = 1000.0; // clamp the shaft so it just heats
        let (mut act, clock) = manual(cfg);
        act.enable().unwrap();
        assert!(!act.read_status().unwrap().error.any());

        hold(&mut act, &clock, 300.0, 20.0, |a| {
            a.set_torque(3.0).unwrap();
        });
        let st = act.read_status().unwrap();
        assert!(st.temperature_c > 30.0, "only {} °C", st.temperature_c);
        assert!(st.error.any(), "no fault flagged at {} °C", st.temperature_c);
    }

    #[test]
    fn status_reports_the_configured_bus_voltage() {
        let (mut act, _c) = manual(SimConfig::rs04());
        assert!((act.read_status().unwrap().voltage_v - 23.91).abs() < 1e-3);
    }

    #[test]
    fn the_run_mode_hint_tracks_the_last_command() {
        let (mut act, _c) = manual(SimConfig::ideal());
        act.enable().unwrap();
        act.set_velocity(1.0).unwrap();
        assert_eq!(act.current_run_mode_hint(), Some(RunMode::Velocity));
        act.set_torque(0.1).unwrap();
        assert_eq!(act.current_run_mode_hint(), Some(RunMode::Torque));
        act.mit_control(0.0, 0.0, 1.0, 0.1, 0.0).unwrap();
        assert_eq!(act.current_run_mode_hint(), Some(RunMode::Mit));
    }

    #[test]
    fn every_preset_can_actually_track_a_position_command() {
        // The bench RS04 held a ±0.6 rad sweep to ±0.0004 rad (§2.4), so a
        // preset whose position loop cannot overcome its own friction is
        // wrong, not realistic. This caught exactly that: at the original
        // gains the RS04's 0.9 N·m stiction pinned the shaft and a position
        // chirp produced a flat line.
        // 0.05 rad, deliberately small: the DM-J4310 preset carries its rig's
        // binding, which needs more than the motor's 10 N·m limit past about
        // 0.23 rad (§1.6). Asking for more than that would test the end stop,
        // not the control loop.
        const TARGET: f32 = 0.05;
        for cfg in [SimConfig::ideal(), SimConfig::dm4310(), SimConfig::rs04()] {
            let name = cfg.name.clone();
            let stiction = cfg.friction.stiction_nm;
            let (mut act, clock) = manual(cfg);
            act.enable().unwrap();
            hold(&mut act, &clock, 3.0, 500.0, |a| {
                a.set_position(TARGET, 5.0).unwrap();
            });
            let pos = act.truth().pos_rad;
            assert!(
                (pos - TARGET).abs() < 0.005,
                "{name} (stiction {stiction} N·m) reached {pos:.4} rad, not {TARGET}"
            );
        }
    }

    #[test]
    fn a_position_chirp_actually_moves_every_preset() {
        // The same failure one level up: a sweep whose response never leaves
        // zero is not a measurement, and the Bode plot drawn from it is
        // numerical noise at -240 dB.
        for cfg in [SimConfig::ideal(), SimConfig::dm4310(), SimConfig::rs04()] {
            let name = cfg.name.clone();
            let (mut act, clock) = manual(cfg);
            act.enable().unwrap();
            let mut lo = f32::INFINITY;
            let mut hi = f32::NEG_INFINITY;
            // 2 Hz, ±0.15 rad — the amplitude the GUI defaults to.
            let mut t = 0.0f32;
            hold(&mut act, &clock, 4.0, 500.0, |a| {
                t += 1.0 / 500.0;
                let target = 0.15 * (2.0 * std::f32::consts::PI * 2.0 * t).sin();
                let fb = a.set_position(target, 10.0).unwrap();
                lo = lo.min(fb.position_rad);
                hi = hi.max(fb.position_rad);
            });
            let peak_to_peak = hi - lo;
            assert!(
                peak_to_peak > 0.15,
                "{name}: the shaft moved {peak_to_peak:.4} rad p-p against a \
                 0.30 rad p-p command — a chirp here would identify nothing"
            );
        }
    }

    #[test]
    fn two_runs_on_a_manual_clock_produce_identical_numbers() {
        let run = || {
            let (mut act, clock) = manual(SimConfig::dm4310());
            act.enable().unwrap();
            let mut samples = Vec::new();
            hold(&mut act, &clock, 1.0, 200.0, |a| {
                samples.push(a.set_position(0.3, 2.0).unwrap().position_rad);
            });
            samples
        };
        assert_eq!(run(), run());
    }
}
