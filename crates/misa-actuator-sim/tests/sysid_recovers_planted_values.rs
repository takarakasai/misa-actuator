//! End-to-end check that `misa-sysid` recovers what the simulator was built
//! with.
//!
//! On a real motor a characterization run can only ever be *plausible* — the
//! true friction is not written down anywhere. Here it is: the preset says
//! stiction is 0.902 N·m, so a breakaway measurement that reports 0.902 N·m
//! has demonstrated that the measurement works, and one that reports 0.3 has
//! found a bug in one of the two.
//!
//! The presets are calibrated from `doc/bench-measurements-2026-07-30.md`, so
//! the numbers asserted here are the ones the real hardware produced.
//!
//! These run in wall-clock time (that is the point — `misa-sysid` drives real
//! control loops), so they are deliberately short: fast ramps and brief holds.
//! The simulator's stiction has no rate dependence, so a fast ramp measures the
//! same value a slow one would.

use std::sync::atomic::AtomicBool;

use misa_actuator::Actuator;
use misa_actuator_sim::{SimActuator, SimConfig};
use misa_sysid::quasistatic::{
    run_breakaway, run_thermal, BreakawaySpec, Direction, ThermalSpec,
};
use misa_sysid::SafetyLimits;

/// An envelope wide enough not to interfere with what we are measuring.
fn permissive() -> SafetyLimits {
    SafetyLimits {
        max_torque_nm: 50.0,
        max_temperature_c: 500.0,
        max_temperature_rise_c_per_s: 100.0,
        position_window_rad: 100.0,
        max_duration_s: 60.0,
    }
}

fn quick_ramp(ceiling_nm: f32, direction: Direction) -> BreakawaySpec {
    BreakawaySpec {
        ramp_nm_per_s: 1.0,
        max_torque_nm: ceiling_nm,
        rest_window_s: 0.2,
        rest_timeout_s: 2.0,
        ..BreakawaySpec::slow(ceiling_nm, direction)
    }
}

#[test]
fn breakaway_recovers_the_rs04_stiction() {
    let cfg = SimConfig::rs04();
    let planted = cfg.friction.stiction_nm; // 0.902 N·m, measured on the bench
    let mut act = SimActuator::new(cfg);
    let abort = AtomicBool::new(false);

    let result = run_breakaway(&mut act, &quick_ramp(2.0, Direction::Positive), permissive(), &abort)
        .expect("breakaway run failed");

    assert!(result.rested, "the shaft never settled before the ramp");
    let measured = result
        .breakaway_torque_nm
        .expect("ramp hit its ceiling without the shaft moving");
    assert!(
        (measured - planted).abs() < 0.05,
        "measured {measured:.3} N·m, planted {planted:.3} N·m"
    );
}

#[test]
fn breakaway_recovers_the_dm4310_stiction() {
    // A very different magnitude — 0.175 vs 0.902 N·m — so this catches a
    // measurement that happens to land near one motor's value by luck.
    let cfg = SimConfig::dm4310();
    let planted = cfg.friction.stiction_nm;
    let mut act = SimActuator::new(cfg);
    let abort = AtomicBool::new(false);

    let result = run_breakaway(&mut act, &quick_ramp(1.0, Direction::Positive), permissive(), &abort)
        .expect("breakaway run failed");

    let measured = result
        .breakaway_torque_nm
        .expect("ramp hit its ceiling without the shaft moving");
    assert!(
        (measured - planted).abs() < 0.03,
        "measured {measured:.3} N·m, planted {planted:.3} N·m"
    );
}

#[test]
fn breakaway_is_signed_by_the_direction_it_pushes() {
    let cfg = SimConfig::dm4310();
    let planted = cfg.friction.stiction_nm;
    let mut act = SimActuator::new(cfg);
    let abort = AtomicBool::new(false);

    let result = run_breakaway(&mut act, &quick_ramp(1.0, Direction::Negative), permissive(), &abort)
        .expect("breakaway run failed");

    let measured = result.breakaway_torque_nm.expect("no breakaway");
    assert!(measured < 0.0, "pushing negative gave {measured:.3} N·m");
    assert!(
        (measured.abs() - planted).abs() < 0.03,
        "measured |{measured:.3}| N·m, planted {planted:.3} N·m"
    );
}

#[test]
fn a_shaft_that_cannot_be_moved_reports_no_breakaway_rather_than_a_number() {
    // Ceiling below stiction. Reporting `None` here — rather than the ceiling
    // value — is what stops a bounded-below result being mistaken for a
    // measurement.
    let mut act = SimActuator::new(SimConfig::rs04());
    let abort = AtomicBool::new(false);

    let result = run_breakaway(&mut act, &quick_ramp(0.4, Direction::Positive), permissive(), &abort)
        .expect("breakaway run failed");

    assert!(
        result.breakaway_torque_nm.is_none(),
        "reported {:?} against 0.902 N·m of stiction with a 0.4 N·m ceiling",
        result.breakaway_torque_nm
    );
}

#[test]
fn thermal_recovers_the_planted_heating_rate() {
    // The rig's binding is what let the real DM-J4310 hold torque (§1.7 / §4);
    // clamping the shaft here stands in for it, so the run measures heating
    // rather than a leash cancelling the command.
    let mut cfg = SimConfig::dm4310();
    cfg.friction.stiction_nm = 100.0;
    let heating = cfg.thermal.heating_c_per_s_per_nm2;

    let hold_nm = 3.0;
    let expected_rate = heating * hold_nm * hold_nm;

    let mut act = SimActuator::new(cfg);
    let abort = AtomicBool::new(false);
    let spec = ThermalSpec {
        hold_torque_nm: hold_nm,
        duration_s: 4.0,
        rate_hz: 50.0,
        leash_kp: 0.0,
        leash_kd: 0.0,
    };
    let result = run_thermal(&mut act, &spec, permissive(), &abort).expect("thermal run failed");
    assert!(result.abort.is_none(), "aborted: {:?}", result.abort);

    let first = result.points.first().expect("no samples");
    let last = result.points.last().expect("no samples");
    let measured_rate = (last.temperature_c - first.temperature_c) / (last.t_s - first.t_s);
    assert!(
        (measured_rate - expected_rate).abs() / expected_rate < 0.10,
        "measured {measured_rate:.4} °C/s, planted {expected_rate:.4} °C/s"
    );
}

#[test]
fn a_free_shaft_makes_the_thermal_run_report_that_the_torque_never_arrived() {
    // The RS04 case from §2.5: a leash on a free shaft settles where it
    // cancels the feed-forward, so almost none of the commanded torque is
    // delivered. The run has to say so instead of reporting a thermal result.
    let mut act = SimActuator::new(SimConfig::rs04());
    let abort = AtomicBool::new(false);
    let spec = ThermalSpec {
        hold_torque_nm: 2.0,
        duration_s: 3.0,
        rate_hz: 50.0,
        leash_kp: 5.0,
        leash_kd: 0.5,
    };
    let result = run_thermal(&mut act, &spec, permissive(), &abort).expect("thermal run failed");

    // `None` means the driver reported no usable torque at all, which on this
    // preset is itself the honest answer (RS04's `MeasuredTorque` reads zero
    // outside MIT) — either way the run must not look like a thermal result.
    match result.delivered_fraction() {
        Some(delivered) => assert!(
            delivered < 0.5,
            "a free shaft delivered {delivered:.2} of the commanded torque — \
             the leash should have cancelled most of it"
        ),
        None => {}
    }
}

#[test]
fn the_simulator_survives_being_driven_by_every_control_mode_in_turn() {
    // A smoke test over the whole `Actuator` surface, so a mode that panics or
    // wedges shows up here rather than inside a characterization run.
    let mut act = SimActuator::new(SimConfig::ideal());
    act.enable().expect("enable");
    act.set_zero().expect("set_zero");
    for _ in 0..50 {
        act.set_position(0.2, 2.0).expect("set_position");
    }
    for _ in 0..50 {
        act.set_velocity(1.0).expect("set_velocity");
    }
    for _ in 0..50 {
        act.set_torque(0.05).expect("set_torque");
    }
    for _ in 0..50 {
        act.mit_control(0.0, 0.0, 5.0, 0.5, 0.0).expect("mit");
    }
    let fb = act.measure().expect("measure");
    assert!(fb.position_rad.is_finite());
    assert!(fb.velocity_rad_per_s.is_finite());
    act.disable().expect("disable");
}
