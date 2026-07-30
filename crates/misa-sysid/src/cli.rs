//! Ready-made `clap` subcommand for the quasi-static characterizations.
//!
//! Enabled by the `cli` feature. Each vendor CLI embeds [`CharacterizeCmd`] as a
//! subcommand and calls [`run_characterize`]; without this the same ~270 lines of
//! argument parsing and reporting would be copied into every CLI.
//!
//! The safety envelope is built by the caller (so each CLI can pick its own
//! defaults and flag names) and enforced inside [`crate::quasistatic`].

use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use clap::Subcommand;
use misa_actuator::{Actuator, Result};

use crate::limits::{AbortReason, SafetyLimits};
use crate::quasistatic::{
    BreakawayMapSpec, BreakawaySpec, Direction, KtSpec, LoadMapSpec, ThermalSpec,
    VelocitySweepSpec,
};
use crate::report::{
    run_breakaway_map_to_csv, run_breakaway_to_csv, run_kt_to_csv, run_load_map_to_csv,
    run_thermal_to_csv, run_velocity_sweep_to_csv,
};

/// Which quasi-static measurement to run.
#[derive(Subcommand, Debug, Clone)]
pub enum CharacterizeCmd {
    /// Holding torque vs position — finds the load's equilibrium and separates
    /// gravity/spring load from friction. Run this first.
    LoadMap {
        /// Sweep +/- this far around the current position (rad).
        #[arg(long, default_value_t = 0.2)]
        span: f32,
        /// Positions visited per pass.
        #[arg(long, default_value_t = 9)]
        steps: usize,
        /// Dwell at each position before recording (s), so the reading is a
        /// holding torque and not the acceleration transient.
        #[arg(long, default_value_t = 0.3)]
        settle: f32,
        /// Speed cap while moving between positions (rad/s).
        #[arg(long, default_value_t = 0.3)]
        speed: f32,
        /// Skip the return pass (which is what exposes hysteresis).
        #[arg(long)]
        no_return: bool,
    },
    /// Torque at which the shaft starts moving — stiction, cogging, gear drag.
    ///
    /// Most informative from the equilibrium position, where the spring/gravity
    /// term does not mask the friction term.
    Breakaway {
        /// Torque ramp rate (N·m/s). Slow keeps it a static measurement.
        #[arg(long, default_value_t = 0.2)]
        ramp: f32,
        /// Ramp ceiling (N·m). Clamped to --max-torque.
        #[arg(long, default_value_t = 1.0)]
        ceiling: f32,
        /// Speed above which the shaft counts as moving (rad/s). Must clear the
        /// velocity noise floor — a bench DM-J4310 idles near 0.02 rad/s.
        #[arg(long, default_value_t = 0.05)]
        motion_speed: f32,
        /// Displacement that also counts as motion (rad), for a shaft that
        /// creeps below the speed threshold.
        #[arg(long, default_value_t = 0.02)]
        motion_travel: f32,
        /// How long motion must persist to count as breakaway (s). Rejects
        /// judder, which otherwise trips the threshold far below the torque
        /// needed to actually drive the shaft. 0 restores single-crossing.
        #[arg(long, default_value_t = 0.1)]
        motion_hold: f32,
        /// Push negative instead of positive.
        #[arg(long)]
        negative: bool,
        /// Ramp both directions in turn; the difference separates friction from
        /// the static load.
        #[arg(long)]
        both: bool,
    },
    /// Load and stiction vs position, measured under **torque** control
    ///
    /// Use instead of `load-map` when the position loop cannot hold still.
    /// `load-map` reads the torque a position loop emits to hold a setpoint, so
    /// a loop that hunts corrupts the measurement (seen on an RS04). Here the
    /// position loop only travels between points and its behaviour is discarded;
    /// each measurement is a torque ramp judged by velocity, needing no torque
    /// feedback at all.
    ///
    /// Slower than `load-map` — every position pays a ramp per direction — and
    /// it measures *static* friction where a `velocity-sweep` measures kinetic.
    BreakawayMap {
        /// Sweep +/- this far around the current position (rad).
        #[arg(long, default_value_t = 0.2)]
        span: f32,
        /// Positions visited.
        #[arg(long, default_value_t = 5)]
        steps: usize,
        /// Torque ramp rate at each position (N·m/s).
        #[arg(long, default_value_t = 0.2)]
        ramp: f32,
        /// Ramp ceiling (N·m). Clamped to --max-torque.
        #[arg(long, default_value_t = 1.0)]
        ceiling: f32,
        /// Speed above which the shaft counts as moving (rad/s).
        #[arg(long, default_value_t = 0.05)]
        motion_speed: f32,
        /// Displacement that also counts as motion (rad).
        #[arg(long, default_value_t = 0.02)]
        motion_travel: f32,
        /// How long motion must persist to count as breakaway (s).
        #[arg(long, default_value_t = 0.1)]
        motion_hold: f32,
        /// Speed cap while travelling between positions (rad/s).
        #[arg(long, default_value_t = 0.3)]
        travel_speed: f32,
        /// Only ramp one direction, which forgoes the load/stiction split.
        #[arg(long)]
        single_direction: bool,
    },
    /// Load and kinetic friction vs position, from a slow constant-speed traverse
    ///
    /// The other way to keep a hunting position loop out of the measurement:
    /// keep the shaft moving instead of holding it. A velocity loop is naturally
    /// damped, and in steady motion torque splits into load +/- kinetic friction
    /// by direction, densely and with no settling.
    ///
    /// Needs working torque feedback (unlike `breakaway-map`) — on RobStride
    /// firmware whose `MeasuredTorque` reads 0, pass `--kt`.
    VelocitySweep {
        /// Traverse +/- this far around the current position (rad).
        #[arg(long, default_value_t = 0.2)]
        span: f32,
        /// Traverse speed (rad/s). Slow keeps inertia out of it.
        #[arg(long, default_value_t = 0.05)]
        speed: f32,
        /// Log rate (Hz).
        #[arg(long, default_value_t = 200.0)]
        rate: f32,
        /// Position bins the two passes are averaged into.
        #[arg(long, default_value_t = 20)]
        bins: usize,
        /// Only traverse one way, which forgoes the load/friction split.
        #[arg(long)]
        single_pass: bool,
    },
    /// Temperature rise while holding a fixed torque — turns "this got hot" into
    /// a continuous-rating number.
    Thermal {
        /// Torque to hold (N·m, signed).
        #[arg(long, allow_hyphen_values = true)]
        torque: f32,
        /// How long to hold (s).
        #[arg(long, default_value_t = 10.0)]
        duration: f32,
        /// Sample rate (Hz). A few Hz suffices — these sensors are slow and
        /// usually quantized to 1 °C.
        #[arg(long, default_value_t = 20.0)]
        rate: f32,
        /// Position-leash stiffness (N·m/rad) holding the shaft where it started
        /// while the torque heats the motor. 0 commands open-loop torque, which
        /// only works on a shaft that is mechanically restrained — otherwise any
        /// torque above breakaway accelerates it out of the safety window before
        /// any thermal data is collected.
        #[arg(long, default_value_t = 8.0)]
        leash_kp: f32,
        /// Leash damping (N·m·s/rad).
        #[arg(long, default_value_t = 0.5)]
        leash_kd: f32,
    },
    /// Torque vs current, fitted to a torque constant.
    ///
    /// Needs a driver that reports current. DAMIAO's feedback frame does not, so
    /// current is derived from the motor's `KT_Value` register when that is set
    /// — compare the fitted figure against the register value from `params`.
    Kt {
        /// Largest |torque| commanded (N·m).
        ///
        /// Named `--amplitude` rather than `--max-torque` so it cannot collide
        /// with the envelope's global `--max-torque`: clap would fold the two
        /// into one value, silently tightening the envelope to the sweep
        /// amplitude and aborting on the first level.
        #[arg(long, default_value_t = 0.5)]
        amplitude: f32,
        /// Levels from -max to +max, including zero.
        #[arg(long, default_value_t = 9)]
        steps: usize,
        /// Dwell per level (s), so the current reading is steady-state.
        #[arg(long, default_value_t = 0.4)]
        settle: f32,
        /// Position-leash stiffness (N·m/rad) holding the shaft while each level
        /// is applied. 0 commands open-loop torque, which needs a mechanically
        /// restrained shaft — otherwise the level accelerates it out of the
        /// safety window before any steady-state current is reached.
        #[arg(long, default_value_t = 8.0)]
        leash_kp: f32,
        /// Leash damping (N·m·s/rad).
        #[arg(long, default_value_t = 0.5)]
        leash_kd: f32,
    },
}


/// Run one quasi-static characterization and print its derived numbers.
///
/// The safety envelope is enforced inside `misa-sysid`; this only reports what
/// came back, including the abort reason when a run stopped early. An aborted
/// run still wrote its CSV — that partial data is usually the interesting part.
pub fn run_characterize(
    act: &mut dyn Actuator,
    what: &CharacterizeCmd,
    limits: SafetyLimits,
    out: &Path,
) -> Result<()> {
    let abort = Arc::new(AtomicBool::new(false));
    {
        let a = abort.clone();
        let _ = ctrlc::set_handler(move || a.store(true, Ordering::SeqCst));
    }
    println!(
        "safety envelope: |tau| <= {} N·m, temp <= {} °C, rise <= {} °C/s, pos within ±{} rad, \
         <= {} s (Ctrl-C aborts)",
        limits.max_torque_nm,
        limits.max_temperature_c,
        limits.max_temperature_rise_c_per_s,
        limits.position_window_rad,
        limits.max_duration_s
    );

    // One writer for the whole command, so --both appends its second ramp.
    let mut csv = std::io::BufWriter::new(std::fs::File::create(out)?);

    match what {
        CharacterizeCmd::LoadMap {
            span,
            steps,
            settle,
            speed,
            no_return,
        } => {
            let spec = LoadMapSpec {
                settle_s: *settle,
                max_speed_rad_s: *speed,
                return_sweep: !*no_return,
                ..LoadMapSpec::symmetric(*span, *steps)
            };
            println!("load map: ±{span} rad, {steps} steps/pass, {settle}s dwell ...");
            let r = run_load_map_to_csv(act, &spec, limits, &abort, &mut csv, true)?;
            println!("  points measured        : {}", r.n_points);
            print_opt("peak holding torque", r.peak_holding_torque_nm, "N·m");
            // The raw and friction-compensated views answer different
            // questions; printing both makes it obvious when friction is doing
            // all the work.
            print_opt("equilibrium (raw)", r.equilibrium_position_rad, "rad");
            print_opt(
                "equilibrium (de-frictioned)",
                r.static_equilibrium_position_rad,
                "rad",
            );
            print_opt("mean friction", r.mean_friction_nm, "N·m");
            print_opt("peak static load", r.peak_static_load_nm, "N·m");
            print_opt("worst dwell spread", r.worst_dwell_spread_nm, "N·m");
            // A dwell that swings as much as the load itself was not holding
            // still; the point is the mean of a limit cycle.
            if let (Some(spread), Some(peak)) = (r.worst_dwell_spread_nm, r.peak_holding_torque_nm) {
                if peak > 0.0 && spread >= peak {
                    println!(
                        "  note: a dwell swung {spread:.4} N·m against a peak holding torque of \
                         {peak:.4} — the shaft was hunting, not settling, so treat the load \
                         figures as averages of that oscillation (try a lower position gain or a \
                         longer --settle)"
                    );
                }
            }
            if let (Some(load), Some(fric)) = (r.peak_static_load_nm, r.mean_friction_nm) {
                if load < fric {
                    println!(
                        "  note: peak static load ({load:.4} N·m) is below the friction \
                         ({fric:.4} N·m) — over this span the rig is friction-dominated, so \
                         the raw equilibrium is not meaningful"
                    );
                }
            }
            report_abort(r.abort);
        }
        CharacterizeCmd::Breakaway {
            ramp,
            ceiling,
            motion_speed,
            motion_travel,
            motion_hold,
            negative,
            both,
        } => {
            let first = if *negative {
                Direction::Negative
            } else {
                Direction::Positive
            };
            let dirs: Vec<Direction> = if *both {
                vec![first, first.flipped()]
            } else {
                vec![first]
            };
            let mut results = Vec::new();
            for (i, d) in dirs.into_iter().enumerate() {
                let spec = BreakawaySpec {
                    ramp_nm_per_s: *ramp,
                    max_torque_nm: *ceiling,
                    motion_threshold_rad_per_s: *motion_speed,
                    motion_threshold_rad: *motion_travel,
                    motion_hold_s: *motion_hold,
                    direction: d,
                    ..BreakawaySpec::slow(*ceiling, d)
                };
                println!("breakaway {d:?}: ramping at {ramp} N·m/s up to {ceiling} N·m ...");
                let r = run_breakaway_to_csv(act, &spec, limits, &abort, &mut csv, i == 0)?;
                match r.breakaway_torque_nm {
                    Some(t) => println!("  breakaway torque       : {t:+.4} N·m"),
                    None => println!(
                        "  breakaway torque       : none — did not move up to {:.4} N·m",
                        r.peak_commanded_nm
                    ),
                }
                print_opt("breakaway position", r.breakaway_position_rad, "rad");
                if !r.rested {
                    eprintln!(
                        "  WARNING: the shaft never came to rest before the ramp — this figure \
                         may be pre-existing motion rather than a breakaway"
                    );
                }
                report_abort(r.abort);
                results.push(r);
            }
            // With both directions, the mean is the static load and half the
            // difference is the friction that opposes motion either way.
            if let [a, b] = results.as_slice() {
                // Only meaningful if both ramps started from rest; otherwise the
                // mean/spread are arithmetic on an artefact.
                if let (Some(ta), Some(tb), true) =
                    (a.breakaway_torque_nm, b.breakaway_torque_nm, a.rested && b.rested)
                {
                    println!("  static load (mean)     : {:+.4} N·m", (ta + tb) / 2.0);
                    println!("  friction (half-spread) : {:.4} N·m", (ta - tb).abs() / 2.0);
                }
            }
        }
        CharacterizeCmd::BreakawayMap {
            span,
            steps,
            ramp,
            ceiling,
            motion_speed,
            motion_travel,
            motion_hold,
            travel_speed,
            single_direction,
        } => {
            let ramp_spec = BreakawaySpec {
                ramp_nm_per_s: *ramp,
                max_torque_nm: *ceiling,
                motion_threshold_rad_per_s: *motion_speed,
                motion_threshold_rad: *motion_travel,
                motion_hold_s: *motion_hold,
                ..BreakawaySpec::slow(*ceiling, Direction::Positive)
            };
            let spec = BreakawayMapSpec {
                travel_speed_rad_s: *travel_speed,
                both_directions: !*single_direction,
                ..BreakawayMapSpec::symmetric(*span, *steps, ramp_spec)
            };
            println!(
                "breakaway map: ±{span} rad in {steps} positions, ramping {ramp} N·m/s to \
                 {ceiling} N·m ..."
            );
            let r = run_breakaway_map_to_csv(act, &spec, limits, &abort, &mut csv, true)?;
            println!("  positions measured     : {}", r.n_positions);
            print_opt("mean stiction", r.mean_stiction_nm, "N·m");
            print_opt("peak static load", r.peak_static_load_nm, "N·m");
            print_opt("equilibrium position", r.equilibrium_position_rad, "rad");
            if let (Some(load), Some(stic)) = (r.peak_static_load_nm, r.mean_stiction_nm) {
                if load < stic {
                    println!(
                        "  note: peak static load ({load:.4} N·m) is below the stiction \
                         ({stic:.4} N·m) — friction-dominated over this span"
                    );
                }
            }
            if !r.unrested_positions.is_empty() {
                eprintln!(
                    "  WARNING: {} position(s) ramped without settling first; their figures may \
                     be pre-existing motion",
                    r.unrested_positions.len()
                );
            }
            if !r.curve.is_empty() {
                println!("  position      static load   stiction");
                for (pos, load, stic) in &r.curve {
                    println!("  {pos:+10.4} {load:+13.4} {stic:10.4}");
                }
            }
            report_abort(r.abort);
        }
        CharacterizeCmd::VelocitySweep {
            span,
            speed,
            rate,
            bins,
            single_pass,
        } => {
            let spec = VelocitySweepSpec {
                speed_rad_s: *speed,
                half_span_rad: *span,
                rate_hz: *rate,
                return_sweep: !*single_pass,
            };
            println!("velocity sweep: ±{span} rad at {speed} rad/s, logging at {rate} Hz ...");
            let r =
                run_velocity_sweep_to_csv(act, &spec, limits, &abort, &mut csv, true, *bins)?;
            println!("  samples                : {}", r.n_samples);
            print_opt("mean speed achieved", r.mean_speed_rad_s, "rad/s");
            print_opt("mean kinetic friction", r.mean_kinetic_friction_nm, "N·m");
            print_opt("peak static load", r.peak_static_load_nm, "N·m");
            // A traverse that stalled is not steady motion, so the split is void.
            if let Some(achieved) = r.mean_speed_rad_s {
                if achieved < speed * 0.5 {
                    println!(
                        "  note: achieved only {achieved:.4} of {speed} rad/s — the traverse \
                         stalled, so load and friction cannot be separated from this log"
                    );
                }
            }
            if !r.curve.is_empty() {
                println!("  position      static load   kinetic friction");
                for (pos, load, fric) in &r.curve {
                    println!("  {pos:+10.4} {load:+13.4} {fric:18.4}");
                }
            }
            report_abort(r.abort);
        }
        CharacterizeCmd::Thermal {
            torque,
            duration,
            rate,
            leash_kp,
            leash_kd,
        } => {
            let spec = ThermalSpec {
                hold_torque_nm: *torque,
                duration_s: *duration,
                rate_hz: *rate,
                leash_kp: *leash_kp,
                leash_kd: *leash_kd,
            };
            if *leash_kp > 0.0 {
                println!(
                    "thermal: holding {torque} N·m for up to {duration}s, leashed at kp={leash_kp} \
                     kd={leash_kd} ..."
                );
            } else {
                println!(
                    "thermal: holding {torque} N·m open-loop for up to {duration}s — the shaft \
                     must be mechanically restrained ..."
                );
            }
            let r = run_thermal_to_csv(act, &spec, limits, &abort, &mut csv, true)?;
            println!("  samples                : {}", r.n_points);
            print_opt("mean torque delivered", r.mean_torque_nm, "N·m");
            match r.rise_rate_c_per_s {
                Some(rate) => println!("  temperature rise rate  : {rate:+.3} °C/s"),
                None => println!("  temperature rise rate  : <not reported by this driver>"),
            }
            // A leash-cancelled hold produces almost no torque, so any thermal
            // conclusion drawn from it would be about a torque the motor never
            // delivered. Say so instead of implying a rating.
            let delivered_ok = r.delivered_fraction.is_none_or(|f| f >= 0.5);
            print_opt("delivered fraction", r.delivered_fraction, "of command");
            match (r.seconds_to_limit, delivered_ok) {
                (_, false) => println!(
                    "  time to limit          : not computed — the motor delivered only {:.0}% of \
                     the commanded torque, so this run says nothing about its thermal limit",
                    r.delivered_fraction.unwrap_or(0.0) * 100.0
                ),
                (Some(s), true) => println!(
                    "  time to {:.0} °C          : {s:.1} s at this torque",
                    limits.max_temperature_c
                ),
                (None, true) => {
                    println!("  time to limit          : not warming at this torque")
                }
            }
            if !delivered_ok {
                eprintln!(
                    "  WARNING: the shaft was free enough that the leash cancelled the \
                     feed-forward. A fixed-torque thermal test needs the output shaft \
                     mechanically restrained."
                );
            }
            report_abort(r.abort);
        }
        CharacterizeCmd::Kt {
            amplitude,
            steps,
            settle,
            leash_kp,
            leash_kd,
        } => {
            let spec = KtSpec {
                max_torque_nm: *amplitude,
                steps: *steps,
                settle_s: *settle,
                leash_kp: *leash_kp,
                leash_kd: *leash_kd,
                ..KtSpec::bipolar(*amplitude)
            };
            println!("Kt sweep: ±{amplitude} N·m in {steps} levels, {settle}s dwell ...");
            let r = run_kt_to_csv(act, &spec, limits, &abort, &mut csv, true)?;
            println!("  levels measured        : {}", r.n_points);
            match r.kt_nm_per_a {
                Some(kt) => {
                    println!("  Kt (least squares)     : {kt:.4} N·m/A");
                    print_opt("Kt (pointwise mean)", r.pointwise_kt_nm_per_a, "N·m/A");
                    print_opt("fit R^2", r.r_squared, "");
                }
                None => println!(
                    "  Kt                     : <no current reported by this driver>. RobStride: \
                     pass --report-current (its feedback frame has no current field). DAMIAO: \
                     pass --refresh-kt so current can be derived from torque/Kt."
                ),
            }
            report_abort(r.abort);
        }
    }

    println!("wrote raw samples to {}", out.display());
    Ok(())
}

fn print_opt(label: &str, v: Option<f32>, unit: &str) {
    match v {
        Some(v) if unit.is_empty() => println!("  {label:<22} : {v:.4}"),
        Some(v) => println!("  {label:<22} : {v:.4} {unit}"),
        None => println!("  {label:<22} : <not available>"),
    }
}

fn report_abort(abort: Option<AbortReason>) {
    if let Some(r) = abort {
        eprintln!("  ABORTED: {} ({r:?}) — data up to that point was still written", r.describe());
    }
}

