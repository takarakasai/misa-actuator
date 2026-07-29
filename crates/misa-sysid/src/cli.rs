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
use crate::quasistatic::{BreakawaySpec, Direction, KtSpec, LoadMapSpec, ThermalSpec};
use crate::report::{
    run_breakaway_to_csv, run_kt_to_csv, run_load_map_to_csv, run_thermal_to_csv,
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
        /// Push negative instead of positive.
        #[arg(long)]
        negative: bool,
        /// Ramp both directions in turn; the difference separates friction from
        /// the static load.
        #[arg(long)]
        both: bool,
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
    },
    /// Torque vs current, fitted to a torque constant.
    ///
    /// Needs a driver that reports current. DAMIAO's feedback frame does not, so
    /// current is derived from the motor's `KT_Value` register when that is set
    /// — compare the fitted figure against the register value from `params`.
    Kt {
        /// Largest |torque| commanded (N·m).
        #[arg(long, default_value_t = 0.5)]
        max_torque: f32,
        /// Levels from -max to +max, including zero.
        #[arg(long, default_value_t = 9)]
        steps: usize,
        /// Dwell per level (s), so the current reading is steady-state.
        #[arg(long, default_value_t = 0.4)]
        settle: f32,
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
            let r = run_load_map_to_csv(act, &spec, limits, &abort, &mut csv)?;
            println!("  points measured        : {}", r.n_points);
            print_opt("equilibrium position", r.equilibrium_position_rad, "rad");
            print_opt("peak holding torque", r.peak_holding_torque_nm, "N·m");
            match r.peak_hysteresis_nm {
                Some(h) => println!(
                    "  peak hysteresis        : {h:.4} N·m  (Coulomb friction ≈ {:.4} N·m)",
                    h / 2.0
                ),
                None => println!("  peak hysteresis        : <no return pass>"),
            }
            report_abort(r.abort);
        }
        CharacterizeCmd::Breakaway {
            ramp,
            ceiling,
            motion_speed,
            motion_travel,
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
            for d in dirs {
                let spec = BreakawaySpec {
                    ramp_nm_per_s: *ramp,
                    max_torque_nm: *ceiling,
                    motion_threshold_rad_per_s: *motion_speed,
                    motion_threshold_rad: *motion_travel,
                    direction: d,
                    ..BreakawaySpec::slow(*ceiling, d)
                };
                println!("breakaway {d:?}: ramping at {ramp} N·m/s up to {ceiling} N·m ...");
                let r = run_breakaway_to_csv(act, &spec, limits, &abort, &mut csv)?;
                match r.breakaway_torque_nm {
                    Some(t) => println!("  breakaway torque       : {t:+.4} N·m"),
                    None => println!(
                        "  breakaway torque       : none — did not move up to {:.4} N·m",
                        r.peak_commanded_nm
                    ),
                }
                print_opt("breakaway position", r.breakaway_position_rad, "rad");
                report_abort(r.abort);
                results.push(r);
            }
            // With both directions, the mean is the static load and half the
            // difference is the friction that opposes motion either way.
            if let [a, b] = results.as_slice() {
                if let (Some(ta), Some(tb)) = (a.breakaway_torque_nm, b.breakaway_torque_nm) {
                    println!("  static load (mean)     : {:+.4} N·m", (ta + tb) / 2.0);
                    println!("  friction (half-spread) : {:.4} N·m", (ta - tb).abs() / 2.0);
                }
            }
        }
        CharacterizeCmd::Thermal {
            torque,
            duration,
            rate,
        } => {
            let spec = ThermalSpec {
                hold_torque_nm: *torque,
                duration_s: *duration,
                rate_hz: *rate,
            };
            println!("thermal: holding {torque} N·m for up to {duration}s ...");
            let r = run_thermal_to_csv(act, &spec, limits, &abort, &mut csv)?;
            println!("  samples                : {}", r.n_points);
            print_opt("mean torque delivered", r.mean_torque_nm, "N·m");
            match r.rise_rate_c_per_s {
                Some(rate) => println!("  temperature rise rate  : {rate:+.3} °C/s"),
                None => println!("  temperature rise rate  : <not reported by this driver>"),
            }
            match r.seconds_to_limit {
                Some(s) => println!(
                    "  time to {:.0} °C          : {s:.1} s at this torque",
                    limits.max_temperature_c
                ),
                None => println!("  time to limit          : not warming — sustainable"),
            }
            report_abort(r.abort);
        }
        CharacterizeCmd::Kt {
            max_torque,
            steps,
            settle,
        } => {
            let spec = KtSpec {
                max_torque_nm: *max_torque,
                steps: *steps,
                settle_s: *settle,
                ..KtSpec::bipolar(*max_torque)
            };
            println!("Kt sweep: ±{max_torque} N·m in {steps} levels, {settle}s dwell ...");
            let r = run_kt_to_csv(act, &spec, limits, &abort, &mut csv)?;
            println!("  levels measured        : {}", r.n_points);
            match r.kt_nm_per_a {
                Some(kt) => {
                    println!("  Kt (least squares)     : {kt:.4} N·m/A");
                    print_opt("Kt (pointwise mean)", r.pointwise_kt_nm_per_a, "N·m/A");
                    print_opt("fit R^2", r.r_squared, "");
                }
                None => println!(
                    "  Kt                     : <no current reported> — DAMIAO's feedback frame \
                     carries no current field; set KT_Value on the motor (see `params`) so the \
                     driver can derive it"
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

