//! CLI-facing convenience wrappers around [`crate::quasistatic`].
//!
//! Same role as [`crate::runner::run_chirp_to_csv`]: run the measurement, write
//! the raw log, and hand back the derived scalars, so each vendor CLI needs
//! only argument parsing and a `println!` rather than its own copy of the
//! analysis. Every wrapper writes the CSV *before* inspecting the abort reason,
//! so an aborted run still leaves its data on disk.
//!
//! `write_header` is passed through so several runs can share one file — a
//! two-direction breakaway is two ramps in one CSV, and only the first should
//! emit the header row.

use std::io::Write;
use std::sync::atomic::AtomicBool;

use misa_actuator::{Actuator, Result};

use crate::limits::{AbortReason, SafetyLimits};
use crate::quasistatic::{
    run_breakaway, run_kt, run_load_map, run_thermal, write_points_csv, BreakawaySpec, KtSpec,
    LoadMapSpec, ThermalSpec,
};

/// Derived results of a load-map run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoadMapReport {
    pub n_points: usize,
    /// Position where *raw* holding torque is smallest. Friction noise unless
    /// the static load dominates — prefer `static_equilibrium_position_rad`.
    pub equilibrium_position_rad: Option<f32>,
    /// Equilibrium after removing friction (mean of the two passes).
    pub static_equilibrium_position_rad: Option<f32>,
    /// Mean Coulomb friction from the outbound/return spread (N·m).
    pub mean_friction_nm: Option<f32>,
    /// Peak |static load| once friction is removed (N·m). Well below
    /// `mean_friction_nm` means this span has no real gravity/spring term.
    pub peak_static_load_nm: Option<f32>,
    /// Largest |holding torque| over the sweep (N·m).
    pub peak_holding_torque_nm: Option<f32>,
    /// Largest outbound-vs-return torque gap (N·m); about twice the Coulomb
    /// friction. `None` unless a return sweep ran.
    pub peak_hysteresis_nm: Option<f32>,
    pub abort: Option<AbortReason>,
}

/// Map holding torque against position and write the raw points as CSV.
pub fn run_load_map_to_csv(
    act: &mut dyn Actuator,
    spec: &LoadMapSpec,
    limits: SafetyLimits,
    abort: &AtomicBool,
    csv: &mut dyn Write,
    write_header: bool,
) -> Result<LoadMapReport> {
    let map = run_load_map(act, spec, limits, abort)?;
    write_points_csv(&map.points, csv, write_header)?;
    Ok(LoadMapReport {
        n_points: map.points.len(),
        equilibrium_position_rad: map.equilibrium_position_rad(),
        static_equilibrium_position_rad: map.static_equilibrium_position_rad(),
        mean_friction_nm: map.mean_friction_nm(),
        peak_static_load_nm: map.peak_static_load_nm(),
        peak_holding_torque_nm: map.peak_holding_torque_nm(),
        peak_hysteresis_nm: map.peak_hysteresis_nm(),
        abort: map.abort,
    })
}

/// Derived results of a breakaway ramp.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BreakawayReport {
    pub n_points: usize,
    /// Torque at which motion started (N·m, signed). `None` means the ramp
    /// reached its ceiling without the shaft moving, which bounds friction from
    /// below instead of measuring it.
    pub breakaway_torque_nm: Option<f32>,
    pub breakaway_position_rad: Option<f32>,
    /// Largest torque actually commanded (N·m) — the bound when there was no
    /// breakaway.
    pub peak_commanded_nm: f32,
    /// Whether the shaft was at rest before the ramp. `false` invalidates the
    /// breakaway torque: the motion test can trip on pre-existing motion.
    pub rested: bool,
    pub abort: Option<AbortReason>,
}

/// Ramp torque until the shaft moves and write the ramp as CSV.
pub fn run_breakaway_to_csv(
    act: &mut dyn Actuator,
    spec: &BreakawaySpec,
    limits: SafetyLimits,
    abort: &AtomicBool,
    csv: &mut dyn Write,
    write_header: bool,
) -> Result<BreakawayReport> {
    let r = run_breakaway(act, spec, limits, abort)?;
    write_points_csv(&r.points, csv, write_header)?;
    Ok(BreakawayReport {
        n_points: r.points.len(),
        breakaway_torque_nm: r.breakaway_torque_nm,
        breakaway_position_rad: r.breakaway_position_rad,
        peak_commanded_nm: r.points.iter().map(|p| p.cmd.abs()).fold(0.0, f32::max),
        rested: r.rested,
        abort: r.abort,
    })
}

/// Derived results of a thermal hold.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalReport {
    pub n_points: usize,
    /// Mean torque actually delivered (N·m), which can fall short of the
    /// command once the motor saturates.
    pub mean_torque_nm: Option<f32>,
    /// Least-squares temperature slope (°C/s). `None` if the driver reports no
    /// temperature.
    pub rise_rate_c_per_s: Option<f32>,
    /// Seconds from the last reading to the envelope's temperature limit at the
    /// measured rate. `None` when not warming.
    pub seconds_to_limit: Option<f32>,
    /// Fraction of the commanded torque actually delivered. Well below 1 means
    /// the shaft was not restrained and the leash cancelled the feed-forward,
    /// so the thermal numbers do not describe the requested torque.
    pub delivered_fraction: Option<f32>,
    pub abort: Option<AbortReason>,
}

/// Hold a torque, log the temperature rise, and write the samples as CSV.
pub fn run_thermal_to_csv(
    act: &mut dyn Actuator,
    spec: &ThermalSpec,
    limits: SafetyLimits,
    abort: &AtomicBool,
    csv: &mut dyn Write,
    write_header: bool,
) -> Result<ThermalReport> {
    let t = run_thermal(act, spec, limits, abort)?;
    write_points_csv(&t.points, csv, write_header)?;
    Ok(ThermalReport {
        n_points: t.points.len(),
        mean_torque_nm: t.mean_torque_nm(),
        rise_rate_c_per_s: t.rise_rate_c_per_s(),
        seconds_to_limit: t.seconds_to_limit(limits.max_temperature_c),
        delivered_fraction: t.delivered_fraction(),
        abort: t.abort,
    })
}

/// Derived results of a torque/current sweep.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KtReport {
    pub n_points: usize,
    /// Torque constant from a least-squares fit (N·m/A). `None` on drivers that
    /// do not report current — DAMIAO's feedback frame has no current field.
    pub kt_nm_per_a: Option<f32>,
    /// Mean |torque/current| over the non-zero levels (N·m/A) — a cross-check
    /// that does not assume a zero intercept.
    pub pointwise_kt_nm_per_a: Option<f32>,
    /// Linearity of the fit, `0.0..=1.0`. Well below 1 means saturation, a
    /// fault, or too few usable points.
    pub r_squared: Option<f32>,
    pub abort: Option<AbortReason>,
}

/// Sweep torque against current, fit a torque constant, and write the points as
/// CSV.
pub fn run_kt_to_csv(
    act: &mut dyn Actuator,
    spec: &KtSpec,
    limits: SafetyLimits,
    abort: &AtomicBool,
    csv: &mut dyn Write,
    write_header: bool,
) -> Result<KtReport> {
    let s = run_kt(act, spec, limits, abort)?;
    write_points_csv(&s.points, csv, write_header)?;
    Ok(KtReport {
        n_points: s.points.len(),
        kt_nm_per_a: s.kt_nm_per_a(),
        pointwise_kt_nm_per_a: s.pointwise_kt_nm_per_a(),
        r_squared: s.r_squared(),
        abort: s.abort,
    })
}
