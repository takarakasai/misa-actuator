//! Repeated friction measurements and their distribution.
//!
//! A single breakaway or sweep gives one number; the scatter across repetitions
//! is what says how much to trust it. On an RS04 five positions of a breakaway
//! map returned stiction between 0.042 and 0.166 N·m — a 4x spread — so any
//! single figure there is nearly meaningless on its own.
//!
//! Every repetition also records temperature, because friction is not expected
//! to be stationary: repeated runs heat the motor, and grease viscosity and
//! magnet flux both move with it. [`FrictionSeries::temperature_trend`] and
//! [`FrictionSeries::friction_vs_temperature`] exist so a drifting series is
//! recognised as drift rather than averaged into a meaningless mean.

use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;

use misa_actuator::{Actuator, Result};

use crate::limits::{AbortReason, SafetyLimits};
use crate::quasistatic::{
    run_breakaway, run_velocity_sweep, BreakawaySpec, Direction, VelocitySweepSpec,
};

/// Which friction a series measured.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrictionKind {
    /// Torque to start motion, from a breakaway ramp.
    Static,
    /// Torque to sustain motion, from a constant-speed traverse.
    Kinetic,
}

impl FrictionKind {
    pub const fn label(self) -> &'static str {
        match self {
            FrictionKind::Static => "static",
            FrictionKind::Kinetic => "kinetic",
        }
    }
}

/// One repetition's outcome.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrictionSample {
    /// 0-based repetition index.
    pub index: u32,
    /// Seconds since the series began, so drift can be plotted against time.
    pub t_s: f32,
    /// Friction estimate (N·m). `None` if that repetition failed to produce one.
    pub friction_nm: Option<f32>,
    /// Static (conservative) load estimate (N·m).
    pub static_load_nm: Option<f32>,
    /// Temperature at the repetition (°C), or `NaN` if the driver reports none.
    pub temperature_c: f32,
    /// Where the measurement was taken (rad).
    pub position_rad: f32,
}

/// Summary of one quantity across a series.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stats {
    pub n: usize,
    pub mean: f32,
    /// Sample standard deviation (n-1). `0.0` for a single sample.
    pub sd: f32,
    pub min: f32,
    pub max: f32,
    pub median: f32,
    pub p10: f32,
    pub p90: f32,
}

impl Stats {
    /// Compute from finite values. `None` if none are finite.
    pub fn of(values: &[f32]) -> Option<Self> {
        let mut v: Vec<f32> = values.iter().copied().filter(|x| x.is_finite()).collect();
        if v.is_empty() {
            return None;
        }
        v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
        let n = v.len();
        let mean = v.iter().sum::<f32>() / n as f32;
        // Sample (n-1) variance: these are repeated draws from a process, not a
        // whole population.
        let sd = if n > 1 {
            (v.iter().map(|x| (x - mean).powi(2)).sum::<f32>() / (n - 1) as f32).sqrt()
        } else {
            0.0
        };
        let pick = |q: f32| v[(((n - 1) as f32) * q).round() as usize];
        Some(Self {
            n,
            mean,
            sd,
            min: v[0],
            max: v[n - 1],
            median: pick(0.5),
            p10: pick(0.10),
            p90: pick(0.90),
        })
    }

    /// Coefficient of variation, `sd / |mean|`. `None` when the mean is ~0, where
    /// the ratio is meaningless rather than large.
    pub fn cv(&self) -> Option<f32> {
        (self.mean.abs() > 1e-6).then(|| self.sd / self.mean.abs())
    }
}

/// A run of repeated friction measurements.
#[derive(Debug, Clone)]
pub struct FrictionSeries {
    pub samples: Vec<FrictionSample>,
    pub kind: FrictionKind,
    pub abort: Option<AbortReason>,
}

impl FrictionSeries {
    fn frictions(&self) -> Vec<f32> {
        self.samples.iter().filter_map(|s| s.friction_nm).collect()
    }

    /// Distribution of the friction estimates.
    pub fn friction_stats(&self) -> Option<Stats> {
        Stats::of(&self.frictions())
    }

    /// Distribution of the static-load estimates.
    pub fn static_load_stats(&self) -> Option<Stats> {
        let v: Vec<f32> = self
            .samples
            .iter()
            .filter_map(|s| s.static_load_nm)
            .collect();
        Stats::of(&v)
    }

    /// Repetitions that produced no friction estimate.
    pub fn failed_repetitions(&self) -> usize {
        self.samples.iter().filter(|s| s.friction_nm.is_none()).count()
    }

    /// Histogram of the friction estimates as `(lo, hi, count)` per bin.
    ///
    /// Bins span min..max, so a series with no spread yields one bin holding
    /// everything.
    pub fn histogram(&self, bins: usize) -> Vec<(f32, f32, usize)> {
        let v = self.frictions();
        let Some(st) = Stats::of(&v) else {
            return Vec::new();
        };
        let n = bins.max(1);
        if st.max <= st.min {
            return vec![(st.min, st.max, v.len())];
        }
        let w = (st.max - st.min) / n as f32;
        let mut counts = vec![0usize; n];
        for x in v.iter().filter(|x| x.is_finite()) {
            let idx = (((x - st.min) / w) as usize).min(n - 1);
            counts[idx] += 1;
        }
        (0..n)
            .map(|i| (st.min + w * i as f32, st.min + w * (i + 1) as f32, counts[i]))
            .collect()
    }

    /// Render the histogram as text, `width` characters at the fullest bin.
    pub fn histogram_text(&self, bins: usize, width: usize) -> String {
        let h = self.histogram(bins);
        let peak = h.iter().map(|&(_, _, c)| c).max().unwrap_or(0).max(1);
        h.iter()
            .map(|&(lo, hi, c)| {
                let cells = c * width.max(1) / peak;
                format!(
                    "  {lo:>8.4} .. {hi:>8.4} |{}{} {c}\n",
                    "#".repeat(cells),
                    " ".repeat(width.max(1) - cells)
                )
            })
            .collect()
    }

    /// Least-squares temperature slope across the series (°C per repetition).
    ///
    /// Non-zero means the motor was warming as it was measured, so the samples
    /// are not draws from one stationary process.
    pub fn temperature_trend(&self) -> Option<f32> {
        let pts: Vec<(f32, f32)> = self
            .samples
            .iter()
            .filter(|s| s.temperature_c.is_finite())
            .map(|s| (s.index as f32, s.temperature_c))
            .collect();
        slope(&pts)
    }

    /// Least-squares slope of friction against temperature (N·m per °C).
    ///
    /// The point of recording temperature: if this is appreciable, the spread in
    /// [`Self::friction_stats`] is partly drift and not just noise.
    pub fn friction_vs_temperature(&self) -> Option<f32> {
        let pts: Vec<(f32, f32)> = self
            .samples
            .iter()
            .filter_map(|s| Some((s.temperature_c, s.friction_nm?)))
            .filter(|(t, f)| t.is_finite() && f.is_finite())
            .collect();
        slope(&pts)
    }

    /// Least-squares slope of friction against repetition index (N·m per rep) —
    /// drift that shows up even when temperature is not reported.
    pub fn friction_trend(&self) -> Option<f32> {
        let pts: Vec<(f32, f32)> = self
            .samples
            .iter()
            .filter_map(|s| Some((s.index as f32, s.friction_nm?)))
            .collect();
        slope(&pts)
    }

    /// Write one row per repetition, for plotting elsewhere.
    pub fn write_csv(&self, w: &mut dyn Write) -> io::Result<()> {
        writeln!(
            w,
            "index,t_s,kind,friction_nm,static_load_nm,temperature_c,position_rad"
        )?;
        for s in &self.samples {
            let f = s.friction_nm.map(|v| v.to_string()).unwrap_or_default();
            let l = s.static_load_nm.map(|v| v.to_string()).unwrap_or_default();
            writeln!(
                w,
                "{},{:.3},{},{},{},{:.3},{:.6}",
                s.index,
                s.t_s,
                self.kind.label(),
                f,
                l,
                s.temperature_c,
                s.position_rad
            )?;
        }
        Ok(())
    }
}

/// Repeat a two-direction breakaway and collect the distribution.
///
/// Each repetition ramps both ways from rest, so it yields one stiction estimate
/// (half the spread) and one static-load estimate (the mean). A repetition where
/// either direction failed to break away contributes no friction estimate and is
/// counted by [`FrictionSeries::failed_repetitions`] rather than dropped
/// silently.
pub fn run_breakaway_series(
    act: &mut dyn Actuator,
    spec: &BreakawaySpec,
    limits: SafetyLimits,
    abort: &AtomicBool,
    reps: u32,
) -> Result<FrictionSeries> {
    let t0 = Instant::now();
    let mut samples = Vec::with_capacity(reps as usize);
    let mut stop = None;

    for index in 0..reps.max(1) {
        if abort.load(Ordering::Relaxed) {
            stop = Some(AbortReason::Cancelled);
            break;
        }
        let fwd = run_breakaway(
            act,
            &BreakawaySpec {
                direction: Direction::Positive,
                ..*spec
            },
            limits,
            abort,
        )?;
        let rev = run_breakaway(
            act,
            &BreakawaySpec {
                direction: Direction::Negative,
                ..*spec
            },
            limits,
            abort,
        )?;

        let (friction_nm, static_load_nm) = match (fwd.breakaway_torque_nm, rev.breakaway_torque_nm)
        {
            // Only combine when both ramps started from rest; otherwise the pair
            // is arithmetic on an artefact.
            (Some(f), Some(r)) if fwd.rested && rev.rested => {
                (Some((f - r).abs() / 2.0), Some((f + r) / 2.0))
            }
            _ => (None, None),
        };
        let last = rev.points.last().or_else(|| fwd.points.last());
        samples.push(FrictionSample {
            index,
            t_s: t0.elapsed().as_secs_f32(),
            friction_nm,
            static_load_nm,
            temperature_c: last.map_or(f32::NAN, |p| p.temperature_c),
            position_rad: last.map_or(f32::NAN, |p| p.position_rad),
        });
        if let Some(r) = fwd.abort.or(rev.abort) {
            stop = Some(r);
            break;
        }
    }

    Ok(FrictionSeries {
        samples,
        kind: FrictionKind::Static,
        abort: stop,
    })
}

/// Repeat a constant-speed sweep and collect the distribution of kinetic
/// friction.
pub fn run_velocity_sweep_series(
    act: &mut dyn Actuator,
    spec: &VelocitySweepSpec,
    limits: SafetyLimits,
    abort: &AtomicBool,
    reps: u32,
    bins: usize,
) -> Result<FrictionSeries> {
    let t0 = Instant::now();
    let mut samples = Vec::with_capacity(reps as usize);
    let mut stop = None;

    for index in 0..reps.max(1) {
        if abort.load(Ordering::Relaxed) {
            stop = Some(AbortReason::Cancelled);
            break;
        }
        let sweep = run_velocity_sweep(act, spec, limits, abort)?;
        let last = sweep.points.last();
        samples.push(FrictionSample {
            index,
            t_s: t0.elapsed().as_secs_f32(),
            friction_nm: sweep.mean_kinetic_friction_nm(bins),
            static_load_nm: sweep.peak_static_load_nm(bins),
            temperature_c: last.map_or(f32::NAN, |p| p.temperature_c),
            position_rad: last.map_or(f32::NAN, |p| p.position_rad),
        });
        if let Some(r) = sweep.abort {
            stop = Some(r);
            break;
        }
    }

    Ok(FrictionSeries {
        samples,
        kind: FrictionKind::Kinetic,
        abort: stop,
    })
}

/// Least-squares slope of `y` on `x`; `None` without two points or any spread.
fn slope(pts: &[(f32, f32)]) -> Option<f32> {
    if pts.len() < 2 {
        return None;
    }
    let n = pts.len() as f32;
    let mx = pts.iter().map(|p| p.0).sum::<f32>() / n;
    let my = pts.iter().map(|p| p.1).sum::<f32>() / n;
    let (mut num, mut den) = (0.0f32, 0.0f32);
    for &(x, y) in pts {
        num += (x - mx) * (y - my);
        den += (x - mx) * (x - mx);
    }
    (den > 0.0).then_some(num / den)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn series(kind: FrictionKind, vals: &[(f32, f32)]) -> FrictionSeries {
        FrictionSeries {
            samples: vals
                .iter()
                .enumerate()
                .map(|(i, &(f, t))| FrictionSample {
                    index: i as u32,
                    t_s: i as f32,
                    friction_nm: Some(f),
                    static_load_nm: Some(0.0),
                    temperature_c: t,
                    position_rad: 0.0,
                })
                .collect(),
            kind,
            abort: None,
        }
    }

    #[test]
    fn stats_summarise_a_known_set() {
        let st = Stats::of(&[1.0, 2.0, 3.0, 4.0, 5.0]).unwrap();
        assert_eq!(st.n, 5);
        assert!((st.mean - 3.0).abs() < 1e-6);
        assert!((st.median - 3.0).abs() < 1e-6);
        assert_eq!((st.min, st.max), (1.0, 5.0));
        // Sample sd of 1..5 is sqrt(2.5).
        assert!((st.sd - 2.5f32.sqrt()).abs() < 1e-5, "sd {}", st.sd);
        assert!((st.cv().unwrap() - 2.5f32.sqrt() / 3.0).abs() < 1e-5);
    }

    #[test]
    fn stats_ignore_non_finite_and_handle_a_single_sample() {
        let st = Stats::of(&[f32::NAN, 2.0, f32::INFINITY]).unwrap();
        assert_eq!(st.n, 1);
        assert_eq!(st.sd, 0.0);
        assert!(Stats::of(&[f32::NAN]).is_none());
        assert!(Stats::of(&[]).is_none());
    }

    /// A mean near zero makes the coefficient of variation meaningless, so it is
    /// withheld rather than reported as a huge number.
    #[test]
    fn cv_is_withheld_when_the_mean_is_about_zero() {
        let st = Stats::of(&[-1.0, 1.0]).unwrap();
        assert!(st.mean.abs() < 1e-6);
        assert_eq!(st.cv(), None);
    }

    #[test]
    fn histogram_bins_and_counts_every_sample() {
        let s = series(
            FrictionKind::Static,
            &[(0.1, 30.0), (0.2, 30.0), (0.2, 30.0), (0.3, 30.0)],
        );
        let h = s.histogram(4);
        assert_eq!(h.len(), 4);
        assert_eq!(h.iter().map(|&(_, _, c)| c).sum::<usize>(), 4);
        // The 0.2 pair must land together.
        assert!(h.iter().any(|&(_, _, c)| c == 2));
    }

    /// A series with no spread must not divide by a zero bin width.
    #[test]
    fn histogram_survives_a_series_with_no_spread() {
        let s = series(FrictionKind::Static, &[(0.5, 30.0), (0.5, 30.0)]);
        let h = s.histogram(8);
        assert_eq!(h.len(), 1);
        assert_eq!(h[0].2, 2);
        assert!(!s.histogram_text(8, 20).is_empty());
    }

    /// The reason temperature is recorded: a warming series is drift, not noise,
    /// and both trends must show it.
    #[test]
    fn a_warming_series_reports_its_drift() {
        // Friction falling 0.01 N·m per rep while temperature climbs 1 °C per rep.
        let vals: Vec<(f32, f32)> = (0..10)
            .map(|i| (0.5 - 0.01 * i as f32, 30.0 + i as f32))
            .collect();
        let s = series(FrictionKind::Kinetic, &vals);
        assert!((s.temperature_trend().unwrap() - 1.0).abs() < 1e-4);
        assert!((s.friction_trend().unwrap() + 0.01).abs() < 1e-4);
        // -0.01 N·m per rep over +1 °C per rep => -0.01 N·m/°C.
        assert!((s.friction_vs_temperature().unwrap() + 0.01).abs() < 1e-4);
    }

    /// A stationary series must report no drift, so drift and noise are
    /// distinguishable.
    #[test]
    fn a_stationary_series_reports_no_drift() {
        let vals = [(0.5, 30.0), (0.52, 30.0), (0.48, 30.0), (0.51, 30.0)];
        let s = series(FrictionKind::Static, &vals);
        // Temperature against index: flat, so the slope is 0.
        assert!(s.temperature_trend().unwrap().abs() < 1e-6);
        assert!(s.friction_trend().unwrap().abs() < 0.02);
        // Friction against *temperature* has no x-spread to regress on, so it is
        // withheld rather than reported as 0 — with the motor at one temperature
        // the data says nothing about the dependence.
        assert_eq!(s.friction_vs_temperature(), None);
    }

    #[test]
    fn failed_repetitions_are_counted_not_silently_dropped() {
        let mut s = series(FrictionKind::Static, &[(0.5, 30.0), (0.6, 30.0)]);
        s.samples[0].friction_nm = None;
        assert_eq!(s.failed_repetitions(), 1);
        assert_eq!(s.friction_stats().unwrap().n, 1);
    }

    #[test]
    fn csv_has_a_header_and_one_row_per_repetition() {
        let s = series(FrictionKind::Kinetic, &[(0.5, 30.0), (0.6, 31.0)]);
        let mut out = Vec::new();
        s.write_csv(&mut out).unwrap();
        let text = String::from_utf8(out).unwrap();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 3);
        assert!(lines[0].starts_with("index,t_s,kind,friction_nm"));
        assert!(lines[1].contains("kinetic"));
    }
}
