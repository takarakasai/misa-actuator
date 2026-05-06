//! Frequency-response (Bode) estimation from a chirp run.
//!
//! Uses a Welch-averaged **H1** estimate: the record is split into overlapping
//! Hann-windowed segments, the cross- and auto-spectra are averaged, and
//! `H1(f) = Pyu(f) / Puu(f)` with magnitude-squared coherence
//! `γ²(f) = |Pyu|² / (Puu·Pyy)`. Coherence near 1 marks well-identified
//! frequencies; it drops where the chirp put little energy or the response is
//! noisy / nonlinear.

use std::io::{self, Write};

use rustfft::{num_complex::Complex, FftPlanner};

use crate::runner::{ChirpLog, Excitation};

const TAU: f32 = core::f32::consts::TAU;

/// Welch H1 options.
#[derive(Debug, Clone, Copy)]
pub struct FrfOptions {
    /// Segment length (samples). Clamped to the record length and made even.
    pub nperseg: usize,
    /// Keep frequencies ≥ this (Hz).
    pub f_lo_hz: f32,
    /// Keep frequencies ≤ this (Hz); `0` → Nyquist.
    pub f_hi_hz: f32,
}

/// Estimated frequency response over a set of frequency bins.
#[derive(Debug, Clone, Default)]
pub struct FreqResponse {
    pub freqs_hz: Vec<f32>,
    pub magnitude_db: Vec<f32>,
    pub phase_deg: Vec<f32>,
    /// Magnitude-squared coherence in `[0, 1]`.
    pub coherence: Vec<f32>,
}

impl FreqResponse {
    /// Write the Bode data as CSV.
    pub fn write_csv(&self, w: &mut dyn Write) -> io::Result<()> {
        writeln!(w, "freq_hz,mag_db,phase_deg,coherence")?;
        for i in 0..self.freqs_hz.len() {
            writeln!(
                w,
                "{:.4},{:.4},{:.4},{:.4}",
                self.freqs_hz[i], self.magnitude_db[i], self.phase_deg[i], self.coherence[i]
            )?;
        }
        Ok(())
    }
}

fn mean(x: &[f32]) -> f32 {
    if x.is_empty() {
        0.0
    } else {
        x.iter().sum::<f32>() / x.len() as f32
    }
}

/// Welch-averaged H1 frequency response of `output` w.r.t. `input` (uniformly
/// sampled at `rate_hz`).
pub fn estimate_frf(input: &[f32], output: &[f32], rate_hz: f32, opts: &FrfOptions) -> FreqResponse {
    let n = input.len().min(output.len());
    if n < 8 {
        return FreqResponse::default();
    }
    let mut nperseg = opts.nperseg.clamp(8, n);
    if nperseg % 2 == 1 {
        nperseg -= 1;
    }
    let step = (nperseg / 2).max(1); // 50% overlap
    let hann: Vec<f32> = (0..nperseg)
        .map(|i| 0.5 - 0.5 * (TAU * i as f32 / (nperseg as f32 - 1.0)).cos())
        .collect();

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(nperseg);

    let nbins = nperseg / 2 + 1;
    let mut puu = vec![0f32; nbins];
    let mut pyy = vec![0f32; nbins];
    let mut pyu = vec![Complex::new(0f32, 0f32); nbins];

    let mut start = 0;
    while start + nperseg <= n {
        let in_seg = &input[start..start + nperseg];
        let out_seg = &output[start..start + nperseg];
        let in_mean = mean(in_seg);
        let out_mean = mean(out_seg);
        let mut ub: Vec<Complex<f32>> = (0..nperseg)
            .map(|i| Complex::new((in_seg[i] - in_mean) * hann[i], 0.0))
            .collect();
        let mut yb: Vec<Complex<f32>> = (0..nperseg)
            .map(|i| Complex::new((out_seg[i] - out_mean) * hann[i], 0.0))
            .collect();
        fft.process(&mut ub);
        fft.process(&mut yb);
        for k in 0..nbins {
            puu[k] += ub[k].norm_sqr();
            pyy[k] += yb[k].norm_sqr();
            pyu[k] += yb[k] * ub[k].conj();
        }
        start += step;
    }

    let f_hi = if opts.f_hi_hz <= 0.0 {
        rate_hz / 2.0
    } else {
        opts.f_hi_hz
    };
    let mut fr = FreqResponse::default();
    for k in 0..nbins {
        let f = k as f32 * rate_hz / nperseg as f32;
        if f < opts.f_lo_hz || f > f_hi {
            continue;
        }
        let h = pyu[k] / puu[k].max(1e-20);
        let coh = pyu[k].norm_sqr() / (puu[k] * pyy[k]).max(1e-20);
        fr.freqs_hz.push(f);
        fr.magnitude_db.push(20.0 * h.norm().max(1e-12).log10());
        fr.phase_deg.push(h.arg().to_degrees());
        fr.coherence.push(coh.min(1.0));
    }
    fr
}

/// Linearly resample `y(times)` onto `n_out` uniform points at `rate_hz`
/// (`times` must be monotonically increasing).
fn resample_uniform(times: &[f32], y: &[f32], n_out: usize, rate_hz: f32) -> Vec<f32> {
    let mut out = Vec::with_capacity(n_out);
    let mut j = 0usize;
    for k in 0..n_out {
        let tq = k as f32 / rate_hz;
        while j + 1 < times.len() && times[j + 1] < tq {
            j += 1;
        }
        if j + 1 >= times.len() {
            out.push(*y.last().unwrap_or(&0.0));
        } else {
            let (t0, t1) = (times[j], times[j + 1]);
            let frac = if t1 > t0 { (tq - t0) / (t1 - t0) } else { 0.0 };
            out.push(y[j] + frac * (y[j + 1] - y[j]));
        }
    }
    out
}

impl ChirpLog {
    /// Estimate the frequency response of this run.
    ///
    /// Input = the commanded signal; output = position for position-referenced
    /// channels, velocity for velocity / torque channels. Samples are resampled
    /// onto a uniform grid (real timestamps may jitter) before the FFT, and the
    /// result is limited to the chirp's `[f_start, f_end]` band.
    pub fn frf(&self) -> FreqResponse {
        let n = self.samples.len();
        if n < 16 {
            return FreqResponse::default();
        }
        let input: Vec<f32> = self.samples.iter().map(|s| s.cmd).collect();
        let output: Vec<f32> = match self.excitation {
            // Torque-input channels identify the plant ω/τ → output is velocity.
            Excitation::Velocity | Excitation::Torque | Excitation::MitTorque { .. } => {
                self.samples.iter().map(|s| s.velocity_rad_per_s).collect()
            }
            _ => self.samples.iter().map(|s| s.position_rad).collect(),
        };
        let times: Vec<f32> = self.samples.iter().map(|s| s.t_s).collect();
        let t_last = *times.last().unwrap();
        let rate = (n as f32 - 1.0) / t_last.max(1e-6);
        let ui = resample_uniform(&times, &input, n, rate);
        let yo = resample_uniform(&times, &output, n, rate);

        let nperseg = (n / 8).max(16).next_power_of_two();
        let opts = FrfOptions {
            nperseg,
            f_lo_hz: self.chirp.f_start_hz,
            f_hi_hz: self.chirp.f_end_hz,
        };
        estimate_frf(&ui, &yo, rate, &opts)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chirp::Chirp;

    /// Feed a chirp through a known 1st-order low-pass and check the estimated
    /// magnitude matches the analytic `H(e^{jω})` at well-excited frequencies.
    #[test]
    fn frf_matches_first_order_lowpass() {
        let fs = 1000.0f32;
        let n = 8192usize;
        let chirp = Chirp::linear(1.0, 200.0, n as f32 / fs, 1.0);
        let x: Vec<f32> = (0..n).map(|i| chirp.value(i as f32 / fs)).collect();

        // y[n] = a y[n-1] + (1-a) x[n]
        let a = 0.9f32;
        let mut y = vec![0f32; n];
        for i in 1..n {
            y[i] = a * y[i - 1] + (1.0 - a) * x[i];
        }

        let opts = FrfOptions {
            nperseg: 1024,
            f_lo_hz: 2.0,
            f_hi_hz: 200.0,
        };
        let fr = estimate_frf(&x, &y, fs, &opts);
        assert!(!fr.freqs_hz.is_empty());

        let mut checked = 0;
        for i in 0..fr.freqs_hz.len() {
            let f = fr.freqs_hz[i];
            if !(5.0..=150.0).contains(&f) || fr.coherence[i] < 0.9 {
                continue;
            }
            let w = TAU * f / fs;
            // H(e^{jw}) = (1-a) / (1 - a e^{-jw})
            let denom = Complex::new(1.0 - a * w.cos(), a * w.sin());
            let h = Complex::new(1.0 - a, 0.0) / denom;
            let mag_db = 20.0 * h.norm().log10();
            assert!(
                (fr.magnitude_db[i] - mag_db).abs() < 2.0,
                "f={f}: est={:.2} dB, exp={:.2} dB",
                fr.magnitude_db[i],
                mag_db
            );
            checked += 1;
        }
        assert!(checked > 5, "too few well-excited frequencies checked");
    }
}
