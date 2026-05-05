//! Chirp (swept-sine) excitation signal.
//!
//! A chirp sweeps a sinusoid from `f_start_hz` to `f_end_hz` over `duration_s`,
//! exciting the plant across a band of frequencies in a single run — the input
//! for a frequency-response identification.

use core::f32::consts::TAU;

/// Frequency-sweep law.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sweep {
    /// Frequency increases linearly with time.
    Linear,
    /// Frequency increases geometrically (constant fractional rate). Spends
    /// equal time per octave — better low-frequency resolution.
    Logarithmic,
}

/// A chirp excitation: `value(t) = bias + amplitude · sin(phase(t))`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Chirp {
    /// Start frequency (Hz) at `t = 0`.
    pub f_start_hz: f32,
    /// End frequency (Hz) at `t = duration_s`.
    pub f_end_hz: f32,
    /// Sweep duration (s).
    pub duration_s: f32,
    /// Sinusoid amplitude (excitation units — rad, rad/s, or N·m by channel).
    pub amplitude: f32,
    /// Constant offset added to the sinusoid (e.g. a center position).
    pub bias: f32,
    /// Sweep law.
    pub sweep: Sweep,
}

impl Chirp {
    /// A linear chirp with zero bias.
    pub fn linear(f_start_hz: f32, f_end_hz: f32, duration_s: f32, amplitude: f32) -> Self {
        Self {
            f_start_hz,
            f_end_hz,
            duration_s,
            amplitude,
            bias: 0.0,
            sweep: Sweep::Linear,
        }
    }

    /// Instantaneous phase (rad) at time `t` (clamped to `[0, duration_s]`).
    pub fn phase(&self, t: f32) -> f32 {
        let t = t.clamp(0.0, self.duration_s);
        let (f0, f1, dur) = (self.f_start_hz, self.f_end_hz, self.duration_s);
        match self.sweep {
            Sweep::Linear => {
                // f(t) = f0 + k t,  k = (f1 - f0)/T  →  φ = 2π(f0 t + k t²/2)
                let k = (f1 - f0) / dur;
                TAU * (f0 * t + 0.5 * k * t * t)
            }
            Sweep::Logarithmic => {
                // f(t) = f0 (f1/f0)^(t/T) → φ = 2π f0 T/ln(r) (r^(t/T) − 1), r=f1/f0
                let r = f1 / f0;
                let ln_r = r.ln();
                if ln_r.abs() < 1e-6 {
                    TAU * f0 * t
                } else {
                    TAU * f0 * dur / ln_r * (r.powf(t / dur) - 1.0)
                }
            }
        }
    }

    /// Instantaneous frequency (Hz) at time `t`.
    pub fn instantaneous_freq(&self, t: f32) -> f32 {
        let t = t.clamp(0.0, self.duration_s);
        let (f0, f1, dur) = (self.f_start_hz, self.f_end_hz, self.duration_s);
        match self.sweep {
            Sweep::Linear => f0 + (f1 - f0) * (t / dur),
            Sweep::Logarithmic => f0 * (f1 / f0).powf(t / dur),
        }
    }

    /// Excitation value at time `t`.
    pub fn value(&self, t: f32) -> f32 {
        self.bias + self.amplitude * self.phase(t).sin()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear_endpoints_match_band() {
        let c = Chirp::linear(1.0, 20.0, 5.0, 1.0);
        assert!((c.instantaneous_freq(0.0) - 1.0).abs() < 1e-4);
        assert!((c.instantaneous_freq(5.0) - 20.0).abs() < 1e-3);
        // midpoint of a linear sweep is the arithmetic mean
        assert!((c.instantaneous_freq(2.5) - 10.5).abs() < 1e-3);
    }

    #[test]
    fn log_endpoints_match_band() {
        let c = Chirp {
            f_start_hz: 1.0,
            f_end_hz: 100.0,
            duration_s: 4.0,
            amplitude: 1.0,
            bias: 0.0,
            sweep: Sweep::Logarithmic,
        };
        assert!((c.instantaneous_freq(0.0) - 1.0).abs() < 1e-4);
        assert!((c.instantaneous_freq(4.0) - 100.0).abs() < 1e-2);
        // halfway in time = geometric mean (10 Hz) for a log sweep
        assert!((c.instantaneous_freq(2.0) - 10.0).abs() < 1e-2);
    }

    #[test]
    fn phase_starts_at_zero_and_is_continuous() {
        let c = Chirp::linear(2.0, 30.0, 3.0, 0.5);
        assert_eq!(c.phase(0.0), 0.0);
        // value at t=0 is just the bias (sin 0 = 0)
        assert_eq!(c.value(0.0), c.bias);
        // monotonic phase (frequency positive)
        assert!(c.phase(1.0) < c.phase(2.0));
    }

    #[test]
    fn bias_and_amplitude_bound_the_value() {
        let c = Chirp {
            f_start_hz: 1.0,
            f_end_hz: 5.0,
            duration_s: 2.0,
            amplitude: 0.2,
            bias: 1.0,
            sweep: Sweep::Linear,
        };
        for i in 0..200 {
            let t = i as f32 * 0.01;
            let v = c.value(t);
            assert!((0.8..=1.2).contains(&v), "v={v} out of [bias±amp]");
        }
    }
}
