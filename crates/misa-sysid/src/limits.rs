//! Safety envelope shared by the quasi-static characterization runs.
//!
//! Unlike a chirp — which is zero-mean and short — the measurements in
//! [`crate::quasistatic`] deliberately push against a stalled or loaded output
//! shaft and can hold torque for many seconds. That heats the motor: a bench
//! DM-J4310 holding 6.7 N·m rose from 30 °C to 35 °C in three seconds. So every
//! run is wrapped in a [`SafetyLimits`] envelope that aborts on the first
//! breach and still returns whatever was collected.

use misa_actuator::MotorFeedback;

/// Why a run stopped before completing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AbortReason {
    /// The caller's `AtomicBool` was set (Ctrl-C).
    Cancelled,
    /// Measured torque exceeded [`SafetyLimits::max_torque_nm`].
    TorqueLimit,
    /// Temperature exceeded [`SafetyLimits::max_temperature_c`].
    TemperatureLimit,
    /// Temperature was climbing faster than
    /// [`SafetyLimits::max_temperature_rise_c_per_s`].
    TemperatureRiseRate,
    /// Position left [`SafetyLimits::position_window_rad`] around the start.
    PositionWindow,
    /// [`SafetyLimits::max_duration_s`] elapsed.
    Timeout,
    /// The motor reported a fault.
    MotorFault,
}

impl AbortReason {
    /// A short human-readable explanation, for CLI output.
    pub const fn describe(self) -> &'static str {
        match self {
            AbortReason::Cancelled => "cancelled by the operator",
            AbortReason::TorqueLimit => "measured torque hit the limit",
            AbortReason::TemperatureLimit => "temperature hit the limit",
            AbortReason::TemperatureRiseRate => "temperature was rising too fast",
            AbortReason::PositionWindow => "position left the allowed window",
            AbortReason::Timeout => "run exceeded its time budget",
            AbortReason::MotorFault => "motor reported a fault",
        }
    }
}

/// Bounds a quasi-static run must stay inside.
///
/// There are deliberately no defaults for the torque and position bounds: what
/// is safe depends on the motor *and* on what its output shaft is attached to,
/// which this crate cannot know. [`SafetyLimits::gentle`] offers a
/// conservative starting point to scale from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SafetyLimits {
    /// Abort once |measured torque| exceeds this (N·m).
    pub max_torque_nm: f32,
    /// Abort once temperature exceeds this (°C). `NAN` disables the check,
    /// which is what happens on drivers that do not report temperature.
    pub max_temperature_c: f32,
    /// Abort once temperature climbs faster than this (°C/s), measured over
    /// [`Self::TEMP_RATE_WINDOW_S`]. Catches a thermal runaway long before the
    /// absolute limit would.
    pub max_temperature_rise_c_per_s: f32,
    /// Abort once position leaves `start ± this` (rad). Protects a mechanism
    /// with end stops.
    pub position_window_rad: f32,
    /// Abort after this long (s), whatever else is happening.
    pub max_duration_s: f32,
}

impl SafetyLimits {
    /// Window over which the temperature rise rate is estimated. Short enough
    /// to react, long enough that 1 °C sensor quantization does not trip it.
    pub const TEMP_RATE_WINDOW_S: f32 = 2.0;

    /// A conservative envelope to scale from: 1 N·m, 60 °C, 2 °C/s, ±0.5 rad,
    /// 30 s.
    ///
    /// Chosen to be survivable on the smallest motor in the workspace
    /// (DM-J3507, 0.8 N·m rated) rather than optimal for any of them. Raise
    /// deliberately once you know the rig.
    pub const fn gentle() -> Self {
        Self {
            max_torque_nm: 1.0,
            max_temperature_c: 60.0,
            max_temperature_rise_c_per_s: 2.0,
            position_window_rad: 0.5,
            max_duration_s: 30.0,
        }
    }

    /// `true` if `torque_nm` is within the torque bound.
    ///
    /// A non-finite reading counts as within bounds: there is nothing to judge,
    /// and treating an unreported torque as a breach would abort every run on a
    /// driver that does not measure it.
    pub fn torque_ok(&self, torque_nm: f32) -> bool {
        !torque_nm.is_finite() || torque_nm.abs() <= self.max_torque_nm
    }
}

/// Tracks a run against its [`SafetyLimits`].
///
/// Feed every sample through [`Guard::check`]; the first `Some(reason)` means
/// stop. The guard is deliberately tolerant of `NAN` readings — a driver that
/// does not report temperature must not be treated as breaching a temperature
/// limit — so it only ever trips on a value it actually observed.
#[derive(Debug, Clone)]
pub struct Guard {
    limits: SafetyLimits,
    start_position_rad: f32,
    /// `(t_s, temperature_c)` history, trimmed to the rate window.
    temp_history: Vec<(f32, f32)>,
}

impl Guard {
    /// Start guarding a run whose output shaft is at `start_position_rad`.
    pub fn new(limits: SafetyLimits, start_position_rad: f32) -> Self {
        Self {
            limits,
            start_position_rad,
            temp_history: Vec::new(),
        }
    }

    /// The envelope in force.
    pub fn limits(&self) -> SafetyLimits {
        self.limits
    }

    /// Check one sample taken `t_s` into the run. Returns the reason to stop,
    /// or `None` to continue.
    pub fn check(&mut self, t_s: f32, fb: &MotorFeedback) -> Option<AbortReason> {
        if t_s > self.limits.max_duration_s {
            return Some(AbortReason::Timeout);
        }
        if fb.torque_nm.is_finite() && !self.limits.torque_ok(fb.torque_nm) {
            return Some(AbortReason::TorqueLimit);
        }
        if fb.position_rad.is_finite()
            && (fb.position_rad - self.start_position_rad).abs() > self.limits.position_window_rad
        {
            return Some(AbortReason::PositionWindow);
        }
        self.check_temperature(t_s, fb.temperature_c)
    }

    fn check_temperature(&mut self, t_s: f32, temperature_c: f32) -> Option<AbortReason> {
        if !temperature_c.is_finite() {
            // Driver does not report temperature — nothing to judge.
            return None;
        }
        if self.limits.max_temperature_c.is_finite()
            && temperature_c > self.limits.max_temperature_c
        {
            return Some(AbortReason::TemperatureLimit);
        }

        self.temp_history.push((t_s, temperature_c));
        // Keep only what falls inside the rate window.
        let cutoff = t_s - SafetyLimits::TEMP_RATE_WINDOW_S;
        self.temp_history.retain(|&(t, _)| t >= cutoff);

        // Need the window actually spanned before a rate means anything;
        // otherwise a 1 °C sensor step over 10 ms reads as 100 °C/s.
        let (&(t0, c0), &(t1, c1)) = (self.temp_history.first()?, self.temp_history.last()?);
        let dt = t1 - t0;
        if dt < SafetyLimits::TEMP_RATE_WINDOW_S * 0.5 {
            return None;
        }
        if self.limits.max_temperature_rise_c_per_s.is_finite()
            && (c1 - c0) / dt > self.limits.max_temperature_rise_c_per_s
        {
            return Some(AbortReason::TemperatureRiseRate);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fb(pos: f32, torque: f32, temp: f32) -> MotorFeedback {
        MotorFeedback {
            position_rad: pos,
            velocity_rad_per_s: 0.0,
            torque_nm: torque,
            current_a: f32::NAN,
            temperature_c: temp,
        }
    }

    #[test]
    fn passes_a_sample_inside_every_bound() {
        let mut g = Guard::new(SafetyLimits::gentle(), 0.0);
        assert_eq!(g.check(0.0, &fb(0.1, 0.5, 30.0)), None);
    }

    #[test]
    fn trips_on_torque_temperature_position_and_time() {
        let l = SafetyLimits::gentle();
        assert_eq!(
            Guard::new(l, 0.0).check(0.0, &fb(0.0, 1.5, 30.0)),
            Some(AbortReason::TorqueLimit)
        );
        assert_eq!(
            Guard::new(l, 0.0).check(0.0, &fb(0.0, 0.0, 70.0)),
            Some(AbortReason::TemperatureLimit)
        );
        // Window is relative to the start position, not to zero.
        assert_eq!(
            Guard::new(l, 1.0).check(0.0, &fb(1.6, 0.0, 30.0)),
            Some(AbortReason::PositionWindow)
        );
        assert_eq!(Guard::new(l, 1.0).check(0.0, &fb(1.4, 0.0, 30.0)), None);
        assert_eq!(
            Guard::new(l, 0.0).check(31.0, &fb(0.0, 0.0, 30.0)),
            Some(AbortReason::Timeout)
        );
    }

    /// A driver that reports NaN temperature/torque must not be read as
    /// breaching a limit — DAMIAO reports NaN current, LK NaN nothing, and a
    /// mock plant may report NaN for both.
    #[test]
    fn nan_readings_never_trip_a_limit() {
        let mut g = Guard::new(SafetyLimits::gentle(), 0.0);
        assert_eq!(g.check(0.0, &fb(0.0, f32::NAN, f32::NAN)), None);
        assert_eq!(g.check(1.0, &fb(f32::NAN, 0.0, f32::NAN)), None);
    }

    /// The rise-rate check must span the window first, otherwise 1 °C of sensor
    /// quantization over one loop iteration reads as a runaway.
    #[test]
    fn rise_rate_ignores_a_short_window_then_trips_on_a_real_climb() {
        let mut g = Guard::new(SafetyLimits::gentle(), 0.0);
        // 1 °C in 10 ms = 100 °C/s, but the window is not spanned yet.
        assert_eq!(g.check(0.00, &fb(0.0, 0.0, 30.0)), None);
        assert_eq!(g.check(0.01, &fb(0.0, 0.0, 31.0)), None);

        // A genuine climb across the full window: 30 -> 40 °C over 2 s = 5 °C/s.
        let mut g = Guard::new(SafetyLimits::gentle(), 0.0);
        assert_eq!(g.check(0.0, &fb(0.0, 0.0, 30.0)), None);
        assert_eq!(
            g.check(2.0, &fb(0.0, 0.0, 40.0)),
            Some(AbortReason::TemperatureRiseRate)
        );
    }

    /// A slow climb inside the allowance must not trip: 1 °C/s under a 2 °C/s
    /// limit.
    #[test]
    fn rise_rate_allows_a_climb_within_the_allowance() {
        let mut g = Guard::new(SafetyLimits::gentle(), 0.0);
        assert_eq!(g.check(0.0, &fb(0.0, 0.0, 30.0)), None);
        assert_eq!(g.check(2.0, &fb(0.0, 0.0, 32.0)), None);
    }
}
