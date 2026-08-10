//! Time source for the simulator.
//!
//! The plant integrates against elapsed wall-clock time so that a GUI or the
//! TUI sees the shaft move while they idle. Tests need the opposite: exact,
//! instant, repeatable time. Both go through [`Clock`].

use std::sync::Mutex;
use std::time::{Duration, Instant};

/// Where the simulator gets "now" and how it waits.
pub trait Clock: Send + Sync {
    /// Seconds since this clock was created. Monotonic.
    fn now_s(&self) -> f64;

    /// Block for `d`. Virtual clocks may return immediately after advancing.
    fn sleep(&self, d: Duration);
}

/// Wall-clock time — what a real run uses.
///
/// Sleeps go through [`misa_actuator::realtime::sleep_precise`], so a
/// simulated bus latency of 1 ms really is ~1 ms even on Windows.
pub struct RealClock {
    start: Instant,
}

impl RealClock {
    pub fn new() -> Self {
        Self {
            start: Instant::now(),
        }
    }
}

impl Default for RealClock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for RealClock {
    fn now_s(&self) -> f64 {
        self.start.elapsed().as_secs_f64()
    }

    fn sleep(&self, d: Duration) {
        if !d.is_zero() {
            misa_actuator::realtime::sleep_precise(d);
        }
    }
}

/// Virtual time under the test's control.
///
/// [`Clock::sleep`] advances the clock instead of blocking, so a test that
/// simulates a 200-second thermal run finishes instantly and always produces
/// the same numbers.
pub struct ManualClock {
    now_s: Mutex<f64>,
}

impl ManualClock {
    pub fn new() -> Self {
        Self {
            now_s: Mutex::new(0.0),
        }
    }

    /// Move virtual time forward.
    pub fn advance(&self, d: Duration) {
        *self.now_s.lock().unwrap() += d.as_secs_f64();
    }

    /// Move virtual time forward by a number of seconds.
    pub fn advance_s(&self, seconds: f64) {
        *self.now_s.lock().unwrap() += seconds;
    }
}

impl Default for ManualClock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for ManualClock {
    fn now_s(&self) -> f64 {
        *self.now_s.lock().unwrap()
    }

    fn sleep(&self, d: Duration) {
        self.advance(d);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn manual_clock_only_moves_when_told() {
        let c = ManualClock::new();
        assert_eq!(c.now_s(), 0.0);
        c.sleep(Duration::from_millis(500));
        assert!((c.now_s() - 0.5).abs() < 1e-9);
        c.advance_s(1.5);
        assert!((c.now_s() - 2.0).abs() < 1e-9);
    }

    #[test]
    fn manual_clock_sleep_does_not_actually_block() {
        let c = ManualClock::new();
        let started = Instant::now();
        c.sleep(Duration::from_secs(60));
        assert!(started.elapsed() < Duration::from_millis(50));
        assert!((c.now_s() - 60.0).abs() < 1e-9);
    }

    #[test]
    fn real_clock_advances() {
        let c = RealClock::new();
        let t0 = c.now_s();
        c.sleep(Duration::from_millis(5));
        assert!(c.now_s() - t0 >= 0.004, "clock did not advance");
    }
}
