//! Loop-timing helpers for periodic control code.
//!
//! Everything here exists because of one platform difference: **Windows
//! defaults to a ~15.6 ms scheduler tick**. `thread::sleep(1ms)` therefore
//! sleeps ~15 ms, which turns a 1 kHz control loop into a ~64 Hz one and
//! silently ruins any chirp identification run. Linux `nanosleep` is already
//! accurate to tens of microseconds, so on Linux these are thin wrappers that
//! cost nothing.
//!
//! Two pieces, used together:
//!
//! 1. [`TimerResolutionGuard`] — hold one for the lifetime of `main` to ask
//!    Windows for a 1 ms tick.
//! 2. [`sleep_until`] / [`sleep_precise`] — sleep most of the way, then spin
//!    the last fraction of a millisecond, so the wake-up lands on time even
//!    when the OS rounds sleeps up.
//!
//! ```no_run
//! use std::time::{Duration, Instant};
//! use misa_actuator::realtime::{TimerResolutionGuard, sleep_until};
//!
//! let _timer = TimerResolutionGuard::acquire();
//! let period = Duration::from_millis(1);
//! let mut next = Instant::now();
//! for _ in 0..1000 {
//!     // ... one control tick ...
//!     next += period;
//!     sleep_until(next);
//! }
//! ```

use std::time::{Duration, Instant};

/// How much of a sleep to spend spinning rather than sleeping.
///
/// Windows rounds a sleep up to the current tick even at 1 ms resolution, so
/// the last ~1.5 ms has to be busy-waited. Linux only needs a small guard for
/// scheduler latency.
const SPIN_MARGIN: Duration = if cfg!(windows) {
    Duration::from_micros(1_500)
} else {
    Duration::from_micros(100)
};

/// Above this much remaining time, yield to the scheduler instead of burning
/// the core outright.
const YIELD_ABOVE: Duration = Duration::from_micros(200);

/// Raises the OS timer resolution for as long as it is held.
///
/// On Windows this is `timeBeginPeriod(1)` / `timeEndPeriod(1)`, resolved out
/// of `winmm.dll` at run time so nothing extra has to be linked. On every
/// other platform it is a no-op that reports itself inactive.
///
/// Acquire one in `main` and keep it alive for the whole run — the setting is
/// per-process and reference-counted by the OS, so dropping it restores the
/// previous tick.
#[derive(Debug)]
pub struct TimerResolutionGuard {
    period_ms: u32,
    active: bool,
}

impl TimerResolutionGuard {
    /// Request a 1 ms timer tick — the finest Windows offers through this
    /// API, and what periodic control loops need.
    pub fn acquire() -> Self {
        Self::acquire_ms(1)
    }

    /// Request a specific tick length in milliseconds.
    pub fn acquire_ms(period_ms: u32) -> Self {
        let active = platform::begin_period(period_ms);
        if cfg!(windows) && !active {
            log::warn!(
                "could not raise the Windows timer resolution to {period_ms} ms; \
                 loop rates above ~64 Hz will be inaccurate"
            );
        }
        Self { period_ms, active }
    }

    /// Whether the request actually changed anything. Always `false` off
    /// Windows, where no change is needed.
    pub fn is_active(&self) -> bool {
        self.active
    }
}

impl Drop for TimerResolutionGuard {
    fn drop(&mut self) {
        if self.active {
            platform::end_period(self.period_ms);
        }
    }
}

/// Sleep until `deadline`, accurately.
///
/// Returns immediately if the deadline has already passed — an overrunning
/// control loop keeps its phase rather than accumulating sleep debt.
pub fn sleep_until(deadline: Instant) {
    let now = Instant::now();
    if deadline <= now {
        return;
    }
    sleep_span(deadline - now, deadline);
}

/// Sleep for `duration`, accurately. Prefer [`sleep_until`] in a periodic
/// loop: it does not drift by the cost of the loop body.
pub fn sleep_precise(duration: Duration) {
    if duration.is_zero() {
        return;
    }
    sleep_span(duration, Instant::now() + duration);
}

fn sleep_span(duration: Duration, deadline: Instant) {
    // Hand the bulk to the OS; only the tail needs busy-waiting.
    if let Some(coarse) = duration.checked_sub(SPIN_MARGIN) {
        if !coarse.is_zero() {
            std::thread::sleep(coarse);
        }
    }
    loop {
        let now = Instant::now();
        if now >= deadline {
            return;
        }
        if deadline - now > YIELD_ABOVE {
            std::thread::yield_now();
        } else {
            std::hint::spin_loop();
        }
    }
}

#[cfg(windows)]
mod platform {
    use std::ffi::{c_char, c_void, CString};
    use std::sync::OnceLock;

    type FnTimePeriod = unsafe extern "system" fn(u32) -> u32;

    /// `timeBeginPeriod` / `timeEndPeriod`, resolved from `winmm.dll`.
    struct Winmm {
        begin: FnTimePeriod,
        end: FnTimePeriod,
    }

    // Plain function pointers into a module that is never unloaded.
    unsafe impl Send for Winmm {}
    unsafe impl Sync for Winmm {}

    #[link(name = "kernel32")]
    extern "system" {
        fn LoadLibraryA(name: *const c_char) -> *mut c_void;
        fn GetProcAddress(module: *mut c_void, name: *const c_char) -> *mut c_void;
    }

    static WINMM: OnceLock<Option<Winmm>> = OnceLock::new();

    fn winmm() -> Option<&'static Winmm> {
        WINMM
            .get_or_init(|| unsafe {
                let dll = CString::new("winmm.dll").unwrap();
                let module = LoadLibraryA(dll.as_ptr());
                if module.is_null() {
                    return None;
                }
                let begin_name = CString::new("timeBeginPeriod").unwrap();
                let end_name = CString::new("timeEndPeriod").unwrap();
                let begin = GetProcAddress(module, begin_name.as_ptr());
                let end = GetProcAddress(module, end_name.as_ptr());
                if begin.is_null() || end.is_null() {
                    return None;
                }
                Some(Winmm {
                    begin: std::mem::transmute::<*mut c_void, FnTimePeriod>(begin),
                    end: std::mem::transmute::<*mut c_void, FnTimePeriod>(end),
                })
            })
            .as_ref()
    }

    /// `TIMERR_NOERROR`
    const TIMERR_NOERROR: u32 = 0;

    pub(super) fn begin_period(period_ms: u32) -> bool {
        match winmm() {
            Some(w) => unsafe { (w.begin)(period_ms) == TIMERR_NOERROR },
            None => false,
        }
    }

    pub(super) fn end_period(period_ms: u32) {
        if let Some(w) = winmm() {
            unsafe {
                (w.end)(period_ms);
            }
        }
    }
}

#[cfg(not(windows))]
mod platform {
    /// Nothing to do: `nanosleep` already resolves far finer than any control
    /// loop here needs.
    pub(super) fn begin_period(_period_ms: u32) -> bool {
        false
    }

    pub(super) fn end_period(_period_ms: u32) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sleep_until_a_past_deadline_returns_at_once() {
        let past = Instant::now() - Duration::from_millis(50);
        let start = Instant::now();
        sleep_until(past);
        assert!(start.elapsed() < Duration::from_millis(5));
    }

    #[test]
    fn sleep_precise_lands_close_to_the_request() {
        let _timer = TimerResolutionGuard::acquire();
        let start = Instant::now();
        sleep_precise(Duration::from_millis(5));
        let slept = start.elapsed();
        assert!(slept >= Duration::from_millis(5), "undershot: {slept:?}");
        // A plain thread::sleep on a default-tick Windows box would land
        // around 15 ms here; the guard plus spin tail should stay well under.
        assert!(slept < Duration::from_millis(12), "overshot: {slept:?}");
    }

    #[test]
    fn zero_duration_is_free() {
        let start = Instant::now();
        sleep_precise(Duration::ZERO);
        assert!(start.elapsed() < Duration::from_millis(2));
    }
}
