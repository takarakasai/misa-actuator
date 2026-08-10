//! The stop, heartbeat and rate state every session shares.
//!
//! One copy, on purpose. This used to exist twice — once for the single-motor
//! [`Session`](crate::Session) and once for [`MultiSession`](crate::MultiSession) —
//! and the two drifted in exactly the way duplicated safety code does:
//!
//! - the GUI's STOP button, Esc, and the error boundary all called a stop that
//!   only looked at the single-motor session, so pressing it while the Multi tab
//!   was driving two RS-04s did nothing at all (2026-08-05, found on hardware)
//! - the single-motor worker seeded its position setpoint from the shaft, with a
//!   comment naming the hazard; the multi worker did not, so its first streaming
//!   pass would have sent every motor to zero
//!
//! Neither was a hard problem. Both were the second implementation not knowing
//! what the first one knew. So the three stop paths, the watchdog, the heartbeat
//! and the rate clamp live here and nowhere else, and a new kind of session gets
//! them by construction rather than by remembering.
//!
//! What is *not* here is anything about setpoints or motors: those genuinely
//! differ between one motor and several, and pretending otherwise is how a
//! shared type turns into a union of two unrelated ones.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::protocol::{StopReason, MAX_RATE_HZ, MIN_RATE_HZ};

/// Codes for the stop-reason atomic. Kept as codes rather than a
/// [`StopReason`] so the reason can be published before the flag it explains.
pub(crate) const STOP_NONE: u8 = 0;
pub(crate) const STOP_USER: u8 = 1;
pub(crate) const STOP_WATCHDOG: u8 = 2;

/// How often the watchdog thread re-checks. Fine enough that the effective
/// timeout is the configured one plus a tick, coarse enough to be free.
pub(crate) const WATCHDOG_POLL: Duration = Duration::from_millis(50);

/// How many consecutive bus failures before a worker stops commanding a motor.
///
/// A single timeout is routine; twenty in a row means the motor is not listening
/// and continuing to command it is worse than admitting it.
///
/// **What happens at the limit differs by session, on purpose, and is not here.**
/// The single-motor worker stops driving but keeps the session alive — ending it
/// made a recoverable mistake unrecoverable, because a wrong run mode is rejected
/// on every tick and twenty rejections arrive in a tenth of a second. The
/// multi-motor worker leaves *that* motor alone and keeps servicing the rest,
/// because one dead motor on a four-motor wire must not take the other three
/// down. Only the threshold is shared, because a threshold that differs between
/// the two would be an accident rather than a decision.
pub(crate) const CONSECUTIVE_FAULT_LIMIT: u32 = 20;

/// Polling cadence for a motor that is not being driven.
///
/// The UI still wants live numbers, but an idle console has no business
/// saturating the bus at the control rate.
pub(crate) const IDLE_POLL: Duration = Duration::from_millis(50);

/// Shared run/stop state, touched by the session handle, the watchdog and the
/// worker.
///
/// Deliberately all atomics: the UI thread touches this every frame, and a
/// contended lock there would show up as jank.
pub(crate) struct Safety {
    started: Instant,
    /// Whether setpoints are being pushed to the motor(s).
    streaming: AtomicBool,
    /// Whether a measurement job owns the worker's loop.
    ///
    /// Kept here rather than in the single-motor worker even though only it has
    /// jobs today: it is an input to [`Self::watchdog_tripped`], and the point of
    /// this type is that the watchdog has one definition.
    job_active: AtomicBool,
    /// Session-relative ms at which the running job began.
    ///
    /// Progress for a quasi-static run has to be read from outside the worker,
    /// because the worker is blocked inside a `misa_sysid` call for the whole
    /// run and cannot send an event until it returns. So it lands here, where
    /// the session handle can read it on a poll.
    job_started_ms: AtomicU64,
    /// What the job expects to take, in ms. 0 means no estimate.
    job_expected_ms: AtomicU64,
    /// Whether [`Self::job_expected_ms`] is a ceiling rather than a schedule.
    job_expected_is_bound: AtomicBool,
    /// Bus transactions the running job has completed.
    ///
    /// The point of counting them is that elapsed time alone cannot tell a run
    /// in progress from a bus that has gone silent — and a motor going quiet
    /// mid-operation is a thing that has actually happened here more than once.
    /// A bar that advances on a dead bus is worse than no bar.
    job_transactions: AtomicU32,
    stop: AtomicBool,
    stop_reason: AtomicU8,
    rate_millihz: AtomicU32,
    last_heartbeat_ms: AtomicU64,
    /// 0 disables the watchdog.
    watchdog_ms: AtomicU64,
    /// Cleared on shutdown so the watchdog thread knows to exit.
    alive: AtomicBool,
}

impl Safety {
    pub(crate) fn new(rate_hz: f32, watchdog: Option<Duration>) -> Self {
        let this = Self {
            started: Instant::now(),
            streaming: AtomicBool::new(false),
            job_active: AtomicBool::new(false),
            job_started_ms: AtomicU64::new(0),
            job_expected_ms: AtomicU64::new(0),
            job_expected_is_bound: AtomicBool::new(false),
            job_transactions: AtomicU32::new(0),
            stop: AtomicBool::new(false),
            stop_reason: AtomicU8::new(STOP_NONE),
            rate_millihz: AtomicU32::new(rate_to_millihz(rate_hz)),
            last_heartbeat_ms: AtomicU64::new(0),
            watchdog_ms: AtomicU64::new(watchdog.map_or(0, |d| d.as_millis() as u64)),
            alive: AtomicBool::new(true),
        };
        // Beat once at construction: a session that is opened and immediately
        // streamed must not trip on a heartbeat it never had a chance to send.
        this.beat();
        this
    }

    pub(crate) fn elapsed(&self) -> Duration {
        self.started.elapsed()
    }

    /// When the session opened, for timestamping against one axis.
    pub(crate) fn started(&self) -> Instant {
        self.started
    }

    /// The raw flag `misa-sysid` polls as its abort.
    ///
    /// Handed out as a reference rather than copied into a second flag on
    /// purpose: a measurement job must abort on **the** stop, not on a mirror of
    /// it that something forgot to set. This is the same bool the STOP button
    /// writes and the watchdog writes.
    pub(crate) fn abort_flag(&self) -> &AtomicBool {
        &self.stop
    }

    fn now_ms(&self) -> u64 {
        self.started.elapsed().as_millis() as u64
    }

    pub(crate) fn beat(&self) {
        self.last_heartbeat_ms
            .store(self.now_ms(), Ordering::Release);
    }

    /// Ask the worker to stop.
    ///
    /// The reason is stored *first*, so whoever consumes the flag never sees a
    /// stop without knowing why.
    pub(crate) fn request_stop(&self, reason: u8) {
        self.stop_reason.store(reason, Ordering::Release);
        self.stop.store(true, Ordering::Release);
    }

    /// Consume a pending stop, if there is one.
    pub(crate) fn take_stop(&self) -> Option<StopReason> {
        if !self.stop.swap(false, Ordering::AcqRel) {
            return None;
        }
        Some(match self.stop_reason.swap(STOP_NONE, Ordering::AcqRel) {
            STOP_WATCHDOG => StopReason::Watchdog,
            _ => StopReason::UserRequest,
        })
    }

    /// Whether a stop is waiting to be taken. Read by the watchdog so it does
    /// not pile a second stop onto one already in flight, and by `misa-sysid` as
    /// its abort.
    pub(crate) fn stop_pending(&self) -> bool {
        self.stop.load(Ordering::Acquire)
    }

    pub(crate) fn is_streaming(&self) -> bool {
        self.streaming.load(Ordering::Acquire)
    }

    pub(crate) fn set_streaming(&self, on: bool) {
        self.streaming.store(on, Ordering::Release);
    }

    /// Turn streaming off and report whether it had been on.
    ///
    /// One atomic swap rather than a load and a store, so two threads cannot both
    /// conclude they were the one that stopped it and both announce it.
    pub(crate) fn take_streaming(&self) -> bool {
        self.streaming.swap(false, Ordering::AcqRel)
    }

    pub(crate) fn set_job_active(&self, on: bool) {
        self.job_active.store(on, Ordering::Release);
    }

    pub(crate) fn job_active(&self) -> bool {
        self.job_active.load(Ordering::Acquire)
    }

    /// Mark a job as starting now, resetting its progress.
    ///
    /// `expected` of `None` (or a zero duration) means "no estimate", which a
    /// reader must render as elapsed time rather than as a fraction.
    pub(crate) fn begin_job(&self, expected: Option<(Duration, bool)>) {
        self.job_transactions.store(0, Ordering::Release);
        let (ms, bound) = match expected {
            Some((d, bound)) => (d.as_millis() as u64, bound),
            None => (0, false),
        };
        self.job_expected_ms.store(ms, Ordering::Release);
        self.job_expected_is_bound.store(bound, Ordering::Release);
        // Written last, so a reader that sees a start time also sees the
        // estimate and a zeroed count that belong to the same job.
        self.job_started_ms
            .store(self.elapsed().as_millis() as u64, Ordering::Release);
        self.set_job_active(true);
    }

    /// Count one completed bus transaction against the running job.
    pub(crate) fn job_transaction(&self) {
        self.job_transactions.fetch_add(1, Ordering::Relaxed);
    }

    /// How far the running job has got: `(elapsed, expected, is_bound, transactions)`.
    ///
    /// `None` when no job is running. `expected` is zero when the run gave no
    /// estimate.
    pub(crate) fn job_progress(&self) -> Option<(Duration, Duration, bool, u32)> {
        if !self.job_active() {
            return None;
        }
        let started = self.job_started_ms.load(Ordering::Acquire);
        let elapsed = self
            .elapsed()
            .as_millis()
            .saturating_sub(started as u128) as u64;
        Some((
            Duration::from_millis(elapsed),
            Duration::from_millis(self.job_expected_ms.load(Ordering::Acquire)),
            self.job_expected_is_bound.load(Ordering::Acquire),
            self.job_transactions.load(Ordering::Relaxed),
        ))
    }

    /// Whether anything is actually being driven, and so whether the watchdog
    /// has something to protect against. Scolding an idle session would be noise.
    pub(crate) fn is_driving(&self) -> bool {
        self.is_streaming() || self.job_active()
    }

    /// `true` if the UI has gone quiet while something is being driven.
    pub(crate) fn watchdog_tripped(&self) -> bool {
        if !self.is_driving() {
            return false;
        }
        let limit = self.watchdog_ms.load(Ordering::Relaxed);
        if limit == 0 {
            return false;
        }
        let last = self.last_heartbeat_ms.load(Ordering::Acquire);
        self.now_ms().saturating_sub(last) > limit
    }

    pub(crate) fn set_watchdog(&self, watchdog: Option<Duration>) {
        self.watchdog_ms
            .store(watchdog.map_or(0, |d| d.as_millis() as u64), Ordering::Relaxed);
    }

    pub(crate) fn set_rate(&self, hz: f32) {
        self.rate_millihz
            .store(rate_to_millihz(hz), Ordering::Relaxed);
    }

    pub(crate) fn period(&self) -> Duration {
        let hz = self.rate_millihz.load(Ordering::Relaxed) as f32 / 1000.0;
        Duration::from_secs_f32(1.0 / hz.clamp(MIN_RATE_HZ, MAX_RATE_HZ))
    }

    pub(crate) fn alive(&self) -> bool {
        self.alive.load(Ordering::Acquire)
    }

    /// Tell the watchdog thread to exit. Called during shutdown, after the
    /// worker has been joined — never before, or a job on its way out would lose
    /// its protection partway.
    pub(crate) fn retire(&self) {
        self.alive.store(false, Ordering::Release);
    }
}

/// The watchdog thread.
///
/// On its own thread rather than inside the worker's loop, and that is not
/// tidiness — it is the only arrangement that works. A chirp owns the worker's
/// loop for tens of seconds, during which a loop-based check would never run,
/// and a hung UI would leave the motor being excited with nobody watching. A
/// separate thread keeps checking regardless of what the worker is busy with,
/// and the flag it sets is the same one `misa-sysid` reads as its abort.
///
/// `what` names what is being stopped, for the log line only.
pub(crate) fn watchdog_loop(safety: Arc<Safety>, what: &'static str) {
    while safety.alive() {
        if safety.watchdog_tripped() && !safety.stop_pending() {
            log::warn!("watchdog: no heartbeat from the UI — stopping {what}");
            safety.request_stop(STOP_WATCHDOG);
        }
        std::thread::sleep(WATCHDOG_POLL);
    }
}

pub(crate) fn rate_to_millihz(hz: f32) -> u32 {
    (hz.clamp(MIN_RATE_HZ, MAX_RATE_HZ) * 1000.0) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stop_carries_its_reason_and_is_taken_once() {
        let s = Safety::new(200.0, None);
        assert!(s.take_stop().is_none(), "nothing to take yet");

        s.request_stop(STOP_USER);
        assert!(s.stop_pending());
        assert_eq!(s.take_stop(), Some(StopReason::UserRequest));
        // Taken means taken: a second consumer must not stop again on the same
        // request, or one press would stop the next thing the operator starts.
        assert!(s.take_stop().is_none());
        assert!(!s.stop_pending());

        s.request_stop(STOP_WATCHDOG);
        assert_eq!(s.take_stop(), Some(StopReason::Watchdog));
    }

    /// The watchdog only guards something that is being driven, and only when it
    /// is enabled. Both halves matter: barking at an idle session is noise, and
    /// `None` has to mean off for tests and headless use.
    #[test]
    fn the_watchdog_guards_only_what_is_being_driven() {
        let s = Safety::new(200.0, Some(Duration::from_millis(1)));
        assert!(!s.is_driving());
        std::thread::sleep(Duration::from_millis(5));
        assert!(!s.watchdog_tripped(), "idle must not trip");

        s.set_streaming(true);
        assert!(s.watchdog_tripped(), "streaming with a stale heartbeat");
        s.beat();
        assert!(!s.watchdog_tripped(), "a beat clears it");

        // A job counts as driving even with streaming off — that is the case a
        // loop-based check would miss entirely.
        s.set_streaming(false);
        s.set_job_active(true);
        std::thread::sleep(Duration::from_millis(5));
        assert!(s.watchdog_tripped(), "a job is being driven too");

        let off = Safety::new(200.0, None);
        off.set_streaming(true);
        std::thread::sleep(Duration::from_millis(5));
        assert!(!off.watchdog_tripped(), "None disables it");
    }

    /// A session opened and immediately streamed must not trip on a heartbeat it
    /// never had the chance to send.
    #[test]
    fn construction_beats_once() {
        let s = Safety::new(200.0, Some(Duration::from_millis(50)));
        s.set_streaming(true);
        assert!(!s.watchdog_tripped());
    }

    #[test]
    fn the_rate_is_clamped_rather_than_trusted() {
        let s = Safety::new(200.0, None);
        assert!((s.period().as_secs_f32() - 1.0 / 200.0).abs() < 1e-6);

        s.set_rate(0.0);
        assert!(
            (s.period().as_secs_f32() - 1.0 / MIN_RATE_HZ).abs() < 1e-6,
            "zero would be a divide by zero and an infinite period"
        );
        s.set_rate(f32::MAX);
        assert!((s.period().as_secs_f32() - 1.0 / MAX_RATE_HZ).abs() < 1e-6);
        // Negative rates come from a UI field that allows a minus sign.
        s.set_rate(-5.0);
        assert!(s.period() > Duration::ZERO);
    }

    #[test]
    fn retiring_stops_the_watchdog_thread() {
        let s = Arc::new(Safety::new(200.0, Some(Duration::from_millis(1))));
        let handle = std::thread::spawn({
            let s = s.clone();
            move || watchdog_loop(s, "the test")
        });
        s.retire();
        handle.join().expect("the watchdog thread should exit");
    }
}
