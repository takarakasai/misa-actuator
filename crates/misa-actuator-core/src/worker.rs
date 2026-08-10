//! The motor I/O thread.
//!
//! Everything that touches the bus happens here, on one thread that owns the
//! `Box<dyn Actuator>` outright. That ownership is the point:
//!
//! - a UI can never block on a bus transaction, however slow the wire is
//! - the control rate is the worker's business, not the frame rate's
//! - if the UI dies, the command channel closes, the worker notices and
//!   disables the motor on its way out — no supervision required

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, Ordering};
use std::sync::mpsc::{Receiver, Sender, SyncSender, TryRecvError, TrySendError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use misa_actuator::realtime::{sleep_until, TimerResolutionGuard};
use misa_actuator::{Actuator, Error as ActuatorError};

use crate::protocol::*;

/// How many consecutive bus failures before the worker gives up and stops the
/// motor. A single timeout is routine; twenty in a row means the motor is gone
/// and continuing to command it is worse than admitting it.
const CONSECUTIVE_FAULT_LIMIT: u32 = 20;

/// Polling cadence while not streaming. The UI still wants live numbers, but
/// an idle console has no business saturating the bus at the control rate.
const IDLE_POLL: Duration = Duration::from_millis(50);

/// Translate a job's channel selection into the sysid excitation, folding in
/// whichever gains that channel actually uses.
fn excitation_of(job: &ChirpJob) -> misa_sysid::Excitation {
    use misa_sysid::Excitation as E;
    match job.excitation {
        ExcitationKind::Position => E::Position {
            max_speed_rad_s: job.max_speed_rad_s,
        },
        ExcitationKind::Velocity => E::Velocity,
        ExcitationKind::Torque => E::Torque,
        ExcitationKind::MitPosition => E::MitPosition {
            kp: job.kp,
            kd: job.kd,
        },
        ExcitationKind::MitTorque => E::MitTorque {
            kp: job.kp,
            kd: job.kd,
        },
    }
}

/// Codes for [`Shared::stop_reason`].
pub(crate) const STOP_NONE: u8 = 0;
pub(crate) const STOP_USER: u8 = 1;
pub(crate) const STOP_WATCHDOG: u8 = 2;

/// State shared between the session handle, the watchdog and the worker.
///
/// Deliberately all atomics plus one small mutex: the UI thread touches this
/// on every frame, and a contended lock there would show up as jank.
pub(crate) struct Shared {
    pub(crate) started: Instant,
    pub(crate) setpoint: Mutex<Setpoint>,
    /// Whether anything has deliberately written a setpoint yet.
    ///
    /// Until it has, the worker is free to seed the position from the shaft so
    /// the UI shows where the motor actually is. Once a caller has chosen a
    /// target, seeding would be overwriting intent — and worse, racing it,
    /// since the setpoint is written immediately while commands are queued.
    pub(crate) setpoint_touched: AtomicBool,
    pub(crate) streaming: AtomicBool,
    /// A long-running measurement is in progress. The worker's loop is blocked
    /// for its duration, so this is how anything else knows the motor is being
    /// driven.
    pub(crate) job_active: AtomicBool,
    /// Set by the UI or the watchdog, consumed by the worker. Separate from
    /// the command channel so a stop can never queue behind other work — and
    /// handed straight to `misa-sysid` as its abort flag, so it cuts a chirp
    /// short too.
    pub(crate) stop: AtomicBool,
    pub(crate) stop_reason: AtomicU8,
    pub(crate) rate_millihz: AtomicU32,
    pub(crate) last_heartbeat_ms: AtomicU64,
    /// 0 disables the watchdog.
    pub(crate) watchdog_ms: AtomicU64,
    /// Cleared on shutdown so the watchdog thread knows to exit.
    pub(crate) alive: AtomicBool,
}

impl Shared {
    pub(crate) fn new(rate_hz: f32, watchdog: Option<Duration>) -> Self {
        Self {
            started: Instant::now(),
            setpoint: Mutex::new(Setpoint::default()),
            setpoint_touched: AtomicBool::new(false),
            streaming: AtomicBool::new(false),
            job_active: AtomicBool::new(false),
            stop: AtomicBool::new(false),
            stop_reason: AtomicU8::new(STOP_NONE),
            rate_millihz: AtomicU32::new(rate_to_millihz(rate_hz)),
            last_heartbeat_ms: AtomicU64::new(0),
            watchdog_ms: AtomicU64::new(
                watchdog.map(|d| d.as_millis() as u64).unwrap_or(0),
            ),
            alive: AtomicBool::new(true),
        }
    }

    pub(crate) fn now_ms(&self) -> u64 {
        self.started.elapsed().as_millis() as u64
    }

    pub(crate) fn beat(&self) {
        self.last_heartbeat_ms.store(self.now_ms(), Ordering::Release);
    }

    /// Ask the worker to stop. The reason is stored first so whoever consumes
    /// the flag never sees a stop without knowing why.
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

    /// Whether the motor is actually being driven, and so whether the watchdog
    /// has anything to protect against.
    pub(crate) fn is_driving(&self) -> bool {
        self.streaming.load(Ordering::Acquire) || self.job_active.load(Ordering::Acquire)
    }

    /// `true` if the UI has gone quiet while the motor is being driven.
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

    fn period(&self) -> Duration {
        let hz = self.rate_millihz.load(Ordering::Relaxed) as f32 / 1000.0;
        Duration::from_secs_f32(1.0 / hz.clamp(MIN_RATE_HZ, MAX_RATE_HZ))
    }
}

/// How often the watchdog thread re-checks. Fine enough that the effective
/// timeout is the configured one plus a tick, coarse enough to be free.
pub(crate) const WATCHDOG_POLL: Duration = Duration::from_millis(50);

/// The watchdog runs on its own thread rather than inside the worker's loop.
///
/// That is not tidiness — it is the only arrangement that works. A chirp owns
/// the worker's loop for tens of seconds, during which a loop-based check
/// would never run, and a hung UI would leave the motor being excited with
/// nobody watching. A separate thread keeps checking regardless of what the
/// worker is busy with, and the flag it sets is the same one `misa-sysid`
/// reads as its abort.
pub(crate) fn watchdog_loop(shared: Arc<Shared>) {
    while shared.alive.load(Ordering::Acquire) {
        if shared.watchdog_tripped() && !shared.stop.load(Ordering::Acquire) {
            log::warn!("watchdog: no heartbeat from the UI — stopping the motor");
            shared.request_stop(STOP_WATCHDOG);
        }
        std::thread::sleep(WATCHDOG_POLL);
    }
}

pub(crate) fn rate_to_millihz(hz: f32) -> u32 {
    (hz.clamp(MIN_RATE_HZ, MAX_RATE_HZ) * 1000.0) as u32
}

/// An in-progress bus scan, advanced one id per tick so the UI keeps painting
/// through it. A silent id costs the full per-id timeout, so a 32-id sweep at
/// 50 ms is well over a second of otherwise-frozen interface.
struct ScanState {
    from: u8,
    to: u8,
    next: u8,
    found: Vec<u8>,
    timeout: Duration,
    cancelled: bool,
}

impl ScanState {
    fn total(&self) -> u32 {
        self.to as u32 + 1 - self.from as u32
    }
    fn done(&self) -> u32 {
        (self.next as u32).saturating_sub(self.from as u32)
    }
    fn finished(&self) -> bool {
        self.cancelled || self.next > self.to
    }
}

pub(crate) struct Worker {
    actuator: Box<dyn Actuator + Send>,
    shared: Arc<Shared>,
    commands: Receiver<Command>,
    events: Sender<Event>,
    telemetry: SyncSender<TelemetryBatch>,

    enabled: bool,
    /// The run mode the driver is believed to be in. `None` means unknown, so
    /// the next streaming tick sets it explicitly.
    active_mode: Option<ControlMode>,
    scan: Option<ScanState>,

    pending: Vec<Sample>,
    dropped: u64,
    samples_since_flush: u32,
    last_flush: Instant,
    last_status: Instant,
    last_idle_poll: Instant,
    consecutive_faults: u32,
}

impl Worker {
    pub(crate) fn new(
        actuator: Box<dyn Actuator + Send>,
        shared: Arc<Shared>,
        commands: Receiver<Command>,
        events: Sender<Event>,
        telemetry: SyncSender<TelemetryBatch>,
    ) -> Self {
        let now = Instant::now();
        Self {
            actuator,
            shared,
            commands,
            events,
            telemetry,
            enabled: false,
            active_mode: None,
            scan: None,
            pending: Vec::with_capacity(256),
            dropped: 0,
            samples_since_flush: 0,
            last_flush: now,
            last_status: now,
            last_idle_poll: now,
            consecutive_faults: 0,
        }
    }

    pub(crate) fn run(mut self) {
        // Windows rounds sleeps up to ~15.6 ms without this, which would cap
        // the control loop near 64 Hz whatever rate was asked for.
        let _timer = TimerResolutionGuard::acquire();

        let mut next_tick = Instant::now();
        let reason = 'outer: loop {
            // Commands first, so a mode change takes effect this tick rather
            // than the next one.
            loop {
                match self.commands.try_recv() {
                    Ok(Command::Shutdown) => break 'outer StopReason::Shutdown,
                    Ok(cmd) => self.handle(cmd),
                    Err(TryRecvError::Empty) => break,
                    // The handle was dropped — the UI is gone. Stop the motor.
                    Err(TryRecvError::Disconnected) => break 'outer StopReason::Shutdown,
                }
            }

            // The watchdog thread sets the same flag, with its own reason.
            if let Some(reason) = self.shared.take_stop() {
                self.halt(reason);
            }

            if self.scan.is_some() {
                self.scan_step();
            } else if let Some(reason) = self.control_step() {
                break 'outer reason;
            }

            self.flush_if_due();
            self.status_if_due();

            next_tick += self.shared.period();
            let now = Instant::now();
            if next_tick < now {
                // Overran: re-base rather than accumulating debt and then
                // sprinting to catch up.
                next_tick = now;
            }
            sleep_until(next_tick);
        };

        self.finish(reason);
    }

    /// Always leaves the motor stopped, whatever ended the loop.
    fn finish(mut self, reason: StopReason) {
        self.flush(true);
        if let Err(e) = self.actuator.disable() {
            log::warn!("worker: disable on shutdown failed: {e}");
        }
        self.enabled = false;
        let _ = self.events.send(Event::Stopped { reason });
        let _ = self
            .events
            .send(Event::State(ConnectionState::Disconnected));
        log::info!("worker: stopped ({reason:?})");
    }

    fn handle(&mut self, cmd: Command) {
        match cmd {
            Command::Enable => match self.actuator.enable() {
                Ok(fb) => {
                    self.enabled = true;
                    self.consecutive_faults = 0;
                    self.seed_position(fb.position_rad);
                    self.push_sample(fb, self.commanded_now());
                    self.log(
                        LogLevel::Info,
                        format!("enabled at {:+.3} rad", fb.position_rad),
                    );
                }
                Err(e) => self.fault("enable", e),
            },
            Command::Disable => {
                self.shared.streaming.store(false, Ordering::Release);
                match self.actuator.disable() {
                    Ok(()) => {
                        self.enabled = false;
                        self.log(LogLevel::Info, "disabled");
                    }
                    Err(e) => self.fault("disable", e),
                }
            }
            Command::SetZero => match self.actuator.set_zero() {
                Ok(()) => self.log(LogLevel::Info, "position anchored"),
                Err(e) => self.fault("set_zero", e),
            },
            Command::SetRunMode { mode } => match self.actuator.set_run_mode(mode.into()) {
                Ok(()) => {
                    self.active_mode = Some(mode);
                    self.log(LogLevel::Info, format!("run mode -> {mode:?}"));
                }
                Err(e) => {
                    self.active_mode = None;
                    self.fault("set_run_mode", e);
                }
            },
            Command::Measure => match self.actuator.measure() {
                Ok(fb) => {
                    let s = self.sample(fb, self.commanded_now());
                    let _ = self.events.send(Event::Reading(s));
                }
                Err(e) => self.fault("measure", e),
            },
            Command::ReadStatus => self.read_status(),
            Command::ReadParameters { deep } => {
                // Dozens of round trips on the same wire the control loop
                // uses. Streaming has to stop for the same reason a scan
                // stops it, and saying so beats leaving the operator to
                // wonder why the motor went limp mid-read.
                if self.shared.streaming.swap(false, Ordering::AcqRel) {
                    self.log(
                        LogLevel::Info,
                        "streaming paused while reading parameters".to_string(),
                    );
                }
                self.log(
                    LogLevel::Info,
                    if deep {
                        "reading parameters, including undocumented spaces".to_string()
                    } else {
                        "reading documented parameters".to_string()
                    },
                );
                match self.actuator.read_parameters(deep) {
                    Ok(params) => {
                        let rows: Vec<ParameterRow> =
                            params.into_iter().map(ParameterRow::from).collect();
                        let _ = self.events.send(Event::Parameters { rows, deep });
                    }
                    // Not a fault: an unsupported read says nothing about the
                    // motor's health, and marking it as one would arm the
                    // fault counter over a driver that simply has no
                    // parameter space wired up.
                    Err(e) => self.log(
                        LogLevel::Warn,
                        format!("could not read parameters: {e}"),
                    ),
                }
            }
            Command::StartScan {
                from,
                to,
                timeout_ms,
            } => {
                let from = from.max(1);
                let to = to.max(from);
                // Scanning while driving would interleave probes with control
                // frames on the same wire.
                self.shared.streaming.store(false, Ordering::Release);
                self.scan = Some(ScanState {
                    from,
                    to,
                    next: from,
                    found: Vec::new(),
                    timeout: Duration::from_millis(timeout_ms.max(1)),
                    cancelled: false,
                });
                self.log(LogLevel::Info, format!("scanning ids {from}..={to}"));
            }
            Command::CancelScan => {
                if let Some(scan) = &mut self.scan {
                    scan.cancelled = true;
                }
            }
            Command::SetStreaming { on } => {
                if on && !self.enabled {
                    self.log(LogLevel::Warn, "cannot stream: motor is not enabled");
                } else {
                    // Refresh the heartbeat as streaming begins, so the
                    // watchdog does not fire on a stale timestamp from before
                    // the UI started beating.
                    self.shared.beat();
                    self.shared.streaming.store(on, Ordering::Release);
                }
            }
            Command::SetRate { hz } => {
                self.shared
                    .rate_millihz
                    .store(rate_to_millihz(hz), Ordering::Relaxed);
            }
            Command::StartJob { spec } => self.run_job(spec),
            // Handled by the caller so it can break the loop.
            Command::Shutdown => unreachable!("shutdown is handled in run()"),
        }
    }

    /// One control or poll transaction. `Some(reason)` ends the session.
    fn control_step(&mut self) -> Option<StopReason> {
        let streaming = self.shared.streaming.load(Ordering::Acquire);

        let result = if streaming && self.enabled {
            let sp = *self.shared.setpoint.lock().unwrap();

            // Put the driver in the mode the setpoint asks for before
            // commanding it.
            //
            // Several drivers reject a command issued in the wrong mode —
            // RobStride's `set_position` returns "requires Position run mode
            // (currently Mit)" — and a UI that only sends `set_run_mode` when
            // the operator clicks a mode button never sends it for whatever
            // mode the form happened to start on. Making the worker maintain
            // the invariant means no front end can get this wrong.
            if self.active_mode != Some(sp.mode) {
                match self.actuator.set_run_mode(sp.mode.into()) {
                    Ok(()) => {
                        self.active_mode = Some(sp.mode);
                        log::debug!("worker: run mode -> {:?}", sp.mode);
                    }
                    Err(e) => {
                        self.fault("set_run_mode", e);
                        return None;
                    }
                }
            }

            let outcome = match sp.mode {
                ControlMode::Position => {
                    self.actuator.set_position(sp.position_rad, sp.max_speed_rad_s)
                }
                ControlMode::Velocity => self.actuator.set_velocity(sp.velocity_rad_s),
                ControlMode::Torque => self.actuator.set_torque(sp.torque_nm),
                ControlMode::Mit => self.actuator.mit_control(
                    sp.position_rad,
                    sp.velocity_rad_s,
                    sp.kp,
                    sp.kd,
                    sp.torque_ff_nm,
                ),
            };
            Some((outcome, sp.primary()))
        } else if self.last_idle_poll.elapsed() >= IDLE_POLL {
            self.last_idle_poll = Instant::now();
            let fb = self.actuator.measure();
            // Seed from the first reading, not just from enable, so the UI
            // shows the real position as soon as it connects rather than a
            // zero that invites a jump.
            if let Ok(fb) = &fb {
                self.seed_position(fb.position_rad);
            }
            Some((fb, self.commanded_now()))
        } else {
            None
        };

        let (outcome, commanded) = result?;
        match outcome {
            Ok(fb) => {
                self.consecutive_faults = 0;
                self.push_sample(fb, commanded);
                None
            }
            Err(e) => {
                self.fault("control", e);
                if self.consecutive_faults >= CONSECUTIVE_FAULT_LIMIT {
                    // Stop driving, but keep the session alive. Ending the
                    // worker here used to make a recoverable mistake
                    // unrecoverable: a wrong run mode is rejected on every
                    // tick, twenty rejections arrive in a tenth of a second,
                    // and the user was left with a connected-looking UI whose
                    // every button returned "the worker has stopped". The
                    // fault limit is there to stop commanding a motor that is
                    // not listening, not to burn the connection down.
                    self.halt(StopReason::Fault);
                    self.consecutive_faults = 0;
                }
                None
            }
        }
    }

    /// Run a measurement to completion, blocking this thread.
    ///
    /// The worker's loop does not run during a job, which is deliberate: a
    /// chirp *is* a control loop, and interleaving anything else on the same
    /// bus would corrupt it. Two things still work because they do not go
    /// through the loop — the stop flag (which `misa-sysid` reads as its
    /// abort) and the watchdog thread that can set it.
    fn run_job(&mut self, spec: JobSpec) {
        let JobSpec::Chirp(job) = spec;

        // Starting with a stop already pending would abort instantly and look
        // like a failure.
        let _ = self.shared.take_stop();
        self.shared.streaming.store(false, Ordering::Release);
        self.shared.job_active.store(true, Ordering::Release);
        // The UI may have been idle up to now; without this the watchdog would
        // fire on a stale timestamp the moment the job arms it.
        self.shared.beat();
        let _ = self.events.send(Event::JobStarted { spec });
        self.log(
            LogLevel::Info,
            format!(
                "chirp: {:?} {} → {} Hz over {} s, amplitude {}",
                job.excitation, job.f_start_hz, job.f_end_hz, job.duration_s, job.amplitude
            ),
        );

        let chirp = misa_sysid::Chirp {
            f_start_hz: job.f_start_hz,
            f_end_hz: job.f_end_hz,
            duration_s: job.duration_s,
            amplitude: job.amplitude,
            bias: 0.0,
            sweep: if job.log_sweep {
                misa_sysid::Sweep::Logarithmic
            } else {
                misa_sysid::Sweep::Linear
            },
        };

        // Cloned out so the progress closure borrows none of `self` — the
        // actuator is borrowed mutably for the whole run.
        let telemetry = self.telemetry.clone();
        let events = self.events.clone();
        let started = self.shared.started;
        let duration_s = job.duration_s;

        let mut pending: Vec<Sample> = Vec::with_capacity(256);
        let mut last_flush = Instant::now();
        let mut last_progress = Instant::now();
        let mut count: u32 = 0;

        let result = misa_sysid::run_chirp_with(
            self.actuator.as_mut(),
            &chirp,
            excitation_of(&job),
            job.rate_hz,
            &self.shared.stop,
            &mut |s| {
                count += 1;
                pending.push(Sample {
                    // The sysid log times from the sweep's start; the UI plots
                    // against session time, so everything shares one axis.
                    t_s: started.elapsed().as_secs_f64(),
                    commanded: s.cmd,
                    position_rad: s.position_rad,
                    velocity_rad_s: s.velocity_rad_per_s,
                    torque_nm: s.torque_nm,
                    current_a: s.current_a,
                    temperature_c: f32::NAN,
                });
                if last_flush.elapsed() >= TELEMETRY_INTERVAL {
                    let elapsed = last_flush.elapsed().as_secs_f32().max(1e-6);
                    let n = pending.len();
                    let _ = telemetry.try_send(TelemetryBatch {
                        samples: std::mem::take(&mut pending),
                        dropped: 0,
                        achieved_rate_hz: n as f32 / elapsed,
                    });
                    last_flush = Instant::now();
                }
                if last_progress.elapsed() >= Duration::from_millis(100) {
                    let _ = events.send(Event::JobProgress {
                        elapsed_s: s.t_s,
                        duration_s,
                        samples: count,
                    });
                    last_progress = Instant::now();
                }
            },
        );

        // Whatever happened, the run left the motor disabled (`run_chirp`
        // commands a safe stop on every exit path) and the job is over.
        self.shared.job_active.store(false, Ordering::Release);
        self.enabled = false;
        if !pending.is_empty() {
            let n = pending.len();
            let _ = self.telemetry.try_send(TelemetryBatch {
                samples: std::mem::take(&mut pending),
                dropped: 0,
                achieved_rate_hz: n as f32,
            });
        }

        // A stop that arrived during the run has already done its job by
        // aborting the sweep; consuming it here keeps the main loop from
        // halting a motor that is already stopped and reporting it twice.
        let aborted = self.shared.take_stop().is_some();

        match result {
            Ok(log) => {
                let samples = log.samples.len() as u32;
                let command_pp = peak_to_peak(log.samples.iter().map(|s| s.cmd));
                // The same output the FRF uses: torque-input channels identify
                // ω/τ, everything else identifies against position.
                let response_pp = match job.excitation {
                    ExcitationKind::Velocity
                    | ExcitationKind::Torque
                    | ExcitationKind::MitTorque => {
                        peak_to_peak(log.samples.iter().map(|s| s.velocity_rad_per_s))
                    }
                    _ => peak_to_peak(log.samples.iter().map(|s| s.position_rad)),
                };
                // A response that never moved yields a Bode plot at about
                // -240 dB — the logarithm of numerical zero. It looks like a
                // measurement and is not one, so say so plainly.
                if command_pp > 0.0 && response_pp < command_pp * 0.001 {
                    self.log(
                        LogLevel::Warn,
                        format!(
                            "the shaft barely moved: {response_pp:.5} response \
                             against {command_pp:.5} commanded — check the \
                             amplitude, the gains, and that the load is free"
                        ),
                    );
                }
                // Below a few dozen samples the FFT has nothing to say, and a
                // Bode plot drawn from it would be noise presented as a
                // measurement.
                let bode = (samples >= 64).then(|| log.frf().into());
                if bode.is_none() {
                    self.log(
                        LogLevel::Warn,
                        format!("chirp produced only {samples} samples — too few to identify"),
                    );
                }
                self.log(
                    LogLevel::Info,
                    format!(
                        "chirp {}: {samples} samples at {:.0} Hz",
                        if aborted { "aborted" } else { "finished" },
                        log.achieved_rate_hz
                    ),
                );
                let _ = self.events.send(Event::JobFinished {
                    bode,
                    achieved_rate_hz: log.achieved_rate_hz,
                    samples,
                    command_pp,
                    response_pp,
                    aborted,
                });
            }
            Err(e) => {
                self.log(LogLevel::Error, format!("chirp failed: {e}"));
                let _ = self.events.send(Event::JobFailed {
                    message: e.to_string(),
                });
            }
        }
    }

    fn scan_step(&mut self) {
        let Some(mut scan) = self.scan.take() else {
            return;
        };
        if scan.finished() {
            // No log line here: `ScanFinished` already carries the result, and
            // a consumer that renders both ends up printing the same thing
            // twice.
            log::info!("worker: scan finished, found {:?}", scan.found);
            let _ = self.events.send(Event::ScanFinished {
                found: scan.found.clone(),
                cancelled: scan.cancelled,
            });
            return;
        }

        let id = scan.next;
        scan.next = scan.next.saturating_add(1);
        match self.actuator.probe_motor(id, scan.timeout) {
            Ok(true) => scan.found.push(id),
            Ok(false) => {}
            // One id failing is not a reason to abandon the sweep — a bus
            // error on id 7 says nothing about id 8.
            Err(e) => log::debug!("probe id={id} failed: {e}"),
        }

        let _ = self.events.send(Event::ScanProgress {
            done: scan.done(),
            total: scan.total(),
            current: id,
            found: scan.found.clone(),
        });
        self.scan = Some(scan);
    }

    fn read_status(&mut self) {
        match self.actuator.read_status() {
            Ok(st) => {
                let _ = self.events.send(Event::Status(StatusSnapshot {
                    voltage_v: st.voltage_v,
                    temperature_c: st.temperature_c,
                    error_bits: st.error.bits(),
                    error_raw: st.error.raw(),
                    enabled: self.enabled,
                }));
            }
            Err(e) => self.fault("read_status", e),
        }
    }

    /// Stop driving the motor, for a reason that is not the end of the session.
    fn halt(&mut self, reason: StopReason) {
        self.shared.streaming.store(false, Ordering::Release);
        self.scan = None;
        match self.actuator.disable() {
            Ok(()) => self.enabled = false,
            Err(e) => log::warn!("worker: disable during halt failed: {e}"),
        }
        let _ = self.events.send(Event::Stopped { reason });
        if reason.is_unexpected() {
            self.log(LogLevel::Error, format!("stopped: {reason:?}"));
        }
    }

    /// Point the position setpoint at where the shaft actually is, but only
    /// while nobody has chosen a target.
    ///
    /// A motor remembers its position across power cycles; a freshly-opened
    /// form does not. Left alone, the first streaming tick would command the
    /// form's default of zero and a shaft resting at 5.9 rad would set off for
    /// the origin at whatever the speed cap allows. Seeding makes the
    /// displayed target agree with reality from the moment there is a reading.
    ///
    /// Conditional on `setpoint_touched` because the alternative — always
    /// overwriting — races the caller: the setpoint is written immediately
    /// while commands are queued, so "set a target, then enable" would have
    /// the enable silently discard the target.
    fn seed_position(&mut self, position_rad: f32) {
        if !position_rad.is_finite()
            || self.shared.setpoint_touched.load(Ordering::Acquire)
        {
            return;
        }
        if let Ok(mut sp) = self.shared.setpoint.lock() {
            sp.position_rad = position_rad;
        }
    }

    fn commanded_now(&self) -> f32 {
        self.shared
            .setpoint
            .lock()
            .map(|sp| sp.primary())
            .unwrap_or(0.0)
    }

    fn sample(&self, fb: misa_actuator::MotorFeedback, commanded: f32) -> Sample {
        Sample {
            t_s: self.shared.started.elapsed().as_secs_f64(),
            commanded,
            position_rad: fb.position_rad,
            velocity_rad_s: fb.velocity_rad_per_s,
            torque_nm: fb.torque_nm,
            current_a: fb.current_a,
            temperature_c: fb.temperature_c,
        }
    }

    fn push_sample(&mut self, fb: misa_actuator::MotorFeedback, commanded: f32) {
        let s = self.sample(fb, commanded);
        self.pending.push(s);
        self.samples_since_flush += 1;
    }

    fn flush_if_due(&mut self) {
        if self.last_flush.elapsed() >= TELEMETRY_INTERVAL {
            self.flush(false);
        }
    }

    fn flush(&mut self, force: bool) {
        if self.pending.is_empty() && !force {
            self.last_flush = Instant::now();
            return;
        }
        let elapsed = self.last_flush.elapsed().as_secs_f32().max(1e-6);
        let batch = TelemetryBatch {
            samples: std::mem::take(&mut self.pending),
            dropped: self.dropped,
            achieved_rate_hz: self.samples_since_flush as f32 / elapsed,
        };
        self.samples_since_flush = 0;
        self.last_flush = Instant::now();

        match self.telemetry.try_send(batch) {
            Ok(()) => self.dropped = 0,
            // Telemetry is the one thing here that is safe to lose. Count it
            // so the UI can show a gap rather than drawing a smooth line
            // through missing data.
            Err(TrySendError::Full(batch)) => {
                self.dropped += batch.samples.len() as u64;
                log::debug!("telemetry consumer is behind; dropped {} samples", batch.samples.len());
            }
            Err(TrySendError::Disconnected(_)) => {}
        }
    }

    fn status_if_due(&mut self) {
        if self.last_status.elapsed() >= STATUS_INTERVAL {
            self.last_status = Instant::now();
            self.read_status();
        }
    }

    fn fault(&mut self, what: &str, e: ActuatorError) {
        self.consecutive_faults += 1;
        // Timeouts are routine on a shared bus; only say something once a run
        // of them suggests the motor has actually gone.
        let level = if matches!(e, ActuatorError::Timeout { .. }) && self.consecutive_faults < 5 {
            LogLevel::Warn
        } else {
            LogLevel::Error
        };
        self.log(level, format!("{what} failed: {e}"));
    }

    fn log(&self, level: LogLevel, message: impl Into<String>) {
        let message = message.into();
        match level {
            LogLevel::Info => log::info!("worker: {message}"),
            LogLevel::Warn => log::warn!("worker: {message}"),
            LogLevel::Error => log::error!("worker: {message}"),
        }
        let _ = self.events.send(Event::Log { level, message });
    }
}

/// Span of a signal. `0.0` for an empty or all-NaN run rather than infinity,
/// so a caller comparing it against a threshold does not get a surprise.
fn peak_to_peak(values: impl Iterator<Item = f32>) -> f32 {
    let mut lo = f32::INFINITY;
    let mut hi = f32::NEG_INFINITY;
    for v in values.filter(|v| v.is_finite()) {
        lo = lo.min(v);
        hi = hi.max(v);
    }
    if lo > hi {
        0.0
    } else {
        hi - lo
    }
}
