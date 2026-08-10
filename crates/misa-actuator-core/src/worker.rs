//! The motor I/O thread.
//!
//! Everything that touches the bus happens here, on one thread that owns the
//! `Box<dyn Actuator>` outright. That ownership is the point:
//!
//! - a UI can never block on a bus transaction, however slow the wire is
//! - the control rate is the worker's business, not the frame rate's
//! - if the UI dies, the command channel closes, the worker notices and
//!   disables the motor on its way out — no supervision required

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender, SyncSender, TryRecvError, TrySendError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use misa_actuator::realtime::{sleep_until, TimerResolutionGuard};
use misa_actuator::{Actuator, Error as ActuatorError, MotorFeedback, RunMode};

use crate::protocol::*;
use crate::safety::Safety;

use crate::safety::{CONSECUTIVE_FAULT_LIMIT, IDLE_POLL};

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

/// An [`Actuator`] that tallies every command it forwards.
///
/// Wrapped around the real one for the duration of a characterization run, so a
/// progress display can show bus traffic and not only a clock. Nothing in
/// `misa_sysid` reports progress — its runs are blocking calls that return once
/// — and rather than thread a callback through four signatures and their tests,
/// the count comes from the one thing the run cannot avoid touching.
///
/// Every method forwards unchanged. Only the ones that put a frame on the bus
/// count; `motor_id` and the two hints are local reads.
struct Counting<'a> {
    inner: &'a mut (dyn Actuator + Send),
    safety: &'a Safety,
}

impl Counting<'_> {
    fn tally<T>(&self, r: T) -> T {
        self.safety.job_transaction();
        r
    }
}

impl Actuator for Counting<'_> {
    fn motor_id(&self) -> u8 {
        self.inner.motor_id()
    }
    fn enable(&mut self) -> misa_actuator::Result<MotorFeedback> {
        let r = self.inner.enable();
        self.tally(r)
    }
    fn disable(&mut self) -> misa_actuator::Result<()> {
        let r = self.inner.disable();
        self.tally(r)
    }
    fn set_zero(&mut self) -> misa_actuator::Result<()> {
        let r = self.inner.set_zero();
        self.tally(r)
    }
    fn set_run_mode(&mut self, mode: RunMode) -> misa_actuator::Result<()> {
        let r = self.inner.set_run_mode(mode);
        self.tally(r)
    }
    fn set_position(
        &mut self,
        pos_rad: f32,
        max_speed_rad_s: f32,
    ) -> misa_actuator::Result<MotorFeedback> {
        let r = self.inner.set_position(pos_rad, max_speed_rad_s);
        self.tally(r)
    }
    fn set_velocity(&mut self, vel_rad_s: f32) -> misa_actuator::Result<MotorFeedback> {
        let r = self.inner.set_velocity(vel_rad_s);
        self.tally(r)
    }
    fn set_torque(&mut self, torque_nm: f32) -> misa_actuator::Result<MotorFeedback> {
        let r = self.inner.set_torque(torque_nm);
        self.tally(r)
    }
    fn mit_control(
        &mut self,
        pos_rad: f32,
        vel_rad_s: f32,
        kp: f32,
        kd: f32,
        torque_ff_nm: f32,
    ) -> misa_actuator::Result<MotorFeedback> {
        let r = self
            .inner
            .mit_control(pos_rad, vel_rad_s, kp, kd, torque_ff_nm);
        self.tally(r)
    }
    fn measure(&mut self) -> misa_actuator::Result<MotorFeedback> {
        let r = self.inner.measure();
        self.tally(r)
    }
    fn read_status(&mut self) -> misa_actuator::Result<misa_actuator::MotorStatus> {
        let r = self.inner.read_status();
        self.tally(r)
    }
    fn current_run_mode_hint(&self) -> Option<RunMode> {
        self.inner.current_run_mode_hint()
    }
    fn is_enabled_hint(&self) -> bool {
        self.inner.is_enabled_hint()
    }
    fn set_report_current(&mut self, on: bool) {
        self.inner.set_report_current(on);
    }
    fn set_torque_constant(&mut self, kt_nm_per_a: f32) {
        self.inner.set_torque_constant(kt_nm_per_a);
    }
}

/// How much of the safety envelope's position window a Kt run's leash may use
/// up when it is holding the largest torque level in the sweep.
///
/// Well under 1.0, because the deflection this bounds is the *steady-state* one:
/// the shaft overshoots on the way there, the window is measured from wherever
/// the run began, and a run that just fits at equilibrium is a run that trips on
/// the way to it.
const KT_LEASH_DEFLECTION: f32 = 0.4;

/// State shared between the session handle, the watchdog and the worker.
///
/// The stop flag, watchdog, heartbeat and rate clamp are in
/// [`crate::safety::Safety`], shared with every other kind of session — see that
/// module for why they are not duplicated here. What is left is the one motor's
/// setpoint, which is what actually differs from a multi-motor session.
///
/// Deliberately atomics plus one small mutex: the UI thread touches this on
/// every frame, and a contended lock there would show up as jank.
pub(crate) struct Shared {
    pub(crate) safety: Arc<Safety>,
    pub(crate) setpoint: Mutex<Setpoint>,
    /// Whether anything has deliberately written a setpoint yet.
    ///
    /// Until it has, the worker is free to seed the position from the shaft so
    /// the UI shows where the motor actually is. Once a caller has chosen a
    /// target, seeding would be overwriting intent — and worse, racing it,
    /// since the setpoint is written immediately while commands are queued.
    pub(crate) setpoint_touched: AtomicBool,
}

impl Shared {
    pub(crate) fn new(rate_hz: f32, watchdog: Option<Duration>) -> Self {
        Self {
            safety: Arc::new(Safety::new(rate_hz, watchdog)),
            setpoint: Mutex::new(Setpoint::default()),
            setpoint_touched: AtomicBool::new(false),
        }
    }

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
            if let Some(reason) = self.shared.safety.take_stop() {
                self.halt(reason);
            }

            if self.scan.is_some() {
                self.scan_step();
            } else if let Some(reason) = self.control_step() {
                break 'outer reason;
            }

            self.flush_if_due();
            self.status_if_due();

            next_tick += self.shared.safety.period();
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
                self.shared.safety.set_streaming(false);
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
                if self.shared.safety.take_streaming() {
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
                self.shared.safety.set_streaming(false);
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
                    self.shared.safety.beat();
                    self.shared.safety.set_streaming(on);
                }
            }
            Command::SetRate { hz } => self.shared.safety.set_rate(hz),
            Command::StartJob { spec } => self.run_job(spec),
            // Handled by the caller so it can break the loop.
            Command::Shutdown => unreachable!("shutdown is handled in run()"),
        }
    }

    /// One control or poll transaction. `Some(reason)` ends the session.
    fn control_step(&mut self) -> Option<StopReason> {
        let streaming = self.shared.safety.is_streaming();

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
        let job = match spec {
            JobSpec::Chirp(job) => job,
            JobSpec::Characterize { run, envelope } => {
                return self.run_characterize(run, envelope);
            }
        };

        // Starting with a stop already pending would abort instantly and look
        // like a failure.
        let _ = self.shared.safety.take_stop();
        self.shared.safety.set_streaming(false);
        self.shared.safety.set_job_active(true);
        // The UI may have been idle up to now; without this the watchdog would
        // fire on a stale timestamp the moment the job arms it.
        self.shared.safety.beat();
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
        let started = self.shared.safety.started();
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
            self.shared.safety.abort_flag(),
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
        self.shared.safety.set_job_active(false);
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
        let aborted = self.shared.safety.take_stop().is_some();

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

    /// A quasi-static characterization run.
    ///
    /// Same lifecycle as a chirp — the worker owns its loop, `STOP` reaches it
    /// through the shared abort flag — but the primitives return a whole
    /// result at the end rather than streaming samples, so there is no
    /// progress to report beyond "running". Both leave the motor disabled on
    /// every exit path.
    #[allow(clippy::type_complexity)]
    fn run_characterize(&mut self, run: CharacterizeJob, envelope: RunEnvelope) {
        let _ = self.shared.safety.take_stop();
        self.shared.safety.set_streaming(false);
        self.shared.safety.begin_job(Some(run.expected_duration()));
        self.shared.safety.beat();
        let _ = self.events.send(Event::JobStarted {
            spec: JobSpec::Characterize { run, envelope },
        });
        self.log(LogLevel::Info, format!("{}: starting", run.name()));

        // Apply the run's Kt and current reporting for the run only, then put
        // both back. They used to be connect-time options, which meant
        // reconnecting to change a Kt and paying for current reporting on
        // every chirp that followed. Both are no-ops on drivers that do not
        // derive torque from current.
        if envelope.kt > 0.0 {
            self.actuator.set_report_current(true);
            self.actuator.set_torque_constant(envelope.kt);
            self.log(
                LogLevel::Info,
                format!("{}: deriving torque from current at Kt {:.4} N·m/A", run.name(), envelope.kt),
            );
        } else if run.needs_current() {
            // The Kt run is the one case where no Kt is known yet — that is
            // what it is measuring — so it needs current on its own account.
            self.actuator.set_report_current(true);
        }

        // `gentle` rather than anything tuned: this envelope is survivable on
        // the smallest motor in the workspace, and the GUI does not know which
        // one is plugged in. A rig that can take more should say so
        // deliberately, not inherit it from a default — which is what the
        // connect-time ceiling is for.
        //
        // It has to be raisable, though: 1 N·m is below what a geared RS-03 needs
        // to move at all, so every friction run on it aborted on the ceiling as
        // soon as a Kt made the torque visible (2026-08-06). A limit that no run
        // can stay inside protects nothing and measures nothing.
        let mut limits = misa_sysid::SafetyLimits::gentle();
        if envelope.max_torque_nm > 0.0 {
            limits.max_torque_nm = envelope.max_torque_nm;
            self.log(
                LogLevel::Warn,
                format!(
                    "torque envelope raised to {:.3} N·m for this run",
                    limits.max_torque_nm
                ),
            );
        }
        let ceiling_nm = limits.max_torque_nm;

        // Both are read out of the reports before the mappers consume them.
        // An envelope abort comes back as `Ok` with a reason *inside* the
        // report rather than as an `Err` — deliberately, so a run that was cut
        // short still hands over what it measured — which makes this the only
        // place the reason is visible.
        let mut fitted_kt: Option<f32> = None;
        let mut abort_reason: Option<misa_sysid::AbortReason> = None;
        let mut note_abort = |abort: Option<misa_sysid::AbortReason>| {
            abort_reason = abort;
        };

        // The samples, kept aside before the mappers reduce each report to the
        // two series the plot draws. Copied rather than borrowed because the
        // report is consumed by the mapper in the same expression.
        let mut samples: Vec<misa_sysid::Point> = Vec::new();
        let mut keep = |ps: &[misa_sysid::Point]| samples.extend_from_slice(ps);

        let outcome = match run {
            CharacterizeJob::LoadMap {
                from_rad,
                to_rad,
                steps,
                settle_s,
                max_speed_rad_s,
                return_sweep,
            } => {
                let spec = misa_sysid::LoadMapSpec {
                    from_rad,
                    to_rad,
                    steps: (steps as usize).max(2),
                    settle_s,
                    max_speed_rad_s,
                    return_sweep,
                    ..misa_sysid::LoadMapSpec::symmetric(0.5, 2)
                };
                misa_sysid::run_load_map(
                    &mut Counting {
                        inner: self.actuator.as_mut(),
                        safety: &self.shared.safety,
                    },
                    &spec,
                    limits,
                    self.shared.safety.abort_flag(),
                )
                    .map(|m| {
                        note_abort(m.abort);
                        keep(&m.points);
                        load_map_data(m)
                    })
            }
            CharacterizeJob::VelocitySweep {
                half_span_rad,
                speed_rad_s,
                rate_hz,
                return_sweep,
                bins,
            } => {
                let spec = misa_sysid::VelocitySweepSpec {
                    speed_rad_s,
                    half_span_rad,
                    rate_hz,
                    return_sweep,
                    // Not a UI field: it exists to keep the reversal transient
                    // out of the end bins, and a value that does not do that is
                    // not a preference anyone should be offered.
                    lead_in_rad: misa_sysid::VelocitySweepSpec::lead_in_for(half_span_rad),
                };
                misa_sysid::run_velocity_sweep(
                    &mut Counting {
                        inner: self.actuator.as_mut(),
                        safety: &self.shared.safety,
                    },
                    &spec,
                    limits,
                    self.shared.safety.abort_flag(),
                )
                .map(|v| {
                    note_abort(v.abort);
                    keep(&v.points);
                    velocity_sweep_data(v, (bins as usize).max(1))
                })
            }
            CharacterizeJob::Breakaway {
                ramp_nm_per_s,
                max_torque_nm,
                positive,
                rate_hz,
            } => {
                let direction = if positive {
                    misa_sysid::Direction::Positive
                } else {
                    misa_sysid::Direction::Negative
                };
                let spec = misa_sysid::BreakawaySpec {
                    ramp_nm_per_s,
                    rate_hz,
                    ..misa_sysid::BreakawaySpec::slow(max_torque_nm, direction)
                };
                misa_sysid::run_breakaway(
                    &mut Counting {
                        inner: self.actuator.as_mut(),
                        safety: &self.shared.safety,
                    },
                    &spec,
                    limits,
                    self.shared.safety.abort_flag(),
                )
                    .map(|b| {
                        note_abort(b.abort);
                        keep(&b.points);
                        breakaway_data(b)
                    })
            }
            CharacterizeJob::BreakawayMap {
                half_span_rad,
                steps,
                ramp_nm_per_s,
                max_torque_nm,
                rate_hz,
                both_directions,
            } => {
                let ramp = misa_sysid::BreakawaySpec {
                    ramp_nm_per_s,
                    rate_hz,
                    ..misa_sysid::BreakawaySpec::slow(max_torque_nm, misa_sysid::Direction::Positive)
                };
                let spec = misa_sysid::BreakawayMapSpec {
                    both_directions,
                    ..misa_sysid::BreakawayMapSpec::symmetric(
                        half_span_rad,
                        (steps as usize).max(1),
                        ramp,
                    )
                };
                misa_sysid::run_breakaway_map(
                    &mut Counting {
                        inner: self.actuator.as_mut(),
                        safety: &self.shared.safety,
                    },
                    &spec,
                    limits,
                    self.shared.safety.abort_flag(),
                )
                .map(|m| {
                    note_abort(m.abort);
                    keep(&m.samples);
                    breakaway_map_data(m)
                })
            }
            CharacterizeJob::Kt {
                max_torque_nm,
                steps,
                settle_s,
                rate_hz,
                leash_kp,
                leash_kd,
            } => {
                // A leash too soft for the amplitude deflects straight out of
                // the position window, and the run dies at its first level with
                // a single point — which `kt_nm_per_a` then reports as "torque
                // and current did not vary enough to fit a slope", describing
                // the symptom and not the cause. Measured on an RS-04 on
                // 2026-08-06: 7.2 N·m against kp 8 settles at 0.9 rad, and the
                // window is ±0.5.
                //
                // The rule lives here rather than in the caller because it is
                // the window that decides it, and the window is this crate's.
                // kp 0 is left alone: it means "the shaft is already
                // restrained", which is a claim about the rig, not an omission.
                let asked_kp = leash_kp;
                let (leash_kp, leash_kd) =
                    kt_leash(max_torque_nm, leash_kp, leash_kd, limits.position_window_rad);
                if leash_kp > asked_kp {
                    self.log(
                        LogLevel::Warn,
                        format!(
                            "kt: leash stiffened from kp {asked_kp:.1} to {leash_kp:.1} so {max_torque_nm:.2} N·m stays inside the ±{:.2} rad window",
                            limits.position_window_rad
                        ),
                    );
                }
                let spec = misa_sysid::KtSpec {
                    max_torque_nm,
                    steps: (steps as usize).max(2),
                    settle_s,
                    rate_hz,
                    leash_kp,
                    leash_kd,
                };
                misa_sysid::run_kt(
                    &mut Counting {
                        inner: self.actuator.as_mut(),
                        safety: &self.shared.safety,
                    },
                    &spec,
                    limits,
                    self.shared.safety.abort_flag(),
                )
                    .map(|k| {
                        note_abort(k.abort);
                        keep(&k.points);
                        let (data, note, kt) = kt_data(k);
                        fitted_kt = kt;
                        (data, note)
                    })
            }
        };

        self.shared.safety.set_job_active(false);
        self.enabled = false;
        let aborted = self.shared.safety.take_stop().is_some();

        // Hand the driver back the way it was found. Leaving a Kt behind would
        // make a later run report synthesized torque without anything on
        // screen saying so, and leaving current reporting on would halve the
        // rate of the next chirp.
        self.actuator.set_torque_constant(0.0);
        self.actuator.set_report_current(false);

        // Which limit stopped the run, named. Without this the operator sees
        // only the consequence — "torque and current did not vary enough to fit
        // a slope" is what a Kt run reports after the position window cut it off
        // at the first level, and that describes the symptom while saying
        // nothing about the cause (2026-08-06, an RS-04 whose leash deflected
        // 0.9 rad against a ±0.5 rad window).
        if let Some(r) = abort_reason {
            self.log(
                LogLevel::Warn,
                format!("{}: the safety envelope stopped it — {}", run.name(), r.describe()),
            );
        }

        // Only worth suggesting when the ceiling is what stopped the run.
        // Doubling it is a starting point, not a finding — see the field's
        // docs for why the samples cannot say more than this.
        let suggested_torque_limit_nm = (abort_reason == Some(misa_sysid::AbortReason::TorqueLimit))
            .then(|| (ceiling_nm * 2.0).max(0.5));
        if let Some(t) = suggested_torque_limit_nm {
            self.log(
                LogLevel::Warn,
                format!(
                    "{}: stopped by the {:.3} N·m torque ceiling — this joint needs more than that to move; retry at {:.3} N·m if the rig can take it",
                    run.name(),
                    ceiling_nm,
                    t
                ),
            );
        }

        match outcome {
            Ok((data, note)) => {
                self.log(
                    LogLevel::Info,
                    format!(
                        "{}: {note}{}",
                        run.name(),
                        if aborted { " (stopped early)" } else { "" }
                    ),
                );
                let csv_path = self.save_run(run.name(), &samples, &data.summary);
                let _ = self.events.send(Event::CharacterizeFinished {
                    run,
                    // An empty series is not a measurement, and plotting one
                    // looks exactly like a measurement of zero.
                    data: (!data.x.is_empty()).then_some(data),
                    aborted,
                    fitted_kt,
                    suggested_torque_limit_nm,
                    csv_path,
                });
            }
            Err(e) => {
                self.log(LogLevel::Error, format!("{} failed: {e}", run.name()));
                // Still written. A run that failed part-way took the motor's
                // time and its samples are how anyone works out why.
                let csv_path = self.save_run(run.name(), &samples, &[]);
                let _ = self.events.send(Event::CharacterizeFinished {
                    run,
                    data: None,
                    aborted,
                    fitted_kt: None,
                    suggested_torque_limit_nm,
                    csv_path,
                });
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
        self.shared.safety.set_streaming(false);
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
            t_s: self.shared.safety.elapsed().as_secs_f64(),
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

    /// Write a finished run's samples, and say where they went.
    ///
    /// A failure here is logged and returns `None` rather than propagating: the
    /// measurement on screen is valid whether or not the disk cooperated, and
    /// throwing it away because a directory was read-only would be the worse
    /// outcome by far.
    fn save_run(
        &self,
        run: &str,
        samples: &[misa_sysid::Point],
        summary: &[(String, String)],
    ) -> Option<String> {
        if samples.is_empty() {
            // Nothing was recorded, so there is no file worth leaving behind to
            // be found later and mistaken for a run.
            return None;
        }
        let dir = crate::export::data_dir();
        let name = crate::export::run_file_name(
            &crate::export::utc_stamp(crate::export::now_secs()),
            run,
            self.actuator.motor_id(),
        );
        match crate::export::write_run_csv(
            &dir,
            &name,
            run,
            self.actuator.motor_id(),
            "",
            summary,
            samples,
        ) {
            Ok(path) => {
                let shown = path.display().to_string();
                self.log(
                    LogLevel::Info,
                    format!("{run}: {} samples saved to {shown}", samples.len()),
                );
                Some(shown)
            }
            Err(e) => {
                self.log(
                    LogLevel::Warn,
                    format!(
                        "{run}: could not save samples to {}: {e} — the result on screen is still valid",
                        dir.display()
                    ),
                );
                None
            }
        }
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
/// Torque against position, split into the two sweep directions.
///
/// Plotted as two series rather than one: a load map that goes out and back
/// separates into two curves exactly when there is hysteresis, and averaging
/// them into a single line would hide the friction that separation measures.
fn load_map_data(m: misa_sysid::LoadMap) -> (CharacterizeData, String) {
    let n = m.points.len();
    // The outbound sweep is the first half when a return sweep ran.
    let split = if m.spec.return_sweep { n.div_ceil(2) } else { n };
    let (out, back) = m.points.split_at(split.min(n));

    let xy = |ps: &[misa_sysid::Point]| {
        (
            ps.iter().map(|p| p.position_rad).collect::<Vec<_>>(),
            ps.iter().map(|p| p.torque_nm).collect::<Vec<_>>(),
        )
    };
    let (x, y) = xy(out);
    let (x2, y2) = xy(back);

    let peak = m
        .points
        .iter()
        .map(|p| p.torque_nm.abs())
        .fold(0.0f32, f32::max);
    let nm = |v: Option<f32>| match v {
        Some(x) => format!("{x:+.3} N·m"),
        None => "not measured".to_string(),
    };

    // The gap between the two legs is what the run is for, so the figures that
    // quantify it lead. Without them the plot showed two separated clouds and
    // left the reader to eyeball the separation — which is the measurement
    // (2026-08-06, an RS-04 whose operator quite reasonably asked what the two
    // clouds were). The CLI had been printing these all along.
    let mut summary = vec![
        (
            "mean friction".to_string(),
            match m.mean_friction_nm() {
                Some(f) => format!("{f:.3} N·m"),
                // Only a return sweep can separate friction from load: one leg
                // measures load ± friction and cannot say which is which.
                None => "needs a return sweep".to_string(),
            },
        ),
        // `static_load_curve` is friction *per position*, and reporting only its
        // mean threw the angle dependence away — which for a geared joint is
        // often the interesting part. The range says whether one number can
        // describe this joint at all.
        (
            "friction over angle".to_string(),
            {
                let f: Vec<f32> = m.static_load_curve().iter().map(|&(_, _, f)| f).collect();
                match (
                    f.iter().copied().reduce(f32::min),
                    f.iter().copied().reduce(f32::max),
                ) {
                    (Some(lo), Some(hi)) => format!("{lo:.3} – {hi:.3} N·m"),
                    _ => "needs a return sweep".to_string(),
                }
            },
        ),
        (
            "peak hysteresis".to_string(),
            match m.peak_hysteresis_nm() {
                Some(h) => format!("{h:.3} N·m"),
                None => "needs a return sweep".to_string(),
            },
        ),
        ("peak static load".to_string(), nm(m.peak_static_load_nm())),
        (
            "equilibrium".to_string(),
            match m.static_equilibrium_position_rad() {
                Some(p) => format!("{p:+.4} rad"),
                // No sign change in the swept range: the load does not balance
                // anywhere inside it, which is a fact about the range.
                None => "not inside the range".to_string(),
            },
        ),
        ("points".to_string(), n.to_string()),
        ("peak |torque|".to_string(), format!("{peak:.3} N·m")),
    ];

    // A dwell spread comparable to the torque itself means the shaft was
    // hunting, so that point averages a limit cycle rather than measuring a
    // steady load. Said here rather than left for the reader to compare two
    // tiles, because it decides whether the other numbers mean anything.
    let spread = m.worst_dwell_spread_nm();
    let hunting = matches!(
        (spread, m.peak_holding_torque_nm()),
        (Some(s), Some(p)) if s >= p.abs()
    );
    summary.push((
        "worst dwell spread".to_string(),
        match spread {
            Some(w) if hunting => format!("{w:.3} N·m — hunting, not settling"),
            Some(w) => format!("{w:.3} N·m"),
            None => "not measured".to_string(),
        },
    ));

    let note = match m.mean_friction_nm() {
        Some(f) if hunting => format!(
            "{n} points, friction {f:.3} N·m — but the dwells were hunting, so treat every figure as an average of a limit cycle; a longer settle would fix it"
        ),
        Some(f) => format!("{n} points, friction {f:.3} N·m, peak {peak:.3} N·m"),
        None => format!("{n} points, peak {peak:.3} N·m"),
    };
    (
        CharacterizeData {
            x_label: "position [rad]".to_string(),
            y_label: "torque [N·m]".to_string(),
            x,
            y,
            series: "outbound".to_string(),
            x2: (!x2.is_empty()).then_some(x2),
            y2: (!y2.is_empty()).then_some(y2),
            series2: (!back.is_empty()).then(|| "return".to_string()),
            summary,
        },
        note,
    )
}

/// The torque ramp and where the shaft let go.
///
/// The **commanded** ramp is the primary series. Plotting the measured torque
/// alone produced a flat line at zero: what this run varies is the command,
/// and on a shaft that has not moved yet the feedback has little to report.
/// Measured torque rides alongside as the second series, so the gap between
/// what was asked for and what the motor registered stays visible.
/// Kinetic friction and load, both against position.
///
/// The plotted curve is the binned split rather than the raw log: the raw samples
/// are torque against position for two directions, which is the *input* to the
/// split, not its answer. There is no speed axis — the traverse holds one speed,
/// which is what makes the split valid.
///
/// Both halves are plotted because they come from the same subtraction and are
/// only meaningful together: friction is the half-difference between the
/// directions, load the half-sum.
fn velocity_sweep_data(v: misa_sysid::VelocitySweep, bins: usize) -> (CharacterizeData, String) {
    // `(position, static_load, kinetic_friction)` per bin both passes visited.
    let curve = v.friction_curve(bins);
    let x: Vec<f32> = curve.iter().map(|(pos, _, _)| *pos).collect();
    let y: Vec<f32> = curve.iter().map(|(_, _, fric)| *fric).collect();
    let load_series: Vec<f32> = curve.iter().map(|(_, load, _)| *load).collect();

    let friction = v.mean_kinetic_friction_nm(bins);
    let load = v.peak_static_load_nm(bins);
    // From the positions visited, not from the velocity the motor reported: on
    // some firmware that field is an exact zero most frames, which called every
    // run stalled while the shaft turned at exactly the commanded speed.
    let achieved = v.traversed_speed_rad_s();
    let claimed = v.mean_speed_rad_s();

    let mut summary = vec![("samples".to_string(), v.points.len().to_string())];
    // Torque that is identically zero is the signature of firmware that reports
    // none (handover §4), not of a frictionless joint. Say which.
    let no_torque = v.points.iter().all(|p| p.torque_nm == 0.0);
    summary.push((
        "kinetic friction".to_string(),
        match friction {
            Some(_) if no_torque => "no torque was reported — pass a Kt".to_string(),
            Some(f) => format!("{f:.3} N·m"),
            None => "not measured".to_string(),
        },
    ));
    summary.push((
        "load (half-sum)".to_string(),
        match load {
            Some(_) if no_torque => "no torque was reported".to_string(),
            Some(l) => format!("{l:.3} N·m"),
            None => "not measured".to_string(),
        },
    ));
    summary.push((
        "speed travelled".to_string(),
        match achieved {
            Some(s) => format!("{s:.4} rad/s of {:.4} commanded", v.spec.speed_rad_s),
            None => "not measured".to_string(),
        },
    ));
    // Reported only when it contradicts the positions, because then it is a fact
    // about the motor rather than about the joint: a velocity field that reads
    // zero while the shaft moves is the same firmware trait as a torque field
    // that does.
    if let (Some(a), Some(c)) = (achieved, claimed) {
        if a > 0.01 && c < a * 0.5 {
            summary.push((
                "reported velocity".to_string(),
                format!("{c:.4} rad/s — under half what the positions show; the motor's velocity field is unreliable"),
            ));
        }
    }
    if !v.spec.return_sweep {
        // Without the return sweep the split has nothing to subtract, so the
        // number above is load plus friction, not friction.
        summary.push((
            "single direction".to_string(),
            "load and friction are not separated".to_string(),
        ));
    }
    // A traverse that stalled was not in steady motion, so the split is void
    // whatever the arithmetic produced.
    let stalled = achieved.is_some_and(|a| a < v.spec.speed_rad_s * 0.5);
    if stalled {
        summary.push((
            "stalled".to_string(),
            "achieved under half the commanded speed — the split is void".to_string(),
        ));
    }

    let note = match friction {
        Some(_) if no_torque => "ran, but the motor reported no torque".to_string(),
        Some(f) if stalled => format!("{f:.3} N·m, but the traverse stalled"),
        Some(f) => format!("kinetic friction {f:.3} N·m"),
        None => "no friction estimate".to_string(),
    };

    (
        CharacterizeData {
            x_label: "position [rad]".to_string(),
            y_label: "torque [N·m]".to_string(),
            x: x.clone(),
            y,
            series: "kinetic friction".to_string(),
            x2: (!load_series.is_empty()).then_some(x),
            y2: (!load_series.is_empty()).then_some(load_series),
            series2: Some("load".to_string()),
            summary,
        },
        note,
    )
}

fn breakaway_data(b: misa_sysid::Breakaway) -> (CharacterizeData, String) {
    let x: Vec<f32> = b.points.iter().map(|p| p.t_s).collect();
    let y: Vec<f32> = b.points.iter().map(|p| p.cmd).collect();
    let measured: Vec<f32> = b.points.iter().map(|p| p.torque_nm).collect();
    let mut summary = Vec::new();
    // A ramp that began while the shaft was still moving measured drag, not
    // stiction. Carried on the figure itself rather than left in a neighbouring
    // tile, because the figure travels: a batch list quotes the leading summary
    // entry and nothing else, so an unrested 0.669 N·m sat in it looking exactly
    // like a good measurement (2026-08-07).
    let caveat = if b.rested { "" } else { " (not at rest)" };
    let note = match b.breakaway_torque_nm {
        Some(t) => {
            summary.push((
                "breakaway torque".to_string(),
                format!("{t:.3} N·m{caveat}"),
            ));
            if let Some(p) = b.breakaway_position_rad {
                summary.push(("at position".to_string(), format!("{p:+.4} rad")));
            }
            format!("broke loose at {t:.3} N·m")
        }
        None => {
            // Saying "0 N·m" here would read as a frictionless joint. It
            // means the ramp ran out before anything moved.
            summary.push((
                "breakaway torque".to_string(),
                format!("not reached below {:.3} N·m{caveat}", b.spec.max_torque_nm),
            ));
            "did not break loose within the torque ceiling".to_string()
        }
    };
    summary.push((
        "rested first".to_string(),
        if b.rested { "yes" } else { "no" }.to_string(),
    ));
    (
        CharacterizeData {
            x_label: "t [s]".to_string(),
            y_label: "torque [N·m]".to_string(),
            x: x.clone(),
            y,
            series: "commanded".to_string(),
            x2: Some(x),
            y2: Some(measured),
            series2: Some("measured".to_string()),
            summary,
        },
        note,
    )
}

/// Stiction against angle, one point per position, both directions.
///
/// The two series are the two ramp directions rather than a curve and its
/// smoothing: what the run measures is their *separation* at each angle, and
/// plotting a single averaged line would erase it — the same reason a load map
/// keeps its two legs apart.
fn breakaway_map_data(m: misa_sysid::BreakawayMap) -> (CharacterizeData, String) {
    // Only the positions that actually broke loose in that direction. A
    // position that never moved has no torque to plot, and plotting the ramp
    // ceiling there would read as a measurement of the ceiling.
    let series_of = |pick: fn(&misa_sysid::BreakawayMapPoint) -> Option<f32>| {
        let pts: Vec<(f32, f32)> = m
            .points
            .iter()
            .filter_map(|p| pick(p).map(|v| (p.position_rad, v)))
            .collect();
        (
            pts.iter().map(|&(x, _)| x).collect::<Vec<_>>(),
            pts.iter().map(|&(_, y)| y).collect::<Vec<_>>(),
        )
    };
    let (x, y) = series_of(|p| p.positive_nm);
    let (x2, y2) = series_of(|p| p.negative_nm);

    let stictions: Vec<f32> = m.points.iter().filter_map(|p| p.stiction_nm()).collect();
    let mut summary = Vec::new();

    // Same reasoning as the single breakaway: the caveat rides on the figure,
    // because the figure is what a batch list quotes.
    let unrested = m.points.iter().filter(|p| !p.rested).count();
    let caveat = if unrested == 0 {
        String::new()
    } else {
        format!(" ({unrested} of {} not at rest)", m.points.len())
    };
    match m.mean_stiction_nm() {
        Some(mean) => {
            summary.push((
                "mean stiction".to_string(),
                format!("{mean:.3} N·m{caveat}"),
            ));
            // The point of sweeping angle at all: a spread comparable to the
            // mean means one figure cannot describe the joint, and that is the
            // finding rather than a caveat about it.
            if let (Some(lo), Some(hi)) = (
                stictions.iter().copied().reduce(f32::min),
                stictions.iter().copied().reduce(f32::max),
            ) {
                summary.push((
                    "stiction over angle".to_string(),
                    format!("{lo:.3} – {hi:.3} N·m"),
                ));
                if mean.abs() > 0.0 {
                    summary.push((
                        "angle dependence".to_string(),
                        format!("{:.0}% of the mean", 100.0 * (hi - lo) / mean.abs()),
                    ));
                }
            }
        }
        None => {
            summary.push((
                "mean stiction".to_string(),
                if m.points.is_empty() {
                    "the run recorded nothing".to_string()
                } else if m.spec.both_directions {
                    "no position broke loose in both directions".to_string()
                } else {
                    // Stiction is half the spread between directions, so one
                    // direction cannot yield it however well that ramp went.
                    "needs both directions".to_string()
                },
            ));
        }
    }

    summary.push((
        "peak static load".to_string(),
        match m.peak_static_load_nm() {
            Some(l) => format!("{l:+.3} N·m"),
            None => "needs both directions".to_string(),
        },
    ));
    summary.push(("positions".to_string(), m.points.len().to_string()));


    let note = match m.mean_stiction_nm() {
        Some(mean) => format!(
            "{} positions, mean stiction {mean:.3} N·m",
            m.points.len()
        ),
        None => format!("{} positions, no stiction figure", m.points.len()),
    };

    (
        CharacterizeData {
            x_label: "position [rad]".to_string(),
            y_label: "breakaway torque [N·m]".to_string(),
            x,
            y,
            series: "positive".to_string(),
            series2: (!y2.is_empty()).then(|| "negative".to_string()),
            x2: (!x2.is_empty()).then_some(x2),
            y2: (!y2.is_empty()).then_some(y2),
            summary,
        },
        note,
    )
}

/// Leash gains for a Kt sweep of `max_torque_nm`, stiffened if the requested
/// ones would push the shaft out of the safety envelope's position window.
///
/// Returns `(kp, kd)`. `kp` is only ever raised, never lowered — a caller
/// asking for a stiffer hold than the window needs is entitled to it.
///
/// `kp == 0.0` passes through untouched: it means "the shaft is already
/// restrained, do not hold it", which is a claim about the rig rather than an
/// omission to correct.
fn kt_leash(max_torque_nm: f32, kp: f32, kd: f32, position_window_rad: f32) -> (f32, f32) {
    let allowance = position_window_rad * KT_LEASH_DEFLECTION;
    if kp <= 0.0 || !allowance.is_finite() || allowance <= 0.0 {
        return (kp, kd);
    }
    let needed = max_torque_nm.abs() / allowance;
    if needed <= kp {
        return (kp, kd);
    }
    // Damping raised in proportion to stiffness rather than to its square root:
    // the critical value needs an inertia nobody here knows, and erring
    // over-damped only costs settling time, while erring under puts an
    // oscillation inside the dwell the fit assumes is steady.
    (needed, kd * needed / kp)
}

/// Torque against current, plus the fitted constant for the UI to apply.
///
/// Returns the Kt separately from the summary rather than only as a display
/// string, because the caller does arithmetic with it — the whole point of the
/// run is that the operator no longer types the number in.
fn kt_data(k: misa_sysid::KtSweep) -> (CharacterizeData, String, Option<f32>) {
    let x: Vec<f32> = k.points.iter().map(|p| p.current_a).collect();
    let y: Vec<f32> = k.points.iter().map(|p| p.torque_nm).collect();
    let commanded: Vec<f32> = k.points.iter().map(|p| p.cmd).collect();

    let fitted = k.kt_nm_per_a();
    let mut summary = Vec::new();
    let note = match fitted {
        Some(kt) => {
            summary.push(("Kt (fit)".to_string(), format!("{kt:.4} N·m/A")));
            if let Some(p) = k.pointwise_kt_nm_per_a() {
                // Shown beside the fit because the two disagreeing is the
                // signal that the relation is not a line through the origin —
                // saturation, or a torque offset the fit absorbed.
                summary.push(("Kt (pointwise)".to_string(), format!("{p:.4} N·m/A")));
            }
            if let Some(r2) = k.r_squared() {
                summary.push(("fit R²".to_string(), format!("{r2:.4}")));
            }
            format!("Kt {kt:.4} N·m/A")
        }
        None => {
            // Two different failures land here and they need different fixes,
            // so neither is described as the other: no current at all means
            // the driver never reported it, no torque means the firmware is
            // the one reporting zero.
            let no_current = k.points.iter().all(|p| !p.current_a.is_finite());
            let reason = if k.points.is_empty() {
                "the run recorded nothing"
            } else if no_current {
                "this driver reports no current, so there is nothing to fit torque against"
            } else {
                "torque and current did not vary enough to fit a slope"
            };
            summary.push(("Kt".to_string(), format!("not measured — {reason}")));
            format!("no Kt: {reason}")
        }
    };

    (
        CharacterizeData {
            x_label: "current [A]".to_string(),
            y_label: "torque [N·m]".to_string(),
            x: x.clone(),
            y,
            series: "measured".to_string(),
            // The commanded level on the same axes shows where the firmware's
            // own scaling sits relative to the measurement.
            x2: Some(x),
            y2: Some(commanded),
            series2: Some("commanded".to_string()),
            summary,
        },
        note,
        fitted,
    )
}

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

#[cfg(test)]
mod tests {
    use super::*;

    /// The failure this rule exists for, as it happened: an RS-04 with the
    /// ceiling at 12 N·m, a sweep asking for 60% of it, and the stock leash.
    #[test]
    fn a_kt_sweep_too_big_for_its_leash_is_held_inside_the_position_window() {
        let window = 0.5;

        // What shipped: kp 8 against 7.2 N·m settles at 0.9 rad, well outside a
        // ±0.5 rad window, so the run died at its first level with one point.
        assert!(7.2 / 8.0 > window, "the case under test must actually escape");

        let (kp, kd) = kt_leash(7.2, 8.0, 0.5, window);
        assert!(
            7.2 / kp <= window * KT_LEASH_DEFLECTION + f32::EPSILON,
            "deflection {} rad should fit the allowance",
            7.2 / kp
        );
        assert!(kd > 0.5, "damping should rise with stiffness, got {kd}");
    }

    #[test]
    fn a_leash_already_stiff_enough_is_left_alone() {
        // 1.2 N·m against kp 8 deflects 0.15 rad — inside 0.4 × 0.5 = 0.2.
        assert_eq!(kt_leash(1.2, 8.0, 0.5, 0.5), (8.0, 0.5));
        // And a caller who wants it stiffer than necessary keeps that.
        assert_eq!(kt_leash(1.2, 500.0, 5.0, 0.5), (500.0, 5.0));
    }

    /// `kp == 0` is the caller saying the shaft is restrained already. Raising
    /// it would energise a leash against a clamped joint.
    #[test]
    fn no_leash_stays_no_leash() {
        assert_eq!(kt_leash(7.2, 0.0, 0.0, 0.5), (0.0, 0.0));
    }

    /// A window of zero or a non-finite one would otherwise divide into an
    /// infinite kp and command a MIT frame full of garbage.
    #[test]
    fn a_degenerate_window_cannot_produce_an_infinite_gain() {
        assert_eq!(kt_leash(7.2, 8.0, 0.5, 0.0), (8.0, 0.5));
        assert_eq!(kt_leash(7.2, 8.0, 0.5, f32::NAN), (8.0, 0.5));
        assert!(kt_leash(7.2, 8.0, 0.5, f32::INFINITY).0.is_finite());
    }
}
