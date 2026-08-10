//! A live connection to several motors on one wire.
//!
//! The single-motor [`Session`](crate::Session) is untouched by this: the five
//! existing GUI tabs ride on its wire contract, and `doc/handover.md` section 5
//! records two black screens caused by breaking that contract. This is a second
//! shape alongside it, not a generalisation of it.
//!
//! One worker owns the [`SharedCanBus`](crate::multi::SharedCanBus) and every
//! motor on it, and visits them in turn. That sequencing is not a simplification
//! — it is what makes sharing the wire safe, because a vendor bus trait sends
//! and receives as separate calls and only one exchange may be in flight (see
//! [`crate::multi`]).
//!
//! # Safety with several motors
//!
//! The three mechanisms of the single-motor session all apply, and all of them
//! act on **every** motor:
//!
//! 1. [`MultiSession::stop`] sets a flag the worker checks before each motor's
//!    transaction, and disables all of them. It is not a queued command.
//! 2. The watchdog disables all of them when the UI stops calling
//!    [`MultiSession::heartbeat`] while streaming.
//! 3. Dropping the session stops every motor and waits for the worker.
//!
//! None of that makes a motor latch de-energised — see `doc/handover.md`
//! section 2. With several motors the wrong assumption is more expensive, not
//! less.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, Ordering};
use std::sync::mpsc::{self, Receiver, Sender, SyncSender, TryRecvError, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use misa_actuator::realtime::{sleep_until, TimerResolutionGuard};
use misa_actuator::{Actuator, Error as ActuatorError};
use serde::{Deserialize, Serialize};

use crate::multi::{build_multi, MotorSpec, MultiConfig};
use crate::protocol::{
    ControlMode, LogLevel, Setpoint, StopReason, DEFAULT_RATE_HZ, DEFAULT_WATCHDOG, MAX_RATE_HZ,
    MIN_RATE_HZ, TELEMETRY_INTERVAL,
};

/// How many snapshots may be in flight before the worker drops them.
///
/// Smaller than the single-motor session's four, because there is nothing to
/// plot here: a monitoring table wants the newest reading, and an older one is
/// worth nothing to it.
const SNAPSHOT_DEPTH: usize = 2;

/// Polling cadence for a motor that is not being driven.
const IDLE_POLL: Duration = Duration::from_millis(50);

/// Consecutive failures before a motor is left alone.
///
/// Per motor, not per session: one dead motor on a four-motor wire must not
/// take the other three down with it.
const CONSECUTIVE_FAULT_LIMIT: u32 = 20;

// ---------------------------------------------------------------------------
// Wire types
// ---------------------------------------------------------------------------

/// One motor's latest reading.
///
/// `NaN` where the driver does not report the quantity, which several do not —
/// see `doc/handover.md` section 4 on RobStride returning no temperature
/// outside MIT mode. A UI must render that as "not reported" rather than as
/// zero.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MotorReading {
    pub motor_id: u8,
    /// The driver family, so a mixed wire can be labelled.
    pub driver: crate::factory::DriverKind,
    pub label: String,
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    pub torque_nm: f32,
    pub current_a: f32,
    pub temperature_c: f32,
    /// Whether the worker believes this motor is energised.
    pub enabled: bool,
    /// Consecutive failed transactions. Non-zero means the numbers above are
    /// stale, which is worth showing rather than hiding behind a last-known
    /// value.
    pub misses: u32,
    /// The most recent failure, if the last transaction failed.
    pub error: Option<String>,
}

/// Every motor's latest reading, taken in one pass.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MultiSnapshot {
    /// Seconds since the session opened.
    pub t_s: f64,
    pub motors: Vec<MotorReading>,
    /// Passes completed per second, measured. A pass visits every motor, so
    /// this is the per-motor update rate and it falls as motors are added.
    pub achieved_rate_hz: f32,
    /// Snapshots dropped because the consumer was behind.
    pub dropped: u64,
}

/// A one-off instruction to the multi-motor worker.
///
/// Both rename attributes, for the reason [`crate::protocol::Command`] gives:
/// `rename_all` covers variant names and `rename_all_fields` covers the fields
/// inside them, and omitting the second sends `motor_id` in snake_case while
/// everything else the UI sees is camelCase.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case", rename_all_fields = "camelCase", tag = "kind")]
pub enum MultiCommand {
    /// Energise every motor. The bulk operation the operator actually wants,
    /// and deliberately one command rather than N: a partial enable issued as N
    /// commands leaves no single thing to report on.
    EnableAll,
    /// De-energise every motor. Always attempted on every motor, even if one
    /// fails.
    DisableAll,
    Enable {
        motor_id: u8,
    },
    Disable {
        motor_id: u8,
    },
    /// Anchor this motor's current position as zero.
    SetZero {
        motor_id: u8,
    },
    /// Start or stop pushing setpoints. Readings continue either way.
    SetStreaming {
        on: bool,
    },
    Shutdown,
}

/// What the worker reports besides snapshots.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case", rename_all_fields = "camelCase", tag = "event")]
pub enum MultiEvent {
    /// A motor's energised state changed.
    MotorState { motor_id: u8, enabled: bool },
    Log { level: LogLevel, message: String },
    Stopped { reason: StopReason },
}

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

const STOP_NONE: u8 = 0;
const STOP_USER: u8 = 1;
const STOP_WATCHDOG: u8 = 2;

/// How often the watchdog re-checks.
const WATCHDOG_POLL: Duration = Duration::from_millis(50);

struct Shared {
    started: Instant,
    /// One setpoint per motor, in the same order as the configuration.
    ///
    /// Overwritten rather than queued, for the reason the single-motor session
    /// gives: dragging a slider must not pile up commands.
    setpoints: Mutex<Vec<Setpoint>>,
    streaming: AtomicBool,
    stop: AtomicBool,
    stop_reason: AtomicU8,
    rate_millihz: AtomicU32,
    last_heartbeat_ms: AtomicU64,
    watchdog_ms: AtomicU64,
    alive: AtomicBool,
}

impl Shared {
    fn now_ms(&self) -> u64 {
        self.started.elapsed().as_millis() as u64
    }

    fn beat(&self) {
        self.last_heartbeat_ms
            .store(self.now_ms(), Ordering::Release);
    }

    fn request_stop(&self, reason: u8) {
        self.stop_reason.store(reason, Ordering::Release);
        self.stop.store(true, Ordering::Release);
    }

    fn take_stop(&self) -> Option<StopReason> {
        if !self.stop.swap(false, Ordering::AcqRel) {
            return None;
        }
        Some(match self.stop_reason.swap(STOP_NONE, Ordering::AcqRel) {
            STOP_WATCHDOG => StopReason::Watchdog,
            _ => StopReason::UserRequest,
        })
    }

    fn watchdog_tripped(&self) -> bool {
        if !self.streaming.load(Ordering::Acquire) {
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

fn watchdog_loop(shared: Arc<Shared>) {
    while shared.alive.load(Ordering::Acquire) {
        if shared.watchdog_tripped() && !shared.stop.load(Ordering::Acquire) {
            log::warn!("multi watchdog: no heartbeat from the UI — stopping every motor");
            shared.request_stop(STOP_WATCHDOG);
        }
        std::thread::sleep(WATCHDOG_POLL);
    }
}

// ---------------------------------------------------------------------------
// Session handle
// ---------------------------------------------------------------------------

/// A connected set of motors on one wire.
pub struct MultiSession {
    shared: Arc<Shared>,
    commands: Sender<MultiCommand>,
    events: Receiver<MultiEvent>,
    snapshots: Receiver<MultiSnapshot>,
    join: Option<JoinHandle<()>>,
    watchdog: Option<JoinHandle<()>>,
    specs: Vec<MotorSpec>,
    description: String,
}

impl MultiSession {
    /// Open the wire, bind every motor and start the worker.
    pub fn connect(cfg: &MultiConfig) -> Result<Self> {
        Self::connect_with(cfg, DEFAULT_RATE_HZ, Some(DEFAULT_WATCHDOG))
    }

    pub fn connect_with(
        cfg: &MultiConfig,
        rate_hz: f32,
        watchdog: Option<Duration>,
    ) -> Result<Self> {
        // On the calling thread, so a bad interface or a mistyped model is an
        // ordinary error return rather than an event some time later.
        let motors = build_multi(cfg).context("failed to open the motors")?;
        let specs: Vec<MotorSpec> = motors.iter().map(|m| m.spec.clone()).collect();
        let description = describe(cfg, &specs);

        let shared = Arc::new(Shared {
            started: Instant::now(),
            setpoints: Mutex::new(vec![Setpoint::default(); motors.len()]),
            streaming: AtomicBool::new(false),
            stop: AtomicBool::new(false),
            stop_reason: AtomicU8::new(STOP_NONE),
            rate_millihz: AtomicU32::new(rate_to_millihz(rate_hz)),
            last_heartbeat_ms: AtomicU64::new(0),
            watchdog_ms: AtomicU64::new(watchdog.map_or(0, |d| d.as_millis() as u64)),
            alive: AtomicBool::new(true),
        });
        shared.beat();

        let (cmd_tx, cmd_rx) = mpsc::channel();
        let (evt_tx, evt_rx) = mpsc::channel();
        let (snap_tx, snap_rx) = mpsc::sync_channel(SNAPSHOT_DEPTH);

        let worker = Worker::new(motors, shared.clone(), cmd_rx, evt_tx, snap_tx);
        let join = std::thread::Builder::new()
            .name("motors".to_string())
            .spawn(move || worker.run())
            .context("failed to spawn the multi-motor worker thread")?;

        // Its own thread for the same reason as the single-motor watchdog: a
        // long transaction must not be able to postpone the check.
        let wd_shared = shared.clone();
        let watchdog = std::thread::Builder::new()
            .name("motors-watchdog".to_string())
            .spawn(move || watchdog_loop(wd_shared))
            .context("failed to spawn the watchdog thread")?;

        Ok(Self {
            shared,
            commands: cmd_tx,
            events: evt_rx,
            snapshots: snap_rx,
            join: Some(join),
            watchdog: Some(watchdog),
            specs,
            description,
        })
    }

    pub fn description(&self) -> &str {
        &self.description
    }

    /// The motors, in the order their readings arrive.
    pub fn specs(&self) -> &[MotorSpec] {
        &self.specs
    }

    pub fn send(&self, cmd: MultiCommand) -> Result<()> {
        self.commands
            .send(cmd)
            .map_err(|_| anyhow::anyhow!("the motor worker has stopped"))
    }

    /// Overwrite one motor's setpoint. Cheap enough to call every frame.
    ///
    /// An unknown id is an error rather than a silent no-op: a UI addressing a
    /// motor that is not there has a bug worth surfacing.
    pub fn set_setpoint(&self, motor_id: u8, sp: Setpoint) -> Result<()> {
        let index = self
            .specs
            .iter()
            .position(|s| s.motor_id == motor_id)
            .with_context(|| format!("no motor with id {motor_id} in this session"))?;
        if let Ok(mut guard) = self.shared.setpoints.lock() {
            guard[index] = sp;
        }
        Ok(())
    }

    pub fn setpoint(&self, motor_id: u8) -> Option<Setpoint> {
        let index = self.specs.iter().position(|s| s.motor_id == motor_id)?;
        self.shared.setpoints.lock().ok().map(|g| g[index])
    }

    /// **Stop every motor now.**
    ///
    /// A flag, not a command, so it cannot queue behind a pass that is already
    /// under way.
    pub fn stop(&self) {
        self.shared.request_stop(STOP_USER);
    }

    pub fn heartbeat(&self) {
        self.shared.beat();
    }

    pub fn set_watchdog(&self, watchdog: Option<Duration>) {
        self.shared
            .watchdog_ms
            .store(watchdog.map_or(0, |d| d.as_millis() as u64), Ordering::Relaxed);
    }

    pub fn set_rate(&self, hz: f32) {
        self.shared
            .rate_millihz
            .store(rate_to_millihz(hz), Ordering::Relaxed);
    }

    pub fn is_streaming(&self) -> bool {
        self.shared.streaming.load(Ordering::Acquire)
    }

    pub fn poll_events(&self) -> Vec<MultiEvent> {
        self.events.try_iter().collect()
    }

    /// The newest snapshot, discarding any older ones still queued.
    ///
    /// Newest-wins because there is nothing to plot: an older reading of the
    /// same motor has no value once a newer one exists.
    pub fn latest_snapshot(&self) -> Option<MultiSnapshot> {
        self.snapshots.try_iter().last()
    }

    /// Stop every motor and shut the worker down, blocking until it confirms.
    pub fn shutdown(&mut self) {
        // Flag first, command second: if the queue is backed up, the flag is
        // what actually stops the motors.
        self.stop();
        let _ = self.commands.send(MultiCommand::Shutdown);
        if let Some(join) = self.join.take() {
            if join.join().is_err() {
                log::error!("multi-motor worker panicked; motors may still be energised");
            }
        }
        // After the worker, so a worker still winding down keeps its protection.
        self.shared.alive.store(false, Ordering::Release);
        if let Some(w) = self.watchdog.take() {
            let _ = w.join();
        }
    }
}

impl Drop for MultiSession {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn rate_to_millihz(hz: f32) -> u32 {
    (hz.clamp(MIN_RATE_HZ, MAX_RATE_HZ) * 1000.0) as u32
}

fn describe(cfg: &MultiConfig, specs: &[MotorSpec]) -> String {
    let ids: Vec<String> = specs
        .iter()
        .map(|s| format!("{} {}", s.driver.as_str(), s.motor_id))
        .collect();
    let where_ = if cfg.interface.trim().is_empty() {
        "simulated".to_string()
    } else {
        cfg.interface.clone()
    };
    format!("{} motors on {}: {}", specs.len(), where_, ids.join(", "))
}

// ---------------------------------------------------------------------------
// Worker
// ---------------------------------------------------------------------------

/// Per-motor state the worker keeps alongside the driver.
struct MotorState {
    spec: MotorSpec,
    actuator: Box<dyn Actuator + Send>,
    enabled: bool,
    active_mode: Option<ControlMode>,
    faults: u32,
    last_error: Option<String>,
    last: misa_actuator::MotorFeedback,
    last_idle_poll: Instant,
}

struct Worker {
    motors: Vec<MotorState>,
    shared: Arc<Shared>,
    commands: Receiver<MultiCommand>,
    events: Sender<MultiEvent>,
    snapshots: SyncSender<MultiSnapshot>,
    dropped: u64,
    passes: u32,
    last_flush: Instant,
}

impl Worker {
    fn new(
        motors: Vec<crate::multi::BuiltMotor>,
        shared: Arc<Shared>,
        commands: Receiver<MultiCommand>,
        events: Sender<MultiEvent>,
        snapshots: SyncSender<MultiSnapshot>,
    ) -> Self {
        let now = Instant::now();
        Self {
            motors: motors
                .into_iter()
                .map(|m| MotorState {
                    spec: m.spec,
                    actuator: m.actuator,
                    enabled: false,
                    active_mode: None,
                    faults: 0,
                    last_error: None,
                    last: misa_actuator::MotorFeedback::zero(),
                    last_idle_poll: now,
                })
                .collect(),
            shared,
            commands,
            events,
            snapshots,
            dropped: 0,
            passes: 0,
            last_flush: now,
        }
    }

    fn run(mut self) {
        // Windows rounds sleeps up to the ~15.6 ms tick without this.
        let _timer = TimerResolutionGuard::acquire();

        let mut next_tick = Instant::now();
        let reason = 'outer: loop {
            loop {
                match self.commands.try_recv() {
                    Ok(MultiCommand::Shutdown) => break 'outer StopReason::Shutdown,
                    Ok(cmd) => self.handle(cmd),
                    Err(TryRecvError::Empty) => break,
                    // The handle was dropped: the UI is gone.
                    Err(TryRecvError::Disconnected) => break 'outer StopReason::Shutdown,
                }
            }

            if let Some(reason) = self.shared.take_stop() {
                self.halt(reason);
            }

            self.pass();
            self.flush_if_due();

            next_tick += self.shared.period();
            let now = Instant::now();
            if next_tick < now {
                next_tick = now;
            }
            sleep_until(next_tick);
        };

        self.finish(reason);
    }

    /// One visit to every motor.
    fn pass(&mut self) {
        let streaming = self.shared.streaming.load(Ordering::Acquire);
        let setpoints = self
            .shared
            .setpoints
            .lock()
            .map(|g| g.clone())
            .unwrap_or_default();

        for (i, m) in self.motors.iter_mut().enumerate() {
            // Checked per motor, not per pass: a stop must not wait for the
            // rest of the wire to be serviced first.
            if self.shared.stop.load(Ordering::Acquire) {
                break;
            }
            if m.faults >= CONSECUTIVE_FAULT_LIMIT {
                continue;
            }

            let sp = setpoints.get(i).copied().unwrap_or_default();
            let outcome = if streaming && m.enabled {
                if m.active_mode != Some(sp.mode) {
                    match m.actuator.set_run_mode(sp.mode.into()) {
                        Ok(()) => m.active_mode = Some(sp.mode),
                        Err(e) => {
                            note_fault(m, "set_run_mode", e, &self.events);
                            continue;
                        }
                    }
                }
                match sp.mode {
                    ControlMode::Position => {
                        m.actuator.set_position(sp.position_rad, sp.max_speed_rad_s)
                    }
                    ControlMode::Velocity => m.actuator.set_velocity(sp.velocity_rad_s),
                    ControlMode::Torque => m.actuator.set_torque(sp.torque_nm),
                    ControlMode::Mit => m.actuator.mit_control(
                        sp.position_rad,
                        sp.velocity_rad_s,
                        sp.kp,
                        sp.kd,
                        sp.torque_ff_nm,
                    ),
                }
            } else if m.last_idle_poll.elapsed() >= IDLE_POLL {
                m.last_idle_poll = Instant::now();
                // Reading is not always passive: on DAMIAO `measure` re-issues
                // the last control frame, so a monitoring table keeps a motor
                // energised. See doc/handover.md section 2.
                m.actuator.measure()
            } else {
                continue;
            };

            match outcome {
                Ok(fb) => {
                    m.faults = 0;
                    m.last_error = None;
                    m.last = fb;
                }
                Err(e) => note_fault(m, "transaction", e, &self.events),
            }
        }
        self.passes += 1;
    }

    fn handle(&mut self, cmd: MultiCommand) {
        match cmd {
            MultiCommand::EnableAll => {
                for i in 0..self.motors.len() {
                    self.enable_one(i);
                }
            }
            MultiCommand::DisableAll => self.disable_all(),
            MultiCommand::Enable { motor_id } => {
                if let Some(i) = self.index_of(motor_id) {
                    self.enable_one(i);
                }
            }
            MultiCommand::Disable { motor_id } => {
                if let Some(i) = self.index_of(motor_id) {
                    self.disable_one(i);
                }
            }
            MultiCommand::SetZero { motor_id } => {
                if let Some(i) = self.index_of(motor_id) {
                    let m = &mut self.motors[i];
                    match m.actuator.set_zero() {
                        Ok(()) => self.log(
                            LogLevel::Info,
                            format!("motor {motor_id}: zero anchored here"),
                        ),
                        Err(e) => self.log(
                            LogLevel::Error,
                            format!("motor {motor_id}: set_zero failed: {e}"),
                        ),
                    }
                }
            }
            MultiCommand::SetStreaming { on } => {
                self.shared.streaming.store(on, Ordering::Release);
                // Beat on the way in, so enabling streaming does not trip the
                // watchdog on the very next check because the last heartbeat
                // predates it.
                self.shared.beat();
                self.log(
                    LogLevel::Info,
                    if on {
                        "streaming setpoints to every enabled motor".to_string()
                    } else {
                        "stopped streaming".to_string()
                    },
                );
            }
            MultiCommand::Shutdown => unreachable!("handled in run()"),
        }
    }

    fn index_of(&self, motor_id: u8) -> Option<usize> {
        self.motors.iter().position(|m| m.spec.motor_id == motor_id)
    }

    fn enable_one(&mut self, i: usize) {
        let id = self.motors[i].spec.motor_id;
        match self.motors[i].actuator.enable() {
            Ok(fb) => {
                let m = &mut self.motors[i];
                m.enabled = true;
                m.faults = 0;
                m.last = fb;
                // Adopt the shaft's position as the target, so enabling does
                // not command a jump to whatever the form last held.
                if let Ok(mut sps) = self.shared.setpoints.lock() {
                    if let Some(sp) = sps.get_mut(i) {
                        sp.position_rad = fb.position_rad;
                    }
                }
                let _ = self.events.send(MultiEvent::MotorState {
                    motor_id: id,
                    enabled: true,
                });
                self.log(
                    LogLevel::Info,
                    format!("motor {id}: enabled at {:+.3} rad", fb.position_rad),
                );
            }
            Err(e) => self.log(LogLevel::Error, format!("motor {id}: enable failed: {e}")),
        }
    }

    fn disable_one(&mut self, i: usize) {
        let id = self.motors[i].spec.motor_id;
        match self.motors[i].actuator.disable() {
            Ok(()) => {
                self.motors[i].enabled = false;
                self.motors[i].active_mode = None;
                let _ = self.events.send(MultiEvent::MotorState {
                    motor_id: id,
                    enabled: false,
                });
            }
            Err(e) => self.log(LogLevel::Error, format!("motor {id}: disable failed: {e}")),
        }
    }

    /// Every motor, and keep going past a failure.
    ///
    /// Stopping at the first error would leave the remaining motors energised
    /// because of a fault in one of them.
    fn disable_all(&mut self) {
        for i in 0..self.motors.len() {
            self.disable_one(i);
        }
    }

    fn halt(&mut self, reason: StopReason) {
        self.shared.streaming.store(false, Ordering::Release);
        self.disable_all();
        self.log(LogLevel::Warn, format!("stopped every motor ({reason:?})"));
    }

    fn flush_if_due(&mut self) {
        if self.last_flush.elapsed() < TELEMETRY_INTERVAL {
            return;
        }
        let elapsed = self.last_flush.elapsed().as_secs_f32().max(1e-6);
        let snapshot = MultiSnapshot {
            t_s: self.shared.started.elapsed().as_secs_f64(),
            motors: self
                .motors
                .iter()
                .map(|m| MotorReading {
                    motor_id: m.spec.motor_id,
                    driver: m.spec.driver,
                    label: m.spec.label(),
                    position_rad: m.last.position_rad,
                    velocity_rad_s: m.last.velocity_rad_per_s,
                    torque_nm: m.last.torque_nm,
                    current_a: m.last.current_a,
                    temperature_c: m.last.temperature_c,
                    enabled: m.enabled,
                    misses: m.faults,
                    error: m.last_error.clone(),
                })
                .collect(),
            achieved_rate_hz: self.passes as f32 / elapsed,
            dropped: self.dropped,
        };
        self.passes = 0;
        self.last_flush = Instant::now();

        match self.snapshots.try_send(snapshot) {
            Ok(()) => self.dropped = 0,
            // Safe to lose: the next pass produces a fresher one. Counted so
            // the UI can say the numbers are behind rather than imply they are
            // current.
            Err(TrySendError::Full(_)) => self.dropped += 1,
            Err(TrySendError::Disconnected(_)) => {}
        }
    }

    fn log(&self, level: LogLevel, message: String) {
        match level {
            LogLevel::Info => log::info!("multi: {message}"),
            LogLevel::Warn => log::warn!("multi: {message}"),
            LogLevel::Error => log::error!("multi: {message}"),
        }
        let _ = self.events.send(MultiEvent::Log { level, message });
    }

    /// Always leaves every motor stopped, whatever ended the loop.
    fn finish(mut self, reason: StopReason) {
        self.shared.streaming.store(false, Ordering::Release);
        for i in 0..self.motors.len() {
            let id = self.motors[i].spec.motor_id;
            if let Err(e) = self.motors[i].actuator.disable() {
                log::warn!("multi: disable of motor {id} on shutdown failed: {e}");
            }
            self.motors[i].enabled = false;
        }
        let _ = self.events.send(MultiEvent::Stopped { reason });
        log::info!("multi worker: stopped ({reason:?})");
    }
}

/// Record a failed transaction against one motor.
///
/// A free function because it borrows one motor mutably while the event sender
/// is borrowed from the worker.
fn note_fault(
    m: &mut MotorState,
    what: &str,
    e: ActuatorError,
    events: &Sender<MultiEvent>,
) {
    m.faults += 1;
    m.last_error = Some(e.to_string());
    // Timeouts are routine on a shared wire; only speak up once a run of them
    // suggests the motor has actually gone.
    let level = if matches!(e, ActuatorError::Timeout { .. }) && m.faults < 5 {
        LogLevel::Warn
    } else {
        LogLevel::Error
    };
    if m.faults == CONSECUTIVE_FAULT_LIMIT {
        let message = format!(
            "motor {}: {what} failed {} times in a row; leaving it alone. The other \
             motors are unaffected.",
            m.spec.motor_id, m.faults
        );
        log::error!("multi: {message}");
        let _ = events.send(MultiEvent::Log {
            level: LogLevel::Error,
            message,
        });
    } else if m.faults < 5 {
        let message = format!("motor {}: {what} failed: {e}", m.spec.motor_id);
        log::debug!("multi: {message}");
        let _ = events.send(MultiEvent::Log { level, message });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::factory::{BusKind, DriverKind};

    fn sim_cfg(ids: &[u8]) -> MultiConfig {
        MultiConfig {
            interface: String::new(),
            bus: BusKind::Can,
            timeout_ms: 100,
            motors: ids
                .iter()
                .map(|&id| MotorSpec {
                    driver: DriverKind::Sim,
                    motor_id: id,
                    model: "ideal".into(),
                    host_id: 0,
                    kt: 0.0,
                    gear_ratio: 0.0,
                    name: String::new(),
                })
                .collect(),
        }
    }

    /// Waits for a snapshot rather than sleeping a fixed time, so the test does
    /// not depend on the scheduler being prompt.
    fn wait_for_snapshot(s: &MultiSession) -> MultiSnapshot {
        for _ in 0..200 {
            if let Some(snap) = s.latest_snapshot() {
                return snap;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        panic!("no snapshot within 2 s");
    }

    #[test]
    fn every_motor_is_reported_in_one_snapshot() {
        let s = MultiSession::connect(&sim_cfg(&[1, 2, 3])).unwrap();
        let snap = wait_for_snapshot(&s);
        assert_eq!(snap.motors.len(), 3);
        let ids: Vec<u8> = snap.motors.iter().map(|m| m.motor_id).collect();
        assert_eq!(ids, vec![1, 2, 3], "order follows the configuration");
    }

    #[test]
    fn enable_all_energises_every_motor() {
        let s = MultiSession::connect(&sim_cfg(&[1, 2])).unwrap();
        s.send(MultiCommand::EnableAll).unwrap();
        for _ in 0..200 {
            let snap = wait_for_snapshot(&s);
            if snap.motors.iter().all(|m| m.enabled) {
                return;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        panic!("not every motor reported enabled");
    }

    /// The bulk stop is the one that matters most with several motors, and it
    /// has to reach all of them.
    #[test]
    fn stop_disables_every_motor() {
        let s = MultiSession::connect(&sim_cfg(&[1, 2, 3])).unwrap();
        s.send(MultiCommand::EnableAll).unwrap();
        for _ in 0..200 {
            if wait_for_snapshot(&s).motors.iter().all(|m| m.enabled) {
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        s.stop();
        for _ in 0..200 {
            let snap = wait_for_snapshot(&s);
            if snap.motors.iter().all(|m| !m.enabled) {
                assert!(!s.is_streaming(), "stop also stops streaming");
                return;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        panic!("a motor was still enabled after stop");
    }

    /// Addressing a motor that is not in the session is a caller bug, so it is
    /// reported rather than dropped.
    #[test]
    fn an_unknown_motor_id_is_rejected() {
        let s = MultiSession::connect(&sim_cfg(&[1, 2])).unwrap();
        assert!(s.set_setpoint(1, Setpoint::default()).is_ok());
        let err = s.set_setpoint(9, Setpoint::default()).unwrap_err();
        assert!(err.to_string().contains("no motor with id 9"), "{err}");
    }

    #[test]
    fn setpoints_are_per_motor() {
        let s = MultiSession::connect(&sim_cfg(&[1, 2])).unwrap();
        let sp = Setpoint {
            position_rad: 0.5,
            ..Setpoint::default()
        };
        s.set_setpoint(2, sp).unwrap();
        assert_eq!(s.setpoint(2).unwrap().position_rad, 0.5);
        assert_eq!(
            s.setpoint(1).unwrap().position_rad,
            0.0,
            "one motor's target must not become another's"
        );
    }

    /// A pass visits every motor, so adding motors divides the per-motor rate.
    /// The number reported has to be the per-motor one, or a UI would claim a
    /// freshness it does not have.
    #[test]
    fn the_reported_rate_is_per_motor() {
        let s = MultiSession::connect(&sim_cfg(&[1, 2, 3, 4])).unwrap();
        // Skip the first snapshot: it covers a partially elapsed interval.
        let _ = wait_for_snapshot(&s);
        std::thread::sleep(Duration::from_millis(200));
        let snap = wait_for_snapshot(&s);
        assert!(
            snap.achieved_rate_hz > 0.0,
            "rate was {}",
            snap.achieved_rate_hz
        );
        assert!(
            snap.achieved_rate_hz <= MAX_RATE_HZ,
            "rate was {}",
            snap.achieved_rate_hz
        );
    }
}
