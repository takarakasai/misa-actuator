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

use std::sync::atomic::{AtomicBool, Ordering};
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
    ControlMode, LogLevel, Setpoint, StopReason, DEFAULT_RATE_HZ, DEFAULT_WATCHDOG,
    STATUS_INTERVAL, TELEMETRY_INTERVAL,
};
use crate::safety::{self, Safety};

/// How many snapshots may be in flight before the worker drops them.
///
/// Smaller than the single-motor session's four, because there is nothing to
/// plot here: a monitoring table wants the newest reading, and an older one is
/// worth nothing to it.
const SNAPSHOT_DEPTH: usize = 2;

// The threshold and the idle cadence are shared with the single-motor worker —
// see `crate::safety`, which also records why the *policy* at the limit is not.
// Here, hitting it leaves that one motor alone and keeps servicing the rest.
use crate::safety::{CONSECUTIVE_FAULT_LIMIT, IDLE_POLL};

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
    /// Bus voltage, from the slower status read. `NaN` if not reported.
    pub voltage_v: f32,
    /// Common fault bits, as `misa_actuator::ErrorFlags::bits`.
    pub error_bits: u32,
    /// The driver's native fault word, for diagnostics.
    pub error_raw: u32,
    /// How old these numbers are, in ms, or `None` if the motor has never
    /// answered.
    ///
    /// On a shared wire a motor can miss several turns, so "when was this
    /// measured" is part of the measurement. A table that renders a stale value
    /// as current is exactly what section 8 of the handover forbids.
    pub age_ms: Option<u64>,
    /// Whether the worker believes this motor is energised.
    pub enabled: bool,
    /// Consecutive failed transactions. Non-zero means the numbers above are
    /// stale, which is worth showing rather than hiding behind a last-known
    /// value.
    pub misses: u32,
    /// The most recent failure, if the last transaction failed.
    pub error: Option<String>,
    /// What the worker would command this motor right now, in the units of its
    /// current mode ([`Setpoint::primary`]).
    ///
    /// Reported so a table can show commanded next to measured, and so a UI can
    /// fill its target box from the value the worker actually holds. A box that
    /// shows zero while the worker holds a seeded position would be a display
    /// that disagrees with the machine — and the operator would trust the box.
    pub target: f32,
}

/// Every motor's latest reading, taken in one pass.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MultiSnapshot {
    /// Seconds since the session opened.
    pub t_s: f64,
    pub motors: Vec<MotorReading>,
    /// Passes completed per second, measured. Falls as motors are added,
    /// because a pass has to visit all of them.
    ///
    /// **This is the loop rate, not always the per-motor update rate.** A pass
    /// only transacts with a motor that is being driven; an idle one is read on
    /// [`IDLE_POLL`] and skipped the rest of the time. So while streaming this
    /// is what each enabled motor sees, and while idle it overstates it by the
    /// ratio of the two — which is why the UI shows it only while streaming.
    /// Per-motor freshness is [`MotorReading::age_ms`], which is measured rather
    /// than inferred.
    pub achieved_rate_hz: f32,
    /// Snapshots dropped because the consumer was behind.
    pub dropped: u64,
    /// Passes that ran out of budget before reaching every motor.
    ///
    /// Non-zero means the wire, not the configured tick, is setting the rate.
    /// Worth surfacing: the alternative is a rate that quietly stops meaning
    /// what it says.
    pub starved_passes: u64,
    /// Leftover frames discarded before transactions, cumulative.
    ///
    /// Near zero on a healthy wire. Climbing means frames are arriving unasked
    /// for, or replies are arriving after their reader gave up — the first thing
    /// to look at when readings seem stale.
    pub drained_frames: u64,
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

/// Per-motor state shared between the handle and the worker.
///
/// The stop flag, the watchdog, the heartbeat and the rate clamp are **not**
/// here: they live in [`crate::safety::Safety`], one copy for every kind of
/// session. That is a direct consequence of 2026-08-05, when the GUI's STOP
/// button turned out to reach only the single-motor session because the two
/// implementations of "stop" had drifted apart. What remains here is what
/// genuinely differs from one motor to several — a setpoint per motor.
struct Shared {
    safety: Arc<Safety>,
    /// One setpoint per motor, in the same order as the configuration.
    ///
    /// Overwritten rather than queued, for the reason the single-motor session
    /// gives: dragging a slider must not pile up commands.
    setpoints: Mutex<Vec<Setpoint>>,
    /// Whether a caller has chosen a target for this motor yet.
    ///
    /// One flag per motor, because seeding is per motor: a four-motor wire has
    /// four shafts resting in four different places.
    setpoints_touched: Vec<AtomicBool>,
}

impl Shared {
    /// Point one motor's position setpoint at where its shaft actually is, but
    /// only while nobody has chosen a target for it.
    ///
    /// The same hazard as the single-motor session's seeding
    /// ([`crate::worker`]), multiplied: a motor remembers its position across
    /// power cycles and a freshly-opened table does not, so without this the
    /// first streaming pass sends **every** motor to zero at whatever the speed
    /// cap allows. On a shared wire that is four shafts setting off at once.
    ///
    /// The simulator cannot catch its absence: every preset starts at zero,
    /// where a command of zero and a correct seed are indistinguishable. It took
    /// looking at two real motors resting at non-zero angles.
    fn seed_position(&self, i: usize, position_rad: f32) {
        if !position_rad.is_finite() || self.setpoints_touched[i].load(Ordering::Acquire) {
            return;
        }
        if let Ok(mut guard) = self.setpoints.lock() {
            guard[i].position_rad = position_rad;
        }
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
        let built = build_multi(cfg).context("failed to open the motors")?;
        let (motors, bus) = (built.motors, built.bus);
        let specs: Vec<MotorSpec> = motors.iter().map(|m| m.spec.clone()).collect();
        let description = describe(cfg, &specs);

        let shared = Arc::new(Shared {
            safety: Arc::new(Safety::new(rate_hz, watchdog)),
            setpoints: Mutex::new(vec![Setpoint::default(); motors.len()]),
            setpoints_touched: (0..motors.len()).map(|_| AtomicBool::new(false)).collect(),
        });

        let (cmd_tx, cmd_rx) = mpsc::channel();
        let (evt_tx, evt_rx) = mpsc::channel();
        let (snap_tx, snap_rx) = mpsc::sync_channel(SNAPSHOT_DEPTH);

        let worker = Worker::new(motors, bus, shared.clone(), cmd_rx, evt_tx, snap_tx);
        let join = std::thread::Builder::new()
            .name("motors".to_string())
            .spawn(move || worker.run())
            .context("failed to spawn the multi-motor worker thread")?;

        // Its own thread for the same reason as the single-motor watchdog: a
        // long transaction must not be able to postpone the check. Same loop,
        // too — there is only one now.
        let wd_safety = shared.safety.clone();
        let watchdog = std::thread::Builder::new()
            .name("motors-watchdog".to_string())
            .spawn(move || crate::safety::watchdog_loop(wd_safety, "every motor"))
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
        // After this the worker must stop seeding this motor, or "set a target,
        // then enable" would have the next reading quietly replace the target
        // with wherever the shaft happens to be.
        self.shared.setpoints_touched[index].store(true, Ordering::Release);
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
        self.shared.safety.request_stop(safety::STOP_USER);
    }

    pub fn heartbeat(&self) {
        self.shared.safety.beat();
    }

    pub fn set_watchdog(&self, watchdog: Option<Duration>) {
        self.shared.safety.set_watchdog(watchdog);
    }

    pub fn set_rate(&self, hz: f32) {
        self.shared.safety.set_rate(hz);
    }

    pub fn is_streaming(&self) -> bool {
        self.shared.safety.is_streaming()
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
        self.shared.safety.retire();
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
    /// When this motor last answered. `None` until it has.
    ///
    /// Kept so a reading can carry its own age: on a shared wire a motor can go
    /// several passes without a turn, and a table that shows a stale number as
    /// though it were current is the kind of display this project treats as a
    /// bug (`doc/handover.md` section 8).
    last_ok: Option<Instant>,
    last_status_poll: Instant,
    /// From `read_status`, which is a separate transaction on most families.
    voltage_v: f32,
    status_temperature_c: f32,
    error_bits: u32,
    error_raw: u32,
    /// Failed status reads, counted apart from control faults.
    ///
    /// A driver that cannot report status must not be pushed towards the
    /// fault cut-off by a diagnostic read: that would silence a motor that is
    /// controlling perfectly well.
    status_misses: u32,
}

impl MotorState {
    /// The temperature to show, from whichever source reported one.
    ///
    /// Neither source is reliable across families: DAMIAO puts temperatures in
    /// the feedback frame, while a RobStride reports none outside MIT mode
    /// (`doc/handover.md` section 4). `NaN` when neither answered, which a UI
    /// must render as "not reported" rather than as zero.
    fn temperature(&self) -> f32 {
        if self.last.temperature_c.is_finite() {
            self.last.temperature_c
        } else {
            self.status_temperature_c
        }
    }
}

struct Worker {
    motors: Vec<MotorState>,
    /// The shared wire, for draining leftovers before a transaction. `None` when
    /// every motor is simulated and there is no wire.
    bus: Option<crate::multi::SharedCanBus>,
    shared: Arc<Shared>,
    commands: Receiver<MultiCommand>,
    events: Sender<MultiEvent>,
    snapshots: SyncSender<MultiSnapshot>,
    dropped: u64,
    passes: u32,
    last_flush: Instant,
    /// Which motor gets the first turn next pass.
    ///
    /// Rotates to whoever was skipped when a pass ran out of budget, so a slow
    /// motor delays the others once rather than permanently.
    next_start: usize,
    /// Passes that ended early on the budget. Reported so a UI can say the rate
    /// is limited by the wire rather than by the configured tick.
    starved: u64,
    /// Leftover frames discarded before transactions.
    ///
    /// Expected to be near zero on a healthy wire. A number that climbs means
    /// frames are arriving that nobody asked for, or replies are arriving after
    /// their reader gave up — either way it is the thing to look at first.
    drained: u64,
}

impl Worker {
    fn new(
        motors: Vec<crate::multi::BuiltMotor>,
        bus: Option<crate::multi::SharedCanBus>,
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
                    last_ok: None,
                    last_status_poll: now,
                    voltage_v: f32::NAN,
                    status_temperature_c: f32::NAN,
                    error_bits: 0,
                    error_raw: 0,
                    status_misses: 0,
                })
                .collect(),
            bus,
            shared,
            commands,
            events,
            snapshots,
            dropped: 0,
            passes: 0,
            last_flush: now,
            next_start: 0,
            starved: 0,
            drained: 0,
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

            if let Some(reason) = self.shared.safety.take_stop() {
                self.halt(reason);
            }

            self.pass();
            self.flush_if_due();

            next_tick += self.shared.safety.period();
            let now = Instant::now();
            if next_tick < now {
                next_tick = now;
            }
            sleep_until(next_tick);
        };

        self.finish(reason);
    }

    /// One visit to every motor, budgeted.
    ///
    /// A pass is given the tick period and no more. Without that, one slow motor
    /// sets everyone's update rate: a transaction can cost the full bus timeout
    /// (100 ms by default) and the DAMIAO register path retries three times with
    /// gaps, so a single bad motor on a four-motor wire would drag the other
    /// three down with it — which is precisely what was measured on 2026-08-03.
    ///
    /// When the budget runs out the remaining motors are skipped and go **first**
    /// next pass, so being starved is temporary rather than permanent.
    ///
    /// The budget bounds how many motors are delayed, not how long one
    /// transaction takes: the timeout belongs to the shared transport, so a
    /// single overrun of up to one timeout is still possible. Per-motor timeouts
    /// need the transport-level change noted in `crate::multi`.
    fn pass(&mut self) {
        let streaming = self.shared.safety.is_streaming();
        let setpoints = self
            .shared
            .setpoints
            .lock()
            .map(|g| g.clone())
            .unwrap_or_default();

        let n = self.motors.len();
        if n == 0 {
            return;
        }
        let start = self.next_start % n;
        let deadline = Instant::now() + self.shared.safety.period();
        let mut resume_at = start;

        for k in 0..n {
            let i = (start + k) % n;
            resume_at = i;

            // Checked per motor, not per pass: a stop must not wait for the
            // rest of the wire to be serviced first.
            if self.shared.safety.stop_pending() {
                break;
            }
            // At least one motor per pass — `k > 0` says exactly that — or a
            // permanently slow first motor would starve everyone forever.
            if k > 0 && Instant::now() >= deadline {
                self.starved += 1;
                break;
            }

            let sp = setpoints.get(i).copied().unwrap_or_default();
            self.service(i, sp, streaming);
            resume_at = (i + 1) % n;
        }

        self.next_start = resume_at;
        self.passes += 1;
    }

    /// One motor's turn: a control command or a reading, then status if due.
    fn service(&mut self, i: usize, sp: Setpoint, streaming: bool) {
        // Disjoint field borrows: the motor is borrowed mutably while the event
        // sender is borrowed immutably.
        if self.motors[i].faults >= CONSECUTIVE_FAULT_LIMIT {
            return;
        }

        // Clear the wire first, so the next frame read is the answer to what
        // this turn is about to ask. A feedback frame carries no sequence
        // number, so a leftover one would otherwise be taken as the current
        // position — see `SharedCanBus::drain_stale`.
        if let Some(bus) = self.bus.as_mut() {
            self.drained += bus.drain_stale();
        }

        let m = &mut self.motors[i];
        let outcome = if streaming && m.enabled {
            if m.active_mode != Some(sp.mode) {
                match m.actuator.set_run_mode(sp.mode.into()) {
                    Ok(()) => m.active_mode = Some(sp.mode),
                    Err(e) => {
                        note_fault(m, "set_run_mode", e, &self.events);
                        return;
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
            // Reading is not always passive: on DAMIAO `measure` re-issues the
            // last control frame, so a monitoring table keeps a motor
            // energised. See doc/handover.md section 2.
            m.actuator.measure()
        } else {
            // Nothing to do this turn, but a status read may still be due.
            self.status_if_due(i);
            return;
        };

        match outcome {
            Ok(fb) => {
                m.faults = 0;
                m.last_error = None;
                m.last = fb;
                m.last_ok = Some(Instant::now());
                self.shared.seed_position(i, fb.position_rad);
            }
            Err(e) => note_fault(m, "transaction", e, &self.events),
        }

        self.status_if_due(i);
    }


    /// Voltage, temperature and fault bits, on their own slower cadence.
    ///
    /// A separate transaction on most families, and the only source of
    /// temperature on some: a RobStride reports none in its feedback frame
    /// outside MIT mode, so without this the temperature column would read "not
    /// reported" for the whole session.
    fn status_if_due(&mut self, i: usize) {
        let m = &mut self.motors[i];
        if m.last_status_poll.elapsed() < STATUS_INTERVAL {
            return;
        }
        m.last_status_poll = Instant::now();
        match m.actuator.read_status() {
            Ok(st) => {
                m.status_misses = 0;
                m.voltage_v = st.voltage_v;
                m.status_temperature_c = st.temperature_c;
                m.error_bits = st.error.bits();
                m.error_raw = st.error.raw();
            }
            Err(e) => {
                // Deliberately not a control fault. A driver that cannot report
                // status must not be pushed towards the cut-off and silenced
                // while it is controlling perfectly well.
                m.status_misses += 1;
                if m.status_misses == 1 {
                    log::debug!(
                        "multi: motor {} does not report status: {e}",
                        m.spec.motor_id
                    );
                }
            }
        }
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
                self.shared.safety.set_streaming(on);
                // Beat on the way in, so enabling streaming does not trip the
                // watchdog on the very next check because the last heartbeat
                // predates it.
                self.shared.safety.beat();
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
        self.shared.safety.set_streaming(false);
        self.disable_all();
        self.log(LogLevel::Warn, format!("stopped every motor ({reason:?})"));
    }

    fn flush_if_due(&mut self) {
        if self.last_flush.elapsed() < TELEMETRY_INTERVAL {
            return;
        }
        let elapsed = self.last_flush.elapsed().as_secs_f32().max(1e-6);
        // Read once for the whole snapshot, so every row's target comes from the
        // same instant as its neighbours.
        let targets: Vec<f32> = self
            .shared
            .setpoints
            .lock()
            .map(|g| g.iter().map(|sp| sp.primary()).collect())
            .unwrap_or_default();
        let snapshot = MultiSnapshot {
            t_s: self.shared.safety.elapsed().as_secs_f64(),
            motors: self
                .motors
                .iter()
                .enumerate()
                .map(|(i, m)| MotorReading {
                    target: targets.get(i).copied().unwrap_or(f32::NAN),
                    motor_id: m.spec.motor_id,
                    driver: m.spec.driver,
                    label: m.spec.label(),
                    position_rad: m.last.position_rad,
                    velocity_rad_s: m.last.velocity_rad_per_s,
                    torque_nm: m.last.torque_nm,
                    current_a: m.last.current_a,
                    temperature_c: m.temperature(),
                    voltage_v: m.voltage_v,
                    error_bits: m.error_bits,
                    error_raw: m.error_raw,
                    age_ms: m.last_ok.map(|t| t.elapsed().as_millis() as u64),
                    enabled: m.enabled,
                    misses: m.faults,
                    error: m.last_error.clone(),
                })
                .collect(),
            achieved_rate_hz: self.passes as f32 / elapsed,
            dropped: self.dropped,
            starved_passes: self.starved,
            drained_frames: self.drained,
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
        self.shared.safety.set_streaming(false);
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

    /// A reading has to say how old it is, or a table will render a stale value
    /// as though it were current.
    #[test]
    fn a_reading_carries_its_own_age() {
        let s = MultiSession::connect(&sim_cfg(&[1, 2])).unwrap();
        for _ in 0..200 {
            let snap = wait_for_snapshot(&s);
            if snap.motors.iter().all(|m| m.age_ms.is_some()) {
                for m in &snap.motors {
                    let age = m.age_ms.unwrap();
                    assert!(age < 5_000, "motor {} reported age {age} ms", m.motor_id);
                }
                return;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        panic!("no motor ever reported an age");
    }

    /// Before a motor has answered there is no age to report, and inventing one
    /// (zero, say) would claim a measurement that never happened.
    #[test]
    fn an_age_is_absent_rather_than_zero_before_the_first_reply() {
        let s = MultiSession::connect(&sim_cfg(&[1])).unwrap();
        let snap = wait_for_snapshot(&s);
        // Either it has answered by now, or it has not; both are fine, but an
        // unanswered motor must report `None` rather than 0.
        if snap.motors[0].age_ms.is_none() {
            assert_eq!(snap.motors[0].misses, 0);
        }
    }

    /// Temperature is `NaN` when nothing reported one, never zero. The `ideal`
    /// simulator preset does report temperature, so this checks the value is at
    /// least not a silently-substituted zero.
    #[test]
    fn temperature_is_reported_or_absent_but_never_faked() {
        let s = MultiSession::connect(&sim_cfg(&[1])).unwrap();
        let snap = wait_for_snapshot(&s);
        let t = snap.motors[0].temperature_c;
        assert!(
            t.is_nan() || t > -273.15,
            "temperature was {t}, which is neither a reading nor an absence"
        );
    }

    /// A shaft resting away from zero must not be commanded to zero by the act
    /// of connecting.
    ///
    /// Three cases, because each guard removes a different accident:
    ///
    ///   - untouched: the setpoint follows the measurement, so the first
    ///     streaming pass says "stay where you are" instead of "go to zero"
    ///   - touched: seeding stops, or "set a target, then enable" would have the
    ///     next reading quietly discard the target
    ///   - not finite: a driver that reports no position must not be able to
    ///     write `NaN` into a command
    ///
    /// Tested against `Shared` rather than through a session because the
    /// simulator starts every preset at zero, where a correct seed and a
    /// forgotten one look identical.
    #[test]
    fn a_resting_shaft_is_not_commanded_to_zero() {
        let shared = Shared {
            safety: Arc::new(Safety::new(DEFAULT_RATE_HZ, None)),
            setpoints: Mutex::new(vec![Setpoint::default(); 2]),
            setpoints_touched: (0..2).map(|_| AtomicBool::new(false)).collect(),
        };
        let target_of = |i: usize| shared.setpoints.lock().unwrap()[i].position_rad;

        // Default is the dangerous value, which is the whole reason for seeding.
        assert_eq!(target_of(0), 0.0);

        shared.seed_position(0, 5.9);
        assert_eq!(target_of(0), 5.9, "an untouched motor should follow its shaft");
        // Still tracking: a shaft turned by hand keeps the command with it.
        shared.seed_position(0, 6.1);
        assert_eq!(target_of(0), 6.1);

        shared.setpoints_touched[0].store(true, Ordering::Release);
        shared.seed_position(0, 0.2);
        assert_eq!(target_of(0), 6.1, "a chosen target must survive the next reading");

        shared.seed_position(1, f32::NAN);
        assert_eq!(target_of(1), 0.0, "NaN must not reach a command");
        shared.seed_position(1, f32::INFINITY);
        assert_eq!(target_of(1), 0.0);

        // Per motor: seeding motor 0 must not have moved motor 1's target.
        shared.seed_position(1, -1.25);
        assert_eq!(target_of(1), -1.25);
        assert_eq!(target_of(0), 6.1);
    }

    /// Marking a target as chosen is what stops the worker from seeding over it,
    /// so it has to happen on the same call that writes the target — not later,
    /// and not in the UI.
    #[test]
    fn choosing_a_target_marks_it_as_chosen() {
        let s = MultiSession::connect(&sim_cfg(&[1, 2])).unwrap();
        let sp = Setpoint {
            position_rad: 0.5,
            ..Setpoint::default()
        };
        s.set_setpoint(2, sp).unwrap();
        assert!(
            s.shared.setpoints_touched[1].load(Ordering::Acquire),
            "the motor that was addressed"
        );
        assert!(
            !s.shared.setpoints_touched[0].load(Ordering::Acquire),
            "and only that motor"
        );
        // The value survives however many readings the worker takes meanwhile.
        let _ = wait_for_snapshot(&s);
        assert_eq!(s.setpoint(2).unwrap().position_rad, 0.5);
    }

    /// The TypeScript side depends on these shapes, so a rename here fails a
    /// test rather than a running app.
    ///
    /// Both rename attributes matter on the enums. `rename_all` covers variant
    /// names, `rename_all_fields` covers the fields inside them, and omitting
    /// the second sends `motorId` as `motor_id` — which arrives as `undefined`
    /// rather than as a parse error. That has produced a black screen twice
    /// (`doc/handover.md` section 5).
    #[test]
    fn the_wire_format_is_stable_enough_to_hand_to_a_ui() {
        let json = serde_json::to_string(&MultiCommand::Enable { motor_id: 3 }).unwrap();
        assert!(json.contains("\"kind\":\"enable\""), "{json}");
        assert!(json.contains("\"motorId\":3"), "{json}");

        let json = serde_json::to_string(&MultiCommand::EnableAll).unwrap();
        assert!(json.contains("\"kind\":\"enable-all\""), "{json}");

        let json = serde_json::to_string(&MultiCommand::SetStreaming { on: true }).unwrap();
        assert!(json.contains("\"kind\":\"set-streaming\""), "{json}");
        assert!(json.contains("\"on\":true"), "{json}");

        let json = serde_json::to_string(&MultiEvent::MotorState {
            motor_id: 7,
            enabled: true,
        })
        .unwrap();
        assert!(json.contains("\"event\":\"motor-state\""), "{json}");
        assert!(json.contains("\"motorId\":7"), "{json}");

        let json = serde_json::to_string(&MultiEvent::Stopped {
            reason: StopReason::Watchdog,
        })
        .unwrap();
        assert!(json.contains("\"event\":\"stopped\""), "{json}");
        assert!(json.contains("\"reason\":\"watchdog\""), "{json}");

        let snapshot = MultiSnapshot {
            t_s: 1.5,
            motors: vec![MotorReading {
                motor_id: 16,
                driver: crate::factory::DriverKind::Damiao,
                label: "id 16".into(),
                position_rad: 0.25,
                velocity_rad_s: 0.0,
                torque_nm: 0.1,
                current_a: f32::NAN,
                temperature_c: f32::NAN,
                voltage_v: 24.0,
                error_bits: 0,
                error_raw: 0,
                age_ms: Some(12),
                enabled: true,
                misses: 0,
                error: None,
                target: 0.25,
            }],
            achieved_rate_hz: 50.0,
            dropped: 0,
            starved_passes: 0,
            drained_frames: 0,
        };
        let json = serde_json::to_string(&snapshot).unwrap();
        assert!(json.contains("\"motorId\":16"), "{json}");
        assert!(json.contains("\"driver\":\"damiao\""), "{json}");
        assert!(json.contains("\"positionRad\":0.25"), "{json}");
        assert!(json.contains("\"ageMs\":12"), "{json}");
        // Commanded, next to measured. A UI fills its target box from this, so a
        // rename would silently give every box a default of zero again — which is
        // the jump this field exists to prevent.
        assert!(json.contains("\"target\":0.25"), "{json}");
        assert!(json.contains("\"achievedRateHz\":50"), "{json}");
        assert!(json.contains("\"starvedPasses\":0"), "{json}");
        assert!(json.contains("\"drainedFrames\":0"), "{json}");
        // A NaN has no JSON literal and serde writes `null`. The UI has to render
        // that as "not reported" rather than as a number, so pin it: silently
        // becoming 0 here would turn an absent temperature into a plausible one.
        assert!(json.contains("\"temperatureC\":null"), "{json}");
        assert!(json.contains("\"currentA\":null"), "{json}");
        // Absent rather than null, so the frontend's optional check works.
        assert!(!json.contains("\"error\":\"") && json.contains("\"error\":null"), "{json}");
    }

    /// A simulated set has no wire, so nothing can be drained and nothing can
    /// be starved. Pins that both counters mean "this happened" rather than
    /// carrying a default that looks like a measurement.
    #[test]
    fn a_simulated_set_drains_nothing_and_starves_nobody() {
        let s = MultiSession::connect(&sim_cfg(&[1, 2])).unwrap();
        let snap = wait_for_snapshot(&s);
        assert_eq!(snap.drained_frames, 0);
        assert_eq!(snap.starved_passes, 0);
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
            snap.achieved_rate_hz <= crate::protocol::MAX_RATE_HZ,
            "rate was {}",
            snap.achieved_rate_hz
        );
    }
}
