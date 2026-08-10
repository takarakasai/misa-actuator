//! A live connection to one motor: the worker thread plus the handle a UI
//! drives it through.

use std::sync::atomic::Ordering;
use std::sync::mpsc::{self, Receiver, Sender, SyncSender};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use anyhow::{Context, Result};

use crate::factory::{build_actuator_checked, DriverConfig};
use crate::protocol::*;
use crate::worker::{rate_to_millihz, Shared, Worker};

/// How many telemetry batches may be in flight before the worker starts
/// dropping them. Four frames of slack: enough to ride out a slow repaint,
/// short enough that the plot never lags visibly behind the motor.
const TELEMETRY_DEPTH: usize = 4;

/// A connected motor.
///
/// Dropping a `Session` stops the motor and waits for the worker to confirm
/// it — see the [`Drop`] impl, which is the last line of defence for a window
/// closed while something is being driven.
pub struct Session {
    shared: Arc<Shared>,
    commands: Sender<Command>,
    events: Receiver<Event>,
    telemetry: Receiver<TelemetryBatch>,
    join: Option<JoinHandle<()>>,
    watchdog: Option<JoinHandle<()>>,
    description: String,
    motor_id: u8,
}

impl Session {
    /// Open the motor and start its worker thread.
    ///
    /// The driver is built on the calling thread so a bad interface, missing
    /// adapter or wrong bitrate surfaces as an ordinary error return rather
    /// than as an event some time later.
    pub fn connect(cfg: &DriverConfig) -> Result<Self> {
        Self::connect_with(cfg, DEFAULT_RATE_HZ, Some(DEFAULT_WATCHDOG))
    }

    /// Connect with an explicit tick rate and watchdog. `None` disables the
    /// watchdog — appropriate for tests and headless use, not for a UI.
    pub fn connect_with(
        cfg: &DriverConfig,
        rate_hz: f32,
        watchdog: Option<Duration>,
    ) -> Result<Self> {
        let (actuator, identity) =
            build_actuator_checked(cfg).context("failed to open the motor")?;
        let motor_id = actuator.motor_id();
        let description = describe(cfg);

        let shared = Arc::new(Shared::new(rate_hz, watchdog));
        shared.beat();

        let (cmd_tx, cmd_rx) = mpsc::channel::<Command>();
        let (evt_tx, evt_rx) = mpsc::channel::<Event>();
        let (tel_tx, tel_rx): (SyncSender<TelemetryBatch>, _) =
            mpsc::sync_channel(TELEMETRY_DEPTH);

        let _ = evt_tx.send(Event::State(ConnectionState::Connected {
            description: description.clone(),
            motor_id,
            identity: (!identity.observed.is_empty()).then(|| identity.observed.clone()),
            identity_warning: identity.warning.clone(),
        }));

        let worker = Worker::new(actuator, shared.clone(), cmd_rx, evt_tx, tel_tx);
        let join = std::thread::Builder::new()
            .name(format!("motor-{motor_id}"))
            .spawn(move || worker.run())
            .context("failed to spawn the motor worker thread")?;

        // The watchdog gets its own thread rather than a check inside the
        // worker's loop, because a measurement job owns that loop for tens of
        // seconds and a loop-based check would simply not run during exactly
        // the window that matters most.
        let watchdog_shared = shared.clone();
        let watchdog = std::thread::Builder::new()
            .name(format!("motor-{motor_id}-watchdog"))
            .spawn(move || crate::worker::watchdog_loop(watchdog_shared))
            .context("failed to spawn the watchdog thread")?;

        Ok(Self {
            shared,
            commands: cmd_tx,
            events: evt_rx,
            telemetry: tel_rx,
            join: Some(join),
            watchdog: Some(watchdog),
            description,
            motor_id,
        })
    }

    /// What was actually opened, for display.
    pub fn description(&self) -> &str {
        &self.description
    }

    pub fn motor_id(&self) -> u8 {
        self.motor_id
    }

    /// Queue a discrete command.
    ///
    /// Fails only once the worker has exited; a live worker never rejects one.
    pub fn send(&self, cmd: Command) -> Result<()> {
        self.commands
            .send(cmd)
            .map_err(|_| anyhow::anyhow!("the motor worker has stopped"))
    }

    /// Overwrite the continuous setpoint. Cheap enough to call every frame —
    /// that is what it is for.
    pub fn set_setpoint(&self, sp: Setpoint) {
        // Marks the setpoint as deliberately chosen, which stops the worker
        // seeding the position from the shaft underneath the caller.
        self.shared
            .setpoint_touched
            .store(true, Ordering::Release);
        if let Ok(mut guard) = self.shared.setpoint.lock() {
            *guard = sp;
        }
    }

    pub fn setpoint(&self) -> Setpoint {
        self.shared
            .setpoint
            .lock()
            .map(|g| *g)
            .unwrap_or_default()
    }

    /// **Stop now.**
    ///
    /// Deliberately not a [`Command`]: a stop that queues behind whatever the
    /// worker is already doing is not a stop. This sets a flag the worker
    /// checks before every transaction — and which `misa-sysid` reads as its
    /// abort, so it also cuts a running measurement short.
    pub fn stop(&self) {
        self.shared.request_stop(crate::worker::STOP_USER);
    }

    /// Whether a measurement job is running.
    pub fn is_job_running(&self) -> bool {
        self.shared.job_active.load(Ordering::Acquire)
    }

    /// Tell the worker the UI is still alive.
    ///
    /// Call this from the UI's frame loop while streaming. Miss it for longer
    /// than the watchdog interval and the motor is disabled — which is the
    /// behaviour you want when the thing that was supposed to be steering has
    /// stopped running.
    pub fn heartbeat(&self) {
        self.shared.beat();
    }

    /// Change the watchdog interval, or disable it with `None`.
    pub fn set_watchdog(&self, watchdog: Option<Duration>) {
        self.shared.watchdog_ms.store(
            watchdog.map(|d| d.as_millis() as u64).unwrap_or(0),
            Ordering::Relaxed,
        );
    }

    /// Change the worker's tick rate without a round trip through the queue.
    pub fn set_rate(&self, hz: f32) {
        self.shared
            .rate_millihz
            .store(rate_to_millihz(hz), Ordering::Relaxed);
    }

    pub fn is_streaming(&self) -> bool {
        self.shared.streaming.load(Ordering::Acquire)
    }

    /// Take every event the worker has emitted since the last call.
    pub fn poll_events(&self) -> Vec<Event> {
        self.events.try_iter().collect()
    }

    /// Take every telemetry batch pending. Batches arrive at roughly frame
    /// rate; a UI that calls this once per frame usually gets one.
    pub fn drain_telemetry(&self) -> Vec<TelemetryBatch> {
        self.telemetry.try_iter().collect()
    }

    /// Flatten pending telemetry into a plain sample list, for callers that do
    /// not care about batch boundaries.
    pub fn drain_samples(&self) -> Vec<Sample> {
        self.drain_telemetry()
            .into_iter()
            .flat_map(|b| b.samples)
            .collect()
    }

    /// Stop the motor and shut the worker down, blocking until it confirms.
    ///
    /// Idempotent. Called automatically on drop.
    pub fn shutdown(&mut self) {
        // Flag first, command second: if the queue is backed up — or a job
        // owns the loop — the flag is what actually stops the motor promptly.
        self.stop();
        let _ = self.commands.send(Command::Shutdown);
        if let Some(join) = self.join.take() {
            if join.join().is_err() {
                log::error!("motor worker panicked; the motor may still be energised");
            }
        }
        // Only now: the watchdog has to outlive the worker, or a worker still
        // winding down from a job would lose its protection partway through.
        self.shared.alive.store(false, Ordering::Release);
        if let Some(watchdog) = self.watchdog.take() {
            let _ = watchdog.join();
        }
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// A short human-readable description of what a config connects to.
///
/// The model is included for the families whose MIT scales depend on it. A
/// RobStride connected as the wrong model quantises torque over the wrong
/// range — ±120 N·m for an RS-04 against ±5.5 for an RS-05 — so which one is
/// in force has to be visible, not buried in a form field that was left blank.
fn describe(cfg: &DriverConfig) -> String {
    use crate::factory::DriverKind;
    match cfg.kind {
        DriverKind::Sim => format!("simulator ({}) id {}", cfg.model, cfg.motor_id),
        DriverKind::Lkmotor => format!(
            "lkmotor {} @ {} baud, id {}",
            cfg.interface, cfg.baud, cfg.motor_id
        ),
        // Keep the name the operator picked, but show the model it resolves
        // to when they differ — an `EduLite05` session reads as
        // `EduLite05 (RS-05)`, because RS-05 is what sets the MIT scaling.
        DriverKind::Robstride => {
            let model = robstride_driver::MotorModel::describe_selection(&cfg.model)
                .map_or_else(|| cfg.model.clone(), |c| c.to_string());
            format!(
                "robstride {} on {} id {}",
                model, cfg.interface, cfg.motor_id
            )
        }
        DriverKind::Damiao => format!(
            "damiao {} on {} id {} ({})",
            cfg.model,
            cfg.interface,
            cfg.motor_id,
            match cfg.bus_kind {
                crate::factory::BusKind::Can => "classic",
                crate::factory::BusKind::CanFd => "CAN-FD",
            }
        ),
        DriverKind::Myactuator => {
            format!("myactuator {} id {}", cfg.interface, cfg.motor_id)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::factory::DriverKind;
    use std::time::Instant;

    fn sim_config() -> DriverConfig {
        DriverConfig {
            kind: DriverKind::Sim,
            model: "ideal".to_string(),
            interface: String::new(),
            motor_id: 1,
            ..DriverConfig::default()
        }
    }

    /// Spin until `pred` sees what it is waiting for, or give up.
    fn wait_for<T>(timeout: Duration, mut poll: impl FnMut() -> Option<T>) -> Option<T> {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if let Some(v) = poll() {
                return Some(v);
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        None
    }

    /// Collect events until one matches, keeping the rest out of the way.
    fn wait_event(
        session: &Session,
        timeout: Duration,
        mut matches: impl FnMut(&Event) -> bool,
    ) -> Option<Event> {
        wait_for(timeout, || {
            session.poll_events().into_iter().find(&mut matches)
        })
    }

    #[test]
    fn connecting_reports_the_connection_and_starts_polling() {
        let session = Session::connect(&sim_config()).expect("connect");
        assert_eq!(session.motor_id(), 1);

        let connected = wait_event(&session, Duration::from_secs(2), |e| {
            matches!(e, Event::State(ConnectionState::Connected { .. }))
        });
        assert!(connected.is_some(), "no Connected event");

        // Idle polling should produce telemetry without anyone asking.
        let samples = wait_for(Duration::from_secs(2), || {
            let s = session.drain_samples();
            (!s.is_empty()).then_some(s)
        });
        assert!(samples.is_some(), "no idle telemetry");
    }

    /// The Monitor tab is built entirely on this event, and it polls for it
    /// rather than receiving it with telemetry. If `ReadStatus` stopped
    /// answering, the tab would sit on "waiting for the first status read"
    /// forever without anything looking broken.
    #[test]
    fn read_status_answers_with_a_usable_health_snapshot() {
        let session = Session::connect(&sim_config()).expect("connect");
        session.send(Command::ReadStatus).expect("send");

        let status = wait_event(&session, Duration::from_secs(2), |e| {
            matches!(e, Event::Status(_))
        })
        .expect("no Status event");
        let Event::Status(s) = status else {
            unreachable!("filtered above")
        };

        // Values a health view can actually render, rather than zeroes that
        // would render as a plausible-looking dead motor.
        assert!(s.voltage_v > 1.0, "voltage {} looks unreported", s.voltage_v);
        assert!(
            (-40.0..150.0).contains(&s.temperature_c),
            "temperature {} is outside anything physical",
            s.temperature_c
        );
        assert_eq!(s.error_bits, 0, "an idle simulated motor should be clean");
    }

    #[test]
    fn a_bad_configuration_fails_at_connect_rather_than_later() {
        let cfg = DriverConfig {
            kind: DriverKind::Sim,
            model: "no-such-preset".to_string(),
            ..sim_config()
        };
        assert!(Session::connect(&cfg).is_err());
    }

    #[test]
    fn streaming_drives_the_motor_towards_the_setpoint() {
        let session =
            Session::connect_with(&sim_config(), 200.0, None).expect("connect");
        session.send(Command::Enable).unwrap();
        session.set_setpoint(Setpoint {
            mode: ControlMode::Position,
            position_rad: 0.5,
            max_speed_rad_s: 5.0,
            ..Setpoint::default()
        });
        session.send(Command::SetStreaming { on: true }).unwrap();

        let reached = wait_for(Duration::from_secs(5), || {
            session
                .drain_samples()
                .into_iter()
                .find(|s| (s.position_rad - 0.5).abs() < 0.02)
        });
        assert!(reached.is_some(), "never reached the commanded position");
    }

    #[test]
    fn the_setpoint_is_overwritten_not_queued() {
        // Hammer the setpoint the way a dragged slider would. If these were
        // queued, the motor would still be working through them long after.
        let session =
            Session::connect_with(&sim_config(), 200.0, None).expect("connect");
        session.send(Command::Enable).unwrap();
        session.send(Command::SetStreaming { on: true }).unwrap();

        for i in 0..2_000 {
            session.set_setpoint(Setpoint {
                mode: ControlMode::Position,
                position_rad: (i as f32) * 1e-4,
                max_speed_rad_s: 5.0,
                ..Setpoint::default()
            });
        }
        // Settle on a final value and check the motor converges on *it*,
        // not on the thousands of intermediate ones.
        session.set_setpoint(Setpoint {
            mode: ControlMode::Position,
            position_rad: 0.0,
            max_speed_rad_s: 5.0,
            ..Setpoint::default()
        });
        let settled = wait_for(Duration::from_secs(5), || {
            session
                .drain_samples()
                .into_iter()
                .find(|s| s.position_rad.abs() < 0.01 && s.t_s > 0.5)
        });
        assert!(settled.is_some(), "did not settle on the final setpoint");
    }

    #[test]
    fn stop_disables_the_motor_and_says_why() {
        let session =
            Session::connect_with(&sim_config(), 200.0, None).expect("connect");
        session.send(Command::Enable).unwrap();
        session.send(Command::SetStreaming { on: true }).unwrap();
        // Let it actually start driving.
        std::thread::sleep(Duration::from_millis(100));
        assert!(session.is_streaming());

        session.stop();

        let stopped = wait_event(&session, Duration::from_secs(2), |e| {
            matches!(
                e,
                Event::Stopped {
                    reason: StopReason::UserRequest
                }
            )
        });
        assert!(stopped.is_some(), "no Stopped event");
        assert!(!session.is_streaming());
    }

    #[test]
    fn the_watchdog_disables_a_motor_whose_ui_went_quiet() {
        let session = Session::connect_with(
            &sim_config(),
            200.0,
            Some(Duration::from_millis(150)),
        )
        .expect("connect");
        session.send(Command::Enable).unwrap();
        session.send(Command::SetStreaming { on: true }).unwrap();

        // Deliberately never call `heartbeat()` — this is the hung-UI case.
        let stopped = wait_event(&session, Duration::from_secs(3), |e| {
            matches!(
                e,
                Event::Stopped {
                    reason: StopReason::Watchdog
                }
            )
        });
        assert!(stopped.is_some(), "watchdog never fired");
        assert!(!session.is_streaming());
    }

    #[test]
    fn a_beating_heart_keeps_the_motor_running() {
        let session = Session::connect_with(
            &sim_config(),
            200.0,
            Some(Duration::from_millis(150)),
        )
        .expect("connect");
        session.send(Command::Enable).unwrap();
        session.send(Command::SetStreaming { on: true }).unwrap();

        let deadline = Instant::now() + Duration::from_millis(800);
        let mut watchdog_fired = false;
        while Instant::now() < deadline {
            session.heartbeat();
            std::thread::sleep(Duration::from_millis(40));
            if session.poll_events().iter().any(|e| {
                matches!(
                    e,
                    Event::Stopped {
                        reason: StopReason::Watchdog
                    }
                )
            }) {
                watchdog_fired = true;
                break;
            }
        }
        assert!(!watchdog_fired, "watchdog fired despite heartbeats");
        assert!(session.is_streaming());
    }

    #[test]
    fn the_watchdog_leaves_a_merely_polling_session_alone() {
        // Not streaming: nothing is being driven, so there is nothing to
        // protect against and no reason to nag.
        let session = Session::connect_with(
            &sim_config(),
            200.0,
            Some(Duration::from_millis(100)),
        )
        .expect("connect");
        std::thread::sleep(Duration::from_millis(500));
        assert!(!session
            .poll_events()
            .iter()
            .any(|e| matches!(e, Event::Stopped { .. })));
    }

    #[test]
    fn a_scan_reports_progress_and_finds_the_simulated_bus() {
        let session =
            Session::connect_with(&sim_config(), 200.0, None).expect("connect");
        session
            .send(Command::StartScan {
                from: 1,
                to: 12,
                timeout_ms: 5,
            })
            .unwrap();

        let mut progress_updates = 0;
        let mut found = None;
        let deadline = Instant::now() + Duration::from_secs(5);
        while Instant::now() < deadline && found.is_none() {
            for e in session.poll_events() {
                match e {
                    Event::ScanProgress { .. } => progress_updates += 1,
                    Event::ScanFinished { found: ids, .. } => found = Some(ids),
                    _ => {}
                }
            }
            std::thread::sleep(Duration::from_millis(5));
        }

        let found = found.expect("scan never finished");
        // The simulated bus carries several ids so a scan UI has something to
        // show; see `factory::DEFAULT_SIM_BUS_IDS`.
        assert!(found.len() >= 3, "found only {found:?}");
        assert!(found.contains(&1));
        assert!(
            progress_updates >= 10,
            "only {progress_updates} progress updates over 12 ids — the UI \
             would have had nothing to paint"
        );
    }

    #[test]
    fn streaming_is_refused_while_disabled_rather_than_silently_ignored() {
        let session =
            Session::connect_with(&sim_config(), 200.0, None).expect("connect");
        session.send(Command::SetStreaming { on: true }).unwrap();

        let warned = wait_event(&session, Duration::from_secs(2), |e| {
            matches!(e, Event::Log { level: LogLevel::Warn, .. })
        });
        assert!(warned.is_some(), "no warning about streaming while disabled");
        assert!(!session.is_streaming());
    }

    #[test]
    fn status_is_polled_without_being_asked() {
        let session =
            Session::connect_with(&sim_config(), 200.0, None).expect("connect");
        let status = wait_event(&session, Duration::from_secs(3), |e| {
            matches!(e, Event::Status(_))
        });
        assert!(status.is_some(), "no status snapshot");
    }

    #[test]
    fn dropping_the_session_stops_the_motor_and_joins_the_worker() {
        let session =
            Session::connect_with(&sim_config(), 200.0, None).expect("connect");
        session.send(Command::Enable).unwrap();
        session.send(Command::SetStreaming { on: true }).unwrap();
        std::thread::sleep(Duration::from_millis(100));

        let started = Instant::now();
        drop(session);
        // Drop must actually wait for the worker, or a closing window could
        // outlive a motor that is still being driven.
        assert!(
            started.elapsed() < Duration::from_secs(2),
            "drop took {:?}",
            started.elapsed()
        );
    }

    fn short_chirp() -> JobSpec {
        JobSpec::Chirp(ChirpJob {
            duration_s: 1.5,
            f_start_hz: 1.0,
            f_end_hz: 40.0,
            rate_hz: 400.0,
            amplitude: 0.1,
            ..ChirpJob::default()
        })
    }

    #[test]
    fn a_chirp_runs_streams_samples_and_produces_a_bode_estimate() {
        let session =
            Session::connect_with(&sim_config(), 200.0, None).expect("connect");
        session.send(Command::StartJob { spec: short_chirp() }).unwrap();

        let mut progress = 0;
        let mut samples = 0;
        let mut finished = None;
        let deadline = Instant::now() + Duration::from_secs(15);
        while Instant::now() < deadline && finished.is_none() {
            samples += session.drain_samples().len();
            for e in session.poll_events() {
                match e {
                    Event::JobProgress { .. } => progress += 1,
                    Event::JobFinished { .. } => finished = Some(e),
                    _ => {}
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }

        let Some(Event::JobFinished {
            bode,
            samples: n,
            aborted,
            achieved_rate_hz,
            ..
        }) = finished
        else {
            panic!("the chirp never finished");
        };
        assert!(!aborted, "finished as aborted without anyone stopping it");
        assert!(n > 100, "only {n} samples");
        assert!(achieved_rate_hz > 100.0, "only {achieved_rate_hz} Hz");

        // Live telemetry during the sweep is the whole reason `run_chirp_with`
        // exists — a job that only reported at the end would need none of it.
        assert!(samples > 100, "only {samples} samples streamed during the run");
        assert!(progress >= 5, "only {progress} progress updates");

        let bode = bode.expect("no Bode estimate");
        assert!(!bode.freqs_hz.is_empty());
        assert_eq!(bode.freqs_hz.len(), bode.magnitude_db.len());
        assert_eq!(bode.freqs_hz.len(), bode.coherence.len());
    }

    #[test]
    fn stop_aborts_a_running_chirp_and_keeps_what_it_measured() {
        let session =
            Session::connect_with(&sim_config(), 200.0, None).expect("connect");
        session
            .send(Command::StartJob {
                spec: JobSpec::Chirp(ChirpJob {
                    duration_s: 30.0,
                    ..ChirpJob::default()
                }),
            })
            .unwrap();

        // Let it get going, then pull the plug.
        assert!(
            wait_for(Duration::from_secs(5), || session.is_job_running().then_some(())).is_some(),
            "the job never started"
        );
        std::thread::sleep(Duration::from_millis(400));
        session.stop();

        let finished = wait_event(&session, Duration::from_secs(5), |e| {
            matches!(e, Event::JobFinished { .. })
        });
        let Some(Event::JobFinished { aborted, samples, .. }) = finished else {
            panic!("the chirp did not stop");
        };
        assert!(aborted, "stopping a 30 s chirp after 0.4 s was not reported as an abort");
        // A partial sweep is still a measurement of the band it reached.
        assert!(samples > 10, "only {samples} samples kept");
        assert!(!session.is_job_running());
    }

    #[test]
    fn the_watchdog_stops_a_chirp_when_the_ui_goes_quiet() {
        // The case the watchdog runs on its own thread for. A job owns the
        // worker's loop for its whole duration, so a loop-based check would
        // never run during exactly the window where a motor is being excited
        // with nobody watching.
        let session = Session::connect_with(
            &sim_config(),
            200.0,
            Some(Duration::from_millis(200)),
        )
        .expect("connect");
        session
            .send(Command::StartJob {
                spec: JobSpec::Chirp(ChirpJob {
                    duration_s: 30.0,
                    ..ChirpJob::default()
                }),
            })
            .unwrap();

        // Never call `heartbeat()`.
        let finished = wait_event(&session, Duration::from_secs(8), |e| {
            matches!(e, Event::JobFinished { .. })
        });
        let Some(Event::JobFinished { aborted, .. }) = finished else {
            panic!("the watchdog did not cut the chirp short");
        };
        assert!(aborted);
        assert!(!session.is_job_running());
    }

    #[test]
    fn a_chirp_too_short_to_identify_reports_no_bode_rather_than_noise() {
        let session =
            Session::connect_with(&sim_config(), 200.0, None).expect("connect");
        session
            .send(Command::StartJob {
                spec: JobSpec::Chirp(ChirpJob {
                    duration_s: 0.05,
                    rate_hz: 100.0,
                    ..ChirpJob::default()
                }),
            })
            .unwrap();

        let finished = wait_event(&session, Duration::from_secs(8), |e| {
            matches!(e, Event::JobFinished { .. })
        });
        let Some(Event::JobFinished { bode, .. }) = finished else {
            panic!("no result");
        };
        assert!(
            bode.is_none(),
            "a handful of samples produced a Bode plot; that would be noise \
             presented as a measurement"
        );
    }

    #[test]
    fn a_run_of_faults_stops_the_motor_but_keeps_the_session_usable() {
        // Found on hardware: a RobStride rejects `set_position` until its run
        // mode is set, the rejection repeats every tick, and twenty of them
        // arrive in a tenth of a second. The worker used to exit on that,
        // leaving a UI that looked connected but whose every button answered
        // "the worker has stopped" — a recoverable mistake made permanent.
        let mut cfg = sim_config();
        cfg.model = "ideal".to_string();
        let session = Session::connect_with(&cfg, 200.0, None).expect("connect");

        // A dead motor fails every transaction, which is the same shape.
        session.send(Command::Enable).unwrap();
        session.send(Command::SetStreaming { on: true }).unwrap();
        std::thread::sleep(Duration::from_millis(300));

        // Whatever happened, the session is still there and still answering.
        assert!(
            session.send(Command::Measure).is_ok(),
            "the session stopped accepting commands"
        );
        let alive = wait_for(Duration::from_secs(2), || {
            (!session.drain_samples().is_empty()).then_some(())
        });
        assert!(alive.is_some(), "the worker stopped producing telemetry");
    }

    #[test]
    fn streaming_sets_the_run_mode_the_setpoint_asks_for() {
        // The worker owns this invariant, not the UI: a front end that only
        // sends `set-run-mode` when a mode button is clicked never sends one
        // for whichever mode its form started on.
        let session =
            Session::connect_with(&sim_config(), 200.0, None).expect("connect");
        session.send(Command::Enable).unwrap();
        // Never send SetRunMode; go straight to streaming in Torque.
        session.set_setpoint(Setpoint {
            mode: ControlMode::Torque,
            torque_nm: 0.05,
            ..Setpoint::default()
        });
        session.send(Command::SetStreaming { on: true }).unwrap();

        let moved = wait_for(Duration::from_secs(3), || {
            session
                .drain_samples()
                .into_iter()
                .find(|s| s.velocity_rad_s.abs() > 0.05)
        });
        assert!(
            moved.is_some(),
            "torque streaming produced no motion — the run mode was never set"
        );
    }

    #[test]
    fn a_chosen_setpoint_survives_a_later_enable() {
        // The first attempt at seeding the position had `enable` overwrite the
        // setpoint unconditionally. Because the setpoint is written
        // immediately while commands are queued, "choose a target, then
        // enable" had the enable silently discard the target — turning a
        // safety measure into a way to ignore the operator.
        let session =
            Session::connect_with(&sim_config(), 200.0, None).expect("connect");
        session.set_setpoint(Setpoint {
            mode: ControlMode::Position,
            position_rad: 0.4,
            max_speed_rad_s: 5.0,
            ..Setpoint::default()
        });
        session.send(Command::Enable).unwrap();
        session.send(Command::SetStreaming { on: true }).unwrap();

        let reached = wait_for(Duration::from_secs(5), || {
            session
                .drain_samples()
                .into_iter()
                .find(|s| (s.position_rad - 0.4).abs() < 0.02)
        });
        assert!(
            reached.is_some(),
            "enable discarded the chosen target; setpoint is now {:.3}",
            session.setpoint().position_rad
        );
    }

    #[test]
    fn seeding_leaves_a_usable_position_before_anything_is_commanded() {
        // The hazard: a motor remembers its position across power cycles, a
        // freshly-opened form does not. Real hardware sat at 5.9 rad while the
        // form still said 0, so the first streaming tick would have commanded
        // a 5.9 rad journey.
        //
        // The simulator always starts at zero, so it cannot reproduce that
        // mismatch — seeded and stale look identical here. What is covered is
        // that seeding runs at all and leaves a finite value rather than the
        // NaN an unreported reading would give, which is how the mechanism
        // would fail silently.
        let session =
            Session::connect_with(&sim_config(), 200.0, None).expect("connect");
        assert!(
            wait_for(Duration::from_secs(2), || {
                (!session.drain_samples().is_empty()).then_some(())
            })
            .is_some(),
            "no reading to seed from"
        );
        let sp = session.setpoint();
        assert!(
            sp.position_rad.is_finite(),
            "seeded a non-finite position: {}",
            sp.position_rad
        );
    }

    #[test]
    fn commands_are_rejected_once_the_worker_is_gone() {
        let mut session =
            Session::connect_with(&sim_config(), 200.0, None).expect("connect");
        session.shutdown();
        assert!(session.send(Command::Enable).is_err());
    }
}
