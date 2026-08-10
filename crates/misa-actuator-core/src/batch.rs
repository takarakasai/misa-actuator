//! Running the same measurements across several motors, one after another.
//!
//! **Sequential, not concurrent, and that is not a shortcut.** A quasi-static
//! run assumes it is the only thing on the bus: it holds a dwell and reads a
//! steady torque, and N motors sharing one CAN segment would perturb each
//! other's timing while claiming to measure it. One motor moving at a time is
//! also the only arrangement anyone can stand next to safely.
//!
//! Each motor gets its own [`Session`], opened and shut in turn, rather than one
//! session reconfigured — a driver is bound to a motor id when it opens, and
//! reusing the handle across motors would mean trusting that every driver
//! resets everything that matters.
//!
//! The batch thread is also the supervisor. It beats the session's heartbeat
//! from its own wait loop, so if it dies the watchdog stops the motor rather
//! than leaving a run driving with nobody watching.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::export;
use crate::factory::{BusKind, DriverConfig, DriverKind};
use crate::protocol::{CharacterizeJob, Command, Event, JobSpec, RunEnvelope};
use crate::session::Session;

/// How long to wait for one run before giving up on it.
///
/// Well above any run's own envelope (30 s) so this only fires when a run has
/// genuinely stopped reporting — a bus that went silent mid-dwell, say. Without
/// it a batch would wait for ever on one dead motor and never reach the rest.
const RUN_TIMEOUT: Duration = Duration::from_secs(180);

/// How often the wait loop wakes to beat the heartbeat and check for a stop.
const WAIT_TICK: Duration = Duration::from_millis(50);

/// One motor to visit, and what it is.
#[derive(Debug, Clone)]
pub struct BatchMotor {
    pub id: u8,
    /// Which family this motor is, where the scan could say.
    ///
    /// Per motor because a CAN wire carries mixed vendors — a MyActuator at id 1
    /// beside RobStrides at 2 and 3 is a real configuration, not a mistake — and
    /// a scan reports the family it answered as. One family for the whole batch
    /// would open every motor with the wrong protocol for some of them, which is
    /// the third time this shape of bug has appeared here: after the model and
    /// the torque ceiling, the driver was still being collapsed.
    ///
    /// `None` falls back to [`BatchSpec::driver`].
    pub driver: Option<DriverKind>,
    /// This motor's own model, where it is known.
    ///
    /// Per motor rather than one for the batch, because the model sets the MIT
    /// quantisation range and that spans 21-fold across the RobStride family: an
    /// RS-04 driven with RS-00 scales reports every torque about 7 times too
    /// small. A scan usually knows each motor's model, and using one figure from
    /// a form for all of them threw that away — measured on 2026-08-07, where a
    /// batch of two RS-04s run as RS-00 returned Kt 0.24 and 0.20 N·m/A against
    /// the 1.52 and 1.60 the same motors had given minutes earlier.
    ///
    /// `None` falls back to [`BatchSpec::model`].
    pub model: Option<String>,
    /// Torque ceiling for this motor's runs (N·m). `None` falls back to
    /// [`BatchSpec::envelope`].
    ///
    /// Per motor for the same reason the model is. A ceiling is a statement
    /// about one motor and its rig, and a bus can carry very different ones: an
    /// RS-05 peaks at 5.5 N·m where an RS-04 peaks at 120. One figure for the
    /// batch has to be either too high for the small motor — a ceiling above
    /// what it can survive protects nothing — or too low for the large one, at
    /// which point every run on it aborts without measuring. Taking the smallest
    /// looks safe and is not: it trades a safety problem for a silent
    /// measurement failure on every other motor.
    pub max_torque_nm: Option<f32>,
}

/// What to measure, and on which motors.
#[derive(Debug, Clone)]
pub struct BatchSpec {
    /// Connection settings shared by every motor. Only the id and model vary.
    pub driver: DriverKind,
    pub interface: String,
    pub bus: BusKind,
    /// Used for any motor whose own model is unknown.
    pub model: String,
    pub motors: Vec<BatchMotor>,
    /// Runs performed on each motor, in order.
    pub runs: Vec<CharacterizeJob>,
    pub envelope: RunEnvelope,
    /// Whether the ramping runs' ceilings are defaults that should follow each
    /// motor's own, rather than figures the operator chose.
    ///
    /// A default carries no intent, and following the wrong thing is how eleven
    /// RS-03s with 6 N·m ceilings were all ramped to 0.85 — half of the *bar's*
    /// stale RS-00 — so several never broke loose (2026-08-07). A value the
    /// operator typed is left alone whatever the motor's ceiling is: clamped
    /// down where it would not fit, never raised.
    pub ramp_follows_ceiling: bool,
    /// Measure each motor's own Kt before its runs, and use it for them.
    ///
    /// Worth defaulting on: Kt is a property of the individual motor, not of the
    /// model — a bench RS-03 measured 1.5872 N·m/A and an RS-04 1.5704 — and a
    /// batch is exactly where nobody is going to type one in per motor.
    pub measure_kt_first: bool,
}

/// One run's outcome, as the batch saw it.
#[derive(Debug, Clone)]
pub struct BatchResult {
    pub motor_id: u8,
    pub run: String,
    pub ok: bool,
    /// The run's own one-line finding, or why there is none.
    pub headline: String,
    pub csv_path: Option<String>,
    /// The curve and findings, kept so the result can be *looked at* rather than
    /// only summarised.
    ///
    /// A headline per run made a batch's output a list of sentences, and the
    /// whole point of measuring several motors is comparing their curves. The
    /// alternative was reopening each CSV by hand, which is not a UI.
    pub data: Option<crate::protocol::CharacterizeData>,
}

/// How far a batch has got, and what it has found so far.
#[derive(Debug, Clone, Default)]
pub struct BatchProgress {
    pub running: bool,
    pub motor_index: usize,
    pub motor_count: usize,
    /// The motor being worked on, or 0 before the first.
    pub motor_id: u8,
    pub run_index: usize,
    pub run_count: usize,
    pub run_name: String,
    /// Appended as each run finishes, so a long batch is readable while it runs
    /// rather than only at the end.
    pub results: Vec<BatchResult>,
    /// Set when the batch stopped without finishing.
    pub cancelled: bool,
    /// A failure that ended the whole batch, as opposed to one run failing.
    pub error: Option<String>,
}

impl BatchProgress {
    /// Runs finished out of runs planned, for a progress bar.
    pub fn fraction(&self) -> f32 {
        let total = (self.motor_count * self.run_count) as f32;
        if total <= 0.0 {
            return 0.0;
        }
        let done = (self.motor_index * self.run_count + self.run_index) as f32;
        (done / total).clamp(0.0, 1.0)
    }
}

/// Shared handle: the UI reads the progress, anything can cancel.
pub struct Batch {
    pub progress: Arc<Mutex<BatchProgress>>,
    cancel: Arc<AtomicBool>,
}

impl Batch {
    /// Ask the batch to stop after the run in flight.
    ///
    /// Also stops that run: the flag is checked by the wait loop, which stops the
    /// live session as soon as it sees it. A cancel that let the current motor
    /// finish would be a cancel that keeps a motor moving.
    pub fn cancel(&self) {
        self.cancel.store(true, Ordering::Release);
    }

    pub fn cancelled(&self) -> bool {
        self.cancel.load(Ordering::Acquire)
    }

    pub fn snapshot(&self) -> BatchProgress {
        self.progress
            .lock()
            .map(|p| p.clone())
            .unwrap_or_default()
    }

    pub fn running(&self) -> bool {
        self.progress.lock().map(|p| p.running).unwrap_or(false)
    }
}

/// Start a batch on its own thread and return the handle to watch it with.
///
/// Errors that stop the whole batch (an interface that will not open at all)
/// land in [`BatchProgress::error`]; a single motor failing is a
/// [`BatchResult`] with `ok: false` and does not stop the rest — a bench with
/// one dead motor on it should still measure the others.
pub fn spawn(spec: BatchSpec) -> Batch {
    let progress = Arc::new(Mutex::new(BatchProgress {
        running: true,
        motor_count: spec.motors.len(),
        run_count: spec.runs.len() + usize::from(spec.measure_kt_first),
        ..Default::default()
    }));
    let cancel = Arc::new(AtomicBool::new(false));
    let batch = Batch {
        progress: progress.clone(),
        cancel: cancel.clone(),
    };
    std::thread::Builder::new()
        .name("batch".to_string())
        .spawn(move || {
            run(spec, &progress, &cancel);
            if let Ok(mut p) = progress.lock() {
                p.running = false;
                p.cancelled = cancel.load(Ordering::Acquire);
            }
        })
        .expect("failed to spawn the batch thread");
    batch
}

fn run(spec: BatchSpec, progress: &Mutex<BatchProgress>, cancel: &AtomicBool) {
    for (mi, motor) in spec.motors.iter().enumerate() {
        if cancel.load(Ordering::Acquire) {
            return;
        }
        let motor_id = motor.id;
        if let Ok(mut p) = progress.lock() {
            p.motor_index = mi;
            p.motor_id = motor_id;
            p.run_index = 0;
        }

        let cfg = DriverConfig {
            kind: motor.driver.unwrap_or(spec.driver),
            interface: spec.interface.clone(),
            motor_id,
            // This motor's own model wins over the batch's fallback. Getting it
            // wrong is not a small error: it mis-scales every torque the run
            // reports, and nothing downstream can tell.
            model: motor.model.clone().unwrap_or_else(|| spec.model.clone()),
            bus_kind: spec.bus,
            ..DriverConfig::default()
        };

        // One motor refusing to open is that motor's result, not the batch's
        // failure. The next id may well be fine, and a bench usually has exactly
        // one thing wrong with it.
        let mut session = match Session::connect(&cfg) {
            Ok(s) => s,
            Err(e) => {
                push(
                    progress,
                    BatchResult {
                        motor_id,
                        run: "connect".to_string(),
                        ok: false,
                        headline: format!("{e:#}"),
                        csv_path: None,
                        data: None,
                    },
                );
                continue;
            }
        };

        // Each motor's own Kt, measured first and then used by its runs. Without
        // this every run on firmware that reports a constant zero torque would
        // measure zero, and a batch is the one place nobody can type a value in
        // between motors.
        let mut envelope = spec.envelope;
        if let Some(nm) = motor.max_torque_nm {
            envelope.max_torque_nm = nm;
        }
        // Ramping runs are scaled to this motor's ceiling too, or a ramp sized
        // for the largest motor on the bus would spend every run on a small one
        // being aborted by its envelope just short of finishing. Half, because
        // the envelope judges *measured* torque and a ramp that reaches it trips
        // it — the two ceilings must not meet.
        //
        // Clamped against the ceiling *in force*, not the raw field: 0 there
        // means "use the gentle default", and feeding it through as a number
        // capped every ramp at 0 N·m. A simulator batch then reported
        // "breakaway torque not reached below 0.000 N·m" — the run did exactly
        // what it was told and told nobody anything (2026-08-07).
        let effective_ceiling = if envelope.max_torque_nm > 0.0 {
            envelope.max_torque_nm
        } else {
            misa_sysid::SafetyLimits::gentle().max_torque_nm
        };
        let runs: Vec<CharacterizeJob> = spec
            .runs
            .iter()
            .map(|r| ramp_for(*r, effective_ceiling, spec.ramp_follows_ceiling))
            .collect();
        if spec.measure_kt_first {
            let kt_run = kt_job(envelope);
            match one_run(&session, kt_run, envelope, cancel) {
                Some(outcome) => {
                    if let Some(kt) = outcome.fitted_kt {
                        envelope.kt = kt;
                    }
                    push(progress, outcome.into_result(motor_id, kt_run.name()));
                }
                None => {
                    session.shutdown();
                    return;
                }
            }
            bump_run(progress);
        }

        for &job in &runs {
            if cancel.load(Ordering::Acquire) {
                session.shutdown();
                return;
            }
            if let Ok(mut p) = progress.lock() {
                p.run_name = job.name().to_string();
            }
            match one_run(&session, job, envelope, cancel) {
                Some(outcome) => push(progress, outcome.into_result(motor_id, job.name())),
                None => {
                    session.shutdown();
                    return;
                }
            }
            bump_run(progress);
        }

        // Shut down between motors rather than at the end: the bus is a shared
        // resource and holding a channel open for a motor we are done with can
        // stop the next one opening at all.
        session.shutdown();
    }
}

/// Fraction of a motor's torque ceiling a ramping run may climb to.
///
/// The two ceilings must not meet: the envelope judges *measured* torque, which
/// carries noise on hardware that derives it from current, so a ramp that
/// reaches the envelope trips it just short of finishing and a valid "did not
/// break loose" is discarded as a fault.
const RAMP_FRACTION: f32 = 0.5;

/// Hold a ramping run's ceiling under this motor's envelope.
///
/// Only lowers. A run configured more gently than the envelope allows is the
/// operator's choice about their rig, and a batch is not the place to overrule
/// it — this exists to stop one figure chosen for a large motor being applied to
/// a small one, not to push every run to the maximum.
/// This motor's ramp ceiling.
///
/// With `follow`, the run's own figure is a default and is replaced by half this
/// motor's ceiling — up or down. Without it, the figure is the operator's and is
/// only ever lowered to fit.
fn ramp_for(job: CharacterizeJob, ceiling_nm: f32, follow: bool) -> CharacterizeJob {
    // A zero or absurd ceiling would cap every ramp at zero, which is a run
    // that cannot measure anything and says so only in its result. Callers
    // resolve "use the default" before getting here; this refuses to turn a
    // sentinel into a limit.
    if !(ceiling_nm > 0.0) {
        return job;
    }
    let cap = ceiling_nm * RAMP_FRACTION;
    let resolve = |asked: f32| if follow { cap } else { asked.min(cap) };
    match job {
        CharacterizeJob::Breakaway {
            ramp_nm_per_s,
            max_torque_nm,
            positive,
            rate_hz,
        } => CharacterizeJob::Breakaway {
            ramp_nm_per_s,
            max_torque_nm: resolve(max_torque_nm),
            positive,
            rate_hz,
        },
        CharacterizeJob::BreakawayMap {
            half_span_rad,
            steps,
            ramp_nm_per_s,
            max_torque_nm,
            rate_hz,
            both_directions,
        } => CharacterizeJob::BreakawayMap {
            half_span_rad,
            steps,
            ramp_nm_per_s,
            max_torque_nm: resolve(max_torque_nm),
            rate_hz,
            both_directions,
        },
        // The others do not ramp torque against a ceiling: a load map and a
        // velocity sweep are bounded by position and speed, and the Kt run sizes
        // itself from the envelope already.
        other => other,
    }
}

/// The Kt run a batch uses, sized from the envelope so it cannot trip its own
/// ceiling — the same rule the GUI's one-click button follows.
fn kt_job(envelope: RunEnvelope) -> CharacterizeJob {
    let ceiling = if envelope.max_torque_nm > 0.0 {
        envelope.max_torque_nm
    } else {
        1.0
    };
    CharacterizeJob::Kt {
        max_torque_nm: (ceiling * 0.6).min(1.5),
        steps: 9,
        settle_s: 0.4,
        rate_hz: 100.0,
        leash_kp: 8.0,
        leash_kd: 0.5,
    }
}

struct Outcome {
    ok: bool,
    headline: String,
    csv_path: Option<String>,
    fitted_kt: Option<f32>,
    data: Option<crate::protocol::CharacterizeData>,
}

impl Outcome {
    fn into_result(self, motor_id: u8, run: &str) -> BatchResult {
        BatchResult {
            motor_id,
            run: run.to_string(),
            ok: self.ok,
            headline: self.headline,
            csv_path: self.csv_path,
            data: self.data,
        }
    }
}

/// Start one run and wait for its result. `None` means the batch was cancelled.
///
/// The wait loop beats the session's heartbeat, which is what makes the batch
/// thread the run's supervisor: stop beating — by dying, or by hanging — and the
/// watchdog disables the motor.
fn one_run(
    session: &Session,
    run: CharacterizeJob,
    envelope: RunEnvelope,
    cancel: &AtomicBool,
) -> Option<Outcome> {
    if session
        .send(Command::StartJob {
            spec: JobSpec::Characterize { run, envelope },
        })
        .is_err()
    {
        return Some(Outcome {
            ok: false,
            headline: "the motor worker stopped before the run began".to_string(),
            csv_path: None,
            fitted_kt: None,
            data: None,
        });
    }

    let deadline = Instant::now() + RUN_TIMEOUT;
    let mut failure: Option<String> = None;
    loop {
        session.heartbeat();

        if cancel.load(Ordering::Acquire) {
            session.stop();
            return None;
        }
        if Instant::now() >= deadline {
            session.stop();
            return Some(Outcome {
                ok: false,
                headline: format!("no result after {} s", RUN_TIMEOUT.as_secs()),
                csv_path: None,
                fitted_kt: None,
                data: None,
            });
        }

        for event in session.poll_events() {
            match event {
                Event::CharacterizeFinished {
                    data,
                    aborted,
                    fitted_kt,
                    csv_path,
                    ..
                } => {
                    // The run's own summary is the headline: whatever it decided
                    // was worth naming is what a reader of the batch wants.
                    let headline = match (&failure, &data) {
                        (Some(why), _) => why.clone(),
                        (None, Some(d)) => d
                            .summary
                            .first()
                            .map(|(k, v)| format!("{k} {v}"))
                            .unwrap_or_else(|| "no findings".to_string()),
                        (None, None) => "no measurement".to_string(),
                    };
                    return Some(Outcome {
                        ok: failure.is_none() && data.is_some(),
                        headline: if aborted {
                            format!("{headline} (stopped early)")
                        } else {
                            headline
                        },
                        csv_path,
                        fitted_kt,
                        data,
                    });
                }
                // Kept so the finished event can report it. `JobFailed` arrives
                // *after* `CharacterizeFinished`, so it cannot be waited for
                // instead — but a failure recorded before this run started would
                // be attributed to the wrong run, hence one variable per run.
                Event::JobFailed { message } => failure = Some(message),
                _ => {}
            }
        }
        std::thread::sleep(WAIT_TICK);
    }
}

fn push(progress: &Mutex<BatchProgress>, result: BatchResult) {
    if let Ok(mut p) = progress.lock() {
        p.results.push(result);
    }
}

fn bump_run(progress: &Mutex<BatchProgress>) {
    if let Ok(mut p) = progress.lock() {
        p.run_index += 1;
    }
}

/// Where a batch's files land, for a UI that wants to say so once rather than
/// per run.
pub fn output_dir() -> std::path::PathBuf {
    export::data_dir()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sim_spec(ids: Vec<u8>, runs: Vec<CharacterizeJob>) -> BatchSpec {
        BatchSpec {
            driver: DriverKind::Sim,
            interface: String::new(),
            bus: BusKind::Can,
            model: "rs04".to_string(),
            motors: ids
                .into_iter()
                .map(|id| BatchMotor {
                    id,
                    driver: None,
                    model: None,
                    max_torque_nm: None,
                })
                .collect(),
            runs,
            envelope: RunEnvelope {
                max_torque_nm: 3.0,
                kt: 0.0,
            },
            // Off: this is testing the batch's sequencing, and a Kt run per
            // motor would triple the wall clock for nothing.
            ramp_follows_ceiling: false,
            measure_kt_first: false,
        }
    }

    fn quick_breakaway() -> CharacterizeJob {
        CharacterizeJob::Breakaway {
            ramp_nm_per_s: 20.0,
            max_torque_nm: 1.0,
            positive: true,
            rate_hz: 500.0,
        }
    }

    fn wait_done(batch: &Batch, limit: Duration) -> BatchProgress {
        let deadline = Instant::now() + limit;
        loop {
            let p = batch.snapshot();
            if !p.running {
                return p;
            }
            assert!(Instant::now() < deadline, "batch did not finish: {p:?}");
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    /// Every motor is visited, and every run recorded — the batch's whole job.
    #[test]
    fn a_batch_visits_every_motor_in_turn() {
        let batch = spawn(sim_spec(vec![1, 2, 3], vec![quick_breakaway()]));
        let p = wait_done(&batch, Duration::from_secs(60));

        assert!(!p.cancelled, "should have run to completion");
        assert_eq!(p.error, None);
        assert_eq!(p.results.len(), 3, "one result per motor: {:?}", p.results);
        // In order, and each attributed to the motor it came from — a batch that
        // mislabels which motor a figure belongs to is worse than no batch.
        assert_eq!(
            p.results.iter().map(|r| r.motor_id).collect::<Vec<_>>(),
            vec![1, 2, 3]
        );
        assert!(
            p.results.iter().all(|r| r.ok),
            "the simulator should measure cleanly: {:?}",
            p.results
        );
    }

    /// Two runs per motor, so the ordering is motor-major: everything for motor
    /// 1, then everything for motor 2. Reconnecting per run instead would cost a
    /// bus open per run and is the obvious way to get this wrong.
    #[test]
    fn runs_are_grouped_by_motor() {
        let spec = sim_spec(
            vec![1, 2],
            vec![
                quick_breakaway(),
                CharacterizeJob::VelocitySweep {
                    half_span_rad: 0.05,
                    speed_rad_s: 0.5,
                    rate_hz: 500.0,
                    return_sweep: true,
                    bins: 4,
                },
            ],
        );
        let p = wait_done(&spawn(spec), Duration::from_secs(90));
        assert_eq!(
            p.results.iter().map(|r| r.motor_id).collect::<Vec<_>>(),
            vec![1, 1, 2, 2],
            "{:?}",
            p.results
        );
    }

    /// Cancelling must stop promptly, not after working through the rest.
    #[test]
    fn a_cancelled_batch_stops_without_finishing() {
        let batch = spawn(sim_spec(
            (1..=8).collect(),
            vec![CharacterizeJob::VelocitySweep {
                half_span_rad: 0.3,
                speed_rad_s: 0.05,
                rate_hz: 200.0,
                return_sweep: true,
                bins: 8,
            }],
        ));
        // Let it get into the first motor, then pull the plug.
        std::thread::sleep(Duration::from_millis(500));
        batch.cancel();

        let p = wait_done(&batch, Duration::from_secs(30));
        assert!(p.cancelled, "should report itself cancelled");
        assert!(
            p.results.len() < 8,
            "should not have completed all 8: {:?}",
            p.results
        );
    }

    /// A motor that will not open is that motor's result, and the batch carries
    /// on — a bench with one dead motor should still measure the others.
    #[test]
    fn a_motor_that_will_not_open_does_not_stop_the_batch() {
        let mut spec = sim_spec(vec![1, 2], vec![quick_breakaway()]);
        // The simulator rejects a model it does not know, so this fails at open
        // for every id without touching any hardware.
        spec.model = "definitely-not-a-preset".to_string();
        let p = wait_done(&spawn(spec), Duration::from_secs(30));

        assert_eq!(p.results.len(), 2, "both attempted: {:?}", p.results);
        assert!(p.results.iter().all(|r| !r.ok && r.run == "connect"));
        assert_eq!(p.error, None, "a per-motor failure is not a batch failure");
    }

    /// Each motor's own model must be used, not the batch's fallback.
    ///
    /// The fallback here is a preset the simulator rejects, so a motor that
    /// carries its own model opens and one that does not fails — which is a
    /// direct read on which value reached the driver. Cheaper and less ambiguous
    /// than inspecting scaling, and it is scaling this protects: an RS-04 opened
    /// with RS-00's MIT range reports every torque about 7 times too small, with
    /// nothing downstream able to tell (2026-08-07).
    #[test]
    fn each_motor_opens_with_its_own_model() {
        let mut spec = sim_spec(vec![], vec![quick_breakaway()]);
        spec.model = "not-a-preset".to_string();
        spec.motors = vec![
            BatchMotor {
                id: 1,
                driver: None,
                model: Some("rs04".to_string()),
                max_torque_nm: None,
            },
            BatchMotor {
                id: 2,
                driver: None,
                model: None,
                max_torque_nm: None,
            },
        ];
        let p = wait_done(&spawn(spec), Duration::from_secs(60));

        assert_eq!(p.results.len(), 2, "{:?}", p.results);
        let by_id = |id: u8| p.results.iter().find(|r| r.motor_id == id).unwrap();
        assert!(
            by_id(1).ok,
            "the motor with its own model should have opened: {:?}",
            by_id(1)
        );
        assert!(
            !by_id(2).ok && by_id(2).run == "connect",
            "the motor without one should have fallen back and failed: {:?}",
            by_id(2)
        );
    }

    /// A bus can carry motors two orders of magnitude apart — an RS-05 peaks at
    /// 5.5 N·m against an RS-04's 120 — so one ceiling for the batch is either
    /// unsafe for the small one or useless on the large one. Taking the smallest
    /// is the trap: it looks safe and quietly stops the large motor measuring
    /// anything.
    #[test]
    fn each_motor_gets_its_own_ceiling_and_its_ramps_follow() {
        let ramp = |nm: f32| CharacterizeJob::Breakaway {
            ramp_nm_per_s: 0.2,
            max_torque_nm: nm,
            positive: true,
            rate_hz: 200.0,
        };

        // A ramp sized for a big motor is cut to half the small motor's ceiling.
        assert_eq!(
            ramp_for(ramp(6.0), 0.55, false),
            ramp(0.275),
            "a 6 N·m ramp must not be applied inside a 0.55 N·m envelope"
        );
        // And the large motor keeps its own.
        assert_eq!(ramp_for(ramp(6.0), 12.0, false), ramp(6.0));

        // Only ever lowered: a deliberately gentle run is not pushed up to the
        // envelope just because there is room.
        assert_eq!(ramp_for(ramp(0.5), 12.0, false), ramp(0.5));

        // A default follows the motor *up* as well as down. Clamping only
        // downwards left eleven RS-03s with 6 N·m ceilings all ramping to 0.85
        // — half of the bar's stale RS-00 — and several never broke loose.
        assert_eq!(ramp_for(ramp(0.85), 6.0, true), ramp(3.0));
        // A figure the operator chose is still only ever lowered.
        assert_eq!(ramp_for(ramp(0.85), 6.0, false), ramp(0.85));
        // Following does not escape the ceiling either way.
        assert_eq!(ramp_for(ramp(20.0), 0.55, true), ramp(0.275));

        // A sentinel ceiling is not a limit. Passing the raw "0 means use the
        // default" field through capped every ramp at zero, and the run then
        // reported "not reached below 0.000 N·m" — obeyed exactly, and useless.
        assert_eq!(ramp_for(ramp(6.0), 0.0, false), ramp(6.0));
        assert_eq!(ramp_for(ramp(6.0), f32::NAN, false), ramp(6.0));

        // Runs that do not ramp against a ceiling are untouched.
        let sweep = CharacterizeJob::VelocitySweep {
            half_span_rad: 0.2,
            speed_rad_s: 0.05,
            rate_hz: 200.0,
            return_sweep: true,
            bins: 12,
        };
        assert_eq!(ramp_for(sweep, 0.55, false), sweep);
    }

    /// A CAN wire carries mixed vendors, so each motor opens as its own family.
    ///
    /// The batch's fallback is a family the simulator is not, so the motor
    /// carrying `Sim` opens and the one relying on the fallback does not — a
    /// direct read on which value reached the driver, as with the model.
    #[test]
    fn each_motor_opens_as_its_own_family() {
        let mut spec = sim_spec(vec![], vec![quick_breakaway()]);
        spec.driver = DriverKind::Robstride;
        spec.motors = vec![
            BatchMotor {
                id: 1,
                driver: Some(DriverKind::Sim),
                model: Some("rs04".to_string()),
                max_torque_nm: None,
            },
            BatchMotor {
                id: 2,
                driver: None,
                model: Some("rs04".to_string()),
                max_torque_nm: None,
            },
        ];
        let p = wait_done(&spawn(spec), Duration::from_secs(60));

        assert_eq!(p.results.len(), 2, "{:?}", p.results);
        let by_id = |id: u8| p.results.iter().find(|r| r.motor_id == id).unwrap();
        assert!(by_id(1).ok, "the simulated motor should have run: {:?}", by_id(1));
        assert!(
            !by_id(2).ok,
            "the one falling back to RobStride has no bus to open: {:?}",
            by_id(2)
        );
    }

    #[test]
    fn the_fraction_counts_runs_not_motors() {
        let p = BatchProgress {
            motor_count: 4,
            run_count: 2,
            motor_index: 1,
            run_index: 1,
            ..Default::default()
        };
        // Three of eight runs done.
        assert!((p.fraction() - 3.0 / 8.0).abs() < 1e-6, "{}", p.fraction());

        // A batch with nothing in it is not divided by zero.
        assert_eq!(BatchProgress::default().fraction(), 0.0);
    }
}
