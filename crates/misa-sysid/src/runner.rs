//! Real-time chirp excitation runner over the [`Actuator`] trait.
//!
//! Motor-agnostic: drives any `&mut dyn Actuator` with a chirp on a chosen
//! channel and logs the synchronized command + response for identification.

use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use misa_actuator::{Actuator, MotorFeedback, Result, RunMode};

use crate::chirp::Chirp;

/// Which command channel the chirp drives.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Excitation {
    /// Position reference (closed-loop). Chirp rides on the start position, so
    /// motion stays bounded around it — the safest channel. `max_speed_rad_s`
    /// caps the position controller.
    Position { max_speed_rad_s: f32 },
    /// Velocity reference. Zero-mean, but net position drifts; bounded duration.
    Velocity,
    /// Torque / current. Open-loop plant excitation (identifies ω/τ); position
    /// can drift or run away — use small amplitude and short runs.
    Torque,
    /// MIT position reference chirp with fixed impedance `kp` / `kd` — a
    /// closed-loop impedance identification (reference → output).
    MitPosition { kp: f32, kd: f32 },
    /// MIT **torque feed-forward** chirp with a small `kp` / `kd` position
    /// "leash" that holds the motor near its start position while the
    /// feed-forward excites the plant. This is the safe way to identify the
    /// open-loop plant (ω/τ) on an impedance-controlled motor: small gains keep
    /// it from running away while staying close to open loop. Native MIT only
    /// (DAMIAO / Robstride); on LK the gains/feed-forward are ignored.
    MitTorque { kp: f32, kd: f32 },
}

impl Excitation {
    fn run_mode(&self) -> RunMode {
        match self {
            Excitation::Position { .. } => RunMode::Position,
            Excitation::Velocity => RunMode::Velocity,
            Excitation::Torque => RunMode::Torque,
            Excitation::MitPosition { .. } | Excitation::MitTorque { .. } => RunMode::Mit,
        }
    }

    /// Whether the channel holds a center position (chirp added to / referenced
    /// against the start position).
    fn is_position_like(&self) -> bool {
        matches!(
            self,
            Excitation::Position { .. }
                | Excitation::MitPosition { .. }
                | Excitation::MitTorque { .. }
        )
    }
}

/// One synchronized command/response sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// Time since the sweep started (s).
    pub t_s: f32,
    /// Commanded value sent to the actuator (the identification *input* `u`):
    /// the absolute position for position channels, else the raw chirp value.
    pub cmd: f32,
    /// Measured position (rad).
    pub position_rad: f32,
    /// Measured velocity (rad/s).
    pub velocity_rad_per_s: f32,
    /// Measured torque (N·m).
    pub torque_nm: f32,
    /// Measured motor current (A; NaN if the driver doesn't report it).
    pub current_a: f32,
}

/// Result of a chirp run: the sample log plus the excitation metadata.
#[derive(Debug, Clone)]
pub struct ChirpLog {
    pub samples: Vec<Sample>,
    pub excitation: Excitation,
    pub chirp: Chirp,
    /// Achieved average loop rate (samples / total time), Hz.
    pub achieved_rate_hz: f32,
}

impl ChirpLog {
    /// Write the raw sample log as CSV (one header row + one row per sample).
    pub fn write_csv(&self, w: &mut dyn Write) -> io::Result<()> {
        writeln!(w, "t_s,cmd,position_rad,velocity_rad_per_s,torque_nm,current_a")?;
        for s in &self.samples {
            writeln!(
                w,
                "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
                s.t_s, s.cmd, s.position_rad, s.velocity_rad_per_s, s.torque_nm, s.current_a
            )?;
        }
        Ok(())
    }
}

/// Run a chirp excitation on `act` and return the logged response.
///
/// Sets the run mode for `exc`, enables the motor, sweeps the chirp at
/// `target_rate_hz` for `chirp.duration_s` (or until `abort` is set), then
/// commands a safe stop and disables. Real timestamps are recorded, so a bus
/// that can't sustain `target_rate_hz` still yields a usable (non-uniform) log
/// — see [`ChirpLog::achieved_rate_hz`].
pub fn run_chirp(
    act: &mut dyn Actuator,
    chirp: &Chirp,
    exc: Excitation,
    target_rate_hz: f32,
    abort: &AtomicBool,
) -> Result<ChirpLog> {
    act.set_run_mode(exc.run_mode())?;
    let enable_fb = act.enable()?;

    // For position-referenced channels, ride the chirp on the current position.
    let center = if exc.is_position_like() {
        enable_fb.position_rad
    } else {
        0.0
    };

    let period = Duration::from_secs_f32(1.0 / target_rate_hz.max(1.0));
    let mut samples = Vec::with_capacity((chirp.duration_s * target_rate_hz) as usize + 1);

    let start = Instant::now();
    loop {
        let t = start.elapsed().as_secs_f32();
        if t >= chirp.duration_s || abort.load(Ordering::Relaxed) {
            break;
        }
        let iter_start = Instant::now();

        let u = chirp.value(t);
        let (cmd, fb) = command(act, exc, center, u)?;
        samples.push(sample(t, cmd, &fb));

        if let Some(rem) = period.checked_sub(iter_start.elapsed()) {
            std::thread::sleep(rem);
        }
    }
    let total = start.elapsed().as_secs_f32().max(1e-6);

    // Safe stop, then coast.
    let _ = safe_stop(act, exc, center);
    let _ = act.disable();

    Ok(ChirpLog {
        achieved_rate_hz: samples.len() as f32 / total,
        samples,
        excitation: exc,
        chirp: *chirp,
    })
}

/// Summary returned by [`run_chirp_to_csv`].
#[derive(Debug, Clone, Copy)]
pub struct RunReport {
    pub n_samples: usize,
    pub achieved_rate_hz: f32,
    pub n_freqs: usize,
}

/// Convenience for CLIs: run a chirp, write the raw log + the estimated Bode to
/// the two writers, and return a short summary. Keeps the per-CLI glue minimal.
pub fn run_chirp_to_csv(
    act: &mut dyn Actuator,
    chirp: &Chirp,
    exc: Excitation,
    target_rate_hz: f32,
    abort: &AtomicBool,
    raw_csv: &mut dyn Write,
    bode_csv: &mut dyn Write,
) -> Result<RunReport> {
    let log = run_chirp(act, chirp, exc, target_rate_hz, abort)?;
    log.write_csv(raw_csv)?; // io::Error → misa Error via From
    let fr = log.frf();
    fr.write_csv(bode_csv)?;
    Ok(RunReport {
        n_samples: log.samples.len(),
        achieved_rate_hz: log.achieved_rate_hz,
        n_freqs: fr.freqs_hz.len(),
    })
}

fn command(
    act: &mut dyn Actuator,
    exc: Excitation,
    center: f32,
    u: f32,
) -> Result<(f32, MotorFeedback)> {
    Ok(match exc {
        Excitation::Position { max_speed_rad_s } => {
            let cmd = center + u;
            (cmd, act.set_position(cmd, max_speed_rad_s)?)
        }
        Excitation::Velocity => (u, act.set_velocity(u)?),
        Excitation::Torque => (u, act.set_torque(u)?),
        Excitation::MitPosition { kp, kd } => {
            let cmd = center + u;
            (cmd, act.mit_control(cmd, 0.0, kp, kd, 0.0)?)
        }
        // Hold at center with a small leash (kp, kd); chirp the torque FF.
        Excitation::MitTorque { kp, kd } => (u, act.mit_control(center, 0.0, kp, kd, u)?),
    })
}

fn safe_stop(act: &mut dyn Actuator, exc: Excitation, center: f32) -> Result<MotorFeedback> {
    match exc {
        Excitation::Position { max_speed_rad_s } => act.set_position(center, max_speed_rad_s),
        Excitation::Velocity => act.set_velocity(0.0),
        Excitation::Torque => act.set_torque(0.0),
        Excitation::MitPosition { kp, kd } | Excitation::MitTorque { kp, kd } => {
            act.mit_control(center, 0.0, kp, kd, 0.0)
        }
    }
}

fn sample(t: f32, cmd: f32, fb: &MotorFeedback) -> Sample {
    Sample {
        t_s: t,
        cmd,
        position_rad: fb.position_rad,
        velocity_rad_per_s: fb.velocity_rad_per_s,
        torque_nm: fb.torque_nm,
        current_a: fb.current_a,
    }
}
