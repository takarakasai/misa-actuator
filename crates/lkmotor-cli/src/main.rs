//! Test CLI for the `lkmotor-driver` crate — LKMTech V3 motors (MG4005 etc.)
//! over an RS485 serial port.
//!
//! ```text
//! lkmotor-cli -i /dev/ttyUSB0 -m 1 scan
//! lkmotor-cli -i /dev/ttyUSB0 -m 1 status
//! lkmotor-cli -i /dev/ttyUSB0 -m 1 zero
//! lkmotor-cli -i /dev/ttyUSB0 -m 1 move-to 1.57 --speed 5 --duration 3
//! lkmotor-cli -i /dev/ttyUSB0 -m 1 spin 2.0 --duration 3
//! ```
//!
//! Notes:
//! - MG4005 has a **10:1** reduction → use `--gear-ratio 10` for output-frame
//!   units (rad / rad/s at the gearbox output). `--gear-ratio 1` reports raw
//!   motor-shaft units (10× the output).
//! - `--baud` must match the motor's configured RS485 baud (the MG4005 here
//!   runs at 1000000; LK V3 default is often 115200).
//! - `spin` / `torque` *latch* on the firmware; the CLI re-sends for the
//!   requested duration and the motor is disabled on exit (Drop), so motion
//!   stops when the command ends.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};

use lkmotor_driver::{LkCommands, LkMotor, MotorConfig, MotorId, Rs485Driver};
use misa_actuator::Actuator;

#[derive(Parser, Debug)]
#[command(version, about = "Test CLI for LKMTech V3 servo motors (MG4005 / MG / MS / RMD-X)")]
struct Cli {
    /// RS485 serial device (e.g. /dev/ttyUSB0).
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    interface: String,

    /// Motor id on the bus (1..=32).
    #[arg(short, long, default_value_t = 1)]
    motor_id: u8,

    /// Serial baud rate (must match the motor's configured RS485 baud).
    #[arg(long, default_value_t = 115_200)]
    baud: u32,

    /// Gear ratio (MG4005 = 10.0 for output-frame units; 1.0 = raw motor shaft).
    #[arg(long, default_value_t = 1.0)]
    gear_ratio: f32,

    /// Torque constant Kt (N·m/A). 0 = current-units mode (torque API ≈ amps).
    #[arg(long, default_value_t = 0.0)]
    kt: f32,

    /// Per-request response timeout, in ms.
    #[arg(long, default_value_t = 50)]
    timeout_ms: u64,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Probe a range of motor ids (side-effect-free `0x9A` reads).
    Scan {
        #[arg(long, default_value_t = 1)]
        from: u8,
        #[arg(long, default_value_t = 32)]
        to: u8,
    },
    /// Read status (voltage, temperature, error) and a motion snapshot.
    Status,
    /// Engage the servo (MotorRun + hold at current position).
    Enable,
    /// Stop the servo (motor coasts).
    Disable,
    /// Anchor the current physical position as zero (software anchor).
    Zero,
    /// Move to an absolute position (rad, relative to the zero anchor).
    MoveTo {
        #[arg(allow_hyphen_values = true)]
        position: f32,
        /// Max speed (rad/s, output frame).
        #[arg(long, default_value_t = 5.0)]
        speed: f32,
        /// Hold/observe duration in seconds.
        #[arg(long, default_value_t = 3.0)]
        duration: f32,
    },
    /// Velocity command (rad/s, output frame); latches, re-sent for duration.
    Spin {
        #[arg(allow_hyphen_values = true)]
        velocity: f32,
        #[arg(long, default_value_t = 3.0)]
        duration: f32,
    },
    /// Torque command (N·m, or amps in current-units mode); latches.
    Torque {
        #[arg(allow_hyphen_values = true)]
        torque: f32,
        #[arg(long, default_value_t = 2.0)]
        duration: f32,
    },
    /// Read the position / speed / current PID triples.
    Pid,
    /// Chirp-excitation system identification → CSV log + Bode (frequency
    /// response). Drives a swept sine on the chosen channel and estimates the
    /// frequency response. Position channel is safest (bounded around current
    /// pos). Use `--gear-ratio 10` for MG4005 output-frame units.
    Chirp {
        /// Channel: position | velocity | torque | mit | mit-torque.
        #[arg(long, default_value = "position")]
        channel: String,
        /// Start frequency (Hz).
        #[arg(long, default_value_t = 0.5)]
        f0: f32,
        /// End frequency (Hz).
        #[arg(long, default_value_t = 30.0)]
        f1: f32,
        /// Sweep duration (s).
        #[arg(long, default_value_t = 10.0)]
        duration: f32,
        /// Amplitude (rad / rad·s / N·m by channel). Keep small.
        #[arg(long, default_value_t = 0.15)]
        amp: f32,
        /// Logarithmic sweep (default linear).
        #[arg(long)]
        log: bool,
        /// Target loop rate (Hz).
        #[arg(long, default_value_t = 500.0)]
        rate: f32,
        /// Position channel max speed (rad/s).
        #[arg(long, default_value_t = 5.0)]
        max_speed: f32,
        /// MIT channel kp / kd.
        #[arg(long, default_value_t = 0.0)]
        kp: f32,
        #[arg(long, default_value_t = 0.0)]
        kd: f32,
        /// Raw log CSV path.
        #[arg(long, default_value = "chirp.csv")]
        out: String,
        /// Bode CSV path.
        #[arg(long, default_value = "chirp_bode.csv")]
        bode: String,
    },
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    let cli = Cli::parse();
    let mut motor = open_motor(&cli)?;

    match &cli.command {
        Command::Scan { from, to } => {
            println!("scanning ids {from}..={to} on {} ...", cli.interface);
            let timeout = Duration::from_millis(cli.timeout_ms);
            let found = motor.scan_bus(*from..=*to, timeout)?;
            if found.is_empty() {
                println!("no motors responded");
            } else {
                for id in found {
                    println!("  motor id {id} responded");
                }
            }
        }
        Command::Status => {
            let st = motor.read_status()?;
            let fb = motor.measure()?;
            let abs = motor.read_absolute_position()?;
            println!(
                "voltage={:.2} V  temp={:.0}°C  error=0x{:08X}{}",
                st.voltage_v,
                st.temperature_c,
                st.error.raw(),
                if st.error.any() { " (FAULT)" } else { "" }
            );
            // pos is the absolute multi-turn angle (power-on frame), the same
            // frame `move-to` targets.
            println!(
                "pos(abs)={:+.3} rad  vel={:+.3} rad/s  torque={:+.3} N·m  iq={:+.3} A",
                abs, fb.velocity_rad_per_s, fb.torque_nm, fb.current_a
            );
        }
        Command::Enable => {
            let fb = motor.enable()?;
            println!("enabled (holding at current position). pos={:+.3} rad", fb.position_rad);
            println!("note: the motor disables on program exit; use a motion command to act.");
        }
        Command::Disable => {
            motor.disable()?;
            println!("disabled.");
        }
        Command::Zero => {
            motor.set_zero()?;
            println!("zero anchored at current position.");
        }
        Command::MoveTo {
            position,
            speed,
            duration,
        } => {
            // Absolute move in the motor's power-on multi-turn frame: `move-to X`
            // always targets the same physical position (no rezero), matching
            // the `status` pos readout.
            motor.run()?; // wake the controller without re-anchoring
            run_loop(*duration, || {
                let fb = motor.move_to_absolute(*position, *speed)?;
                let abs = motor.read_absolute_position()?;
                Ok(format!(
                    "pos(abs)={:+.3} rad  vel={:+.3} rad/s  torque={:+.3} N·m",
                    abs, fb.velocity_rad_per_s, fb.torque_nm
                ))
            })?;
        }
        Command::Spin { velocity, duration } => {
            motor.run()?;
            run_loop(*duration, || {
                let fb = motor.set_velocity(*velocity)?;
                Ok(format!(
                    "vel={:+.3} rad/s  torque={:+.3} N·m",
                    fb.velocity_rad_per_s, fb.torque_nm
                ))
            })?;
            motor.disable()?;
        }
        Command::Torque { torque, duration } => {
            motor.run()?;
            run_loop(*duration, || {
                let fb = motor.set_torque(*torque)?;
                Ok(format!(
                    "vel={:+.3} rad/s  torque={:+.3} N·m  iq={:+.3} A",
                    fb.velocity_rad_per_s, fb.torque_nm, fb.current_a
                ))
            })?;
            motor.disable()?;
        }
        Command::Pid => {
            let id = MotorId::new(cli.motor_id).context("invalid motor id")?;
            let bus = motor.bus();
            let pos = bus.read_position_pid(id)?;
            let spd = bus.read_speed_pid(id)?;
            let cur = bus.read_current_pid(id)?;
            println!("position PID: kp={} ki={} kd={}", pos.kp, pos.ki, pos.kd);
            println!("speed    PID: kp={} ki={} kd={}", spd.kp, spd.ki, spd.kd);
            println!("current  PID: kp={} ki={} kd={}", cur.kp, cur.ki, cur.kd);
        }
        Command::Chirp {
            channel,
            f0,
            f1,
            duration,
            amp,
            log,
            rate,
            max_speed,
            kp,
            kd,
            out,
            bode,
        } => {
            chirp_cmd(
                &mut motor, channel, *f0, *f1, *duration, *amp, *log, *rate, *max_speed, *kp, *kd,
                out, bode,
            )?;
        }
    }
    Ok(())
}

/// Shared chirp-identification command: build the excitation, run it, and write
/// the raw log + Bode CSVs. Generic over the (already-built) actuator.
#[allow(clippy::too_many_arguments)]
fn chirp_cmd(
    act: &mut dyn Actuator,
    channel: &str,
    f0: f32,
    f1: f32,
    duration: f32,
    amp: f32,
    log_sweep: bool,
    rate: f32,
    max_speed: f32,
    kp: f32,
    kd: f32,
    out: &str,
    bode: &str,
) -> Result<()> {
    use misa_sysid::{run_chirp_to_csv, Chirp, Excitation, Sweep};

    let exc = match channel.to_lowercase().as_str() {
        "position" | "pos" => Excitation::Position {
            max_speed_rad_s: max_speed,
        },
        "velocity" | "vel" => Excitation::Velocity,
        "torque" | "current" => Excitation::Torque,
        "mit" => Excitation::MitPosition { kp, kd },
        "mit-torque" | "mittorque" => Excitation::MitTorque { kp, kd },
        other => {
            bail!("unknown --channel '{other}' (position|velocity|torque|mit|mit-torque)")
        }
    };
    let chirp = Chirp {
        f_start_hz: f0,
        f_end_hz: f1,
        duration_s: duration,
        amplitude: amp,
        bias: 0.0,
        sweep: if log_sweep {
            Sweep::Logarithmic
        } else {
            Sweep::Linear
        },
    };

    let abort = Arc::new(AtomicBool::new(false));
    {
        let a = abort.clone();
        let _ = ctrlc::set_handler(move || a.store(true, Ordering::SeqCst));
    }
    println!("chirp {f0}->{f1} Hz over {duration}s, amp {amp}, channel={channel} (Ctrl-C aborts)...");

    let mut raw =
        std::io::BufWriter::new(std::fs::File::create(out).with_context(|| format!("create {out}"))?);
    let mut bod = std::io::BufWriter::new(
        std::fs::File::create(bode).with_context(|| format!("create {bode}"))?,
    );
    let rep = run_chirp_to_csv(act, &chirp, exc, rate, &abort, &mut raw, &mut bod)?;

    println!(
        "collected {} samples @ {:.0} Hz (target {:.0}), median coherence {:.2}",
        rep.n_samples, rep.achieved_rate_hz, rate, rep.median_coherence
    );
    if rep.achieved_rate_hz < rate * 0.8 {
        eprintln!(
            "note: achieved rate is well below target — usable band limited to ~{:.0} Hz",
            rep.achieved_rate_hz / 2.0
        );
    }
    if rep.median_coherence < 0.5 {
        eprintln!(
            "WARNING: low coherence ({:.2}) — poor identification. Is the motor enabled and \
             fault-free? Try a larger --amp, or check wiring.",
            rep.median_coherence
        );
    }
    println!("wrote {out} (raw log) and {bode} ({} freq points)", rep.n_freqs);
    Ok(())
}

fn open_motor(cli: &Cli) -> Result<LkMotor<Rs485Driver>> {
    let id = MotorId::new(cli.motor_id)
        .with_context(|| format!("invalid motor id {} (must be 1..=32)", cli.motor_id))?;
    let config = if cli.kt > 0.0 {
        MotorConfig::new(cli.gear_ratio, cli.kt)
    } else {
        MotorConfig::current_units(cli.gear_ratio)
    };
    LkMotor::open_rs485(
        &cli.interface,
        cli.baud,
        id,
        config,
        Duration::from_millis(cli.timeout_ms),
    )
    .with_context(|| format!("failed to open {} @ {} baud", cli.interface, cli.baud))
}

/// Re-issue a command at ~100 Hz for `duration` seconds (or until Ctrl-C),
/// printing the status line returned by `tick` at most every 200 ms.
///
/// `tick` is responsible for sending the command and formatting the line; this
/// just handles the loop timing, Ctrl-C, and print throttling.
fn run_loop<F>(duration: f32, mut tick: F) -> Result<()>
where
    F: FnMut() -> Result<String>,
{
    let running = Arc::new(AtomicBool::new(true));
    {
        let r = running.clone();
        let _ = ctrlc::set_handler(move || r.store(false, Ordering::SeqCst));
    }
    let start = Instant::now();
    let mut last_print: Option<Instant> = None;
    let period = Duration::from_millis(10);
    while running.load(Ordering::SeqCst) && start.elapsed().as_secs_f32() < duration {
        let loop_start = Instant::now();
        match tick() {
            Ok(line) => {
                if last_print.is_none_or(|t| t.elapsed() >= Duration::from_millis(200)) {
                    println!("{line}");
                    last_print = Some(Instant::now());
                }
            }
            Err(e) => {
                eprintln!("command error: {e}");
                break;
            }
        }
        if let Some(rem) = period.checked_sub(loop_start.elapsed()) {
            std::thread::sleep(rem);
        }
    }
    Ok(())
}
