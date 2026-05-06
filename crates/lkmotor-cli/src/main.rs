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
//! - MG4005 (bare gimbal) is direct-drive → `--gear-ratio 1.0`; geared variants
//!   (e.g. `-i10`) use their reduction (`--gear-ratio 10`).
//! - `--baud` must match the motor's configured RS485 baud (LK V3 default is
//!   often 115200).
//! - `spin` / `torque` *latch* on the firmware; the CLI re-sends for the
//!   requested duration and the motor is disabled on exit (Drop), so motion
//!   stops when the command ends.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
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

    /// Gear ratio (MG4005 bare = 1.0, geared variants e.g. 10.0).
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
            println!(
                "voltage={:.2} V  temp={:.0}°C  error=0x{:08X}{}",
                st.voltage_v,
                st.temperature_c,
                st.error.raw(),
                if st.error.any() { " (FAULT)" } else { "" }
            );
            println!(
                "pos={:+.3} rad  vel={:+.3} rad/s  torque={:+.3} N·m  iq={:+.3} A",
                fb.position_rad, fb.velocity_rad_per_s, fb.torque_nm, fb.current_a
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
            motor.enable()?;
            run_for(*duration, || {
                let fb = motor.set_position(*position, *speed)?;
                Ok(fb)
            })?;
        }
        Command::Spin { velocity, duration } => {
            motor.enable()?;
            run_for(*duration, || motor.set_velocity(*velocity))?;
            // Drop disables, but stop explicitly for clarity.
            motor.disable()?;
        }
        Command::Torque { torque, duration } => {
            motor.enable()?;
            run_for(*duration, || motor.set_torque(*torque))?;
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
    }
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
/// printing the latest feedback periodically.
fn run_for<F>(duration: f32, mut tick: F) -> Result<()>
where
    F: FnMut() -> misa_actuator::Result<misa_actuator::MotorFeedback>,
{
    let running = Arc::new(AtomicBool::new(true));
    {
        let r = running.clone();
        let _ = ctrlc::set_handler(move || r.store(false, Ordering::SeqCst));
    }
    let start = Instant::now();
    let mut last_print = Instant::now();
    let period = Duration::from_millis(10);
    while running.load(Ordering::SeqCst) && start.elapsed().as_secs_f32() < duration {
        let loop_start = Instant::now();
        match tick() {
            Ok(fb) => {
                if last_print.elapsed() >= Duration::from_millis(200) {
                    println!(
                        "pos={:+.3} rad  vel={:+.3} rad/s  torque={:+.3} N·m",
                        fb.position_rad, fb.velocity_rad_per_s, fb.torque_nm
                    );
                    last_print = Instant::now();
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
