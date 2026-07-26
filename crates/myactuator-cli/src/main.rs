//! Test CLI for the `myactuator-driver` crate (MyActuator RMD, CAN V3).
//!
//! ```text
//! sudo ip link set can0 type can bitrate 1000000 up
//! myactuator-cli -i can0 scan
//! myactuator-cli -i can0 -m 1 status
//! myactuator-cli -i can0 -m 1 move-to 1.57 --speed 2 --duration 3
//! myactuator-cli -i can0 -m 1 spin 1.0 --duration 3
//! myactuator-cli -i can0 -m 1 --kt 0.83 torque 0.5 --duration 2
//! myactuator-cli -i can0 -m 1 mit --kp 10 --kd 1 --duration 5
//! ```
//!
//! Without `--kt`, torque values are raw current in **A** (current-units
//! mode); pass the datasheet torque constant to speak N·m.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};

use myactuator_driver::{scan_bus_on, MotorConfig, MotorFeedback, MyActuatorMotor};
use myactuator_protocol::AccelIndex;

/// clap-friendly mirror of [`AccelIndex`] (kept in `myactuator-protocol`,
/// which is `no_std` and doesn't depend on `clap`).
#[derive(clap::ValueEnum, Clone, Copy, Debug)]
enum AccelArg {
    PositionAccel,
    PositionDecel,
    SpeedAccel,
    SpeedDecel,
}

impl From<AccelArg> for AccelIndex {
    fn from(a: AccelArg) -> Self {
        match a {
            AccelArg::PositionAccel => AccelIndex::PositionAccel,
            AccelArg::PositionDecel => AccelIndex::PositionDecel,
            AccelArg::SpeedAccel => AccelIndex::SpeedAccel,
            AccelArg::SpeedDecel => AccelIndex::SpeedDecel,
        }
    }
}

#[derive(Parser, Debug)]
#[command(version, about = "Test CLI for MyActuator RMD servo motors (CAN V3)")]
struct Cli {
    /// SocketCAN interface name (e.g. can0).
    #[arg(short, long, default_value = "can0")]
    interface: String,

    /// Motor id on the bus (1..=32).
    #[arg(short, long, default_value_t = 1)]
    motor_id: u8,

    /// Output-frame torque constant Kt (N·m/A). 0 = current-units mode: the
    /// torque API carries amps instead of N·m.
    #[arg(long, default_value_t = 0.0)]
    kt: f32,

    /// Per-request timeout, in ms.
    #[arg(long, default_value_t = 100)]
    timeout_ms: u64,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Probe a range of motor ids for responding motors (0x9A read — no motion).
    Scan {
        /// First id to probe.
        #[arg(long, default_value_t = 1)]
        from: u8,
        /// Last id to probe.
        #[arg(long, default_value_t = 32)]
        to: u8,
    },
    /// One-shot measurement (0x92 position + 0x9C state) and Status1 health.
    Status,
    /// Read the multi-turn absolute angle (0x92), raw motor frame.
    Angle,
    /// Read firmware version date (0xB2) and motor model name (0xB5).
    Version,
    /// Read current/speed/position-loop PID gains (0x30), 0-255 normalized units.
    Pid,
    /// Read one acceleration/deceleration value (0x42), 1 dps/s.
    Accel {
        /// Which value to read.
        #[arg(value_enum)]
        index: AccelArg,
    },
    /// Anchor the soft zero at the current position. --rom persists to
    /// encoder ROM instead (wears flash; needs a reset to take effect).
    Zero {
        /// Write to encoder ROM (0x64) instead of the in-memory soft zero.
        #[arg(long)]
        rom: bool,
        /// After --rom, also send a system reset so the zero takes effect.
        #[arg(long)]
        reset: bool,
    },
    /// Stop motion but stay in closed loop (0x81).
    Stop,
    /// Turn the output off entirely (0x80).
    Shutdown,
    /// Restart the motor firmware (0x76).
    Reset,
    /// Release (default) or lock the holding brake.
    Brake {
        /// Lock instead of release.
        #[arg(long)]
        lock: bool,
    },
    /// Velocity command (rad/s, output shaft).
    Spin {
        #[arg(allow_hyphen_values = true)]
        velocity: f32,
        /// Run duration in seconds.
        #[arg(long, default_value_t = 3.0)]
        duration: f32,
    },
    /// Position move relative to the soft zero (anchored at start).
    MoveTo {
        #[arg(allow_hyphen_values = true)]
        position: f32,
        /// Max output-shaft speed (rad/s).
        #[arg(long, default_value_t = 2.0)]
        speed: f32,
        /// Hold duration in seconds.
        #[arg(long, default_value_t = 3.0)]
        duration: f32,
    },
    /// Torque command (N·m with --kt, raw A without). Re-issued in a loop —
    /// 0xA1 latches on the firmware, so a one-shot would keep pushing forever.
    Torque {
        #[arg(allow_hyphen_values = true)]
        value: f32,
        /// Run duration in seconds.
        #[arg(long, default_value_t = 2.0)]
        duration: f32,
    },
    /// Motion-mode (MIT) control loop on the 0x400 channel (RMD-X V3 only).
    Mit {
        #[arg(long, allow_hyphen_values = true, default_value_t = 0.0)]
        pos: f32,
        #[arg(long, allow_hyphen_values = true, default_value_t = 0.0)]
        vel: f32,
        #[arg(long, default_value_t = 0.0)]
        kp: f32,
        #[arg(long, default_value_t = 0.0)]
        kd: f32,
        #[arg(long, allow_hyphen_values = true, default_value_t = 0.0)]
        tau: f32,
        /// Hold duration in seconds (omit for Ctrl-C).
        #[arg(long)]
        duration: Option<f32>,
    },
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    let cli = Cli::parse();

    let config = if cli.kt > 0.0 {
        MotorConfig::new(cli.kt)
    } else {
        MotorConfig::current_units()
    };
    let mut motor = MyActuatorMotor::open(&cli.interface, cli.motor_id, config)
        .with_context(|| format!("failed to open CAN interface {}", cli.interface))?;
    motor.set_timeout(Duration::from_millis(cli.timeout_ms))?;

    match &cli.command {
        Command::Scan { from, to } => {
            if to < from {
                bail!("--to must be >= --from");
            }
            println!("scanning motor ids {from}..={to} on {} ...", cli.interface);
            let found = scan_bus_on(
                motor.bus(),
                *from..=*to,
                Duration::from_millis(cli.timeout_ms),
                None,
            )?;
            if found.is_empty() {
                println!("no motors responded");
            } else {
                for id in found {
                    println!("  motor id {id} (0x{id:02X}) responded");
                }
            }
        }
        Command::Status => {
            let st = motor.read_status()?;
            println!(
                "voltage={:.1} V  temp={} °C  brake_released={}  err=0x{:04X}{}",
                st.voltage_v,
                st.temperature_c,
                st.brake_released,
                st.error.raw(),
                if st.error.any() { "  (FAULT)" } else { "" }
            );
            let fb = motor.measure()?;
            println!("{}", fmt_fb(fb));
        }
        Command::Angle => {
            let centideg = motor.read_multi_turn_centideg()?;
            println!(
                "multi-turn angle = {:.2}° ({:.4} rad, raw {} centideg)",
                centideg as f32 / 100.0,
                (centideg as f32 / 100.0).to_radians(),
                centideg
            );
        }
        Command::Version => {
            let date = motor.read_version_date()?;
            let model = motor.read_motor_model()?;
            // The manual's own example (20211126) is 8 digits (YYYYMMDD), but
            // some firmware appends extra trailing digits (observed: a 2-digit
            // revision). Show the raw value plus a best-effort YYYYMMDD guess
            // from the leading 8 digits rather than assuming a fixed width.
            let digits = if date == 0 { 1 } else { (date as f64).log10() as u32 + 1 };
            let guess = if digits > 8 {
                let suffix_digits = digits - 8;
                Some((date / 10u32.pow(suffix_digits), date % 10u32.pow(suffix_digits)))
            } else {
                None
            };
            print!("model={} (empty if unsupported by this firmware)  ", if model.is_empty() { "<none>" } else { &model });
            match guess {
                Some((ymd, rev)) => println!(
                    "raw version={date}  guessed date={:04}-{:02}-{:02} rev={rev:02}",
                    ymd / 10_000, (ymd / 100) % 100, ymd % 100
                ),
                None => println!(
                    "raw version={date}  guessed date={:04}-{:02}-{:02}",
                    date / 10_000, (date / 100) % 100, date % 100
                ),
            }
        }
        Command::Pid => {
            let g = motor.read_pid()?;
            println!(
                "current: kp={} ki={}  speed: kp={} ki={}  position: kp={} ki={}  (0-255 normalized, see manual for the per-model scale)",
                g.current_kp, g.current_ki, g.speed_kp, g.speed_ki, g.position_kp, g.position_ki
            );
        }
        Command::Accel { index } => {
            let dps_s = motor.read_acceleration((*index).into())?;
            println!("{index:?} = {dps_s} dps/s");
        }
        Command::Zero { rom, reset } => {
            if *rom {
                motor.set_zero_rom()?;
                println!("zero written to encoder ROM (0x64).");
                if *reset {
                    motor.system_reset()?;
                    println!("system reset sent — zero takes effect after reboot.");
                } else {
                    println!("note: takes effect only after a restart (myactuator-cli reset).");
                }
            } else {
                motor.rezero()?;
                println!(
                    "soft zero anchored at current position (offset {} centideg, in-memory).",
                    motor.zero_centideg().unwrap_or(0)
                );
            }
        }
        Command::Stop => {
            motor.stop()?;
            println!("stopped (0x81).");
        }
        Command::Shutdown => {
            motor.shutdown()?;
            println!("shut down (0x80).");
        }
        Command::Reset => {
            motor.system_reset()?;
            println!("system reset sent (0x76).");
        }
        Command::Brake { lock } => {
            if *lock {
                motor.brake_lock()?;
                println!("brake locked (0x78).");
            } else {
                motor.brake_release()?;
                println!("brake released (0x77).");
            }
        }
        Command::Spin { velocity, duration } => {
            let r = control_loop(Some(*duration), || motor.set_velocity(*velocity));
            motor.stop()?;
            r?;
        }
        Command::MoveTo {
            position,
            speed,
            duration,
        } => {
            motor.rezero()?;
            let r = control_loop(Some(*duration), || motor.set_position(*position, *speed));
            motor.stop()?;
            r?;
        }
        Command::Torque { value, duration } => {
            let r = control_loop(Some(*duration), || motor.set_torque(*value));
            motor.stop()?;
            r?;
        }
        Command::Mit {
            pos,
            vel,
            kp,
            kd,
            tau,
            duration,
        } => {
            motor.rezero()?;
            let kt = motor.config().torque_constant_nm_per_a;
            let r = control_loop(*duration, || {
                motor
                    .motion_control(*pos, *vel, *kp, *kd, *tau)
                    .map(|fb| MotorFeedback {
                        position_rad: fb.position_rad,
                        velocity_rad_per_s: fb.velocity_rad_per_s,
                        torque_nm: fb.torque_nm,
                        current_a: fb.torque_nm / kt,
                        temperature_c: 0,
                    })
            });
            motor.stop()?;
            r?;
        }
    }
    Ok(())
}

fn control_loop<F>(duration: Option<f32>, mut tick: F) -> Result<()>
where
    F: FnMut() -> myactuator_driver::Result<MotorFeedback>,
{
    let running = Arc::new(AtomicBool::new(true));
    {
        let r = running.clone();
        let _ = ctrlc::set_handler(move || r.store(false, Ordering::SeqCst));
    }

    let start = Instant::now();
    let mut last_print = Instant::now();
    let period = Duration::from_millis(10);
    while running.load(Ordering::SeqCst) {
        if let Some(d) = duration {
            if start.elapsed().as_secs_f32() >= d {
                break;
            }
        }
        let loop_start = Instant::now();
        match tick() {
            Ok(fb) => {
                if last_print.elapsed() >= Duration::from_millis(200) {
                    println!("{}", fmt_fb(fb));
                    last_print = Instant::now();
                }
            }
            Err(e) => {
                eprintln!("control error: {e}");
                break;
            }
        }
        if let Some(rem) = period.checked_sub(loop_start.elapsed()) {
            std::thread::sleep(rem);
        }
    }
    Ok(())
}

fn fmt_fb(fb: MotorFeedback) -> String {
    format!(
        "pos={:+.3} rad  vel={:+.3} rad/s  tau={:+.3} Nm  iq={:+.2} A  T={} °C",
        fb.position_rad, fb.velocity_rad_per_s, fb.torque_nm, fb.current_a, fb.temperature_c
    )
}
