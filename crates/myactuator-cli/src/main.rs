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
//! myactuator-cli -i can0 -m 1 params --toml --out dump.toml
//! ```
//!
//! Without `--kt`, torque values are raw current in **A** (current-units
//! mode); pass the datasheet torque constant to speak N·m.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};
use serde::Serialize;

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
    /// Read every known readable parameter in one pass (0x9A/0x9C/0x92/0x30/
    /// 0x42×4/0xB2/0xB5) and print it. A read that times out or fails is
    /// reported and omitted rather than aborting the whole dump.
    Params {
        /// Also emit a TOML dump to stdout (or to --out if given).
        #[arg(long)]
        toml: bool,
        /// Write the TOML dump to this file (implies --toml).
        #[arg(long)]
        out: Option<PathBuf>,
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
        Command::Params { toml, out } => {
            let params = AllParams::read_all(&mut motor);
            params.print();
            if *toml || out.is_some() {
                let text =
                    toml::to_string_pretty(&params).context("failed to serialize TOML dump")?;
                match out {
                    Some(path) => {
                        std::fs::write(path, &text)
                            .with_context(|| format!("failed to write {}", path.display()))?;
                        println!("\nwrote TOML dump to {}", path.display());
                    }
                    None => {
                        println!("\n--- TOML ---");
                        print!("{text}");
                    }
                }
            }
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

/// Every readable parameter this crate knows about, gathered in one pass.
/// Fields are `Option` because a motor model/firmware may lack a command
/// (e.g. `0x30`/`0x42` on non-motion-mode firmware); a failed read is
/// reported to stderr and just omitted rather than aborting the whole dump.
#[derive(Serialize)]
struct AllParams {
    motor_id: u8,
    // Status1 (0x9A)
    #[serde(skip_serializing_if = "Option::is_none")]
    temperature_c: Option<i8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    voltage_v: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    brake_released: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    error_flags: Option<u16>,
    // Status2 (0x9C)
    #[serde(skip_serializing_if = "Option::is_none")]
    current_a: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_dps: Option<i16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    angle_deg: Option<i16>,
    // Multi-turn absolute angle (0x92)
    #[serde(skip_serializing_if = "Option::is_none")]
    multi_turn_centideg: Option<i32>,
    // PID gains (0x30)
    #[serde(skip_serializing_if = "Option::is_none")]
    current_kp: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    current_ki: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_kp: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_ki: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    position_kp: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    position_ki: Option<u8>,
    // Acceleration/deceleration limits (0x42)
    #[serde(skip_serializing_if = "Option::is_none")]
    position_accel_dps_s: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    position_decel_dps_s: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_accel_dps_s: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_decel_dps_s: Option<i32>,
    // Version / model (0xB2 / 0xB5)
    #[serde(skip_serializing_if = "Option::is_none")]
    version_date_raw: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    motor_model: Option<String>,
}

impl AllParams {
    fn read_all<B: myactuator_driver::MyActuatorBus>(motor: &mut MyActuatorMotor<B>) -> Self {
        let status1 = try_read("Status1 (0x9A)", || motor.read_status1());
        let status2 = try_read("Status2 (0x9C)", || motor.read_status2());
        let pid = try_read("PID gains (0x30)", || motor.read_pid());

        Self {
            motor_id: motor.motor_id(),
            temperature_c: status1.map(|s| s.temperature_c),
            voltage_v: status1.map(|s| s.voltage_v()),
            brake_released: status1.map(|s| s.brake_released),
            error_flags: status1.map(|s| s.error.raw()),
            current_a: status2.map(|s| s.current_a()),
            speed_dps: status2.map(|s| s.speed_dps),
            angle_deg: status2.map(|s| s.angle_deg),
            multi_turn_centideg: try_read("multi-turn angle (0x92)", || {
                motor.read_multi_turn_centideg()
            }),
            current_kp: pid.map(|g| g.current_kp),
            current_ki: pid.map(|g| g.current_ki),
            speed_kp: pid.map(|g| g.speed_kp),
            speed_ki: pid.map(|g| g.speed_ki),
            position_kp: pid.map(|g| g.position_kp),
            position_ki: pid.map(|g| g.position_ki),
            position_accel_dps_s: try_read("position accel (0x42/0x00)", || {
                motor.read_acceleration(AccelIndex::PositionAccel)
            }),
            position_decel_dps_s: try_read("position decel (0x42/0x01)", || {
                motor.read_acceleration(AccelIndex::PositionDecel)
            }),
            speed_accel_dps_s: try_read("speed accel (0x42/0x02)", || {
                motor.read_acceleration(AccelIndex::SpeedAccel)
            }),
            speed_decel_dps_s: try_read("speed decel (0x42/0x03)", || {
                motor.read_acceleration(AccelIndex::SpeedDecel)
            }),
            version_date_raw: try_read("version date (0xB2)", || motor.read_version_date()),
            motor_model: try_read("motor model (0xB5)", || motor.read_motor_model()),
        }
    }

    fn print(&self) {
        fn fmt<T: std::fmt::Display>(v: Option<T>) -> String {
            v.map(|v| v.to_string()).unwrap_or_else(|| "—".to_string())
        }
        fn fmt_f32(v: Option<f32>) -> String {
            v.map(|v| format!("{v:.2}")).unwrap_or_else(|| "—".to_string())
        }

        println!("== MyActuator params (motor_id={}) ==", self.motor_id);
        println!(
            "[Status1]  temp={}°C  voltage={}V  brake_released={}  error=0x{}",
            fmt(self.temperature_c),
            fmt_f32(self.voltage_v),
            fmt(self.brake_released),
            self.error_flags
                .map(|e| format!("{e:04X}"))
                .unwrap_or_else(|| "----".to_string())
        );
        println!(
            "[Status2]  iq={}A  speed={}dps  angle={}°",
            fmt_f32(self.current_a),
            fmt(self.speed_dps),
            fmt(self.angle_deg)
        );
        println!(
            "[MultiTurn] 0x92 = {} centideg",
            fmt(self.multi_turn_centideg)
        );
        println!(
            "[PID]      current kp={} ki={}  speed kp={} ki={}  position kp={} ki={}",
            fmt(self.current_kp),
            fmt(self.current_ki),
            fmt(self.speed_kp),
            fmt(self.speed_ki),
            fmt(self.position_kp),
            fmt(self.position_ki)
        );
        println!(
            "[Accel]    pos_accel={}  pos_decel={}  speed_accel={}  speed_decel={}  (dps/s)",
            fmt(self.position_accel_dps_s),
            fmt(self.position_decel_dps_s),
            fmt(self.speed_accel_dps_s),
            fmt(self.speed_decel_dps_s)
        );
        println!(
            "[Version]  raw={}  model={}",
            fmt(self.version_date_raw),
            match self.motor_model.as_deref() {
                None => "—",
                Some("") => "<none> (empty on this firmware)",
                Some(m) => m,
            }
        );
    }
}

/// Run `f`, reporting a failure to stderr and returning `None` instead of
/// aborting — used by [`AllParams::read_all`] so one unsupported command
/// doesn't blank out the rest of the dump.
fn try_read<T>(label: &str, f: impl FnOnce() -> myactuator_driver::Result<T>) -> Option<T> {
    match f() {
        Ok(v) => Some(v),
        Err(e) => {
            eprintln!("warn: {label} read failed: {e}");
            None
        }
    }
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
