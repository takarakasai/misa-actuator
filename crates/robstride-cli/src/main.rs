//! Test CLI for the `robstride-driver` crate.
//!
//! Examples:
//! ```text
//! robstride-cli -i can0 scan
//! robstride-cli -i can0 -m 1 status
//! robstride-cli -i can0 -m 1 enable
//! robstride-cli -i can0 -m 1 move-to 1.57
//! robstride-cli -i can0 -m 1 spin 3.0 --duration 5
//! robstride-cli -i can0 -m 1 mit --pos 0.0 --vel 0.0 --kp 50 --kd 1.0
//! ```

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};

use robstride_driver::{dump_bus, scan_bus, Motor, MotorModel, ParamIndex, RunMode, DEFAULT_HOST_ID};

mod param_commands;

#[derive(Parser, Debug)]
#[command(version, about = "Test CLI for the Robstride CAN servo motor driver")]
struct Cli {
    /// SocketCAN interface name (e.g. can0).
    #[arg(short, long, default_value = "can0")]
    interface: String,

    /// Motor CAN ID (1..=127). Required for per-motor commands.
    #[arg(short, long, default_value_t = 1)]
    motor_id: u8,

    /// Host CAN ID (must be greater than every motor id on the bus).
    #[arg(long, default_value_t = DEFAULT_HOST_ID)]
    host_id: u8,

    /// Motor model — accepts `RS-05`, `rs05`, `Edulite05`, etc.
    #[arg(long, default_value = "Edulite05")]
    model: String,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Read a single status frame.
    Status,
    /// Snapshot of pos/vel/Iq/τ/Vbus (and temperature, optionally) using
    /// per-parameter reads. Safe to call in any run mode by default; pass
    /// `--with-temp` to also fetch temperature (requires a status frame, which
    /// sends a zero MIT command and can disrupt non-MIT control loops).
    Info {
        /// Also read temperature (sends a status frame).
        #[arg(long)]
        with_temp: bool,
    },
    /// Enable the motor (must be sent before motion commands).
    Enable,
    /// Disable the motor (coast).
    Disable,
    /// Set the current position as the mechanical zero.
    SetZero,
    /// Move to an absolute position (rad) using the position-control mode.
    MoveTo {
        /// Target position in radians.
        #[arg(allow_hyphen_values = true)]
        position: f32,
        /// Speed limit (rad/s).
        #[arg(long, default_value_t = 5.0)]
        speed: f32,
        /// Position tolerance to declare arrival (rad).
        #[arg(long, default_value_t = 0.05)]
        tolerance: f32,
        /// Timeout (s).
        #[arg(long, default_value_t = 10.0)]
        timeout: f32,
    },
    /// Continuous velocity command (rad/s).
    Spin {
        /// Velocity in rad/s (negative reverses).
        #[arg(allow_hyphen_values = true)]
        velocity: f32,
        /// Stop after this many seconds (omit for Ctrl-C).
        #[arg(long)]
        duration: Option<f32>,
        /// Drive via MIT velocity-tracking (kd) instead of the velocity loop.
        /// MIT feedback reports torque (the velocity loop does not on some
        /// firmware), so use this to read the friction torque at speed.
        #[arg(long)]
        mit: bool,
        /// MIT velocity-tracking gain Kd (N·m·s/rad), used with --mit.
        #[arg(long, default_value_t = 1.0)]
        mit_kd: f32,
    },
    /// Continuous torque command (Nm).
    Torque {
        #[arg(allow_hyphen_values = true)]
        torque: f32,
        #[arg(long)]
        duration: Option<f32>,
    },
    /// One-shot MIT-mode command.
    Mit {
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        pos: f32,
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        vel: f32,
        #[arg(long, default_value_t = 0.0)]
        kp: f32,
        #[arg(long, default_value_t = 0.0)]
        kd: f32,
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        torque: f32,
    },
    /// Continuously command MIT-mode position hold (PD impedance) until Ctrl-C.
    MitHold {
        /// Hold position in radians.
        #[arg(long, allow_hyphen_values = true)]
        pos: f32,
        /// Kp gain (Nm/rad).
        #[arg(long, default_value_t = 30.0)]
        kp: f32,
        /// Kd gain (Nm·s/rad).
        #[arg(long, default_value_t = 1.0)]
        kd: f32,
        /// Command interval (ms).
        #[arg(long, default_value_t = 50)]
        interval: u64,
        /// Stop after this many seconds (omit for Ctrl-C).
        #[arg(long)]
        duration: Option<f32>,
    },
    /// Reassign this motor's CAN bus address (addressed at `-m/--motor-id`).
    ///
    /// Takes effect immediately and persists across power cycles — do this
    /// **one motor at a time** on the bus so the address change is
    /// unambiguous.
    SetId {
        /// New CAN id to assign (1..=127).
        new_id: u8,
    },
    /// Probe each motor id in the range and print responders.
    Scan {
        #[arg(long, default_value_t = 1)]
        from: u8,
        #[arg(long, default_value_t = 32)]
        to: u8,
        /// Per-id timeout (ms).
        #[arg(long, default_value_t = 50)]
        timeout: u64,
    },
    /// Read firmware version (AppCodeVersion) via RobStride's undocumented
    /// bulk parameter table (reverse-engineered from motorstudio — see
    /// robstride_protocol::param_table).
    Version {
        /// Overall read timeout (ms).
        #[arg(long, default_value_t = 500)]
        timeout_ms: u64,
    },
    /// Read one row's raw value by FunctionCode via the cheap single-row
    /// read (5 frames total — see robstride_protocol::param_table). Accepts
    /// hex (`0x1003`) or decimal. Use `param-table` first to look up a
    /// row's FunctionCode/name if you don't already know it.
    ///
    /// WARNING: unlike `param-table`, this path gets no name back from the
    /// motor, so its type is looked up by FunctionCode against the
    /// RS04/EL05 manual alone — a live capture found at least one firmware
    /// build repurposes specific FunctionCodes relative to the manual
    /// (verified names didn't match at 0x2006/0x2007). If the shown type
    /// looks wrong, cross-check the FunctionCode/name pairing with
    /// `param-table` first.
    ReadParam {
        /// FunctionCode to read, e.g. 0x1003 (AppCodeVersion) or 0x2005
        /// (MechOffset).
        #[arg(value_parser = parse_function_code)]
        function_code: u16,
        /// Overall read timeout (ms).
        #[arg(long, default_value_t = 500)]
        timeout_ms: u64,
    },
    /// Dump every row of the undocumented bulk parameter table (name +
    /// raw value bytes; see robstride_protocol::param_table).
    ParamTable {
        /// Overall read timeout (ms).
        #[arg(long, default_value_t = 500)]
        timeout_ms: u64,
    },
    /// Passively listen on the bus and print every frame.
    Dump {
        /// Listen duration (s).
        #[arg(long, default_value_t = 5.0)]
        duration: f32,
    },
    /// Periodically print position / velocity / torque.
    Monitor {
        /// Sampling interval (ms).
        #[arg(long, default_value_t = 100)]
        interval: u64,
    },
    /// Run a small end-to-end smoke test exercising position, velocity, torque
    /// and MIT control modes in sequence. Always returns the motor to disabled.
    SmokeTest {
        /// Position-control offset (rad). Test moves to start ± offset and back.
        #[arg(long, default_value_t = 0.3)]
        pos_offset: f32,
        /// Position-control speed limit (rad/s).
        #[arg(long, default_value_t = 2.0)]
        pos_speed: f32,
        /// Velocity-control magnitude (rad/s).
        #[arg(long, default_value_t = 0.5)]
        vel: f32,
        /// Torque-control magnitude (Nm). Must overcome gearbox stiction; on
        /// Edulite05 (1:10 gearbox) the bench default 0.2 Nm reliably starts
        /// motion. Below ~0.1 Nm the shaft typically does not break loose.
        #[arg(long, default_value_t = 0.2)]
        torque: f32,
        /// MIT-mode hold gain Kp (Nm/rad).
        #[arg(long, default_value_t = 20.0)]
        mit_kp: f32,
        /// MIT-mode hold gain Kd (Nm·s/rad).
        #[arg(long, default_value_t = 0.5)]
        mit_kd: f32,
        /// Per-leg duration (s) for each mode.
        #[arg(long, default_value_t = 0.6)]
        duration: f32,
        /// Abort the torque-control phase if |velocity| exceeds this (rad/s).
        #[arg(long, default_value_t = 5.0)]
        torque_vel_limit: f32,
    },
    /// Chirp-excitation system identification → CSV log + Bode (frequency
    /// response). Position channel is safest (bounded around current pos).
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

fn parse_model(s: &str) -> Result<MotorModel> {
    MotorModel::from_name(s).with_context(|| format!("unknown motor model: {s}"))
}

/// Parse a FunctionCode as hex (`0x1003`, `1003h`) or decimal (`4099`).
fn parse_function_code(s: &str) -> std::result::Result<u16, String> {
    let s = s.trim();
    let digits = s
        .strip_prefix("0x")
        .or_else(|| s.strip_prefix("0X"))
        .or_else(|| s.strip_suffix('h'))
        .or_else(|| s.strip_suffix('H'));
    match digits {
        Some(hex) => u16::from_str_radix(hex, 16).map_err(|e| e.to_string()),
        None => s.parse::<u16>().map_err(|e| e.to_string()),
    }
}

fn open_motor(cli: &Cli) -> Result<Motor> {
    let model = parse_model(&cli.model)?;
    Motor::open_with_host(&cli.interface, cli.motor_id, cli.host_id, model)
        .with_context(|| format!("failed to open {} for motor {}", cli.interface, cli.motor_id))
}

fn print_feedback(label: &str, fb: &robstride_driver::MotorFeedback) {
    println!(
        "{label}: motor={:>3} pos={:+.3} rad  vel={:+.3} rad/s  τ={:+.3} Nm  T={:.1}°C  mode={}",
        fb.motor_id, fb.position, fb.velocity, fb.torque, fb.temperature, fb.status.mode,
    );
    let s = &fb.status;
    if s.uncalibrated || s.stall || s.magnetic_encoder_fault || s.overtemperature || s.overcurrent || s.undervoltage {
        println!(
            "  flags:{}{}{}{}{}{}",
            if s.uncalibrated { " UNCAL" } else { "" },
            if s.stall { " STALL" } else { "" },
            if s.magnetic_encoder_fault { " MAG_FAULT" } else { "" },
            if s.overtemperature { " OVER_TEMP" } else { "" },
            if s.overcurrent { " OVER_CUR" } else { "" },
            if s.undervoltage { " UNDER_V" } else { "" },
        );
    }
}

fn install_ctrl_c(flag: Arc<AtomicBool>) {
    let _ = ctrlc::set_handler(move || flag.store(true, Ordering::SeqCst));
}

fn run(cli: Cli) -> Result<()> {
    match &cli.command {
        Command::Status => {
            let mut motor = open_motor(&cli)?;
            let fb = motor.read_status()?;
            print_feedback("status", &fb);
        }
        Command::Info { with_temp } => {
            let mut motor = open_motor(&cli)?;
            let pos = motor.read_position()?;
            let vel = motor.read_velocity()?;
            let iq = motor.read_current()?;
            let tau = motor.read_torque()?;
            let vbus = motor.read_vbus()?;
            if *with_temp {
                let temp = motor.read_temperature()?;
                println!(
                    "motor={} pos={:+.3} rad  vel={:+.3} rad/s  Iq={:+.3} A  τ={:+.3} Nm  Vbus={:.2} V  T={:.1}°C",
                    cli.motor_id, pos, vel, iq, tau, vbus, temp
                );
            } else {
                println!(
                    "motor={} pos={:+.3} rad  vel={:+.3} rad/s  Iq={:+.3} A  τ={:+.3} Nm  Vbus={:.2} V",
                    cli.motor_id, pos, vel, iq, tau, vbus
                );
            }
        }
        Command::Enable => {
            let mut motor = open_motor(&cli)?;
            let fb = motor.enable()?;
            print_feedback("enabled", &fb);
            // Caller likely wants the motor to stay enabled — leak the handle
            // so Drop doesn't auto-disable.
            std::mem::forget(motor);
        }
        Command::Disable => {
            let mut motor = open_motor(&cli)?;
            let fb = motor.disable()?;
            print_feedback("disabled", &fb);
        }
        Command::SetZero => {
            let mut motor = open_motor(&cli)?;
            motor.set_zero()?;
            println!("zero set on motor {}", cli.motor_id);
        }
        Command::MoveTo {
            position,
            speed,
            tolerance,
            timeout,
        } => {
            let mut motor = open_motor(&cli)?;
            // Run mode must be set while disabled (firmware ignores the write
            // otherwise), then enable, then issue the target. Setting LocRef
            // before enable does NOT take effect — the firmware only follows
            // LocRef writes that arrive while the motor is already enabled.
            let _ = motor.disable();
            motor.set_run_mode(RunMode::Position)?;
            motor.set_position_speed_limit(*speed)?;
            motor.enable()?;
            motor.set_position(*position)?;

            let stop = Arc::new(AtomicBool::new(false));
            install_ctrl_c(stop.clone());
            let start = Instant::now();
            loop {
                if stop.load(Ordering::SeqCst) {
                    println!("interrupted");
                    break;
                }
                if start.elapsed() > Duration::from_secs_f32(*timeout) {
                    println!("timed out before reaching target");
                    break;
                }
                // Use read_param instead of read_status: read_status sends a
                // zero MIT control frame which yanks Position mode back to MIT.
                let pos = motor.read_param(ParamIndex::MechPos)?;
                let vel = motor.read_param(ParamIndex::MechVel)?;
                println!("  pos={:+.3} rad  vel={:+.3} rad/s", pos, vel);
                if (pos - *position).abs() < *tolerance {
                    println!("arrived at {:.3} rad", pos);
                    break;
                }
                std::thread::sleep(Duration::from_millis(100));
            }
            motor.disable()?;
        }
        Command::Spin {
            velocity,
            duration,
            mit,
            mit_kd,
        } => {
            let mut motor = open_motor(&cli)?;
            let _ = motor.disable();
            let stop = Arc::new(AtomicBool::new(false));
            install_ctrl_c(stop.clone());
            let start = Instant::now();
            let max = duration.map(Duration::from_secs_f32);

            if *mit {
                // MIT velocity-tracking: τ = kd·(vel_des − vel). MIT feedback
                // carries torque, so this reads the friction torque at speed.
                motor.set_run_mode(RunMode::Mit)?;
                motor.enable()?;
                println!("spinning at {velocity} rad/s via MIT (kd={mit_kd}) — Ctrl-C to stop");
                while !stop.load(Ordering::SeqCst) {
                    if max.is_some_and(|d| start.elapsed() > d) {
                        break;
                    }
                    let fb = motor.mit_control(0.0, *velocity, 0.0, *mit_kd, 0.0)?;
                    println!(
                        "  pos={:+.3} rad  vel={:+.3} rad/s  τ={:+.3} Nm",
                        fb.position, fb.velocity, fb.torque
                    );
                    std::thread::sleep(Duration::from_millis(100));
                }
            } else {
                motor.set_run_mode(RunMode::Velocity)?;
                motor.set_velocity(0.0)?;
                motor.enable()?;
                motor.set_velocity(*velocity)?;
                println!("spinning at {velocity} rad/s — Ctrl-C to stop");
                while !stop.load(Ordering::SeqCst) {
                    if max.is_some_and(|d| start.elapsed() > d) {
                        break;
                    }
                    let pos = motor.read_param(ParamIndex::MechPos)?;
                    let vel = motor.read_param(ParamIndex::MechVel)?;
                    let tau = motor.read_torque().unwrap_or(f32::NAN);
                    println!("  pos={:+.3} rad  vel={:+.3} rad/s  τ={:+.3} Nm", pos, vel, tau);
                    std::thread::sleep(Duration::from_millis(100));
                }
                motor.set_velocity(0.0)?;
            }
            motor.disable()?;
        }
        Command::Torque { torque, duration } => {
            let mut motor = open_motor(&cli)?;
            let _ = motor.disable();
            motor.set_run_mode(RunMode::Torque)?;
            motor.set_torque(0.0)?;
            motor.enable()?;
            motor.set_torque(*torque)?;
            println!("applying {torque} Nm — Ctrl-C to stop");

            let stop = Arc::new(AtomicBool::new(false));
            install_ctrl_c(stop.clone());
            let start = Instant::now();
            let max = duration.map(Duration::from_secs_f32);
            while !stop.load(Ordering::SeqCst) {
                if max.is_some_and(|d| start.elapsed() > d) {
                    break;
                }
                let pos = motor.read_param(ParamIndex::MechPos)?;
                let vel = motor.read_param(ParamIndex::MechVel)?;
                println!("  pos={:+.3} rad  vel={:+.3} rad/s", pos, vel);
                std::thread::sleep(Duration::from_millis(100));
            }
            motor.set_torque(0.0)?;
            motor.disable()?;
        }
        Command::Mit {
            pos,
            vel,
            kp,
            kd,
            torque,
        } => {
            let mut motor = open_motor(&cli)?;
            let _ = motor.disable();
            motor.set_run_mode(RunMode::Mit)?;
            motor.enable()?;
            let fb = motor.mit_control(*pos, *vel, *kp, *kd, *torque)?;
            print_feedback("mit", &fb);
            motor.disable()?;
        }
        Command::MitHold {
            pos,
            kp,
            kd,
            interval,
            duration,
        } => {
            let mut motor = open_motor(&cli)?;
            let _ = motor.disable();
            motor.set_run_mode(RunMode::Mit)?;
            motor.enable()?;
            println!(
                "MIT hold at {:.3} rad, kp={:.1}, kd={:.2} — Ctrl-C to stop",
                pos, kp, kd
            );

            let stop = Arc::new(AtomicBool::new(false));
            install_ctrl_c(stop.clone());
            let dt = Duration::from_millis(*interval);
            let max = duration.map(Duration::from_secs_f32);
            let start = Instant::now();
            while !stop.load(Ordering::SeqCst) {
                if max.is_some_and(|d| start.elapsed() > d) {
                    break;
                }
                match motor.mit_control(*pos, 0.0, *kp, *kd, 0.0) {
                    Ok(fb) => print_feedback("  ", &fb),
                    Err(e) => eprintln!("mit error: {e}"),
                }
                std::thread::sleep(dt);
            }
            motor.disable()?;
        }
        Command::SetId { new_id } => {
            if *new_id == 0 {
                bail!("new id must be >= 1");
            }
            let mut motor = open_motor(&cli)?;
            println!(
                "reassigning motor {} -> {} on {} ...",
                cli.motor_id, new_id, cli.interface
            );
            motor.set_device_id(*new_id)?;
            println!("done. verifying at new id {new_id} ...");
            match motor.ping() {
                Ok(_) => println!("confirmed: motor now responds at id {new_id}."),
                Err(e) => println!(
                    "warning: no ping reply at id {new_id} ({e}) — rerun `scan` to confirm."
                ),
            }
            println!(
                "\nNEXT: use  -m {new_id}  for all further commands on this motor."
            );
        }
        Command::Scan { from, to, timeout } => {
            let timeout_per_id = Duration::from_millis(*timeout);
            let mut last_id: u8 = 0;
            let results = {
                let mut cb = |idx: usize, total: usize, motor_id: u8| {
                    if motor_id != 0 && motor_id != last_id {
                        last_id = motor_id;
                        eprint!("\rprobing {idx}/{total} (id={motor_id})   ");
                    }
                };
                scan_bus(
                    &cli.interface,
                    cli.host_id,
                    *from..=*to,
                    timeout_per_id,
                    Some(&mut cb),
                )?
            };
            eprintln!();
            if results.is_empty() {
                println!("no motors found");
            } else {
                println!("found {} motor(s):", results.len());
                for r in results {
                    print!("  id={:<3} payload=", r.motor_id);
                    for b in &r.payload {
                        print!("{:02X} ", b);
                    }
                    println!();
                }
            }
        }
        Command::Version { timeout_ms } => {
            let mut motor = open_motor(&cli)?;
            param_commands::run_version(&mut motor, cli.motor_id, *timeout_ms)?;
        }
        Command::ReadParam {
            function_code,
            timeout_ms,
        } => {
            let mut motor = open_motor(&cli)?;
            param_commands::run_read_param(&mut motor, *function_code, *timeout_ms)?;
        }
        Command::ParamTable { timeout_ms } => {
            let mut motor = open_motor(&cli)?;
            param_commands::run_param_table(&mut motor, *timeout_ms)?;
        }
        Command::Dump { duration } => {
            let frames = dump_bus(&cli.interface, Duration::from_secs_f32(*duration))?;
            println!("captured {} frame(s)", frames.len());
            for (id, data) in frames {
                print!("  0x{:08X} ", id);
                for b in &data {
                    print!("{:02X} ", b);
                }
                println!();
            }
        }
        Command::Monitor { interval } => {
            let mut motor = open_motor(&cli)?;
            let stop = Arc::new(AtomicBool::new(false));
            install_ctrl_c(stop.clone());
            let dt = Duration::from_millis(*interval);
            while !stop.load(Ordering::SeqCst) {
                match motor.read_status() {
                    Ok(fb) => print_feedback("  ", &fb),
                    Err(e) => eprintln!("read error: {e}"),
                }
                std::thread::sleep(dt);
            }
        }
        Command::SmokeTest {
            pos_offset,
            pos_speed,
            vel,
            torque,
            mit_kp,
            mit_kd,
            duration,
            torque_vel_limit,
        } => {
            run_smoke_test(
                &cli,
                SmokeParams {
                    pos_offset: *pos_offset,
                    pos_speed: *pos_speed,
                    vel: *vel,
                    torque: *torque,
                    mit_kp: *mit_kp,
                    mit_kd: *mit_kd,
                    duration: *duration,
                    torque_vel_limit: *torque_vel_limit,
                },
            )?;
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
            let mut motor = open_motor(&cli)?;
            chirp_cmd(
                &mut motor, channel, *f0, *f1, *duration, *amp, *log, *rate, *max_speed, *kp, *kd,
                out, bode,
            )?;
        }
    }
    Ok(())
}

/// Chirp-identification command: build the excitation, run it, write the raw
/// log + Bode CSVs. Takes the actuator as `dyn` so it's transport-agnostic.
#[allow(clippy::too_many_arguments)]
fn chirp_cmd(
    act: &mut dyn misa_actuator::Actuator,
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
            anyhow::bail!("unknown --channel '{other}' (position|velocity|torque|mit|mit-torque)")
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
    install_ctrl_c(abort.clone());
    println!("chirp {f0}->{f1} Hz over {duration}s, amp {amp}, channel={channel} (Ctrl-C aborts)...");

    let mut raw = std::io::BufWriter::new(
        std::fs::File::create(out).with_context(|| format!("create {out}"))?,
    );
    let mut bod = std::io::BufWriter::new(
        std::fs::File::create(bode).with_context(|| format!("create {bode}"))?,
    );
    let rep = run_chirp_to_csv(act, &chirp, exc, rate, &abort, &mut raw, &mut bod)?;

    println!(
        "collected {} samples @ {:.0} Hz (target {:.0}), median coherence {:.2}",
        rep.n_samples, rep.achieved_rate_hz, rate, rep.median_coherence
    );
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

struct SmokeParams {
    pos_offset: f32,
    pos_speed: f32,
    vel: f32,
    torque: f32,
    mit_kp: f32,
    mit_kd: f32,
    duration: f32,
    torque_vel_limit: f32,
}

/// Sample (pos, vel) at ~20 Hz for `duration` seconds via `read_param` so we
/// don't disturb the active control mode (a `read_status` send a zero MIT
/// command, which yanks Velocity/Torque mode back to MIT).
/// Returns the last (position, velocity) seen.
fn observe(motor: &mut Motor, label: &str, duration: f32, stop: &Arc<AtomicBool>) -> Option<(f32, f32)> {
    let start = Instant::now();
    let dt = Duration::from_millis(50);
    let mut last = None;
    while start.elapsed() < Duration::from_secs_f32(duration) {
        if stop.load(Ordering::SeqCst) {
            break;
        }
        let pos = match motor.read_param(ParamIndex::MechPos) {
            Ok(v) => v,
            Err(e) => {
                eprintln!("  {label} pos read error: {e}");
                std::thread::sleep(dt);
                continue;
            }
        };
        let vel = match motor.read_param(ParamIndex::MechVel) {
            Ok(v) => v,
            Err(e) => {
                eprintln!("  {label} vel read error: {e}");
                std::thread::sleep(dt);
                continue;
            }
        };
        println!("{label}: pos={:+.3} rad  vel={:+.3} rad/s", pos, vel);
        last = Some((pos, vel));
        std::thread::sleep(dt);
    }
    last
}

fn run_smoke_test(cli: &Cli, p: SmokeParams) -> Result<()> {
    let stop = Arc::new(AtomicBool::new(false));
    install_ctrl_c(stop.clone());

    let mut motor = open_motor(cli)?;
    let initial = motor.read_status().context("initial status read failed")?;
    println!(
        "=== Smoke test: motor id={} model={} ===",
        cli.motor_id, cli.model
    );
    print_feedback("initial", &initial);

    // ---- 1) Position control --------------------------------------------
    println!(
        "\n[1/4] Position control: target = initial ± {:.3} rad (speed limit {:.2} rad/s)",
        p.pos_offset, p.pos_speed
    );
    let _ = motor.disable();
    motor.set_run_mode(RunMode::Position)?;
    motor.set_position_speed_limit(p.pos_speed)?;
    motor.enable()?;

    let target_a = initial.position + p.pos_offset;
    println!("  -> {:.3} rad", target_a);
    motor.set_position(target_a)?;
    let after_a = observe(&mut motor, "  pos+", p.duration, &stop);

    let target_b = initial.position - p.pos_offset;
    println!("  -> {:.3} rad", target_b);
    motor.set_position(target_b)?;
    let after_b = observe(&mut motor, "  pos-", p.duration, &stop);

    println!("  -> {:.3} rad (return)", initial.position);
    motor.set_position(initial.position)?;
    let _ = observe(&mut motor, "  home", p.duration, &stop);
    motor.disable()?;

    let pos_tol = 0.1_f32;
    let pos_ok = after_a.is_some_and(|(pos, _)| (pos - target_a).abs() < pos_tol)
        && after_b.is_some_and(|(pos, _)| (pos - target_b).abs() < pos_tol);
    println!("  position phase: {}", if pos_ok { "OK" } else { "did not converge within tolerance" });

    if stop.load(Ordering::SeqCst) {
        println!("interrupted — skipping remaining phases");
        return Ok(());
    }

    // ---- 2) Velocity control --------------------------------------------
    println!("\n[2/4] Velocity control: ±{:.2} rad/s", p.vel);
    let _ = motor.disable();
    motor.set_run_mode(RunMode::Velocity)?;
    motor.set_velocity(0.0)?;
    motor.enable()?;
    motor.set_velocity(p.vel)?;
    let vel_pos = observe(&mut motor, "  vel+", p.duration, &stop);
    motor.set_velocity(-p.vel)?;
    let vel_neg = observe(&mut motor, "  vel-", p.duration, &stop);
    motor.set_velocity(0.0)?;
    let _ = observe(&mut motor, "  stop", 0.5, &stop);
    motor.disable()?;

    let vel_threshold = p.vel * 0.3;
    let vel_ok = vel_pos.is_some_and(|(_, v)| v > vel_threshold)
        && vel_neg.is_some_and(|(_, v)| v < -vel_threshold);
    println!("  velocity phase: {}", if vel_ok { "OK" } else { "did not reach target velocity" });

    if stop.load(Ordering::SeqCst) {
        println!("interrupted — skipping remaining phases");
        return Ok(());
    }

    // ---- 3) Torque control ----------------------------------------------
    println!(
        "\n[3/4] Torque control: ±{:.3} Nm (watchdog at |vel| > {:.1} rad/s)",
        p.torque, p.torque_vel_limit
    );
    let _ = motor.disable();
    motor.set_run_mode(RunMode::Torque)?;
    motor.set_torque(0.0)?;
    motor.enable()?;

    let mut torque_watchdog_ok = true;
    let mut torque_motion: f32 = 0.0;
    let pos_before_torque = motor.read_param(ParamIndex::MechPos).unwrap_or(0.0);
    for &tau in &[p.torque, -p.torque] {
        motor.set_torque(tau)?;
        let label = if tau > 0.0 { "  tau+" } else { "  tau-" };
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs_f32(p.duration) {
            if stop.load(Ordering::SeqCst) {
                break;
            }
            match (motor.read_param(ParamIndex::MechPos), motor.read_param(ParamIndex::MechVel)) {
                (Ok(pos), Ok(vel)) => {
                    println!("{label}: pos={:+.3} rad  vel={:+.3} rad/s", pos, vel);
                    torque_motion = (pos - pos_before_torque).abs().max(torque_motion);
                    if vel.abs() > p.torque_vel_limit {
                        eprintln!(
                            "  WATCHDOG: |vel| = {:.2} rad/s exceeded {:.1} — aborting torque leg",
                            vel, p.torque_vel_limit
                        );
                        torque_watchdog_ok = false;
                        break;
                    }
                }
                (Err(e), _) | (_, Err(e)) => eprintln!("  {label} read error: {e}"),
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        motor.set_torque(0.0)?;
        if stop.load(Ordering::SeqCst) {
            break;
        }
        // Let the shaft coast to a stop before the next leg so we don't
        // mistake residual motion for the next torque pulse.
        let _ = observe(&mut motor, "  coast", 0.3, &stop);
    }
    motor.disable()?;
    // Pass criterion: torque caused motion. Watchdog firing is informational —
    // it confirms torque was applied and is just the safety stop kicking in.
    let torque_ok = torque_motion > 0.05;
    let note = if torque_watchdog_ok { "" } else { " [watchdog stopped runaway]" };
    println!(
        "  torque phase: {} (max |Δpos|={:.3} rad{})",
        if torque_ok { "OK" } else { "no motion observed — try larger --torque" },
        torque_motion,
        note,
    );

    if stop.load(Ordering::SeqCst) {
        println!("interrupted — skipping remaining phases");
        return Ok(());
    }

    // ---- 4) MIT mode ----------------------------------------------------
    println!(
        "\n[4/4] MIT mode: hold at current with kp={:.1}, kd={:.2}",
        p.mit_kp, p.mit_kd
    );
    let _ = motor.disable();
    motor.set_run_mode(RunMode::Mit)?;
    motor.enable()?;
    let pre_mit = motor.read_status()?;
    let hold = pre_mit.position;
    println!("  hold target: {:.3} rad", hold);
    let start = Instant::now();
    let mut mit_last = None;
    while start.elapsed() < Duration::from_secs_f32(p.duration) {
        if stop.load(Ordering::SeqCst) {
            break;
        }
        match motor.mit_control(hold, 0.0, p.mit_kp, p.mit_kd, 0.0) {
            Ok(fb) => {
                print_feedback("  mit", &fb);
                mit_last = Some(fb);
            }
            Err(e) => eprintln!("  mit error: {e}"),
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    motor.disable()?;
    let mit_ok = mit_last.as_ref().is_some_and(|fb| (fb.position - hold).abs() < 0.2);
    println!("  MIT phase: {}", if mit_ok { "OK" } else { "did not hold target"});

    println!("\n=== Smoke test complete ===");
    println!(
        "results: position={} velocity={} torque={} mit={}",
        if pos_ok { "OK" } else { "FAIL" },
        if vel_ok { "OK" } else { "FAIL" },
        if torque_ok { "OK" } else { "FAIL" },
        if mit_ok { "OK" } else { "FAIL" },
    );
    Ok(())
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    run(Cli::parse())
}
