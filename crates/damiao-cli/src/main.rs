//! Test CLI for the `damiao-driver` crate (DM-J4310-2EC and friends).
//!
//! Classic CAN (1 Mbps):
//! ```text
//! sudo ip link set can0 type can bitrate 1000000 up
//! damiao-cli -i can0 scan
//! damiao-cli -i can0 -m 1 enable
//! damiao-cli -i can0 -m 1 move-to 1.57 --speed 5 --duration 3
//! damiao-cli -i can0 -m 1 mit --pos 0 --vel 0 --kp 30 --kd 1 --duration 5
//! ```
//!
//! CAN-FD (1 Mbps nominal / 5 Mbps data):
//! ```text
//! sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on up
//! damiao-cli -i can0 --fd -m 1 spin 2.0 --duration 3
//! ```
//!
//! Note: feedback returns on the motor's Master ID (`--master-id`, default 0).
//! On a multi-motor bus give each motor a unique Master ID first
//! (`reg-write 7 <id> --int`, then power-cycle).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};

use damiao_driver::{scan_bus_on, ControlMode, DamiaoBus, DamiaoMotor, Feedback, MotorModel, Rid};

#[derive(Parser, Debug)]
#[command(version, about = "Test CLI for DAMIAO CAN/CAN-FD servo motors")]
struct Cli {
    /// SocketCAN interface name (e.g. can0).
    #[arg(short, long, default_value = "can0")]
    interface: String,

    /// Motor CAN_ID (slave id).
    #[arg(short, long, default_value_t = 1)]
    motor_id: u8,

    /// Motor Master ID — the standard CAN id feedback is reported on (11-bit).
    /// Default 0, which accepts any responder (matched by the feedback id
    /// nibble). Set this on a multi-motor bus where CAN_IDs share a low nibble.
    #[arg(long, default_value_t = 0)]
    master_id: u16,

    /// Motor model (e.g. DM4310 / DM-J4310-2EC).
    #[arg(long, default_value = "DM4310")]
    model: String,

    /// Use a CAN-FD bus (interface must be `fd on`). Default is classic CAN.
    #[arg(long)]
    fd: bool,

    /// Per-request timeout, in ms.
    #[arg(long, default_value_t = 100)]
    timeout_ms: u64,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Probe a range of CAN_IDs for responding motors.
    Scan {
        /// First CAN_ID to probe.
        #[arg(long, default_value_t = 1)]
        from: u8,
        /// Last CAN_ID to probe.
        #[arg(long, default_value_t = 16)]
        to: u8,
    },
    /// Enable the motor (required before motion).
    Enable,
    /// Disable the motor (coast).
    Disable,
    /// Zero the current position. By default an in-memory *soft* zero (briefly
    /// enables the motor to read position); pass --nvm to persist to flash.
    Zero {
        /// Write the zero to motor NVM (FF..FE) instead of a soft zero.
        #[arg(long)]
        nvm: bool,
    },
    /// Clear a latched error (or just toggle disable→enable).
    ClearError,
    /// One status read (re-issues the last command / a zero MIT frame).
    Status,
    /// MIT impedance control. Holds for `--duration` seconds (Ctrl-C to stop).
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
    /// Position-Velocity move to an absolute angle (rad).
    MoveTo {
        #[arg(allow_hyphen_values = true)]
        position: f32,
        /// Max profile speed (rad/s).
        #[arg(long, default_value_t = 5.0)]
        speed: f32,
        /// Hold duration in seconds.
        #[arg(long, default_value_t = 3.0)]
        duration: f32,
    },
    /// Velocity command (rad/s).
    Spin {
        #[arg(allow_hyphen_values = true)]
        velocity: f32,
        /// Run duration in seconds.
        #[arg(long, default_value_t = 3.0)]
        duration: f32,
    },
    /// Show the motor's identity/config registers (CAN_ID, MST_ID, mode, ...).
    Info,
    /// Assign this motor's CAN_ID and/or MST_ID and persist to flash.
    ///
    /// Do this **one motor at a time** on the bus: connect a single motor,
    /// run set-id, power-cycle it, then connect the next. Addresses the motor
    /// at the current `-m`; by convention MST_ID defaults to `0x10 + CAN_ID`.
    SetId {
        /// New CAN_ID (listen/slave id) to assign. Omit to keep the current id.
        #[arg(long)]
        new_can_id: Option<u8>,
        /// New MST_ID (feedback id). Omit to use the `0x10 + CAN_ID` convention.
        #[arg(long)]
        new_master_id: Option<u16>,
        /// Don't persist to flash (volatile — lost on power cycle).
        #[arg(long)]
        no_save: bool,
    },
    /// Read a register (RID).
    RegRead {
        rid: u8,
    },
    /// Write a register (RID). Float by default; pass --int for an integer.
    RegWrite {
        rid: u8,
        #[arg(allow_hyphen_values = true)]
        value: f32,
        /// Interpret/write the value as an integer.
        #[arg(long)]
        int: bool,
        /// Also save to flash after writing.
        #[arg(long)]
        save: bool,
    },
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    let cli = Cli::parse();

    let model = MotorModel::from_name(&cli.model)
        .with_context(|| format!("unknown DAMIAO model: {}", cli.model))?;
    let timeout = Duration::from_millis(cli.timeout_ms);

    // The bus type is chosen at runtime; dispatch into a generic runner so the
    // command logic is written once for both transports.
    if cli.fd {
        let mut motor = DamiaoMotor::open_fd(&cli.interface, cli.motor_id, model)
            .with_context(|| format!("failed to open CAN-FD interface {}", cli.interface))?;
        configure(&mut motor, &cli, timeout)?;
        run(&mut motor, &cli)
    } else {
        let mut motor = DamiaoMotor::open(&cli.interface, cli.motor_id, model)
            .with_context(|| format!("failed to open CAN interface {}", cli.interface))?;
        configure(&mut motor, &cli, timeout)?;
        run(&mut motor, &cli)
    }
}

fn configure<B: DamiaoBus>(
    motor: &mut DamiaoMotor<B>,
    cli: &Cli,
    timeout: Duration,
) -> Result<()> {
    motor.set_timeout(timeout)?;
    // Match feedback on the requested Master ID (default 0).
    motor.set_master_id(cli.master_id);
    Ok(())
}

fn run<B: DamiaoBus>(motor: &mut DamiaoMotor<B>, cli: &Cli) -> Result<()> {
    match &cli.command {
        Command::Scan { from, to } => {
            if to < from {
                bail!("--to must be >= --from");
            }
            println!("scanning CAN_IDs {from}..={to} on {} ...", cli.interface);
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
                    println!("  motor CAN_ID {id} (0x{id:02X}) responded");
                }
            }
        }
        Command::Enable => {
            let fb = motor.enable()?;
            println!("enabled. {}", fb.map(fmt_fb).unwrap_or_else(|| "(no feedback)".into()));
        }
        Command::Disable => {
            motor.disable()?;
            println!("disabled.");
        }
        Command::Zero { nvm } => {
            if *nvm {
                motor.set_zero_nvm()?;
                println!("zero written to motor NVM (flash).");
            } else {
                // A soft zero needs a fresh position reading, which in MIT mode
                // requires the motor enabled. Energize with zero gains (no
                // holding torque), read, then coast.
                motor.switch_mode(ControlMode::Mit)?;
                motor.enable()?;
                let r = motor.set_zero();
                motor.disable()?;
                r?;
                println!(
                    "soft zero set at current position (in-memory, offset={:.3} rad).",
                    motor.soft_zero()
                );
            }
        }
        Command::ClearError => {
            motor.clear_error()?;
            println!("clear-error frame (FF..FB) sent.");
        }
        Command::Status => {
            let fb = motor.measure()?;
            println!("{}", fmt_fb(fb));
        }
        Command::Mit {
            pos,
            vel,
            kp,
            kd,
            tau,
            duration,
        } => {
            motor.switch_mode(ControlMode::Mit)?;
            motor.enable()?;
            control_loop(*duration, || {
                motor.mit_control(*pos, *vel, *kp, *kd, *tau)
            })?;
            motor.disable()?;
        }
        Command::MoveTo {
            position,
            speed,
            duration,
        } => {
            motor.switch_mode(ControlMode::PosVel)?;
            motor.enable()?;
            control_loop(Some(*duration), || motor.set_pos_vel(*position, *speed))?;
            motor.disable()?;
        }
        Command::Spin { velocity, duration } => {
            motor.switch_mode(ControlMode::Vel)?;
            motor.enable()?;
            control_loop(Some(*duration), || motor.set_vel(*velocity))?;
            motor.disable()?;
        }
        Command::Info => {
            // (label, RID, is_int)
            let regs: &[(&str, u8, bool)] = &[
                ("CAN_ID (ESC_ID)", Rid::ESC_ID, true),
                ("MST_ID", Rid::MST_ID, true),
                ("CTRL_MODE", Rid::CTRL_MODE, true),
                ("PMAX", Rid::PMAX, false),
                ("VMAX", Rid::VMAX, false),
                ("TMAX", Rid::TMAX, false),
            ];
            println!("motor config (addressed at CAN_ID {}):", cli.motor_id);
            for &(label, rid, is_int) in regs {
                match motor.read_register(rid) {
                    Ok(r) if is_int => println!("  {label:<16} (RID {rid:>2}) = {}", r.as_i32()),
                    Ok(r) => println!("  {label:<16} (RID {rid:>2}) = {}", r.as_f32()),
                    Err(e) => println!("  {label:<16} (RID {rid:>2}) = <no reply: {e}>"),
                }
            }
        }
        Command::SetId {
            new_can_id,
            new_master_id,
            no_save,
        } => {
            let target_can = new_can_id.unwrap_or(cli.motor_id);
            let master = new_master_id.unwrap_or(0x10 + target_can as u16);
            println!(
                "assigning (addressed at CAN_ID {}): MST_ID=0x{:X}{}",
                cli.motor_id,
                master,
                new_can_id
                    .map(|c| format!(", CAN_ID={c}"))
                    .unwrap_or_default()
            );
            // Write MST_ID first; write CAN_ID last (we keep addressing on the
            // old CAN_ID until the power cycle, so order is for clarity only).
            motor.write_register_int(Rid::MST_ID, master as i32)?;
            if let Some(nc) = new_can_id {
                motor.write_register_int(Rid::ESC_ID, *nc as i32)?;
            }
            if *no_save {
                eprintln!("note: --no-save — changes are in RAM only and will be lost on power-cycle");
            } else {
                motor.save_to_flash()?;
                println!("saved to flash.");
            }
            println!(
                "\nNEXT: power-cycle the motor, then address it with:\n  -m {} --master-id {}",
                target_can, master
            );
            println!("verify with:  damiao-cli -i {} -m {} info", cli.interface, target_can);
        }
        Command::RegRead { rid } => {
            let reply = motor.read_register(*rid)?;
            if Rid::is_int(*rid) {
                println!("RID {rid} = {} (int)", reply.as_i32());
            } else {
                println!("RID {rid} = {} (f32)", reply.as_f32());
            }
        }
        Command::RegWrite {
            rid,
            value,
            int,
            save,
        } => {
            if *int {
                motor.write_register_int(*rid, *value as i32)?;
                println!("wrote RID {rid} = {} (int)", *value as i32);
            } else {
                motor.write_register_f32(*rid, *value)?;
                println!("wrote RID {rid} = {value} (f32)");
            }
            if *save {
                // Re-read to confirm the RAM write landed, then commit to flash.
                if let Ok(reply) = motor.read_register(*rid) {
                    let v = if Rid::is_int(*rid) {
                        reply.as_i32() as f32
                    } else {
                        reply.as_f32()
                    };
                    println!("read-back RID {rid} = {v}");
                }
                motor.save_to_flash()?;
                println!("saved to flash.");
                eprintln!("note: CAN_ID/MST_ID changes take effect only after a power cycle");
            }
        }
    }
    Ok(())
}

/// Send a control command repeatedly at ~100 Hz (to satisfy the comm-loss
/// watchdog) for `duration` seconds, or until Ctrl-C if `duration` is `None`.
/// Prints the latest feedback periodically.
fn control_loop<F>(duration: Option<f32>, mut tick: F) -> Result<()>
where
    F: FnMut() -> damiao_driver::Result<Feedback>,
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

fn fmt_fb(fb: Feedback) -> String {
    format!(
        "id={} pos={:+.3} rad  vel={:+.3} rad/s  tau={:+.3} Nm  T_mos={:.0}°C  T_rotor={:.0}°C  err={:?}",
        fb.motor_id, fb.position, fb.velocity, fb.torque, fb.t_mos, fb.t_rotor, fb.err
    )
}
