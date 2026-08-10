//! Test CLI for the `lkmotor-driver` crate — LKMTech V3 motors (MG4005 etc.)
//! over an RS485 serial port.
//!
//! ```text
//! lkmotor-cli ports                          # list serial ports (Windows: COM*)
//! lkmotor-cli -i /dev/ttyUSB0 -m 1 scan       # -i COM5 on Windows
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

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};
use serde::Serialize;

use lkmotor_driver::{LkBus, LkCommands, LkMotor, MotorConfig, MotorId, Rs485Driver, SettingParamId};
use misa_actuator::Actuator;

/// clap-friendly mirror of [`SettingParamId`] (kept in `lkmotor-protocol`,
/// which is `no_std` and doesn't depend on `clap`).
#[derive(clap::ValueEnum, Clone, Copy, Debug)]
enum SettingParamArg {
    DriverId,
    BusType,
    Rs485Baudrate,
    CanBaudrate,
    MaxPower,
    MaxSpeed,
    MaxAngle,
    CurrentRamp,
    SpeedRamp,
}

impl SettingParamArg {
    fn to_id(self) -> SettingParamId {
        match self {
            SettingParamArg::DriverId => SettingParamId::DriverId,
            SettingParamArg::BusType => SettingParamId::BusType,
            SettingParamArg::Rs485Baudrate => SettingParamId::Rs485Baudrate,
            SettingParamArg::CanBaudrate => SettingParamId::CanBaudrate,
            SettingParamArg::MaxPower => SettingParamId::MaxPower,
            SettingParamArg::MaxSpeed => SettingParamId::MaxSpeed,
            SettingParamArg::MaxAngle => SettingParamId::MaxAngle,
            SettingParamArg::CurrentRamp => SettingParamId::CurrentRamp,
            SettingParamArg::SpeedRamp => SettingParamId::SpeedRamp,
        }
    }

    /// `true` for parameters that change the motor's bus address/reachability
    /// if set incorrectly (mirrors the RobStride `SET_CAN_ID` incident
    /// precedent — see lkmotor-protocol/doc/can-rs485-manual-analysis.md).
    fn is_risky(self) -> bool {
        matches!(
            self,
            SettingParamArg::DriverId
                | SettingParamArg::BusType
                | SettingParamArg::Rs485Baudrate
                | SettingParamArg::CanBaudrate
        )
    }
}

/// Rotation direction for single-turn/incremental position commands.
#[derive(clap::ValueEnum, Clone, Copy, Debug)]
enum Direction {
    Cw,
    Ccw,
}

/// Target brake state for the `brake` subcommand.
#[derive(clap::ValueEnum, Clone, Copy, Debug)]
enum BrakeStateArg {
    /// Engage the brake (holding, output locked).
    Engage,
    /// Release the brake (free, output unlocked).
    Release,
}

/// `motorCmd` byte options for a Hybrid broadcast slot.
#[derive(clap::ValueEnum, Clone, Copy, Debug)]
enum HybridCmdArg {
    /// Unused slot. The manual defines no explicit "no-op" `motorCmd` byte;
    /// `0x00` mirrors the implicit unused-slot convention the Torque/Speed/
    /// Position broadcast commands use.
    None,
    ReadState1,
    ClearError,
    ReadState2,
    MotorOff,
    MotorOn,
    MotorStop,
}

impl HybridCmdArg {
    fn code(self) -> u8 {
        match self {
            HybridCmdArg::None => 0x00,
            HybridCmdArg::ReadState1 => 0x9A,
            HybridCmdArg::ClearError => 0x9B,
            HybridCmdArg::ReadState2 => 0x9C,
            HybridCmdArg::MotorOff => 0x80,
            HybridCmdArg::MotorOn => 0x88,
            HybridCmdArg::MotorStop => 0x81,
        }
    }
}

#[derive(Parser, Debug)]
#[command(version, about = "Test CLI for LKMTech V3 servo motors (MG4005 / MG / MS / RMD-X)")]
struct Cli {
    /// RS485 serial device — `/dev/ttyUSB0` on Linux, `COM5` on Windows.
    /// Run `lkmotor-cli ports` to list what is attached.
    #[arg(short, long, default_value = lkmotor_driver::default_serial_port())]
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
    /// Single-turn absolute move (`0xA5`/`0xA6`) — direction + angle within
    /// one revolution, in the motor's raw wire units (no gear-ratio/anchor
    /// conversion, unlike `move-to`).
    MoveSingleTurn {
        /// Target angle within one turn, in degrees (0.0..360.0).
        angle_deg: f32,
        #[arg(long, value_enum, default_value_t = Direction::Cw)]
        direction: Direction,
        /// Optional max-speed cap (deg/s). Omit to use the uncapped variant.
        #[arg(long)]
        max_speed_dps: Option<u16>,
    },
    /// Incremental (relative) position move (`0xA7`/`0xA8`) — signed degrees,
    /// sign selects direction; raw wire units (no gear-ratio conversion).
    MoveIncremental {
        #[arg(allow_hyphen_values = true)]
        angle_deg: f32,
        /// Optional max-speed cap (deg/s). Omit to use the uncapped variant.
        #[arg(long)]
        max_speed_dps: Option<u16>,
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
    /// Read every known control parameter in one pass (PID triples + torque/
    /// speed/angle limits + current/speed ramp, all via `0xC0`). A parameter
    /// that fails to reply is reported and omitted rather than aborting the
    /// whole dump. Legacy `0x30`/`0x33` (ReadPid/ReadAccel) are not included —
    /// unimplemented, see lkmotor-protocol::command.
    Params {
        /// Also emit a TOML dump to stdout (or to --out if given).
        #[arg(long)]
        toml: bool,
        /// Write the TOML dump to this file (implies --toml).
        #[arg(long)]
        out: Option<PathBuf>,
    },
    /// Write one Setting Parameter Table value to RAM (`0x42`).
    ///
    /// **Caution**: driver-id/bus-type/rs485-baudrate/can-baudrate can make
    /// the motor unreachable at its current address/interface if set
    /// incorrectly — same risk class as RobStride's `SET_CAN_ID` incident.
    /// This writes to RAM only; pass `--save` to also persist to ROM
    /// (equivalent to running `save-settings` immediately after).
    SetSetting {
        #[arg(value_enum)]
        param: SettingParamArg,
        /// New value. Integer-valued for all current settings.
        #[arg(allow_hyphen_values = true)]
        value: i64,
        /// Persist to ROM immediately after writing (sends the Save
        /// Parameters command). Without this flag the change is RAM-only
        /// and lost on power-cycle.
        #[arg(long)]
        save: bool,
    },
    /// Commit any RAM-only Setting Parameter Table writes to ROM (`0x44`).
    /// Takes effect after `restart` or a power cycle.
    SaveSettings,
    /// Restart the motor (`0x07`), equivalent to a power cycle. The motor
    /// sends no reply to this command.
    Restart,
    /// Engage or release the holding brake (`0x8C`).
    Brake {
        #[arg(value_enum)]
        state: BrakeStateArg,
    },
    /// Read the current brake state (`0x8C` read-marker).
    BrakeStatus,
    /// Broadcast torque/open-loop control to up to 4 motors in one RS485
    /// frame (`0x80`). Addresses motors by slot position (1..=4), **not**
    /// `--motor-id` — motors must already be configured with distinct IDs
    /// 1..=4 (see `set-setting driver-id`) and broadcast mode enabled.
    BroadcastTorque {
        #[arg(long, allow_hyphen_values = true, default_value_t = 0)]
        m1: i16,
        #[arg(long, allow_hyphen_values = true, default_value_t = 0)]
        m2: i16,
        #[arg(long, allow_hyphen_values = true, default_value_t = 0)]
        m3: i16,
        #[arg(long, allow_hyphen_values = true, default_value_t = 0)]
        m4: i16,
        /// How many single-motor replies to wait for.
        #[arg(long, default_value_t = 4)]
        max_replies: usize,
    },
    /// Broadcast speed control to up to 4 motors in one RS485 frame (`0x81`).
    /// Slot-addressed, not `--motor-id` — see `broadcast-torque`.
    BroadcastSpeed {
        /// Speed in deg/s for motor #1.
        #[arg(long, allow_hyphen_values = true, default_value_t = 0)]
        m1: i16,
        #[arg(long, allow_hyphen_values = true, default_value_t = 0)]
        m2: i16,
        #[arg(long, allow_hyphen_values = true, default_value_t = 0)]
        m3: i16,
        #[arg(long, allow_hyphen_values = true, default_value_t = 0)]
        m4: i16,
        #[arg(long, default_value_t = 4)]
        max_replies: usize,
    },
    /// Broadcast absolute position control to up to 4 motors in one RS485
    /// frame (`0x82`). Slot-addressed, not `--motor-id` — see
    /// `broadcast-torque`. Angles are clamped to the wire field's
    /// `±327.67°` range.
    BroadcastPosition {
        /// Target angle in degrees for motor #1.
        #[arg(long, allow_hyphen_values = true, default_value_t = 0.0)]
        m1: f32,
        #[arg(long, allow_hyphen_values = true, default_value_t = 0.0)]
        m2: f32,
        #[arg(long, allow_hyphen_values = true, default_value_t = 0.0)]
        m3: f32,
        #[arg(long, allow_hyphen_values = true, default_value_t = 0.0)]
        m4: f32,
        #[arg(long, default_value_t = 4)]
        max_replies: usize,
    },
    /// Broadcast a distinct single-motor command per motor slot in one
    /// RS485 frame (`0x88`). Slot-addressed, not `--motor-id` — see
    /// `broadcast-torque`.
    BroadcastHybrid {
        #[arg(long, value_enum, default_value_t = HybridCmdArg::None)]
        m1: HybridCmdArg,
        #[arg(long, value_enum, default_value_t = HybridCmdArg::None)]
        m2: HybridCmdArg,
        #[arg(long, value_enum, default_value_t = HybridCmdArg::None)]
        m3: HybridCmdArg,
        #[arg(long, value_enum, default_value_t = HybridCmdArg::None)]
        m4: HybridCmdArg,
        #[arg(long, default_value_t = 4)]
        max_replies: usize,
    },
    /// Quasi-static characterization: load map, breakaway torque, thermal, Kt
    ///
    /// These deliberately hold torque against a loaded or stalled shaft, so
    /// every run is bounded by a safety envelope and aborts on the first breach,
    /// keeping whatever it collected. Ctrl-C also aborts.
    ///
    /// Run `load-map` first: its equilibrium position and peak holding torque
    /// tell you where to sit and what torque budget the other runs may use.
    Characterize {
        #[command(subcommand)]
        what: misa_sysid::CharacterizeCmd,

        /// Abort once |measured torque| exceeds this (N·m).
        #[arg(long, default_value_t = 1.0, global = true)]
        max_torque: f32,
        /// Abort once temperature exceeds this (°C).
        #[arg(long, default_value_t = 60.0, global = true)]
        max_temp: f32,
        /// Abort once temperature climbs faster than this (°C/s).
        #[arg(long, default_value_t = 2.0, global = true)]
        max_temp_rate: f32,
        /// Abort once position leaves start ± this (rad).
        #[arg(long, default_value_t = 0.5, global = true)]
        window: f32,
        /// Abort after this long (s), whatever else is happening.
        #[arg(long, default_value_t = 30.0, global = true)]
        max_duration: f32,
        /// Raw sample CSV path.
        #[arg(long, default_value = "characterize.csv", global = true)]
        out: PathBuf,
    },
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
    /// List the serial ports this machine reports, with USB ids.
    ///
    /// Mostly for Windows, where the RS485 adapter's COM number is handed out
    /// by the OS and differs from machine to machine.
    Ports,
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    // Windows sleeps round up to the ~15.6 ms scheduler tick by default,
    // which would throttle every timed loop below. No-op on Linux.
    let _timer = misa_actuator::realtime::TimerResolutionGuard::acquire();
    let cli = Cli::parse();

    // `ports` exists precisely because you don't know which port to open yet,
    // so it has to run before `open_motor`.
    if matches!(cli.command, Command::Ports) {
        return list_serial_ports();
    }

    let mut motor = open_motor(&cli)?;

    match &cli.command {
        Command::Ports => unreachable!("handled before the port is opened"),
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
        Command::MoveSingleTurn {
            angle_deg,
            direction,
            max_speed_dps,
        } => {
            let id = MotorId::new(cli.motor_id).context("invalid motor id")?;
            let ccw = matches!(direction, Direction::Ccw);
            let angle_centideg = (angle_deg.rem_euclid(360.0) * 100.0).round() as u32;
            let bus = motor.bus();
            let resp = match max_speed_dps {
                Some(speed) => {
                    bus.position_control_singleturn_with_speed(id, ccw, *speed, angle_centideg)?
                }
                None => bus.position_control_singleturn(id, ccw, angle_centideg)?,
            };
            let fb = lkmotor_driver::parse_state2_from_response(&resp)?;
            println!(
                "temp={:>3}°C  iq_raw={:+}  speed={:+} deg/s  encoder={}",
                fb.temperature_c, fb.iq_raw, fb.speed_deg_per_s, fb.encoder_raw
            );
        }
        Command::MoveIncremental {
            angle_deg,
            max_speed_dps,
        } => {
            let id = MotorId::new(cli.motor_id).context("invalid motor id")?;
            let angle_increment_centideg = (*angle_deg * 100.0).round() as i32;
            let bus = motor.bus();
            let resp = match max_speed_dps {
                Some(speed) => bus.position_control_incremental_with_speed(
                    id,
                    *speed,
                    angle_increment_centideg,
                )?,
                None => bus.position_control_incremental(id, angle_increment_centideg)?,
            };
            let fb = lkmotor_driver::parse_state2_from_response(&resp)?;
            println!(
                "temp={:>3}°C  iq_raw={:+}  speed={:+} deg/s  encoder={}",
                fb.temperature_c, fb.iq_raw, fb.speed_deg_per_s, fb.encoder_raw
            );
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
        Command::Params { toml, out } => {
            let id = MotorId::new(cli.motor_id).context("invalid motor id")?;
            let params = read_all_control_params(motor.bus(), id, cli.motor_id);
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
        Command::SetSetting { param, value, save } => {
            let id = MotorId::new(cli.motor_id).context("invalid motor id")?;
            if param.is_risky() {
                eprintln!(
                    "warning: {param:?} changes the motor's bus address/reachability — \
                     if set incorrectly the motor may stop responding on this interface. \
                     Change one motor at a time and keep a note of the previous value."
                );
            }
            let bus = motor.bus();
            let result = match param.to_id() {
                SettingParamId::DriverId
                | SettingParamId::BusType
                | SettingParamId::Rs485Baudrate
                | SettingParamId::CanBaudrate => {
                    bus.write_setting_param_u8(id, param.to_id(), *value as u8)
                }
                SettingParamId::MaxPower | SettingParamId::CurrentRamp => {
                    bus.write_setting_param_i16(id, param.to_id(), *value as i16)
                }
                SettingParamId::MaxSpeed | SettingParamId::MaxAngle | SettingParamId::SpeedRamp => {
                    bus.write_setting_param_i32(id, param.to_id(), *value as i32)
                }
            }?;
            println!("wrote {param:?} = {value} (RAM only) — read back: {result:?}");
            if *save {
                bus.save_setting_params(id)?;
                println!(
                    "saved to ROM. Run `restart` (or power-cycle) for the change to take effect."
                );
            } else {
                println!("note: RAM only — run `save-settings` to persist, then `restart`.");
            }
        }
        Command::SaveSettings => {
            let id = MotorId::new(cli.motor_id).context("invalid motor id")?;
            motor.bus().save_setting_params(id)?;
            println!("saved Setting Parameter Table to ROM. Run `restart` (or power-cycle) for changes to take effect.");
        }
        Command::Restart => {
            let id = MotorId::new(cli.motor_id).context("invalid motor id")?;
            motor.bus().motor_restart(id)?;
            println!("restart command sent (no reply expected).");
        }
        Command::Brake { state } => {
            let id = MotorId::new(cli.motor_id).context("invalid motor id")?;
            let released = matches!(state, BrakeStateArg::Release);
            let now_released = motor.bus().set_brake(id, released)?;
            println!(
                "brake {}",
                if now_released { "released (free)" } else { "engaged (holding)" }
            );
        }
        Command::BrakeStatus => {
            let id = MotorId::new(cli.motor_id).context("invalid motor id")?;
            let released = motor.bus().read_brake_state(id)?;
            println!(
                "brake is {}",
                if released { "released (free)" } else { "engaged (holding)" }
            );
        }
        Command::BroadcastTorque {
            m1,
            m2,
            m3,
            m4,
            max_replies,
        } => {
            let replies = motor.bus().broadcast_torque([*m1, *m2, *m3, *m4], *max_replies)?;
            print_broadcast_replies(&replies);
        }
        Command::BroadcastSpeed {
            m1,
            m2,
            m3,
            m4,
            max_replies,
        } => {
            let replies = motor.bus().broadcast_speed([*m1, *m2, *m3, *m4], *max_replies)?;
            print_broadcast_replies(&replies);
        }
        Command::BroadcastPosition {
            m1,
            m2,
            m3,
            m4,
            max_replies,
        } => {
            let to_centideg =
                |deg: f32| (deg * 100.0).round().clamp(i16::MIN as f32, i16::MAX as f32) as i16;
            let values = [
                to_centideg(*m1),
                to_centideg(*m2),
                to_centideg(*m3),
                to_centideg(*m4),
            ];
            let replies = motor.bus().broadcast_position(values, *max_replies)?;
            print_broadcast_replies(&replies);
        }
        Command::BroadcastHybrid {
            m1,
            m2,
            m3,
            m4,
            max_replies,
        } => {
            let cmds = [m1.code(), m2.code(), m3.code(), m4.code()];
            let replies = motor.bus().broadcast_hybrid(cmds, *max_replies)?;
            print_broadcast_replies(&replies);
        }
        Command::Characterize {
            what,
            max_torque,
            max_temp,
            max_temp_rate,
            window,
            max_duration,
            out,
        } => {
            let limits = misa_sysid::SafetyLimits {
                max_torque_nm: *max_torque,
                max_temperature_c: *max_temp,
                max_temperature_rise_c_per_s: *max_temp_rate,
                position_window_rad: *window,
                max_duration_s: *max_duration,
            };
            misa_sysid::run_characterize(&mut motor, what, limits, out)?;
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

fn print_broadcast_replies(replies: &[lkmotor_driver::Response]) {
    if replies.is_empty() {
        println!("  no replies received");
        return;
    }
    for r in replies {
        println!(
            "  motor {} replied: cmd=0x{:02X} data={:02X?}",
            r.motor_id, r.command, r.data
        );
    }
}

/// Print every serial port the OS knows about, so the right `--interface`
/// can be picked without guessing.
fn list_serial_ports() -> Result<()> {
    let ports = lkmotor_driver::list_ports();
    if ports.is_empty() {
        println!("no serial ports found");
        if cfg!(windows) {
            println!("check Device Manager → Ports (COM & LPT); a USB-RS485 adapter");
            println!("usually needs its CH340 / FTDI / CP210x driver installed first");
        }
        return Ok(());
    }
    println!("{:<12}  {}", "PORT", "DETAIL");
    for (name, detail) in ports {
        println!("{name:<12}  {detail}");
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

/// All control parameters (`0xC0`) worth including in a bulk settings dump.
/// Legacy `ReadPid` (`0x30`) / `ReadAccel` (`0x33`) are deliberately excluded:
/// neither has an encode/decode implementation in `lkmotor-protocol` yet, and
/// wiring up unverified wire format for a bulk dump isn't worth the risk —
/// see lkmotor-driver's bus.rs for the fully-implemented `0xC0` wrappers this
/// reuses.
#[derive(Debug, Serialize)]
struct AllControlParams {
    motor_id: u8,
    #[serde(skip_serializing_if = "Option::is_none")]
    position_kp: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    position_ki: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    position_kd: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_kp: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_ki: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_kd: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    current_kp: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    current_ki: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    current_kd: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    torque_limit_raw: Option<i16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_limit_centideg_per_s: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    angle_limit_centideg: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    current_ramp: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_ramp_dps_per_s: Option<i32>,

    // -- Setting Parameter Table (0x40) — a *different* space from the
    // 0xC0-based fields above, see lkmotor-protocol::command::SettingParamId.
    #[serde(skip_serializing_if = "Option::is_none")]
    driver_id: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    bus_type: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    rs485_baudrate_code: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    can_baudrate_code: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    max_power: Option<i16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    max_speed_setting_centideg_per_s: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    max_angle_setting_centideg: Option<i32>,
    /// **Note**: `int16` per the Setting Parameter Table, unlike
    /// `current_ramp` (`int32`) above for the same name/range — a
    /// discrepancy in the source manual itself, see
    /// lkmotor-protocol/doc/can-rs485-manual-analysis.md.
    #[serde(skip_serializing_if = "Option::is_none")]
    setting_current_ramp: Option<i16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    setting_speed_ramp_dps_per_s: Option<i32>,
}

impl AllControlParams {
    fn print(&self) {
        println!("== LKMotor control parameters (motor_id={}) ==", self.motor_id);
        fn fmt<T: std::fmt::Display>(label: &str, v: Option<T>) {
            match v {
                Some(v) => println!("  {label:<28} = {v}"),
                None => println!("  {label:<28} = <no reply>"),
            }
        }
        fmt("position_kp", self.position_kp);
        fmt("position_ki", self.position_ki);
        fmt("position_kd", self.position_kd);
        fmt("speed_kp", self.speed_kp);
        fmt("speed_ki", self.speed_ki);
        fmt("speed_kd", self.speed_kd);
        fmt("current_kp", self.current_kp);
        fmt("current_ki", self.current_ki);
        fmt("current_kd", self.current_kd);
        fmt("torque_limit_raw", self.torque_limit_raw);
        fmt("speed_limit_centideg_per_s", self.speed_limit_centideg_per_s);
        fmt("angle_limit_centideg", self.angle_limit_centideg);
        fmt("current_ramp", self.current_ramp);
        fmt("speed_ramp_dps_per_s", self.speed_ramp_dps_per_s);
        fmt("driver_id", self.driver_id);
        fmt("bus_type", self.bus_type);
        fmt("rs485_baudrate_code", self.rs485_baudrate_code);
        fmt("can_baudrate_code", self.can_baudrate_code);
        fmt("max_power", self.max_power);
        fmt("max_speed_setting_centideg_per_s", self.max_speed_setting_centideg_per_s);
        fmt("max_angle_setting_centideg", self.max_angle_setting_centideg);
        fmt("setting_current_ramp", self.setting_current_ramp);
        fmt("setting_speed_ramp_dps_per_s", self.setting_speed_ramp_dps_per_s);
    }
}

fn read_all_control_params<B: LkBus>(
    bus: &mut B,
    id: MotorId,
    motor_id: u8,
) -> AllControlParams {
    macro_rules! log_err {
        ($label:expr, $e:expr) => {
            eprintln!("  {} : <no reply: {}>", $label, $e)
        };
    }
    let (position_kp, position_ki, position_kd) = match bus.read_position_pid(id) {
        Ok(p) => (Some(p.kp), Some(p.ki), Some(p.kd)),
        Err(e) => {
            log_err!("position_pid", e);
            (None, None, None)
        }
    };
    let (speed_kp, speed_ki, speed_kd) = match bus.read_speed_pid(id) {
        Ok(p) => (Some(p.kp), Some(p.ki), Some(p.kd)),
        Err(e) => {
            log_err!("speed_pid", e);
            (None, None, None)
        }
    };
    let (current_kp, current_ki, current_kd) = match bus.read_current_pid(id) {
        Ok(p) => (Some(p.kp), Some(p.ki), Some(p.kd)),
        Err(e) => {
            log_err!("current_pid", e);
            (None, None, None)
        }
    };
    let torque_limit_raw = bus
        .read_torque_limit(id)
        .inspect_err(|e| log_err!("torque_limit", e))
        .ok();
    let speed_limit_centideg_per_s = bus
        .read_speed_limit(id)
        .inspect_err(|e| log_err!("speed_limit", e))
        .ok();
    let angle_limit_centideg = bus
        .read_angle_limit(id)
        .inspect_err(|e| log_err!("angle_limit", e))
        .ok();
    let current_ramp = bus
        .read_current_ramp(id)
        .inspect_err(|e| log_err!("current_ramp", e))
        .ok();
    let speed_ramp_dps_per_s = bus
        .read_speed_ramp(id)
        .inspect_err(|e| log_err!("speed_ramp", e))
        .ok();

    let driver_id = bus.read_driver_id(id).inspect_err(|e| log_err!("driver_id", e)).ok();
    let bus_type = bus.read_bus_type(id).inspect_err(|e| log_err!("bus_type", e)).ok();
    let rs485_baudrate_code = bus
        .read_rs485_baudrate(id)
        .inspect_err(|e| log_err!("rs485_baudrate", e))
        .ok();
    let can_baudrate_code = bus
        .read_can_baudrate(id)
        .inspect_err(|e| log_err!("can_baudrate", e))
        .ok();
    let max_power = bus.read_max_power(id).inspect_err(|e| log_err!("max_power", e)).ok();
    let max_speed_setting_centideg_per_s = bus
        .read_max_speed_setting(id)
        .inspect_err(|e| log_err!("max_speed_setting", e))
        .ok();
    let max_angle_setting_centideg = bus
        .read_max_angle_setting(id)
        .inspect_err(|e| log_err!("max_angle_setting", e))
        .ok();
    let setting_current_ramp = bus
        .read_setting_current_ramp(id)
        .inspect_err(|e| log_err!("setting_current_ramp", e))
        .ok();
    let setting_speed_ramp_dps_per_s = bus
        .read_setting_speed_ramp(id)
        .inspect_err(|e| log_err!("setting_speed_ramp", e))
        .ok();

    AllControlParams {
        motor_id,
        position_kp,
        position_ki,
        position_kd,
        speed_kp,
        speed_ki,
        speed_kd,
        current_kp,
        current_ki,
        current_kd,
        torque_limit_raw,
        speed_limit_centideg_per_s,
        angle_limit_centideg,
        current_ramp,
        speed_ramp_dps_per_s,
        driver_id,
        bus_type,
        rs485_baudrate_code,
        can_baudrate_code,
        max_power,
        max_speed_setting_centideg_per_s,
        max_angle_setting_centideg,
        setting_current_ramp,
        setting_speed_ramp_dps_per_s,
    }
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
            misa_actuator::realtime::sleep_precise(rem);
        }
    }
    Ok(())
}
