//! Test CLI for the `myactuator-driver` crate (MyActuator RMD, CAN V3).
//!
//! `-i` names the CAN interface: `can0` on Linux, `pcan:usb1` for a PEAK
//! adapter on Windows, `slcan:COM5` for a USB-CAN dongle on either.
//!
//! ```text
//! sudo ip link set can0 type can bitrate 1000000 up      # Linux only
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

use myactuator_driver::{dump_bus_on, scan_bus_on, MotorConfig, MotorFeedback, MyActuatorMotor};
use myactuator_protocol::{AccelIndex, ParamIndex};

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
    /// CAN interface: `can0` (Linux SocketCAN), `pcan:usb1` (PEAK adapter on
    /// Windows) or `slcan:COM5` (an adapter running **slcan** firmware — most
    /// dongles sold as "USB-CAN" speak something else and will not work).
    /// Append `@500K` to override the 1 Mbit/s default bitrate.
    #[arg(short, long, default_value = misa_can::default_interface())]
    interface: String,

    /// Motor id on the bus (1..=32).
    #[arg(short, long, default_value_t = 1)]
    motor_id: u8,

    /// Output-frame torque constant Kt (N·m/A). 0 = current-units mode: the
    /// torque API carries amps instead of N·m.
    ///
    /// Prefer `--refresh-kt`: the motor knows this value and you do not have
    /// to be right about it.
    #[arg(long, default_value_t = 0.0, conflicts_with = "refresh_kt")]
    kt: f32,

    /// Read the torque constant from the motor (`KT_OUT`, 0xC0) instead of
    /// taking it from `--kt`.
    ///
    /// Kt scales torque in both directions, so a wrong one is silently wrong
    /// rather than an error — and there is no model name to look a datasheet
    /// value up by, because the documented `0xB5` "motor model" command comes
    /// back empty on this firmware. A real RMD reports 1.1 N·m/A.
    #[arg(long)]
    refresh_kt: bool,

    /// Per-request timeout, in ms.
    #[arg(long, default_value_t = 100)]
    timeout_ms: u64,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// List the CAN interfaces attached right now.
    ///
    /// Opens nothing and puts no frame on any wire. A typed `-i` still works for
    /// anything this misses.
    Interfaces {
        /// Test one adapter instead of listing: put it in loopback, send a frame,
        /// and check the same frame comes back. See the usbcan backend docs.
        #[arg(long, value_name = "INTERFACE")]
        selftest: Option<String>,
    },
    /// Probe a range of motor ids for responding motors (0x9A read — no motion).
    Scan {
        /// First id to probe.
        #[arg(long, default_value_t = 1)]
        from: u8,
        /// Last id to probe.
        #[arg(long, default_value_t = 32)]
        to: u8,
    },
    /// Passively listen on the bus and print every frame as it arrives,
    /// decoded into MyActuator ids and command codes.
    ///
    /// Sends nothing. Use it to watch what another tool (MYACTUATOR Setup
    /// Software, a vendor SDK) puts on the wire while you drive its UI — put
    /// this adapter and the other one on the same CANH/CANL/GND at the same
    /// bitrate, start the capture, then press the button you want to
    /// understand.
    ///
    /// Ctrl-C ends the capture early and keeps what was printed.
    Dump {
        /// Listen duration (s).
        #[arg(long, default_value_t = 30.0)]
        duration: f32,
        /// Print raw ids only, without the command decode.
        #[arg(long)]
        raw: bool,
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
    /// Read the multi-turn encoder position (0x60, zero offset applied), pulses.
    Encoder,
    /// Read the current run mode (0x70).
    RunMode,
    /// Read the motor's instantaneous output power (0x71), watts.
    Power,
    /// Read the single-turn encoder position (0x90, direct-drive models).
    SingleTurnEncoder,
    /// Read the single-turn angle (0x94), 0-359.99°.
    SingleTurnAngle,
    /// Read Status3 (0x9D): temperature + per-phase current.
    Status3,
    /// Read system uptime since last reset/reboot (0xB1).
    Uptime,
    /// Read every known readable parameter in one pass (0x60/0x61/0x62/0x70/
    /// 0x71/0x90/0x92/0x94/0x9A/0x9C/0x9D/0x30×7/0x42×4/0xB1/0xB2/0xB5, plus
    /// the undocumented 0xC0 indexed space — Protect/Plan/Motor Parameters
    /// and a second PID gain set, see
    /// myactuator-protocol/doc/setup-software-c0-param-protocol.md) and
    /// print it. A read that times out or fails is reported and omitted
    /// rather than aborting the whole dump.
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
    /// Move to an absolute output-shaft position, in the motor's own frame
    /// (the angle `angle` prints).
    ///
    /// Absolute by default so that repeating the command holds the shaft
    /// instead of walking it. `--relative` restores the older behaviour of
    /// measuring from wherever the shaft happens to be.
    ///
    /// **"Absolute" is unique only modulo one output turn.** The encoder ROM
    /// offset survives a power cycle but the turn count does not (measured
    /// 2026-08-08: a stationary shaft came back 359.85° away). Within one power
    /// session the frame is exact.
    MoveTo {
        #[arg(allow_hyphen_values = true)]
        position: f32,
        /// Treat `position` as an offset from the current position instead of
        /// an absolute angle.
        #[arg(long)]
        relative: bool,
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
    /// Quasi-static characterization: load map, breakaway torque, thermal, Kt
    ///
    /// These deliberately hold torque against a loaded or stalled shaft, so every
    /// run is bounded by a safety envelope and aborts on the first breach,
    /// keeping whatever it collected. Ctrl-C also aborts.
    ///
    /// Release the holding brake first — measuring against an engaged brake
    /// reports the brake, not the mechanism.
    ///
    /// Pass `--kt` so torque is in N·m; without it the figures are amps.
    Characterize {
        #[command(subcommand)]
        what: misa_sysid::CharacterizeCmd,

        /// Abort once |measured torque| exceeds this (N·m, or A without --kt).
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

/// One captured frame, decoded into the MyActuator id layout and command byte.
///
/// The ASCII column earns its place here: the model name comes back as
/// five-character chunks of a `0xB5` reply, and spotting `RMD-X` in a hex dump
/// by eye is needless work.
fn print_dump_frame(t: Duration, id: u16, data: &[u8], raw: bool) {
    use myactuator_protocol::can_id::{
        COMMAND_BASE, MOTION_BASE, MOTION_REPLY_BASE, MULTI_MOTOR_ID, REPLY_BASE,
    };

    let hex = data
        .iter()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(" ");
    if raw {
        println!("{:9.3}  0x{id:03X}  {hex}", t.as_secs_f64() * 1000.0);
        return;
    }

    let channel = match id {
        MULTI_MOTOR_ID => "broadcast".to_string(),
        i if (COMMAND_BASE + 1..=COMMAND_BASE + 32).contains(&i) => {
            format!("cmd -> motor {}", i - COMMAND_BASE)
        }
        i if (REPLY_BASE + 1..=REPLY_BASE + 32).contains(&i) => {
            format!("reply <- motor {}", i - REPLY_BASE)
        }
        i if (MOTION_BASE + 1..=MOTION_BASE + 32).contains(&i) => {
            format!("motion -> motor {}", i - MOTION_BASE)
        }
        i if (MOTION_REPLY_BASE + 1..=MOTION_REPLY_BASE + 32).contains(&i) => {
            format!("motion <- motor {}", i - MOTION_REPLY_BASE)
        }
        _ => "?".to_string(),
    };
    // Byte 0 is the command code on every single-motor frame, in both
    // directions — the motor echoes it back.
    let cmd = data.first().map_or_else(String::new, |c| format!("0x{c:02X}"));
    let ascii: String = data
        .iter()
        .map(|&b| if b.is_ascii_graphic() { b as char } else { '.' })
        .collect();
    println!(
        "{:9.3}  0x{id:03X}  {channel:<22}  {cmd:<9}  {hex:<23}  |{ascii}|",
        t.as_secs_f64() * 1000.0
    );
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    // Windows sleeps round up to the ~15.6 ms scheduler tick by default,
    // which would throttle every timed loop below. No-op on Linux.
    let _timer = misa_actuator::realtime::TimerResolutionGuard::acquire();
    let cli = Cli::parse();

    let config = if cli.kt > 0.0 {
        MotorConfig::new(cli.kt)
    } else {
        MotorConfig::current_units()
    };
    let mut motor = MyActuatorMotor::open(&cli.interface, cli.motor_id, config)
        .with_context(|| format!("failed to open CAN interface {}", cli.interface))?;
    motor.set_timeout(Duration::from_millis(cli.timeout_ms))?;

    if cli.refresh_kt {
        let kt = motor
            .refresh_torque_constant()
            .context("failed to read KT_OUT from the motor")?;
        eprintln!("Kt from the motor: {kt} N·m/A");
    }

    match &cli.command {
        Command::Dump { duration, raw } => {
            let stop = Arc::new(AtomicBool::new(false));
            {
                let s = stop.clone();
                let _ = ctrlc::set_handler(move || s.store(true, Ordering::SeqCst));
            }
            eprintln!(
                "listening on {} for {duration:.0}s (Ctrl-C to stop early) — \
                 nothing is transmitted",
                cli.interface
            );
            if !raw {
                println!(
                    "{:>9}  {:>5}  {:<22}  {:<9}  {:<23}  ascii",
                    "t [ms]", "id", "channel", "cmd", "data"
                );
            }
            let n = dump_bus_on(
                motor.bus(),
                Duration::from_secs_f32(*duration),
                &|| stop.load(Ordering::SeqCst),
                &mut |t, id, data| print_dump_frame(t, id, data, *raw),
            )?;
            eprintln!("captured {n} frame(s)");
        }
        Command::Interfaces { selftest } => match selftest {
            Some(spec) => println!("{}", misa_can::loopback_selftest(spec)?),
            None => print!("{}", misa_can::format_list(&misa_can::list_interfaces())),
        },
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
            // Only caveat an answer that is actually missing. Tagging a real
            // model name with "empty if unsupported" reads as doubt about a
            // value the motor stated plainly.
            if model.is_empty() {
                print!("model=<none> (neither 0xB5 form answered)  ");
            } else {
                print!("model={model}  ");
            }
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
                "current: kp={:.4} ki={:.4}  speed: kp={:.4} ki={:.4}  position: kp={:.4} ki={:.4} kd={:.4}",
                g.current_kp, g.current_ki, g.speed_kp, g.speed_ki, g.position_kp, g.position_ki,
                g.position_kd
            );
        }
        Command::Accel { index } => {
            let dps_s = motor.read_acceleration((*index).into())?;
            println!("{index:?} = {dps_s} dps/s");
        }
        Command::Encoder => {
            let encoder = motor.read_multi_turn_encoder()?;
            let raw = motor.read_multi_turn_encoder_raw()?;
            let offset = motor.read_multi_turn_zero_offset()?;
            println!("encoder={encoder} pulses  raw={raw} pulses  zero_offset={offset} pulses");
        }
        Command::RunMode => {
            println!("run mode = {:?}", motor.read_run_mode()?);
        }
        Command::Power => {
            println!("motor power = {:.1} W", motor.read_motor_power()?);
        }
        Command::SingleTurnEncoder => {
            let e = motor.read_single_turn_encoder()?;
            println!(
                "encoder={} pulses  raw={} pulses  zero_offset={} pulses",
                e.encoder, e.encoder_raw, e.encoder_offset
            );
        }
        Command::SingleTurnAngle => {
            let centideg = motor.read_single_turn_angle()?;
            println!("single-turn angle = {:.2}°", centideg as f32 / 100.0);
        }
        Command::Status3 => {
            let s = motor.read_status3()?;
            println!(
                "temp={} °C  iA={:.2}A  iB={:.2}A  iC={:.2}A",
                s.temperature_c,
                s.phase_a_centi_amps as f32 / 100.0,
                s.phase_b_centi_amps as f32 / 100.0,
                s.phase_c_centi_amps as f32 / 100.0
            );
        }
        Command::Uptime => {
            let ms = motor.read_uptime_ms()?;
            println!("uptime = {ms} ms ({:.2} h)", ms as f64 / 3_600_000.0);
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
            print_settled(&mut motor);
        }
        Command::MoveTo {
            position,
            relative,
            speed,
            duration,
        } => {
            // Read where the shaft is *before* choosing the frame, so the move
            // can be announced in the same units it is commanded in. `0x92` is
            // 0.01°/LSB, ~7x finer than the control reply's 1°/LSB angle.
            let here = motor.read_multi_turn_centideg()? as f32 / 100.0;
            let here_rad = here.to_radians();
            if *relative {
                motor.rezero()?;
            } else {
                motor.anchor_at_motor_zero();
            }
            let target_rad = if *relative { here_rad + *position } else { *position };
            let delta = target_rad - here_rad;

            // Announce before moving, never after. The hazard of an absolute
            // default is a command that reads small and moves far — `move-to
            // 0.5` is a nudge when the shaft is near zero and most of a turn
            // when it is not. Printing the delta is what makes that visible;
            // the CLI warns and continues rather than prompting, as everywhere
            // else here.
            println!(
                "move: {here_rad:+.4} -> {target_rad:+.4} rad  (delta {delta:+.4} rad, {:+.2}°)",
                delta.to_degrees()
            );
            if delta.abs() > std::f32::consts::PI {
                eprintln!(
                    "warning: this is more than half an output turn ({:+.2}°). \
                     Check the shaft is free and nothing is tethered to it.",
                    delta.to_degrees()
                );
            }

            let r = control_loop(Some(*duration), || motor.set_position(target_rad, *speed));
            motor.stop()?;
            r?;
            print_settled(&mut motor);
        }
        Command::Torque { value, duration } => {
            let r = control_loop(Some(*duration), || motor.set_torque(*value));
            motor.stop()?;
            r?;
            print_settled(&mut motor);
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
/// (e.g. `0x90` on non-direct-drive models); a failed read is reported to
/// stderr and just omitted rather than aborting the whole dump.
#[derive(Serialize)]
struct AllParams {
    motor_id: u8,
    // Status1 (0x9A)
    #[serde(skip_serializing_if = "Option::is_none")]
    temperature_c: Option<i8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    mos_temperature_c: Option<i8>,
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
    // Status3 (0x9D)
    #[serde(skip_serializing_if = "Option::is_none")]
    phase_a_amps: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    phase_b_amps: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    phase_c_amps: Option<f32>,
    // Multi-turn absolute angle (0x92)
    #[serde(skip_serializing_if = "Option::is_none")]
    multi_turn_centideg: Option<i32>,
    // Multi-turn encoder (0x60/0x61/0x62)
    #[serde(skip_serializing_if = "Option::is_none")]
    multi_turn_encoder_pulses: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    multi_turn_encoder_raw_pulses: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    multi_turn_zero_offset_pulses: Option<i32>,
    // Single-turn encoder/angle (0x90/0x94) — direct-drive models only
    #[serde(skip_serializing_if = "Option::is_none")]
    single_turn_encoder_pulses: Option<i16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    single_turn_encoder_raw_pulses: Option<i16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    single_turn_zero_offset_pulses: Option<i16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    single_turn_angle_centideg: Option<u16>,
    // Run mode / power (0x70/0x71)
    #[serde(skip_serializing_if = "Option::is_none")]
    run_mode: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    motor_power_w: Option<f32>,
    // PID gains (0x30 × 7)
    #[serde(skip_serializing_if = "Option::is_none")]
    current_kp: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    current_ki: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_kp: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_ki: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    position_kp: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    position_ki: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    position_kd: Option<f32>,
    // Acceleration/deceleration limits (0x42)
    #[serde(skip_serializing_if = "Option::is_none")]
    position_accel_dps_s: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    position_decel_dps_s: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_accel_dps_s: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_decel_dps_s: Option<i32>,
    // Uptime / version / model (0xB1 / 0xB2 / 0xB5)
    #[serde(skip_serializing_if = "Option::is_none")]
    uptime_ms: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    version_date_raw: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    motor_model: Option<String>,

    // -- The following are all read via the undocumented `0xC0` generic
    // indexed parameter space (reverse-engineered from Setup Software V4.0
    // traffic against a real X4-36 — see
    // myactuator-protocol/doc/setup-software-c0-param-protocol.md). Distinct
    // wire command from the `0x30`-based PID fields above.

    // Motor Information (0xC0)
    #[serde(skip_serializing_if = "Option::is_none")]
    motor_number: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    factory_time: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    reduction_ratio: Option<f32>,

    // PID Parameters, second gain set: Kd/R(Slope)/T(Filter) per loop (0xC0).
    // No independent D-Axis Current indices exist on the wire — the GUI
    // mirrors Q-Axis Current onto the D-Axis display (doc §1.2).
    #[serde(skip_serializing_if = "Option::is_none")]
    c0_position_kp: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    c0_position_ki: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    c0_position_kd: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    c0_position_t_filter: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    c0_speed_kp: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    c0_speed_ki: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    c0_speed_kd: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    c0_speed_t_filter: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    c0_qaxis_current_kp: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    c0_qaxis_current_ki: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    c0_qaxis_current_kd: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    c0_qaxis_current_r_slope: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    c0_qaxis_current_t_filter: Option<f32>,

    // Encoder / calibration misc (0xC0)
    #[serde(skip_serializing_if = "Option::is_none")]
    enabled_powerdown_save_multiturn: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pole_pairs: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    single_resolution_pulses: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    calibrate_current_a: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    change_motor_direction: Option<f32>,
    /// Uncertain pairing with `encoder_calibrate_value` — see doc §1.4.
    #[serde(skip_serializing_if = "Option::is_none")]
    exchange_phase: Option<f32>,
    /// Uncertain pairing with `exchange_phase` — see doc §1.4.
    #[serde(skip_serializing_if = "Option::is_none")]
    encoder_calibrate_value: Option<f32>,

    // Protect Parameters panel (0xC0)
    #[serde(skip_serializing_if = "Option::is_none")]
    over_voltage_v: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    low_voltage_v: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    stall_time_limit_s: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    ebrake_start_duty_cycle_pct: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    current_sample_res_mohm: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    ebrake_hold_duty_cycle_pct: Option<f32>,
    /// enum: `0.0` = E-Brake, `1.0` = Resistor (doc §1.6).
    #[serde(skip_serializing_if = "Option::is_none")]
    brake_mode: Option<f32>,

    // Plan Parameters panel (0xC0)
    #[serde(skip_serializing_if = "Option::is_none")]
    max_positive_position_deg: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    min_negative_position_deg: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    position_plan_max_acc_dps_s: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    position_plan_max_dec_dps_s: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    position_plan_max_speed_rpm: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_plan_max_acc_dps_s: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    speed_plan_max_dec_dps_s: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    motor_position_zero: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    kt_out: Option<f32>,

    // Motor Parameters panel (0xC0)
    #[serde(skip_serializing_if = "Option::is_none")]
    rated_current_a: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    max_current_a: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    stall_current_a: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    shutdown_temp_c: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    resume_temp_c: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    max_speed_rpm: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    nominal_speed_rpm: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    enable_ethercat: Option<f32>,
    /// Read-only mirror — the actual write path is `function_control(2, _)`,
    /// not `0xC0` (doc §1.5).
    #[serde(skip_serializing_if = "Option::is_none")]
    enable_can_filter: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    enable_2nd_encoder: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    select_thermistor: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    encoder2_abnormal_value: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    encoder2_abnormal_speed: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    automatic_error_recovery: Option<f32>,

    // Output-encoder mapping (0xC0) — names confirmed via
    // `memo_myactuator_001.xlsx` export, exact per-field semantics unknown.
    #[serde(skip_serializing_if = "Option::is_none")]
    out_encoder: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    out_encoder_1: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    out_encoder_2: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    out_encoder_3: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    out_encoder2_1: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    out_encoder2_2: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    out_encoder2_3: Option<f32>,
}

impl AllParams {
    fn read_all<B: myactuator_driver::MyActuatorBus>(motor: &mut MyActuatorMotor<B>) -> Self {
        let status1 = try_read("Status1 (0x9A)", || motor.read_status1());
        let status2 = try_read("Status2 (0x9C)", || motor.read_status2());
        let status3 = try_read("Status3 (0x9D)", || motor.read_status3());
        let pid = try_read("PID gains (0x30)", || motor.read_pid());
        let single_turn = try_read("single-turn encoder (0x90)", || {
            motor.read_single_turn_encoder()
        });
        let run_mode = try_read("run mode (0x70)", || motor.read_run_mode());

        Self {
            motor_id: motor.motor_id(),
            temperature_c: status1.map(|s| s.temperature_c),
            mos_temperature_c: status1.map(|s| s.mos_temperature_c),
            voltage_v: status1.map(|s| s.voltage_v()),
            brake_released: status1.map(|s| s.brake_released),
            error_flags: status1.map(|s| s.error.raw()),
            current_a: status2.map(|s| s.current_a()),
            speed_dps: status2.map(|s| s.speed_dps),
            angle_deg: status2.map(|s| s.angle_deg),
            phase_a_amps: status3.map(|s| s.phase_a_centi_amps as f32 * 0.01),
            phase_b_amps: status3.map(|s| s.phase_b_centi_amps as f32 * 0.01),
            phase_c_amps: status3.map(|s| s.phase_c_centi_amps as f32 * 0.01),
            multi_turn_centideg: try_read("multi-turn angle (0x92)", || {
                motor.read_multi_turn_centideg()
            }),
            multi_turn_encoder_pulses: try_read("multi-turn encoder (0x60)", || {
                motor.read_multi_turn_encoder()
            }),
            multi_turn_encoder_raw_pulses: try_read("multi-turn encoder raw (0x61)", || {
                motor.read_multi_turn_encoder_raw()
            }),
            multi_turn_zero_offset_pulses: try_read("multi-turn zero offset (0x62)", || {
                motor.read_multi_turn_zero_offset()
            }),
            single_turn_encoder_pulses: single_turn.map(|e| e.encoder),
            single_turn_encoder_raw_pulses: single_turn.map(|e| e.encoder_raw),
            single_turn_zero_offset_pulses: single_turn.map(|e| e.encoder_offset),
            single_turn_angle_centideg: try_read("single-turn angle (0x94)", || {
                motor.read_single_turn_angle()
            }),
            run_mode: run_mode.map(|m| format!("{m:?}")),
            motor_power_w: try_read("motor power (0x71)", || motor.read_motor_power()),
            current_kp: pid.map(|g| g.current_kp),
            current_ki: pid.map(|g| g.current_ki),
            speed_kp: pid.map(|g| g.speed_kp),
            speed_ki: pid.map(|g| g.speed_ki),
            position_kp: pid.map(|g| g.position_kp),
            position_ki: pid.map(|g| g.position_ki),
            position_kd: pid.map(|g| g.position_kd),
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
            uptime_ms: try_read("uptime (0xB1)", || motor.read_uptime_ms()),
            version_date_raw: try_read("version date (0xB2)", || motor.read_version_date()),
            motor_model: try_read("motor model (0xB5)", || motor.read_motor_model()),

            // -- 0xC0 generic indexed parameter space --
            motor_number: rp(motor, "motor number", ParamIndex::MotorNumber),
            factory_time: rp(motor, "factory time", ParamIndex::FactoryTime),
            reduction_ratio: rp(motor, "reduction ratio", ParamIndex::ReductionRatio),

            c0_position_kp: rp(motor, "position Kp", ParamIndex::PositionLoopKp),
            c0_position_ki: rp(motor, "position Ki", ParamIndex::PositionLoopKi),
            c0_position_kd: rp(motor, "position Kd", ParamIndex::PositionLoopKd),
            c0_position_t_filter: rp(motor, "position T(Filter)", ParamIndex::PositionLoopTFilter),
            c0_speed_kp: rp(motor, "speed Kp", ParamIndex::SpeedLoopKp),
            c0_speed_ki: rp(motor, "speed Ki", ParamIndex::SpeedLoopKi),
            c0_speed_kd: rp(motor, "speed Kd", ParamIndex::SpeedLoopKd),
            c0_speed_t_filter: rp(motor, "speed T(Filter)", ParamIndex::SpeedLoopTFilter),
            c0_qaxis_current_kp: rp(motor, "Q-axis current Kp", ParamIndex::QAxisCurrentKp),
            c0_qaxis_current_ki: rp(motor, "Q-axis current Ki", ParamIndex::QAxisCurrentKi),
            c0_qaxis_current_kd: rp(motor, "Q-axis current Kd", ParamIndex::QAxisCurrentKd),
            c0_qaxis_current_r_slope: rp(
                motor,
                "Q-axis current R(Slope)",
                ParamIndex::QAxisCurrentRSlope,
            ),
            c0_qaxis_current_t_filter: rp(
                motor,
                "Q-axis current T(Filter)",
                ParamIndex::QAxisCurrentTFilter,
            ),

            enabled_powerdown_save_multiturn: rp(
                motor,
                "powerdown save multiturn",
                ParamIndex::EnabledPowerdownSaveMultiTurn,
            ),
            pole_pairs: rp(motor, "pole pairs", ParamIndex::PolePairs),
            single_resolution_pulses: rp(
                motor,
                "single-resolution pulses",
                ParamIndex::SingleResolutionPulses,
            ),
            calibrate_current_a: rp(motor, "calibrate current", ParamIndex::CalibrateCurrent),
            change_motor_direction: rp(
                motor,
                "change motor direction",
                ParamIndex::ChangeMotorDirection,
            ),
            exchange_phase: rp(motor, "exchange phase", ParamIndex::ExchangePhase),
            encoder_calibrate_value: rp(
                motor,
                "encoder calibrate value",
                ParamIndex::EncoderCalibrateValue,
            ),

            over_voltage_v: rp(motor, "over voltage", ParamIndex::OverVoltage),
            low_voltage_v: rp(motor, "low voltage", ParamIndex::LowVoltage),
            stall_time_limit_s: rp(motor, "stall time limit", ParamIndex::StallTimeLimit),
            ebrake_start_duty_cycle_pct: rp(
                motor,
                "E-Brake start duty cycle",
                ParamIndex::EBrakeStartDutyCycle,
            ),
            current_sample_res_mohm: rp(
                motor,
                "current sample res",
                ParamIndex::CurrentSampleRes,
            ),
            ebrake_hold_duty_cycle_pct: rp(
                motor,
                "E-Brake hold duty cycle",
                ParamIndex::EBrakeHoldDutyCycle,
            ),
            brake_mode: rp(motor, "brake mode", ParamIndex::BrakeMode),

            max_positive_position_deg: rp(
                motor,
                "max positive position",
                ParamIndex::MaxPositivePosition,
            ),
            min_negative_position_deg: rp(
                motor,
                "min negative position",
                ParamIndex::MinNegativePosition,
            ),
            position_plan_max_acc_dps_s: rp(
                motor,
                "position plan max acc",
                ParamIndex::PositionPlanMaxAcc,
            ),
            position_plan_max_dec_dps_s: rp(
                motor,
                "position plan max dec",
                ParamIndex::PositionPlanMaxDec,
            ),
            position_plan_max_speed_rpm: rp(
                motor,
                "position plan max speed",
                ParamIndex::PositionPlanMaxSpeed,
            ),
            speed_plan_max_acc_dps_s: rp(
                motor,
                "speed plan max acc",
                ParamIndex::SpeedPlanMaxAcc,
            ),
            speed_plan_max_dec_dps_s: rp(
                motor,
                "speed plan max dec",
                ParamIndex::SpeedPlanMaxDec,
            ),
            motor_position_zero: rp(motor, "motor position zero", ParamIndex::MotorPositionZero),
            kt_out: rp(motor, "KT_OUT", ParamIndex::KtOut),

            rated_current_a: rp(motor, "rated current", ParamIndex::RatedCurrent),
            max_current_a: rp(motor, "max current", ParamIndex::MaxCurrent),
            stall_current_a: rp(motor, "stall current", ParamIndex::StallCurrent),
            shutdown_temp_c: rp(motor, "shutdown temp", ParamIndex::ShutdownTemp),
            resume_temp_c: rp(motor, "resume temp", ParamIndex::ResumeTemp),
            max_speed_rpm: rp(motor, "max speed", ParamIndex::MaxSpeed),
            nominal_speed_rpm: rp(motor, "nominal speed", ParamIndex::NominalSpeed),
            enable_ethercat: rp(motor, "enable EtherCAT", ParamIndex::EnableEtherCat),
            enable_can_filter: rp(motor, "enable CAN filter", ParamIndex::EnableCanFilter),
            enable_2nd_encoder: rp(motor, "enable 2nd encoder", ParamIndex::Enable2ndEncoder),
            select_thermistor: rp(motor, "select thermistor", ParamIndex::SelectThermistor),
            encoder2_abnormal_value: rp(
                motor,
                "encoder2 abnormal value",
                ParamIndex::Encoder2AbnormalValue,
            ),
            encoder2_abnormal_speed: rp(
                motor,
                "encoder2 abnormal speed",
                ParamIndex::Encoder2AbnormalSpeed,
            ),
            automatic_error_recovery: rp(
                motor,
                "automatic error recovery",
                ParamIndex::AutomaticErrorRecovery,
            ),

            out_encoder: rp(motor, "OUTENCODER", ParamIndex::OutEncoder),
            out_encoder_1: rp(motor, "OUTENCODER_1", ParamIndex::OutEncoder1),
            out_encoder_2: rp(motor, "OUTENCODER_2", ParamIndex::OutEncoder2),
            out_encoder_3: rp(motor, "OUTENCODER_3", ParamIndex::OutEncoder3),
            out_encoder2_1: rp(motor, "OUTENCODER2_1", ParamIndex::OutEncoder2_1),
            out_encoder2_2: rp(motor, "OUTENCODER2_2", ParamIndex::OutEncoder2_2),
            out_encoder2_3: rp(motor, "OUTENCODER2_3", ParamIndex::OutEncoder2_3),
        }
    }

    /// One table row: the source command byte, field name, formatted value,
    /// and unit — kept as plain strings so the column widths can be measured
    /// and the whole thing aligned regardless of terminal width.
    fn rows(&self) -> Vec<[String; 4]> {
        fn fmt<T: std::fmt::Display>(v: &Option<T>) -> String {
            v.as_ref().map(|v| v.to_string()).unwrap_or_else(|| "—".to_string())
        }
        fn fmt_f32(v: Option<f32>, prec: usize) -> String {
            v.map(|v| format!("{v:.prec$}")).unwrap_or_else(|| "—".to_string())
        }
        fn row(cmd: &str, field: &str, value: impl Into<String>, unit: &str) -> [String; 4] {
            [cmd.to_string(), field.to_string(), value.into(), unit.to_string()]
        }

        let model = match self.motor_model.as_deref() {
            None => "—".to_string(),
            Some("") => "<none> (empty on this FW)".to_string(),
            Some(m) => m.to_string(),
        };
        let error_hex = self
            .error_flags
            .map(|e| format!("0x{e:04X}"))
            .unwrap_or_else(|| "—".to_string());
        let single_turn_deg = self
            .single_turn_angle_centideg
            .map(|v| format!("{:.2}", v as f32 / 100.0))
            .unwrap_or_else(|| "—".to_string());

        vec![
            row("0x9A", "temperature_c", fmt(&self.temperature_c), "°C"),
            row("0x9A", "mos_temperature_c", fmt(&self.mos_temperature_c), "°C"),
            row("0x9A", "voltage_v", fmt_f32(self.voltage_v, 2), "V"),
            row("0x9A", "brake_released", fmt(&self.brake_released), ""),
            row("0x9A", "error_flags", error_hex, ""),
            row("0x9C", "current_a", fmt_f32(self.current_a, 2), "A"),
            row("0x9C", "speed_dps", fmt(&self.speed_dps), "dps"),
            row("0x9C", "angle_deg", fmt(&self.angle_deg), "°"),
            row("0x9D", "phase_a_amps", fmt_f32(self.phase_a_amps, 2), "A"),
            row("0x9D", "phase_b_amps", fmt_f32(self.phase_b_amps, 2), "A"),
            row("0x9D", "phase_c_amps", fmt_f32(self.phase_c_amps, 2), "A"),
            row("0x92", "multi_turn_centideg", fmt(&self.multi_turn_centideg), "centideg"),
            row("0x60", "multi_turn_encoder_pulses", fmt(&self.multi_turn_encoder_pulses), "pulses"),
            row("0x61", "multi_turn_encoder_raw_pulses", fmt(&self.multi_turn_encoder_raw_pulses), "pulses"),
            row("0x62", "multi_turn_zero_offset_pulses", fmt(&self.multi_turn_zero_offset_pulses), "pulses"),
            row("0x90", "single_turn_encoder_pulses", fmt(&self.single_turn_encoder_pulses), "pulses"),
            row("0x90", "single_turn_encoder_raw_pulses", fmt(&self.single_turn_encoder_raw_pulses), "pulses"),
            row("0x90", "single_turn_zero_offset_pulses", fmt(&self.single_turn_zero_offset_pulses), "pulses"),
            row("0x94", "single_turn_angle_deg", single_turn_deg, "°"),
            row("0x70", "run_mode", self.run_mode.clone().unwrap_or_else(|| "—".to_string()), ""),
            row("0x71", "motor_power_w", fmt_f32(self.motor_power_w, 1), "W"),
            row("0x30", "current_kp", fmt_f32(self.current_kp, 4), ""),
            row("0x30", "current_ki", fmt_f32(self.current_ki, 4), ""),
            row("0x30", "speed_kp", fmt_f32(self.speed_kp, 4), ""),
            row("0x30", "speed_ki", fmt_f32(self.speed_ki, 4), ""),
            row("0x30", "position_kp", fmt_f32(self.position_kp, 4), ""),
            row("0x30", "position_ki", fmt_f32(self.position_ki, 4), ""),
            row("0x30", "position_kd", fmt_f32(self.position_kd, 4), ""),
            row("0x42", "position_accel_dps_s", fmt(&self.position_accel_dps_s), "dps/s"),
            row("0x42", "position_decel_dps_s", fmt(&self.position_decel_dps_s), "dps/s"),
            row("0x42", "speed_accel_dps_s", fmt(&self.speed_accel_dps_s), "dps/s"),
            row("0x42", "speed_decel_dps_s", fmt(&self.speed_decel_dps_s), "dps/s"),
            row("0xB1", "uptime_ms", fmt(&self.uptime_ms), "ms"),
            row("0xB2", "version_date_raw", fmt(&self.version_date_raw), ""),
            row("0xB5", "motor_model", model, ""),
            // -- 0xC0 generic indexed parameter space --
            row("0xC0", "motor_number", fmt_f32(self.motor_number, 0), ""),
            row("0xC0", "factory_time", fmt_f32(self.factory_time, 0), ""),
            row("0xC0", "reduction_ratio", fmt_f32(self.reduction_ratio, 2), ""),
            row("0xC0", "c0_position_kp", fmt_f32(self.c0_position_kp, 4), ""),
            row("0xC0", "c0_position_ki", fmt_f32(self.c0_position_ki, 4), ""),
            row("0xC0", "c0_position_kd", fmt_f32(self.c0_position_kd, 4), ""),
            row("0xC0", "c0_position_t_filter", fmt_f32(self.c0_position_t_filter, 4), ""),
            row("0xC0", "c0_speed_kp", fmt_f32(self.c0_speed_kp, 4), ""),
            row("0xC0", "c0_speed_ki", fmt_f32(self.c0_speed_ki, 4), ""),
            row("0xC0", "c0_speed_kd", fmt_f32(self.c0_speed_kd, 4), ""),
            row("0xC0", "c0_speed_t_filter", fmt_f32(self.c0_speed_t_filter, 4), ""),
            row("0xC0", "c0_qaxis_current_kp", fmt_f32(self.c0_qaxis_current_kp, 4), ""),
            row("0xC0", "c0_qaxis_current_ki", fmt_f32(self.c0_qaxis_current_ki, 4), ""),
            row("0xC0", "c0_qaxis_current_kd", fmt_f32(self.c0_qaxis_current_kd, 4), ""),
            row(
                "0xC0",
                "c0_qaxis_current_r_slope",
                fmt_f32(self.c0_qaxis_current_r_slope, 4),
                "",
            ),
            row(
                "0xC0",
                "c0_qaxis_current_t_filter",
                fmt_f32(self.c0_qaxis_current_t_filter, 4),
                "",
            ),
            row(
                "0xC0",
                "enabled_powerdown_save_multiturn",
                fmt_f32(self.enabled_powerdown_save_multiturn, 0),
                "",
            ),
            row("0xC0", "pole_pairs", fmt_f32(self.pole_pairs, 0), ""),
            row(
                "0xC0",
                "single_resolution_pulses",
                fmt_f32(self.single_resolution_pulses, 0),
                "pulses",
            ),
            row("0xC0", "calibrate_current_a", fmt_f32(self.calibrate_current_a, 2), "A"),
            row(
                "0xC0",
                "change_motor_direction",
                fmt_f32(self.change_motor_direction, 0),
                "",
            ),
            row("0xC0", "exchange_phase", fmt_f32(self.exchange_phase, 0), ""),
            row(
                "0xC0",
                "encoder_calibrate_value",
                fmt_f32(self.encoder_calibrate_value, 0),
                "",
            ),
            row("0xC0", "over_voltage_v", fmt_f32(self.over_voltage_v, 2), "V"),
            row("0xC0", "low_voltage_v", fmt_f32(self.low_voltage_v, 2), "V"),
            row("0xC0", "stall_time_limit_s", fmt_f32(self.stall_time_limit_s, 2), "s"),
            row(
                "0xC0",
                "ebrake_start_duty_cycle_pct",
                fmt_f32(self.ebrake_start_duty_cycle_pct, 1),
                "%",
            ),
            row(
                "0xC0",
                "current_sample_res_mohm",
                fmt_f32(self.current_sample_res_mohm, 2),
                "mΩ",
            ),
            row(
                "0xC0",
                "ebrake_hold_duty_cycle_pct",
                fmt_f32(self.ebrake_hold_duty_cycle_pct, 1),
                "%",
            ),
            row("0xC0", "brake_mode", fmt_f32(self.brake_mode, 0), ""),
            row(
                "0xC0",
                "max_positive_position_deg",
                fmt_f32(self.max_positive_position_deg, 1),
                "°",
            ),
            row(
                "0xC0",
                "min_negative_position_deg",
                fmt_f32(self.min_negative_position_deg, 1),
                "°",
            ),
            row(
                "0xC0",
                "position_plan_max_acc_dps_s",
                fmt_f32(self.position_plan_max_acc_dps_s, 1),
                "dps/s",
            ),
            row(
                "0xC0",
                "position_plan_max_dec_dps_s",
                fmt_f32(self.position_plan_max_dec_dps_s, 1),
                "dps/s",
            ),
            row(
                "0xC0",
                "position_plan_max_speed_rpm",
                fmt_f32(self.position_plan_max_speed_rpm, 1),
                "rpm",
            ),
            row(
                "0xC0",
                "speed_plan_max_acc_dps_s",
                fmt_f32(self.speed_plan_max_acc_dps_s, 1),
                "dps/s",
            ),
            row(
                "0xC0",
                "speed_plan_max_dec_dps_s",
                fmt_f32(self.speed_plan_max_dec_dps_s, 1),
                "dps/s",
            ),
            row(
                "0xC0",
                "motor_position_zero",
                fmt_f32(self.motor_position_zero, 1),
                "",
            ),
            row("0xC0", "kt_out", fmt_f32(self.kt_out, 4), "N·m/A"),
            row("0xC0", "rated_current_a", fmt_f32(self.rated_current_a, 3), "A"),
            row("0xC0", "max_current_a", fmt_f32(self.max_current_a, 2), "A"),
            row("0xC0", "stall_current_a", fmt_f32(self.stall_current_a, 2), "A"),
            row("0xC0", "shutdown_temp_c", fmt_f32(self.shutdown_temp_c, 1), "°C"),
            row("0xC0", "resume_temp_c", fmt_f32(self.resume_temp_c, 1), "°C"),
            row("0xC0", "max_speed_rpm", fmt_f32(self.max_speed_rpm, 1), "rpm"),
            row("0xC0", "nominal_speed_rpm", fmt_f32(self.nominal_speed_rpm, 1), "rpm"),
            row("0xC0", "enable_ethercat", fmt_f32(self.enable_ethercat, 0), ""),
            row("0xC0", "enable_can_filter", fmt_f32(self.enable_can_filter, 0), ""),
            row("0xC0", "enable_2nd_encoder", fmt_f32(self.enable_2nd_encoder, 0), ""),
            row("0xC0", "select_thermistor", fmt_f32(self.select_thermistor, 0), ""),
            row(
                "0xC0",
                "encoder2_abnormal_value",
                fmt_f32(self.encoder2_abnormal_value, 1),
                "",
            ),
            row(
                "0xC0",
                "encoder2_abnormal_speed",
                fmt_f32(self.encoder2_abnormal_speed, 2),
                "",
            ),
            row(
                "0xC0",
                "automatic_error_recovery",
                fmt_f32(self.automatic_error_recovery, 0),
                "",
            ),
            row("0xC0", "out_encoder", fmt_f32(self.out_encoder, 0), ""),
            row("0xC0", "out_encoder_1", fmt_f32(self.out_encoder_1, 0), ""),
            row("0xC0", "out_encoder_2", fmt_f32(self.out_encoder_2, 0), ""),
            row("0xC0", "out_encoder_3", fmt_f32(self.out_encoder_3, 0), ""),
            row("0xC0", "out_encoder2_1", fmt_f32(self.out_encoder2_1, 0), ""),
            row("0xC0", "out_encoder2_2", fmt_f32(self.out_encoder2_2, 0), ""),
            row("0xC0", "out_encoder2_3", fmt_f32(self.out_encoder2_3, 0), ""),
        ]
    }

    fn print(&self) {
        let rows = self.rows();
        let cmd_w = rows.iter().map(|r| r[0].len()).max().unwrap_or(0).max(3);
        let field_w = rows.iter().map(|r| r[1].len()).max().unwrap_or(0).max(5);
        let value_w = rows.iter().map(|r| r[2].len()).max().unwrap_or(0).max(5);

        println!("== MyActuator params (motor_id={}) ==", self.motor_id);
        let header = format!(
            "{:<cmd_w$}  {:<field_w$}  {:<value_w$}  unit",
            "cmd", "field", "value"
        );
        println!("{header}");
        println!("{}", "-".repeat(header.chars().count()));
        for r in &rows {
            println!(
                "{:<cmd_w$}  {:<field_w$}  {:<value_w$}  {}",
                r[0], r[1], r[2], r[3]
            );
        }
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

/// [`try_read`] specialized for the `0xC0` generic indexed parameter space —
/// formats the warning label as `"{label} (0xC0/0x{index:02X})"`.
fn rp<B: myactuator_driver::MyActuatorBus>(
    motor: &mut MyActuatorMotor<B>,
    label: &str,
    index: ParamIndex,
) -> Option<f32> {
    try_read(&format!("{label} (0xC0/0x{:02X})", index as u8), || {
        motor.read_param(index)
    })
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
                    println!("{}", fmt_fb_coarse(fb));
                    last_print = Instant::now();
                }
            }
            Err(e) => {
                eprintln!("control error: {e}");
                break;
            }
        }
        if let Some(rem) = period.checked_sub(loop_start.elapsed()) {
            misa_actuator::realtime::sleep_precise(rem);
        }
    }
    Ok(())
}

/// Where the shaft actually ended up, read at full resolution.
///
/// The loop above prints the coarse 1°/LSB position that control replies
/// carry; this asks `0x92` (0.01°/LSB) once the motion is over, so the number
/// you judge the move by is the accurate one. Reported, not fatal — a failed
/// read here says nothing about whether the move succeeded.
fn print_settled<B: myactuator_driver::MyActuatorBus>(
    motor: &mut myactuator_driver::MyActuatorMotor<B>,
) {
    match motor.measure() {
        Ok(fb) => println!("settled: {}", fmt_fb(fb)),
        Err(e) => eprintln!("settled position unavailable: {e}"),
    }
}

fn fmt_fb(fb: MotorFeedback) -> String {
    format!(
        "pos={:+.3} rad  vel={:+.3} rad/s  tau={:+.3} Nm  iq={:+.2} A  T={} °C",
        fb.position_rad, fb.velocity_rad_per_s, fb.torque_nm, fb.current_a, fb.temperature_c
    )
}

/// Position out of a control reply, printed at the precision it actually has.
///
/// Control replies (`0x9C`) carry the angle at **1°/LSB**, so the position in
/// them is good to about 0.017 rad — three decimals is false precision. On a
/// small move that reads as a large error: a `move-to 0.1` that lands exactly
/// on target displays as `+0.116`, which looks like a 16% overshoot and is
/// really one count of quantisation. Two decimals, plus an explicit final
/// reading from `0x92` (0.01°/LSB) once the move is done.
fn fmt_fb_coarse(fb: MotorFeedback) -> String {
    format!(
        "pos={:+.2} rad (±0.02, 1°/LSB)  vel={:+.3} rad/s  tau={:+.3} Nm  iq={:+.2} A  T={} °C",
        fb.position_rad, fb.velocity_rad_per_s, fb.torque_nm, fb.current_a, fb.temperature_c
    )
}
