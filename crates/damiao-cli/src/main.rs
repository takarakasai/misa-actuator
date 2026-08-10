//! Test CLI for the `damiao-driver` crate (DM-J4310-2EC and friends).
//!
//! `-i` names the CAN interface: `can0` on Linux, `pcan:usb1` for a PEAK
//! adapter on Windows, `slcan:COM5` for a USB-CAN dongle. `--fd` needs a
//! CAN-FD-capable transport — SocketCAN with `fd on`, or a PEAK FD adapter;
//! SLCAN cannot do FD.
//!
//! Classic CAN (1 Mbps):
//! ```text
//! sudo ip link set can0 type can bitrate 1000000 up      # Linux only
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

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};
use serde::Serialize;

use damiao_driver::{
    scan_bus_on, ControlMode, DamiaoBus, DamiaoMotor, Feedback, LimitsSource, ModelReg, MotorModel,
    Rid,
};

#[derive(Parser, Debug)]
#[command(version, about = "Test CLI for DAMIAO CAN/CAN-FD servo motors")]
struct Cli {
    /// CAN interface: `can0` (Linux SocketCAN), `pcan:usb1` (PEAK adapter on
    /// Windows) or `slcan:COM5` (USB-CAN adapter, classic CAN only). Append
    /// `@1M,5M` to set the arbitration and CAN-FD data bitrates.
    #[arg(short, long, default_value = misa_can::default_interface())]
    interface: String,

    /// Motor CAN_ID (slave id).
    #[arg(short, long, default_value_t = 1)]
    motor_id: u8,

    /// Motor Master ID — the standard CAN id feedback is reported on (11-bit).
    /// Default 0, which accepts any responder (matched by the feedback id
    /// nibble). Set this on a multi-motor bus where CAN_IDs share a low nibble.
    #[arg(long, default_value_t = 0)]
    master_id: u16,

    /// Motor model. One of DM4310, DM4310P, DM4340, DM4340P, DM3507, DM6248P,
    /// DM8009, DM8009P, DM10422P (full part numbers like `DM-J4340P-2EC` also
    /// work). Selects the MIT quantization ranges and the register layout.
    #[arg(long, default_value = "DM4310")]
    model: String,

    /// Read the motor's PMAX/VMAX/TMAX registers and use those as the MIT
    /// quantization ranges, instead of this model's compiled-in defaults.
    ///
    /// The defaults are only defaults: no official manual states concrete
    /// mapping ranges, and the vendor SDK's table disagrees with itself. Pass
    /// this before relying on `mit` scaling — a mismatch silently mis-scales
    /// commands and feedback rather than erroring.
    #[arg(long)]
    refresh_limits: bool,

    /// Read the motor's torque constant so feedback can report current.
    ///
    /// DAMIAO's feedback frame carries torque but no current, so current is
    /// derived as `torque / Kt`. Kt comes from `KT_Value` when that holds an
    /// override, else from the manuals' `1.5 * Npp * flux * Gr * GREF` using the
    /// motor's own identified parameters — `KT_Value == 0` is the normal state,
    /// meaning "use those parameters". Costs a few register reads.
    #[arg(long)]
    refresh_kt: bool,

    /// Use a CAN-FD bus (interface must be `fd on`). Default is classic CAN.
    #[arg(long)]
    fd: bool,

    /// Diagnostic: initialise the link as CAN-FD but send classic frames.
    ///
    /// Use when a motor answers on `--fd`-less runs and goes silent with
    /// `--fd`. If it answers here, the FD initialisation and bit timing are
    /// good and the motor itself is not an FD node; if it does not, the
    /// problem is on our side of the link.
    #[arg(long, conflicts_with = "fd")]
    fd_link_classic_frames: bool,

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
    /// De-energize the motor (coast) — not a latching safe state
    ///
    /// The next control frame re-energizes the motor, so this does not make it
    /// ignore commands: verified on a DM-J4310, a `mit --kp 5` right after
    /// `disable` produced 1.013 N·m. `status` also sends a zero-gain MIT frame,
    /// which is why it still reports `err=Enabled` afterwards (at zero gains it
    /// produces no torque). Cut power for a state the motor will hold.
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
    /// One status read (re-issues the last command / a zero MIT frame)
    ///
    /// Note this sends a control frame, so it re-energizes a motor that was
    /// just disabled and will report `err=Enabled`. Harmless — the frame
    /// carries zero gains and zero torque — but it means `status` is not a
    /// passive observer. See `disable`.
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
    /// Read all 38 registers of the manuals' `0x00`-`0x25` block in one pass
    ///
    /// Covers protection thresholds, motion profile, addressing, MIT mapping
    /// ranges, all control-loop gains, identified motor constants and version
    /// fields. A register that doesn't reply is reported and omitted rather
    /// than aborting the whole dump — expected for `boot_ver` on a
    /// DM-J3507-2EC, which does not document RID `0x25`. Model-specific
    /// registers above `0x25` are excluded: their addresses differ between
    /// DM4310 and DM3507, so a flat dump would read the wrong register on one
    /// of the two models.
    Params {
        /// Also emit a TOML dump to stdout (or to --out if given).
        #[arg(long)]
        toml: bool,
        /// Write the TOML dump to this file (implies --toml).
        #[arg(long)]
        out: Option<PathBuf>,
    },
    /// Read the model-specific diagnostic registers above `0x25`
    ///
    /// These live where the DM4310 and DM3507 register maps diverge, so each
    /// one is resolved against `--model` rather than a fixed RID (`m_off`
    /// alone sits at `0x38` on DM4310 but `0x36` on DM3507). Registers this
    /// model's map does not document are listed as `<not on this model>` and
    /// are never read, so no undocumented address is ever polled.
    ///
    /// On DM4310 this includes bus voltage and PCB/motor temperatures, which
    /// the CAN feedback frame does not carry.
    Diag {
        /// Also emit a TOML dump to stdout (or to --out if given).
        #[arg(long)]
        toml: bool,
        /// Write the TOML dump to this file (implies --toml).
        #[arg(long)]
        out: Option<PathBuf>,
    },
    /// Quasi-static characterization: load map, breakaway torque, thermal, Kt
    ///
    /// These deliberately hold torque against a loaded or stalled shaft, so
    /// every run is bounded by a safety envelope (`--max-torque`, `--max-temp`,
    /// `--max-temp-rate`, `--window`, `--max-duration`) and aborts on the first
    /// breach, keeping whatever it collected. Ctrl-C also aborts.
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
        /// Abort once temperature climbs faster than this (°C/s). This is the
        /// check that actually protects a motor which heats at several °C/s —
        /// the absolute limit trips too late.
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
    /// Read a register (RID).
    RegRead {
        rid: u8,
    },
    /// Write a register (RID). Float by default; pass --int for an integer.
    ///
    /// The value is written as given. Nothing here refuses a write, but two
    /// warnings come first: one when the register can make the motor
    /// unreachable (ids, bitrate, timeout), silently mis-scale every command
    /// (PMAX/VMAX/TMAX), weaken a protection threshold or overwrite a
    /// read-only calibration value; and one when the value falls outside the
    /// range the manual documents for it.
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

/// How a `reg-write` to a given register can go wrong.
///
/// `reg-write` takes a bare RID and writes whatever number follows it, and
/// `--save` commits that to flash in the same breath. `lkmotor-cli` already
/// warns before the writes that can make a motor unreachable
/// (`SettingParamArg::is_risky`); this is the same guard, widened to the
/// failure modes this register block adds.
///
/// Warn and proceed, rather than refuse: this is a diagnostic tool and the
/// operator may well mean it. What was missing is any statement of what the
/// write does before it goes out on the wire.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RegisterRisk {
    /// Changes where or how the motor talks. Get it wrong and it stops
    /// answering on this interface.
    Communication,
    /// The MIT quantisation range itself.
    Scaling,
    /// A protection threshold.
    Protection,
    /// Read-only per the manuals: calibration constants and identity.
    ReadOnly,
    /// Writable, with no documented hazard beyond `--save` being permanent.
    Ordinary,
}

impl RegisterRisk {
    /// What to tell the operator, or `None` when there is nothing specific to
    /// say. Phrased as the consequence, not the category.
    fn hazard(self) -> Option<&'static str> {
        match self {
            RegisterRisk::Communication => Some(
                "changes the CAN id, master id, bitrate or timeout this motor communicates \
                 on. If the new value is wrong the motor stops answering on this interface \
                 — a RobStride unit took a day to come back from the equivalent mistake. \
                 Change one motor at a time and write down the previous value.",
            ),
            RegisterRisk::Scaling => Some(
                "is one of PMAX/VMAX/TMAX, the MIT quantisation ranges. Every position, \
                 velocity and torque value on the wire is scaled by these, so a wrong one \
                 produces no error and no fault — just every command silently off by a \
                 factor.",
            ),
            RegisterRisk::Protection => Some(
                "is a protection threshold (under/over-voltage, over-temperature, \
                 over-current). Loosening it removes the limit that stops the motor \
                 damaging itself.",
            ),
            RegisterRisk::ReadOnly => Some(
                "is read-only in the manuals — a calibration or identity value the motor \
                 measured for itself. Of the gear ratio the manual says: \"This parameter \
                 is preconfigured. Do not modify.\" Nothing in this tool can restore the \
                 original value.",
            ),
            RegisterRisk::Ordinary => None,
        }
    }
}

/// Classify a raw RID. Ranges come from [`Rid`]'s own documentation, which is
/// transcribed from the DM-J4310-2EC / DM-J3507-2EC manuals.
fn register_risk(rid: u8) -> RegisterRisk {
    match rid {
        Rid::MST_ID | Rid::ESC_ID | Rid::TIMEOUT | Rid::CAN_BR => RegisterRisk::Communication,
        Rid::PMAX | Rid::VMAX | Rid::TMAX => RegisterRisk::Scaling,
        Rid::UV_VALUE | Rid::OV_VALUE | Rid::OT_VALUE | Rid::OC_VALUE => RegisterRisk::Protection,
        Rid::DAMP
        | Rid::INERTIA
        | Rid::HW_VER
        | Rid::SW_VER
        | Rid::SN
        | Rid::NPP
        | Rid::RS
        | Rid::LS
        | Rid::FLUX
        | Rid::GR
        | Rid::SUB_VER
        | Rid::BOOT_VER => RegisterRisk::ReadOnly,
        _ => RegisterRisk::Ordinary,
    }
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    // Windows sleeps round up to the ~15.6 ms scheduler tick by default,
    // which would throttle every timed loop below. No-op on Linux.
    let _timer = misa_actuator::realtime::TimerResolutionGuard::acquire();
    let cli = Cli::parse();

    let model = MotorModel::from_name(&cli.model)
        .with_context(|| format!("unknown DAMIAO model: {}", cli.model))?;
    let timeout = Duration::from_millis(cli.timeout_ms);

    // The bus type is chosen at runtime; dispatch into a generic runner so the
    // command logic is written once for both transports.
    if cli.fd_link_classic_frames {
        let mut motor =
            DamiaoMotor::open_fd_link_classic_frames(&cli.interface, cli.motor_id, model)
                .with_context(|| format!("failed to open CAN-FD interface {}", cli.interface))?;
        eprintln!(
            "diagnostic: link initialised as CAN-FD, frames sent as classic CAN.\n\
             If the motor answers here but not with --fd, the FD bit timing is \
             fine and the motor is not configured for CAN-FD."
        );
        configure(&mut motor, &cli, timeout)?;
        run_and_report(&mut motor, &cli)
    } else if cli.fd {
        let mut motor = DamiaoMotor::open_fd(&cli.interface, cli.motor_id, model)
            .with_context(|| format!("failed to open CAN-FD interface {}", cli.interface))?;
        configure(&mut motor, &cli, timeout)?;
        run_and_report(&mut motor, &cli)
    } else {
        let mut motor = DamiaoMotor::open(&cli.interface, cli.motor_id, model)
            .with_context(|| format!("failed to open CAN interface {}", cli.interface))?;
        configure(&mut motor, &cli, timeout)?;
        run_and_report(&mut motor, &cli)
    }
}

/// Run the command, then say whether the transport lost anything on the way.
///
/// Reported after every subcommand rather than on request, because at the
/// moment it matters nobody knows to ask: a reply dropped in the host receive
/// queue and a motor that never answered produce the same timeout, and the
/// difference decides whether the bus is even a suspect. `0` is silent, so
/// ordinary use stays quiet — see `doc/handover.md` §9 for how to read that
/// during the CAN-FD re-measurement.
fn run_and_report<B: DamiaoBus>(motor: &mut DamiaoMotor<B>, cli: &Cli) -> Result<()> {
    let result = run(motor, cli);

    let lost = motor.rx_overruns();
    if lost > 0 {
        eprintln!(
            "\nnote: the transport reported {lost} receive overrun(s) during this run. \
             That many reads found frames already dropped on this side of the wire, so \
             any timeout above may be a lost reply rather than a silent motor."
        );
    }

    result
}

fn configure<B: DamiaoBus>(
    motor: &mut DamiaoMotor<B>,
    cli: &Cli,
    timeout: Duration,
) -> Result<()> {
    motor.set_timeout(timeout)?;
    // Match feedback on the requested Master ID (default 0).
    motor.set_master_id(cli.master_id);

    if cli.refresh_kt {
        match motor.refresh_torque_constant()? {
            Some(kt) => println!("torque constant: {kt} N·m/A — feedback current = torque / Kt"),
            None => eprintln!(
                "warning: could not determine a torque constant; feedback current stays NaN"
            ),
        }
    }

    let model = motor.model();
    if cli.refresh_limits {
        let l = motor
            .refresh_limits_from_registers()
            .context("failed to read PMAX/VMAX/TMAX from the motor")?;
        println!(
            "MIT ranges from motor registers: PMAX={} VMAX={} TMAX={}",
            l.p_max, l.v_max, l.t_max
        );
    } else if !matches!(model.limits_source(), LimitsSource::Sdk) {
        // Only warn where the defaults are not even SDK-backed; staying quiet
        // on every invocation would bury the models that actually need it.
        let why = match model.limits_source() {
            LimitsSource::SdkSiblingIdenticalSpecs => {
                "inherited from its non-P sibling (identical published specs)"
            }
            LimitsSource::ManualSpecsFloor => {
                "a floor derived from the manual's spec table, most likely not the firmware's value"
            }
            LimitsSource::Sdk => unreachable!(),
        };
        eprintln!(
            "warning: {}'s MIT ranges are {why}. Pass --refresh-limits to read \
             PMAX/VMAX/TMAX from the motor before trusting `mit` scaling.",
            model.name()
        );
    }
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
                // The soft path below is free and reversible; this one is
                // neither, and the flag that picks it is one character.
                eprintln!(
                    "warning: --nvm writes the zero to flash. It survives a power cycle, \
                     it replaces the previous zero with no way back to it from here, and \
                     flash wears out. Without --nvm the zero is in memory only."
                );
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
                ("GR (gear ratio)", Rid::GR, false),
                ("PMAX", Rid::PMAX, false),
                ("VMAX", Rid::VMAX, false),
                ("TMAX", Rid::TMAX, false),
            ];
            let model = motor.model();
            println!("motor config (addressed at CAN_ID {}):", cli.motor_id);
            println!(
                "  {:<16}          = {} (register layout {:?})  [selected]",
                "model",
                model.name(),
                model.register_layout()
            );
            // What the motor says it is, which need not be what --model says.
            match motor.identify_model() {
                Ok(m) if m.register_layout() == model.register_layout() => println!(
                    "  {:<16}          = {} (gear ratio {}:1)  [from the motor]",
                    "identified",
                    m.name(),
                    m.gear_ratio()
                ),
                Ok(m) => println!(
                    "  {:<16}          = {} (gear ratio {}:1)  [from the motor] \
                     — MISMATCH, use --model {}",
                    "identified",
                    m.name(),
                    m.gear_ratio(),
                    m.name()
                ),
                Err(e) => println!("  {:<16}          = <{e}>", "identified"),
            }
            for &(label, rid, is_int) in regs {
                match motor.read_register(rid) {
                    Ok(r) if is_int => println!("  {label:<16} (RID {rid:>2}) = {}", r.as_i32()),
                    Ok(r) => println!("  {label:<16} (RID {rid:>2}) = {}", r.as_f32()),
                    Err(e) => println!("  {label:<16} (RID {rid:>2}) = <no reply: {e}>"),
                }
            }
            // PMAX/VMAX/TMAX above are the motor's own values; contrast them
            // with the compiled-in defaults MIT scaling uses by default, so a
            // mismatch is visible rather than silent.
            let l = motor.limits();
            println!(
                "  MIT ranges in effect: PMAX={} VMAX={} TMAX={}  (source: {:?})",
                l.p_max, l.v_max, l.t_max, model.limits_source()
            );
            if !cli.refresh_limits {
                println!(
                    "  note: pass --refresh-limits to adopt the register values above for MIT scaling."
                );
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
        Command::Params { toml, out } => {
            let regs = read_all_regs(motor, cli.motor_id);
            regs.print();
            if *toml || out.is_some() {
                let text =
                    toml::to_string_pretty(&regs).context("failed to serialize TOML dump")?;
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
        Command::Diag { toml, out } => {
            let diag = read_model_regs(motor, cli.motor_id);
            diag.print();
            if *toml || out.is_some() {
                let text = toml::to_string_pretty(&diag.to_toml())
                    .context("failed to serialize TOML dump")?;
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
            // Before the write, not after: this is the only thing on the path
            // that says what the RID means.
            if let Some(hazard) = register_risk(*rid).hazard() {
                eprintln!("warning: RID {rid} {hazard}");
            }
            // Warn rather than refuse, for the same reason as the hazard above
            // and one more: the manuals and the firmware are known to disagree
            // (handover §8), so a range from the manual is evidence, not
            // authority. Refusing on it would block writes the motor accepts.
            if let Some(range) = Rid::documented_range(*rid) {
                if !range.contains(*value) {
                    eprintln!(
                        "warning: {value} is outside the range the manual documents for \
                         RID {rid} ({range}). Check the value before letting it reach flash."
                    );
                }
            }
            if *save {
                eprintln!(
                    "warning: --save commits this to flash straight away. It survives a \
                     power cycle, and flash wears out."
                );
            }
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
            misa_sysid::run_characterize(&mut *motor, what, limits, out)?;
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
                &mut *motor,
                channel,
                *f0,
                *f1,
                *duration,
                *amp,
                *log,
                *rate,
                *max_speed,
                *kp,
                *kd,
                out,
                bode,
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

/// Send a control command repeatedly at ~100 Hz (to satisfy the comm-loss
/// watchdog) for `duration` seconds, or until Ctrl-C if `duration` is `None`.
/// Prints the latest feedback periodically.
///
/// How many feedback frames may be missed in a row before the loop stops
/// commanding. Five at a 10 ms period is 50 ms of driving blind — long enough
/// to ride out the isolated drops seen on CAN-FD, short enough that a motor
/// which has genuinely stopped answering is not commanded onward.
const MAX_CONSECUTIVE_MISSES: u32 = 5;

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
    let mut missed = 0u32;
    let mut missed_total = 0u32;
    let mut ticks = 0u32;
    while running.load(Ordering::SeqCst) {
        if let Some(d) = duration {
            if start.elapsed().as_secs_f32() >= d {
                break;
            }
        }
        let loop_start = Instant::now();
        ticks += 1;
        match tick() {
            Ok(fb) => {
                missed = 0;
                if last_print.elapsed() >= Duration::from_millis(200) {
                    println!("{}", fmt_fb(fb));
                    last_print = Instant::now();
                }
            }
            // A dropped feedback frame is not a reason to stop driving.
            //
            // Over CAN-FD this motor leaves roughly one reply in seven
            // unanswered (see `DamiaoMotor::read_register`), so aborting on
            // the first miss ends a one-second hold almost immediately — and
            // the command itself had already taken effect, which made it look
            // like the motor was unreachable when it was in fact moving.
            //
            // Consecutive misses are different: that is a motor that has
            // stopped talking, and continuing to command one you cannot
            // observe is how a runaway goes unnoticed. So tolerate isolated
            // drops, give up on a run of them.
            Err(damiao_driver::Error::Timeout { .. }) if missed + 1 < MAX_CONSECUTIVE_MISSES => {
                missed += 1;
                missed_total += 1;
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
    if missed_total > 0 {
        eprintln!(
            "note: {missed_total} of {ticks} feedback reads went unanswered \
             (tolerated; the motor was still commanded)"
        );
    }
    Ok(())
}

/// Every register in the official manuals' documented `0x00`-`0x25` block
/// (`damiao_protocol::Rid`), dumped in one pass and ordered by RID. A
/// register that fails to reply is left as `None` rather than aborting the
/// whole read. `boot_ver` (`0x25`) is skipped entirely on models whose
/// register map does not document it (see `MotorModel::has_boot_ver`).
///
/// Model-specific registers above `0x25` are deliberately excluded: their
/// addresses vary by register layout, so a flat dump would read the wrong
/// register on some models. Use `diag` for those.
#[derive(Debug, Serialize)]
struct AllRegs {
    motor_id: u8,
    #[serde(skip_serializing_if = "Option::is_none")]
    uv_value: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    kt_value: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    ot_value: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    oc_value: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    acc: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    dec: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    max_spd: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    mst_id: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    esc_id: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    timeout: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    ctrl_mode: Option<i32>,
    /// Identified during calibration (read-only).
    #[serde(skip_serializing_if = "Option::is_none")]
    damp: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    inertia: Option<f32>,
    /// Labeled "Reserved" in the official manuals despite the name, and a
    /// DM-J4310 does read back 0, so the label is accurate.
    #[serde(skip_serializing_if = "Option::is_none")]
    hw_ver: Option<i32>,
    /// The actual firmware version (confirmed against the official manuals —
    /// this is what the vendor's own "Read Version" tool reads, not `sub_ver`).
    /// Rendered as a string because the register holds ASCII: a DM-J4310
    /// returns `0x39313035` = `"5019"`.
    #[serde(skip_serializing_if = "Option::is_none")]
    sw_ver: Option<String>,
    /// Labeled "Reserved" in the official manuals except DM-J10422P's, which
    /// calls it a serial number. A DM-J4310 reads back 0, matching its own
    /// manual's "Reserved".
    #[serde(skip_serializing_if = "Option::is_none")]
    sn: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    npp: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    rs: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    ls: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    flux: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    gr: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pmax: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    vmax: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    tmax: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    i_bw: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    kp_asr: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    ki_asr: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    kp_apr: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    ki_apr: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    ov_value: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    gref: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    deta: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    v_bw: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    iq_c1: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    vl_c1: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    can_br: Option<i32>,
    /// ASCII like `sw_ver` — a DM-J4310 returns `0x35` = `"5"`.
    #[serde(skip_serializing_if = "Option::is_none")]
    sub_ver: Option<String>,
    /// Only on models whose map documents RID `0x25` (see
    /// `MotorModel::has_boot_ver`). Byte-packed rather than ASCII: a DM-J4310
    /// returns `0x06000203`, so this is rendered as hex.
    #[serde(skip_serializing_if = "Option::is_none")]
    boot_ver: Option<String>,
}

impl AllRegs {
    fn print(&self) {
        println!("== DAMIAO registers (motor_id={}) ==", self.motor_id);
        // Takes the option by reference so `String`-valued fields (the version
        // registers) don't need cloning at every call site.
        fn fmt<T: std::fmt::Display>(label: &str, v: &Option<T>) {
            match v {
                Some(v) => println!("  {label:<10} = {v}"),
                None => println!("  {label:<10} = <no reply>"),
            }
        }
        println!("-- protection thresholds --");
        fmt("uv_value", &self.uv_value);
        fmt("ov_value", &self.ov_value);
        fmt("ot_value", &self.ot_value);
        fmt("oc_value", &self.oc_value);
        println!("-- motion profile --");
        fmt("acc", &self.acc);
        fmt("dec", &self.dec);
        fmt("max_spd", &self.max_spd);
        println!("-- addressing / comms --");
        fmt("mst_id", &self.mst_id);
        fmt("esc_id", &self.esc_id);
        fmt("can_br", &self.can_br);
        fmt("timeout", &self.timeout);
        fmt("ctrl_mode", &self.ctrl_mode);
        println!("-- MIT mapping ranges --");
        fmt("pmax", &self.pmax);
        fmt("vmax", &self.vmax);
        fmt("tmax", &self.tmax);
        println!("-- control loop gains --");
        fmt("i_bw", &self.i_bw);
        fmt("iq_c1", &self.iq_c1);
        fmt("kp_asr", &self.kp_asr);
        fmt("ki_asr", &self.ki_asr);
        fmt("deta", &self.deta);
        fmt("v_bw", &self.v_bw);
        fmt("vl_c1", &self.vl_c1);
        fmt("kp_apr", &self.kp_apr);
        fmt("ki_apr", &self.ki_apr);
        println!("-- identified motor constants (read-only) --");
        fmt("kt_value", &self.kt_value);
        fmt("rs", &self.rs);
        fmt("ls", &self.ls);
        fmt("flux", &self.flux);
        fmt("damp", &self.damp);
        fmt("inertia", &self.inertia);
        fmt("npp", &self.npp);
        fmt("gr", &self.gr);
        fmt("gref", &self.gref);
        println!("-- identity / version (read-only) --");
        fmt("sw_ver", &self.sw_ver);
        fmt("sub_ver", &self.sub_ver);
        fmt("boot_ver", &self.boot_ver);
        fmt("hw_ver", &self.hw_ver);
        fmt("sn", &self.sn);
    }
}

fn read_all_regs<B: DamiaoBus>(motor: &mut DamiaoMotor<B>, motor_id: u8) -> AllRegs {
    let f32_reg = |motor: &mut DamiaoMotor<B>, rid: u8, label: &str| match motor.read_register(rid) {
        Ok(r) => Some(r.as_f32()),
        Err(e) => {
            eprintln!("  {label} (RID {rid}): <no reply: {e}>");
            None
        }
    };
    let i32_reg = |motor: &mut DamiaoMotor<B>, rid: u8, label: &str| match motor.read_register(rid) {
        Ok(r) => Some(r.as_i32()),
        Err(e) => {
            eprintln!("  {label} (RID {rid}): <no reply: {e}>");
            None
        }
    };
    // Version registers hold ASCII on this family (sw_ver -> "5019",
    // sub_ver -> "5"); boot_ver is byte-packed, so it falls back to hex.
    // Either way the decimal u32 is meaningless to a reader.
    let ver_reg = |motor: &mut DamiaoMotor<B>, rid: u8, label: &str| match motor.read_register(rid) {
        Ok(r) => Some(
            r.as_ascii_str()
                .map(str::to_owned)
                .unwrap_or_else(|| format!("0x{:08X}", r.as_i32())),
        ),
        Err(e) => {
            eprintln!("  {label} (RID {rid}): <no reply: {e}>");
            None
        }
    };
    // Ordered by RID so the read sequence mirrors the manuals' Register Map.
    AllRegs {
        motor_id,
        uv_value: f32_reg(motor, Rid::UV_VALUE, "uv_value"),
        kt_value: f32_reg(motor, Rid::KT_VALUE, "kt_value"),
        ot_value: f32_reg(motor, Rid::OT_VALUE, "ot_value"),
        oc_value: f32_reg(motor, Rid::OC_VALUE, "oc_value"),
        acc: f32_reg(motor, Rid::ACC, "acc"),
        dec: f32_reg(motor, Rid::DEC, "dec"),
        max_spd: f32_reg(motor, Rid::MAX_SPD, "max_spd"),
        mst_id: i32_reg(motor, Rid::MST_ID, "mst_id"),
        esc_id: i32_reg(motor, Rid::ESC_ID, "esc_id"),
        timeout: i32_reg(motor, Rid::TIMEOUT, "timeout"),
        ctrl_mode: i32_reg(motor, Rid::CTRL_MODE, "ctrl_mode"),
        damp: f32_reg(motor, Rid::DAMP, "damp"),
        inertia: f32_reg(motor, Rid::INERTIA, "inertia"),
        hw_ver: i32_reg(motor, Rid::HW_VER, "hw_ver"),
        sw_ver: ver_reg(motor, Rid::SW_VER, "sw_ver"),
        sn: i32_reg(motor, Rid::SN, "sn"),
        npp: i32_reg(motor, Rid::NPP, "npp"),
        rs: f32_reg(motor, Rid::RS, "rs"),
        ls: f32_reg(motor, Rid::LS, "ls"),
        flux: f32_reg(motor, Rid::FLUX, "flux"),
        gr: f32_reg(motor, Rid::GR, "gr"),
        pmax: f32_reg(motor, Rid::PMAX, "pmax"),
        vmax: f32_reg(motor, Rid::VMAX, "vmax"),
        tmax: f32_reg(motor, Rid::TMAX, "tmax"),
        i_bw: f32_reg(motor, Rid::I_BW, "i_bw"),
        kp_asr: f32_reg(motor, Rid::KP_ASR, "kp_asr"),
        ki_asr: f32_reg(motor, Rid::KI_ASR, "ki_asr"),
        kp_apr: f32_reg(motor, Rid::KP_APR, "kp_apr"),
        ki_apr: f32_reg(motor, Rid::KI_APR, "ki_apr"),
        ov_value: f32_reg(motor, Rid::OV_VALUE, "ov_value"),
        gref: f32_reg(motor, Rid::GREF, "gref"),
        deta: f32_reg(motor, Rid::DETA, "deta"),
        v_bw: f32_reg(motor, Rid::V_BW, "v_bw"),
        iq_c1: f32_reg(motor, Rid::IQ_C1, "iq_c1"),
        vl_c1: f32_reg(motor, Rid::VL_C1, "vl_c1"),
        can_br: i32_reg(motor, Rid::CAN_BR, "can_br"),
        sub_ver: ver_reg(motor, Rid::SUB_VER, "sub_ver"),
        // The one non-universal register in this block: models on the J3507
        // layout do not document 0x25, so skip it rather than poll an
        // undocumented address and report a spurious timeout.
        boot_ver: if motor.model().has_boot_ver() {
            ver_reg(motor, Rid::BOOT_VER, "boot_ver")
        } else {
            None
        },
    }
}

/// One model-specific register's dump result.
struct DiagEntry {
    reg: ModelReg,
    /// `None` when this model's register map does not document `reg` — the
    /// register is then never read.
    rid: Option<u8>,
    /// `None` when unavailable on this model, or when the read got no reply.
    value: Option<f32>,
}

/// The model-specific registers above `0x25`, read via `ModelReg` so each RID
/// is resolved against the configured model rather than assumed.
struct ModelRegs {
    motor_id: u8,
    model: &'static str,
    /// In [`ModelReg::ALL`] order.
    entries: Vec<DiagEntry>,
}

/// Serializable view of [`ModelRegs`]. Scalars and arrays are declared before
/// `values` because TOML requires plain values to precede tables.
#[derive(Debug, Serialize)]
struct ModelRegsToml {
    motor_id: u8,
    model: &'static str,
    /// Registers absent from this model's register map (never polled).
    #[serde(skip_serializing_if = "Vec::is_empty")]
    not_on_this_model: Vec<&'static str>,
    /// Registers this model documents but which did not reply.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    no_reply: Vec<&'static str>,
    values: std::collections::BTreeMap<&'static str, f32>,
}

impl ModelRegs {
    fn to_toml(&self) -> ModelRegsToml {
        ModelRegsToml {
            motor_id: self.motor_id,
            model: self.model,
            not_on_this_model: self
                .entries
                .iter()
                .filter(|e| e.rid.is_none())
                .map(|e| e.reg.name())
                .collect(),
            no_reply: self
                .entries
                .iter()
                .filter(|e| e.rid.is_some() && e.value.is_none())
                .map(|e| e.reg.name())
                .collect(),
            values: self
                .entries
                .iter()
                .filter_map(|e| e.value.map(|v| (e.reg.name(), v)))
                .collect(),
        }
    }

    fn print(&self) {
        println!(
            "== DAMIAO model-specific registers (motor_id={}, model={}) ==",
            self.motor_id, self.model
        );
        for e in &self.entries {
            let name = e.reg.name();
            match (e.rid, e.value) {
                (None, _) => println!("  {name:<8} = <not on this model>  ({})", e.reg.description()),
                (Some(rid), Some(v)) => {
                    println!("  {name:<8} = {v}  (RID {rid:#04X}, {})", e.reg.description())
                }
                (Some(rid), None) => {
                    println!("  {name:<8} = <no reply>  (RID {rid:#04X}, {})", e.reg.description())
                }
            }
        }
    }
}

fn read_model_regs<B: DamiaoBus>(motor: &mut DamiaoMotor<B>, motor_id: u8) -> ModelRegs {
    let model = motor.model();
    let entries = ModelReg::ALL
        .iter()
        .map(|&reg| {
            let rid = reg.rid(model);
            // Only registers this model documents are polled at all; the rest
            // are reported as absent without touching the bus.
            let value = rid.and_then(|rid| match motor.read_model_register(reg) {
                Ok(r) => Some(r.as_f32()),
                Err(e) => {
                    eprintln!("  {} (RID {rid:#04X}): <no reply: {e}>", reg.name());
                    None
                }
            });
            DiagEntry { reg, rid, value }
        })
        .collect();
    ModelRegs {
        motor_id,
        model: model.name(),
        entries,
    }
}

fn fmt_fb(fb: Feedback) -> String {
    format!(
        "id={} pos={:+.3} rad  vel={:+.3} rad/s  tau={:+.3} Nm  T_mos={:.0}°C  T_rotor={:.0}°C  err={:?}",
        fb.motor_id, fb.position, fb.velocity, fb.torque, fb.t_mos, fb.t_rotor, fb.err
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build the dump shape a real read would produce, without a bus.
    fn diag_for(model: MotorModel) -> ModelRegs {
        let entries = ModelReg::ALL
            .iter()
            .map(|&reg| {
                let rid = reg.rid(model);
                DiagEntry {
                    reg,
                    rid,
                    value: rid.map(|_| 1.5),
                }
            })
            .collect();
        ModelRegs {
            motor_id: 1,
            model: model.name(),
            entries,
        }
    }

    /// TOML requires plain values before tables; `values` must therefore be the
    /// last field. Serializing is the only way to catch a field reorder.
    #[test]
    fn diag_toml_serializes_for_both_models() {
        for &model in MotorModel::ALL {
            let text = toml::to_string_pretty(&diag_for(model).to_toml())
                .unwrap_or_else(|e| panic!("{} dump failed to serialize: {e}", model.name()));
            assert!(text.contains("motor_id = 1"));
            assert!(text.contains(&format!("model = \"{}\"", model.name())));
        }
    }

    /// Every model must be reachable through `--model`, and each must land on
    /// a dump that has at least one readable register.
    #[test]
    fn every_model_is_selectable_and_dumps_something() {
        for &model in MotorModel::ALL {
            assert_eq!(
                MotorModel::from_name(model.name()),
                Some(model),
                "--model {} does not parse",
                model.name()
            );
            let d = diag_for(model).to_toml();
            assert!(
                !d.values.is_empty(),
                "{} exposes no model-specific registers",
                model.name()
            );
        }
    }

    /// DM-J10422P is the only model on its own layout, with two registers no
    /// other model has and none of the J3507-only ones.
    #[test]
    fn dm10422p_exposes_its_unique_registers() {
        let d = diag_for(MotorModel::Dm10422P).to_toml();
        assert!(d.values.contains_key("x_off"), "x_off is DM10422P-only");
        assert!(d.values.contains_key("IBase"), "IBase is DM10422P-only");
        // `Imax` is a different quantity at the same address on J4310.
        assert!(d.not_on_this_model.contains(&"Imax"));
        assert!(d.not_on_this_model.contains(&"k1"));
        // No other model exposes x_off / IBase.
        for &model in MotorModel::ALL {
            if model == MotorModel::Dm10422P {
                continue;
            }
            let other = diag_for(model).to_toml();
            assert!(!other.values.contains_key("x_off"), "{}", model.name());
            assert!(!other.values.contains_key("IBase"), "{}", model.name());
        }
    }

    /// Registers absent from a model are reported, never read, and never land
    /// in `values` — the whole point of routing through `ModelReg`.
    #[test]
    fn diag_partitions_registers_by_model() {
        let dm4310 = diag_for(MotorModel::Dm4310).to_toml();
        assert!(dm4310.values.contains_key("VBus"), "DM4310 exposes VBus");
        assert!(!dm4310.values.contains_key("k1"), "k1 is DM3507-only");
        assert!(dm4310.not_on_this_model.contains(&"k1"));

        let dm3507 = diag_for(MotorModel::Dm3507).to_toml();
        assert!(dm3507.values.contains_key("k1"), "DM3507 exposes k1");
        assert!(!dm3507.values.contains_key("VBus"), "VBus is DM4310-only");
        assert!(dm3507.not_on_this_model.contains(&"VBus"));

        // m_off exists on both, so it must never show up as unavailable.
        for d in [&dm4310, &dm3507] {
            assert!(d.values.contains_key("m_off"));
            assert!(!d.not_on_this_model.contains(&"m_off"));
        }
    }

    /// The hazard table is hand-written against the manuals' register map, and
    /// a register in the wrong bucket fails silently — the write still goes
    /// out, just without its warning. Pin the classes that matter.
    #[test]
    fn dangerous_registers_are_classified() {
        for rid in [Rid::MST_ID, Rid::ESC_ID, Rid::TIMEOUT, Rid::CAN_BR] {
            assert_eq!(register_risk(rid), RegisterRisk::Communication, "RID {rid}");
        }
        for rid in [Rid::PMAX, Rid::VMAX, Rid::TMAX] {
            assert_eq!(register_risk(rid), RegisterRisk::Scaling, "RID {rid}");
        }
        for rid in [Rid::UV_VALUE, Rid::OV_VALUE, Rid::OT_VALUE, Rid::OC_VALUE] {
            assert_eq!(register_risk(rid), RegisterRisk::Protection, "RID {rid}");
        }
        // Every register the manuals mark read-only, including the calibration
        // constants the motor identified for itself.
        for rid in [
            Rid::DAMP,
            Rid::INERTIA,
            Rid::HW_VER,
            Rid::SW_VER,
            Rid::SN,
            Rid::NPP,
            Rid::RS,
            Rid::LS,
            Rid::FLUX,
            Rid::GR,
            Rid::SUB_VER,
            Rid::BOOT_VER,
        ] {
            assert_eq!(register_risk(rid), RegisterRisk::ReadOnly, "RID {rid}");
        }
    }

    /// Ordinary tuning registers must stay quiet, or the warning becomes noise
    /// that gets ignored on the writes that matter.
    #[test]
    fn ordinary_registers_warn_about_nothing() {
        for rid in [
            Rid::ACC,
            Rid::DEC,
            Rid::MAX_SPD,
            Rid::CTRL_MODE,
            Rid::KP_ASR,
            Rid::KI_ASR,
            Rid::KP_APR,
            Rid::KI_APR,
            Rid::I_BW,
            Rid::GREF,
            Rid::DETA,
            Rid::V_BW,
            Rid::IQ_C1,
            Rid::VL_C1,
            Rid::KT_VALUE,
        ] {
            assert_eq!(register_risk(rid), RegisterRisk::Ordinary, "RID {rid}");
            assert!(register_risk(rid).hazard().is_none(), "RID {rid}");
        }
    }

    /// Anything classified as risky must actually have something to say.
    #[test]
    fn every_risky_class_has_a_hazard_message() {
        for rid in 0..=Rid::BOOT_VER {
            let risk = register_risk(rid);
            assert_eq!(
                risk.hazard().is_some(),
                risk != RegisterRisk::Ordinary,
                "RID {rid} classified {risk:?} but its message disagrees"
            );
        }
    }
}
