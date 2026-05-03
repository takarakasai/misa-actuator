//! Command catalog and parameter dialogs for the TUI.

use std::time::Duration;

use misa_actuator::{Actuator, RunMode};

/// Every command the debug TUI can dispatch through the [`Actuator`] trait.
///
/// The four `SetMode*` variants are split out (instead of one `SetRunMode`
/// with a dropdown param) so the user can switch modes with a single
/// Enter — no editing required.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Command {
    Scan,
    Enable,
    Disable,
    SetZero,
    SetModeMit,
    SetModePosition,
    SetModeVelocity,
    SetModeTorque,
    Measure,
    ReadStatus,
    SetPosition,
    SetVelocity,
    SetTorque,
    MitControl,
}

impl Command {
    pub const ALL: &'static [Command] = &[
        Command::Scan,
        Command::Enable,
        Command::Disable,
        Command::SetZero,
        Command::SetModeMit,
        Command::SetModePosition,
        Command::SetModeVelocity,
        Command::SetModeTorque,
        Command::Measure,
        Command::ReadStatus,
        Command::SetPosition,
        Command::SetVelocity,
        Command::SetTorque,
        Command::MitControl,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Command::Scan            => "Scan Bus",
            Command::Enable          => "Enable",
            Command::Disable         => "Disable",
            Command::SetZero         => "Set Zero (anchor position)",
            Command::SetModeMit      => "Set Mode → MIT",
            Command::SetModePosition => "Set Mode → Position",
            Command::SetModeVelocity => "Set Mode → Velocity",
            Command::SetModeTorque   => "Set Mode → Torque",
            Command::Measure         => "Measure (poll feedback)",
            Command::ReadStatus      => "Read Status (Vbus / temp / errors)",
            Command::SetPosition     => "Set Position",
            Command::SetVelocity     => "Set Velocity",
            Command::SetTorque       => "Set Torque",
            Command::MitControl      => "MIT Control",
        }
    }
}

/// One editable parameter shown in the params pane for the selected command.
#[derive(Debug, Clone)]
pub struct ParamField {
    pub name: &'static str,
    pub value: String,
    pub desc: &'static str,
    pub choices: Option<&'static str>,
}

impl ParamField {
    fn new(name: &'static str, default: &'static str, desc: &'static str) -> Self {
        Self {
            name,
            value: default.to_string(),
            desc,
            choices: None,
        }
    }
    #[allow(dead_code)]
    fn with_choices(
        name: &'static str,
        default: &'static str,
        desc: &'static str,
        choices: &'static str,
    ) -> Self {
        Self {
            name,
            value: default.to_string(),
            desc,
            choices: Some(choices),
        }
    }
}

pub fn params_for(cmd: Command) -> Vec<ParamField> {
    match cmd {
        Command::Scan => vec![
            ParamField::new("from", "1", "Start motor ID"),
            ParamField::new("to", "32", "End motor ID (lkmotor max=32, robstride max=127)"),
            ParamField::new("timeout_ms", "50", "Per-ID timeout [ms]"),
        ],
        Command::SetPosition => vec![
            ParamField::new("pos", "0.0", "Target position [rad]"),
            ParamField::new("max_speed", "5.0", "Max speed [rad/s]"),
        ],
        Command::SetVelocity => vec![ParamField::new("vel", "1.0", "Target velocity [rad/s]")],
        Command::SetTorque => vec![ParamField::new("torque", "0.1", "Target torque [Nm]")],
        Command::MitControl => vec![
            ParamField::new("pos", "0.0", "Position ref [rad]"),
            ParamField::new("vel", "0.0", "Velocity ref [rad/s]"),
            ParamField::new("kp", "10.0", "Stiffness [Nm/rad]"),
            ParamField::new("kd", "0.5", "Damping [Nm·s/rad]"),
            ParamField::new("torque_ff", "0.0", "Torque feed-forward [Nm]"),
        ],
        // No params: Enable, Disable, SetZero, SetMode*, Measure, ReadStatus
        _ => vec![],
    }
}

fn pf(params: &[ParamField], name: &str) -> Option<f32> {
    params.iter().find(|p| p.name == name)?.value.parse::<f32>().ok()
}

fn pf_u8(params: &[ParamField], name: &str, default: u8) -> u8 {
    params
        .iter()
        .find(|p| p.name == name)
        .and_then(|p| p.value.parse::<u8>().ok())
        .unwrap_or(default)
}

fn pf_u64(params: &[ParamField], name: &str, default: u64) -> u64 {
    params
        .iter()
        .find(|p| p.name == name)
        .and_then(|p| p.value.parse::<u64>().ok())
        .unwrap_or(default)
}

/// Outcome of dispatching a command through the actuator.
pub struct Dispatch {
    pub log_line: String,
    pub feedback: Option<misa_actuator::MotorFeedback>,
    /// For commands that produce extra log lines (e.g. per-id scan results),
    /// these get appended after `log_line`.
    pub extra_log_lines: Vec<String>,
}

pub fn dispatch(
    cmd: Command,
    params: &[ParamField],
    actuator: &mut dyn Actuator,
) -> Dispatch {
    let mut extra: Vec<String> = Vec::new();
    let result = match cmd {
        Command::Scan => {
            let from = pf_u8(params, "from", 1);
            let to = pf_u8(params, "to", 32);
            let timeout_ms = pf_u64(params, "timeout_ms", 50);
            let lo = from.max(1);
            let hi = to.max(lo);
            match actuator.scan_bus(lo..=hi, Duration::from_millis(timeout_ms)) {
                Ok(ids) => {
                    if ids.is_empty() {
                        extra.push(format!("  no motors found in {}..={}", lo, hi));
                    } else {
                        extra.push(format!("  found {} motor(s):", ids.len()));
                        for id in &ids {
                            extra.push(format!("    id={}", id));
                        }
                    }
                    Ok(None)
                }
                Err(e) => Err(e),
            }
        }
        Command::Enable => actuator.enable().map(Some),
        Command::Disable => actuator.disable().map(|_| None),
        Command::SetZero => actuator.set_zero().map(|_| None),
        Command::SetModeMit      => actuator.set_run_mode(RunMode::Mit).map(|_| None),
        Command::SetModePosition => actuator.set_run_mode(RunMode::Position).map(|_| None),
        Command::SetModeVelocity => actuator.set_run_mode(RunMode::Velocity).map(|_| None),
        Command::SetModeTorque   => actuator.set_run_mode(RunMode::Torque).map(|_| None),
        Command::Measure => actuator.measure().map(Some),
        Command::ReadStatus => actuator.read_status().map(|s| {
            log::info!("status: {:?}", s);
            None
        }),
        Command::SetPosition => {
            let pos = pf(params, "pos").unwrap_or(0.0);
            let max_speed = pf(params, "max_speed").unwrap_or(5.0);
            actuator.set_position(pos, max_speed).map(Some)
        }
        Command::SetVelocity => {
            let vel = pf(params, "vel").unwrap_or(0.0);
            actuator.set_velocity(vel).map(Some)
        }
        Command::SetTorque => {
            let torque = pf(params, "torque").unwrap_or(0.0);
            actuator.set_torque(torque).map(Some)
        }
        Command::MitControl => {
            let pos = pf(params, "pos").unwrap_or(0.0);
            let vel = pf(params, "vel").unwrap_or(0.0);
            let kp = pf(params, "kp").unwrap_or(0.0);
            let kd = pf(params, "kd").unwrap_or(0.0);
            let tau_ff = pf(params, "torque_ff").unwrap_or(0.0);
            actuator.mit_control(pos, vel, kp, kd, tau_ff).map(Some)
        }
    };

    match result {
        Ok(fb) => {
            let line = match fb {
                Some(f) => format!(
                    "{} OK  pos={:+.3} rad  vel={:+.3} rad/s  τ={:+.3} Nm  T={:.1}°C",
                    cmd.label(),
                    f.position_rad,
                    f.velocity_rad_per_s,
                    f.torque_nm,
                    f.temperature_c
                ),
                None => format!("{} OK", cmd.label()),
            };
            Dispatch {
                log_line: line,
                feedback: fb,
                extra_log_lines: extra,
            }
        }
        Err(e) => Dispatch {
            log_line: format!("{} FAILED: {}", cmd.label(), e),
            feedback: None,
            extra_log_lines: extra,
        },
    }
}
