//! Driver factory — builds a `Box<dyn Actuator>` from CLI args.
//!
//! The TUI is driver-agnostic and only speaks the unified
//! [`misa_actuator::Actuator`] trait. This module bridges the CLI's
//! `--driver / --interface / ...` flags to the concrete driver type.

use std::time::Duration;

use anyhow::{Context, Result, bail};
use clap::ValueEnum;
use misa_actuator::Actuator;

use damiao_driver::{DamiaoMotor, MotorModel as DmModel};
use lkmotor_driver::{LkMotor, MotorConfig as LkMotorConfig, MotorId as LkMotorId};
use robstride_driver::{Motor as RsMotor, MotorModel};

/// The shared `--model` default. Robstride-centric for historical reasons; the
/// DAMIAO driver treats this sentinel as "use the DAMIAO default model".
const DEFAULT_ROBSTRIDE_MODEL: &str = "Edulite05";

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum DriverKind {
    /// Robstride CAN motor on a SocketCAN interface (Linux).
    Robstride,
    /// LK Motor (RMD) on an RS485 serial port.
    Lkmotor,
    /// DAMIAO CAN / CAN-FD motor on a SocketCAN interface (Linux).
    Damiao,
}

/// Physical CAN layer for the DAMIAO driver. Classic CAN and CAN-FD carry the
/// identical DAMIAO payload; this only selects the socket type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum BusKind {
    /// Classic CAN (1 Mbps).
    Can,
    /// CAN-FD (1–5 Mbps); the interface must be `fd on`.
    CanFd,
}

#[derive(Debug, Clone)]
pub struct DriverConfig {
    pub kind: DriverKind,
    /// SocketCAN interface (robstride) or serial-port path (lkmotor).
    pub interface: String,
    /// Motor address on the bus (1..=127).
    pub motor_id: u8,
    /// Robstride: motor model name (`RS-05`, `Edulite05`, ...). Ignored for lkmotor.
    pub model: String,
    /// Robstride: host CAN ID. Ignored for lkmotor.
    pub host_id: u8,
    /// Lkmotor: serial baud rate. Ignored for robstride.
    pub baud: u32,
    /// Lkmotor: gear ratio (e.g. 10.0 for 1:10 gearbox). Ignored for robstride.
    pub gear_ratio: f32,
    /// Lkmotor: torque constant Kt (N·m/A). 0 → use `MotorConfig::current_units`.
    pub kt: f32,
    /// Damiao: physical CAN layer (classic CAN or CAN-FD). Ignored otherwise.
    pub bus_kind: BusKind,
    /// Per-request timeout.
    pub timeout: Duration,
}

impl Default for DriverConfig {
    fn default() -> Self {
        Self {
            kind: DriverKind::Robstride,
            interface: "can0".to_string(),
            motor_id: 1,
            model: DEFAULT_ROBSTRIDE_MODEL.to_string(),
            host_id: robstride_driver::DEFAULT_HOST_ID,
            baud: 1_000_000,
            gear_ratio: 10.0,
            kt: 0.0,
            bus_kind: BusKind::Can,
            timeout: Duration::from_millis(100),
        }
    }
}

pub fn build_actuator(cfg: &DriverConfig) -> Result<Box<dyn Actuator + Send>> {
    match cfg.kind {
        DriverKind::Robstride => {
            let model = MotorModel::from_name(&cfg.model)
                .with_context(|| format!("unknown Robstride motor model: {}", cfg.model))?;
            let mut motor =
                RsMotor::open_with_host(&cfg.interface, cfg.motor_id, cfg.host_id, model)
                    .with_context(|| {
                        format!(
                            "failed to open SocketCAN interface {} for motor {}",
                            cfg.interface, cfg.motor_id
                        )
                    })?;
            motor
                .set_timeout(cfg.timeout)
                .context("failed to set CAN socket timeout")?;
            Ok(Box::new(motor))
        }
        DriverKind::Lkmotor => {
            let id = LkMotorId::new(cfg.motor_id)
                .with_context(|| format!("invalid lkmotor id {} (must be 1..=32)", cfg.motor_id))?;
            let motor_config = if cfg.kt > 0.0 {
                LkMotorConfig::new(cfg.gear_ratio, cfg.kt)
            } else {
                LkMotorConfig::current_units(cfg.gear_ratio)
            };
            let motor =
                LkMotor::open_rs485(&cfg.interface, cfg.baud, id, motor_config, cfg.timeout)
                    .with_context(|| {
                        format!(
                            "failed to open serial port {} @ {} baud for motor {}",
                            cfg.interface, cfg.baud, cfg.motor_id
                        )
                    })?;
            Ok(Box::new(motor))
        }
        DriverKind::Damiao => {
            // The shared `--model` default is Robstride-centric; on the DAMIAO
            // driver, treat the untouched default as "use the DAMIAO default"
            // rather than erroring. A genuinely wrong override still errors.
            let model = match DmModel::from_name(&cfg.model) {
                Some(m) => m,
                None if cfg.model == DEFAULT_ROBSTRIDE_MODEL => DmModel::Dm4310,
                None => bail!(
                    "unknown DAMIAO motor model: {} (try --model DM4310)",
                    cfg.model
                ),
            };
            // Classic CAN and CAN-FD share the DAMIAO protocol; only the socket
            // type differs. Both yield a `DamiaoMotor<B>` that implements
            // `Actuator`, so box whichever the user selected.
            let motor: Box<dyn Actuator + Send> = match cfg.bus_kind {
                BusKind::Can => {
                    let mut m = DamiaoMotor::open(&cfg.interface, cfg.motor_id, model)
                        .with_context(|| {
                            format!(
                                "failed to open classic-CAN interface {} for motor {}",
                                cfg.interface, cfg.motor_id
                            )
                        })?;
                    m.set_timeout(cfg.timeout)
                        .context("failed to set CAN socket timeout")?;
                    Box::new(m)
                }
                BusKind::CanFd => {
                    let mut m = DamiaoMotor::open_fd(&cfg.interface, cfg.motor_id, model)
                        .with_context(|| {
                            format!(
                                "failed to open CAN-FD interface {} for motor {} (is it `fd on`?)",
                                cfg.interface, cfg.motor_id
                            )
                        })?;
                    m.set_timeout(cfg.timeout)
                        .context("failed to set CAN-FD socket timeout")?;
                    Box::new(m)
                }
            };
            Ok(motor)
        }
    }
}

pub fn validate_driver_args(cfg: &DriverConfig) -> Result<()> {
    if cfg.interface.is_empty() {
        bail!("--interface is required");
    }
    if cfg.motor_id == 0 {
        bail!("--motor-id must be > 0");
    }
    Ok(())
}
