//! Driver factory — builds a `Box<dyn Actuator>` from a driver selection.
//!
//! The TUI and the GUI are both driver-agnostic and speak only the unified
//! [`misa_actuator::Actuator`] trait. This module bridges a
//! `--driver / --interface / ...` selection to the concrete driver type.

use std::time::Duration;

use anyhow::{bail, Context, Result};
use misa_actuator::Actuator;
use serde::{Deserialize, Serialize};

use damiao_driver::{DamiaoMotor, MotorModel as DmModel};
use lkmotor_driver::{LkMotor, MotorConfig as LkMotorConfig, MotorId as LkMotorId};
use misa_actuator_sim::{SimActuator, SimConfig};
use myactuator_driver::{MotorConfig as MyaMotorConfig, MyActuatorMotor};
use robstride_driver::{Motor as RsMotor, MotorModel};

/// The shared `--model` value meaning "the caller did not choose one".
///
/// Empty rather than a real model name. It used to be `"Edulite05"`, which
/// made an unspecified model indistinguishable from a deliberate EduLite05 —
/// so a RobStride user who never passed `--model` silently got RS-05 scaling,
/// and on an RS-04 that is a 21.8-fold torque error in both directions
/// (commands *and* decoded feedback) with nothing on screen to show it.
/// A sentinel has to be a value no user can legitimately mean.
///
/// Drivers with a safe fallback (DAMIAO, the simulator) still substitute their
/// own default; RobStride refuses, because it has no safe guess to make.
pub const MODEL_UNSPECIFIED: &str = "";

/// Extra motor ids a simulated bus answers on, beyond whatever the preset and
/// the bound id already put there.
///
/// A one-motor bus makes scanning pointless to look at: the progress dialog,
/// the "found N motors" list and the motor-selection pane all have nothing to
/// exercise. Spreading a few ids out also leaves gaps, so a scan spends most
/// of its time timing out — which is exactly the behaviour the UI has to stay
/// responsive through.
pub const DEFAULT_SIM_BUS_IDS: &[u8] = &[1, 2, 5, 11];

/// The id set a simulated bus should answer on: the preset's own, the id the
/// caller bound to, and [`DEFAULT_SIM_BUS_IDS`].
///
/// Including the bound id matters — without it, a scan would report the very
/// motor you are talking to as absent.
fn simulated_bus_ids(preset_ids: &[u8], bound_id: u8) -> Vec<u8> {
    let mut ids: Vec<u8> = preset_ids
        .iter()
        .copied()
        .chain(DEFAULT_SIM_BUS_IDS.iter().copied())
        .chain(std::iter::once(bound_id))
        .filter(|&id| id != 0)
        .collect();
    ids.sort_unstable();
    ids.dedup();
    ids
}

// Serialize as well as Deserialize: a multi-motor reading names the family it
// came from, so a mixed wire can be labelled on screen. The kebab-case rename
// makes that match `as_str` below.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "clap", derive(clap::ValueEnum))]
#[serde(rename_all = "kebab-case")]
pub enum DriverKind {
    /// Robstride CAN motor on any CAN interface (SocketCAN / PCAN / SLCAN).
    Robstride,
    /// LK Motor (RMD) on an RS485 serial port.
    Lkmotor,
    /// DAMIAO CAN / CAN-FD motor on any CAN interface.
    Damiao,
    /// MyActuator RMD motor (CAN protocol V3) on any CAN interface.
    Myactuator,
    /// Simulated motor — no hardware, no interface. `--model` picks the
    /// preset (`ideal`, `dm4310`, `rs04`); the last two are calibrated from
    /// `doc/bench-measurements-2026-07-30.md`.
    Sim,
}

/// Physical CAN layer for the DAMIAO driver. Classic CAN and CAN-FD carry the
/// identical DAMIAO payload; this only selects the socket type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[cfg_attr(feature = "clap", derive(clap::ValueEnum))]
#[serde(rename_all = "kebab-case")]
pub enum BusKind {
    /// Classic CAN (1 Mbps).
    Can,
    /// CAN-FD (1–5 Mbps); the interface must be `fd on`.
    CanFd,
}

#[derive(Debug, Clone)]
pub struct DriverConfig {
    pub kind: DriverKind,
    /// CAN interface (`can0`, `pcan:usb1`, `slcan:COM5`) for the CAN drivers,
    /// or a serial-port name (`/dev/ttyUSB0`, `COM5`) for lkmotor.
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
    /// Torque constant Kt (N·m/A). 0 means "use whatever the motor reports".
    ///
    /// Lkmotor / Myactuator: 0 selects current-units mode. **RobStride: a
    /// non-zero value makes the driver synthesize torque from current**, which is
    /// the only way to measure anything torque-based on firmware that reports
    /// `MeasuredTorque` as a constant zero. Measure it with
    /// `robstride-cli characterize kt` (an RS-04 measured 1.5093) or take it from
    /// the datasheet; a real reading is always preferred, so this changes nothing
    /// on healthy firmware.
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
            interface: misa_can::default_interface().to_string(),
            motor_id: 1,
            model: MODEL_UNSPECIFIED.to_string(),
            host_id: robstride_driver::DEFAULT_HOST_ID,
            baud: 1_000_000,
            gear_ratio: 10.0,
            kt: 0.0,
            bus_kind: BusKind::Can,
            timeout: Duration::from_millis(100),
        }
    }
}

/// Model names a driver family accepts, or empty for families that ignore the
/// model field.
///
/// Exists so a UI can offer a list instead of a text box. That is not a
/// convenience: on RobStride and DAMIAO the model selects the MIT
/// quantisation range, and the ranges differ enormously — an RS-04 quantises
/// torque over ±120 N·m where an RS-05 uses ±5.5. Connect an RS-04 as an
/// RS-05 and a 1 N·m command leaves as 21.8 N·m, with nothing to indicate it.
/// A free-text field that silently falls back to a default is the wrong shape
/// for that decision.
pub fn known_models(kind: DriverKind) -> Vec<&'static str> {
    match kind {
        // The catalogue rather than `ALL`: it includes the EduLite rebadges,
        // which are the name printed on the motor for anyone who has one.
        DriverKind::Robstride => MotorModel::CATALOGUE.iter().map(|(name, _)| *name).collect(),
        DriverKind::Damiao => DmModel::ALL.iter().map(|m| m.name()).collect(),
        DriverKind::Sim => SimConfig::PRESETS.to_vec(),
        // LK Motor is configured by gear ratio and Kt, MyActuator by Kt; both
        // ignore the model field entirely.
        DriverKind::Lkmotor | DriverKind::Myactuator => Vec::new(),
    }
}

/// Every driver family, for building a selector.
pub const ALL_DRIVERS: &[DriverKind] = &[
    DriverKind::Sim,
    DriverKind::Robstride,
    DriverKind::Damiao,
    DriverKind::Myactuator,
    DriverKind::Lkmotor,
];

impl DriverKind {
    /// The lower-case name used on the wire and in `--driver`.
    pub fn as_str(self) -> &'static str {
        match self {
            DriverKind::Robstride => "robstride",
            DriverKind::Lkmotor => "lkmotor",
            DriverKind::Damiao => "damiao",
            DriverKind::Myactuator => "myactuator",
            DriverKind::Sim => "sim",
        }
    }
}

/// What the motor said about itself when the connection opened.
///
/// The model field is not cosmetic — on RobStride and DAMIAO it fixes the MIT
/// quantisation ranges, and picking the wrong one mis-scales every torque
/// command by up to twenty-fold with nothing in the data to show it. Whatever
/// the bus can tell us about that belongs in front of the operator at connect
/// time, not in a log nobody reads.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct IdentityReport {
    /// What was read, phrased for a status line. Empty when nothing was.
    pub observed: String,
    /// Set when the readings contradict the selected model.
    pub warning: Option<String>,
}

impl IdentityReport {
    pub fn is_empty(&self) -> bool {
        self.observed.is_empty() && self.warning.is_none()
    }
}

pub fn build_actuator(cfg: &DriverConfig) -> Result<Box<dyn Actuator + Send>> {
    build_actuator_checked(cfg).map(|(actuator, _)| actuator)
}

/// Open the motor and, where the protocol allows it, ask the motor to confirm
/// the model that was selected for it.
///
/// Separate from [`build_actuator`] only because the interrogation has to
/// happen against the concrete driver type, before it is boxed into
/// `dyn Actuator`.
pub fn build_actuator_checked(
    cfg: &DriverConfig,
) -> Result<(Box<dyn Actuator + Send>, IdentityReport)> {
    match cfg.kind {
        DriverKind::Robstride => {
            // No fallback here on purpose. Every other driver either ignores
            // the model or has one safe default; RobStride's MIT scales span
            // ±5.5 to ±120 N·m across the range, so guessing is how you send
            // twenty times the torque you meant to.
            if cfg.model.trim().is_empty() {
                bail!(
                    "--model is required for RobStride: it sets the MIT quantisation \
                     range, and the range differs 21-fold across the family (RS-05 \
                     is ±5.5 N·m, RS-04 ±120). Known: {}",
                    known_models(DriverKind::Robstride).join(", ")
                );
            }
            let model = MotorModel::from_name(&cfg.model).with_context(|| {
                format!(
                    "unknown Robstride motor model: {} (known: {})",
                    cfg.model,
                    known_models(DriverKind::Robstride).join(", ")
                )
            })?;
            let mut motor =
                RsMotor::open_with_host(&cfg.interface, cfg.motor_id, cfg.host_id, model)
                    .with_context(|| {
                        format!(
                            "failed to open CAN interface {} for motor {}",
                            cfg.interface, cfg.motor_id
                        )
                    })?;
            motor
                .set_timeout(cfg.timeout)
                .context("failed to set CAN socket timeout")?;
            // Some firmware returns a constant 0 for `MeasuredTorque`, which
            // leaves every torque-based measurement with nothing to measure —
            // confirmed again on 2026-08-06 on an RS-03, where a velocity-mode
            // spin reported 0.000 N·m throughout while the shaft turned at the
            // commanded speed. `IqFilt` works there, so `torque = current · Kt`
            // recovers it.
            //
            // Deriving torque needs a current reading, so a Kt implies asking for
            // one — the same rule `robstride-cli --kt` follows, rather than
            // accepting a Kt and silently doing nothing with it.
            if cfg.kt > 0.0 {
                motor.set_report_current(true);
                motor.set_torque_constant(cfg.kt);
                if motor.torque_constant().is_none() {
                    bail!("kt must be finite and positive, got {}", cfg.kt);
                }
            }
            let report = robstride_identity(&mut motor, model, &cfg.model);
            Ok((Box::new(motor), report))
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
            Ok((Box::new(motor), IdentityReport::default()))
        }
        DriverKind::Damiao => {
            // The shared `--model` default is Robstride-centric; on the DAMIAO
            // driver, treat the untouched default as "use the DAMIAO default"
            // rather than erroring. A genuinely wrong override still errors.
            let model = match DmModel::from_name(&cfg.model) {
                Some(m) => m,
                None if cfg.model == MODEL_UNSPECIFIED => DmModel::Dm4310,
                None => {
                    let known: Vec<&str> = DmModel::ALL.iter().map(|m| m.name()).collect();
                    bail!(
                        "unknown DAMIAO motor model: {} (known: {})",
                        cfg.model,
                        known.join(", ")
                    )
                }
            };
            // Classic CAN and CAN-FD share the DAMIAO protocol; only the socket
            // type differs. Both yield a `DamiaoMotor<B>` that implements
            // `Actuator`, so box whichever the user selected.
            let (motor, report): (Box<dyn Actuator + Send>, _) = match cfg.bus_kind {
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
                    let report = damiao_identity(&mut m, model);
                    (Box::new(m), report)
                }
                BusKind::CanFd => {
                    let mut m = DamiaoMotor::open_fd(&cfg.interface, cfg.motor_id, model)
                        .with_context(|| {
                            format!(
                                "failed to open CAN-FD interface {} for motor {} \
                                 (SocketCAN needs `fd on`; SLCAN cannot do FD)",
                                cfg.interface, cfg.motor_id
                            )
                        })?;
                    m.set_timeout(cfg.timeout)
                        .context("failed to set CAN-FD socket timeout")?;
                    let report = damiao_identity(&mut m, model);
                    (Box::new(m), report)
                }
            };
            Ok((motor, report))
        }
        DriverKind::Myactuator => {
            // V3 speaks output-shaft units on the wire, so only Kt matters
            // (reuses the lkmotor `--kt` flag; 0 → current-units mode).
            let config = if cfg.kt > 0.0 {
                MyaMotorConfig::new(cfg.kt)
            } else {
                MyaMotorConfig::current_units()
            };
            let mut motor = MyActuatorMotor::open(&cfg.interface, cfg.motor_id, config)
                .with_context(|| {
                    format!(
                        "failed to open CAN interface {} for motor {}",
                        cfg.interface, cfg.motor_id
                    )
                })?;
            motor
                .set_timeout(cfg.timeout)
                .context("failed to set CAN socket timeout")?;
            Ok((Box::new(motor), IdentityReport::default()))
        }
        DriverKind::Sim => {
            // Same sentinel handling as DAMIAO: the shared `--model` default
            // is Robstride-centric, so an untouched one means "the default
            // preset" rather than an error.
            let mut preset = match SimConfig::from_name(&cfg.model) {
                Some(p) => p,
                None if cfg.model == MODEL_UNSPECIFIED => SimConfig::ideal(),
                None => bail!(
                    "unknown sim preset: {} (known: {})",
                    cfg.model,
                    SimConfig::PRESETS.join(", ")
                ),
            };
            preset.present_ids = simulated_bus_ids(&preset.present_ids, cfg.motor_id);
            log::info!(
                "sim: {} bound to id {} on a bus of {:?}",
                preset.name,
                cfg.motor_id,
                preset.present_ids
            );
            Ok((
                Box::new(SimActuator::new(preset).with_motor_id(cfg.motor_id)),
                IdentityReport::default(),
            ))
        }
    }
}

/// What to report when the firmware version named the model, given what the
/// second channel said about it.
///
/// Pure, so the disagreement logic can be tested without a bus — RobStride has
/// no mock transport, and this is the branch where being wrong is quietest.
fn robstride_named_identity(
    model: MotorModel,
    line: robstride_driver::ProductLine,
    selected: MotorModel,
    label: &str,
    version_text: &str,
    seen: robstride_driver::ReportedLimits,
) -> IdentityReport {
    let called = line.catalogue_name(model);

    let mut observed = format!("firmware {version_text} identifies this as {called}");
    if let Some(t) = seen.limit_torque {
        observed.push_str(&format!("; limit_torque {t:.3} N·m"));
    }
    if let Some(v) = seen.limit_spd {
        observed.push_str(&format!("; limit_spd {v:.3} rad/s"));
    }

    // Only a refutation counts. An unanswered read is not a wrong answer, and
    // `consistent_with` passes anything it cannot rule out.
    let contradiction = (!seen.is_empty() && !seen.consistent_with(model)).then(|| {
        let scales = robstride_driver::MitScales::for_model(model);
        format!(
            "The two channels disagree: the firmware version says {called}, whose MIT \
             range is ±{:.1} N·m / ±{:.1} rad/s, but the motor reports limits that range \
             cannot express. Trust neither until this is resolved — the version→model \
             mapping comes from one unit per product line and breaks if the vendor bumps \
             a version component (doc/handover.md §4). Torque may be mis-scaled whichever \
             channel is right.",
            scales.torque, scales.velocity
        )
    });

    let selection_warning = (model != selected).then(|| {
        format!(
            "{observed}, but the session was opened as {label}. MIT scaling is wrong — \
             reconnect as {called}."
        )
    });

    // Both can fire at once, and they say different things: one that the
    // operator picked the wrong model, the other that we may not know which
    // model is right.
    let warning = match (selection_warning, contradiction) {
        (Some(a), Some(b)) => Some(format!("{a} {b}")),
        (Some(a), None) => Some(a),
        (None, Some(b)) => Some(format!("{observed}. {b}")),
        (None, None) => None,
    };

    IdentityReport { observed, warning }
}

/// Ask a RobStride motor whether the selected model is even possible.
///
/// Uses only the documented classic parameter reads (`limit_torque`,
/// `limit_spd`). That is weaker than asking the motor its name, and
/// deliberately so — see below.
///
/// This can only *refute*: a motor honouring a 17 N·m torque limit is not a
/// model whose MIT encoding tops out at ±5.5. It cannot identify one, because
/// RobStride publishes no model register.
///
/// It also reads `AppCodeName`, the firmware build's own name for itself — one
/// row, five frames. That is the only self-describing row observed populated
/// on real hardware: an EduLite05 reports `"EL_motor"` where `Name` and
/// `BarCode` are erased flash on every unit measured. It names the product
/// line but not the size within it, so it is shown to the operator rather than
/// acted on.
///
/// Never fails the connection. A motor that will not answer a parameter read
/// is still a motor you may need to talk to, and an unanswered question is not
/// a wrong answer.
pub(crate) fn robstride_identity<B: robstride_driver::RobstrideBus>(
    motor: &mut RsMotor<B>,
    selected: MotorModel,
    selected_name: &str,
) -> IdentityReport {
    // Report against the name the operator actually picked. Telling someone
    // who chose `EduLite05` that "RS-05 cannot express this" reads as a
    // message about a motor they did not select.
    let label = MotorModel::describe_selection(selected_name)
        .map_or_else(|| selected.to_string(), |c| c.to_string());

    // The documented single-frame version read, not the parameter-table
    // strings.
    //
    // `AppCodeName` was on this path briefly and is not any more. Two reasons,
    // both measured: it comes back **truncated** (a real RS-04 returns `"moto"`
    // where the vendor tool shows `"motor"`, so we abandon the reply while the
    // motor is still streaming sub-frames), and on this unit the reads that
    // follow it then time out until the motor is power-cycled. Whether the
    // undocumented extended-parameter path causes that or merely precedes it
    // is unproven — which is reason enough to keep it off the path every
    // session must survive. `robstride-cli identify` still runs it on demand.
    let version = motor.read_version().ok();

    // The firmware version names the motor outright: its minor component is
    // the model number, as RobStride's own release assets show
    // (`rs04-0.4.1.32.bin`) and a live RS-04 confirms by reporting exactly
    // that version. This is the strongest signal available and the one the
    // vendor tool uses.
    let named = version.and_then(|v| MotorModel::from_firmware_version(v.version));
    if let Some((model, line)) = named {
        // Ask the second channel too, even though the version already answered.
        //
        // The version→model table rests on n = 1 per product line and has zero
        // headroom: it decodes the minor byte as the model number, so a vendor
        // who bumps the major once breaks it, and `0.5.x.x` on an EduLite would
        // break it silently in the other direction. `limit_torque` is an
        // independent channel — measured 115 N·m on the RS-04, 6 on the
        // EduLite05 — and until now nothing compared the two. A disagreement is
        // the only evidence available that the table has gone stale.
        //
        // Costs two documented register reads on every connect. They are the
        // same reads the fallback path below already makes, so this adds round
        // trips rather than risk, and `read_reported_limits` reports failure by
        // returning nothing rather than by failing.
        let seen = motor.read_reported_limits();
        let report = robstride_named_identity(
            model,
            line,
            selected,
            &label,
            &version.expect("named implies a version").to_string(),
            seen,
        );
        match &report.warning {
            Some(w) => log::warn!("{w}"),
            None => log::info!("{}; matches the selection", report.observed),
        }
        return report;
    }

    let seen = motor.read_reported_limits();
    if seen.is_empty() {
        return match version {
            Some(v) => IdentityReport {
                observed: format!("motor firmware version {v}"),
                warning: None,
            },
            None => IdentityReport::default(),
        };
    }

    let mut parts = Vec::new();
    if let Some(v) = &version {
        parts.push(format!("firmware {v}"));
    }
    if let Some(t) = seen.limit_torque {
        parts.push(format!("limit_torque {t:.3} N·m"));
    }
    if let Some(v) = seen.limit_spd {
        parts.push(format!("limit_spd {v:.3} rad/s"));
    }
    let observed = format!("motor reports {}", parts.join(", "));

    let limits_warning = (!seen.consistent_with(selected)).then(|| {
        let scales = robstride_driver::MitScales::for_model(selected);
        let candidates: Vec<&str> = seen.candidates().map(|m| m.name()).collect();
        let suggestion = if candidates.is_empty() {
            "No known model matches those readings — check the interface and the motor id."
                .to_string()
        } else {
            format!("Consistent with: {}.", candidates.join(", "))
        };
        format!(
            "{observed}, which {label} cannot express in MIT mode \
             (±{:.1} N·m, ±{:.1} rad/s). Torque commands and readings would be \
             mis-scaled. {suggestion}",
            scales.torque, scales.velocity
        )
    });

    let warning = limits_warning;
    if let Some(w) = &warning {
        log::warn!("{w}");
    } else {
        log::info!("{observed}; consistent with {label}");
    }
    IdentityReport { observed, warning }
}

/// Read a DAMIAO motor's real MIT ranges out of `PMAX`/`VMAX`/`TMAX`.
///
/// Unlike RobStride this is authoritative rather than circumstantial: the
/// registers *are* the quantisation ranges the firmware uses, so reading them
/// removes the guess entirely and the selected model stops mattering for
/// scaling. Failure is reported, not fatal — the per-model defaults still
/// apply, which is what the driver would have used anyway.
pub(crate) fn damiao_identity<B: damiao_driver::DamiaoBus>(
    motor: &mut DamiaoMotor<B>,
    selected: DmModel,
) -> IdentityReport {
    // The reduction ratio names the motor exactly — see
    // `DamiaoMotor::identify_model`. Scaling does not depend on it (the
    // registers below do), but the register layout above RID 0x25 does, and so
    // does telling the operator what is actually plugged in.
    let named = motor.identify_model().ok();
    let model_warning = named.and_then(|m| {
        (m.register_layout() != selected.register_layout()).then(|| {
            format!(
                "motor reports a {}:1 reduction, which makes it a {} — the session was \
                 opened as {}, and the two use different register layouts above RID 0x25.",
                m.gear_ratio(),
                m.name(),
                selected.name()
            )
        })
    });

    match motor.refresh_limits_from_registers() {
        Ok(l) => {
            let observed = match named {
                Some(m) => format!(
                    "motor identifies as {} (gear ratio {}:1); MIT ranges from its own \
                     registers: ±{:.3} rad, ±{:.3} rad/s, ±{:.3} N·m",
                    m.name(),
                    m.gear_ratio(),
                    l.p_max,
                    l.v_max,
                    l.t_max
                ),
                None => format!(
                    "MIT ranges read from the motor: ±{:.3} rad, ±{:.3} rad/s, ±{:.3} N·m",
                    l.p_max, l.v_max, l.t_max
                ),
            };
            log::info!("{observed}");
            IdentityReport {
                observed,
                warning: model_warning,
            }
        }
        Err(e) => IdentityReport {
            observed: String::new(),
            warning: Some(format!(
                "could not read PMAX/VMAX/TMAX ({e}); MIT scaling falls back to the \
                 defaults for the selected model, which no manual states"
            )),
        },
    }
}

pub fn validate_driver_args(cfg: &DriverConfig) -> Result<()> {
    // The simulator has no bus to name.
    if cfg.interface.is_empty() && cfg.kind != DriverKind::Sim {
        bail!("--interface is required");
    }
    if cfg.motor_id == 0 {
        bail!("--motor-id must be > 0");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use robstride_driver::{ProductLine, ReportedLimits};

    fn limits(torque: Option<f32>, speed: Option<f32>) -> ReportedLimits {
        ReportedLimits {
            limit_torque: torque,
            limit_spd: speed,
        }
    }

    /// The real RS-04: firmware `0.4.1.32`, `limit_torque` 115 N·m against a
    /// ±120 MIT range. Both channels agree, so nothing should be said.
    #[test]
    fn agreeing_channels_produce_no_warning() {
        let r = robstride_named_identity(
            MotorModel::Rs04,
            ProductLine::Rs,
            MotorModel::Rs04,
            "RS-04",
            "0.4.1.32",
            limits(Some(115.0), Some(1.0)),
        );
        assert!(r.warning.is_none(), "{:?}", r.warning);
        assert!(r.observed.contains("115.000"), "the reading is shown: {}", r.observed);
    }

    /// The case this cross-check exists for: the version names a model whose
    /// MIT range cannot express the limit the motor reports. Before this, the
    /// version won unchallenged and the contradiction was never looked at.
    ///
    /// Fires **even though the operator's selection matches the version** —
    /// agreement between the two things that could both be wrong is not
    /// evidence.
    #[test]
    fn a_limit_the_named_model_cannot_express_is_reported() {
        let r = robstride_named_identity(
            MotorModel::Rs05,
            ProductLine::EduLite,
            MotorModel::Rs05,
            "EduLite05",
            "10.5.0.1",
            // 115 N·m against an EduLite05's ±5.5.
            limits(Some(115.0), None),
        );
        let w = r.warning.expect("a contradiction must be reported");
        assert!(w.contains("disagree"), "{w}");
        assert!(w.contains("handover.md"), "points at the evidence: {w}");
    }

    /// A wrong selection still reports as before, and says what to do.
    #[test]
    fn a_mismatched_selection_is_still_reported() {
        let r = robstride_named_identity(
            MotorModel::Rs04,
            ProductLine::Rs,
            MotorModel::Rs05,
            "EduLite05",
            "0.4.1.32",
            limits(Some(115.0), None),
        );
        let w = r.warning.expect("a mismatch must be reported");
        assert!(w.contains("reconnect as"), "{w}");
        assert!(!w.contains("disagree"), "the channels agree here: {w}");
    }

    /// Both at once: the operator picked the wrong model *and* the channels
    /// disagree. Neither message may swallow the other — they call for
    /// different actions.
    #[test]
    fn both_problems_are_reported_together() {
        let r = robstride_named_identity(
            MotorModel::Rs05,
            ProductLine::EduLite,
            MotorModel::Rs04,
            "RS-04",
            "10.5.0.1",
            limits(Some(115.0), None),
        );
        let w = r.warning.expect("two problems must be reported");
        assert!(w.contains("reconnect as"), "{w}");
        assert!(w.contains("disagree"), "{w}");
    }

    /// A motor that will not answer the parameter read must not look like a
    /// motor that answered wrongly. Silence is not a contradiction.
    #[test]
    fn unanswered_limits_are_not_a_contradiction() {
        let r = robstride_named_identity(
            MotorModel::Rs05,
            ProductLine::EduLite,
            MotorModel::Rs05,
            "EduLite05",
            "10.5.0.1",
            limits(None, None),
        );
        assert!(r.warning.is_none(), "{:?}", r.warning);
    }

    /// The sentinel has to be something no operator can legitimately type.
    /// It used to be `"Edulite05"`, so "I didn't say" and "I want an EduLite05"
    /// were the same string — and an RS-04 driven under that default ran at
    /// RS-05 scaling, a 21.8x torque error in silence.
    #[test]
    fn the_unspecified_sentinel_is_not_a_real_model() {
        assert!(MotorModel::from_name(MODEL_UNSPECIFIED).is_none());
        assert!(!known_models(DriverKind::Robstride).contains(&MODEL_UNSPECIFIED));
    }

    #[test]
    fn robstride_refuses_to_guess_a_model() {
        let cfg = DriverConfig {
            kind: DriverKind::Robstride,
            interface: "pcan:usb1".into(),
            ..Default::default()
        };
        assert_eq!(cfg.model, MODEL_UNSPECIFIED);
        let Err(err) = build_actuator(&cfg) else {
            panic!("must not open without a model");
        };
        let msg = format!("{err:#}");
        assert!(msg.contains("--model is required"), "{msg}");
        // The message has to say what to pass, including the EduLite names.
        assert!(msg.contains("RS-04"), "{msg}");
        assert!(msg.contains("EduLite05"), "{msg}");
    }

    /// Drivers that *do* have a safe fallback must keep working unchanged —
    /// the point was to remove a bad guess, not to make everything mandatory.
    #[test]
    fn the_simulator_still_runs_with_no_model() {
        let cfg = DriverConfig {
            kind: DriverKind::Sim,
            interface: String::new(),
            ..Default::default()
        };
        assert!(build_actuator(&cfg).is_ok(), "sim must not need a model");
    }

    #[test]
    fn the_picker_offers_the_edulite_names() {
        let names = known_models(DriverKind::Robstride);
        for expected in ["RS-04", "RS-05", "EduLite05"] {
            assert!(names.contains(&expected), "{expected} missing from {names:?}");
        }
    }
}
