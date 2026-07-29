//! Vendor-identification tool: for a range of CAN/bus addresses, tries every
//! vendor's own [`misa_actuator::Actuator::scan_bus`] in turn and reports
//! which vendor(s) responded to each id.
//!
//! Useful when a bus has actuators from more than one vendor and you don't
//! already know which id belongs to which — `misa-actuator-monitor`'s config
//! requires that mapping up front; this tool discovers it.
//!
//! # Example
//! ```text
//! misa-actuator-identify --interface can0 --from 1 --to 32
//! ```
//!
//! No new protocol code: this just reuses each vendor's existing
//! `scan_bus`, opened independently on the same interface (SocketCAN allows
//! multiple listening sockets on one interface; a vendor whose transport
//! doesn't match the given interface — e.g. lkmotor on a SocketCAN name —
//! simply fails to open and is skipped, not fatal to the whole run).
//!
//! For each id that answers, also fetches model name / firmware version
//! where the vendor exposes a wire read for it — these aren't part of the
//! generic [`misa_actuator::Actuator`] trait, so this reaches past `factory`
//! into each vendor's concrete driver type directly (`robstride_driver`'s
//! `read_firmware_version`, `myactuator_driver`'s `read_motor_model`/
//! `read_version_date`, `damiao_driver`'s `read_register(Rid::SUB_VER)`).
//!
//! Coverage checked against the official local manuals (see
//! `doc/vendor-identity-coverage.md` for the full writeup):
//! - **lkmotor**: neither the CAN (V2.36) nor RS485 protocol manual
//!   (`/home/takara/work/dp/lkmotor-driver/ref/`) documents *any* model-name
//!   or firmware-version command — genuinely not available, not a driver
//!   gap.
//! - **damiao**: no official manual is present in this workspace, but
//!   `damiao-protocol::Rid::SUB_VER` (register 36, already used elsewhere in
//!   this project) is documented as "firmware sub-version" — no model-name
//!   register was found. Shown as `fw_version` only.

use std::collections::BTreeMap;
use std::time::Duration;

use anyhow::Result;
use clap::Parser;

use damiao_driver::{DamiaoBus, DamiaoMotor, MotorModel as DmMotorModel, Rid as DmRid};
use myactuator_driver::{MotorConfig as MyaMotorConfig, MyActuatorMotor};
use robstride_driver::{Motor as RsMotor, MotorModel as RsMotorModel};

use misa_actuator_tui::factory::{build_actuator, BusKind, DriverConfig, DriverKind};

#[derive(Parser, Debug)]
#[command(
    version,
    about = "Identify which vendor's actuator answers on each bus address, by trying every driver's scan in turn"
)]
struct Cli {
    /// SocketCAN interface (e.g. can0) or serial port (e.g. /dev/ttyUSB0) —
    /// tried against every vendor; ones that don't match this transport
    /// simply fail to open and are skipped.
    #[arg(long)]
    interface: String,

    /// First id to probe.
    #[arg(long, default_value_t = 1)]
    from: u8,

    /// Last id to probe.
    #[arg(long, default_value_t = 32)]
    to: u8,

    /// Per-id timeout, in ms.
    #[arg(long, default_value_t = 50)]
    timeout_ms: u64,

    /// Which vendors to try. Defaults to all four — the point of this tool
    /// is not already knowing which one it is.
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = [
        DriverKind::Robstride, DriverKind::Damiao, DriverKind::Myactuator, DriverKind::Lkmotor,
    ])]
    vendors: Vec<DriverKind>,

    /// Lkmotor: serial baud rate.
    #[arg(long, default_value_t = 1_000_000)]
    baud: u32,

    /// Robstride/Damiao: motor model name (only matters if a vendor's
    /// `open()` validates it before scanning).
    #[arg(long, default_value = "Edulite05")]
    model: String,

    /// Damiao: physical CAN layer.
    #[arg(long, value_enum, default_value_t = BusKind::Can)]
    bus: BusKind,

    /// Skip the per-id model/firmware-version detail read (faster over wide
    /// ranges; just report which vendor(s) answered each id).
    #[arg(long)]
    no_details: bool,
}

/// Model name / firmware version for one (vendor, id), where the vendor's
/// driver exposes a wire read for it. `None` means either the read failed or
/// this vendor has no such command implemented — the two aren't
/// distinguished in the table (either way there's nothing to show).
struct Details {
    model: Option<String>,
    fw_version: Option<String>,
}

impl Details {
    const NONE: Details = Details { model: None, fw_version: None };
}

/// `DamiaoMotor<B>` is generic over the bus type (classic vs CAN-FD are
/// distinct concrete types), so `open`/`open_fd` can't share a `match` arm —
/// this factors out the common "set timeout, read SUB_VER" tail.
fn damiao_details<B: DamiaoBus>(mut motor: DamiaoMotor<B>, timeout: Duration) -> Details {
    let _ = motor.set_timeout(timeout);
    Details {
        model: None,
        fw_version: motor.read_register(DmRid::SW_VER).ok().map(|r| r.as_i32().to_string()),
    }
}

/// Opens a fresh connection addressed at `id` specifically (the scan step
/// above only opens a placeholder id to share the bus) and reads whatever
/// identification the vendor's driver supports.
fn read_details(
    kind: DriverKind,
    interface: &str,
    id: u8,
    model_name: &str,
    bus_kind: BusKind,
    timeout: Duration,
) -> Details {
    match kind {
        DriverKind::Robstride => {
            let Some(model) = RsMotorModel::from_name(model_name) else {
                return Details::NONE;
            };
            let Ok(mut motor) = RsMotor::open(interface, id, model) else {
                return Details::NONE;
            };
            let _ = motor.set_timeout(timeout);
            Details {
                model: None, // not readable from the wire on this vendor
                fw_version: motor.read_firmware_version(timeout).ok(),
            }
        }
        DriverKind::Myactuator => {
            let Ok(mut motor) = MyActuatorMotor::open(interface, id, MyaMotorConfig::current_units())
            else {
                return Details::NONE;
            };
            let _ = motor.set_timeout(timeout);
            Details {
                model: motor.read_motor_model().ok().filter(|s| !s.is_empty()),
                fw_version: motor.read_version_date().ok().map(|v| v.to_string()),
            }
        }
        DriverKind::Damiao => {
            // No model-name wire read exists (confirmed against the official
            // DM-J4310-2EC/DM-J3507-2EC manuals' Register Map — no
            // ASCII/string-typed register of any kind). `Rid::SW_VER` (RID
            // 14) is the actual firmware-version register per those same
            // manuals — not `Rid::SUB_VER` (RID 36, a separate minor/sub
            // version field), which an earlier pass here used before the
            // manuals were available.
            let Some(model) = DmMotorModel::from_name(model_name) else {
                return Details::NONE;
            };
            match bus_kind {
                BusKind::Can => match DamiaoMotor::open(interface, id, model) {
                    Ok(motor) => damiao_details(motor, timeout),
                    Err(_) => Details::NONE,
                },
                BusKind::CanFd => match DamiaoMotor::open_fd(interface, id, model) {
                    Ok(motor) => damiao_details(motor, timeout),
                    Err(_) => Details::NONE,
                },
            }
        }
        // Neither the CAN nor RS485 manual for this vendor documents a
        // model-name or firmware-version command (see module docs).
        DriverKind::Lkmotor => Details::NONE,
    }
}

fn vendor_label(kind: DriverKind) -> &'static str {
    match kind {
        DriverKind::Robstride => "robstride",
        DriverKind::Damiao => "damiao",
        DriverKind::Myactuator => "myactuator",
        DriverKind::Lkmotor => "lkmotor",
    }
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    let cli = Cli::parse();

    if cli.to < cli.from {
        anyhow::bail!("--to must be >= --from");
    }

    println!(
        "scanning {} for id {}..={} across {} vendor(s): {}",
        cli.interface,
        cli.from,
        cli.to,
        cli.vendors.len(),
        cli.vendors.iter().map(|v| vendor_label(*v)).collect::<Vec<_>>().join(", "),
    );

    // id -> vendors that responded.
    let mut hits: BTreeMap<u8, Vec<DriverKind>> = BTreeMap::new();

    for &kind in &cli.vendors {
        let label = vendor_label(kind);
        let cfg = DriverConfig {
            kind,
            interface: cli.interface.clone(),
            motor_id: 1, // placeholder — scan_bus shares the open bus, not tied to this id
            model: cli.model.clone(),
            baud: cli.baud,
            bus_kind: cli.bus,
            timeout: Duration::from_millis(cli.timeout_ms),
            ..DriverConfig::default()
        };

        let mut actuator = match build_actuator(&cfg) {
            Ok(a) => a,
            Err(e) => {
                println!("  {label}: failed to open bus ({e}) — skipped");
                continue;
            }
        };

        print!("  {label}: scanning... ");
        match actuator.scan_bus(cli.from..=cli.to, Duration::from_millis(cli.timeout_ms)) {
            Ok(ids) => {
                println!("{} responded", ids.len());
                for id in ids {
                    hits.entry(id).or_default().push(kind);
                }
            }
            Err(e) => println!("scan failed ({e})"),
        }
    }

    println!();
    if hits.is_empty() {
        println!("no motors found in {}..={} on {}", cli.from, cli.to, cli.interface);
        return Ok(());
    }

    let timeout = Duration::from_millis(cli.timeout_ms);
    let dash = "—".to_string();

    // One row per (id, vendor) so model/fw can differ per vendor on an
    // ambiguous id; rows are still grouped by id (BTreeMap iteration order).
    struct Row {
        id: u8,
        vendor: &'static str,
        model: String,
        fw: String,
        ambiguous: bool,
    }
    let mut rows = Vec::new();
    for (&id, vendors) in &hits {
        let ambiguous = vendors.len() > 1;
        for &kind in vendors {
            let d = if cli.no_details {
                Details::NONE
            } else {
                read_details(kind, &cli.interface, id, &cli.model, cli.bus, timeout)
            };
            rows.push(Row {
                id,
                vendor: vendor_label(kind),
                model: d.model.unwrap_or_else(|| dash.clone()),
                fw: d.fw_version.unwrap_or_else(|| dash.clone()),
                ambiguous,
            });
        }
    }

    let vendor_w = rows.iter().map(|r| r.vendor.len()).max().unwrap_or(6).max(6);
    let model_w = rows.iter().map(|r| r.model.chars().count()).max().unwrap_or(5).max(5);
    println!("{:<4} {:<vendor_w$} {:<model_w$} fw_version", "id", "vendor", "model");
    println!("{:-<4} {:-<vendor_w$} {:-<model_w$} {:-<10}", "", "", "", "");
    for r in &rows {
        let flag = if r.ambiguous {
            "   ⚠ ambiguous — verify wiring/addressing"
        } else {
            ""
        };
        println!(
            "{:<4} {:<vendor_w$} {:<model_w$} {}{}",
            r.id, r.vendor, r.model, r.fw, flag
        );
    }

    Ok(())
}
