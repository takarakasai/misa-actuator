//! `version` / `read-param` / `param-table` command handlers — RobStride's
//! undocumented bulk/single "parameter table" reads (see
//! `robstride_protocol::param_table` and `param_type_table`).
//!
//! Split out of `main.rs` since these three commands share the same
//! type-decoding display logic (the manual-authoritative decode, with a
//! byte-width fallback for FunctionCodes the manual doesn't list).

use std::collections::BTreeMap;
use std::path::Path;
use std::time::Duration;

use anyhow::{Context, Result};
use robstride_driver::{
    lookup_param_type, BuildName, Motor, MotorModel, ParamIndex, ParamType, TypedValue,
};
use serde::Serialize;

pub fn run_version(motor: &mut Motor, motor_id: u8, timeout_ms: u64) -> Result<()> {
    let version = motor.read_firmware_version(Duration::from_millis(timeout_ms))?;
    println!("motor={motor_id}  AppCodeVersion={version}");
    Ok(())
}

/// Print everything the motor can be made to say about which model it is.
///
/// Deliberately shows the evidence rather than just a verdict. The strong
/// route (a self-reported name) is unwritten on at least one real unit, and
/// the fallback can only rule models out — an operator deciding whether to
/// trust the answer needs to see which of those they got, and the raw bytes of
/// any row that came back illegible.
pub fn run_identify(
    motor: &mut Motor,
    selected: Option<MotorModel>,
    deep: bool,
    timeout_ms: u64,
) -> Result<()> {
    match selected {
        Some(m) => println!("opened as {m}"),
        None => println!("no --model given — reporting evidence only"),
    }

    // The version read is what the vendor tool actually sends when it decides
    // a motor's type, so it goes first and its raw bytes are shown — byte 7 is
    // undocumented and is the only unexplained field left in that exchange.
    let mut from_version = None;
    match motor.read_version() {
        Ok(v) => {
            print!(
                "\nversion read (comm type 26): {v}  [trailing byte 0x{:02X}]",
                v.trailing
            );
            match MotorModel::from_firmware_version(v.version) {
                Some((model, line)) => {
                    println!("  → {}", line.catalogue_name(model));
                    from_version = Some(model);
                }
                None => println!("  (no model in this version)"),
            }
        }
        Err(e) => println!("\nversion read (comm type 26): no answer ({e})"),
    }

    let strings = if deep {
        motor.read_identity_strings(Duration::from_millis(timeout_ms))
    } else {
        println!(
            "\nself-reported names: skipped (pass --deep). \
             The parameter-table path truncates and has left a real RS-04 \
             unresponsive until power-cycled."
        );
        Vec::new()
    };
    if deep && strings.is_empty() {
        println!("\nself-reported names: no row answered");
    } else if deep {
        println!("\nself-reported names:");
        for s in &strings {
            print!("  0x{:04X} {:<13} ", s.function_code, s.label);
            if s.is_printable() {
                print!("{:?}", s.text);
            } else {
                print!("unwritten or unreadable — raw [{}]", s.raw_hex());
            }
            match s.model {
                Some(m) => println!("  → {m}"),
                None => println!(),
            }
        }
    }

    // The firmware version wins: it names the motor outright, where the
    // parameter-table strings at best narrow to a product line.
    let named = from_version.or_else(|| strings.iter().find_map(|s| s.model));

    // `AppCodeName` names the product line, not the size within it — a real
    // EduLite05 says `"EL_motor"`, with no `05` anywhere. So it narrows the
    // field rather than answering, and saying so is the whole point.
    let build = strings
        .iter()
        .find(|s| s.function_code == 0x1007 && s.is_printable());
    let mut family = None;
    if let Some(b) = build {
        print!("\nfirmware build {:?} — ", b.text);
        match MotorModel::classify_build_name(&b.text) {
            Some(BuildName::Narrows(models)) => {
                let names: Vec<&str> = models.iter().map(|m| m.name()).collect();
                println!(
                    "the {} line (the build name does not carry the size)",
                    names.join(" / ")
                );
                family = Some(models);
            }
            Some(BuildName::Uninformative) => println!(
                "carries no model information. The RS line reports this \
                 verbatim on every size."
            ),
            None => println!("not a build name we have seen; nothing inferred from it."),
        }
    }

    let limits = motor.read_reported_limits();
    println!("\nreported limits:");
    match limits.limit_torque {
        Some(v) => println!("  limit_torque  {v} N·m"),
        None => println!("  limit_torque  (no answer)"),
    }
    match limits.limit_spd {
        Some(v) => println!("  limit_spd     {v} rad/s"),
        None => println!("  limit_spd     (no answer)"),
    }

    println!();
    // Intersect the two independent narrowings when both are available.
    let candidates: Vec<&str> = limits
        .candidates()
        .filter(|m| family.is_none_or(|f| f.contains(m)))
        .map(|m| m.name())
        .collect();
    match (named, selected) {
        (Some(m), Some(s)) if m == s => println!("VERDICT: the motor identifies as {m} — matches."),
        (Some(m), Some(s)) => println!(
            "VERDICT: the motor identifies as {m}, but this session opened as {s}. \
             Use --model {m} — MIT scaling is wrong until you do."
        ),
        (Some(m), None) => println!("VERDICT: the motor identifies as {m}. Use --model {m}."),
        (None, _) if limits.is_empty() => {
            println!("VERDICT: the motor said nothing usable. The model cannot be checked.")
        }
        (None, Some(s)) if !limits.consistent_with(s) => println!(
            "VERDICT: the limits RULE OUT {s}. Consistent with: {}.",
            if candidates.is_empty() {
                "nothing known".to_string()
            } else {
                candidates.join(", ")
            }
        ),
        // One survivor is an identification, not a narrowing. Saying "this
        // rules models out; it does not confirm one" when exactly one model
        // fits would understate real evidence.
        (None, _) if candidates.len() == 1 => println!(
            "VERDICT: the limits fit exactly one model in the family — {}. \
             Nothing else quantises that far.",
            candidates[0]
        ),
        (None, Some(s)) => println!(
            "VERDICT: the limits are consistent with {s}, and with {}. \
             That rules models out; it does not pick one.",
            candidates
                .iter()
                .filter(|c| **c != s.name())
                .copied()
                .collect::<Vec<_>>()
                .join(", ")
        ),
        (None, None) => println!(
            "VERDICT: the limits are consistent with: {}. \
             That narrows the field but does not pick one — check the label on the motor.",
            if candidates.is_empty() {
                "nothing known".to_string()
            } else {
                candidates.join(", ")
            }
        ),
    }
    Ok(())
}

pub fn run_read_param(motor: &mut Motor, function_code: u16, timeout_ms: u64) -> Result<()> {
    let raw = motor.read_single_param(function_code, Duration::from_millis(timeout_ms))?;
    let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
    let as_string = String::from_utf8_lossy(&raw[..end]).trim().to_string();
    println!("FunctionCode 0x{function_code:04X}:  ({} byte(s) received)", raw.len());
    println!("  raw bytes = {raw:02X?}");

    // Authoritative decode via the RS04/EL05 manual's parameter type table,
    // when this FunctionCode is listed there. Unlike `param-table`, this
    // single-row path gets no name back from the motor, so the lookup is by
    // FunctionCode alone — see `lookup_param_type`'s doc comment for why
    // that's not fully reliable on every firmware build.
    match lookup_param_type(function_code) {
        Some(info) => {
            // Label it as coming from the manual, not from the motor. Measured
            // on an RS04 (AppCodeVersion 0.4.1.32): 32 of its 84 rows disagree
            // with the manual's table, which is shifted by one or more places
            // across 0x2006-0x2021 and 0x3010-0x301C. That makes the name and
            // the decode width below a guess on this firmware — 0x2009 reads
            // back the motor's CAN_ID while the table calls it `motor_baud`.
            // See doc/param-table-firmware-divergence.md.
            print!("  per the manual's table: {} ({:?}) = ", info.name, info.ty);
            match info.ty {
                ParamType::String => println!("{as_string:?}"),
                ParamType::U8 => println!("{:?}", raw.first().copied()),
                ParamType::U16 => println!(
                    "{:?}",
                    raw.get(0..2).map(|b| u16::from_le_bytes([b[0], b[1]]))
                ),
                ParamType::I16 => println!(
                    "{:?}",
                    raw.get(0..2).map(|b| i16::from_le_bytes([b[0], b[1]]))
                ),
                ParamType::U32 => println!(
                    "{:?}",
                    raw.get(0..4)
                        .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                ),
                ParamType::I32 => println!(
                    "{:?}",
                    raw.get(0..4)
                        .map(|b| i32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                ),
                ParamType::F32 => println!(
                    "{:?}",
                    raw.get(0..4)
                        .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                ),
            }
        }
        None => println!(
            "  as string = {as_string:?}  (unknown type — not in the manual's table; every plausible width shown below)"
        ),
    }
    if lookup_param_type(function_code).is_some() {
        println!(
            "  NOTE: that name/type is the manual's, not the motor's. This firmware may lay the \
             table out differently (measured: RS04 0.4.1.32 disagrees on 32 rows). Run \
             `param-table` for the names the motor reports itself, and prefer the raw widths below."
        );
    }

    // Numeric single-frame reply (4 bytes): CurrentValue only, at offset 0
    // (unlike the bulk stream's offset 8 — see the module docs on
    // `read_single_param`).
    if raw.len() == 4 {
        let b = [raw[0], raw[1], raw[2], raw[3]];
        println!(
            "  current: u8={}  u16={}  i16={}  u32={}  i32={}  f32={}",
            raw[0],
            u16::from_le_bytes([b[0], b[1]]),
            i16::from_le_bytes([b[0], b[1]]),
            u32::from_le_bytes(b),
            i32::from_le_bytes(b),
            f32::from_le_bytes(b),
        );
    }
    // Bulk-stream-shaped numeric reply (12 bytes):
    // [min_i16][max_i16][4 unidentified][current, native width].
    if raw.len() >= 12 {
        let min = i16::from_le_bytes([raw[0], raw[1]]);
        let max = i16::from_le_bytes([raw[2], raw[3]]);
        println!("  min={min}  max={max}");
        println!(
            "  current: u8={:?}  u16={:?}  i16={:?}  u32={:?}  i32={:?}  f32={:?}",
            raw.get(8).copied(),
            raw.get(8..10).map(|b| u16::from_le_bytes([b[0], b[1]])),
            raw.get(8..10).map(|b| i16::from_le_bytes([b[0], b[1]])),
            raw.get(8..12)
                .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]])),
            raw.get(8..12)
                .map(|b| i32::from_le_bytes([b[0], b[1], b[2], b[3]])),
            raw.get(8..12)
                .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])),
        );
    }
    Ok(())
}

/// Named entries from `robstride_protocol::param::ParamIndex` (the
/// officially documented 0x7xxx-ish "Read/Write a Single Parameter List"
/// space) worth including in a bulk settings dump. Keep in sync with
/// `ParamIndex`'s variants.
const CLASSIC_PARAMS: &[(&str, ParamIndex)] = &[
    ("mech_offset", ParamIndex::MechOffset),
    ("measured_position", ParamIndex::MeasuredPosition),
    ("measured_velocity", ParamIndex::MeasuredVelocity),
    ("measured_torque", ParamIndex::MeasuredTorque),
    ("run_mode", ParamIndex::RunMode),
    ("iq_ref", ParamIndex::IqRef),
    ("spd_ref", ParamIndex::SpdRef),
    ("limit_torque", ParamIndex::LimitTorque),
    ("cur_kp", ParamIndex::CurKp),
    ("cur_ki", ParamIndex::CurKi),
    ("cur_filt_gain", ParamIndex::CurFiltGain),
    ("loc_ref", ParamIndex::LocRef),
    ("limit_spd", ParamIndex::LimitSpd),
    ("limit_cur", ParamIndex::LimitCur),
    ("mech_pos", ParamIndex::MechPos),
    ("iq_filt", ParamIndex::IqFilt),
    ("mech_vel", ParamIndex::MechVel),
    ("vbus", ParamIndex::Vbus),
    ("loc_kp", ParamIndex::LocKp),
    ("spd_kp", ParamIndex::SpdKp),
    ("spd_ki", ParamIndex::SpdKi),
    ("spd_filt_gain", ParamIndex::SpdFiltGain),
    ("acc_rad", ParamIndex::AccRad),
    ("vel_max", ParamIndex::VelMax),
    ("acc_set", ParamIndex::AccSet),
    ("can_timeout", ParamIndex::CanTimeout),
    ("zero_state", ParamIndex::ZeroState),
];

#[derive(Debug, Serialize)]
struct ParamsDump {
    motor_id: u8,
    /// Officially documented 0x7xxx `ParamIndex` space (comm_type 17).
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    classic_params: BTreeMap<String, f32>,
    /// RobStride's undocumented bulk parameter table (comm_type 19).
    ///
    /// **Not necessarily the same backing store as `classic_params`** — a
    /// live capture found at least one address (`MechOffset` / `0x2005`)
    /// read back different values through the two paths on real hardware.
    /// Kept as a separate section rather than merged so this ambiguity
    /// stays visible in the dump instead of being silently resolved.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    extended_params: Vec<ExtendedParamDump>,
}

#[derive(Debug, Serialize)]
struct ExtendedParamDump {
    function_code: String,
    name: String,
    value: toml::Value,
}

fn typed_to_toml(v: &TypedValue) -> toml::Value {
    match v {
        TypedValue::String(s) => toml::Value::String(s.clone()),
        TypedValue::U8(x) => toml::Value::Integer(*x as i64),
        TypedValue::U16(x) => toml::Value::Integer(*x as i64),
        TypedValue::I16(x) => toml::Value::Integer(*x as i64),
        TypedValue::U32(x) => toml::Value::Integer(*x as i64),
        TypedValue::I32(x) => toml::Value::Integer(*x as i64),
        TypedValue::F32(x) => toml::Value::Float(*x as f64),
    }
}

/// Read every known motor-resident setting in one pass: the documented
/// classic `ParamIndex` space (one read per entry, comm_type 17 — a failed
/// read is reported and skipped rather than aborting the dump) plus the
/// undocumented bulk parameter table (comm_type 19, `param-table`'s data
/// source). Optionally serialize the result to TOML, matching
/// `myactuator-cli params --toml --out`'s behavior.
#[allow(clippy::too_many_arguments)]
pub fn run_params(
    motor: &mut Motor,
    motor_id: u8,
    table_timeout_ms: u64,
    want_toml: bool,
    out: Option<&Path>,
) -> Result<()> {
    println!("== classic ParamIndex (0x7xxx, comm_type 17) — motor {motor_id} ==");
    let mut classic_params = BTreeMap::new();
    for &(label, param) in CLASSIC_PARAMS {
        match motor.read_param(param) {
            Ok(v) => {
                println!("  {label:<16} = {v}");
                classic_params.insert(label.to_string(), v);
            }
            Err(e) => println!("  {label:<16} = <no reply: {e}>"),
        }
    }

    println!("\n== extended parameter table (comm_type 19) — motor {motor_id} ==");
    let table = motor.read_param_table(Duration::from_millis(table_timeout_ms))?;
    let mut extended_params = Vec::with_capacity(table.len());
    for e in &table {
        let value = match e.value_current_typed() {
            Some(typed) => {
                println!("  0x{:04X}  {:<20} = {}", e.function_code, e.name, typed);
                typed_to_toml(&typed)
            }
            None => {
                let raw = format!("{:02X?}", e.value_raw);
                println!(
                    "  0x{:04X}  {:<20} = <unknown type, raw={raw}>",
                    e.function_code, e.name
                );
                toml::Value::String(format!("raw:{raw}"))
            }
        };
        extended_params.push(ExtendedParamDump {
            function_code: format!("0x{:04X}", e.function_code),
            name: e.name.clone(),
            value,
        });
    }

    if want_toml || out.is_some() {
        let dump = ParamsDump {
            motor_id,
            classic_params,
            extended_params,
        };
        let text = toml::to_string_pretty(&dump).context("failed to serialize TOML dump")?;
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
    Ok(())
}

pub fn run_param_table(motor: &mut Motor, timeout_ms: u64) -> Result<()> {
    let table = motor.read_param_table(Duration::from_millis(timeout_ms))?;
    println!("{} row(s):", table.len());
    for e in &table {
        match e.value_current_typed() {
            // Authoritative decode via the RS04/EL05 manual's parameter
            // type table (robstride_protocol::param_type_table), matched by
            // this row's own wire-reported name.
            Some(typed) => {
                println!("  0x{:04X}  {:<20} = {}", e.function_code, e.name, typed)
            }
            // FunctionCode/name not in the manual's table (reserved/
            // undocumented row) — fall back to showing every plausible
            // width, same as before this table existed.
            None => println!(
                "  0x{:04X}  {:<16} (unknown type — not in the manual's table) min={:<7} max={:<7} u8={:<5} u16={:<7} i16={:<7} u32={:<11} i32={:<11} f32={}  raw={:02X?}",
                e.function_code,
                e.name,
                e.value_min_i16().map_or("?".to_string(), |v| v.to_string()),
                e.value_max_i16().map_or("?".to_string(), |v| v.to_string()),
                e.value_current_u8().map_or("?".to_string(), |v| v.to_string()),
                e.value_current_u16().map_or("?".to_string(), |v| v.to_string()),
                e.value_current_i16().map_or("?".to_string(), |v| v.to_string()),
                e.value_current_u32().map_or("?".to_string(), |v| v.to_string()),
                e.value_current_i32().map_or("?".to_string(), |v| v.to_string()),
                e.value_current_f32().map_or("?".to_string(), |v| v.to_string()),
                e.value_raw,
            ),
        }
    }
    Ok(())
}
