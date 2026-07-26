//! `version` / `read-param` / `param-table` command handlers — RobStride's
//! undocumented bulk/single "parameter table" reads (see
//! `robstride_protocol::param_table` and `param_type_table`).
//!
//! Split out of `main.rs` since these three commands share the same
//! type-decoding display logic (the manual-authoritative decode, with a
//! byte-width fallback for FunctionCodes the manual doesn't list).

use std::time::Duration;

use anyhow::Result;
use robstride_driver::{lookup_param_type, Motor, ParamType};

pub fn run_version(motor: &mut Motor, motor_id: u8, timeout_ms: u64) -> Result<()> {
    let version = motor.read_firmware_version(Duration::from_millis(timeout_ms))?;
    println!("motor={motor_id}  AppCodeVersion={version}");
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
            print!("  {} ({:?}) = ", info.name, info.ty);
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
