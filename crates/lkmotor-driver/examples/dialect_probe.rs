//! Probe firmware dialect: V2.36 (`0xC0` family) vs older (`0x30`/`0x33`).
//!
//! `dump_params` and `param_probe` confirmed that `0xC0` (the V2.36 read-control
//! parameter command) gets zero response on this bench, while `0x9C` works fine.
//! That suggests the firmware is older and uses the legacy `0x30..0x34` commands
//! (which `lkmotor-protocol::Command::ReadPid = 0x30` already references).
//!
//! This probe sends a battery of frame variants with different payload lengths
//! and dumps the raw bytes that come back, so we can pick the right one.
//!
//! ```ignore
//! cargo run -p lkmotor-driver --example dialect_probe -- \
//!     --device /dev/ttyUSB0 --baud 1000000 --id 1
//! ```

use std::time::Duration;

use clap::Parser;
use lkmotor_driver::protocol::frame::{MAX_FRAME, encode};
use lkmotor_driver::LkCommands;
use lkmotor_driver::{MotorId, Rs485Driver};

#[derive(Parser, Debug)]
#[command(about = "Probe several read-PID/read-param command shapes to find what this firmware speaks")]
struct Args {
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    device: String,
    #[arg(short, long, default_value_t = 1_000_000)]
    baud: u32,
    #[arg(short, long, default_value_t = 1)]
    id: u8,
    #[arg(long, default_value_t = 200)]
    window_ms: u64,
    #[arg(long, default_value_t = 100)]
    preflight_timeout_ms: u64,
}

fn hex(label: &str, bytes: &[u8]) {
    print!("    {label:<14} [{:>2} bytes]", bytes.len());
    if bytes.is_empty() {
        println!("  (none)");
        return;
    }
    print!("  ");
    for (i, b) in bytes.iter().enumerate() {
        print!("{:02X}", b);
        if i + 1 < bytes.len() {
            print!(" ");
        }
    }
    println!();
}

fn try_one(motor: &mut Rs485Driver, id: MotorId, label: &str, cmd: u8, data: &[u8], window: Duration) {
    println!("\n== {} ==", label);
    let _ = motor.flush_rx();

    let mut tx = [0u8; MAX_FRAME];
    let n = match encode(cmd, id.get(), data, &mut tx) {
        Ok(n) => n,
        Err(e) => {
            println!("    encode err: {:?}", e);
            return;
        }
    };
    hex("TX", &tx[..n]);

    if let Err(e) = motor.send_raw(cmd, id, data) {
        println!("    send_raw err: {e}");
        return;
    }

    let rx = match motor.read_raw_for(window) {
        Ok(b) => b,
        Err(e) => {
            println!("    read err: {e}");
            return;
        }
    };
    hex("RX", &rx);

    if rx.is_empty() {
        println!("    -> silent");
    } else if let Some(idx) = rx.iter().position(|&b| b == 0x3E) {
        let after = &rx[idx..];
        if after.len() >= 4 {
            println!(
                "    -> first frame: cmd=0x{:02X} id=0x{:02X} len=0x{:02X}  {}",
                after[1],
                after[2],
                after[3],
                if after[1] == cmd {
                    "(echo OK)"
                } else {
                    "(different cmd code!)"
                }
            );
        }
    } else {
        println!("    -> bytes received but no 0x3E header (junk)");
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let args = Args::parse();

    let id = MotorId::new(args.id).ok_or("motor id must be 1..=32")?;
    let mut motor = Rs485Driver::open(
        &args.device,
        args.baud,
        Duration::from_millis(args.preflight_timeout_ms),
    )?;

    println!("== pre-flight: read_state2 (0x9C) ==");
    let _ = motor.flush_rx();
    match motor.read_state2(id) {
        Ok(s) => println!("  OK — temp={}°C enc={}", s.temperature_c, s.encoder_raw),
        Err(e) => {
            println!("  FAIL: {e} — bus problem, abort");
            return Ok(());
        }
    }

    let window = Duration::from_millis(args.window_ms);

    // ---- V2.36 family: 0xC0 with various payload sizes ----
    try_one(&mut motor, id, "0xC0 len=0  (no payload)", 0xC0, &[], window);
    try_one(&mut motor, id, "0xC0 len=1  paramID=0x0A only", 0xC0, &[0x0A], window);
    try_one(
        &mut motor,
        id,
        "0xC0 len=7  paramID=0x0A + 6×0 (spec)",
        0xC0,
        &[0x0A, 0, 0, 0, 0, 0, 0],
        window,
    );

    // ---- Legacy (V2-era) commands present in command.rs ----
    try_one(&mut motor, id, "0x30 len=0  (legacy ReadPid)", 0x30, &[], window);
    try_one(&mut motor, id, "0x30 len=1  (legacy ReadPid + idx?)", 0x30, &[0x00], window);
    try_one(&mut motor, id, "0x33 len=0  (legacy ReadAccel — known V2)", 0x33, &[], window);

    // ---- Sanity: 0x90 read encoder (V3, but simple, often universal) ----
    try_one(&mut motor, id, "0x90 len=0  (ReadEncoder, sanity)", 0x90, &[], window);

    println!("\nDone. Look for any '(echo OK)' lines — those are the commands this firmware accepts.");
    Ok(())
}
