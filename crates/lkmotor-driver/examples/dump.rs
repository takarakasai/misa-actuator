//! Dump raw response payloads for State1/State2/Encoder/MultiTurnAngle.
//!
//! Used to confirm field layout against the firmware manual when the typed
//! parsers return suspicious values (e.g., voltage reading 0).
//!
//! ```ignore
//! cargo run -p lkmotor-driver --example dump -- --device /dev/ttyUSB0 --baud 1000000 --id 1
//! ```

use std::time::Duration;

use clap::Parser;
use lkmotor_driver::{MotorId, Rs485Driver};

#[derive(Parser, Debug)]
#[command(about = "Hex-dump raw payloads from common LK Motor read commands")]
struct Args {
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    device: String,
    #[arg(short, long, default_value_t = 1_000_000)]
    baud: u32,
    #[arg(short, long, default_value_t = 1)]
    id: u8,
    #[arg(long, default_value_t = 100)]
    timeout_ms: u64,
}

fn dump(label: &str, bytes: &[u8]) {
    print!("{label:<24}");
    for (i, b) in bytes.iter().enumerate() {
        print!("{:02X}", b);
        if i + 1 < bytes.len() {
            print!(" ");
        }
    }
    println!();
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let args = Args::parse();
    let id = MotorId::new(args.id).ok_or("motor id must be 1..=32")?;
    let timeout = Duration::from_millis(args.timeout_ms);
    let mut motor = Rs485Driver::open(&args.device, args.baud, timeout)?;

    for (name, cmd) in [
        ("ReadMotorState1 (0x9A)", 0x9Au8),
        ("ReadMotorState2 (0x9C)", 0x9Cu8),
        ("ReadMotorState3 (0x9D)", 0x9Du8),
        ("ReadEncoder    (0x90)", 0x90u8),
        ("ReadMultiTurn  (0x92)", 0x92u8),
        ("ReadSingleTurn (0x94)", 0x94u8),
    ] {
        let _ = motor.flush_rx();
        match motor.transact(cmd, id, &[]) {
            Ok(resp) => dump(name, &resp.data),
            Err(e) => println!("{name:<24}ERR: {e}"),
        }
    }

    Ok(())
}
