//! Sweep the RS485 bus for any responding LK Motor V3 servo by polling
//! `ReadMotorState1` (0x9A) at every legal motor ID (1..=32).
//!
//! Useful when the motor ID has been forgotten or the firmware came preset
//! to a non-default address. `ReadMotorState1` is a pure read, so it is
//! safe to broadcast across the entire address space.
//!
//! ```ignore
//! cargo run -p lkmotor-driver --example scan -- --device /dev/ttyUSB0 --baud 115200
//! ```

use std::time::Duration;

use clap::Parser;
use lkmotor_driver::{MotorId, Rs485Driver};
use lkmotor_driver::LkCommands;

#[derive(Parser, Debug)]
#[command(about = "Scan the RS485 bus for any responding LK Motor V3 servo")]
struct Args {
    /// Serial device path (e.g., /dev/ttyUSB0).
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    device: String,

    /// Serial baud rate.
    #[arg(short, long, default_value_t = 115_200)]
    baud: u32,

    /// Per-ID response timeout, in milliseconds.
    #[arg(long, default_value_t = 50)]
    timeout_ms: u64,

    /// Lowest motor ID to probe (inclusive, 1..=32).
    #[arg(long, default_value_t = 1)]
    from: u8,

    /// Highest motor ID to probe (inclusive, 1..=32).
    #[arg(long, default_value_t = 32)]
    to: u8,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args = Args::parse();
    let timeout = Duration::from_millis(args.timeout_ms);

    let mut motor = Rs485Driver::open(&args.device, args.baud, timeout)?;

    let lo = args.from.max(1);
    let hi = args.to.min(MotorId::MAX);
    if lo > hi {
        return Err(format!("invalid range: {lo}..={hi}").into());
    }

    println!(
        "scanning {} @ {} baud, IDs {}..={}, {} ms/ID",
        args.device, args.baud, lo, hi, args.timeout_ms
    );

    let mut found = Vec::new();
    for id in lo..=hi {
        let motor_id = MotorId::new(id).expect("range checked above");
        let _ = motor.flush_rx();
        match motor.read_state1(motor_id) {
            Ok(s) => {
                println!(
                    "  id={:>2} OK  temp={}°C voltage={:.1}V error=0x{:02X}",
                    id,
                    s.temperature_c,
                    s.voltage_v(),
                    s.error_state
                );
                found.push(id);
            }
            Err(e) => {
                println!("  id={:>2} --  {e}", id);
            }
        }
    }

    println!();
    if found.is_empty() {
        println!("no motors responded on {}..={}", lo, hi);
    } else {
        println!("responding IDs: {:?}", found);
    }

    Ok(())
}
