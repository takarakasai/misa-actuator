//! Probe a single LK Motor V3 servo over RS485 and print its state.
//!
//! ```ignore
//! cargo run -p lkmotor-driver --example probe -- --device /dev/ttyUSB0 --id 1
//! ```

use std::time::Duration;

use clap::Parser;
use lkmotor_driver::{MotorId, Rs485Driver};
use lkmotor_driver::LkCommands;

#[derive(Parser, Debug)]
#[command(about = "Probe an LK Motor V3 servo on RS485")]
struct Args {
    /// Serial device path (e.g., /dev/ttyUSB0).
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    device: String,

    /// Serial baud rate.
    #[arg(short, long, default_value_t = 115_200)]
    baud: u32,

    /// Motor ID on the bus (1..=32).
    #[arg(short, long, default_value_t = 1)]
    id: u8,

    /// Per-request response timeout, in milliseconds.
    #[arg(long, default_value_t = 100)]
    timeout_ms: u64,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args = Args::parse();
    let motor_id = MotorId::new(args.id).ok_or("motor id must be 1..=32")?;
    let timeout = Duration::from_millis(args.timeout_ms);

    let mut motor = Rs485Driver::open(&args.device, args.baud, timeout)?;

    println!("== state 1 ==");
    match motor.read_state1(motor_id) {
        Ok(s) => println!(
            "  temp={}°C  voltage={:.1}V  error=0x{:02X}",
            s.temperature_c,
            s.voltage_v(),
            s.error_state
        ),
        Err(e) => println!("  failed: {e}"),
    }

    println!("== state 2 ==");
    match motor.read_state2(motor_id) {
        Ok(s) => println!(
            "  temp={}°C  iq={:.2}A  speed={} deg/s  encoder={} ({:.1}%)",
            s.temperature_c,
            s.current_amps(),
            s.speed_deg_per_s,
            s.encoder_raw,
            s.encoder_fraction() * 100.0,
        ),
        Err(e) => println!("  failed: {e}"),
    }

    Ok(())
}
