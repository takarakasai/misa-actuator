//! Ping servo ids `0..=max-id` and list the ones that answer.
//!
//! Read-only: PING changes nothing on the servo.
//!
//! ```text
//! cargo run -p fashionstar-driver --example scan -- --port /dev/ttyUSB0 --max-id 7
//! ```

use std::time::Duration;

use clap::Parser;
use fashionstar_driver::{default_serial_port, FashionStarBus, FsCommands, DEFAULT_BAUD};

#[derive(Parser)]
struct Args {
    #[arg(long, default_value_t = default_serial_port().to_string())]
    port: String,
    #[arg(long, default_value_t = DEFAULT_BAUD)]
    baud: u32,
    /// Highest id to try (the Star Arm 102 LD uses 0..=6; 7 is the HD
    /// variant's button board).
    #[arg(long, default_value_t = 7)]
    max_id: u8,
    /// Per-id reply deadline.
    #[arg(long, default_value_t = 10)]
    timeout_ms: u64,
    /// Tries per id before calling it absent — one lost frame should not
    /// read as a missing servo.
    #[arg(long, default_value_t = 2)]
    attempts: u32,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let a = Args::parse();
    let mut bus = FashionStarBus::open(&a.port, a.baud, Duration::from_millis(a.timeout_ms))?;
    println!("scanning ids 0..={} on {} @ {}", a.max_id, a.port, a.baud);
    let mut found = Vec::new();
    for id in 0..=a.max_id {
        let mut hit = false;
        for _ in 0..a.attempts.max(1) {
            if bus.ping(id)? {
                hit = true;
                break;
            }
        }
        if hit {
            // Also show where it is — useful to confirm which joint is which.
            match bus.monitor(id) {
                Ok(m) => println!(
                    "  id {id:3}: present  angle {:8.1}°  {:5.2} V  {:5.1} °C  status 0x{:02X}",
                    m.angle_deg(),
                    m.voltage_v,
                    m.temperature_c,
                    m.status
                ),
                Err(e) => println!("  id {id:3}: present  (monitor failed: {e})"),
            }
            found.push(id);
        }
    }
    println!("found {} servo(s): {found:?}", found.len());
    Ok(())
}

