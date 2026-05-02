//! Send `MotorRun` (0x88) to try to wake a controller that has gone silent
//! after `MotorOff` (0x80). The send is fire-and-forget; afterwards we poll
//! State1 a few times to see if it came back up.
//!
//! ```ignore
//! cargo run -p lkmotor-driver --example wake -- --device /dev/ttyUSB0 --baud 1000000 --id 1
//! ```

use std::thread::sleep;
use std::time::Duration;

use clap::Parser;
use lkmotor_driver::protocol::command::Command;
use lkmotor_driver::LkCommands;
use lkmotor_driver::{MotorId, Rs485Driver};

#[derive(Parser, Debug)]
#[command(about = "Try to wake a silent LK Motor V3 controller")]
struct Args {
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    device: String,
    #[arg(short, long, default_value_t = 1_000_000)]
    baud: u32,
    #[arg(short, long, default_value_t = 1)]
    id: u8,
    #[arg(long, default_value_t = 100)]
    timeout_ms: u64,
    #[arg(long, default_value_t = 5)]
    attempts: u32,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let args = Args::parse();
    let id = MotorId::new(args.id).ok_or("motor id must be 1..=32")?;
    let mut motor = Rs485Driver::open(&args.device, args.baud, Duration::from_millis(args.timeout_ms))?;

    for i in 1..=args.attempts {
        let _ = motor.flush_rx();
        // Fire motor_run; ignore the response (the controller may not reply yet).
        let _ = motor.send_raw(Command::MotorRun.code(), id, &[]);
        sleep(Duration::from_millis(50));
        match motor.read_state1(id) {
            Ok(s) => {
                println!(
                    "attempt {i}: WOKE — temp={}°C voltage={:.1}V error=0x{:02X}",
                    s.temperature_c,
                    s.voltage_v(),
                    s.error_state
                );
                return Ok(());
            }
            Err(e) => println!("attempt {i}: still silent ({e})"),
        }
        sleep(Duration::from_millis(150));
    }
    Err("motor did not respond to MotorRun; a power cycle is likely required".into())
}
