//! Read all of the V2.36 control parameters via `0xC0` and print them.
//!
//! Read-only, no side effects on the motor — safe to run on a powered bench.
//!
//! ```ignore
//! cargo run -p lkmotor-driver --example dump_params -- \
//!     --device /dev/ttyUSB0 --baud 1000000 --id 1
//! ```

use std::time::Duration;

use clap::Parser;
use lkmotor_driver::{MotorId, Rs485Driver};
use lkmotor_driver::LkCommands;
use lkmotor_driver::protocol::ControlParamId;
use lkmotor_driver::protocol::response::ControlParamValue;

#[derive(Parser, Debug)]
#[command(about = "Dump all readable control parameters (0xC0) for one motor")]
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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let args = Args::parse();

    let id = MotorId::new(args.id).ok_or("motor id must be 1..=32")?;
    let mut motor = Rs485Driver::open(
        &args.device,
        args.baud,
        Duration::from_millis(args.timeout_ms),
    )?;

    let params = [
        ("position_loop_pid", ControlParamId::PositionLoopPid),
        ("speed_loop_pid",    ControlParamId::SpeedLoopPid),
        ("current_loop_pid",  ControlParamId::CurrentLoopPid),
        ("torque_limit",      ControlParamId::TorqueLimit),
        ("speed_limit",       ControlParamId::SpeedLimit),
        ("angle_limit",       ControlParamId::AngleLimit),
        ("current_ramp",      ControlParamId::CurrentRamp),
        ("speed_ramp",        ControlParamId::SpeedRamp),
    ];

    println!("== motor id={} ==", args.id);
    for (label, id_) in params {
        match motor.read_control_param(id, id_) {
            Ok(v) => print_value(label, v),
            Err(e) => println!("  {label:<18} -> ERR {e}"),
        }
    }

    Ok(())
}

fn print_value(label: &str, v: ControlParamValue) {
    match v {
        ControlParamValue::Pid(p) => println!(
            "  {label:<18} kp={:>5}  ki={:>5}  kd={:>5}",
            p.kp, p.ki, p.kd
        ),
        ControlParamValue::TorqueLimit(x) => {
            println!("  {label:<18} {x} (raw int16)")
        }
        ControlParamValue::SpeedLimit(x) => {
            println!(
                "  {label:<18} {x} (0.01 dps)  = {:.2} dps",
                x as f32 * 0.01
            )
        }
        ControlParamValue::AngleLimit(x) => {
            println!(
                "  {label:<18} {x} (0.01 deg) = {:.2} deg",
                x as f32 * 0.01
            )
        }
        ControlParamValue::CurrentRamp(x) => {
            println!("  {label:<18} {x}")
        }
        ControlParamValue::SpeedRamp(x) => {
            println!("  {label:<18} {x} dps/s")
        }
    }
}
