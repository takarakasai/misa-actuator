//! Diagnostic probe for `0xC0` (Read control parameter).
//!
//! For each ParamID we want to read:
//!   1. Pre-flight: send `0x9C` (read state2) — known good, confirms the bus + ID work.
//!   2. Send the `0xC0` request and HEX-DUMP every byte that comes back over a
//!      configurable window (regardless of whether it parses as a valid frame).
//!   3. Compare the response against the expected `0xC0` echo.
//!
//! Use this when `dump_params` times out — the raw bytes tell us whether the
//! motor responded at all, replied with a different command code, returned a
//! malformed frame, or simply didn't answer.
//!
//! ```ignore
//! cargo run -p lkmotor-driver --example param_probe -- \
//!     --device /dev/ttyUSB0 --baud 1000000 --id 1
//! ```

use std::time::Duration;

use clap::Parser;
use lkmotor_driver::protocol::ControlParamId;
use lkmotor_driver::LkCommands;
use lkmotor_driver::protocol::frame::{MAX_FRAME, encode};
use lkmotor_driver::{MotorId, Rs485Driver};

#[derive(Parser, Debug)]
#[command(about = "Raw-byte probe for the 0xC0 control parameter read")]
struct Args {
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    device: String,
    #[arg(short, long, default_value_t = 1_000_000)]
    baud: u32,
    #[arg(short, long, default_value_t = 1)]
    id: u8,
    /// How long to wait for response bytes after each request, in ms.
    #[arg(long, default_value_t = 200)]
    window_ms: u64,
    /// Timeout for the pre-flight 0x9C check, in ms.
    #[arg(long, default_value_t = 100)]
    preflight_timeout_ms: u64,
}

fn hex(label: &str, bytes: &[u8]) {
    print!("    {label:<10} [{:>2} bytes]", bytes.len());
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

fn build_read_param(motor_id: u8, param: ControlParamId) -> Vec<u8> {
    let mut out = [0u8; MAX_FRAME];
    let mut data = [0u8; 7];
    data[0] = param.code();
    let n = encode(0xC0, motor_id, &data, &mut out).expect("encode");
    out[..n].to_vec()
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
        Ok(s) => println!(
            "  OK — temp={}°C iq={} speed={} dps enc={}",
            s.temperature_c, s.iq_raw, s.speed_deg_per_s, s.encoder_raw
        ),
        Err(e) => {
            println!("  FAIL: {e}");
            println!("  bus / id / baud are the problem, not 0xC0. abort.");
            return Ok(());
        }
    }

    let params = [
        ("PositionLoopPid", ControlParamId::PositionLoopPid),
        ("SpeedLoopPid", ControlParamId::SpeedLoopPid),
        ("CurrentLoopPid", ControlParamId::CurrentLoopPid),
        ("TorqueLimit", ControlParamId::TorqueLimit),
        ("SpeedLimit", ControlParamId::SpeedLimit),
        ("AngleLimit", ControlParamId::AngleLimit),
        ("CurrentRamp", ControlParamId::CurrentRamp),
        ("SpeedRamp", ControlParamId::SpeedRamp),
    ];

    let window = Duration::from_millis(args.window_ms);
    for (name, p) in params {
        println!("\n== {} (0xC0, paramID=0x{:02X}) ==", name, p.code());
        let _ = motor.flush_rx();
        let tx = build_read_param(args.id, p);
        hex("TX", &tx);

        // Use the low-level send_raw + read_raw_for to bypass frame parsing.
        if let Err(e) = motor.send_raw(0xC0, id, &{
            let mut d = [0u8; 7];
            d[0] = p.code();
            d
        }) {
            println!("    send_raw err: {e}");
            continue;
        }

        let rx = motor.read_raw_for(window)?;
        hex("RX", &rx);

        // Quick interpretation hints.
        if rx.is_empty() {
            println!("    -> no response (motor silent or wrong wire format)");
        } else if let Some(idx) = rx.iter().position(|&b| b == 0x3E) {
            let after = &rx[idx..];
            if after.len() >= 4 {
                println!(
                    "    -> first frame: header=0x{:02X} cmd=0x{:02X} id=0x{:02X} len=0x{:02X}",
                    after[0], after[1], after[2], after[3]
                );
            }
        } else {
            println!("    -> response has no 0x3E header byte (junk / framing issue)");
        }
    }

    Ok(())
}
