//! Spin the motor until the OUTPUT shaft has rotated a target number of
//! revolutions, judged from raw encoder delta accumulation (wrap-corrected).
//!
//! The single-turn encoder reading lives on the motor shaft and wraps at
//! 65536 (see `lkmotor_protocol::response::ENCODER_PERIOD`), so
//! output rotation = accumulated_motor_counts / (gear_ratio * 65536).
//!
//! ```ignore
//! cargo run -p lkmotor-driver --example spin_revs -- \
//!     --device /dev/ttyUSB0 --baud 1000000 --id 1 \
//!     --gear-ratio 10 --output-revs 1 --speed-dps 60
//! ```

use std::thread::sleep;
use std::time::{Duration, Instant};

use clap::Parser;
use lkmotor_driver::protocol::response::ENCODER_PERIOD;
use lkmotor_driver::LkCommands;
use lkmotor_driver::{MotorId, Rs485Driver};

const ENC_FULL: i32 = ENCODER_PERIOD as i32;
const ENC_HALF: i32 = ENC_FULL / 2;

#[derive(Parser, Debug)]
#[command(about = "Spin until the output shaft rotates N revolutions (encoder-judged)")]
struct Args {
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    device: String,
    #[arg(short, long, default_value_t = 1_000_000)]
    baud: u32,
    #[arg(short, long, default_value_t = 1)]
    id: u8,
    /// Motor revolutions per output revolution.
    #[arg(long, default_value_t = 10)]
    gear_ratio: u32,
    /// How many full output-shaft revolutions to perform (signed).
    #[arg(long, default_value_t = 1.0)]
    output_revs: f64,
    /// Target speed in deg/s passed to speed_control (sign matched to output_revs).
    #[arg(long, default_value_t = 60)]
    speed_dps: i32,
    /// Encoder polling interval, in ms.
    #[arg(long, default_value_t = 50)]
    sample_ms: u64,
    /// Per-request response timeout, in ms.
    #[arg(long, default_value_t = 100)]
    timeout_ms: u64,
    /// Hard timeout: bail and stop the motor if the goal isn't reached, in seconds.
    #[arg(long, default_value_t = 60)]
    max_seconds: u64,
}

/// Drop-guard that releases torque on panic / early return. Only sends
/// `motor_stop` (0x81); see `spin.rs` for why we avoid `motor_off` (0x80).
struct MotorGuard<'a> {
    motor: &'a mut Rs485Driver,
    id: MotorId,
}

impl<'a> Drop for MotorGuard<'a> {
    fn drop(&mut self) {
        if let Err(e) = self.motor.motor_stop(self.id) {
            eprintln!("warn: motor_stop on shutdown failed: {e}");
        }
    }
}

/// Wrap-corrected delta between two single-turn encoder readings.
///
/// Picks the candidate with the smallest absolute value among
/// `raw`, `raw + ENC_FULL`, `raw - ENC_FULL`. Correct as long as the actual
/// motion between samples is less than half a motor revolution.
fn wrapped_delta(prev: u16, curr: u16) -> i32 {
    let raw = curr as i32 - prev as i32;
    if raw > ENC_HALF {
        raw - ENC_FULL
    } else if raw < -ENC_HALF {
        raw + ENC_FULL
    } else {
        raw
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let args = Args::parse();

    if args.speed_dps.abs() > 720 {
        return Err(format!(
            "refusing |speed_dps|={} > 720; raise the limit in code if intentional",
            args.speed_dps.abs()
        )
        .into());
    }
    if args.gear_ratio == 0 {
        return Err("gear_ratio must be >= 1".into());
    }

    let id = MotorId::new(args.id).ok_or("motor id must be 1..=32")?;
    let timeout = Duration::from_millis(args.timeout_ms);
    let mut motor = Rs485Driver::open(&args.device, args.baud, timeout)?;

    let direction = if args.output_revs >= 0.0 { 1 } else { -1 };
    let target_motor_counts =
        (args.output_revs.abs() * args.gear_ratio as f64 * ENC_FULL as f64).round() as i64;
    let signed_speed = args.speed_dps.abs() * direction;
    let centideg_per_s = signed_speed.saturating_mul(100);

    println!("== baseline ==");
    let s2 = motor.read_state2(id)?;
    println!(
        "  enc={} ({:.1}%)  speed={} deg/s",
        s2.encoder_raw,
        s2.encoder_fraction() * 100.0,
        s2.speed_deg_per_s
    );
    println!(
        "target: {:+.3} output revs = {:+} motor counts at {} deg/s",
        args.output_revs, target_motor_counts as i64 * direction as i64, signed_speed
    );

    motor.motor_run(id)?;

    let mut prev_enc = s2.encoder_raw;
    let mut accumulated: i64 = 0;
    let started = Instant::now();
    let sample = Duration::from_millis(args.sample_ms);
    let hard_deadline = started + Duration::from_secs(args.max_seconds);

    {
        let guard = MotorGuard { motor: &mut motor, id };
        guard.motor.speed_control(id, centideg_per_s)?;

        loop {
            sleep(sample);
            let s = match guard.motor.read_state2(id) {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("read_state2 failed mid-spin: {e}");
                    break;
                }
            };
            let d = wrapped_delta(prev_enc, s.encoder_raw);
            accumulated += d as i64;
            prev_enc = s.encoder_raw;

            let elapsed_ms = started.elapsed().as_millis();
            let output_deg = (accumulated as f64) * 360.0
                / (args.gear_ratio as f64 * ENC_FULL as f64);
            println!(
                "  t={:>5} ms  enc={:>5}  Δacc={:+7}  output={:+7.2}°  iq={:+.2}A  speed={:+5}",
                elapsed_ms, s.encoder_raw, accumulated, output_deg, s.current_amps(), s.speed_deg_per_s
            );

            if accumulated.abs() >= target_motor_counts {
                println!("-- target reached --");
                break;
            }
            if Instant::now() >= hard_deadline {
                eprintln!("-- hard timeout ({} s) before reaching target --", args.max_seconds);
                break;
            }
        }
    } // guard's Drop sends motor_stop + motor_off

    let final_state = motor.read_state2(id)?;
    let final_output_deg = (accumulated as f64) * 360.0
        / (args.gear_ratio as f64 * ENC_FULL as f64);
    println!(
        "== final ==  accumulated={} motor counts ({:+.2}° output)  enc={}  elapsed={:.2}s",
        accumulated,
        final_output_deg,
        final_state.encoder_raw,
        started.elapsed().as_secs_f64()
    );

    Ok(())
}
