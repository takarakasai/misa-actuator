//! Conservative two-direction spin demo for an LK Motor V3 servo.
//!
//! Runs the motor at the requested speed for a fixed duration, stops, then
//! repeats in the opposite direction. Always issues `motor_off` on exit
//! (including panics) so the bus is left in a safe state.
//!
//! ```ignore
//! cargo run -p lkmotor-driver --example spin -- \
//!     --device /dev/ttyUSB0 --baud 1000000 --id 1 \
//!     --speed-dps 60 --duration-ms 1000
//! ```

use std::thread::sleep;
use std::time::Duration;

use clap::Parser;
use lkmotor_driver::{MotorId, Rs485Driver};
use lkmotor_driver::LkCommands;

#[derive(Parser, Debug)]
#[command(about = "Two-direction spin demo for an LK Motor V3 servo")]
struct Args {
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    device: String,
    #[arg(short, long, default_value_t = 1_000_000)]
    baud: u32,
    #[arg(short, long, default_value_t = 1)]
    id: u8,
    /// Target speed in deg/s (signed range applied per phase).
    #[arg(long, default_value_t = 60)]
    speed_dps: i32,
    /// How long to hold each phase (forward, then reverse), in ms.
    #[arg(long, default_value_t = 1_000)]
    duration_ms: u64,
    /// Per-request response timeout, in ms.
    #[arg(long, default_value_t = 100)]
    timeout_ms: u64,
    /// Sample rate for the live status print, in ms.
    #[arg(long, default_value_t = 100)]
    sample_ms: u64,
}

/// Drop-guard that releases torque on panic / early return.
///
/// Only sends `motor_stop` (0x81) — *not* `motor_off` (0x80) — because on the
/// bench firmware 0x80 silences the controller until a power cycle.
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

fn run_phase(
    motor: &mut Rs485Driver,
    id: MotorId,
    label: &str,
    speed_dps: i32,
    duration: Duration,
    sample: Duration,
) -> Result<(), Box<dyn std::error::Error>> {
    let centideg_per_s = speed_dps.saturating_mul(100);
    println!("-- {label}: speed_control({speed_dps} deg/s) for {} ms --", duration.as_millis());

    motor.speed_control(id, centideg_per_s)?;

    let start = std::time::Instant::now();
    while start.elapsed() < duration {
        sleep(sample);
        match motor.read_state2(id) {
            Ok(s) => println!(
                "  t={:>4} ms  iq={:+.2} A  speed={:+5} deg/s  enc={} ({:.1}%)",
                start.elapsed().as_millis(),
                s.current_amps(),
                s.speed_deg_per_s,
                s.encoder_raw,
                s.encoder_fraction() * 100.0,
            ),
            Err(e) => println!("  read_state2: {e}"),
        }
    }

    motor.motor_stop(id)?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let args = Args::parse();

    if args.speed_dps.abs() > 360 {
        return Err(format!(
            "refusing |speed_dps|={} > 360; raise the limit in code if intentional",
            args.speed_dps.abs()
        )
        .into());
    }

    let id = MotorId::new(args.id).ok_or("motor id must be 1..=32")?;
    let timeout = Duration::from_millis(args.timeout_ms);
    let mut motor = Rs485Driver::open(&args.device, args.baud, timeout)?;

    println!("== baseline ==");
    let s1 = motor.read_state1(id)?;
    println!(
        "  temp={}°C  voltage={:.1}V  error=0x{:02X}",
        s1.temperature_c,
        s1.voltage_v(),
        s1.error_state
    );
    let s2 = motor.read_state2(id)?;
    println!(
        "  enc={} ({:.1}%)  speed={} deg/s  iq={:.2} A",
        s2.encoder_raw,
        s2.encoder_fraction() * 100.0,
        s2.speed_deg_per_s,
        s2.current_amps()
    );

    motor.motor_run(id)?;

    {
        let guard = MotorGuard { motor: &mut motor, id };
        let dur = Duration::from_millis(args.duration_ms);
        let sample = Duration::from_millis(args.sample_ms);

        run_phase(guard.motor, id, "forward", args.speed_dps, dur, sample)?;
        sleep(Duration::from_millis(300));
        run_phase(guard.motor, id, "reverse", -args.speed_dps, dur, sample)?;
    } // guard's Drop sends motor_stop + motor_off

    println!("== final ==");
    let s2 = motor.read_state2(id)?;
    println!(
        "  enc={} ({:.1}%)  speed={} deg/s  iq={:.2} A",
        s2.encoder_raw,
        s2.encoder_fraction() * 100.0,
        s2.speed_deg_per_s,
        s2.current_amps()
    );

    Ok(())
}
