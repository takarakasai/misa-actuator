//! MIT-mode emulation demo: hold a target output-frame angle with PD + feed-forward.
//!
//! Each cycle reads State2 (`0x9C`), computes
//! `i = Kp·(p_d − p) + Kd·(v_d − v) + i_ff` on the host, and sends
//! TorqueClosedLoop (`0xA1`).
//!
//! Built on the SI-unit [`Motor`](lkmotor_driver::Motor) API, configured with
//! [`MotorConfig::current_units`] so that `kp` / `kd` / `i_ff` keep their
//! amperes-based units (Kt of the bench V2 motor is unknown).
//!
//! ```ignore
//! cargo run -p lkmotor-driver --example mit_hold -- \
//!     --device /dev/ttyUSB0 --baud 1000000 --id 1 \
//!     --gear-ratio 10.0 --kp 0.6 --kd 0.02 \
//!     --target-deg 0 --duration-ms 5000 --period-ms 5
//! ```

use std::thread::sleep;
use std::time::{Duration, Instant};

use clap::Parser;
use lkmotor_driver::{Motor, MotorConfig, MotorId, Rs485Driver};
use lkmotor_driver::LkCommands;

#[derive(Parser, Debug)]
#[command(about = "MIT-mode emulation hold demo (case B: 0x9C + 0xA1)")]
struct Args {
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    device: String,
    #[arg(short, long, default_value_t = 1_000_000)]
    baud: u32,
    #[arg(short, long, default_value_t = 1)]
    id: u8,
    /// Motor-shaft revs per output-shaft rev (e.g. 10.0 for 1:10 gearbox).
    #[arg(long, default_value_t = 1.0)]
    gear_ratio: f32,
    /// Kp in A/rad (output-frame). With current_units config, the SI
    /// `kp_nm_per_rad` argument carries A/rad directly.
    #[arg(long, default_value_t = 0.5)]
    kp: f32,
    /// Kd in A/(rad/s) (output-frame).
    #[arg(long, default_value_t = 0.02)]
    kd: f32,
    /// Feed-forward current in amps.
    #[arg(long, default_value_t = 0.0)]
    tau_ff_a: f32,
    /// Target hold angle in **output-frame degrees** (relative to rezero).
    #[arg(long, default_value_t = 0.0)]
    target_deg: f32,
    /// Total run duration in ms.
    #[arg(long, default_value_t = 5_000)]
    duration_ms: u64,
    /// Cycle period in ms (each cycle = 1 read + 1 write).
    #[arg(long, default_value_t = 5)]
    period_ms: u64,
    /// Print every Nth tick.
    #[arg(long, default_value_t = 20)]
    log_every: u32,
    /// Per-request response timeout in ms.
    #[arg(long, default_value_t = 50)]
    timeout_ms: u64,
    /// Hard cap on commanded current (A) to keep the bench safe.
    #[arg(long, default_value_t = 3.0)]
    current_limit_a: f32,
}

struct StopGuard<'a> {
    bus: &'a mut Rs485Driver,
    id: MotorId,
}

impl<'a> Drop for StopGuard<'a> {
    fn drop(&mut self) {
        // Send a 0 A command and then motor_stop. Avoid 0x80 (hangs the bus on
        // bench firmware).
        let _ = self.bus.torque_control(self.id, 0.0);
        if let Err(e) = self.bus.motor_stop(self.id) {
            eprintln!("warn: motor_stop on shutdown failed: {e}");
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let args = Args::parse();

    let id = MotorId::new(args.id).ok_or("motor id must be 1..=32")?;
    let timeout = Duration::from_millis(args.timeout_ms);
    let mut bus = Rs485Driver::open(&args.device, args.baud, timeout)?;

    bus.motor_run(id)?;

    let target_rad = args.target_deg.to_radians();
    let period = Duration::from_millis(args.period_ms);
    let total = Duration::from_millis(args.duration_ms);

    let mut motor = Motor::new(id, MotorConfig::current_units(args.gear_ratio));

    let guard = StopGuard {
        bus: &mut bus,
        id,
    };

    println!(
        "rezero @ current encoder; target={:.3} rad ({:.2} deg, output)  Kp={} A/rad  Kd={} A/(rad/s)  i_ff={} A",
        target_rad, args.target_deg, args.kp, args.kd, args.tau_ff_a
    );
    motor.rezero(guard.bus)?;

    let start = Instant::now();
    let mut tick: u32 = 0;
    while start.elapsed() < total {
        let loop_start = Instant::now();

        let m = motor.measure(guard.bus)?;
        let ff = args
            .tau_ff_a
            .clamp(-args.current_limit_a, args.current_limit_a);
        let cmd = args.kp * (target_rad - m.position_rad)
            + args.kd * (0.0 - m.velocity_rad_per_s)
            + ff;
        motor.set_current(guard.bus, cmd)?;

        if cmd.abs() > args.current_limit_a {
            // Saturated above safety limit — clamp on next tick by lowering kp/kd
            // or raising current_limit_a. We just warn here.
            eprintln!(
                "warn: command {:.2} A exceeded current_limit {:.2} A",
                cmd, args.current_limit_a
            );
        }

        if tick.is_multiple_of(args.log_every) {
            println!(
                "t={:>5} ms  pos={:+7.3} rad ({:+7.2}°)  vel={:+7.3} rad/s  i_meas={:+5.2} A  i_cmd={:+5.2} A  T={}°C",
                start.elapsed().as_millis(),
                m.position_rad,
                m.position_rad.to_degrees(),
                m.velocity_rad_per_s,
                m.current_a,
                cmd,
                m.temperature_c,
            );
        }
        tick += 1;

        let elapsed = loop_start.elapsed();
        if elapsed < period {
            sleep(period - elapsed);
        }
    }

    drop(guard);
    println!("done — {} ticks", tick);
    Ok(())
}
