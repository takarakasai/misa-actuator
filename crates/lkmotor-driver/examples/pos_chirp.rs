//! Position-mode (`0xA4`) chirp stability test.
//!
//! Same chirp signal as `mit_chirp`, but instead of computing torque on the
//! host, every tick is a single `Motor::set_position` (`0xA4` write) — the
//! reply payload itself carries State2, so each tick is **1 RS485 transaction**
//! (vs. 2 for MIT-mode), letting the bench run roughly 2× faster.
//!
//! What this characterises: the **motor's internal position-PID closed loop**
//! (you can't change the on-board PID via RS485 without `0xC1`, which V2
//! firmware does not implement — see lkmotor_bench memory).
//!
//!   target_pos(t) = A · sin(φ(t))
//!   φ(t)         = 2π · ( f0·t + ½·(f1−f0)/T · t² )
//!   f(t)         = f0 + (f1 − f0) · t / T
//!
//! Built on the SI-unit [`Motor`](lkmotor_driver::Motor) API; gear ratio,
//! turn tracking, and absolute-zero anchoring are handled internally.
//!
//! ```ignore
//! cargo run -p lkmotor-driver --example pos_chirp -- \
//!     --device /dev/ttyUSB0 --baud 1000000 --id 1 \
//!     --gear-ratio 10.0 \
//!     --amplitude-deg 5 --f-start 0.5 --f-end 5 --duration-s 10 \
//!     --period-ms 5 --csv analyze/2026-05-03/pos_chirp.csv
//! ```

use std::fs::File;
use std::io::{BufWriter, Write as _};
use std::thread::sleep;
use std::time::{Duration, Instant};

use clap::Parser;
use lkmotor_driver::{Motor, MotorConfig, MotorId, Rs485Driver};
use lkmotor_driver::LkCommands;

#[derive(Parser, Debug)]
#[command(about = "Position-mode (0xA4) chirp stability test — exercises the motor's internal position PID")]
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
    /// Chirp amplitude in **output-frame degrees** (peak, not peak-to-peak).
    #[arg(long, default_value_t = 5.0)]
    amplitude_deg: f32,
    /// Chirp start frequency in Hz.
    #[arg(long, default_value_t = 0.5)]
    f_start: f32,
    /// Chirp end frequency in Hz.
    #[arg(long, default_value_t = 5.0)]
    f_end: f32,
    /// Sweep duration in seconds.
    #[arg(long, default_value_t = 10.0)]
    duration_s: f32,
    /// Cycle period in ms.
    #[arg(long, default_value_t = 5)]
    period_ms: u64,
    /// Per-request response timeout in ms.
    #[arg(long, default_value_t = 50)]
    timeout_ms: u64,
    /// Headroom factor on max-speed cap (analytical peak vel × this multiplier).
    #[arg(long, default_value_t = 4.0)]
    max_speed_headroom: f32,
    /// Hard cap on |measured iq| (A); aborts if exceeded.
    #[arg(long, default_value_t = 10.0)]
    current_abort_a: f32,
    /// Print every Nth tick to stdout (CSV always logs every tick).
    #[arg(long, default_value_t = 50)]
    log_every: u32,
    /// CSV output path. Parent directory is created if missing.
    /// Convention: `analyze/YYYY-MM-DD/<run_name>.csv`.
    #[arg(long)]
    csv: Option<String>,
}

struct StopGuard<'a> {
    bus: &'a mut Rs485Driver,
    id: MotorId,
}

impl<'a> Drop for StopGuard<'a> {
    fn drop(&mut self) {
        if let Err(e) = self.bus.motor_stop(self.id) {
            eprintln!("warn: motor_stop on shutdown failed: {e}");
        }
    }
}

/// Linear chirp position + analytical velocity. Returns `(pos_rad, vel_rad_s, freq_hz)`.
fn chirp(t: f32, t_total: f32, f0: f32, f1: f32, amplitude_rad: f32) -> (f32, f32, f32) {
    let k = (f1 - f0) / t_total;
    let f = f0 + k * t;
    let phase = 2.0 * std::f32::consts::PI * (f0 * t + 0.5 * k * t * t);
    let pos = amplitude_rad * phase.sin();
    let vel = amplitude_rad * 2.0 * std::f32::consts::PI * f * phase.cos();
    (pos, vel, f)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let args = Args::parse();

    if args.f_end <= args.f_start {
        return Err("f_end must be greater than f_start".into());
    }
    if args.duration_s <= 0.0 {
        return Err("duration_s must be positive".into());
    }
    let cycle_hz = 1000.0 / args.period_ms as f32;
    if cycle_hz < 2.0 * args.f_end {
        return Err(format!(
            "cycle rate {:.1} Hz cannot resolve f_end={} Hz (need ≥ 2× Nyquist; ≥ 10× recommended)",
            cycle_hz, args.f_end
        )
        .into());
    }

    let id = MotorId::new(args.id).ok_or("motor id must be 1..=32")?;
    let timeout = Duration::from_millis(args.timeout_ms);
    let mut bus = Rs485Driver::open(&args.device, args.baud, timeout)?;

    let amplitude_rad = args.amplitude_deg.to_radians();
    let total = Duration::from_secs_f32(args.duration_s);
    let period = Duration::from_millis(args.period_ms);

    // Output-frame analytical peak velocity, plus headroom, drives the
    // `max_speed` cap fed to set_position. SI throughout — Motor handles the
    // motor-frame conversion internally.
    let peak_vel_rad_s = amplitude_rad * std::f32::consts::TAU * args.f_end;
    let max_speed_rad_s = (peak_vel_rad_s * args.max_speed_headroom).max(0.01);

    let mut motor = Motor::new(id, MotorConfig::current_units(args.gear_ratio));
    motor.enable(&mut bus)?;

    let mut csv_writer = match &args.csv {
        Some(path) => {
            if let Some(parent) = std::path::Path::new(path).parent() {
                if !parent.as_os_str().is_empty() {
                    std::fs::create_dir_all(parent)?;
                }
            }
            let f = File::create(path)?;
            let mut w = BufWriter::new(f);
            writeln!(
                w,
                "t_s,freq_hz,target_pos_rad,target_vel_rad_s,meas_pos_rad,meas_vel_rad_s,meas_current_a,cycle_ms"
            )?;
            Some(w)
        }
        None => None,
    };

    println!(
        "pos_chirp: A={:.2}° (out)  f={}→{} Hz over {}s   cycle={} ms ({:.0} Hz)",
        args.amplitude_deg, args.f_start, args.f_end, args.duration_s, args.period_ms, cycle_hz
    );
    println!(
        "  max_speed (output) = {:.3} rad/s ({:.1}°/s)  current_abort = {:.2} A",
        max_speed_rad_s,
        max_speed_rad_s.to_degrees(),
        args.current_abort_a
    );

    let guard = StopGuard {
        bus: &mut bus,
        id,
    };

    // rezero anchors both the encoder turn counter and the absolute multi-turn
    // angle origin (via 0x92) — needed before set_position.
    motor.rezero(guard.bus)?;

    let mut peak_err_rad: f32 = 0.0;
    let mut peak_meas_current: f32 = 0.0;
    let mut overrun_count: u32 = 0;
    let mut tick: u32 = 0;
    let mut aborted: Option<String> = None;

    let start = Instant::now();
    while start.elapsed() < total {
        let loop_start = Instant::now();
        let t = start.elapsed().as_secs_f32();

        let (target_pos_rad, target_vel_rad_s, freq) =
            chirp(t, args.duration_s, args.f_start, args.f_end, amplitude_rad);

        // Single transaction: 0xA4 reply carries State2, parsed into MotorFeedback.
        let fb = motor.set_position(guard.bus, target_pos_rad, max_speed_rad_s)?;

        let err = (target_pos_rad - fb.position_rad).abs();
        if err > peak_err_rad {
            peak_err_rad = err;
        }
        if fb.current_a.abs() > peak_meas_current {
            peak_meas_current = fb.current_a.abs();
        }

        if fb.current_a.abs() > args.current_abort_a {
            aborted = Some(format!(
                "measured |iq|={:.2} A exceeded current_abort {:.2} A at t={:.3}s (f={:.2} Hz)",
                fb.current_a, args.current_abort_a, t, freq
            ));
            break;
        }

        let cycle_ms = loop_start.elapsed().as_secs_f32() * 1000.0;
        if cycle_ms > args.period_ms as f32 {
            overrun_count += 1;
        }

        if let Some(w) = csv_writer.as_mut() {
            writeln!(
                w,
                "{:.4},{:.4},{:.6},{:.6},{:.6},{:.6},{:.4},{:.3}",
                t,
                freq,
                target_pos_rad,
                target_vel_rad_s,
                fb.position_rad,
                fb.velocity_rad_per_s,
                fb.current_a,
                cycle_ms
            )?;
        }

        if tick.is_multiple_of(args.log_every) {
            println!(
                "t={:>6.3}s  f={:>5.2} Hz  tgt={:+6.3}rad  meas={:+6.3}rad  err={:+5.3}rad  i={:+5.2}A  cyc={:.2}ms",
                t, freq, target_pos_rad, fb.position_rad, target_pos_rad - fb.position_rad, fb.current_a, cycle_ms,
            );
        }
        tick += 1;

        let elapsed = loop_start.elapsed();
        if elapsed < period {
            sleep(period - elapsed);
        }
    }

    drop(guard);
    if let Some(mut w) = csv_writer {
        w.flush()?;
    }

    println!("\n== summary ==");
    println!("  ticks               : {}", tick);
    if tick > 0 {
        println!(
            "  peak tracking error : {:.4} rad ({:.3}°)",
            peak_err_rad,
            peak_err_rad.to_degrees()
        );
        println!("  peak measured |iq|  : {:.2} A", peak_meas_current);
        println!(
            "  cycle overruns      : {} / {} ({:.1}%)",
            overrun_count,
            tick,
            100.0 * overrun_count as f32 / tick as f32
        );
    }
    if let Some(reason) = aborted {
        println!("  ABORTED: {reason}");
    }

    Ok(())
}
