//! MIT-mode emulation stability test: linear-frequency chirp on the position
//! target, with the analytical velocity derivative supplied as `velocity_target`
//! so both Kp and Kd are exercised actively.
//!
//!   target_pos(t) = A · sin(φ(t))
//!   target_vel(t) = A · 2π · f(t) · cos(φ(t))
//!   φ(t)         = 2π · ( f0·t + ½·(f1−f0)/T · t² )
//!   f(t)         = f0 + (f1 − f0) · t / T
//!
//! Optional CSV output lets you plot tracking error / phase lag / current draw
//! after the run. The summary at the end reports peak tracking error, peak
//! command, and how often the cycle missed its deadline.
//!
//! ```ignore
//! cargo run -p lkmotor-driver --example mit_chirp -- \
//!     --device /dev/ttyUSB0 --baud 1000000 --id 1 \
//!     --gear-ratio 10.0 --kp 30 --kd 0.8 \
//!     --amplitude-deg 5 --f-start 0.5 --f-end 5 --duration-s 10 \
//!     --period-ms 5 --csv chirp_log.csv
//! ```

use std::fs::File;
use std::io::{BufWriter, Write as _};
use std::thread::sleep;
use std::time::{Duration, Instant};

use clap::Parser;
use lkmotor_driver::{Motor, MotorConfig, MotorId, Rs485Driver};
use lkmotor_driver::LkCommands;

#[derive(Parser, Debug)]
#[command(about = "Linear-chirp stability sweep for the case-B MIT-mode emulation")]
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
    /// Kp in A/rad (output-frame).
    #[arg(long, default_value_t = 10.0)]
    kp: f32,
    /// Kd in A/(rad/s) (output-frame).
    #[arg(long, default_value_t = 0.5)]
    kd: f32,
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
    /// Cycle period in ms (≥ 2× max period of the highest test frequency).
    #[arg(long, default_value_t = 5)]
    period_ms: u64,
    /// Disable analytical velocity feed-forward (set v_d = 0). Tests Kd as pure
    /// damping on measured velocity instead of tracking.
    #[arg(long, default_value_t = false)]
    no_vel_ff: bool,
    /// Soft cap on commanded current (A). With `--clip` this is the saturation
    /// level (chirp continues, command pinned to ±this); without `--clip` it is
    /// the abort threshold.
    #[arg(long, default_value_t = 5.0)]
    current_limit_a: f32,
    /// Clip the host-computed command to ±current-limit-a instead of aborting.
    /// Useful for chirp characterisation when high-frequency saturation is
    /// expected and you want the sweep to complete anyway.
    #[arg(long, default_value_t = false)]
    clip: bool,
    /// Hard runaway-detection threshold (A) — aborts even with `--clip` when
    /// the *unclipped* command magnitude exceeds this. Default sits well above
    /// any plausible motor saturation (MG: ±33 A) so it only trips on bugs.
    #[arg(long, default_value_t = 100.0)]
    current_abort_a: f32,
    /// Per-request response timeout in ms.
    #[arg(long, default_value_t = 50)]
    timeout_ms: u64,
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
        let _ = self.bus.torque_control(self.id, 0.0);
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
    let period = Duration::from_millis(args.period_ms);
    // Nyquist sanity: cycle rate must be at least 2× f_end (preferably ≥10×).
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
    let mut motor = Motor::new(id, MotorConfig::current_units(args.gear_ratio));

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
                "t_s,freq_hz,target_pos_rad,target_vel_rad_s,meas_pos_rad,meas_vel_rad_s,meas_current_a,cmd_current_a,cycle_ms"
            )?;
            Some(w)
        }
        None => None,
    };

    bus.motor_run(id)?;

    println!(
        "chirp: A={:.2}° (out)  f={}→{} Hz over {}s   cycle={} ms ({:.0} Hz)",
        args.amplitude_deg, args.f_start, args.f_end, args.duration_s, args.period_ms, cycle_hz
    );
    println!(
        "gains: Kp={} A/rad  Kd={} A/(rad/s)  vel_ff={}  i_limit={} A  clip={}  i_abort={} A",
        args.kp,
        args.kd,
        if args.no_vel_ff { "off" } else { "analytical" },
        args.current_limit_a,
        args.clip,
        args.current_abort_a
    );

    let guard = StopGuard {
        bus: &mut bus,
        id,
    };
    motor.rezero(guard.bus)?;

    // Stats.
    let mut peak_err_rad: f32 = 0.0;
    let mut peak_cmd_unclipped_a: f32 = 0.0;
    let mut peak_meas_current_a: f32 = 0.0;
    let mut saturated_count: u32 = 0;
    let mut overrun_count: u32 = 0;
    let mut tick: u32 = 0;
    let mut aborted: Option<String> = None;

    let start = Instant::now();
    while start.elapsed() < total {
        let loop_start = Instant::now();
        let t = start.elapsed().as_secs_f32();

        let (target_pos, target_vel, freq) =
            chirp(t, args.duration_s, args.f_start, args.f_end, amplitude_rad);
        let v_d = if args.no_vel_ff { 0.0 } else { target_vel };

        // Read State2 + update turn tracking, then compute cmd on the host so we
        // can clip / abort *before* sending.
        let m = motor.measure(guard.bus)?;
        let cmd_unclipped = args.kp * (target_pos - m.position_rad)
            + args.kd * (v_d - m.velocity_rad_per_s);

        if cmd_unclipped.abs() > peak_cmd_unclipped_a {
            peak_cmd_unclipped_a = cmd_unclipped.abs();
        }

        // Hard runaway check applies in both modes.
        if cmd_unclipped.abs() > args.current_abort_a {
            aborted = Some(format!(
                "unclipped command {:.2} A exceeded current_abort {:.2} A at t={:.3}s (f={:.2} Hz)",
                cmd_unclipped, args.current_abort_a, t, freq
            ));
            break;
        }

        // Soft saturation: clip or abort based on --clip.
        let cmd_to_send = if cmd_unclipped.abs() > args.current_limit_a {
            if args.clip {
                saturated_count += 1;
                cmd_unclipped.signum() * args.current_limit_a
            } else {
                aborted = Some(format!(
                    "command {:.2} A exceeded limit {:.2} A at t={:.3}s (f={:.2} Hz) — aborting (use --clip to continue)",
                    cmd_unclipped, args.current_limit_a, t, freq
                ));
                break;
            }
        } else {
            cmd_unclipped
        };

        motor.set_current(guard.bus, cmd_to_send)?;

        let err = (target_pos - m.position_rad).abs();
        if err > peak_err_rad {
            peak_err_rad = err;
        }
        if m.current_a.abs() > peak_meas_current_a {
            peak_meas_current_a = m.current_a.abs();
        }

        let cycle_ms = loop_start.elapsed().as_secs_f32() * 1000.0;
        if cycle_ms > args.period_ms as f32 {
            overrun_count += 1;
        }

        if let Some(w) = csv_writer.as_mut() {
            writeln!(
                w,
                "{:.4},{:.4},{:.6},{:.6},{:.6},{:.6},{:.4},{:.4},{:.3}",
                t,
                freq,
                target_pos,
                target_vel,
                m.position_rad,
                m.velocity_rad_per_s,
                m.current_a,
                cmd_to_send,
                cycle_ms
            )?;
        }

        if tick.is_multiple_of(args.log_every) {
            let sat_marker = if cmd_unclipped.abs() > args.current_limit_a {
                "*"
            } else {
                " "
            };
            println!(
                "t={:>6.3}s  f={:>5.2} Hz  tgt={:+6.3}rad  meas={:+6.3}rad  err={:+5.3}rad  i_cmd={:+5.2}A{}  i_meas={:+5.2}A  cyc={:.2}ms",
                t, freq, target_pos, m.position_rad, target_pos - m.position_rad,
                cmd_to_send, sat_marker, m.current_a, cycle_ms,
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
        println!(
            "  peak |cmd_unclipped|: {:.2} A    (limit {:.2} A, abort {:.2} A)",
            peak_cmd_unclipped_a, args.current_limit_a, args.current_abort_a
        );
        println!("  peak |meas iq|      : {:.2} A", peak_meas_current_a);
        println!(
            "  saturated ticks     : {} / {} ({:.1}%)",
            saturated_count,
            tick,
            100.0 * saturated_count as f32 / tick as f32
        );
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
