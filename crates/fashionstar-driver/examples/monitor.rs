//! Sync-monitor loop: one request per cycle for all ids, as the vendor
//! teleop script does. Prints the angles a few times a second together with
//! the achieved cycle rate and missed replies.
//!
//! Read-only by default. `--release` makes every listed joint limp and
//! `--reset-turns` clears the turn counters first — what the vendor script
//! does at start-up for a leader arm. Do not use `--release` on an arm that
//! is holding itself up.
//!
//! ```text
//! cargo run -p fashionstar-driver --example monitor -- --port /dev/ttyUSB0
//! ```

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use clap::Parser;
use fashionstar_driver::{
    default_serial_port, FashionStarBus, FsCommands, StopMode, DEFAULT_BAUD,
};

#[derive(Parser)]
struct Args {
    #[arg(long, default_value_t = default_serial_port().to_string())]
    port: String,
    #[arg(long, default_value_t = DEFAULT_BAUD)]
    baud: u32,
    /// Comma-separated servo ids (Star Arm 102 LD: joints 1-6 + gripper).
    #[arg(long, value_delimiter = ',', default_value = "0,1,2,3,4,5,6")]
    ids: Vec<u8>,
    /// Reply deadline for one whole sync-monitor cycle.
    #[arg(long, default_value_t = 10)]
    timeout_ms: u64,
    /// Release (limp) every listed joint before starting.
    #[arg(long)]
    release: bool,
    /// Reset every listed joint's turn counter before starting.
    #[arg(long)]
    reset_turns: bool,
    /// How often to print, Hz.
    #[arg(long, default_value_t = 5.0)]
    print_hz: f64,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let a = Args::parse();
    let mut bus = FashionStarBus::open(&a.port, a.baud, Duration::from_millis(a.timeout_ms))?;

    for &id in &a.ids {
        if a.release {
            bus.stop(id, StopMode::Release)?;
        }
        if a.reset_turns {
            bus.reset_multi_turn(id)?;
        }
    }
    if a.release || a.reset_turns {
        // Both are fire-and-forget; give the servos a moment before the
        // first read, as the vendor scripts do with their 10 ms sleeps.
        std::thread::sleep(Duration::from_millis(20));
    }

    let stop = Arc::new(AtomicBool::new(false));
    {
        let stop = stop.clone();
        ctrlc::set_handler(move || stop.store(true, Ordering::SeqCst))?;
    }

    let print_every = Duration::from_secs_f64(1.0 / a.print_hz.max(0.1));
    let mut window_start = Instant::now();
    let mut cycles = 0u64;
    let mut misses = vec![0u64; a.ids.len()];

    while !stop.load(Ordering::SeqCst) {
        let last = bus.sync_monitor(&a.ids)?;
        cycles += 1;
        for (miss, m) in misses.iter_mut().zip(&last) {
            if m.is_none() {
                *miss += 1;
            }
        }

        let elapsed = window_start.elapsed();
        if elapsed >= print_every {
            let angles: Vec<String> = a
                .ids
                .iter()
                .zip(&last)
                .map(|(id, m)| match m {
                    Some(m) => format!("{id}:{:8.1}", m.angle_deg()),
                    None => format!("{id}:{:>8}", "--"),
                })
                .collect();
            println!(
                "{:7.1} Hz  miss {:?}  garbage {}B | {}",
                cycles as f64 / elapsed.as_secs_f64(),
                misses,
                bus.discarded_bytes(),
                angles.join("  ")
            );
            window_start = Instant::now();
            cycles = 0;
            misses.iter_mut().for_each(|m| *m = 0);
        }
    }
    Ok(())
}
