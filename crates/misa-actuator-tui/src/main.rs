//! Driver-agnostic debug TUI for any motor that implements
//! [`misa_actuator::Actuator`].
//!
//! # Examples
//! ```text
//! # Robstride RS-05 on SocketCAN can0, motor id 1
//! misa-actuator-tui --driver robstride --interface can0 --motor-id 1 --model Edulite05
//!
//! # ...the same motor on Windows, through a PEAK adapter
//! misa-actuator-tui --driver robstride --interface pcan:usb1 --motor-id 1 --model Edulite05
//!
//! # LK Motor V3 on /dev/ttyUSB0 @ 1 Mbps, motor id 1, 1:10 gearbox
//! misa-actuator-tui --driver lkmotor --interface /dev/ttyUSB0 --motor-id 1 --baud 1000000 --gear-ratio 10.0
//!
//! # MyActuator RMD (CAN V3) on can0, motor id 1, Kt 0.83 N·m/A
//! misa-actuator-tui --driver myactuator --interface can0 --motor-id 1 --kt 0.83
//! ```

mod app;
mod commands;

use std::io;
use std::time::Duration;

use anyhow::Result;
use clap::Parser;
use crossterm::event::{self, Event};
use crossterm::terminal::{
    EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};
use crossterm::ExecutableCommand;
use ratatui::backend::CrosstermBackend;
use ratatui::Terminal;

use misa_actuator_tui::factory::{BusKind, DriverConfig, DriverKind, build_actuator, validate_driver_args};

use crate::app::App;

#[derive(Parser, Debug)]
#[command(version, about = "Driver-agnostic debug TUI for misa-actuator-compatible motors")]
struct Cli {
    /// Which motor family to talk to.
    #[arg(long, value_enum)]
    driver: DriverKind,

    /// Bus interface. For the CAN drivers: `can0` (Linux SocketCAN),
    /// `pcan:usb1` (PEAK adapter on Windows) or `slcan:COM5` (an adapter
    /// running **slcan** firmware — most dongles sold as "USB-CAN" speak
    /// something else). For lkmotor: a serial port (`/dev/ttyUSB0`, `COM5`).
    #[arg(long)]
    interface: String,

    /// Motor address on the bus (1..=127).
    #[arg(long, default_value_t = 1)]
    motor_id: u8,

    // -- robstride-only --
    /// Robstride: motor model name (`RS-04`, `EduLite05`, ...) — required,
    /// since it sets the MIT quantisation range and the family spans ±5.5 to
    /// ±120 N·m. Damiao: optional model override. Sim: preset name.
    #[arg(long, default_value = misa_actuator_tui::factory::MODEL_UNSPECIFIED)]
    model: String,
    /// Robstride: host CAN ID.
    #[arg(long, default_value_t = robstride_driver::DEFAULT_HOST_ID)]
    host_id: u8,

    // -- lkmotor-only --
    /// Lkmotor: serial baud rate.
    #[arg(long, default_value_t = 1_000_000)]
    baud: u32,
    /// Lkmotor: gear ratio (e.g. 10.0 for a 1:10 gearbox).
    #[arg(long, default_value_t = 10.0)]
    gear_ratio: f32,
    /// Lkmotor: torque constant Kt (N·m/A). 0 = use current-units mode (Nm
    /// API surfaces motor-frame current in A).
    #[arg(long, default_value_t = 0.0)]
    kt: f32,

    // -- damiao-only --
    /// Damiao: physical CAN layer — `can` (classic 1 Mbps) or `can-fd`.
    #[arg(long, value_enum, default_value_t = BusKind::Can)]
    bus: BusKind,

    /// Per-request timeout, in ms.
    #[arg(long, default_value_t = 100)]
    timeout_ms: u64,
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    // Windows sleeps round up to the ~15.6 ms scheduler tick by default,
    // which would throttle every timed loop below. No-op on Linux.
    let _timer = misa_actuator::realtime::TimerResolutionGuard::acquire();

    let cli = Cli::parse();
    let cfg = DriverConfig {
        kind: cli.driver,
        interface: cli.interface,
        motor_id: cli.motor_id,
        model: cli.model,
        host_id: cli.host_id,
        baud: cli.baud,
        gear_ratio: cli.gear_ratio,
        kt: cli.kt,
        // The TUI runs no characterization jobs, so the envelope never applies.
        max_torque_nm: 0.0,
        bus_kind: cli.bus,
        timeout: Duration::from_millis(cli.timeout_ms),
    };
    validate_driver_args(&cfg)?;

    let actuator = build_actuator(&cfg)?;
    let app = App::new(actuator, cfg);

    run_tui(app)
}

fn run_tui(mut app: App) -> Result<()> {
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    stdout.execute(EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(&mut stdout);
    let mut terminal = Terminal::new(backend)?;

    let result = run_loop(&mut terminal, &mut app);

    disable_raw_mode()?;
    io::stdout().execute(LeaveAlternateScreen)?;
    terminal.show_cursor().ok();
    result
}

fn run_loop<B: ratatui::backend::Backend>(
    terminal: &mut Terminal<B>,
    app: &mut App,
) -> Result<()> {
    while !app.quit {
        terminal.draw(|f| app.ui(f))?;

        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                if key.kind == event::KeyEventKind::Press {
                    app.handle_key(key);
                }
            }
        }

        app.tick();
    }
    Ok(())
}
