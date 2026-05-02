//! Driver-agnostic debug TUI for any motor that implements
//! [`misa_actuator::Actuator`].
//!
//! # Examples
//! ```text
//! # Robstride RS-05 on SocketCAN can0, motor id 1
//! misa-actuator-tui --driver robstride --interface can0 --motor-id 1 --model Edulite05
//!
//! # LK Motor V3 on /dev/ttyUSB0 @ 1 Mbps, motor id 1, 1:10 gearbox
//! misa-actuator-tui --driver lkmotor --interface /dev/ttyUSB0 --motor-id 1 --baud 1000000 --gear-ratio 10.0
//! ```

mod app;
mod commands;
mod factory;

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

use crate::app::App;
use crate::factory::{DriverConfig, DriverKind, build_actuator, validate_driver_args};

#[derive(Parser, Debug)]
#[command(version, about = "Driver-agnostic debug TUI for misa-actuator-compatible motors")]
struct Cli {
    /// Which motor family to talk to.
    #[arg(long, value_enum)]
    driver: DriverKind,

    /// Bus interface — SocketCAN name (`can0`) for robstride, or serial
    /// device path (`/dev/ttyUSB0`) for lkmotor.
    #[arg(long)]
    interface: String,

    /// Motor address on the bus (1..=127).
    #[arg(long, default_value_t = 1)]
    motor_id: u8,

    // -- robstride-only --
    /// Robstride: motor model name (`RS-05`, `Edulite05`, ...).
    #[arg(long, default_value = "Edulite05")]
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

    /// Per-request timeout, in ms.
    #[arg(long, default_value_t = 100)]
    timeout_ms: u64,
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();

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
