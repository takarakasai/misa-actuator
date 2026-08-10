//! Read-only multi-motor monitoring dashboard.
//!
//! Unlike `misa-actuator-tui` (single-motor command console), this binary
//! takes a TOML config listing any number of motors — possibly a mix of
//! vendors/interfaces — and polls all of them on one screen. It never sends
//! control commands; it only calls `Actuator::measure()`. For per-motor
//! control (enable/set-position/MIT/...), use `misa-actuator-tui`.
//!
//! # Example
//! ```text
//! misa-actuator-monitor --config motors.toml --poll-ms 200
//! ```
//!
//! # Example `motors.toml`
//! ```toml
//! [[motor]]
//! name = "left-hip"
//! kind = "robstride"
//! interface = "can0"
//! motor_id = 1
//! model = "Edulite05"
//!
//! [[motor]]
//! name = "right-wrist"
//! kind = "lkmotor"
//! interface = "/dev/ttyUSB0"
//! motor_id = 1
//! baud = 1000000
//! gear_ratio = 10.0
//! ```

use std::io;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::Parser;
use crossterm::ExecutableCommand;
use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyModifiers};
use crossterm::terminal::{
    EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};
use ratatui::Terminal;
use ratatui::backend::CrosstermBackend;
use ratatui::prelude::*;
use ratatui::widgets::*;
use serde::Deserialize;

use misa_actuator::{Actuator, MotorFeedback};
use misa_actuator_tui::factory::{BusKind, DriverConfig, DriverKind, build_actuator};

#[derive(Parser, Debug)]
#[command(version, about = "Read-only multi-motor monitoring dashboard for misa-actuator drivers")]
struct Cli {
    /// TOML config listing the motors to watch (see module docs for the
    /// `[[motor]]` schema).
    #[arg(long)]
    config: PathBuf,

    /// Screen refresh / poll interval, in ms.
    #[arg(long, default_value_t = 200)]
    poll_ms: u64,

    /// Per-request bus timeout, in ms (applied to every motor).
    #[arg(long, default_value_t = 100)]
    timeout_ms: u64,
}

#[derive(Debug, Deserialize)]
struct MonitorConfig {
    motor: Vec<MotorEntry>,
}

#[derive(Debug, Deserialize)]
struct MotorEntry {
    /// Display label. Defaults to `"<kind>#<motor_id>"` if omitted.
    name: Option<String>,
    kind: DriverKind,
    interface: String,
    motor_id: u8,
    #[serde(default = "default_model")]
    model: String,
    #[serde(default = "default_host_id")]
    host_id: u8,
    #[serde(default = "default_baud")]
    baud: u32,
    #[serde(default = "default_gear_ratio")]
    gear_ratio: f32,
    #[serde(default)]
    kt: f32,
    #[serde(default = "default_bus_kind")]
    bus_kind: BusKind,
}

fn default_model() -> String {
    // Not a real model: a config that omits `model` for a RobStride motor must
    // fail loudly, not silently pick one whose MIT range is 21x off.
    misa_actuator_tui::factory::MODEL_UNSPECIFIED.to_string()
}
fn default_host_id() -> u8 {
    robstride_driver::DEFAULT_HOST_ID
}
fn default_baud() -> u32 {
    1_000_000
}
fn default_gear_ratio() -> f32 {
    10.0
}
fn default_bus_kind() -> BusKind {
    BusKind::Can
}

impl MotorEntry {
    fn label(&self) -> String {
        self.name
            .clone()
            .unwrap_or_else(|| format!("{:?}#{}", self.kind, self.motor_id).to_lowercase())
    }

    fn to_driver_config(&self, timeout: Duration) -> DriverConfig {
        DriverConfig {
            kind: self.kind,
            interface: self.interface.clone(),
            motor_id: self.motor_id,
            model: self.model.clone(),
            host_id: self.host_id,
            baud: self.baud,
            gear_ratio: self.gear_ratio,
            kt: self.kt,
            bus_kind: self.bus_kind,
            timeout,
        }
    }
}

fn load_config(path: &Path) -> Result<MonitorConfig> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read config {}", path.display()))?;
    toml::from_str(&text).with_context(|| format!("failed to parse config {}", path.display()))
}

/// One row's live state. A row that failed to connect at startup stays
/// `Failed` forever — this tool is a point-in-time diagnostic, not a
/// reconnect daemon.
enum RowState {
    Connected {
        actuator: Box<dyn Actuator + Send>,
        last: Option<MotorFeedback>,
        last_at: Option<Instant>,
        last_err: Option<String>,
    },
    Failed(String),
}

struct MotorRow {
    label: String,
    kind: DriverKind,
    interface: String,
    motor_id: u8,
    state: RowState,
}

fn build_rows(entries: &[MotorEntry], timeout: Duration) -> Vec<MotorRow> {
    entries
        .iter()
        .map(|e| {
            let cfg = e.to_driver_config(timeout);
            let state = match build_actuator(&cfg) {
                Ok(actuator) => RowState::Connected {
                    actuator,
                    last: None,
                    last_at: None,
                    last_err: None,
                },
                Err(err) => RowState::Failed(err.to_string()),
            };
            MotorRow {
                label: e.label(),
                kind: e.kind,
                interface: e.interface.clone(),
                motor_id: e.motor_id,
                state,
            }
        })
        .collect()
}

/// Poll every connected row once. A single motor's failure is recorded on
/// that row and does not stop the others from being polled.
fn poll_all(rows: &mut [MotorRow]) {
    for row in rows.iter_mut() {
        if let RowState::Connected {
            actuator,
            last,
            last_at,
            last_err,
        } = &mut row.state
        {
            match actuator.measure() {
                Ok(fb) => {
                    *last = Some(fb);
                    *last_at = Some(Instant::now());
                    *last_err = None;
                }
                Err(e) => {
                    *last_err = Some(e.to_string());
                }
            }
        }
    }
}

struct App {
    rows: Vec<MotorRow>,
    selected: usize,
    quit: bool,
    auto_poll: bool,
    poll_interval: Duration,
    last_poll: Instant,
    hint: String,
}

impl App {
    fn new(rows: Vec<MotorRow>, poll_interval: Duration) -> Self {
        let connected = rows
            .iter()
            .filter(|r| matches!(r.state, RowState::Connected { .. }))
            .count();
        Self {
            hint: format!(
                "{}/{} motor(s) connected — p: toggle poll, r: refresh now, ↑/↓: select, q: quit",
                connected,
                rows.len()
            ),
            rows,
            selected: 0,
            quit: false,
            auto_poll: true,
            poll_interval,
            last_poll: Instant::now(),
        }
    }

    fn tick(&mut self) {
        if self.auto_poll && self.last_poll.elapsed() >= self.poll_interval {
            self.last_poll = Instant::now();
            poll_all(&mut self.rows);
        }
    }

    fn handle_key(&mut self, key: KeyEvent) {
        if key.modifiers.contains(KeyModifiers::CONTROL) && key.code == KeyCode::Char('c') {
            self.quit = true;
            return;
        }
        match key.code {
            KeyCode::Char('q') | KeyCode::Esc => self.quit = true,
            KeyCode::Char('p') => {
                self.auto_poll = !self.auto_poll;
            }
            KeyCode::Char('r') => {
                poll_all(&mut self.rows);
                self.last_poll = Instant::now();
            }
            KeyCode::Up => {
                if !self.rows.is_empty() {
                    self.selected =
                        (self.selected + self.rows.len() - 1) % self.rows.len();
                }
            }
            KeyCode::Down => {
                if !self.rows.is_empty() {
                    self.selected = (self.selected + 1) % self.rows.len();
                }
            }
            _ => {}
        }
    }

    fn ui(&self, frame: &mut Frame) {
        let area = frame.area();
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Min(5), Constraint::Length(1)])
            .split(area);

        self.render_table(frame, chunks[0]);
        self.render_hint(frame, chunks[1]);
    }

    fn render_table(&self, frame: &mut Frame, area: Rect) {
        let header = Row::new(vec![
            "name", "driver", "iface/id", "pos(rad)", "vel(rad/s)", "τ(Nm)", "T(°C)", "age", "status",
        ])
        .style(Style::default().add_modifier(Modifier::BOLD));

        let rows: Vec<Row> = self
            .rows
            .iter()
            .enumerate()
            .map(|(i, row)| self.render_row(i, row))
            .collect();

        let widths = [
            Constraint::Length(14),
            Constraint::Length(11),
            Constraint::Length(16),
            Constraint::Length(10),
            Constraint::Length(11),
            Constraint::Length(8),
            Constraint::Length(7),
            Constraint::Length(8),
            Constraint::Min(20),
        ];

        let title = format!(
            " misa-actuator-monitor ({} motor(s)) — auto-poll {} (interval {} ms) ",
            self.rows.len(),
            if self.auto_poll { "ON" } else { "OFF" },
            self.poll_interval.as_millis(),
        );

        let table = Table::new(rows, widths)
            .header(header)
            .block(Block::default().title(title).borders(Borders::ALL));
        frame.render_widget(table, area);
    }

    fn render_row(&self, idx: usize, row: &MotorRow) -> Row<'static> {
        let selected = idx == self.selected;
        let base_style = if selected {
            Style::default().bg(Color::Blue).fg(Color::White)
        } else {
            Style::default()
        };

        let iface_id = format!("{}#{}", row.interface, row.motor_id);
        let kind_str = format!("{:?}", row.kind);

        let (pos, vel, tau, temp, age, status, status_style) = match &row.state {
            RowState::Failed(err) => (
                "—".to_string(),
                "—".to_string(),
                "—".to_string(),
                "—".to_string(),
                "—".to_string(),
                format!("CONNECT FAILED: {err}"),
                Style::default().fg(Color::DarkGray),
            ),
            RowState::Connected {
                last,
                last_at,
                last_err,
                ..
            } => {
                let (pos, vel, tau, temp) = match last {
                    Some(fb) => (
                        format!("{:+.3}", fb.position_rad),
                        format!("{:+.3}", fb.velocity_rad_per_s),
                        format!("{:+.3}", fb.torque_nm),
                        format!("{:.1}", fb.temperature_c),
                    ),
                    None => (
                        "—".to_string(),
                        "—".to_string(),
                        "—".to_string(),
                        "—".to_string(),
                    ),
                };
                let age = last_at
                    .map(|t| format!("{}ms", t.elapsed().as_millis()))
                    .unwrap_or_else(|| "—".to_string());
                match last_err {
                    Some(e) => (
                        pos,
                        vel,
                        tau,
                        temp,
                        age,
                        format!("ERROR: {e}"),
                        Style::default().fg(Color::Red),
                    ),
                    None if last.is_some() => (
                        pos,
                        vel,
                        tau,
                        temp,
                        age,
                        "OK".to_string(),
                        Style::default().fg(Color::Green),
                    ),
                    None => (
                        pos,
                        vel,
                        tau,
                        temp,
                        age,
                        "(waiting for first poll)".to_string(),
                        Style::default().fg(Color::DarkGray),
                    ),
                }
            }
        };

        Row::new(vec![
            Cell::from(row.label.clone()),
            Cell::from(kind_str),
            Cell::from(iface_id),
            Cell::from(pos),
            Cell::from(vel),
            Cell::from(tau),
            Cell::from(temp),
            Cell::from(age),
            Cell::from(Span::styled(status, status_style)),
        ])
        .style(base_style)
    }

    fn render_hint(&self, frame: &mut Frame, area: Rect) {
        frame.render_widget(
            Paragraph::new(Span::styled(
                self.hint.as_str(),
                Style::default().fg(Color::Black).bg(Color::Gray),
            )),
            area,
        );
    }
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    // Windows sleeps round up to the ~15.6 ms scheduler tick by default,
    // which would throttle every timed loop below. No-op on Linux.
    let _timer = misa_actuator::realtime::TimerResolutionGuard::acquire();
    let cli = Cli::parse();

    let config = load_config(&cli.config)?;
    if config.motor.is_empty() {
        anyhow::bail!("config {} has no [[motor]] entries", cli.config.display());
    }
    let timeout = Duration::from_millis(cli.timeout_ms);
    let rows = build_rows(&config.motor, timeout);
    let failed = rows
        .iter()
        .filter(|r| matches!(r.state, RowState::Failed(_)))
        .count();
    if failed > 0 {
        eprintln!(
            "warning: {failed} of {} motor(s) failed to connect — see the dashboard for details",
            rows.len()
        );
    }

    let mut app = App::new(rows, Duration::from_millis(cli.poll_ms));
    run_tui(&mut app)
}

fn run_tui(app: &mut App) -> Result<()> {
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    stdout.execute(EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(&mut stdout);
    let mut terminal = Terminal::new(backend)?;

    let result = run_loop(&mut terminal, app);

    disable_raw_mode()?;
    io::stdout().execute(LeaveAlternateScreen)?;
    terminal.show_cursor().ok();
    result
}

fn run_loop<B: ratatui::backend::Backend>(terminal: &mut Terminal<B>, app: &mut App) -> Result<()> {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_minimal_entry_with_defaults() {
        let cfg: MonitorConfig = toml::from_str(
            r#"
            [[motor]]
            kind = "lkmotor"
            interface = "/dev/ttyUSB0"
            motor_id = 1
            "#,
        )
        .unwrap();
        assert_eq!(cfg.motor.len(), 1);
        let m = &cfg.motor[0];
        assert_eq!(m.baud, 1_000_000);
        assert_eq!(m.gear_ratio, 10.0);
        assert_eq!(m.kt, 0.0);
        assert!(matches!(m.bus_kind, BusKind::Can));
        assert_eq!(m.label(), "lkmotor#1");
    }

    #[test]
    fn parses_multiple_entries_with_explicit_name_and_overrides() {
        let cfg: MonitorConfig = toml::from_str(
            r#"
            [[motor]]
            name = "left-hip"
            kind = "robstride"
            interface = "can0"
            motor_id = 3
            model = "RS-05"
            host_id = 200

            [[motor]]
            kind = "damiao"
            interface = "can1"
            motor_id = 2
            bus_kind = "can-fd"
            "#,
        )
        .unwrap();
        assert_eq!(cfg.motor.len(), 2);
        assert_eq!(cfg.motor[0].label(), "left-hip");
        assert_eq!(cfg.motor[0].model, "RS-05");
        assert_eq!(cfg.motor[0].host_id, 200);
        assert!(matches!(cfg.motor[1].bus_kind, BusKind::CanFd));
        assert_eq!(cfg.motor[1].label(), "damiao#2");
    }

    #[test]
    fn rejects_unknown_driver_kind() {
        let result: std::result::Result<MonitorConfig, _> = toml::from_str(
            r#"
            [[motor]]
            kind = "not-a-real-vendor"
            interface = "can0"
            motor_id = 1
            "#,
        );
        assert!(result.is_err());
    }
}
