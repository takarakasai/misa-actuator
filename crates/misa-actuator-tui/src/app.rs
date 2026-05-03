//! TUI app state, event handling, and rendering.

use std::collections::HashMap;
use std::time::{Duration, Instant};

use anyhow::Result as AnyhowResult;
use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
use misa_actuator::{Actuator, MotorFeedback, MotorStatus};
use ratatui::prelude::*;
use ratatui::widgets::*;

use crate::commands::{Command, ParamField, dispatch, params_for};
use crate::factory::{DriverConfig, build_actuator};

const LOG_MAX: usize = 256;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Focus {
    Motors,
    Commands,
    Params,
}

pub struct App {
    actuator: Box<dyn Actuator + Send>,
    cfg: DriverConfig,
    /// Latest feedback (updated on every command + by the auto-poll tick).
    last_feedback: Option<MotorFeedback>,
    /// Latest read_status snapshot.
    last_status: Option<MotorStatus>,
    /// Wall-clock of the last successful feedback.
    last_feedback_at: Option<Instant>,

    /// Motor IDs visible in the Motors pane. Always contains the currently
    /// active id (`cfg.motor_id`); populated/replaced by Scan results.
    discovered_motors: Vec<u8>,
    /// Index into [`Self::discovered_motors`] for the Motors-pane cursor.
    selected_motor_idx: usize,

    selected_cmd: usize,
    /// Per-command params, kept across navigation so user edits don't get
    /// silently reset when moving the Commands selection.
    params_by_cmd: HashMap<Command, Vec<ParamField>>,
    selected_param: usize,
    editing_param: bool,
    edit_buf: String,
    focus: Focus,

    log: Vec<String>,
    log_scroll: u16,

    pub quit: bool,

    /// Auto-poll the actuator's `measure()` at this interval.
    poll_interval: Duration,
    last_poll: Instant,
    auto_poll_enabled: bool,

    /// In-progress scan state. `Some` while scanning; `None` otherwise.
    scan_state: Option<ScanInProgress>,
}

/// State for an in-progress incremental bus scan.
///
/// The scan runs one ID per `App::tick`, so the UI updates between probes
/// (the progress dialog reflects the latest `current` and `found` count).
struct ScanInProgress {
    from: u8,
    to: u8,
    /// Next ID to probe. When `> to`, the scan is complete.
    current: u8,
    found: Vec<u8>,
    started_at: Instant,
    timeout_per_id: Duration,
    cancelled: bool,
}

impl ScanInProgress {
    fn total(&self) -> u32 {
        self.to as u32 + 1 - self.from as u32
    }

    fn done_count(&self) -> u32 {
        // `current` is the *next* id; everything < current was already probed.
        (self.current as u32).saturating_sub(self.from as u32)
    }

    fn is_complete(&self) -> bool {
        self.cancelled || self.current > self.to
    }

    fn progress_ratio(&self) -> f64 {
        let total = self.total();
        if total == 0 {
            return 1.0;
        }
        self.done_count() as f64 / total as f64
    }
}

impl App {
    pub fn new(actuator: Box<dyn Actuator + Send>, cfg: DriverConfig) -> Self {
        let mut params_by_cmd = HashMap::new();
        for cmd in Command::ALL {
            params_by_cmd.insert(*cmd, params_for(*cmd));
        }
        let initial_id = cfg.motor_id;
        Self {
            actuator,
            cfg,
            last_feedback: None,
            last_status: None,
            last_feedback_at: None,
            discovered_motors: vec![initial_id],
            selected_motor_idx: 0,
            selected_cmd: 0,
            params_by_cmd,
            selected_param: 0,
            editing_param: false,
            edit_buf: String::new(),
            focus: Focus::Commands,
            log: vec!["misa-actuator-tui started — Tab cycles focus, Enter runs/edits/switches, q quits".into()],
            log_scroll: 0,
            quit: false,
            poll_interval: Duration::from_millis(200),
            last_poll: Instant::now(),
            auto_poll_enabled: false,
            scan_state: None,
        }
    }

    pub fn current_command(&self) -> Command {
        Command::ALL[self.selected_cmd]
    }

    fn current_params(&self) -> &[ParamField] {
        self.params_by_cmd
            .get(&self.current_command())
            .map(|v| v.as_slice())
            .unwrap_or(&[])
    }

    fn current_params_mut(&mut self) -> &mut Vec<ParamField> {
        let cmd = self.current_command();
        self.params_by_cmd.entry(cmd).or_insert_with(|| params_for(cmd))
    }

    fn on_command_changed(&mut self) {
        // Clamp selected_param to the new command's params length.
        let len = self.current_params().len();
        if self.selected_param >= len {
            self.selected_param = 0;
        }
        self.editing_param = false;
        self.edit_buf.clear();
    }

    fn log_msg(&mut self, msg: impl Into<String>) {
        let line = format!("[{}] {}", chrono_like_ts(), msg.into());
        self.log.push(line);
        if self.log.len() > LOG_MAX {
            self.log.remove(0);
        }
        self.log_scroll = self.log.len().saturating_sub(1) as u16;
    }

    pub fn tick(&mut self) {
        // Drive an in-progress scan one id per tick so the UI stays
        // responsive and the progress dialog updates between probes.
        if let Some(mut scan) = self.scan_state.take() {
            if scan.is_complete() {
                self.finalize_scan(scan);
            } else {
                let id = scan.current;
                scan.current = scan.current.saturating_add(1);
                let timeout = scan.timeout_per_id;
                match self.actuator.probe_motor(id, timeout) {
                    Ok(true) => scan.found.push(id),
                    Ok(false) => {}
                    Err(e) => {
                        // Log and continue — a single id failing shouldn't
                        // abort the whole sweep.
                        log::debug!("probe id={} error: {}", id, e);
                    }
                }
                self.scan_state = Some(scan);
            }
            // While scanning, skip auto-poll (busy on the bus).
            return;
        }

        if self.auto_poll_enabled && self.last_poll.elapsed() >= self.poll_interval {
            self.last_poll = Instant::now();
            match self.actuator.measure() {
                Ok(fb) => {
                    self.last_feedback = Some(fb);
                    self.last_feedback_at = Some(Instant::now());
                }
                Err(e) => {
                    self.log_msg(format!("auto-poll failed: {e}"));
                    self.auto_poll_enabled = false;
                }
            }
        }
    }

    fn finalize_scan(&mut self, scan: ScanInProgress) {
        let elapsed = scan.started_at.elapsed();
        let current = self.cfg.motor_id;

        if scan.cancelled {
            self.log_msg(format!(
                "Scan cancelled after {} of {} probes ({:.1} s)",
                scan.done_count(),
                scan.total(),
                elapsed.as_secs_f32()
            ));
            return;
        }

        let mut found = scan.found;
        if !found.contains(&current) {
            found.insert(0, current);
        }
        self.discovered_motors = found;
        self.selected_motor_idx = self
            .discovered_motors
            .iter()
            .position(|&id| id == current)
            .unwrap_or(0);

        self.log_msg(format!(
            "Scan complete: {} motor(s) in {:.1} s",
            self.discovered_motors.len(),
            elapsed.as_secs_f32()
        ));
        let snapshot: Vec<u8> = self.discovered_motors.clone();
        for id in snapshot {
            self.log_msg(format!(
                "  id={}{}",
                id,
                if id == current { "  (active)" } else { "" }
            ));
        }
        self.log_msg("Tab → Motors → ↑/↓ → Enter to switch");
        self.focus = Focus::Motors;
    }

    pub fn handle_key(&mut self, key: KeyEvent) {
        // Ctrl-C always quits.
        if key.modifiers.contains(KeyModifiers::CONTROL) && key.code == KeyCode::Char('c') {
            self.quit = true;
            return;
        }

        // While a scan is running, the only allowed key is Esc (cancel).
        // Block everything else so the user can't accidentally start
        // another command while the bus is busy.
        if self.scan_state.is_some() {
            if key.code == KeyCode::Esc {
                if let Some(scan) = self.scan_state.as_mut() {
                    scan.cancelled = true;
                }
            }
            return;
        }

        if self.editing_param {
            self.handle_edit_key(key);
            return;
        }

        match key.code {
            KeyCode::Char('q') | KeyCode::Esc => self.quit = true,
            KeyCode::Tab => {
                self.focus = match self.focus {
                    Focus::Motors => Focus::Commands,
                    Focus::Commands => Focus::Params,
                    Focus::Params => Focus::Motors,
                };
            }
            KeyCode::Char('p') => {
                self.auto_poll_enabled = !self.auto_poll_enabled;
                self.log_msg(format!(
                    "auto-poll {}",
                    if self.auto_poll_enabled { "ON" } else { "OFF" }
                ));
            }
            KeyCode::Up => self.move_selection(-1),
            KeyCode::Down => self.move_selection(1),
            KeyCode::Enter => self.activate(),
            _ => {}
        }
    }

    fn handle_edit_key(&mut self, key: KeyEvent) {
        match key.code {
            KeyCode::Esc => {
                self.editing_param = false;
                self.edit_buf.clear();
            }
            KeyCode::Enter => {
                let idx = self.selected_param;
                let buf = self.edit_buf.clone();
                if let Some(p) = self.current_params_mut().get_mut(idx) {
                    p.value = buf;
                }
                self.editing_param = false;
                self.edit_buf.clear();
            }
            KeyCode::Backspace => {
                self.edit_buf.pop();
            }
            KeyCode::Char(c) => {
                self.edit_buf.push(c);
            }
            _ => {}
        }
    }

    fn move_selection(&mut self, delta: isize) {
        match self.focus {
            Focus::Motors => {
                if self.discovered_motors.is_empty() {
                    return;
                }
                let n = self.discovered_motors.len() as isize;
                let new = (self.selected_motor_idx as isize + delta).rem_euclid(n);
                self.selected_motor_idx = new as usize;
            }
            Focus::Commands => {
                let n = Command::ALL.len() as isize;
                let new = (self.selected_cmd as isize + delta).rem_euclid(n);
                self.selected_cmd = new as usize;
                self.on_command_changed();
            }
            Focus::Params => {
                let len = self.current_params().len();
                if len == 0 {
                    return;
                }
                let n = len as isize;
                let new = (self.selected_param as isize + delta).rem_euclid(n);
                self.selected_param = new as usize;
            }
        }
    }

    fn activate(&mut self) {
        match self.focus {
            Focus::Motors => {
                let id = self.discovered_motors.get(self.selected_motor_idx).copied();
                if let Some(id) = id {
                    if id == self.cfg.motor_id {
                        self.log_msg(format!("motor {} already active", id));
                    } else {
                        match self.switch_to_motor(id) {
                            Ok(()) => self.log_msg(format!("switched to motor {}", id)),
                            Err(e) => self.log_msg(format!("switch to motor {} FAILED: {}", id, e)),
                        }
                    }
                }
            }
            Focus::Commands => {
                if matches!(self.current_command(), Command::Scan) {
                    self.start_scan();
                } else {
                    self.run_selected();
                }
            }
            Focus::Params => {
                let value = self
                    .current_params()
                    .get(self.selected_param)
                    .map(|p| p.value.clone());
                if let Some(v) = value {
                    self.editing_param = true;
                    self.edit_buf = v;
                }
            }
        }
    }

    /// Kick off an incremental scan. The actual probing happens one id per
    /// [`Self::tick`] so the UI stays responsive and the progress dialog
    /// updates between probes.
    fn start_scan(&mut self) {
        if self.scan_state.is_some() {
            self.log_msg("Scan already in progress");
            return;
        }
        let params: Vec<ParamField> = self
            .params_by_cmd
            .get(&Command::Scan)
            .cloned()
            .unwrap_or_default();
        let from: u8 = params
            .iter()
            .find(|p| p.name == "from")
            .and_then(|p| p.value.parse().ok())
            .unwrap_or(1);
        let to: u8 = params
            .iter()
            .find(|p| p.name == "to")
            .and_then(|p| p.value.parse().ok())
            .unwrap_or(32);
        let timeout_ms: u64 = params
            .iter()
            .find(|p| p.name == "timeout_ms")
            .and_then(|p| p.value.parse().ok())
            .unwrap_or(50);
        let lo = from.max(1);
        let hi = to.max(lo);

        self.log_msg(format!(
            "Scanning {}..={} (timeout {} ms/id)…  Esc to cancel",
            lo, hi, timeout_ms
        ));
        self.scan_state = Some(ScanInProgress {
            from: lo,
            to: hi,
            current: lo,
            found: Vec::new(),
            started_at: Instant::now(),
            timeout_per_id: Duration::from_millis(timeout_ms),
            cancelled: false,
        });
    }

    /// Tear down the current actuator and build a fresh one bound to
    /// `new_id`. The old actuator is dropped (which releases the bus and,
    /// for motors that auto-disable on Drop, sends a disable frame).
    fn switch_to_motor(&mut self, new_id: u8) -> AnyhowResult<()> {
        // Build fresh first so we don't tear down the working bus on
        // failure to reach the new motor's bus.
        let mut new_cfg = self.cfg.clone();
        new_cfg.motor_id = new_id;
        let new_actuator = build_actuator(&new_cfg)?;

        self.actuator = new_actuator;
        self.cfg = new_cfg;
        // Reset cached state — it belongs to the old motor.
        self.last_feedback = None;
        self.last_feedback_at = None;
        self.last_status = None;
        self.auto_poll_enabled = false;
        Ok(())
    }

    fn run_selected(&mut self) {
        let cmd = self.current_command();
        // Snapshot params so we can pass them by reference without holding a
        // long-lived borrow of self while calling the actuator.
        let params_snapshot: Vec<ParamField> = self.current_params().to_vec();
        let dispatch = dispatch(cmd, &params_snapshot, self.actuator.as_mut());
        if let Some(fb) = dispatch.feedback {
            self.last_feedback = Some(fb);
            self.last_feedback_at = Some(Instant::now());
        }
        if matches!(cmd, Command::ReadStatus) {
            // dispatch logs the status via log::info!; also surface in the
            // header by polling read_status separately so the UI updates.
            if let Ok(s) = self.actuator.read_status() {
                self.last_status = Some(s);
            }
        }
        self.log_msg(dispatch.log_line);
        for extra in dispatch.extra_log_lines {
            self.log_msg(extra);
        }
    }

    // -----------------------------------------------------------------
    // Rendering
    // -----------------------------------------------------------------

    pub fn ui(&self, frame: &mut Frame) {
        let area = frame.area();
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(6),  // header / live status
                Constraint::Min(8),     // commands + params
                Constraint::Length(10), // log
                Constraint::Length(1),  // hint bar
            ])
            .split(area);

        self.render_header(frame, chunks[0]);
        self.render_main(frame, chunks[1]);
        self.render_log(frame, chunks[2]);
        self.render_hint_bar(frame, chunks[3]);

        // Modal overlay: scan progress dialog. Drawn last so it sits on
        // top of everything else.
        if let Some(scan) = &self.scan_state {
            self.render_scan_dialog(frame, area, scan);
        }
    }

    fn render_scan_dialog(&self, frame: &mut Frame, screen: Rect, scan: &ScanInProgress) {
        // Center a 60-wide × 9-tall dialog on the screen (or shrink to fit).
        let w = 60.min(screen.width.saturating_sub(4));
        let h = 9.min(screen.height.saturating_sub(2));
        let x = screen.x + (screen.width.saturating_sub(w)) / 2;
        let y = screen.y + (screen.height.saturating_sub(h)) / 2;
        let area = Rect { x, y, width: w, height: h };

        // Clear underneath so the dialog isn't visually muddled.
        frame.render_widget(Clear, area);

        let block = Block::default()
            .title(" Scanning bus ")
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::Yellow));
        let inner = block.inner(area);
        frame.render_widget(block, area);

        // Layout: 2 lines of text + gauge + footer
        let rows = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(1), // range / current
                Constraint::Length(1), // found count
                Constraint::Length(1), // (spacer)
                Constraint::Length(1), // gauge
                Constraint::Min(1),    // footer
            ])
            .split(inner);

        let probing_id = scan.current.min(scan.to);
        frame.render_widget(
            Paragraph::new(format!(
                "Range: {}..={}   Probing id={}   Elapsed: {:.1}s",
                scan.from,
                scan.to,
                probing_id,
                scan.started_at.elapsed().as_secs_f32()
            )),
            rows[0],
        );
        frame.render_widget(
            Paragraph::new(format!(
                "Found so far: {}   Probed: {}/{}",
                scan.found.len(),
                scan.done_count(),
                scan.total()
            )),
            rows[1],
        );
        let gauge = Gauge::default()
            .gauge_style(Style::default().fg(Color::Cyan).bg(Color::DarkGray))
            .ratio(scan.progress_ratio().clamp(0.0, 1.0))
            .label(format!("{:.0}%", scan.progress_ratio() * 100.0));
        frame.render_widget(gauge, rows[3]);

        let footer = if scan.cancelled {
            Span::styled("cancelling…", Style::default().fg(Color::Red))
        } else {
            Span::styled(
                "Esc to cancel",
                Style::default().fg(Color::DarkGray),
            )
        };
        frame.render_widget(Paragraph::new(footer), rows[4]);
    }

    fn render_header(&self, frame: &mut Frame, area: Rect) {
        let mode_str = self
            .actuator
            .current_run_mode_hint()
            .map(|m| format!("{:?}", m))
            .unwrap_or_else(|| "?".into());
        let enabled = self.actuator.is_enabled_hint();
        let title = format!(
            " misa-actuator-tui — driver={:?} iface={} id={}  mode={}  enabled={} ",
            self.cfg.kind,
            self.cfg.interface,
            self.cfg.motor_id,
            mode_str,
            if enabled { "YES" } else { "no" },
        );
        let inner = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Percentage(60), Constraint::Percentage(40)])
            .split(area);

        // Live feedback panel
        let mut lines: Vec<Line> = Vec::new();
        match (&self.last_feedback, &self.last_feedback_at) {
            (Some(fb), Some(t)) => {
                let age = t.elapsed().as_millis();
                lines.push(Line::from(vec![
                    Span::raw("pos="),
                    Span::styled(
                        format!("{:+.4} rad", fb.position_rad),
                        Style::default().fg(Color::Cyan),
                    ),
                    Span::raw("  vel="),
                    Span::styled(
                        format!("{:+.4} rad/s", fb.velocity_rad_per_s),
                        Style::default().fg(Color::Cyan),
                    ),
                ]));
                lines.push(Line::from(vec![
                    Span::raw("τ="),
                    Span::styled(
                        format!("{:+.4} Nm", fb.torque_nm),
                        Style::default().fg(Color::Cyan),
                    ),
                    Span::raw("  iq="),
                    Span::styled(
                        format!("{:+.3} A", fb.current_a),
                        Style::default().fg(Color::Cyan),
                    ),
                    Span::raw(format!("  T={:.1}°C", fb.temperature_c)),
                    Span::styled(
                        format!("  ({} ms ago)", age),
                        Style::default().fg(Color::DarkGray),
                    ),
                ]));
            }
            _ => lines.push(Line::from(Span::styled(
                "no feedback yet — press Enter on Measure",
                Style::default().fg(Color::DarkGray),
            ))),
        }
        if let Some(s) = &self.last_status {
            lines.push(Line::from(format!(
                "Vbus={:.2} V  raw_err=0x{:08X}{}",
                s.voltage_v,
                s.error.raw(),
                if s.error.any() { "  ⚠" } else { "" },
            )));
        }
        lines.push(Line::from(Span::styled(
            format!(
                "auto-poll {} (toggle: p) — interval={} ms",
                if self.auto_poll_enabled { "ON" } else { "OFF" },
                self.poll_interval.as_millis()
            ),
            Style::default().fg(Color::DarkGray),
        )));
        frame.render_widget(
            Paragraph::new(lines)
                .block(Block::default().title(title).borders(Borders::ALL)),
            inner[0],
        );

        // Right: short instructions and recommended workflow
        let help = vec![
            Line::from("↑/↓ select  Tab focus  Enter edit/run  p auto-poll  q quit"),
            Line::from(""),
            Line::from(Span::styled(
                "Workflows (each line = one Enter on Commands):",
                Style::default().fg(Color::Yellow),
            )),
            Line::from("  Set Mode → Position → Enable → Set Position"),
            Line::from("  Set Mode → Velocity → Enable → Set Velocity"),
            Line::from("  Set Mode → MIT      → Enable → MIT Control"),
        ];
        frame.render_widget(
            Paragraph::new(help).block(Block::default().title(" keys / workflow ").borders(Borders::ALL)),
            inner[1],
        );
    }

    fn render_main(&self, frame: &mut Frame, area: Rect) {
        let chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Length(20),    // Motors (narrow)
                Constraint::Percentage(35), // Commands
                Constraint::Min(20),        // Params (rest)
            ])
            .split(area);
        self.render_motors(frame, chunks[0]);
        self.render_commands(frame, chunks[1]);
        self.render_params(frame, chunks[2]);
    }

    fn render_motors(&self, frame: &mut Frame, area: Rect) {
        let border_style = if self.focus == Focus::Motors {
            Style::default().fg(Color::Yellow)
        } else {
            Style::default()
        };
        let title = format!(" motors ({}) ", self.discovered_motors.len());

        if self.discovered_motors.is_empty() {
            frame.render_widget(
                Paragraph::new("(none — run Scan)")
                    .style(Style::default().fg(Color::DarkGray))
                    .block(
                        Block::default()
                            .title(title)
                            .borders(Borders::ALL)
                            .border_style(border_style),
                    ),
                area,
            );
            return;
        }

        let items: Vec<ListItem> = self
            .discovered_motors
            .iter()
            .enumerate()
            .map(|(i, &id)| {
                let active = id == self.cfg.motor_id;
                let cursor = self.focus == Focus::Motors && i == self.selected_motor_idx;
                let mut style = Style::default();
                if cursor {
                    style = style.bg(Color::Blue).fg(Color::White);
                }
                let label = if active {
                    format!("● id={:<3} (active)", id)
                } else {
                    format!("○ id={}", id)
                };
                ListItem::new(Span::styled(label, style))
            })
            .collect();
        frame.render_widget(
            List::new(items).block(
                Block::default()
                    .title(title)
                    .borders(Borders::ALL)
                    .border_style(border_style),
            ),
            area,
        );
    }

    fn render_commands(&self, frame: &mut Frame, area: Rect) {
        let items: Vec<ListItem> = Command::ALL
            .iter()
            .enumerate()
            .map(|(i, c)| {
                let style = if i == self.selected_cmd {
                    Style::default().bg(Color::Blue).fg(Color::White)
                } else {
                    Style::default()
                };
                ListItem::new(Span::styled(c.label(), style))
            })
            .collect();
        let border_style = if self.focus == Focus::Commands {
            Style::default().fg(Color::Yellow)
        } else {
            Style::default()
        };
        frame.render_widget(
            List::new(items).block(
                Block::default()
                    .title(" commands ")
                    .borders(Borders::ALL)
                    .border_style(border_style),
            ),
            area,
        );
    }

    fn render_params(&self, frame: &mut Frame, area: Rect) {
        let border_style = if self.focus == Focus::Params {
            Style::default().fg(Color::Yellow)
        } else {
            Style::default()
        };
        let title = format!(" params: {} ", self.current_command().label());
        let params = self.current_params();

        if params.is_empty() {
            frame.render_widget(
                Paragraph::new("(no parameters — press Enter on Commands pane to run)")
                    .style(Style::default().fg(Color::DarkGray))
                    .block(
                        Block::default()
                            .title(title)
                            .borders(Borders::ALL)
                            .border_style(border_style),
                    ),
                area,
            );
            return;
        }

        let mut lines: Vec<Line> = Vec::new();
        for (i, p) in params.iter().enumerate() {
            let selected = self.focus == Focus::Params && i == self.selected_param;
            let value_display: String = if selected && self.editing_param {
                format!("{}_", self.edit_buf)
            } else {
                p.value.clone()
            };
            let style = if selected {
                Style::default().bg(Color::DarkGray).fg(Color::White)
            } else {
                Style::default()
            };
            let mut spans = vec![
                Span::styled(format!("  {:<10} = ", p.name), style),
                Span::styled(format!("{:<14}", value_display), style.fg(Color::Cyan)),
                Span::styled(
                    format!("  {}", p.desc),
                    Style::default().fg(Color::DarkGray),
                ),
            ];
            if let Some(c) = p.choices {
                spans.push(Span::styled(
                    format!("  [{}]", c),
                    Style::default().fg(Color::DarkGray),
                ));
            }
            lines.push(Line::from(spans));
        }
        if self.editing_param {
            lines.push(Line::from(""));
            lines.push(Line::from(Span::styled(
                "  Editing — Enter to commit, Esc to cancel",
                Style::default().fg(Color::Yellow),
            )));
        }

        frame.render_widget(
            Paragraph::new(lines).block(
                Block::default()
                    .title(title)
                    .borders(Borders::ALL)
                    .border_style(border_style),
            ),
            area,
        );
    }

    fn render_log(&self, frame: &mut Frame, area: Rect) {
        let visible: Vec<Line> = self
            .log
            .iter()
            .rev()
            .take(area.height.saturating_sub(2) as usize)
            .rev()
            .map(|l| Line::from(l.as_str()))
            .collect();
        frame.render_widget(
            Paragraph::new(visible).block(Block::default().title(" log ").borders(Borders::ALL)),
            area,
        );
    }

    fn render_hint_bar(&self, frame: &mut Frame, area: Rect) {
        let txt = if self.editing_param {
            "EDIT  Enter=commit  Esc=cancel  Backspace=delete"
        } else {
            "↑/↓ select | Tab cycle Motors→Commands→Params | Enter switch/run/edit | p auto-poll | q quit"
        };
        frame.render_widget(
            Paragraph::new(Span::styled(
                txt,
                Style::default().fg(Color::Black).bg(Color::Gray),
            )),
            area,
        );
    }
}

fn chrono_like_ts() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
    let secs = now.as_secs();
    let h = (secs / 3600) % 24;
    let m = (secs / 60) % 60;
    let s = secs % 60;
    format!("{:02}:{:02}:{:02}", h, m, s)
}
