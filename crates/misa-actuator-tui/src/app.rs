//! TUI app state, event handling, and rendering.

use std::collections::HashMap;
use std::time::{Duration, Instant};

use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
use misa_actuator::{Actuator, MotorFeedback, MotorStatus};
use ratatui::prelude::*;
use ratatui::widgets::*;

use crate::commands::{Command, ParamField, dispatch, params_for};
use crate::factory::DriverConfig;

const LOG_MAX: usize = 256;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Focus {
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
}

impl App {
    pub fn new(actuator: Box<dyn Actuator + Send>, cfg: DriverConfig) -> Self {
        let mut params_by_cmd = HashMap::new();
        for cmd in Command::ALL {
            params_by_cmd.insert(*cmd, params_for(*cmd));
        }
        Self {
            actuator,
            cfg,
            last_feedback: None,
            last_status: None,
            last_feedback_at: None,
            selected_cmd: 0,
            params_by_cmd,
            selected_param: 0,
            editing_param: false,
            edit_buf: String::new(),
            focus: Focus::Commands,
            log: vec!["misa-actuator-tui started — Tab cycles focus, Enter runs the command, q quits".into()],
            log_scroll: 0,
            quit: false,
            poll_interval: Duration::from_millis(200),
            last_poll: Instant::now(),
            auto_poll_enabled: false,
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

    pub fn handle_key(&mut self, key: KeyEvent) {
        // Ctrl-C always quits.
        if key.modifiers.contains(KeyModifiers::CONTROL) && key.code == KeyCode::Char('c') {
            self.quit = true;
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
                    Focus::Commands => Focus::Params,
                    Focus::Params => Focus::Commands,
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
            Focus::Commands => self.run_selected(),
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
            .constraints([Constraint::Percentage(40), Constraint::Percentage(60)])
            .split(area);
        self.render_commands(frame, chunks[0]);
        self.render_params(frame, chunks[1]);
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
            "↑/↓ select | Tab focus | Enter run/edit | p auto-poll | q quit"
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
