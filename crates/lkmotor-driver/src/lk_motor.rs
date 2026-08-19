//! [`LkMotor`] — owns a [`LkBus`] and a [`Motor`], implements
//! [`misa_actuator::Actuator`].
//!
//! This is the high-level type that downstream applications and the
//! `misa-actuator-tui` debug TUI consume. It hides the bus type behind a
//! generic parameter so the same `LkMotor::set_position`, `enable`, etc.
//! work over RS485, CAN, or any future transport.
//!
//! ## lkmotor V3 quirks vs Robstride
//!
//! - **No "Enable" command that turns on the servo** — `MotorRun` (`0x88`)
//!   only resumes from `MotorStop`. To get holding torque the motor must
//!   be told *what* to control. [`Self::enable`] therefore: sends
//!   `MotorRun`, reads the absolute angle to anchor zero, and issues a
//!   position-control command at the current location to engage the
//!   position controller (= holding torque).
//! - **No native MIT mode** — [`Motor::mit_control`] is a host-side PD
//!   emulation that sends a `torque_control` (`0xA1`) frame. `0xA1`
//!   *latches* on the firmware: the motor keeps applying that torque
//!   until a new command arrives, so a one-shot host-side mit_control
//!   would spin the motor forever. For the polymorphic `Actuator` API,
//!   [`Self::mit_control`] therefore translates to a safe one-shot
//!   `position_control_with_speed` and **ignores `kp` / `kd` /
//!   `torque_ff`**. For real PD-loop MIT control, use
//!   [`Motor::mit_control`] directly inside your own control loop.
//! - **No run-mode parameter** — each motion command picks its own
//!   controller, so `Actuator::set_run_mode` is a no-op.

use std::ops::RangeInclusive;
use std::time::Duration;

use misa_actuator::{
    Actuator, ErrorFlags as MisaErrorFlags, MotorFeedback as MisaFeedback,
    MotorStatus as MisaStatus, Result as MisaResult, RunMode,
};

use crate::bus::{LkBus, LkCommands};
use crate::driver::Rs485Driver;
use crate::error::Result as LkResult;
use crate::motor::{ErrorFlags as LkErrorFlags, Motor, MotorConfig, MotorFeedback as LkFeedback,
    MotorStatus as LkStatus};
use crate::motor_id::MotorId;

/// Default max-speed (rad/s, output frame) used by `Actuator::enable` when
/// engaging the position-hold controller, and as a fallback by
/// `Actuator::mit_control` when `vel_rad_s == 0`.
const DEFAULT_HOLD_MAX_SPEED: f32 = 1.0;

/// How many times [`LkMotor::probe_motor`] asks before reporting an id absent.
///
/// A single lost frame must not read as "no motor here". On a marginal RS485
/// link the per-transaction loss rate is a few percent and drifts over time —
/// measured between 1% and 13% on one bench at 1 Mbit/s — which a single-shot
/// probe turns directly into a missed motor. Three tries take that from ~13%
/// to ~0.2% at the same loss rate.
///
/// Only *absent* ids pay for the retries: a motor that answers returns on the
/// first try, so a scan slows down in proportion to the empty ids it walks,
/// not the motors it finds.
const PROBE_ATTEMPTS: usize = 3;

/// How long to keep listening after a probe reply, watching for a second
/// device that holds the same id.
///
/// Long enough for a slower unit's reply to land after a faster one's — the
/// two observed sharing an id answered close enough together that both frames
/// were already buffered, so this mostly guards against the wider spacing.
const DUPLICATE_LISTEN: Duration = Duration::from_millis(20);

/// Position-anchoring policy for the first `set_position` call.
///
/// `set_position` requires an absolute zero anchor (lkmotor V3 has no
/// hardware "set zero" command — the firmware always reports motor-frame
/// absolute angle). [`PositionAnchor::OnFirstUse`] auto-rezeros on the
/// first call; [`PositionAnchor::Manual`] requires the caller to invoke
/// [`Actuator::set_zero`] explicitly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionAnchor {
    /// Auto-rezero on the first `set_position`. Convenient for the TUI.
    OnFirstUse,
    /// Caller must explicitly invoke `set_zero()` before `set_position`.
    Manual,
}

/// `Actuator`-implementing wrapper around an [`LkBus`].
///
/// Owns the bus and a [`Motor`] (encoder turn tracker, gear ratio,
/// torque-constant, position anchor). The bus is generic so the same type
/// can be backed by RS485 (`LkMotor<Rs485Driver>`) or future CAN
/// transports.
pub struct LkMotor<B: LkBus> {
    bus: B,
    motor: Motor,
    anchor: PositionAnchor,
    /// Tracked enable state (set by `Actuator::enable` / `Actuator::disable`).
    /// Surfaced to the TUI through [`Actuator::is_enabled_hint`].
    enabled: bool,
}

impl<B: LkBus> LkMotor<B> {
    /// Build a new `LkMotor` over an already-open bus.
    pub fn new(bus: B, id: MotorId, config: MotorConfig) -> Self {
        Self {
            bus,
            motor: Motor::new(id, config),
            anchor: PositionAnchor::OnFirstUse,
            enabled: false,
        }
    }

    /// Override the position-anchoring policy. Default is [`PositionAnchor::OnFirstUse`].
    pub fn with_position_anchor(mut self, anchor: PositionAnchor) -> Self {
        self.anchor = anchor;
        self
    }

    /// Borrow the underlying bus (for low-level diagnostic access).
    pub fn bus(&mut self) -> &mut B {
        &mut self.bus
    }

    /// Borrow the underlying [`Motor`] (for state inspection).
    pub fn motor(&self) -> &Motor {
        &self.motor
    }

    /// Wake the closed-loop controller (`MotorRun`) **without** re-anchoring
    /// zero (unlike [`Actuator::enable`], which also rezeros and holds).
    pub fn run(&mut self) -> LkResult<()> {
        self.motor.enable(&mut self.bus)
    }

    /// Move to an **absolute** output-frame angle in the motor's power-on
    /// multi-turn frame (LK-specific; bypasses the rezero anchor). Within a
    /// power cycle, `move_to_absolute(x)` always targets the same physical
    /// position, and pairs with [`Self::read_absolute_position`]. The returned
    /// feedback's velocity / torque are valid; read the absolute position
    /// separately.
    pub fn move_to_absolute(
        &mut self,
        pos_rad: f32,
        max_speed_rad_s: f32,
    ) -> LkResult<MisaFeedback> {
        Ok(lk_to_misa_feedback(self.motor.set_position_absolute(
            &mut self.bus,
            pos_rad,
            max_speed_rad_s,
        )?))
    }

    /// Read the absolute multi-turn position (output-frame rad, power-on frame).
    pub fn read_absolute_position(&mut self) -> LkResult<f32> {
        self.motor.read_absolute_angle(&mut self.bus)
    }
}

impl LkMotor<Rs485Driver> {
    /// Convenience constructor: open an RS485 port and wrap it.
    pub fn open_rs485(
        device: &str,
        baud: u32,
        id: MotorId,
        config: MotorConfig,
        response_timeout: Duration,
    ) -> LkResult<Self> {
        let bus = Rs485Driver::open(device, baud, response_timeout)?;
        Ok(Self::new(bus, id, config))
    }
}

fn lk_to_misa_feedback(fb: LkFeedback) -> MisaFeedback {
    MisaFeedback {
        position_rad: fb.position_rad,
        velocity_rad_per_s: fb.velocity_rad_per_s,
        torque_nm: fb.torque_nm,
        current_a: fb.current_a,
        temperature_c: fb.temperature_c as f32,
    }
}

fn lk_to_misa_status(s: LkStatus) -> MisaStatus {
    MisaStatus {
        voltage_v: s.voltage_v,
        temperature_c: s.temperature_c as f32,
        error: lk_to_misa_error(s.error),
    }
}

fn lk_to_misa_error(f: LkErrorFlags) -> MisaErrorFlags {
    let raw = f.raw() as u32;
    let mut bits = 0u32;
    if f.under_voltage()    { bits |= MisaErrorFlags::UNDER_VOLTAGE; }
    if f.over_voltage()     { bits |= MisaErrorFlags::OVER_VOLTAGE; }
    if f.over_current()     { bits |= MisaErrorFlags::OVER_CURRENT; }
    if f.motor_overheat()   { bits |= MisaErrorFlags::MOTOR_OVERHEAT; }
    if f.driver_overheat()  { bits |= MisaErrorFlags::DRIVER_OVERHEAT; }
    if f.stalled()          { bits |= MisaErrorFlags::STALL; }
    if f.motor_short()      { bits |= MisaErrorFlags::MOTOR_SHORT; }
    if f.signal_timeout()   { bits |= MisaErrorFlags::SIGNAL_TIMEOUT; }
    MisaErrorFlags::new(bits, raw)
}

impl<B: LkBus> Actuator for LkMotor<B> {
    fn motor_id(&self) -> u8 {
        self.motor.id().get()
    }

    fn enable(&mut self) -> MisaResult<MisaFeedback> {
        // Step 1: wake the motor from a possible MotorStop (0x81) state.
        self.motor.enable(&mut self.bus)?;

        // Step 2: engage the position controller at the current location.
        // `MotorRun` alone doesn't activate any closed-loop controller —
        // sending a position-control command (0xA4) where the shaft already is
        // makes the motor hold in place (= "servo on" feel that the user
        // expects from Enable).
        //
        // The zero anchor is set here only on the first call (lkmotor V3
        // firmware always reports motor-frame absolute angle, so a software-side
        // anchor is needed at all). A *re*-enable keeps the existing frame —
        // see [`Motor::engage_hold`] for the breakaway-map failure that
        // re-anchoring caused on the sibling MyActuator driver.
        let fb = self
            .motor
            .engage_hold(&mut self.bus, DEFAULT_HOLD_MAX_SPEED)?;

        self.enabled = true;
        Ok(lk_to_misa_feedback(fb))
    }

    fn disable(&mut self) -> MisaResult<()> {
        self.motor.disable(&mut self.bus)?;
        self.enabled = false;
        Ok(())
    }

    fn position_zero_in_motor_frame_rad(&self) -> Option<f32> {
        self.motor.zero_in_motor_frame_rad()
    }

    fn set_zero(&mut self) -> MisaResult<()> {
        self.motor.rezero(&mut self.bus)?;
        Ok(())
    }

    fn set_run_mode(&mut self, _mode: RunMode) -> MisaResult<()> {
        // lkmotor V3 has no run-mode parameter — each motion command picks
        // its own controller. Treat as no-op so generic callers don't need
        // to special-case the family.
        Ok(())
    }

    fn set_position(
        &mut self,
        pos_rad: f32,
        max_speed_rad_s: f32,
    ) -> MisaResult<MisaFeedback> {
        let result = self.motor.set_position(&mut self.bus, pos_rad, max_speed_rad_s);
        match result {
            Ok(fb) => Ok(lk_to_misa_feedback(fb)),
            Err(crate::error::Error::PositionNotAnchored { .. })
                if matches!(self.anchor, PositionAnchor::OnFirstUse) =>
            {
                self.motor.rezero(&mut self.bus)?;
                Ok(lk_to_misa_feedback(
                    self.motor.set_position(&mut self.bus, pos_rad, max_speed_rad_s)?,
                ))
            }
            Err(e) => Err(e.into()),
        }
    }

    fn set_velocity(&mut self, vel_rad_s: f32) -> MisaResult<MisaFeedback> {
        Ok(lk_to_misa_feedback(
            self.motor.set_velocity(&mut self.bus, vel_rad_s)?,
        ))
    }

    fn set_torque(&mut self, torque_nm: f32) -> MisaResult<MisaFeedback> {
        // NOTE: torque_control (0xA1) latches on lkmotor V3 — the motor
        // keeps applying the commanded torque until a new command arrives.
        // For one-shot use through this trait this can spin the motor
        // indefinitely if the load doesn't balance the torque. Callers
        // that want torque control should re-issue this in a tight loop
        // (or use a velocity / position controller for safer one-shot use).
        Ok(lk_to_misa_feedback(
            self.motor.set_torque(&mut self.bus, torque_nm)?,
        ))
    }

    fn mit_control(
        &mut self,
        pos_rad: f32,
        vel_rad_s: f32,
        _kp_nm_per_rad: f32,
        _kd_nm_per_rad_s: f32,
        _torque_ff_nm: f32,
    ) -> MisaResult<MisaFeedback> {
        // SAFETY: the underlying `Motor::mit_control` does host-side PD
        // and emits a `torque_control` (0xA1) frame, which *latches* on
        // the firmware. A one-shot call would compute torque from the
        // current position error and the motor would then spin forever
        // applying that torque — clearly the wrong behaviour through the
        // misa::Actuator trait, which is meant to be safe to call once.
        //
        // We translate one-shot mit_control into a safe
        // `position_control_with_speed`: target = `pos_rad`, max_speed
        // derived from |vel_rad_s| (or a default). This loses the kp/kd/
        // tau_ff semantics, but the motor reaches the target safely and
        // holds there. For real PD-loop MIT, drive `Motor::mit_control`
        // from your own control loop.
        let max_speed = vel_rad_s.abs().max(DEFAULT_HOLD_MAX_SPEED);
        self.set_position(pos_rad, max_speed)
    }

    fn measure(&mut self) -> MisaResult<MisaFeedback> {
        Ok(lk_to_misa_feedback(self.motor.measure(&mut self.bus)?))
    }

    fn read_status(&mut self) -> MisaResult<MisaStatus> {
        Ok(lk_to_misa_status(self.motor.read_status(&mut self.bus)?))
    }

    fn current_run_mode_hint(&self) -> Option<RunMode> {
        // lkmotor has no run-mode concept, so leave this as None.
        None
    }

    fn is_enabled_hint(&self) -> bool {
        self.enabled
    }

    fn scan_bus(
        &mut self,
        id_range: RangeInclusive<u8>,
        timeout_per_id: Duration,
    ) -> MisaResult<Vec<u8>> {
        let mut found = Vec::new();
        for id in id_range {
            if self.probe_motor(id, timeout_per_id)? {
                found.push(id);
            }
        }
        Ok(found)
    }

    fn probe_motor(&mut self, id: u8, _timeout: Duration) -> MisaResult<bool> {
        // lkmotor V3 has no broadcast / device-discovery command. We
        // probe by sending `ReadMotorState1` (`0x9A`) — a side-effect-free
        // read — and check for a response.
        //
        // Note: `_timeout` is currently ignored because the LkBus trait
        // doesn't expose per-request timeout setting; we use whatever the
        // bus was configured with at construction. Override `--timeout-ms`
        // at the TUI / driver level to control scan speed.
        let Some(motor_id) = MotorId::new(id) else {
            return Ok(false);
        };
        // Retry before concluding the id is empty — see `PROBE_ATTEMPTS`. A
        // motor that answers returns immediately, so only absent ids pay.
        for _ in 0..PROBE_ATTEMPTS {
            let _ = self.bus.flush_rx();
            if self.bus.read_state1(motor_id).is_ok() {
                return Ok(true);
            }
        }
        Ok(false)
    }
}

/// What [`LkMotor::probe_motor_report`] found at one id.
///
/// `Actuator::probe_motor` answers "did anyone reply", which is not the same
/// question as "is exactly one device here". Two motors sharing an id both
/// reply, so the plain probe reports the id as healthy — twice on this bench a
/// duplicate id went unnoticed by a scan while every transaction on that bus
/// was quietly colliding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProbeReport {
    /// At least one device answered.
    pub present: bool,
    /// A second reply arrived for a single request: two devices hold this id.
    /// Only ever true on a transport that can listen passively — check
    /// [`Self::duplicate_check_ran`] before reading `false` as "no duplicate".
    pub duplicate: bool,
    /// Whether the duplicate check actually ran on this transport.
    pub duplicate_check_ran: bool,
}

impl<B: LkBus> LkMotor<B> {
    /// Probe one id and report both whether anyone answered and whether more
    /// than one device did.
    ///
    /// Detection is one-sided: a second reply proves a duplicate, but silence
    /// does not prove there is only one device. Two motors that transmit at
    /// exactly the same moment collide on the half-duplex line and can destroy
    /// each other's frames rather than producing two readable ones. That case
    /// still shows up — as checksum failures rather than as a duplicate — so
    /// the bus never looks healthy when it is not.
    pub fn probe_motor_report(&mut self, id: u8) -> MisaResult<ProbeReport> {
        let can_check = self.bus.can_detect_duplicate_ids();
        let Some(motor_id) = MotorId::new(id) else {
            return Ok(ProbeReport {
                present: false,
                duplicate: false,
                duplicate_check_ran: false,
            });
        };

        for _ in 0..PROBE_ATTEMPTS {
            let _ = self.bus.flush_rx();
            if self.bus.read_state1(motor_id).is_ok() {
                let duplicate = self
                    .bus
                    .recv_extra(motor_id, DUPLICATE_LISTEN)
                    .unwrap_or(None)
                    .is_some();
                return Ok(ProbeReport {
                    present: true,
                    duplicate,
                    duplicate_check_ran: can_check,
                });
            }
        }
        Ok(ProbeReport {
            present: false,
            duplicate: false,
            duplicate_check_ran: can_check,
        })
    }
}

impl<B: LkBus> Drop for LkMotor<B> {
    fn drop(&mut self) {
        let _ = self.motor.disable(&mut self.bus);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::{LkBus, Response};
    use crate::error::Result as LkResult;
    use crate::motor::MotorConfig;
    use crate::motor_id::MotorId;
    use misa_actuator::Actuator;

    /// In-memory [`LkBus`] that records every transaction and replies with a
    /// command-appropriate zero payload, so the `Actuator` command routing can
    /// be checked without a serial port.
    #[derive(Default)]
    struct MockBus {
        /// (command, motor_id, data) for every `transact`.
        sent: Vec<(u8, u8, Vec<u8>)>,
    }

    impl LkBus for MockBus {
        fn transact(&mut self, command: u8, motor_id: MotorId, data: &[u8]) -> LkResult<Response> {
            self.sent.push((command, motor_id.get(), data.to_vec()));
            // ReadMultiTurnAngle (0x92) replies with 8 bytes; State1/State2 and
            // motion replies are 7 bytes.
            let len = if command == 0x92 { 8 } else { 7 };
            Ok(Response {
                command,
                motor_id: motor_id.get(),
                data: vec![0u8; len],
            })
        }
        fn send_only(&mut self, command: u8, motor_id: MotorId, data: &[u8]) -> LkResult<()> {
            self.sent.push((command, motor_id.get(), data.to_vec()));
            Ok(())
        }

        fn flush_rx(&mut self) -> LkResult<()> {
            Ok(())
        }
    }

    fn motor() -> LkMotor<MockBus> {
        LkMotor::new(
            MockBus::default(),
            MotorId::new(1).unwrap(),
            MotorConfig::current_units(1.0),
        )
    }

    /// Bus that models a bench with a chosen set of ids populated, and
    /// optionally one id held by two devices at once.
    struct DupBus {
        present: Vec<u8>,
        /// Id that two devices answer on, if any.
        shared: Option<u8>,
        /// Set when `recv_extra` has already handed over the second reply, so
        /// one duplicate produces one extra frame rather than an endless run.
        drained: bool,
        can_detect: bool,
    }

    impl LkBus for DupBus {
        fn transact(&mut self, command: u8, motor_id: MotorId, _d: &[u8]) -> LkResult<Response> {
            if !self.present.contains(&motor_id.get()) {
                return Err(crate::error::Error::Timeout {
                    motor_id: motor_id.get(),
                });
            }
            self.drained = false;
            Ok(Response {
                command,
                motor_id: motor_id.get(),
                data: vec![0u8; 7],
            })
        }
        fn send_only(&mut self, _c: u8, _m: MotorId, _d: &[u8]) -> LkResult<()> {
            Ok(())
        }
        fn flush_rx(&mut self) -> LkResult<()> {
            Ok(())
        }
        fn recv_extra(&mut self, motor_id: MotorId, _w: Duration) -> LkResult<Option<Response>> {
            if self.shared == Some(motor_id.get()) && !self.drained {
                self.drained = true;
                return Ok(Some(Response {
                    command: 0x9A,
                    motor_id: motor_id.get(),
                    data: vec![0u8; 7],
                }));
            }
            Ok(None)
        }
        fn can_detect_duplicate_ids(&self) -> bool {
            self.can_detect
        }
    }

    fn dup_motor(present: &[u8], shared: Option<u8>, can_detect: bool) -> LkMotor<DupBus> {
        LkMotor::new(
            DupBus {
                present: present.to_vec(),
                shared,
                drained: false,
                can_detect,
            },
            MotorId::new(1).unwrap(),
            MotorConfig::current_units(1.0),
        )
    }

    #[test]
    fn probe_report_flags_two_devices_on_one_id() {
        let mut m = dup_motor(&[1, 2, 3], Some(3), true);
        let clean = m.probe_motor_report(2).unwrap();
        assert!(clean.present);
        assert!(!clean.duplicate, "id 2 has a single device");

        let dup = m.probe_motor_report(3).unwrap();
        assert!(dup.present);
        assert!(dup.duplicate, "id 3 is held by two devices");
    }

    #[test]
    fn probe_report_marks_absent_ids() {
        let mut m = dup_motor(&[1], None, true);
        let r = m.probe_motor_report(7).unwrap();
        assert!(!r.present);
        assert!(!r.duplicate);
    }

    #[test]
    fn duplicate_flag_stays_false_when_the_transport_cannot_look() {
        // A transport that cannot listen passively must not have its silence
        // read as "no duplicate" — `duplicate_check_ran` is what says so.
        let mut m = dup_motor(&[1], None, false);
        let r = m.probe_motor_report(1).unwrap();
        assert!(r.present);
        assert!(!r.duplicate);
        assert!(!r.duplicate_check_ran);
    }

    #[test]
    fn plain_probe_motor_still_reports_a_duplicated_id_as_present() {
        // The narrower `Actuator::probe_motor` answers "did anyone reply", and
        // a duplicate replies. Keeping this explicit records why `scan` needed
        // `probe_motor_report` instead.
        let mut m = dup_motor(&[1, 2, 3], Some(3), true);
        assert!(m.probe_motor(3, Duration::from_millis(50)).unwrap());
    }

    fn sent_commands(m: &mut LkMotor<MockBus>) -> Vec<u8> {
        m.bus().sent.iter().map(|(c, _, _)| *c).collect()
    }

    #[test]
    fn set_velocity_sends_speed_closed_loop() {
        let mut m = motor();
        m.set_velocity(1.0).unwrap();
        assert!(sent_commands(&mut m).contains(&0xA2)); // SpeedClosedLoop
    }

    #[test]
    fn set_torque_sends_torque_closed_loop() {
        let mut m = motor();
        m.set_torque(0.5).unwrap();
        assert!(sent_commands(&mut m).contains(&0xA1)); // TorqueClosedLoop
    }

    #[test]
    fn probe_motor_true_on_reply() {
        let mut m = motor();
        assert!(m.probe_motor(1, Duration::from_millis(10)).unwrap());
    }

    #[test]
    fn probe_motor_false_and_silent_for_out_of_range_id() {
        let mut m = motor();
        assert!(!m.probe_motor(33, Duration::from_millis(10)).unwrap());
        assert!(m.bus().sent.is_empty(), "no frame should be sent for an invalid id");
    }

    #[test]
    fn scan_bus_collects_responders() {
        let mut m = motor();
        let found = m.scan_bus(1..=3, Duration::from_millis(10)).unwrap();
        assert_eq!(found, vec![1, 2, 3]);
    }

    #[test]
    fn read_status_parses_zeroed_state1() {
        let mut m = motor();
        let st = m.read_status().unwrap();
        assert_eq!(st.voltage_v, 0.0);
        assert!(!st.error.any());
    }

    #[test]
    fn move_to_absolute_commands_absolute_centideg() {
        let mut m = motor(); // gear_ratio 1.0
        m.move_to_absolute(std::f32::consts::PI, 5.0).unwrap();
        // π rad output = 180° = 18000 centideg, sent on PositionClosedLoop2 (0xA4).
        let frame = m
            .bus()
            .sent
            .iter()
            .find(|(c, _, _)| *c == 0xA4)
            .expect("a position frame was sent");
        let centideg = i64::from_le_bytes(frame.2[0..8].try_into().unwrap());
        assert!((centideg - 18_000).abs() <= 1, "centideg={centideg}");
    }

    /// Re-enabling must not re-anchor. `0x92` is the read that defines the
    /// frame, so its absence on the second call is the contract.
    ///
    /// The sibling MyActuator driver had the unconditional version, and it cost
    /// a breakaway map: `misa_sysid::run_breakaway` re-enables before every
    /// ramp, so each ramp's position was reported against a frame anchored a
    /// millisecond earlier and came back as ~0 (2026-08-08, RMD-X4).
    #[test]
    fn re_enable_does_not_re_anchor_the_frame() {
        let mut m = motor();
        Actuator::enable(&mut m).unwrap();
        assert!(
            sent_commands(&mut m).contains(&0x92),
            "the first enable anchors the frame"
        );

        m.bus().sent.clear();
        Actuator::enable(&mut m).unwrap();
        let cmds = sent_commands(&mut m);
        assert!(
            !cmds.contains(&0x92),
            "re-enable must keep the existing anchor: {cmds:02X?}"
        );
        assert!(cmds.contains(&0xA4), "it should still engage the hold");
    }

    #[test]
    fn read_absolute_position_reads_multi_turn_angle() {
        let mut m = motor();
        // Mock returns an all-zero 0x92 reply → 0 rad.
        assert_eq!(m.read_absolute_position().unwrap(), 0.0);
        assert!(sent_commands(&mut m).contains(&0x92)); // ReadMultiTurnAngle
    }
}
