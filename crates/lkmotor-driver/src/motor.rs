//! SI-unit motor controller — unified position / velocity / torque / MIT API.
//!
//! Wraps the encoder turn tracker, gear ratio, and torque constant so that
//! every command and measurement is expressed in **output-frame SI**:
//! position in radians, velocity in rad/s, torque in N·m. Designed to mirror
//! the robstride-driver `Motor`-style surface (`set_position` /
//! `set_velocity` / `set_torque` / `mit_control`) so that downstream code can
//! later be made generic over the motor backend.
//!
//! ### What's the relationship to [`crate::mit::MitController`]?
//!
//! `MitController` predates this module and uses **A / A·rad⁻¹** units (raw
//! current). It still works (mit_chirp / mit_hold depend on it) and is kept
//! intact. New code should prefer [`Motor`].
//!
//! ### Torque vs current
//!
//! lkmotor's `0xA1` command takes a current (`iq` in A). To accept torque
//! commands in N·m, [`MotorConfig::torque_constant_nm_per_a`] (`Kt`) must be
//! known. If you don't know `Kt` for your motor, use
//! [`MotorConfig::current_units`] which sets `Kt = 1/gear_ratio` so that the
//! `torque_nm` API surface effectively carries `current_a` instead.
//!
//! ### Position anchoring
//!
//! `0xA4` uses the motor's **internal** multi-turn position counter (whatever
//! it was at boot or last `0x95`). To make `set_position(0.0, ...)` mean
//! "current location", call [`Motor::rezero`] once before issuing position
//! commands — it reads the absolute angle (`0x92`) and stores it as the
//! origin. `set_position` returns `Error::PositionNotAnchored` if you skip
//! this step.

use core::f32::consts::{PI, TAU};

use lkmotor_protocol::response::{ENCODER_PERIOD, MotorState2};

use crate::bus::{LkBus, LkCommands, parse_state2_from_response};
use crate::error::{Error, Result};
use crate::motor_id::MotorId;

const DEG_PER_RAD: f32 = 180.0 / PI;

/// Mechanical and electrical configuration for one motor.
#[derive(Debug, Clone, Copy)]
pub struct MotorConfig {
    /// Motor-shaft revolutions per output-shaft revolution (e.g. `10.0` for a
    /// 1:10 gearbox; `1.0` for direct drive).
    pub gear_ratio: f32,
    /// Motor torque constant `Kt` in **N·m/A** (motor frame, *before* gear
    /// reduction). Used to translate between torque commands/measurements in
    /// N·m and on-the-wire current in A. If unknown, prefer
    /// [`Self::current_units`].
    pub torque_constant_nm_per_a: f32,
}

impl MotorConfig {
    pub fn new(gear_ratio: f32, torque_constant_nm_per_a: f32) -> Self {
        Self {
            gear_ratio,
            torque_constant_nm_per_a,
        }
    }

    /// Build a config that surfaces raw current (A) through the `torque_nm`
    /// API. Use when you don't know `Kt` and want to think in amps. Picks
    /// `Kt = 1/gear_ratio` so that `set_torque(x_nm) → x A` on the wire.
    pub fn current_units(gear_ratio: f32) -> Self {
        Self {
            gear_ratio,
            torque_constant_nm_per_a: 1.0 / gear_ratio,
        }
    }
}

/// Bitfield of motor error flags (State1 byte 6). Each bit is `1` when the
/// corresponding fault is active.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ErrorFlags(pub u8);

impl ErrorFlags {
    /// Raw bitfield value as reported by State1.
    #[inline]
    pub fn raw(self) -> u8 {
        self.0
    }
    /// `true` when any error bit is set.
    #[inline]
    pub fn any(self) -> bool {
        self.0 != 0
    }
    /// Bit 0 — bus voltage below the lower limit.
    #[inline]
    pub fn under_voltage(self) -> bool {
        self.0 & 0x01 != 0
    }
    /// Bit 1 — bus voltage above the upper limit.
    #[inline]
    pub fn over_voltage(self) -> bool {
        self.0 & 0x02 != 0
    }
    /// Bit 2 — driver IC temperature too high.
    #[inline]
    pub fn driver_overheat(self) -> bool {
        self.0 & 0x04 != 0
    }
    /// Bit 3 — motor temperature too high.
    #[inline]
    pub fn motor_overheat(self) -> bool {
        self.0 & 0x08 != 0
    }
    /// Bit 4 — motor over-current.
    #[inline]
    pub fn over_current(self) -> bool {
        self.0 & 0x10 != 0
    }
    /// Bit 5 — motor short circuit.
    #[inline]
    pub fn motor_short(self) -> bool {
        self.0 & 0x20 != 0
    }
    /// Bit 6 — motor stalled (rotor locked).
    #[inline]
    pub fn stalled(self) -> bool {
        self.0 & 0x40 != 0
    }
    /// Bit 7 — host command timeout.
    #[inline]
    pub fn signal_timeout(self) -> bool {
        self.0 & 0x80 != 0
    }
}

/// Slow-changing motor status (bus voltage, error flags, temperature).
///
/// Filled by [`Motor::read_status`] from a State1 (`0x9A`) read — a separate
/// transaction from [`Motor::measure`] (which reads State2 / `0x9C`). Voltage
/// and error flags only live in State1, so polling them at control-loop rate
/// is wasteful; call `read_status` occasionally (e.g. 1 Hz) for monitoring
/// instead.
#[derive(Debug, Clone, Copy)]
pub struct MotorStatus {
    /// Motor temperature in °C (also reported in [`MotorFeedback`]).
    pub temperature_c: i8,
    /// Bus voltage in **volts**.
    pub voltage_v: f32,
    /// Error flag bitfield. See [`ErrorFlags`] accessors.
    pub error: ErrorFlags,
}

/// Per-cycle measurement, output-frame SI units.
#[derive(Debug, Clone, Copy)]
pub struct MotorFeedback {
    /// Continuous (multi-turn) position in **output-frame radians**, relative
    /// to the encoder reading at the last [`Motor::rezero`].
    pub position_rad: f32,
    /// Velocity in **output-frame rad/s**.
    pub velocity_rad_per_s: f32,
    /// Output-frame torque in **N·m** = `current_a · Kt · gear_ratio`. Garbage
    /// if `Kt` was set to a placeholder value.
    pub torque_nm: f32,
    /// Raw motor-frame `iq` in **A** (always meaningful regardless of `Kt`).
    pub current_a: f32,
    /// Motor temperature in °C.
    pub temperature_c: i8,
}

/// SI-unit motor handle. Owns the encoder turn tracker and the absolute
/// position anchor; takes the bus by `&mut` on each call.
pub struct Motor {
    id: MotorId,
    config: MotorConfig,
    turns: i64,
    prev_raw: Option<u16>,
    raw_origin: u16,
    motor_zero_centideg: Option<i64>,
}

impl Motor {
    pub fn new(id: MotorId, config: MotorConfig) -> Self {
        Self {
            id,
            config,
            turns: 0,
            prev_raw: None,
            raw_origin: 0,
            motor_zero_centideg: None,
        }
    }

    pub fn id(&self) -> MotorId {
        self.id
    }
    pub fn config(&self) -> MotorConfig {
        self.config
    }
    pub fn turns(&self) -> i64 {
        self.turns
    }

    /// Resume motor operation (`0x88`). Must precede control commands when
    /// the motor was previously `motor_stop`'d.
    pub fn enable<B: LkBus + ?Sized>(&mut self, bus: &mut B) -> Result<()> {
        bus.motor_run(self.id)
    }

    /// Stop the motor (`0x81`) but keep the running flag. Safe shutdown.
    /// **Do not** use `motor_off` (`0x80`) on V2 firmware — it hangs the bus.
    pub fn disable<B: LkBus + ?Sized>(&mut self, bus: &mut B) -> Result<()> {
        bus.motor_stop(self.id)
    }

    /// Anchor position 0 to the motor's current state. Reads State2 + the
    /// absolute multi-turn angle (`0x92`). Required before [`Self::set_position`].
    pub fn rezero<B: LkBus + ?Sized>(&mut self, bus: &mut B) -> Result<()> {
        let s = bus.read_state2(self.id)?;
        self.prev_raw = Some(s.encoder_raw);
        self.raw_origin = s.encoder_raw;
        self.turns = 0;
        self.motor_zero_centideg = Some(bus.read_multi_turn_angle(self.id)?);
        Ok(())
    }

    /// Read State2 only — no command sent. Updates the turn tracker.
    pub fn measure<B: LkBus + ?Sized>(&mut self, bus: &mut B) -> Result<MotorFeedback> {
        let s = bus.read_state2(self.id)?;
        Ok(self.feedback_from_state2(s))
    }

    /// Read State1 (`0x9A`) — bus voltage, temperature, error flags.
    ///
    /// Separate transaction from [`Self::measure`]: voltage and error bits
    /// only appear in State1, not in State2. Suitable for slow monitoring
    /// (e.g. 1 Hz health check), not the inner control loop.
    pub fn read_status<B: LkBus + ?Sized>(&mut self, bus: &mut B) -> Result<MotorStatus> {
        let s = bus.read_state1(self.id)?;
        Ok(MotorStatus {
            temperature_c: s.temperature_c,
            voltage_v: s.voltage_v(),
            error: ErrorFlags(s.error_state),
        })
    }

    /// Position control (`0xA4`). `pos_rad` is output-frame, relative to the
    /// last `rezero` origin. `max_speed_rad_s` caps motion speed (output frame).
    pub fn set_position<B: LkBus + ?Sized>(
        &mut self,
        bus: &mut B,
        pos_rad: f32,
        max_speed_rad_s: f32,
    ) -> Result<MotorFeedback> {
        let zero = self.motor_zero_centideg.ok_or(Error::PositionNotAnchored {
            motor_id: self.id.get(),
        })?;
        let delta_centideg =
            (pos_rad * DEG_PER_RAD * 100.0 * self.config.gear_ratio) as i64;
        let max_speed = (max_speed_rad_s.abs() * DEG_PER_RAD * 100.0 * self.config.gear_ratio)
            .max(100.0) as u32;
        let resp = bus.position_control_with_speed(self.id, zero + delta_centideg, max_speed)?;
        let s = parse_state2_from_response(&resp)?;
        Ok(self.feedback_from_state2(s))
    }

    /// Velocity control (`0xA2`). `vel_rad_s` is output-frame.
    pub fn set_velocity<B: LkBus + ?Sized>(
        &mut self,
        bus: &mut B,
        vel_rad_s: f32,
    ) -> Result<MotorFeedback> {
        let centideg_per_s =
            (vel_rad_s * DEG_PER_RAD * 100.0 * self.config.gear_ratio) as i32;
        let resp = bus.speed_control(self.id, centideg_per_s)?;
        let s = parse_state2_from_response(&resp)?;
        Ok(self.feedback_from_state2(s))
    }

    /// Torque control (`0xA1`) — output-frame N·m. Requires correct
    /// `Kt` in [`MotorConfig`]. Falls through to [`Self::set_current`].
    pub fn set_torque<B: LkBus + ?Sized>(
        &mut self,
        bus: &mut B,
        torque_nm: f32,
    ) -> Result<MotorFeedback> {
        let current_a =
            torque_nm / (self.config.torque_constant_nm_per_a * self.config.gear_ratio);
        self.set_current(bus, current_a)
    }

    /// Current control (`0xA1`) — direct motor-frame `iq` in A. Bypasses
    /// `Kt` / gear-ratio scaling.
    pub fn set_current<B: LkBus + ?Sized>(
        &mut self,
        bus: &mut B,
        current_a: f32,
    ) -> Result<MotorFeedback> {
        let resp = bus.torque_control(self.id, current_a)?;
        let s = parse_state2_from_response(&resp)?;
        Ok(self.feedback_from_state2(s))
    }

    /// MIT-mode emulation (case-B host-side PD): `measure` → compute torque
    /// → `set_torque`. Two RS485 transactions per call.
    ///
    /// Gains are output-frame: `kp` in **N·m/rad**, `kd` in **N·m/(rad/s)**,
    /// `torque_ff_nm` in **N·m**. Returns the feedback from the *post-command*
    /// state2 reply (matches robstride's `mit_control` convention).
    pub fn mit_control<B: LkBus + ?Sized>(
        &mut self,
        bus: &mut B,
        pos_rad: f32,
        vel_rad_s: f32,
        kp_nm_per_rad: f32,
        kd_nm_per_rad_s: f32,
        torque_ff_nm: f32,
    ) -> Result<MotorFeedback> {
        let m = self.measure(bus)?;
        let cmd_torque_nm = kp_nm_per_rad * (pos_rad - m.position_rad)
            + kd_nm_per_rad_s * (vel_rad_s - m.velocity_rad_per_s)
            + torque_ff_nm;
        self.set_torque(bus, cmd_torque_nm)
    }

    /// Internal: convert State2 → MotorFeedback while updating turn tracking.
    fn feedback_from_state2(&mut self, s: MotorState2) -> MotorFeedback {
        let raw = s.encoder_raw;
        if let Some(prev) = self.prev_raw {
            let half = (ENCODER_PERIOD / 2) as i32;
            let delta = raw as i32 - prev as i32;
            if delta > half {
                self.turns -= 1;
            } else if delta < -half {
                self.turns += 1;
            }
        }
        self.prev_raw = Some(raw);

        let motor_pos_rev = self.turns as f64
            + ((raw as i32 - self.raw_origin as i32) as f64) / (ENCODER_PERIOD as f64);
        let position_rad = (motor_pos_rev as f32) * TAU / self.config.gear_ratio;
        let velocity_rad_per_s =
            (s.speed_deg_per_s as f32) / DEG_PER_RAD / self.config.gear_ratio;
        let current_a = s.current_amps();
        let torque_nm = current_a * self.config.torque_constant_nm_per_a * self.config.gear_ratio;

        MotorFeedback {
            position_rad,
            velocity_rad_per_s,
            torque_nm,
            current_a,
            temperature_c: s.temperature_c,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn current_units_round_trips_via_set_torque() {
        // With current_units(gear=10): Kt = 1/10. `set_torque(5.0)` should map
        // to current_a = 5.0 / (0.1 * 10) = 5.0 A.
        let cfg = MotorConfig::current_units(10.0);
        let torque_to_amps = 5.0 / (cfg.torque_constant_nm_per_a * cfg.gear_ratio);
        assert!((torque_to_amps - 5.0).abs() < 1e-6);
    }

    #[test]
    fn feedback_zero_velocity_for_zero_speed() {
        // Lightweight: construct a Motor, feed it a synthetic State2.
        let m_id = MotorId::new(1).unwrap();
        let mut m = Motor::new(m_id, MotorConfig::current_units(10.0));
        m.raw_origin = 0;
        m.prev_raw = Some(0);
        let s = MotorState2 {
            temperature_c: 25,
            iq_raw: 0,
            speed_deg_per_s: 0,
            encoder_raw: 0,
        };
        let fb = m.feedback_from_state2(s);
        assert_eq!(fb.position_rad, 0.0);
        assert_eq!(fb.velocity_rad_per_s, 0.0);
        assert_eq!(fb.current_a, 0.0);
    }

    #[test]
    fn feedback_quarter_motor_turn_is_quarter_of_2pi_over_gear() {
        let m_id = MotorId::new(1).unwrap();
        let mut m = Motor::new(m_id, MotorConfig::current_units(10.0));
        m.raw_origin = 0;
        m.prev_raw = Some(0);
        let quarter = (ENCODER_PERIOD / 4) as u16;
        let s = MotorState2 {
            temperature_c: 0,
            iq_raw: 0,
            speed_deg_per_s: 0,
            encoder_raw: quarter,
        };
        let fb = m.feedback_from_state2(s);
        // 1/4 motor rev = TAU/4 motor rad = TAU/4 / 10 output rad
        let expected = TAU / 4.0 / 10.0;
        assert!((fb.position_rad - expected).abs() < 1e-4);
    }

    #[test]
    fn feedback_motor_speed_to_output_velocity() {
        // 360 deg/s motor with 1:10 gearbox = 36 deg/s output
        // = 36 * π/180 rad/s ≈ 0.6283 rad/s
        let m_id = MotorId::new(1).unwrap();
        let mut m = Motor::new(m_id, MotorConfig::current_units(10.0));
        m.raw_origin = 0;
        m.prev_raw = Some(0);
        let s = MotorState2 {
            temperature_c: 0,
            iq_raw: 0,
            speed_deg_per_s: 360,
            encoder_raw: 0,
        };
        let fb = m.feedback_from_state2(s);
        assert!((fb.velocity_rad_per_s - (36.0_f32).to_radians()).abs() < 1e-4);
    }

    #[test]
    fn position_not_anchored_error() {
        // Can't easily test the Result without a bus mock — just verify the
        // initial state has motor_zero_centideg = None.
        let m = Motor::new(MotorId::new(1).unwrap(), MotorConfig::current_units(1.0));
        assert!(m.motor_zero_centideg.is_none());
    }

    #[test]
    fn error_flags_individual_bits() {
        assert!(ErrorFlags(0x01).under_voltage());
        assert!(ErrorFlags(0x02).over_voltage());
        assert!(ErrorFlags(0x04).driver_overheat());
        assert!(ErrorFlags(0x08).motor_overheat());
        assert!(ErrorFlags(0x10).over_current());
        assert!(ErrorFlags(0x20).motor_short());
        assert!(ErrorFlags(0x40).stalled());
        assert!(ErrorFlags(0x80).signal_timeout());

        let none = ErrorFlags(0x00);
        assert!(!none.any());
        assert!(!none.under_voltage());
        assert!(!none.signal_timeout());

        let multi = ErrorFlags(0x42);
        assert!(multi.over_voltage());
        assert!(multi.stalled());
        assert!(!multi.under_voltage());
        assert_eq!(multi.raw(), 0x42);
        assert!(multi.any());
    }
}
