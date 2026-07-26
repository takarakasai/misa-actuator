//! High-level MyActuator RMD motor driver, generic over the
//! [`MyActuatorBus`] transport.
//!
//! ## Units
//!
//! The V3 protocol speaks **output-shaft** units on the wire (0.01° / 1 dps /
//! 0.01 A), so no gear-ratio bookkeeping is needed — only the torque constant
//! `Kt` to translate between N·m and `iq` amps (see [`MotorConfig`]).
//!
//! ## Position anchoring
//!
//! `0xA4` targets the motor's multi-turn frame whose zero lives in the
//! encoder ROM. To make `set_position(0.0, …)` mean "current location", call
//! [`MyActuatorMotor::rezero`] once — it reads the absolute angle (`0x92`)
//! and stores it as a **soft** zero applied to all commands and feedback.
//! Persisting the zero into ROM ([`MyActuatorMotor::set_zero_rom`]) wears
//! flash and needs a reset to take effect, so it is reserved for explicit
//! calibration.

use std::f32::consts::PI;
use std::time::{Duration, Instant};

use myactuator_protocol::{
    build_brake_lock, build_brake_release, build_function_control, build_motion_control,
    build_position_control, build_read_acceleration, build_read_motor_model,
    build_read_multi_turn_angle, build_read_pid, build_read_status1, build_read_status2,
    build_read_version_date, build_set_zero_rom, build_shutdown, build_speed_control, build_stop,
    build_system_reset, build_torque_control, can_id, parse_acceleration, parse_motion_reply,
    parse_motor_model, parse_multi_turn_angle, parse_pid_gains, parse_status1, parse_status2,
    parse_version_date, AccelIndex, ErrorState, MotionFeedback, PidGains, Status1, Status2,
    DATA_LEN,
};

use crate::bus::{MyActuatorBus, SocketCanBus};
use crate::error::{Error, Result};

const DEG_PER_RAD: f32 = 180.0 / PI;

/// Default per-request timeout.
const DEFAULT_TIMEOUT: Duration = Duration::from_millis(100);

/// Electrical configuration for one motor.
#[derive(Debug, Clone, Copy)]
pub struct MotorConfig {
    /// Output-frame torque constant `Kt` in **N·m/A** (torque per amp of `iq`,
    /// measured at the output shaft — the value MyActuator datasheets quote as
    /// "torque constant" for geared RMD units). Translates between the N·m API
    /// surface and the on-the-wire `0xA1` current. If unknown, prefer
    /// [`Self::current_units`].
    pub torque_constant_nm_per_a: f32,
}

impl MotorConfig {
    pub fn new(torque_constant_nm_per_a: f32) -> Self {
        Self {
            torque_constant_nm_per_a,
        }
    }

    /// Build a config that surfaces raw current (A) through the `torque_nm`
    /// API: `Kt = 1`, so `set_torque(x) → x A` on the wire and reported
    /// `torque_nm` equals `current_a`.
    pub fn current_units() -> Self {
        Self {
            torque_constant_nm_per_a: 1.0,
        }
    }
}

/// Per-cycle measurement, output-frame SI units.
#[derive(Debug, Clone, Copy)]
pub struct MotorFeedback {
    /// Multi-turn position in **rad**, relative to the last
    /// [`MyActuatorMotor::rezero`] anchor (motor power-on frame when not
    /// anchored).
    pub position_rad: f32,
    /// Output-shaft velocity in **rad/s**.
    pub velocity_rad_per_s: f32,
    /// Output torque in **N·m** = `current_a · Kt`. Equals `current_a` when
    /// [`MotorConfig::current_units`] is used.
    pub torque_nm: f32,
    /// Torque current `iq` in **A**.
    pub current_a: f32,
    /// Motor temperature in °C.
    pub temperature_c: i8,
}

/// Slow-changing status from Status1 (`0x9A`).
#[derive(Debug, Clone, Copy)]
pub struct MotorStatus {
    /// Motor temperature in °C.
    pub temperature_c: i8,
    /// Bus voltage in volts.
    pub voltage_v: f32,
    /// `true` = the brake is released (brake-equipped models).
    pub brake_released: bool,
    /// Error flag bitfield.
    pub error: ErrorState,
}

/// High-level MyActuator RMD motor handle.
pub struct MyActuatorMotor<B: MyActuatorBus = SocketCanBus> {
    bus: B,
    motor_id: u8,
    config: MotorConfig,
    timeout: Duration,
    /// Soft zero: the `0x92` multi-turn angle (0.01°/LSB) captured at the last
    /// [`Self::rezero`]. Commanded positions add it, reported ones subtract it.
    zero_centideg: Option<i64>,
    /// Tracked enable state (see `Actuator::is_enabled_hint`).
    pub(crate) enabled: bool,
}

impl MyActuatorMotor<SocketCanBus> {
    /// Open a SocketCAN interface (e.g. `"can0"`) and bind to one motor.
    pub fn open(interface: &str, motor_id: u8, config: MotorConfig) -> Result<Self> {
        let bus = SocketCanBus::open(interface)?;
        Self::with_bus(bus, motor_id, config)
    }
}

impl<B: MyActuatorBus> MyActuatorMotor<B> {
    /// Wrap an already-open bus. `motor_id` must be 1..=32.
    pub fn with_bus(bus: B, motor_id: u8, config: MotorConfig) -> Result<Self> {
        if !can_id::is_valid_motor_id(motor_id) {
            return Err(Error::InvalidMotorId(motor_id));
        }
        Ok(Self {
            bus,
            motor_id,
            config,
            timeout: DEFAULT_TIMEOUT,
            zero_centideg: None,
            enabled: false,
        })
    }

    // -- accessors --

    /// The motor's bus id (1..=32).
    pub fn motor_id(&self) -> u8 {
        self.motor_id
    }
    /// The electrical configuration.
    pub fn config(&self) -> MotorConfig {
        self.config
    }
    /// The soft-zero anchor (0.01°/LSB), if [`Self::rezero`] has run.
    pub fn zero_centideg(&self) -> Option<i64> {
        self.zero_centideg
    }
    /// Mutable access to the underlying bus (used by scan helpers).
    pub fn bus(&mut self) -> &mut B {
        &mut self.bus
    }

    /// Set the per-request timeout.
    pub fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.bus.set_timeout(timeout)?;
        self.timeout = timeout;
        Ok(())
    }

    // -- low-level I/O --

    /// Send `payload` on the single-motor channel and wait for the reply that
    /// echoes its command byte on `0x240 + id`.
    fn transact(&mut self, payload: [u8; DATA_LEN]) -> Result<[u8; DATA_LEN]> {
        self.send_recv(
            can_id::command_id(self.motor_id),
            can_id::reply_id(self.motor_id),
            payload,
            Some(payload[0]),
        )
    }

    fn send_recv(
        &mut self,
        tx_id: u16,
        rx_id: u16,
        payload: [u8; DATA_LEN],
        echo_cmd: Option<u8>,
    ) -> Result<[u8; DATA_LEN]> {
        log::debug!("TX id=0x{:03X} data={:02X?}", tx_id, payload);
        self.bus.send(tx_id, &payload)?;
        let start = Instant::now();
        loop {
            if start.elapsed() > self.timeout {
                return Err(Error::Timeout {
                    motor_id: self.motor_id,
                });
            }
            match self.bus.recv() {
                Ok(frame) => {
                    if frame.can_id != rx_id || frame.data.len() < DATA_LEN {
                        continue;
                    }
                    if let Some(cmd) = echo_cmd {
                        if frame.data[0] != cmd {
                            continue;
                        }
                    }
                    log::debug!("RX id=0x{:03X} data={:02X?}", frame.can_id, frame.data);
                    let mut out = [0u8; DATA_LEN];
                    out.copy_from_slice(&frame.data[..DATA_LEN]);
                    return Ok(out);
                }
                Err(Error::Timeout { .. }) => continue,
                Err(e) => return Err(e),
            }
        }
    }

    // -- reads --

    /// Read Status1 (`0x9A`): temperature, voltage, error flags.
    pub fn read_status1(&mut self) -> Result<Status1> {
        let reply = self.transact(build_read_status1())?;
        parse_status1(&reply).ok_or_else(|| Error::InvalidResponse("bad 0x9A reply".into()))
    }

    /// Read Status2 (`0x9C`): temperature, iq, speed, coarse angle.
    pub fn read_status2(&mut self) -> Result<Status2> {
        let reply = self.transact(build_read_status2())?;
        parse_status2(&reply).ok_or_else(|| Error::InvalidResponse("bad 0x9C reply".into()))
    }

    /// Read the multi-turn absolute angle (`0x92`) in 0.01°/LSB (raw motor
    /// frame — the soft zero is *not* applied).
    pub fn read_multi_turn_centideg(&mut self) -> Result<i32> {
        let reply = self.transact(build_read_multi_turn_angle())?;
        parse_multi_turn_angle(&reply)
            .ok_or_else(|| Error::InvalidResponse("bad 0x92 reply".into()))
    }

    /// Read the system software version date (`0xB2`) as `YYYYMMDD`
    /// (e.g. `20211126`).
    pub fn read_version_date(&mut self) -> Result<u32> {
        let reply = self.transact(build_read_version_date())?;
        parse_version_date(&reply).ok_or_else(|| Error::InvalidResponse("bad 0xB2 reply".into()))
    }

    /// Read the motor model name (`0xB5`), trimmed of trailing NUL/space
    /// padding. Returns an owned `String` since the underlying bytes are
    /// ASCII but not guaranteed valid UTF-8 on malformed replies.
    pub fn read_motor_model(&mut self) -> Result<String> {
        let reply = self.transact(build_read_motor_model())?;
        let raw = parse_motor_model(&reply)
            .ok_or_else(|| Error::InvalidResponse("bad 0xB5 reply".into()))?;
        let end = raw
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(raw.len());
        Ok(String::from_utf8_lossy(&raw[..end]).trim().to_string())
    }

    /// Read current/speed/position-loop PID gains (`0x30`), all six at once.
    /// Each value is a 0-255 normalized unit — see [`PidGains`] for how to
    /// turn it into a physical gain.
    pub fn read_pid(&mut self) -> Result<PidGains> {
        let reply = self.transact(build_read_pid())?;
        parse_pid_gains(&reply).ok_or_else(|| Error::InvalidResponse("bad 0x30 reply".into()))
    }

    /// Read one acceleration/deceleration value (`0x42`) in 1 dps/s
    /// (manual range 50-60000).
    pub fn read_acceleration(&mut self, index: AccelIndex) -> Result<i32> {
        let reply = self.transact(build_read_acceleration(index))?;
        parse_acceleration(&reply).ok_or_else(|| Error::InvalidResponse("bad 0x42 reply".into()))
    }

    /// Slow status read for monitoring (voltage / error flags / brake).
    pub fn read_status(&mut self) -> Result<MotorStatus> {
        let s = self.read_status1()?;
        Ok(MotorStatus {
            temperature_c: s.temperature_c,
            voltage_v: s.voltage_v(),
            brake_released: s.brake_released,
            error: s.error,
        })
    }

    /// Accurate measurement: `0x92` for position + `0x9C` for
    /// velocity/current/temperature (two transactions).
    pub fn measure(&mut self) -> Result<MotorFeedback> {
        let centideg = self.read_multi_turn_centideg()?;
        let s = self.read_status2()?;
        let mut fb = self.feedback_from_status2(&s);
        fb.position_rad = self.centideg_to_position_rad(centideg as i64);
        Ok(fb)
    }

    // -- zeroing --

    /// Anchor position 0 to the motor's current location (soft zero — no NVM
    /// write). Required before [`Self::set_position`].
    pub fn rezero(&mut self) -> Result<()> {
        let centideg = self.read_multi_turn_centideg()?;
        self.zero_centideg = Some(centideg as i64);
        Ok(())
    }

    /// Write the current position to encoder ROM as the motor zero (`0x64`).
    /// Takes effect after [`Self::system_reset`]; wears flash — for routine
    /// re-zeroing use [`Self::rezero`] instead.
    pub fn set_zero_rom(&mut self) -> Result<()> {
        self.transact(build_set_zero_rom())?;
        Ok(())
    }

    // -- special commands --

    /// Stop motion but stay in closed-loop mode (`0x81`).
    pub fn stop(&mut self) -> Result<()> {
        self.transact(build_stop())?;
        self.enabled = false;
        Ok(())
    }

    /// Turn the output off and clear the running state (`0x80`).
    pub fn shutdown(&mut self) -> Result<()> {
        self.transact(build_shutdown())?;
        self.enabled = false;
        Ok(())
    }

    /// Restart the motor firmware (`0x76`). The motor drops off the bus while
    /// rebooting, so no reply is awaited.
    pub fn system_reset(&mut self) -> Result<()> {
        let payload = build_system_reset();
        self.bus
            .send(can_id::command_id(self.motor_id), &payload)?;
        Ok(())
    }

    /// Release the holding brake (`0x77`, brake-equipped models).
    pub fn brake_release(&mut self) -> Result<()> {
        self.transact(build_brake_release())?;
        Ok(())
    }

    /// Lock the holding brake (`0x78`, brake-equipped models).
    pub fn brake_lock(&mut self) -> Result<()> {
        self.transact(build_brake_lock())?;
        Ok(())
    }

    /// Function control (`0x20`): `index` 1 = clear multi-turn value (takes
    /// effect after restart), 2 = CANID-filter enable. See the V3.9 manual.
    pub fn function_control(&mut self, index: u8, value: i32) -> Result<()> {
        self.transact(build_function_control(index, value))?;
        Ok(())
    }

    // -- control --

    /// Position control (`0xA4`). `pos_rad` is relative to the last
    /// [`Self::rezero`] anchor; errors with
    /// [`Error::PositionNotAnchored`] when no anchor is set.
    /// `max_speed_rad_s` caps output-shaft speed (rounded to ≥ 1 dps so a
    /// small nonzero cap never degenerates to "uncapped").
    pub fn set_position(&mut self, pos_rad: f32, max_speed_rad_s: f32) -> Result<MotorFeedback> {
        let zero = self.zero_centideg.ok_or(Error::PositionNotAnchored {
            motor_id: self.motor_id,
        })?;
        let target = zero + (pos_rad * DEG_PER_RAD * 100.0) as i64;
        let target = target.clamp(i32::MIN as i64, i32::MAX as i64) as i32;
        let max_speed_dps = (max_speed_rad_s.abs() * DEG_PER_RAD)
            .clamp(1.0, u16::MAX as f32) as u16;
        let reply = self.transact(build_position_control(max_speed_dps, target))?;
        self.parse_control_reply(&reply)
    }

    /// Velocity control (`0xA2`), output-shaft rad/s.
    pub fn set_velocity(&mut self, vel_rad_s: f32) -> Result<MotorFeedback> {
        let centi_dps = (vel_rad_s * DEG_PER_RAD * 100.0) as i32;
        let reply = self.transact(build_speed_control(centi_dps))?;
        self.parse_control_reply(&reply)
    }

    /// Torque control (`0xA1`) in output-frame N·m via `Kt`. **Latches**: the
    /// motor keeps applying the torque until another command arrives.
    pub fn set_torque(&mut self, torque_nm: f32) -> Result<MotorFeedback> {
        self.set_current(torque_nm / self.config.torque_constant_nm_per_a)
    }

    /// Current control (`0xA1`) — direct `iq` in A, bypassing `Kt`.
    pub fn set_current(&mut self, current_a: f32) -> Result<MotorFeedback> {
        let centi_amps = (current_a * 100.0)
            .clamp(i16::MIN as f32, i16::MAX as f32) as i16;
        let reply = self.transact(build_torque_control(centi_amps))?;
        self.parse_control_reply(&reply)
    }

    /// Native motion-mode (MIT) control on the `0x400 + ID` channel:
    /// `iq = kp·(pos − p) + kd·(vel − v) + t_ff` (per-protocol fixed ranges,
    /// see [`myactuator_protocol::motion`]). The soft zero is applied to the
    /// commanded and reported position. Requires motion-mode-capable firmware
    /// (RMD-X V3); a timeout here usually means the firmware lacks the mode.
    pub fn motion_control(
        &mut self,
        pos_rad: f32,
        vel_rad_s: f32,
        kp: f32,
        kd: f32,
        torque_ff_nm: f32,
    ) -> Result<MotionFeedback> {
        let zero_rad = self.zero_offset_rad();
        let payload = build_motion_control(pos_rad + zero_rad, vel_rad_s, kp, kd, torque_ff_nm);
        let reply = self.send_recv(
            can_id::motion_id(self.motor_id),
            can_id::motion_reply_id(self.motor_id),
            payload,
            None,
        )?;
        let mut fb = parse_motion_reply(&reply)
            .ok_or_else(|| Error::InvalidResponse("bad motion reply".into()))?;
        fb.position_rad -= zero_rad;
        Ok(fb)
    }

    // -- conversions --

    fn zero_offset_rad(&self) -> f32 {
        self.zero_centideg.unwrap_or(0) as f32 / 100.0 / DEG_PER_RAD
    }

    fn centideg_to_position_rad(&self, centideg: i64) -> f32 {
        (centideg - self.zero_centideg.unwrap_or(0)) as f32 / 100.0 / DEG_PER_RAD
    }

    fn parse_control_reply(&mut self, reply: &[u8]) -> Result<MotorFeedback> {
        let s = parse_status2(reply)
            .ok_or_else(|| Error::InvalidResponse("bad control reply".into()))?;
        Ok(self.feedback_from_status2(&s))
    }

    /// Status2 → SI feedback. The inline angle is a coarse 1°/LSB value, so
    /// position from control replies has ~1° resolution; [`Self::measure`]
    /// replaces it with the 0.01° `0x92` reading.
    fn feedback_from_status2(&self, s: &Status2) -> MotorFeedback {
        let current_a = s.current_a();
        MotorFeedback {
            position_rad: self.centideg_to_position_rad(s.angle_deg as i64 * 100),
            velocity_rad_per_s: s.speed_dps as f32 / DEG_PER_RAD,
            torque_nm: current_a * self.config.torque_constant_nm_per_a,
            current_a,
            temperature_c: s.temperature_c,
        }
    }
}
