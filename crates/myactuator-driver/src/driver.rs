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

use misa_actuator::Parameter;

use myactuator_protocol::{
    build_brake_lock, build_brake_release, build_commit_params, build_function_control,
    build_motion_control, build_position_control, build_read_acceleration, build_read_motor_model,
    build_read_motor_model_chunk, parse_motor_model_chunk, MOTOR_MODEL_CHUNKS,
    build_read_motor_power, build_read_multi_turn_angle, build_read_multi_turn_encoder,
    build_read_multi_turn_encoder_raw, build_read_multi_turn_zero_offset, build_read_param,
    build_read_pid, build_read_run_mode, build_read_single_turn_angle,
    build_read_single_turn_encoder, build_read_status1, build_read_status2, build_read_status3,
    build_read_uptime, build_read_version_date, build_set_zero_rom, build_shutdown,
    build_speed_control, build_stop, build_system_reset, build_torque_control, build_write_param,
    can_id, parse_acceleration, parse_motion_reply, parse_motor_model, parse_motor_power,
    parse_multi_turn_angle, parse_multi_turn_encoder, parse_multi_turn_encoder_raw,
    parse_multi_turn_zero_offset, parse_param_value, parse_pid_value, parse_run_mode,
    parse_single_turn_angle, parse_single_turn_encoder, parse_status1, parse_status2,
    parse_status3, parse_uptime, parse_version_date, AccelIndex, ErrorState, MotionFeedback,
    ParamIndex, PidGains, PidIndex, RunMode, SingleTurnEncoder, Status1, Status2, Status3,
    DATA_LEN,
};

use crate::bus::{AnyCanBus, MyActuatorBus};
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
pub struct MyActuatorMotor<B: MyActuatorBus = AnyCanBus> {
    bus: B,
    motor_id: u8,
    config: MotorConfig,
    timeout: Duration,
    /// Soft zero: the `0x92` multi-turn angle (0.01°/LSB) captured at the last
    /// [`Self::rezero`]. Commanded positions add it, reported ones subtract it.
    zero_centideg: Option<i64>,
    /// Whether control replies should carry the fine position.
    ///
    /// Off by default because it costs a second transaction per command. See
    /// [`Self::set_fine_position`] for what it buys and when it is worth it.
    fine_position: bool,
    /// Tracked enable state (see `Actuator::is_enabled_hint`).
    pub(crate) enabled: bool,
}

impl MyActuatorMotor<AnyCanBus> {
    /// Open a CAN interface (`"can0"`, `"pcan:usb1"`, `"slcan:COM5"`, ...) and
    /// bind to one motor. See [`AnyCanBus::open`] for the accepted forms.
    pub fn open(interface: &str, motor_id: u8, config: MotorConfig) -> Result<Self> {
        let bus = AnyCanBus::open(interface)?;
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
            fine_position: false,
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
        // Chunked first: the name does not fit in one frame, and a V4.4-era
        // firmware answers only the indexed form. A real RMD-X4-P36-36 returns
        // nothing usable from the single-shot request the older manuals
        // document, which is why this used to report an empty model on a motor
        // whose vendor tool displays the name perfectly well.
        let mut name = String::new();
        let mut chunked_ok = false;
        for index in 1..=MOTOR_MODEL_CHUNKS {
            let reply = match self.transact(build_read_motor_model_chunk(index)) {
                Ok(r) => r,
                Err(_) => break,
            };
            let Some((echoed, chunk)) = parse_motor_model_chunk(&reply) else {
                break;
            };
            log::debug!("0xB5 chunk {index}: echoed={echoed} raw={chunk:02X?}");
            // The index is echoed; a mismatch means a stale reply, and
            // appending it would scramble the name rather than fail.
            if echoed != index {
                break;
            }
            chunked_ok = true;
            for &b in &chunk {
                if b == 0 {
                    break;
                }
                name.push(b as char);
            }
        }
        let trimmed = name.trim().to_string();
        if chunked_ok && !trimmed.is_empty() {
            return Ok(trimmed);
        }

        // Fall back to the single-shot form for firmwares that use it.
        let reply = self.transact(build_read_motor_model())?;
        let raw = parse_motor_model(&reply)
            .ok_or_else(|| Error::InvalidResponse("bad 0xB5 reply".into()))?;
        let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
        Ok(String::from_utf8_lossy(&raw[..end]).trim().to_string())
    }

    /// Read one PID gain (`0x30`), selected by [`PidIndex`] (V4.2+ indexed
    /// `Float` protocol — see [`PidGains`]'s doc comment for why this
    /// supersedes the older six-at-once `uint8` layout).
    pub fn read_pid_gain(&mut self, index: PidIndex) -> Result<f32> {
        let reply = self.transact(build_read_pid(index))?;
        parse_pid_value(&reply).ok_or_else(|| Error::InvalidResponse("bad 0x30 reply".into()))
    }

    /// Read all seven PID gains (`0x30`, one transaction per [`PidIndex`]).
    pub fn read_pid(&mut self) -> Result<PidGains> {
        Ok(PidGains {
            current_kp: self.read_pid_gain(PidIndex::CurrentKp)?,
            current_ki: self.read_pid_gain(PidIndex::CurrentKi)?,
            speed_kp: self.read_pid_gain(PidIndex::SpeedKp)?,
            speed_ki: self.read_pid_gain(PidIndex::SpeedKi)?,
            position_kp: self.read_pid_gain(PidIndex::PositionKp)?,
            position_ki: self.read_pid_gain(PidIndex::PositionKi)?,
            position_kd: self.read_pid_gain(PidIndex::PositionKd)?,
        })
    }

    /// Read one parameter from the undocumented `0xC0` indexed space,
    /// selected by [`ParamIndex`] — Protect/Plan/Motor Parameters and a
    /// Every setting the motor will report, grouped for display.
    ///
    /// The documented per-topic commands always. With `deep`, also the `0xC0`
    /// indexed block, which was reverse-engineered from Setup Software V4.0
    /// traffic — it holds the reduction ratio, `KT_OUT`, the current limits
    /// and the second PID gain set, none of which appear anywhere documented.
    pub fn read_parameters(&mut self, deep: bool) -> Vec<Parameter> {
        let mut out = Vec::new();

        // The documented commands, each read individually so one absent
        // command does not cost the rest. `0x90` is direct-drive only and
        // `0x71` is missing on some firmwares — both come back as an
        // explained gap rather than an absence.
        match self.read_status1() {
            Ok(s) => {
                out.push(Parameter::float("health", "temperature", s.temperature_c as f32, "°C", "0x9A"));
                out.push(Parameter::float("health", "mos_temperature", s.mos_temperature_c as f32, "°C", "0x9A"));
                out.push(Parameter::float("health", "voltage", s.voltage_v(), "V", "0x9A"));
                out.push(Parameter::int("health", "brake_released", s.brake_released as i64, "", "0x9A"));
                out.push(Parameter::int("health", "error_flags", s.error.0 as i64, "", "0x9A"));
            }
            Err(e) => out.push(Parameter::unavailable("health", "status1", e.to_string(), "", "0x9A")),
        }

        for (idx, label) in [
            (PidIndex::CurrentKp, "current_kp"),
            (PidIndex::CurrentKi, "current_ki"),
            (PidIndex::SpeedKp, "speed_kp"),
            (PidIndex::SpeedKi, "speed_ki"),
            (PidIndex::PositionKp, "position_kp"),
            (PidIndex::PositionKi, "position_ki"),
            (PidIndex::PositionKd, "position_kd"),
        ] {
            out.push(match self.read_pid_gain(idx) {
                Ok(v) => Parameter::float("PID gains", label, v, "", "0x30"),
                Err(e) => Parameter::unavailable("PID gains", label, e.to_string(), "", "0x30"),
            });
        }

        for (idx, label) in [
            (AccelIndex::PositionAccel, "position_accel"),
            (AccelIndex::PositionDecel, "position_decel"),
            (AccelIndex::SpeedAccel, "speed_accel"),
            (AccelIndex::SpeedDecel, "speed_decel"),
        ] {
            out.push(match self.read_acceleration(idx) {
                Ok(v) => Parameter::int("motion profile", label, v as i64, "dps/s", "0x42"),
                Err(e) => {
                    Parameter::unavailable("motion profile", label, e.to_string(), "dps/s", "0x42")
                }
            });
        }

        out.push(match self.read_version_date() {
            Ok(v) => Parameter::int("identity", "version_date", v as i64, "", "0xB2"),
            Err(e) => Parameter::unavailable("identity", "version_date", e.to_string(), "", "0xB2"),
        });
        out.push(match self.read_motor_model() {
            Ok(v) if !v.is_empty() => Parameter::text("identity", "motor_model", v, "0xB5"),
            Ok(_) => Parameter::unavailable("identity", "motor_model", "empty", "", "0xB5"),
            Err(e) => Parameter::unavailable("identity", "motor_model", e.to_string(), "", "0xB5"),
        });

        if !deep {
            return out;
        }

        // (index, label, unit)
        const C0: &[(ParamIndex, &str, &str)] = &[
            (ParamIndex::MotorNumber, "motor_number", ""),
            (ParamIndex::FactoryTime, "factory_time", ""),
            (ParamIndex::ReductionRatio, "reduction_ratio", ""),
            (ParamIndex::KtOut, "kt_out", "N·m/A"),
            (ParamIndex::PolePairs, "pole_pairs", ""),
            (ParamIndex::SingleResolutionPulses, "single_resolution", "pulses"),
            (ParamIndex::RatedCurrent, "rated_current", "A"),
            (ParamIndex::MaxCurrent, "max_current", "A"),
            (ParamIndex::StallCurrent, "stall_current", "A"),
            (ParamIndex::OverVoltage, "over_voltage", "V"),
            (ParamIndex::LowVoltage, "low_voltage", "V"),
            (ParamIndex::ShutdownTemp, "shutdown_temp", "°C"),
            (ParamIndex::ResumeTemp, "resume_temp", "°C"),
            (ParamIndex::MaxSpeed, "max_speed", "rpm"),
            (ParamIndex::NominalSpeed, "nominal_speed", "rpm"),
            (ParamIndex::StallTimeLimit, "stall_time_limit", "s"),
            (ParamIndex::MaxPositivePosition, "max_positive_position", "°"),
            (ParamIndex::MinNegativePosition, "min_negative_position", "°"),
            (ParamIndex::PositionPlanMaxSpeed, "position_plan_max_speed", "rpm"),
            (ParamIndex::PositionPlanMaxAcc, "position_plan_max_acc", "dps/s"),
            (ParamIndex::PositionPlanMaxDec, "position_plan_max_dec", "dps/s"),
            (ParamIndex::SpeedPlanMaxAcc, "speed_plan_max_acc", "dps/s"),
            (ParamIndex::SpeedPlanMaxDec, "speed_plan_max_dec", "dps/s"),
            (ParamIndex::MotorPositionZero, "motor_position_zero", ""),
            (ParamIndex::BrakeMode, "brake_mode", ""),
            (ParamIndex::ChangeMotorDirection, "change_motor_direction", ""),
            (ParamIndex::AutomaticErrorRecovery, "automatic_error_recovery", ""),
        ];
        for &(idx, label, unit) in C0 {
            let address = format!("0xC0[0x{:02X}]", idx as u8);
            out.push(
                match self.read_param(idx) {
                    Ok(v) => Parameter::float("0xC0 block (undocumented)", label, v, unit, address),
                    Err(e) => Parameter::unavailable(
                        "0xC0 block (undocumented)",
                        label,
                        e.to_string(),
                        unit,
                        address,
                    ),
                }
                .undocumented(),
            );
        }
        out
    }

    /// Adopt the torque constant the motor reports about itself
    /// (`KT_OUT`, `0xC0`), and return it.
    ///
    /// Kt is the only electrical constant this driver needs: V3 speaks
    /// output-shaft units on the wire, so torque is just `current × Kt` in
    /// both directions. Getting it wrong scales every torque command and every
    /// reported torque by the ratio, silently — a wrong `1.0` against a real
    /// `1.1` is a 10% error that looks like nothing, and a wrong `0.1` is a
    /// factor of eleven that looks like a broken motor.
    ///
    /// The motor knows the value. A real RMD reports `KT_OUT = 1.1 N·m/A`,
    /// while `--kt` defaults to "current units" and otherwise expects a human
    /// to type the number — which is a worse source than the register.
    ///
    /// Reading it also rules out the failure the default hides: `0xB5`, the
    /// documented "motor model" command, comes back **empty** on this
    /// firmware, so there is no model name to look a datasheet value up by.
    pub fn refresh_torque_constant(&mut self) -> Result<f32> {
        let kt = self.read_param(ParamIndex::KtOut)?;
        if !(kt.is_finite() && kt > 0.0) {
            return Err(Error::InvalidResponse(format!(
                "motor {} reported KT_OUT = {kt}, which cannot scale torque; \
                 keeping the previous value",
                self.motor_id()
            )));
        }
        self.config.torque_constant_nm_per_a = kt;
        log::info!("myactuator motor {}: KT_OUT = {kt} N·m/A", self.motor_id());
        Ok(kt)
    }

    /// second PID-gain set (Kd/R(Slope)/T(Filter) per loop), reverse
    /// engineered from Setup Software V4.0 traffic. See
    /// `myactuator-protocol/doc/setup-software-c0-param-protocol.md`.
    pub fn read_param(&mut self, index: ParamIndex) -> Result<f32> {
        let reply = self.transact(build_read_param(index))?;
        parse_param_value(&reply).ok_or_else(|| Error::InvalidResponse("bad 0xC0 read reply".into()))
    }

    /// Write one parameter to RAM (`0xC0`, undocumented). Lost on power-cycle
    /// unless followed by [`Self::commit_params`]. [`ParamIndex::EnableCanFilter`]
    /// is known **not** to take effect via this path on real hardware — see
    /// the doc referenced on [`Self::read_param`], §1.5.
    pub fn write_param(&mut self, index: ParamIndex, value: f32) -> Result<()> {
        self.transact(build_write_param(index, value))?;
        Ok(())
    }

    /// Commit all `0xC0` RAM writes to flash (`0xC1`, undocumented).
    pub fn commit_params(&mut self) -> Result<()> {
        self.transact(build_commit_params())?;
        Ok(())
    }

    /// Read one acceleration/deceleration value (`0x42`) in 1 dps/s (manual
    /// range 100-60000 per the X4-36 V4.3 manual; the plain V3.9 manual's own
    /// text is internally inconsistent, citing 50 in one section and 100 in
    /// another for the same field).
    pub fn read_acceleration(&mut self, index: AccelIndex) -> Result<i32> {
        let reply = self.transact(build_read_acceleration(index))?;
        parse_acceleration(&reply).ok_or_else(|| Error::InvalidResponse("bad 0x42 reply".into()))
    }

    /// Read the multi-turn encoder position (`0x60`), pulses, zero offset applied.
    pub fn read_multi_turn_encoder(&mut self) -> Result<i32> {
        let reply = self.transact(build_read_multi_turn_encoder())?;
        parse_multi_turn_encoder(&reply)
            .ok_or_else(|| Error::InvalidResponse("bad 0x60 reply".into()))
    }

    /// Read the raw multi-turn encoder position (`0x61`), pulses, no zero offset.
    pub fn read_multi_turn_encoder_raw(&mut self) -> Result<i32> {
        let reply = self.transact(build_read_multi_turn_encoder_raw())?;
        parse_multi_turn_encoder_raw(&reply)
            .ok_or_else(|| Error::InvalidResponse("bad 0x61 reply".into()))
    }

    /// Read the multi-turn encoder's zero-offset value (`0x62`), pulses.
    pub fn read_multi_turn_zero_offset(&mut self) -> Result<i32> {
        let reply = self.transact(build_read_multi_turn_zero_offset())?;
        parse_multi_turn_zero_offset(&reply)
            .ok_or_else(|| Error::InvalidResponse("bad 0x62 reply".into()))
    }

    /// Read the current run mode (`0x70`).
    pub fn read_run_mode(&mut self) -> Result<RunMode> {
        let reply = self.transact(build_read_run_mode())?;
        parse_run_mode(&reply).ok_or_else(|| Error::InvalidResponse("bad 0x70 reply".into()))
    }

    /// Read the motor's instantaneous output power (`0x71`) in watts. Documented
    /// in the plain V3.9 manual but absent from the X4-36 V4.3 manual;
    /// confirmed on real X4-36 hardware to simply time out (not implemented
    /// by that firmware).
    pub fn read_motor_power(&mut self) -> Result<f32> {
        let reply = self.transact(build_read_motor_power())?;
        parse_motor_power(&reply).ok_or_else(|| Error::InvalidResponse("bad 0x71 reply".into()))
    }

    /// Read the single-turn encoder position (`0x90`, direct-drive models).
    /// Confirmed on real X4-36 (a geared model) hardware to simply time out —
    /// this command appears to only be meaningful on direct-drive variants.
    pub fn read_single_turn_encoder(&mut self) -> Result<SingleTurnEncoder> {
        let reply = self.transact(build_read_single_turn_encoder())?;
        parse_single_turn_encoder(&reply)
            .ok_or_else(|| Error::InvalidResponse("bad 0x90 reply".into()))
    }

    /// Read the single-turn angle (`0x94`) in 0.01°/LSB (0-359.99°). Unlike
    /// `0x90`, real X4-36 (geared) hardware *does* reply to this one, but
    /// with an out-of-range `0xFFFF` (655.35°) sentinel rather than a real
    /// angle — treat any value ≥ 36000 as "not meaningful on this model"
    /// rather than a real single-turn position.
    pub fn read_single_turn_angle(&mut self) -> Result<u16> {
        let reply = self.transact(build_read_single_turn_angle())?;
        parse_single_turn_angle(&reply)
            .ok_or_else(|| Error::InvalidResponse("bad 0x94 reply".into()))
    }

    /// Read Status3 (`0x9D`): temperature + per-phase current.
    pub fn read_status3(&mut self) -> Result<Status3> {
        let reply = self.transact(build_read_status3())?;
        parse_status3(&reply).ok_or_else(|| Error::InvalidResponse("bad 0x9D reply".into()))
    }

    /// Read system uptime since last reset/reboot (`0xB1`), in ms.
    pub fn read_uptime_ms(&mut self) -> Result<u32> {
        let reply = self.transact(build_read_uptime())?;
        parse_uptime(&reply).ok_or_else(|| Error::InvalidResponse("bad 0xB1 reply".into()))
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
    ///
    /// **Real-hardware caveat** (index 2, disable direction): sending
    /// `index=2, value=0` to disable the CAN-ID filter, even followed by
    /// [`Self::commit_params`], was observed on a real X4-36 to **not** take
    /// effect — a subsequent read of [`ParamIndex::EnableCanFilter`] via
    /// [`Self::read_param`] still reported the filter enabled. Unclear
    /// whether a power-cycle is required or this direction is simply
    /// unsupported on this firmware. See
    /// `myactuator-protocol/doc/setup-software-c0-param-protocol.md` §1.5.
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
        let mut fb = self.feedback_from_status2(&s);
        if self.fine_position {
            // Same substitution `measure` makes, on request. A failed read
            // leaves the coarse value rather than failing the command: the
            // control itself succeeded, and losing a motion command over a
            // resolution upgrade would be the worse trade.
            if let Ok(centideg) = self.read_multi_turn_centideg() {
                fb.position_rad = self.centideg_to_position_rad(centideg as i64);
            }
        }
        Ok(fb)
    }

    /// Ask control replies to carry the **fine** position (`0x92`, 0.01°/LSB)
    /// instead of the coarse one Status2 inlines (1°/LSB).
    ///
    /// Off by default: it costs a second transaction per command, halving the
    /// rate a control loop can reach. Streaming and chirps want the rate and do
    /// not care about the last degree; quasi-static runs are the opposite, and
    /// dwell long enough that the extra read is free.
    ///
    /// Without it, everything a run learns about position through a control
    /// reply is quantised to 1° — about 0.017 rad. That is coarser than the
    /// breakaway map's own 0.01 rad arrival tolerance, so the travel between
    /// positions could never register as arrived, and it showed on the bench as
    /// a shaft that appeared to jump a degree the moment streaming started and
    /// back the moment it stopped (2026-08-08, an RMD-X4-P36-36).
    pub fn set_fine_position(&mut self, on: bool) {
        self.fine_position = on;
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
