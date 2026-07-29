//! High-level DAMIAO motor driver, generic over the [`DamiaoBus`] transport.
//!
//! # Watchdog / comm-loss
//!
//! DAMIAO motors run a communication watchdog: if no command arrives within
//! the firmware timeout they fault with `CommLost` (`0xD`) and auto-disable.
//! Real-time control loops must therefore send commands continuously; a single
//! `enable()` is not enough to keep the motor live.
//!
//! # Feedback addressing
//!
//! Feedback frames are returned on the motor's **Master ID** (`MST_ID`,
//! default `0`), not on the command id. On a multi-motor bus every motor must
//! be given a unique `MST_ID` (see [`DamiaoMotor::write_register_int`] with
//! [`Rid::MST_ID`]) or replies cannot be told apart.

use std::time::{Duration, Instant};

use damiao_protocol::{
    build_clear_error_frame, build_disable_frame, build_enable_frame, build_mit_frame,
    build_pos_vel_frame, build_read_reg, build_save_all, build_set_zero_frame, build_vel_frame,
    build_write_reg_f32, build_write_reg_int, parse_feedback, parse_reg_reply, ControlMode,
    Feedback, Limits, ModelReg, MotorModel, RegReply, Rid, DATA_LEN, DEFAULT_MASTER_ID,
    REGISTER_ID,
};

use crate::bus::{DamiaoBus, SocketCanBus, SocketCanFdBus};
use crate::error::{Error, Result};

/// Default per-request timeout.
const DEFAULT_TIMEOUT: Duration = Duration::from_millis(100);

/// Max attempts when verifying a control-mode switch (matches the SDK's retry
/// loop on `CTRL_MODE`).
const MODE_SWITCH_RETRIES: u32 = 20;

/// High-level DAMIAO motor handle.
pub struct DamiaoMotor<B: DamiaoBus = SocketCanBus> {
    bus: B,
    /// The motor's listen CAN_ID (ESC_ID).
    can_id: u8,
    /// The motor's feedback CAN id (MST_ID). `0` = accept any responder.
    master_id: u16,
    model: MotorModel,
    limits: Limits,
    mode: ControlMode,
    /// Whether [`Self::mode`] reflects a mode we have actually written to /
    /// read back from the motor (vs. the initial assumption). Until this is
    /// `true`, `switch_mode` always issues the register write — the motor may
    /// have powered up in a different saved mode than we assume.
    mode_confirmed: bool,
    enabled: bool,
    timeout: Duration,
    /// Soft (in-memory) zero offset, in the motor's **raw** position frame
    /// (rad). Commanded positions add it; reported positions subtract it. Set
    /// by [`DamiaoMotor::set_zero`] so we avoid wearing the motor's NVM.
    soft_zero: f32,
    /// Last control frame sent (id, payload), re-issued by `measure()` to read
    /// state without disturbing the active setpoint.
    last_cmd: Option<(u16, [u8; DATA_LEN])>,
    /// Torque constant (N·m/A) for deriving current from reported torque, or
    /// `None` while unknown. Populated by
    /// [`DamiaoMotor::refresh_torque_constant`]; see that method for why it is
    /// not read eagerly.
    torque_constant: Option<f32>,
}

impl DamiaoMotor<SocketCanBus> {
    /// Open a **classic CAN** interface and bind to one motor.
    ///
    /// `can_id` is the motor's CAN_ID; the Master ID defaults to
    /// [`DEFAULT_MASTER_ID`] (`0`).
    pub fn open(interface: &str, can_id: u8, model: MotorModel) -> Result<Self> {
        let bus = SocketCanBus::open(interface)?;
        Ok(Self::with_bus(bus, can_id, model))
    }

    /// Open a **classic CAN** interface binding to a motor with an explicit
    /// Master ID — use this on a multi-motor bus where each motor has a unique
    /// `MST_ID` (see the crate docs on the `0x10 + CAN_ID` convention).
    pub fn open_with_master(
        interface: &str,
        can_id: u8,
        master_id: u16,
        model: MotorModel,
    ) -> Result<Self> {
        let bus = SocketCanBus::open(interface)?;
        Ok(Self::with_bus_and_master(bus, can_id, master_id, model))
    }
}

impl DamiaoMotor<SocketCanFdBus> {
    /// Open a **CAN-FD** interface and bind to one motor. The interface must be
    /// configured with `fd on` (see [`SocketCanFdBus`]).
    pub fn open_fd(interface: &str, can_id: u8, model: MotorModel) -> Result<Self> {
        let bus = SocketCanFdBus::open(interface)?;
        Ok(Self::with_bus(bus, can_id, model))
    }

    /// Open a **CAN-FD** interface with an explicit Master ID (multi-motor bus).
    pub fn open_fd_with_master(
        interface: &str,
        can_id: u8,
        master_id: u16,
        model: MotorModel,
    ) -> Result<Self> {
        let bus = SocketCanFdBus::open(interface)?;
        Ok(Self::with_bus_and_master(bus, can_id, master_id, model))
    }
}

impl<B: DamiaoBus> DamiaoMotor<B> {
    /// Wrap an already-open bus with the default Master ID (`0`).
    pub fn with_bus(bus: B, can_id: u8, model: MotorModel) -> Self {
        Self::with_bus_and_master(bus, can_id, DEFAULT_MASTER_ID, model)
    }

    /// Wrap an already-open bus with an explicit Master ID.
    pub fn with_bus_and_master(bus: B, can_id: u8, master_id: u16, model: MotorModel) -> Self {
        Self {
            bus,
            can_id,
            master_id,
            model,
            limits: model.limits(),
            mode: ControlMode::Mit,
            mode_confirmed: false,
            enabled: false,
            timeout: DEFAULT_TIMEOUT,
            soft_zero: 0.0,
            last_cmd: None,
            torque_constant: None,
        }
    }

    // -- accessors --

    /// The motor's CAN_ID.
    pub fn can_id(&self) -> u8 {
        self.can_id
    }
    /// The motor's Master ID (feedback id). `0` means "accept any responder".
    pub fn master_id(&self) -> u16 {
        self.master_id
    }
    /// Set the Master ID this driver matches feedback frames against. Use this
    /// when a motor has been configured with a non-default `MST_ID`. Pass `0`
    /// to accept any responder (matched by the feedback id nibble only).
    pub fn set_master_id(&mut self, master_id: u16) {
        self.master_id = master_id;
    }
    /// The configured motor model.
    pub fn model(&self) -> MotorModel {
        self.model
    }
    /// The quantization limits in effect.
    pub fn limits(&self) -> Limits {
        self.limits
    }

    /// The torque constant in effect (N·m/A), or `None` while unknown.
    ///
    /// Used to derive `current_a` in [`misa_actuator::MotorFeedback`], which is
    /// otherwise `NaN` on this family: the DAMIAO feedback frame reports torque
    /// but carries no current field.
    pub fn torque_constant(&self) -> Option<f32> {
        self.torque_constant
    }

    /// Set the torque constant used to derive current (N·m/A).
    ///
    /// Prefer [`Self::refresh_torque_constant`], which reads the motor's own
    /// value. Use this when the motor's `KT_Value` register is unset — a bench
    /// DM-J4310 reports `0` — and you have the figure from elsewhere (the
    /// datasheet, or a `misa_sysid::run_kt` sweep on a motor that does
    /// report current). Values that are not finite and positive are rejected,
    /// leaving the constant unknown rather than poisoning every later reading.
    pub fn set_torque_constant(&mut self, kt_nm_per_a: f32) {
        self.torque_constant = (kt_nm_per_a.is_finite() && kt_nm_per_a > 0.0).then_some(kt_nm_per_a);
    }

    /// Read the motor's `KT_Value` register (RID `0x01`) and adopt it as the
    /// torque constant, returning what was decided.
    ///
    /// `Ok(None)` means the motor answered but the value is unusable — a bench
    /// DM-J4310 reports `0.0`, i.e. the constant was never identified on that
    /// unit. That is reported rather than treated as an error, because the read
    /// itself succeeded and the caller may legitimately continue without
    /// current readings (or supply one via [`Self::set_torque_constant`]).
    ///
    /// Not read during construction on purpose: it costs a bus round-trip, and
    /// a driver that cannot be built without one would be unusable for the
    /// scan/probe paths that run before a motor is known to be present.
    pub fn refresh_torque_constant(&mut self) -> Result<Option<f32>> {
        let kt = self.read_register(Rid::KT_VALUE)?.as_f32();
        self.set_torque_constant(kt);
        if self.torque_constant.is_none() {
            log::warn!(
                "damiao motor {}: KT_Value reads {} — current cannot be derived from torque; \
                 set it explicitly if you know it",
                self.can_id,
                kt
            );
        }
        Ok(self.torque_constant)
    }

    /// Overwrite the MIT quantization limits in effect.
    ///
    /// Prefer [`Self::refresh_limits_from_registers`], which reads them from
    /// the motor. Use this only when the values come from somewhere the driver
    /// cannot reach (a saved config, a model not in [`MotorModel`]).
    pub fn set_limits(&mut self, limits: Limits) {
        self.limits = limits;
    }
    /// Driver's tracked control mode.
    pub fn mode(&self) -> ControlMode {
        self.mode
    }
    /// Driver's tracked enabled flag.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
    /// Current soft-zero offset (raw-frame rad).
    pub fn soft_zero(&self) -> f32 {
        self.soft_zero
    }
    /// Set the soft-zero offset directly (raw-frame rad). Mostly for restoring
    /// a previously-saved calibration; prefer [`Self::set_zero`].
    pub fn set_soft_zero(&mut self, raw_offset: f32) {
        self.soft_zero = raw_offset;
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

    fn send(&mut self, can_id: u16, data: &[u8]) -> Result<()> {
        log::debug!("TX id=0x{:03X} data={:02X?}", can_id, data);
        self.bus.send(can_id, data)
    }

    /// Wait for the next feedback frame for this motor and decode it.
    ///
    /// Matching rules (mirroring the reference DAMIAO driver):
    /// - if `master_id != 0`, the frame's standard id must equal it;
    /// - the feedback payload's byte-0 low nibble (the motor-id field) must
    ///   match `can_id`'s low nibble.
    ///
    /// The id nibble is only 4 bits, so motors whose `CAN_ID`s share a low
    /// nibble must be given distinct `MST_ID`s to be told apart.
    ///
    /// The status nibble (including fault codes) is preserved in
    /// [`Feedback::err`] rather than raised as an error — DAMIAO reports a
    /// status code on *every* frame, so callers that want to react to faults
    /// inspect `fb.err` (see [`crate::error::Error::MotorFault`] for the typed
    /// variant a higher layer can raise).
    fn recv_feedback(&mut self) -> Result<Feedback> {
        let start = Instant::now();
        loop {
            if start.elapsed() > self.timeout {
                return Err(Error::Timeout {
                    motor_id: self.can_id,
                });
            }
            match self.bus.recv() {
                Ok(frame) => {
                    if self.master_id != 0 && frame.can_id != self.master_id {
                        continue;
                    }
                    if frame.data.len() < DATA_LEN {
                        continue;
                    }
                    if (frame.data[0] & 0x0F) != (self.can_id & 0x0F) {
                        continue;
                    }
                    let mut fb = parse_feedback(&frame.data, &self.limits)
                        .ok_or(Error::InvalidResponse("short feedback frame".into()))?;
                    // Report position relative to the soft zero.
                    fb.position -= self.soft_zero;
                    return Ok(fb);
                }
                Err(Error::Timeout { .. }) => continue,
                Err(e) => return Err(e),
            }
        }
    }

    /// Best-effort feedback read that tolerates a missing reply (used after
    /// enable/disable, where some firmware does not always answer).
    fn try_recv_feedback(&mut self) -> Result<Option<Feedback>> {
        match self.recv_feedback() {
            Ok(fb) => Ok(Some(fb)),
            Err(Error::Timeout { .. }) => Ok(None),
            Err(e) => Err(e),
        }
    }

    // -- special commands --

    /// Enable closed-loop control (`FF*7 FC`). Returns feedback if the motor
    /// replies.
    ///
    /// Clears any latched fault first (`FF*7 FB`): a faulted motor silently
    /// ignores control commands while still reporting feedback — observed on the
    /// DM-J3507, where a stale fault made MIT commands produce no torque. The
    /// clear is harmless when there is no fault.
    pub fn enable(&mut self) -> Result<Option<Feedback>> {
        let (cid, cdata) = build_clear_error_frame(self.can_id);
        self.send(cid, &cdata)?;
        let _ = self.try_recv_feedback();
        let (id, data) = build_enable_frame(self.can_id);
        self.send(id, &data)?;
        let fb = self.try_recv_feedback()?;
        self.enabled = true;
        Ok(fb)
    }

    /// Disable closed-loop control (`FF*7 FD`). The motor coasts.
    ///
    /// **This is not a latching safe state.** Verified on a DM-J4310: sending
    /// a MIT frame with `kp = 5` immediately after `disable()` produced
    /// 1.013 N·m — the control frame re-energizes the motor. So `disable()`
    /// de-energizes only until the next control frame arrives; it does not
    /// make the motor ignore commands.
    ///
    /// Consequences worth knowing:
    /// - Any later `mit`/`move_to`/`set_velocity` call silently re-energizes,
    ///   even if nothing called `enable()` in between.
    /// - [`Self::measure`] and anything else that re-issues a control frame
    ///   (including a zero-gain MIT frame) counts as such a command. Those
    ///   produce no torque at zero gains, but the motor reports
    ///   [`ErrorCode::Enabled`](damiao_protocol::ErrorCode::Enabled) again.
    /// - The bench unit also reported `Enabled` from power-on, before any
    ///   `enable()` was ever sent, so do not treat a fresh connection as
    ///   de-energized either.
    ///
    /// For a state the motor will actually hold, cut power. To stop motion
    /// while keeping the link up, keep issuing zero-gain/zero-torque commands
    /// rather than relying on `disable()` alone.
    pub fn disable(&mut self) -> Result<()> {
        let (id, data) = build_disable_frame(self.can_id);
        self.send(id, &data)?;
        let _ = self.try_recv_feedback()?;
        self.enabled = false;
        Ok(())
    }

    /// Make the current physical position read as zero, **without** touching
    /// the motor's NVM.
    ///
    /// This is a *soft* zero: the driver reads the current position (via
    /// [`Self::measure`]) and stores it as an in-memory offset that is applied
    /// to all subsequent commands and feedback. It therefore requires the motor
    /// to be reporting position (i.e. enabled, or with a cached command in a
    /// non-MIT mode). The offset is lost on drop; persist it with
    /// [`Self::soft_zero`] / [`Self::set_soft_zero`] if you need it to survive a
    /// restart. To write the zero into motor flash instead, use
    /// [`Self::set_zero_nvm`].
    pub fn set_zero(&mut self) -> Result<()> {
        let fb = self.measure()?;
        // `fb.position` already has the old offset subtracted, so adding it to
        // the old offset yields the current raw position = the new offset.
        self.soft_zero += fb.position;
        Ok(())
    }

    /// Save the current physical position as the new zero **in the motor's NVM**
    /// (`FF*7 FE`).
    ///
    /// Fails with [`Error::Unsupported`] if the configured model does not
    /// support the magic frame (see [`MotorModel::supports_nvm_zero`]). Flash
    /// endurance is finite, so reserve this for occasional calibration — for
    /// routine re-zeroing use [`Self::set_zero`]. On success the soft-zero
    /// offset is reset, since the motor's own frame is now re-zeroed.
    pub fn set_zero_nvm(&mut self) -> Result<()> {
        if !self.model.supports_nvm_zero() {
            return Err(Error::Unsupported {
                motor_id: self.can_id,
                model: self.model.name(),
                op: "set_zero_nvm",
            });
        }
        let (id, data) = build_set_zero_frame(self.can_id);
        self.send(id, &data)?;
        let _ = self.try_recv_feedback()?;
        self.soft_zero = 0.0;
        Ok(())
    }

    /// Clear a latched fault (`FF*7 FB`). Toggling `disable()` → `enable()` is
    /// an alternative recovery path on firmware that does not honour this.
    pub fn clear_error(&mut self) -> Result<()> {
        let (id, data) = build_clear_error_frame(self.can_id);
        self.send(id, &data)?;
        let _ = self.try_recv_feedback()?;
        Ok(())
    }

    // -- control --

    /// MIT impedance control. Requires the motor to be enabled and in MIT mode.
    pub fn mit_control(
        &mut self,
        position: f32,
        velocity: f32,
        kp: f32,
        kd: f32,
        torque: f32,
    ) -> Result<Feedback> {
        if !self.enabled {
            return Err(Error::NotEnabled {
                motor_id: self.can_id,
            });
        }
        let (id, data) = build_mit_frame(
            self.can_id,
            &self.limits,
            position + self.soft_zero,
            velocity,
            kp,
            kd,
            torque,
        );
        self.send(id, &data)?;
        self.last_cmd = Some((id, data));
        self.recv_feedback()
    }

    /// Position-Velocity control (trapezoidal profile to `position` capped at
    /// `max_speed`). Requires POS_VEL mode.
    pub fn set_pos_vel(&mut self, position: f32, max_speed: f32) -> Result<Feedback> {
        if !self.enabled {
            return Err(Error::NotEnabled {
                motor_id: self.can_id,
            });
        }
        let (id, data) = build_pos_vel_frame(self.can_id, position + self.soft_zero, max_speed);
        self.send(id, &data)?;
        self.last_cmd = Some((id, data));
        self.recv_feedback()
    }

    /// Velocity control. Requires VEL mode.
    pub fn set_vel(&mut self, velocity: f32) -> Result<Feedback> {
        if !self.enabled {
            return Err(Error::NotEnabled {
                motor_id: self.can_id,
            });
        }
        let (id, data) = build_vel_frame(self.can_id, velocity);
        self.send(id, &data)?;
        self.last_cmd = Some((id, data));
        self.recv_feedback()
    }

    /// Read state without changing the active setpoint.
    ///
    /// DAMIAO only emits a feedback frame in response to a command, so a truly
    /// passive read is not possible. Strategy:
    /// - if a control command has already been issued, re-send the cached frame
    ///   (holds the same setpoint);
    /// - otherwise, in MIT mode, send a **zero-gain / zero-torque** MIT frame —
    ///   harmless (no holding effort) and works regardless of the enable state,
    ///   which matters because a fresh process has no idea whether the motor is
    ///   already enabled in hardware.
    ///
    /// In POS_VEL / VEL mode with no cached command a MIT frame would be on the
    /// wrong channel and elicit no reply, so this returns an error asking for a
    /// command in that mode first.
    pub fn measure(&mut self) -> Result<Feedback> {
        if let Some((id, data)) = self.last_cmd {
            self.send(id, &data)?;
            return self.recv_feedback();
        }
        if self.mode == ControlMode::Mit {
            // Zero gains/torque ⇒ no torque commanded; position arg is irrelevant.
            let (id, data) = build_mit_frame(self.can_id, &self.limits, 0.0, 0.0, 0.0, 0.0, 0.0);
            self.send(id, &data)?;
            return self.recv_feedback();
        }
        Err(Error::InvalidResponse(
            "measure() in POS_VEL/VEL mode needs a prior command in that mode; send one first"
                .into(),
        ))
    }

    // -- register access --

    /// Wait for a register reply for `rid`.
    ///
    /// The motor replies on its **Master ID** (default `0`), not on the `0x7FF`
    /// config channel, so we identify replies by content and skip our own
    /// `0x7FF` transmit echoes. Replies are matched on `can_id` + `rid`.
    fn recv_reg_reply(&mut self, rid: u8) -> Result<RegReply> {
        let start = Instant::now();
        loop {
            if start.elapsed() > self.timeout {
                return Err(Error::Timeout {
                    motor_id: self.can_id,
                });
            }
            match self.bus.recv() {
                Ok(frame) => {
                    // Skip frames on the config channel (our own requests / echoes).
                    if frame.can_id == REGISTER_ID {
                        continue;
                    }
                    if let Some(reply) = parse_reg_reply(&frame.data) {
                        if reply.can_id == self.can_id && reply.rid == rid {
                            return Ok(reply);
                        }
                    }
                }
                Err(Error::Timeout { .. }) => continue,
                Err(e) => return Err(e),
            }
        }
    }

    /// Read a register (RID). Request goes to `0x7FF`; the reply arrives on the
    /// motor's Master ID.
    pub fn read_register(&mut self, rid: u8) -> Result<RegReply> {
        let (id, data) = build_read_reg(self.can_id, rid);
        self.send(id, &data)?;
        self.recv_reg_reply(rid)
    }

    /// Replace the MIT quantization limits with the motor's own
    /// `PMAX`/`VMAX`/`TMAX` registers (RID `0x15`/`0x16`/`0x17`), and return
    /// what was read.
    ///
    /// **This is the authoritative source for MIT scaling.** The per-model
    /// defaults from [`MotorModel::limits`] are only that — defaults: no
    /// official manual states concrete mapping ranges for any model (they are
    /// documented purely as writable registers with range `(0.0, fmax]`), and
    /// the vendor SDK's `Limit_Param` table contradicts itself across its own
    /// four copies. The vendor SDK has the same escape hatch
    /// (`changeMotorLimit`), so reading the registers is the intended flow
    /// rather than a workaround.
    ///
    /// Call this once after opening a motor — especially when
    /// [`MotorModel::limits_source`] is not
    /// [`LimitsSource::Sdk`](damiao_protocol::LimitsSource::Sdk) — before
    /// issuing MIT commands. Position, velocity and torque are all quantized
    /// against these ranges, so a mismatch silently mis-scales every MIT
    /// command and every decoded feedback value rather than raising an error.
    ///
    /// Kp/Kd ranges are fixed by the protocol (`[0,500]` / `[0,5]`) and are
    /// left untouched.
    pub fn refresh_limits_from_registers(&mut self) -> Result<Limits> {
        let p_max = self.read_register(Rid::PMAX)?.as_f32();
        let v_max = self.read_register(Rid::VMAX)?.as_f32();
        let t_max = self.read_register(Rid::TMAX)?.as_f32();
        // A zero or negative range would make the quantizer divide the field
        // across an empty interval; the manual's own range is `(0.0, fmax]`,
        // so treat anything outside that as a failed/garbled read and keep
        // the previous limits rather than poisoning all later scaling.
        if !(p_max > 0.0 && v_max > 0.0 && t_max > 0.0) {
            return Err(Error::InvalidResponse(format!(
                "motor {} returned non-positive MIT ranges (PMAX={p_max}, VMAX={v_max}, \
                 TMAX={t_max}); keeping previous limits",
                self.can_id
            )));
        }
        self.limits = Limits {
            p_max,
            v_max,
            t_max,
            ..self.limits
        };
        log::info!(
            "damiao motor {}: MIT limits from registers — PMAX={} VMAX={} TMAX={}",
            self.can_id,
            p_max,
            v_max,
            t_max
        );
        Ok(self.limits)
    }

    /// Read a model-specific register by meaning rather than by RID.
    ///
    /// These registers live above the common `0x00`–`0x25` block, where the
    /// DM4310 and DM3507 maps diverge — `m_off` in particular sits at a
    /// different address on each. Resolving through [`ModelReg`] against the
    /// configured [`Self::model`] makes it impossible to read the wrong
    /// address for the motor in hand.
    ///
    /// Fails with [`Error::Unsupported`] if this model's register map does
    /// not document `reg`, rather than reading an undocumented address and
    /// returning a meaningless value.
    pub fn read_model_register(&mut self, reg: ModelReg) -> Result<RegReply> {
        match reg.rid(self.model) {
            Some(rid) => self.read_register(rid),
            None => Err(Error::Unsupported {
                motor_id: self.can_id,
                model: self.model.name(),
                op: reg.name(),
            }),
        }
    }

    /// Write an integer register and persist nothing (volatile until saved).
    pub fn write_register_int(&mut self, rid: u8, value: i32) -> Result<()> {
        let (id, data) = build_write_reg_int(self.can_id, rid, value);
        self.send(id, &data)?;
        // The motor echoes the write back on its Master ID; drain it so it does
        // not get mistaken for a later read reply. Best-effort.
        let _ = self.recv_reg_reply(rid);
        Ok(())
    }

    /// Write an `f32` register.
    pub fn write_register_f32(&mut self, rid: u8, value: f32) -> Result<()> {
        let (id, data) = build_write_reg_f32(self.can_id, rid, value);
        self.send(id, &data)?;
        let _ = self.recv_reg_reply(rid);
        Ok(())
    }

    /// Commit all RAM parameters to flash so they survive a power cycle.
    ///
    /// Register writes ([`Self::write_register_int`] / `..._f32`) only update
    /// RAM; call this to persist them. The motor is disabled first (required by
    /// firmware before a flash write). **Changes to `CAN_ID` / `MST_ID` take
    /// effect only after a power cycle.** Use sparingly — flash endurance is
    /// finite.
    pub fn save_to_flash(&mut self) -> Result<()> {
        // Firmware requires the motor disabled before committing to flash.
        self.disable()?;
        let (id, data) = build_save_all(self.can_id);
        self.send(id, &data)?;
        // Flash write takes ~10 ms; wait conservatively, then drain any reply.
        std::thread::sleep(Duration::from_millis(50));
        let _ = self.try_recv_feedback();
        Ok(())
    }

    /// Switch control mode via `CTRL_MODE` (RID 10), verifying the read-back.
    ///
    /// No-ops if already in `mode`. Retries the verification up to
    /// [`MODE_SWITCH_RETRIES`] times before giving up.
    pub fn switch_mode(&mut self, mode: ControlMode) -> Result<()> {
        // Only early-out once we've actually confirmed the hardware mode — the
        // initial `mode` is an assumption and the motor may have a different
        // saved mode, so the first switch must always write the register.
        if self.mode_confirmed && self.mode == mode {
            return Ok(());
        }
        let mut got_reply = false;
        let mut last_value = -1;
        for _ in 0..MODE_SWITCH_RETRIES {
            self.write_register_int(Rid::CTRL_MODE, mode as i32)?;
            match self.read_register(Rid::CTRL_MODE) {
                Ok(reply) => {
                    got_reply = true;
                    last_value = reply.as_i32();
                    if last_value == mode as i32 {
                        self.mode = mode;
                        self.mode_confirmed = true;
                        return Ok(());
                    }
                }
                Err(Error::Timeout { .. }) | Err(Error::InvalidResponse(_)) => continue,
                Err(e) => return Err(e),
            }
        }
        if !got_reply {
            // The motor never acknowledged a CTRL_MODE read-back. Some
            // firmware/configs don't answer register reads over CAN; trust the
            // write and proceed. If the mode did not actually change, the
            // subsequent mode-specific command is simply ignored (it goes to a
            // different CAN id), so this is safe — verify by observing motion.
            log::warn!(
                "motor {}: no CTRL_MODE read-back; assuming mode {} set (unverified)",
                self.can_id,
                mode as i32
            );
            self.mode = mode;
            self.mode_confirmed = true;
            return Ok(());
        }
        Err(Error::ModeSwitchFailed {
            motor_id: self.can_id,
            requested: mode as i32,
            actual: last_value,
        })
    }
}
