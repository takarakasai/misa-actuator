//! High-level [`Motor`] driver — generic over any [`RobstrideBus`].
//!
//! `Motor` was historically tied to Linux SocketCAN. It is now generic over
//! the bus, with [`SocketCanBus`] as the default. Concrete usage:
//!
//! - `Motor::open("can0", 1, MotorModel::Rs05)` — convenience constructor
//!   that opens a SocketCAN interface (defaults to `Motor<SocketCanBus>`).
//! - `Motor::with_bus(my_usb_can_bus, 1, MotorModel::Rs05)` — bring your
//!   own bus (e.g. a future USB-CAN serial adapter).

use std::time::{Duration, Instant};

use robstride_protocol::{
    CommType, MitScales, MotorFeedback, MotorModel, ParamIndex, RunMode, build_can_id_raw,
    build_disable_frame, build_enable_frame, build_mit_frame, build_ping_frame,
    build_read_param_frame, build_run_mode_frame, build_set_device_id_frame, build_set_zero_frame,
    build_write_param_f32_frame, parse_can_id, parse_param_response, parse_status_frame,
    DEFAULT_HOST_ID,
};

use crate::bus::{RobstrideBus, SocketCanBus};
use crate::error::{Error, Result};

const DEFAULT_TIMEOUT: Duration = Duration::from_millis(100);

/// High-level controller for a single Robstride motor on any [`RobstrideBus`].
pub struct Motor<B: RobstrideBus = SocketCanBus> {
    bus: B,
    motor_id: u8,
    host_id: u8,
    model: MotorModel,
    scales: MitScales,
    enabled: bool,
    run_mode: RunMode,
    timeout: Duration,
    /// Cached `LimitSpd` value (rad/s). Used by [`Self::set_position_with_speed`]
    /// to avoid redundant `LimitSpd` writes when the same speed is repeated.
    cached_speed_limit: Option<f32>,
    /// Whether feedback should also carry current, at the cost of one extra
    /// parameter read per sample. See [`Self::set_report_current`].
    report_current: bool,
    /// Torque constant (N·m/A) used to synthesize torque from current when the
    /// motor's own `MeasuredTorque` is unusable. See
    /// [`Self::set_torque_constant`].
    torque_constant: Option<f32>,
}

impl Motor<SocketCanBus> {
    /// Open the given SocketCAN interface and bind to a single motor id.
    pub fn open(interface: &str, motor_id: u8, model: MotorModel) -> Result<Self> {
        Self::open_with_host(interface, motor_id, DEFAULT_HOST_ID, model)
    }

    /// Open with a custom host id (must be greater than every motor id on the
    /// bus for optimal scheduling on the controller side).
    pub fn open_with_host(
        interface: &str,
        motor_id: u8,
        host_id: u8,
        model: MotorModel,
    ) -> Result<Self> {
        let bus = SocketCanBus::open(interface)?;
        Ok(Self::with_bus_and_host(bus, motor_id, host_id, model))
    }
}

impl<B: RobstrideBus> Motor<B> {
    /// Build over a caller-provided bus. Uses [`DEFAULT_HOST_ID`] for the
    /// controller's own CAN id.
    pub fn with_bus(bus: B, motor_id: u8, model: MotorModel) -> Self {
        Self::with_bus_and_host(bus, motor_id, DEFAULT_HOST_ID, model)
    }

    /// Build over a caller-provided bus, picking the host id explicitly.
    pub fn with_bus_and_host(bus: B, motor_id: u8, host_id: u8, model: MotorModel) -> Self {
        Self {
            bus,
            motor_id,
            host_id,
            model,
            scales: MitScales::for_model(model),
            enabled: false,
            run_mode: RunMode::Mit,
            timeout: DEFAULT_TIMEOUT,
            cached_speed_limit: None,
            report_current: false,
            torque_constant: None,
        }
    }

    pub fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.bus.set_timeout(timeout)?;
        self.timeout = timeout;
        Ok(())
    }

    pub fn motor_id(&self) -> u8 {
        self.motor_id
    }

    pub fn host_id(&self) -> u8 {
        self.host_id
    }

    pub fn model(&self) -> MotorModel {
        self.model
    }

    pub fn scales(&self) -> &MitScales {
        &self.scales
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    pub fn current_run_mode(&self) -> RunMode {
        self.run_mode
    }

    /// Borrow the underlying bus (for low-level diagnostic access).
    pub fn bus(&mut self) -> &mut B {
        &mut self.bus
    }

    // ---------------------------------------------------------------------
    // Low-level I/O
    // ---------------------------------------------------------------------

    pub(crate) fn send(&mut self, can_id: u32, data: &[u8]) -> Result<()> {
        log::debug!("TX id=0x{:08X} data={:02X?}", can_id, data);
        self.bus.send(can_id, data)
    }

    /// The configured per-request timeout (see [`Self::set_timeout`]).
    pub(crate) fn timeout(&self) -> Duration {
        self.timeout
    }

    /// Receive frames until one matches `accept`, dropping unmatched frames.
    /// Returns timeout error if no accepted frame arrives within `self.timeout`.
    fn recv_filtered<F>(&mut self, mut accept: F) -> Result<(u8, u16, u8, Vec<u8>)>
    where
        F: FnMut(u8, u16, u8) -> bool,
    {
        let start = Instant::now();
        loop {
            if start.elapsed() > self.timeout {
                return Err(Error::Timeout {
                    motor_id: self.motor_id,
                });
            }

            match self.bus.recv() {
                Ok(frame) => {
                    let (comm_type, extra_data, device_id) = parse_can_id(frame.can_id);
                    log::debug!(
                        "RX id=0x{:08X} comm={} extra=0x{:04X} dev={} data={:02X?}",
                        frame.can_id, comm_type, extra_data, device_id, &frame.data,
                    );
                    if accept(comm_type, extra_data, device_id) {
                        return Ok((comm_type, extra_data, device_id, frame.data));
                    }
                }
                Err(Error::Timeout { .. }) => continue,
                Err(e) => return Err(e),
            }
        }
    }

    /// Receive the next frame regardless of type.
    fn recv(&mut self) -> Result<(u8, u16, u8, Vec<u8>)> {
        self.recv_filtered(|_, _, _| true)
    }

    fn recv_status(&mut self) -> Result<MotorFeedback> {
        let (comm_type, extra_data, device_id, data) = self.recv_filtered(|ct, _, _| {
            ct == CommType::OperationStatus as u8 || ct == CommType::FaultReport as u8
        })?;

        if comm_type == CommType::FaultReport as u8 {
            return Err(Error::MotorFault {
                motor_id: device_id,
                extra_data,
            });
        }

        parse_status_frame(
            build_can_id_raw(comm_type, extra_data, device_id),
            &data,
            &self.scales,
        )
        .ok_or_else(|| Error::InvalidResponse("failed to parse status frame".into()))
    }

    // ---------------------------------------------------------------------
    // Bus-level commands
    // ---------------------------------------------------------------------

    /// Send a `GET_DEVICE_ID` (ping). Returns `(device_id_field, payload)`.
    pub fn ping(&mut self) -> Result<(u16, Vec<u8>)> {
        let (id, data) = build_ping_frame(self.host_id, self.motor_id);
        self.send(id, &data)?;
        let (_ct, extra, _dev, payload) = self.recv()?;
        Ok((extra, payload))
    }

    pub fn enable(&mut self) -> Result<MotorFeedback> {
        let (id, data) = build_enable_frame(self.host_id, self.motor_id);
        self.send(id, &data)?;
        let fb = self.recv_status()?;
        self.enabled = true;
        Ok(fb)
    }

    pub fn disable(&mut self) -> Result<MotorFeedback> {
        let (id, data) = build_disable_frame(self.host_id, self.motor_id);
        self.send(id, &data)?;
        let fb = self.recv_status()?;
        self.enabled = false;
        Ok(fb)
    }

    pub fn set_zero(&mut self) -> Result<()> {
        let (id, data) = build_set_zero_frame(self.host_id, self.motor_id);
        self.send(id, &data)?;
        std::thread::sleep(Duration::from_millis(50));
        Ok(())
    }

    pub fn set_run_mode(&mut self, mode: RunMode) -> Result<()> {
        let (id, data) = build_run_mode_frame(self.host_id, self.motor_id, mode);
        self.send(id, &data)?;
        self.run_mode = mode;
        std::thread::sleep(Duration::from_millis(10));
        Ok(())
    }

    /// Reassign this motor's CAN bus address (`SET_CAN_ID`, type 7).
    ///
    /// Takes effect immediately and persists across power cycles — no
    /// restart or separate flash-save step is needed. On success, `self` is
    /// updated to address the motor at `new_id` for all subsequent calls.
    ///
    /// A reply is not guaranteed on every firmware revision, so this waits
    /// best-effort for one and does not fail the call if none arrives —
    /// verify with a bus scan at `new_id` afterward if you need certainty.
    pub fn set_device_id(&mut self, new_id: u8) -> Result<()> {
        let old_id = self.motor_id;
        let (id, data) = build_set_device_id_frame(old_id, new_id);
        self.send(id, &data)?;
        // Skip our own TX echo (gs_usb/PCAN-style adapters re-emit the sent
        // frame to the local socket) so it isn't mistaken for the motor's
        // reply — same pattern as scan_bus_on's ping-echo filter.
        let echo_comm = CommType::SetDeviceId as u8;
        match self.recv_filtered(|ct, extra, dev| {
            !(ct == echo_comm && extra == new_id as u16 && dev == old_id)
        }) {
            Ok((comm_type, extra_data, device_id, _)) => log::debug!(
                "set_device_id: reply comm={comm_type} extra=0x{extra_data:04X} dev={device_id}"
            ),
            Err(Error::Timeout { .. }) => {
                log::debug!("set_device_id: no reply within timeout (some firmware doesn't ack)")
            }
            Err(e) => return Err(e),
        }
        self.motor_id = new_id;
        std::thread::sleep(Duration::from_millis(50));
        Ok(())
    }

    // ---------------------------------------------------------------------
    // MIT mode
    // ---------------------------------------------------------------------

    /// Send a MIT-mode control command. Requires the motor to be enabled.
    pub fn mit_control(
        &mut self,
        position: f32,
        velocity: f32,
        kp: f32,
        kd: f32,
        torque: f32,
    ) -> Result<MotorFeedback> {
        if !self.enabled {
            return Err(Error::NotEnabled {
                motor_id: self.motor_id,
            });
        }
        let (id, data) = build_mit_frame(
            self.motor_id,
            &self.scales,
            position,
            velocity,
            kp,
            kd,
            torque,
        );
        self.send(id, &data)?;
        self.recv_status()
    }

    // ---------------------------------------------------------------------
    // Position / velocity / torque parameter shortcuts
    // ---------------------------------------------------------------------

    pub fn set_position(&mut self, position: f32) -> Result<()> {
        self.write_param_f32(ParamIndex::LocRef, position)
    }

    pub fn set_position_speed_limit(&mut self, speed: f32) -> Result<()> {
        self.write_param_f32(ParamIndex::LimitSpd, speed)?;
        self.cached_speed_limit = Some(speed);
        Ok(())
    }

    /// Combined position-mode setpoint: writes `LimitSpd` only when it
    /// changes (vs. the cached last value), then writes `LocRef`. This is
    /// what the misa-actuator [`misa_actuator::Actuator::set_position`]
    /// impl dispatches to.
    pub fn set_position_with_speed(&mut self, position: f32, max_speed: f32) -> Result<()> {
        let speed_changed = self
            .cached_speed_limit
            .map(|s| (s - max_speed).abs() > f32::EPSILON)
            .unwrap_or(true);
        if speed_changed {
            self.write_param_f32(ParamIndex::LimitSpd, max_speed)?;
            self.cached_speed_limit = Some(max_speed);
        }
        self.write_param_f32(ParamIndex::LocRef, position)
    }

    pub fn set_torque_limit(&mut self, torque: f32) -> Result<()> {
        self.write_param_f32(ParamIndex::LimitTorque, torque)
    }

    pub fn set_current_limit(&mut self, current: f32) -> Result<()> {
        self.write_param_f32(ParamIndex::LimitCur, current)
    }

    pub fn set_velocity(&mut self, velocity: f32) -> Result<()> {
        self.write_param_f32(ParamIndex::SpdRef, velocity)
    }

    pub fn set_torque(&mut self, iq: f32) -> Result<()> {
        self.write_param_f32(ParamIndex::IqRef, iq)
    }

    // ---------------------------------------------------------------------
    // Parameter access
    // ---------------------------------------------------------------------

    pub fn read_param(&mut self, param: ParamIndex) -> Result<f32> {
        let (id, data) = build_read_param_frame(self.host_id, self.motor_id, param);
        self.send(id, &data)?;
        let target_idx = param as u16;
        let start = Instant::now();
        loop {
            if start.elapsed() > self.timeout {
                return Err(Error::Timeout {
                    motor_id: self.motor_id,
                });
            }
            let (_ct, _extra, _dev, payload) = self.recv_filtered(|ct, _, _| {
                ct == CommType::ReadParameter as u8
            })?;
            let (idx, val) = parse_param_response(&payload)
                .ok_or_else(|| Error::InvalidResponse("failed to parse param response".into()))?;
            if idx == target_idx {
                return Ok(val);
            }
            log::debug!(
                "read_param: discarding stale response idx=0x{:04X} (wanted 0x{:04X})",
                idx, target_idx
            );
        }
    }

    pub fn write_param_f32(&mut self, param: ParamIndex, value: f32) -> Result<()> {
        let (id, data) = build_write_param_f32_frame(self.host_id, self.motor_id, param, value);
        self.send(id, &data)?;
        std::thread::sleep(Duration::from_millis(5));
        Ok(())
    }

    // ---------------------------------------------------------------------
    // Status
    // ---------------------------------------------------------------------

    /// Send a zero-amplitude MIT command to elicit a status frame, even when
    /// the motor is not enabled.
    pub fn read_status(&mut self) -> Result<MotorFeedback> {
        let (id, data) = build_mit_frame(self.motor_id, &self.scales, 0.0, 0.0, 0.0, 0.0, 0.0);
        self.send(id, &data)?;
        self.recv_status()
    }

    pub fn read_position(&mut self) -> Result<f32> {
        self.read_param(ParamIndex::MechPos)
    }

    pub fn read_velocity(&mut self) -> Result<f32> {
        self.read_param(ParamIndex::MechVel)
    }

    pub fn read_current(&mut self) -> Result<f32> {
        self.read_param(ParamIndex::IqFilt)
    }

    /// Whether feedback carries current. See [`Self::set_report_current`].
    pub fn reports_current(&self) -> bool {
        self.report_current
    }

    /// Make [`misa_actuator::Actuator`] feedback report current as well as
    /// torque, at the cost of **one extra bus round-trip per sample**.
    ///
    /// Off by default. RobStride's feedback frame carries position, velocity,
    /// torque and temperature but *no current*, so the only way to get it is a
    /// separate `IqFilt` (`0x701A`) parameter read — which is what
    /// [`Self::read_current`] and the CLI's `info` do. Paying that on every
    /// sample roughly halves the achievable loop rate, which would quietly
    /// shrink the usable identification band of a chirp, so control and
    /// identification paths leave it off.
    ///
    /// Turn it on for quasi-static measurements that dwell anyway — a Kt sweep
    /// holds each level for hundreds of milliseconds, so an extra read costs
    /// nothing and is the difference between measuring a torque constant and
    /// reporting `NaN`.
    pub fn set_report_current(&mut self, on: bool) {
        self.report_current = on;
    }

    /// The torque constant in effect (N·m/A), or `None`.
    pub fn torque_constant(&self) -> Option<f32> {
        self.torque_constant
    }

    /// Supply a torque constant (N·m/A) so torque can be synthesized from
    /// current when the motor does not report it.
    ///
    /// Needed because `MeasuredTorque` (`0x302C`) reads a constant 0 on some
    /// firmware. Measured on an RS04 (AppCodeVersion 0.4.1.32): across three
    /// runs and 72 samples in Position and Torque mode, feedback torque was 0
    /// every time, while the MIT path reported real values in 109 of 109. The
    /// difference is which source the feedback comes from — `measure_safe`
    /// deliberately avoids sending a MIT frame (it would disturb the active
    /// control mode) and reads parameters instead, and on that firmware the
    /// torque parameter is the one that does not work. Suspected to be another
    /// instance of the parameter-table shift documented in
    /// `robstride-protocol/doc/param-table-firmware-divergence.md`, though
    /// `0x302C` sits outside the range that was dumped, so that is unconfirmed.
    ///
    /// `IqFilt` (`0x701A`) *does* work on that firmware, including in Position
    /// mode, so `torque = current * Kt` recovers it. This is the mirror image of
    /// the DAMIAO driver, which has torque and derives current.
    ///
    /// Requires [`Self::set_report_current`] — without a current reading there
    /// is nothing to scale. Values that are not finite and positive are
    /// rejected, leaving the constant unset rather than poisoning every reading.
    ///
    /// Get the figure from a `characterize kt` sweep (an RS04 measured
    /// 1.5093 N·m/A) or the datasheet. Kt sags under magnetic saturation and
    /// drifts with magnet temperature, so it holds within the rated range.
    pub fn set_torque_constant(&mut self, kt_nm_per_a: f32) {
        self.torque_constant =
            (kt_nm_per_a.is_finite() && kt_nm_per_a > 0.0).then_some(kt_nm_per_a);
    }

    pub fn read_vbus(&mut self) -> Result<f32> {
        self.read_param(ParamIndex::Vbus)
    }

    /// Read filtered measured torque (Nm) via parameter access. Safe to call
    /// in any run mode — does not send a control frame.
    pub fn read_torque(&mut self) -> Result<f32> {
        self.read_param(ParamIndex::MeasuredTorque)
    }

    /// Read motor temperature (°C). The protocol exposes temperature only via
    /// the status frame, so this calls [`Self::read_status`] internally — that
    /// sends a zero-amplitude MIT control frame, which can disrupt non-MIT
    /// run modes. Avoid calling while actively position/velocity/torque
    /// controlling; use it for one-shot snapshots when the motor is idle.
    pub fn read_temperature(&mut self) -> Result<f32> {
        Ok(self.read_status()?.temperature)
    }

    /// Run-mode-safe feedback poll: in MIT mode this is just
    /// [`Self::read_status`]; in Position/Velocity/Torque modes it falls
    /// back to per-parameter reads (`MechPos`, `MechVel`, `MeasuredTorque`,
    /// `IqFilt`) so it does **not** send a zero-MIT frame that would yank
    /// the firmware back into MIT mode with zero gains (= loss of holding
    /// torque, looks like a disabled motor).
    ///
    /// Returns a [`MotorFeedback`] with `temperature = NaN` and
    /// `status = MotorStatusBits::default()` in non-MIT modes (those fields
    /// are only populated by the status frame).
    pub fn measure_safe(&mut self) -> Result<MotorFeedback> {
        if self.run_mode == RunMode::Mit {
            return self.read_status();
        }
        let position = self.read_position()?;
        let velocity = self.read_velocity()?;
        let torque = self.read_torque().unwrap_or(f32::NAN);
        Ok(MotorFeedback {
            motor_id: self.motor_id,
            position,
            velocity,
            torque,
            temperature: f32::NAN,
            status: robstride_protocol::MotorStatusBits::default(),
        })
    }
}
