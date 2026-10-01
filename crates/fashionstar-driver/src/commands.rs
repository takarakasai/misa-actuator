//! Typed commands, available on every [`FsBus`] via [`FsCommands`].
//!
//! Methods speak SI (rad, V, A, W, °C, [`Duration`]); each has a `_raw`
//! sibling that takes or returns wire units for callers that need exact
//! values. Clamps mirror the SDK's `set_servo_angle`.

use std::time::Duration;

use fashionstar_protocol::{
    self as proto, request, Code, MturnAngle, MturnAngleByInterval,
    MultiTurnAngle, SetAngle, StopMode, MAX_FRAME, MTURN_ANGLE_LIMIT, MTURN_INTERVAL_MAX_MS,
    SINGLE_TURN_ANGLE_LIMIT,
};

use crate::bus::FsBus;
use crate::error::{Error, Result};
use crate::units::{ntc_to_celsius, rad_to_raw, raw_to_rad};

/// Raw monitor reply, re-exported from the protocol crate.
pub use fashionstar_protocol::Monitor as RawMonitor;

/// Minimum accel/decel time the SDK allows (`if t_acc < 20: t_acc = 20`).
pub const MIN_ACCEL_TIME: Duration = Duration::from_millis(20);

/// One `QUERY_SERVO_MONITOR` reading in SI units, with the raw reply kept.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Monitor {
    pub id: u8,
    /// Multi-turn angle, rad, turns included. Measured from the servo's own
    /// origin ([`FsCommands::set_origin_point`]); may carry whole extra
    /// turns — see [`crate::units::unwrap_to_window`].
    pub angle_rad: f32,
    /// Bus voltage, V.
    pub voltage_v: f32,
    /// Current magnitude, A (unsigned on the wire).
    pub current_a: f32,
    /// Power, W.
    pub power_w: f32,
    /// Temperature, °C (NaN if the ADC count is out of range).
    pub temperature_c: f32,
    /// Status byte, see [`fashionstar_protocol::status_bits`].
    pub status: u8,
    /// Turn counter, informational (already included in `angle_rad`).
    pub turns: i16,
    /// The reply exactly as received.
    pub raw: RawMonitor,
}

impl Monitor {
    /// Convert a raw reply. Returns `None` for the invalid sentinel.
    pub fn from_raw(raw: RawMonitor) -> Option<Self> {
        if !raw.is_valid() {
            return None;
        }
        Some(Self {
            id: raw.id,
            angle_rad: raw_to_rad(raw.angle),
            voltage_v: raw.voltage_mv as f32 * 1e-3,
            current_a: raw.current_ma as f32 * 1e-3,
            power_w: raw.power_mw as f32 * 1e-3,
            temperature_c: ntc_to_celsius(raw.temp_raw),
            status: raw.status,
            turns: raw.turns,
            raw,
        })
    }

    /// Angle in degrees, the unit the vendor tools print.
    pub fn angle_deg(&self) -> f32 {
        self.raw.angle as f32 * 0.1
    }
}

fn ms_u16(d: Duration) -> u16 {
    d.as_millis().min(u16::MAX as u128) as u16
}

fn ms_u32(d: Duration) -> u32 {
    d.as_millis().min(u32::MAX as u128) as u32
}

/// Typed commands on any [`FsBus`]. Bring into scope with
/// `use fashionstar_driver::FsCommands;`.
pub trait FsCommands: FsBus {
    /// Encode with `f` and send without waiting.
    #[doc(hidden)]
    fn send_encoded(
        &mut self,
        f: impl FnOnce(&mut [u8]) -> std::result::Result<usize, proto::EncodeError>,
    ) -> Result<()> {
        let mut buf = [0u8; MAX_FRAME];
        let n = f(&mut buf)?;
        self.send(&buf[..n])
    }

    /// Encode with `f`, send, and wait for the `reply` from `id`.
    #[doc(hidden)]
    fn transact_encoded(
        &mut self,
        reply: Code,
        id: u8,
        f: impl FnOnce(&mut [u8]) -> std::result::Result<usize, proto::EncodeError>,
    ) -> Result<proto::Response> {
        let mut buf = [0u8; MAX_FRAME];
        let n = f(&mut buf)?;
        self.transact(&buf[..n], reply.code(), id)
    }

    /// `true` if servo `id` answers a PING within the bus timeout.
    fn ping(&mut self, id: u8) -> Result<bool> {
        match self.transact_encoded(Code::Ping, id, |o| request::encode_ping(id, o)) {
            Ok(r) => {
                proto::parse_ping(r.code, &r.params)?;
                Ok(true)
            }
            Err(Error::Timeout { .. }) => Ok(false),
            Err(e) => Err(e),
        }
    }

    /// Single-turn angle, raw 0.1° units (±1800).
    fn query_angle_raw(&mut self, id: u8) -> Result<i16> {
        let r = self.transact_encoded(Code::QueryAngle, id, |o| {
            request::encode_query_angle(id, o)
        })?;
        Ok(proto::parse_angle(r.code, &r.params)?.angle)
    }

    /// Single-turn angle, rad (±π).
    fn query_angle(&mut self, id: u8) -> Result<f32> {
        Ok(raw_to_rad(self.query_angle_raw(id)? as i32))
    }

    /// Multi-turn angle and turn counter, raw units.
    fn query_angle_multiturn_raw(&mut self, id: u8) -> Result<MultiTurnAngle> {
        let r = self.transact_encoded(Code::QueryAngleMturn, id, |o| {
            request::encode_query_angle_mturn(id, o)
        })?;
        Ok(proto::parse_angle_mturn(r.code, &r.params)?)
    }

    /// Multi-turn angle, rad (turns included).
    fn query_angle_multiturn(&mut self, id: u8) -> Result<f32> {
        Ok(raw_to_rad(self.query_angle_multiturn_raw(id)?.angle))
    }

    /// Raw monitor reply, the invalid sentinel included.
    fn monitor_raw(&mut self, id: u8) -> Result<RawMonitor> {
        let r = self.transact_encoded(Code::QueryMonitor, id, |o| {
            request::encode_query_monitor(id, o)
        })?;
        Ok(proto::parse_monitor(r.code, &r.params)?)
    }

    /// Monitor one servo. The invalid sentinel is [`Error::InvalidReading`].
    fn monitor(&mut self, id: u8) -> Result<Monitor> {
        Monitor::from_raw(self.monitor_raw(id)?).ok_or(Error::InvalidReading { servo_id: id })
    }

    /// Monitor several servos with **one** request (`SYNC_COMMAND` carrying
    /// `QUERY_SERVO_MONITOR`, as the vendor teleop loop does).
    ///
    /// Slot `i` is the reading of `ids[i]`. A servo that did not answer
    /// within the bus timeout, answered with a malformed frame, or answered
    /// with the invalid sentinel gives `None` — not an error — so one flaky
    /// joint does not cost the other six readings. Transport failures are
    /// still errors.
    fn sync_monitor(&mut self, ids: &[u8]) -> Result<Vec<Option<Monitor>>> {
        let mut buf = [0u8; MAX_FRAME];
        let n = request::encode_sync_monitor(ids, &mut buf)?;
        let replies = self.transact_many(&buf[..n], Code::QueryMonitor.code(), ids)?;
        Ok(replies
            .into_iter()
            .map(|r| {
                let r = r?;
                match proto::parse_monitor(r.code, &r.params) {
                    Ok(raw) => Monitor::from_raw(raw),
                    Err(e) => {
                        log::warn!("fashionstar: malformed monitor reply {e:?}");
                        None
                    }
                }
            })
            .collect())
    }

    /// [`Self::sync_monitor`] split into requests of at most `chunk` servos.
    ///
    /// Measured on a Star Arm 102 LD through its CH340 hub at 1 Mbps
    /// (2026-10-02): in one sync request the **4th and later** replies lose
    /// bytes 25–35 % of the time, whichever servos they are, while groups of
    /// up to 3 (63 bytes of back-to-back replies) arrive intact every time.
    /// The adapter's receive FIFO apparently overruns on longer bursts. With
    /// 7 servos: one request 133 Hz with 3 joints dropping a quarter of their
    /// readings; chunks of 3 (3+3+1) 95 Hz with none dropped.
    fn sync_monitor_chunked(&mut self, ids: &[u8], chunk: usize) -> Result<Vec<Option<Monitor>>> {
        let mut out = Vec::with_capacity(ids.len());
        for group in ids.chunks(chunk.max(1)) {
            out.extend(self.sync_monitor(group)?);
        }
        Ok(out)
    }

    /// Stop with `mode` and the SDK's default power argument (0).
    ///
    /// [`StopMode::Release`] makes the joint limp (how the leader arm is
    /// used), [`StopMode::Hold`] locks it in place. `id` may be
    /// [`fashionstar_protocol::BROADCAST_ID`].
    fn stop(&mut self, id: u8, mode: StopMode) -> Result<()> {
        self.stop_with_power(id, mode, 0)
    }

    /// Stop with an explicit holding power (mW) for hold / damping.
    fn stop_with_power(&mut self, id: u8, mode: StopMode, power_mw: u16) -> Result<()> {
        self.send_encoded(|o| request::encode_stop_on_control(id, mode, power_mw, o))
    }

    /// Single-turn move (`SET_SERVO_ANGLE`), raw record.
    fn set_angle_raw(&mut self, cmd: SetAngle) -> Result<()> {
        self.send_encoded(|o| request::encode_command(&cmd, o))
    }

    /// Single-turn move to `angle_rad` (clamped to ±π) taking `interval`.
    /// `power_mw = 0` as in the SDK's default.
    fn set_angle(&mut self, id: u8, angle_rad: f32, interval: Duration) -> Result<()> {
        let a = rad_to_raw(angle_rad)
            .clamp(-(SINGLE_TURN_ANGLE_LIMIT as i32), SINGLE_TURN_ANGLE_LIMIT as i32);
        self.set_angle_raw(SetAngle {
            id,
            angle: a as i16,
            interval_ms: ms_u16(interval),
            power_mw: 0,
        })
    }

    /// Multi-turn move (`SET_SERVO_ANGLE_MTURN`), raw record.
    fn set_angle_multiturn_raw(&mut self, cmd: MturnAngle) -> Result<()> {
        self.send_encoded(|o| request::encode_command(&cmd, o))
    }

    /// Multi-turn move to `angle_rad` (clamped to ±1024 turns) taking
    /// `interval`.
    fn set_angle_multiturn(&mut self, id: u8, angle_rad: f32, interval: Duration) -> Result<()> {
        self.set_angle_multiturn_raw(MturnAngle {
            id,
            angle: rad_to_raw(angle_rad).clamp(-MTURN_ANGLE_LIMIT, MTURN_ANGLE_LIMIT),
            interval_ms: ms_u32(interval).min(MTURN_INTERVAL_MAX_MS),
            power_mw: 0,
        })
    }

    /// Multi-turn move with accel/decel (`SET_SERVO_ANGLE_MTURN_BY_INTERVAL`),
    /// raw record.
    fn set_angle_multiturn_by_interval_raw(&mut self, cmd: MturnAngleByInterval) -> Result<()> {
        self.send_encoded(|o| request::encode_command(&cmd, o))
    }

    /// Multi-turn move with a trapezoidal profile: accelerate for `t_acc`,
    /// decelerate for `t_dec`, arrive after `interval`.
    ///
    /// Clamped as the SDK does: `t_acc`, `t_dec` ≥ 20 ms; `interval` ≥
    /// `t_acc + t_dec` and ≤ 4096 s; angle to ±1024 turns.
    fn set_angle_multiturn_by_interval(
        &mut self,
        id: u8,
        angle_rad: f32,
        interval: Duration,
        t_acc: Duration,
        t_dec: Duration,
    ) -> Result<()> {
        let cmd = mturn_by_interval(id, angle_rad, interval, t_acc, t_dec);
        self.set_angle_multiturn_by_interval_raw(cmd)
    }

    /// Several multi-turn moves in one `SYNC_COMMAND` frame — how the vendor
    /// teleop loop drives the follower arm. Build records with
    /// [`mturn_by_interval`] or by hand.
    fn sync_set_angle_multiturn_by_interval(
        &mut self,
        cmds: &[MturnAngleByInterval],
    ) -> Result<()> {
        self.send_encoded(|o| request::encode_sync(cmds, o))
    }

    /// Clear the turn counter (`RESET_MULTI_TURN_ANGLE`) so the multi-turn
    /// angle folds back into one turn. Volatile. Send per id: Seeed found
    /// the broadcast form ineffective.
    fn reset_multi_turn(&mut self, id: u8) -> Result<()> {
        self.send_encoded(|o| request::encode_reset_multi_turn(id, o))
    }

    /// **Writes non-volatile memory.** Makes the current position the servo's
    /// permanent zero (`SET_ORIGIN_POINT`); it survives power cycles, and
    /// every angle recorded before refers to a frame that no longer exists.
    ///
    /// Calibration-time only — never call it from a control loop, both for
    /// the reason above and because flash has a finite write endurance.
    /// Seeed's teleoperator releases the joint (`stop(id, Release)`) and
    /// waits ~10 ms before calling this.
    fn set_origin_point(&mut self, id: u8) -> Result<()> {
        self.send_encoded(|o| request::encode_set_origin_point(id, o))
    }

    /// Read one memory-table entry; returns its raw content.
    fn read_data(&mut self, id: u8, address: u8) -> Result<Vec<u8>> {
        let r = self.transact_encoded(Code::ReadData, id, |o| {
            request::encode_read_data(id, address, o)
        })?;
        let d = proto::parse_read_data(r.code, &r.params)?;
        if d.address != address {
            return Err(Error::UnexpectedResponse {
                servo_id: id,
                detail: format!("read_data asked address {address}, got {}", d.address),
            });
        }
        Ok(d.content.to_vec())
    }

    /// Write one memory-table entry; `Ok(true)` if the servo accepted it.
    ///
    /// The SDK names only the five telemetry addresses in
    /// [`fashionstar_protocol::DataAddress`]; the rest of the table (and
    /// what a write to it does) is undocumented there, so treat any write as
    /// a configuration change until verified on hardware.
    fn write_data(&mut self, id: u8, address: u8, content: &[u8]) -> Result<bool> {
        let r = self.transact_encoded(Code::WriteData, id, |o| {
            request::encode_write_data(id, address, content, o)
        })?;
        Ok(proto::parse_write_data(r.code, &r.params)?.ok)
    }
}

impl<B: FsBus + ?Sized> FsCommands for B {}

/// Build a clamped `SET_SERVO_ANGLE_MTURN_BY_INTERVAL` record (see
/// [`FsCommands::set_angle_multiturn_by_interval`]).
pub fn mturn_by_interval(
    id: u8,
    angle_rad: f32,
    interval: Duration,
    t_acc: Duration,
    t_dec: Duration,
) -> MturnAngleByInterval {
    let t_acc_ms = ms_u16(t_acc.max(MIN_ACCEL_TIME));
    let t_dec_ms = ms_u16(t_dec.max(MIN_ACCEL_TIME));
    let interval_ms = ms_u32(interval)
        .max(t_acc_ms as u32 + t_dec_ms as u32)
        .min(MTURN_INTERVAL_MAX_MS);
    MturnAngleByInterval {
        id,
        angle: rad_to_raw(angle_rad).clamp(-MTURN_ANGLE_LIMIT, MTURN_ANGLE_LIMIT),
        interval_ms,
        t_acc_ms,
        t_dec_ms,
        power_mw: 0,
    }
}
