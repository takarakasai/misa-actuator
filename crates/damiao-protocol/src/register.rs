//! Register (RID) read/write/save protocol on the `0x7FF` config channel.
//!
//! All register access frames are 8 bytes sent to CAN ID [`REGISTER_ID`]
//! (`0x7FF`):
//!
//! | Byte | D0      | D1      | D2  | D3  | D4..D7        |
//! |------|---------|---------|-----|-----|---------------|
//! | Read | id_lo   | id_hi   |0x33 | rid | (don't care)  |
//! | Write| id_lo   | id_hi   |0x55 | rid | value (LE)    |
//! | Save | id_lo   | id_hi   |0xAA | rid | (don't care)  |
//!
//! `id_lo`/`id_hi` are the *target motor's* CAN_ID (little-endian). Integer
//! registers carry a little-endian `i32`; all other registers carry a
//! little-endian `f32`.

use crate::can_id::REGISTER_ID;

/// Register access command byte (placed in D2).
const CMD_READ: u8 = 0x33;
const CMD_WRITE: u8 = 0x55;
const CMD_SAVE: u8 = 0xAA;

/// Control mode written to [`Rid::CTRL_MODE`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ControlMode {
    /// MIT impedance control (pos/vel/kp/kd/tau feed-forward).
    Mit = 1,
    /// Position-Velocity (trapezoidal profile to a position).
    PosVel = 2,
    /// Velocity control.
    Vel = 3,
    /// Force-Position control.
    ForcePos = 4,
}

impl ControlMode {
    /// Decode a register value into a control mode.
    pub const fn from_raw(v: i32) -> Option<Self> {
        match v {
            1 => Some(ControlMode::Mit),
            2 => Some(ControlMode::PosVel),
            3 => Some(ControlMode::Vel),
            4 => Some(ControlMode::ForcePos),
            _ => None,
        }
    }
}

/// Well-known register IDs (a subset of the DAMIAO RID table).
pub struct Rid;

impl Rid {
    /// Under-voltage threshold (f32).
    pub const UV_VALUE: u8 = 0;
    /// Torque constant Kt (f32).
    pub const KT_VALUE: u8 = 1;
    /// Master / feedback CAN ID (int).
    pub const MST_ID: u8 = 7;
    /// Motor's listen CAN_ID / slave id (int).
    pub const ESC_ID: u8 = 8;
    /// Control mode: 1=MIT, 2=POS_VEL, 3=VEL, 4=FORCE_POS (int).
    pub const CTRL_MODE: u8 = 10;
    /// Position range limit PMAX (f32).
    pub const PMAX: u8 = 21;
    /// Velocity range limit VMAX (f32).
    pub const VMAX: u8 = 22;
    /// Torque range limit TMAX (f32).
    pub const TMAX: u8 = 23;
    /// CAN baud-rate code (int).
    pub const CAN_BR: u8 = 35;
    /// Firmware sub-version (int).
    pub const SUB_VER: u8 = 36;

    /// Whether register `rid` carries an integer value (vs. f32).
    ///
    /// Per the SDK: RIDs 7–10, 13–16 and 35–36 are integers; all others are
    /// `f32`.
    pub const fn is_int(rid: u8) -> bool {
        matches!(rid, 7..=10 | 13..=16 | 35..=36)
    }
}

/// Build a register-read request frame (CAN ID = `0x7FF`).
pub fn build_read_reg(can_id: u8, rid: u8) -> (u16, [u8; 8]) {
    let data = [
        can_id,
        0x00, // CAN_ID is 8-bit; high byte is always 0
        CMD_READ,
        rid,
        0,
        0,
        0,
        0,
    ];
    (REGISTER_ID, data)
}

/// Build a register-write frame carrying a little-endian `f32`.
pub fn build_write_reg_f32(can_id: u8, rid: u8, value: f32) -> (u16, [u8; 8]) {
    let v = value.to_le_bytes();
    let data = [can_id, 0x00, CMD_WRITE, rid, v[0], v[1], v[2], v[3]];
    (REGISTER_ID, data)
}

/// Build a register-write frame carrying a little-endian `i32`.
pub fn build_write_reg_int(can_id: u8, rid: u8, value: i32) -> (u16, [u8; 8]) {
    let v = value.to_le_bytes();
    let data = [can_id, 0x00, CMD_WRITE, rid, v[0], v[1], v[2], v[3]];
    (REGISTER_ID, data)
}

/// Build a "save all parameters to flash" frame.
///
/// DAMIAO register writes (`0x55`) only update RAM; this `0xAA` command commits
/// the current parameters to flash so they survive a power cycle. The motor
/// should be **disabled** before saving (per the official SDK). D3 is the fixed
/// sub-command `0x01`, not a RID — this saves *all* params, not one register.
///
/// Frame format verified against real DM-J4310 hardware: the motor ACKs the
/// `01 00 AA 01 ..` frame with a short `01 00 AA 01` reply on its MST_ID.
pub fn build_save_all(can_id: u8) -> (u16, [u8; 8]) {
    let data = [can_id, 0x00, CMD_SAVE, 0x01, 0, 0, 0, 0];
    (REGISTER_ID, data)
}

/// A decoded register reply: which motor, which register, and the 4 value
/// bytes (interpret as `f32` or `i32` per [`Rid::is_int`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RegReply {
    /// Echoed motor CAN_ID.
    pub can_id: u8,
    /// Register id.
    pub rid: u8,
    /// Raw little-endian value bytes (D4..D7).
    pub value: [u8; 4],
}

impl RegReply {
    /// Interpret the value as a little-endian `f32`.
    pub fn as_f32(&self) -> f32 {
        f32::from_le_bytes(self.value)
    }
    /// Interpret the value as a little-endian `i32`.
    pub fn as_i32(&self) -> i32 {
        i32::from_le_bytes(self.value)
    }
}

/// Parse a register reply payload.
///
/// **Important:** the motor sends register replies back on its **Master ID**
/// (`MST_ID`, default `0`), *not* on the `0x7FF` config channel — verified by
/// `candump` against real DM-J4310 hardware. So this matches on payload content
/// (D2 echoes the read/write command byte), not on the arbitration id. The
/// caller must skip its own `0x7FF` transmit echoes before calling this.
///
/// Returns `None` if `data` is not a register reply.
pub fn parse_reg_reply(data: &[u8]) -> Option<RegReply> {
    if data.len() < 8 {
        return None;
    }
    let cmd = data[2];
    if cmd != CMD_READ && cmd != CMD_WRITE {
        return None;
    }
    Some(RegReply {
        can_id: data[0],
        rid: data[3],
        value: [data[4], data[5], data[6], data[7]],
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_frame_layout() {
        let (id, data) = build_read_reg(0x05, Rid::ESC_ID);
        assert_eq!(id, 0x7FF);
        assert_eq!(data, [0x05, 0x00, 0x33, 8, 0, 0, 0, 0]);
    }

    #[test]
    fn write_int_mode_frame() {
        // Switch motor 1 to POS_VEL (mode value 2) via RID 10.
        let (id, data) = build_write_reg_int(0x01, Rid::CTRL_MODE, ControlMode::PosVel as i32);
        assert_eq!(id, 0x7FF);
        assert_eq!(data, [0x01, 0x00, 0x55, 10, 0x02, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn write_f32_frame() {
        let (_, data) = build_write_reg_f32(0x02, Rid::PMAX, 12.5);
        assert_eq!(&data[0..4], &[0x02, 0x00, 0x55, 21]);
        assert_eq!(f32::from_le_bytes([data[4], data[5], data[6], data[7]]), 12.5);
    }

    #[test]
    fn reg_reply_round_trip() {
        // A real reply (captured on MST_ID 0x000 from a DM-J4310):
        //   000  01 00 33 0A 02 00 00 00  → CTRL_MODE(10) read-back = 2 (PosVel)
        let reply = parse_reg_reply(&[0x01, 0x00, 0x33, Rid::CTRL_MODE, 0x02, 0, 0, 0]).unwrap();
        assert_eq!(reply.can_id, 0x01);
        assert_eq!(reply.rid, Rid::CTRL_MODE);
        assert_eq!(reply.as_i32(), 2);
    }

    #[test]
    fn non_reg_frame_rejected() {
        // A motion-feedback-shaped frame (D2 not a reg cmd) must not parse.
        assert!(parse_reg_reply(&[0x11, 0x00, 0x00, 0x00, 0, 0, 0, 0]).is_none());
    }

    #[test]
    fn save_all_frame_layout() {
        let (id, data) = build_save_all(0x01);
        assert_eq!(id, 0x7FF);
        assert_eq!(data, [0x01, 0x00, 0xAA, 0x01, 0, 0, 0, 0]);
    }

    #[test]
    fn int_register_classification() {
        assert!(Rid::is_int(Rid::MST_ID));
        assert!(Rid::is_int(Rid::ESC_ID));
        assert!(Rid::is_int(Rid::CTRL_MODE));
        assert!(Rid::is_int(Rid::CAN_BR));
        assert!(!Rid::is_int(Rid::PMAX));
        assert!(!Rid::is_int(Rid::KT_VALUE));
    }
}
