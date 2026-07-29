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
use crate::limits::{MotorModel, RegisterLayout};

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

/// Register IDs from the DAMIAO RID table.
///
/// Covers the full contiguous `0x00`–`0x25` block that the official
/// DM-J4310-2EC and DM-J3507-2EC manuals both document ("Register Map"),
/// with one exception noted on [`Self::BOOT_VER`]. Range/access/type
/// annotations below are transcribed from those manuals; `fmax` appears in
/// the source as an unresolved symbolic bound (a firmware-defined ceiling,
/// not a literal).
///
/// **Not covered here**: the model-specific registers above `0x25`, whose
/// addresses *differ between models* (DM4310 has `Imax`/`VBus`/`Tpcb`/`Tmtr`/
/// phase-current offsets at `0x3B`–`0x41` and `m_off` at `0x38`; DM3507 has
/// `u_off`/`v_off`/`k1`/`k2` at `0x32`–`0x35` and `m_off` at `0x36`). Reading
/// those through a single shared constant would address the wrong register on
/// the other model, so they need a model-aware API rather than a flat
/// constant — see `doc/dm4310-dm3507-manual-analysis.md`. `dir` (`0x37`),
/// `p_m` (`0x50`) and `xout` (`0x51`) *are* common to both models but are
/// runtime state rather than configuration, so they are likewise left out of
/// this configuration-oriented set.
pub struct Rid;

impl Rid {
    /// Under-voltage threshold (f32, RW, `(10.0, fmax]`).
    pub const UV_VALUE: u8 = 0;
    /// Torque constant Kt (f32, RW, `[0.0, fmax]`).
    pub const KT_VALUE: u8 = 1;
    /// Over-temperature threshold (f32, RW, `[80.0, 200)`).
    pub const OT_VALUE: u8 = 2;
    /// Over-current threshold (f32, RW, `(0.0, 1.0)`).
    pub const OC_VALUE: u8 = 3;
    /// Acceleration for the trapezoidal profile (f32, RW, `(0.0, fmax)`).
    pub const ACC: u8 = 4;
    /// Deceleration for the trapezoidal profile (f32, RW, `[-fmax, 0.0)` —
    /// note the manual's range is **negative**, unlike [`Self::ACC`]).
    pub const DEC: u8 = 5;
    /// Max velocity (f32, RW, `(0.0, fmax]`).
    pub const MAX_SPD: u8 = 6;
    /// Master / feedback CAN ID (int, RW, `[0, 0x7FF]`).
    pub const MST_ID: u8 = 7;
    /// Motor's listen CAN_ID / slave id (int, RW, `[0, 0x7FF]`).
    pub const ESC_ID: u8 = 8;
    /// Communication timeout threshold (int, RW, `[0, 2^32-1]`).
    pub const TIMEOUT: u8 = 9;
    /// Control mode: 1=MIT, 2=POS_VEL, 3=VEL, 4=FORCE_POS (int, RW, `[0, 4]`).
    pub const CTRL_MODE: u8 = 10;
    /// Viscous damping (f32, RO) — identified during calibration.
    pub const DAMP: u8 = 11;
    /// Rotor inertia (f32, RO) — identified during calibration.
    pub const INERTIA: u8 = 12;
    /// Hardware version (int, RO). Labeled "Reserved" in the official
    /// DM-J4310-2EC/DM-J3507-2EC manuals' register map despite the `hw_ver`
    /// name — treat with the same caution as any other "Reserved" field.
    pub const HW_VER: u8 = 13;
    /// Firmware version (int, RO). Confirmed against the official
    /// DM-J4310-2EC/DM-J3507-2EC manuals ("Register Map" table, `sw_ver` /
    /// "Firmware Version") — this is the register the vendor's own "Read
    /// Version" GUI button reads, not [`Self::SUB_VER`].
    pub const SW_VER: u8 = 14;
    /// Serial number (int, RO). Labeled "Reserved" in the official manuals
    /// despite the `SN` name — treat with the same caution as [`Self::HW_VER`].
    pub const SN: u8 = 15;
    /// Number of pole pairs (int, RO), auto-identified during calibration.
    pub const NPP: u8 = 16;
    /// Phase resistance (f32, RO) — identified during calibration.
    pub const RS: u8 = 17;
    /// Phase inductance (f32, RO) — identified during calibration.
    pub const LS: u8 = 18;
    /// Flux linkage (f32, RO) — identified during calibration.
    pub const FLUX: u8 = 19;
    /// Gear reduction ratio (f32, RO), auto-identified/preconfigured — per
    /// the manual's Parameter Management section, "This parameter is
    /// preconfigured. Do not modify."
    pub const GR: u8 = 20;
    /// Position range limit PMAX (f32, RW, `(0.0, fmax]`).
    pub const PMAX: u8 = 21;
    /// Velocity range limit VMAX (f32, RW, `(0.0, fmax]`).
    pub const VMAX: u8 = 22;
    /// Torque range limit TMAX (f32, RW, `(0.0, fmax]`).
    pub const TMAX: u8 = 23;
    /// Current-loop bandwidth (f32, RW, `[100.0, 1.0e4]`).
    pub const I_BW: u8 = 24;
    /// Velocity-loop Kp (f32, RW, `[0.0, fmax]`).
    pub const KP_ASR: u8 = 25;
    /// Velocity-loop Ki (f32, RW, `[0.0, fmax]`).
    pub const KI_ASR: u8 = 26;
    /// Position-loop Kp (f32, RW, `[0.0, fmax]`).
    pub const KP_APR: u8 = 27;
    /// Position-loop Ki (f32, RW, `[0.0, fmax]`).
    pub const KI_APR: u8 = 28;
    /// Over-voltage threshold (f32, RW). **Note**: the manuals print this
    /// row's valid range literally as "TBD" — unlike every other row, the
    /// vendor leaves it undocumented. Cross-reference the Specifications
    /// table's recommended over-voltage threshold if a bound is needed.
    pub const OV_VALUE: u8 = 29;
    /// Gear torque efficiency (f32, RW, `(0.0, 1.0]`).
    pub const GREF: u8 = 30;
    /// Velocity-loop damping coefficient (f32, RW, `[1.0, 30.0]`).
    pub const DETA: u8 = 31;
    /// Velocity-loop filter bandwidth (f32, RW, `(0.0, 500.0)`).
    pub const V_BW: u8 = 32;
    /// Iq gain (f32, RW, `[100.0, 1.0e4]`).
    pub const IQ_C1: u8 = 33;
    /// Velocity-loop gain factor (f32, RW, `(0.0, 1.0e4]`).
    pub const VL_C1: u8 = 34;
    /// CAN baud-rate code (int, RW, `[0, 9]`).
    pub const CAN_BR: u8 = 35;
    /// Minor/sub version (int, RO) — a secondary version field, **not** the
    /// main firmware version (see [`Self::SW_VER`]). Named plainly
    /// "Sub-version" in the official manuals.
    pub const SUB_VER: u8 = 36;
    /// Bootloader version (int, RO).
    ///
    /// **Not present on every model** — the only register in this block that
    /// is not universal. Models on the J3507 register layout jump straight
    /// from `0x24` (`sub_ver`) to `0x32` (`u_off`), so reading this register
    /// there targets an undocumented address. Gate on
    /// [`MotorModel::has_boot_ver`] first.
    pub const BOOT_VER: u8 = 37;

    /// Whether register `rid` carries an integer value (vs. f32).
    ///
    /// Per the SDK: RIDs 7–10, 13–16 and 35–37 are integers; all others are
    /// `f32`. Cross-checked against the official DM-J4310-2EC/DM-J3507-2EC
    /// manuals' "Register Map" Type column — confirmed exactly matching for
    /// every RID both manuals document, including the whole `0x00`–`0x25`
    /// block enumerated above (the only integer-typed register outside the
    /// existing ranges would be `TIMEOUT` = 9, already covered by `7..=10`).
    pub const fn is_int(rid: u8) -> bool {
        matches!(rid, 7..=10 | 13..=16 | 35..=37)
    }
}

/// A register above the common `0x00`–`0x25` block, addressed **by meaning
/// rather than by number** because its RID varies between motor models.
///
/// The DM-J4310-2EC and DM-J3507-2EC register maps diverge above `0x25`: they
/// expose different diagnostic registers, and the one register they share by
/// name (`m_off`) sits at a *different address* on each (`0x38` vs `0x36`).
/// A single flat constant per name would therefore silently read the wrong
/// register on one of the two models — the reason these are not in [`Rid`].
///
/// Resolve to a wire RID with [`ModelReg::rid`], which returns `None` when the
/// register does not exist on that model. Every register here is `f32`-typed
/// (consistent with [`Rid::is_int`] returning `false` for all of them).
///
/// All values are `RO` (read-only) per both manuals' Access column.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModelReg {
    /// `dir` — Direction. Same RID (`0x37`) on both models.
    Direction,
    /// `m_off` — motor-side angle offset. **Different RID per model**
    /// (`0x38` on DM4310, `0x36` on DM3507): the reason this enum exists.
    AngleOffset,
    /// `x_off` — output-shaft angle offset. **DM-J10422P only** (`0x36`).
    /// That address carries `m_off` under the J3507 layout instead.
    OutputShaftAngleOffset,
    /// `p_m` — motor-side position, in rad. Same RID (`0x50`) on every model.
    /// Computed from the rotor position rather than measured directly.
    MotorPosition,
    /// `xout` — output-shaft position, in rad, measured directly by the
    /// output-shaft encoder. Same RID (`0x51`) on every model.
    OutputShaftPosition,

    /// `Imax` — driver current *limit*. J4310 layout only (`0x3B`).
    /// DM-J10422P uses that address for [`Self::PhaseCurrentBase`] instead,
    /// which is a different quantity.
    DriverCurrentLimit,
    /// `IBase` — phase-current base peak value. **DM-J10422P only** (`0x3B`).
    /// See [`Self::DriverCurrentLimit`] for why these are not one variant.
    PhaseCurrentBase,
    /// `VBus` — bus voltage. J4310 and J10422 layouts (`0x3C`).
    BusVoltage,
    /// `Tpcb` — PCB temperature. J4310 and J10422 layouts (`0x3D`).
    /// DM-J10422P's manual names this register `TMOS` ("MOS temperature") —
    /// the MOSFETs sit on the PCB, so this is taken to be the same sensor
    /// under a different label.
    PcbTemperature,
    /// `Tmtr` — motor temperature. J4310 and J10422 layouts (`0x3E`).
    /// DM-J10422P's manual names this register `T_CEL` ("motor winding
    /// temperature").
    MotorTemperature,
    /// `I_U_OFF` — phase-U current offset. J4310 and J10422 layouts (`0x3F`).
    PhaseUCurrentOffset,
    /// `I_V_OFF` — phase-V current offset. J4310 and J10422 layouts (`0x40`).
    PhaseVCurrentOffset,
    /// `I_W_OFF` — phase-W current offset. J4310 and J10422 layouts (`0x41`).
    PhaseWCurrentOffset,

    /// `u_off` — phase-U offset. DM3507 only (`0x32`).
    ///
    /// **Note**: the manuals describe this as a plain "U-phase offset", while
    /// DM4310's [`Self::PhaseUCurrentOffset`] is a "Phase U *Current*
    /// Offset" at an unrelated address. They may well be the same quantity
    /// under different names, but the two manuals word them differently and
    /// DM3507 has no `w_off` counterpart, so they are kept distinct rather
    /// than merged on a guess.
    PhaseUOffset,
    /// `v_off` — phase-V offset. DM3507 only (`0x33`). See
    /// [`Self::PhaseUOffset`] on why this is not merged with the DM4310 name.
    PhaseVOffset,
    /// `k1` — compensation coefficient 1. DM3507 only (`0x34`).
    CompensationK1,
    /// `k2` — compensation coefficient 2. DM3507 only (`0x35`).
    CompensationK2,
}

impl ModelReg {
    /// Every model-specific register, in the order a diagnostics dump should
    /// present them: shared registers first, then per-model blocks.
    pub const ALL: &'static [ModelReg] = &[
        ModelReg::Direction,
        ModelReg::AngleOffset,
        ModelReg::OutputShaftAngleOffset,
        ModelReg::MotorPosition,
        ModelReg::OutputShaftPosition,
        ModelReg::DriverCurrentLimit,
        ModelReg::PhaseCurrentBase,
        ModelReg::BusVoltage,
        ModelReg::PcbTemperature,
        ModelReg::MotorTemperature,
        ModelReg::PhaseUCurrentOffset,
        ModelReg::PhaseVCurrentOffset,
        ModelReg::PhaseWCurrentOffset,
        ModelReg::PhaseUOffset,
        ModelReg::PhaseVOffset,
        ModelReg::CompensationK1,
        ModelReg::CompensationK2,
    ];

    /// The wire RID of this register on `model`, or `None` if `model`'s
    /// register map does not document it.
    ///
    /// Dispatches on [`MotorModel::register_layout`] rather than on the model
    /// itself, so adding a model is a one-line mapping in `register_layout`
    /// and needs no change here.
    pub const fn rid(self, model: MotorModel) -> Option<u8> {
        use RegisterLayout as L;
        match (self, model.register_layout()) {
            // Same address under every layout.
            (ModelReg::Direction, _) => Some(0x37),
            (ModelReg::MotorPosition, _) => Some(0x50),
            (ModelReg::OutputShaftPosition, _) => Some(0x51),

            // `m_off`: 0x38 on the J4310/J10422 layouts, 0x36 on J3507 —
            // the reason this enum exists.
            (ModelReg::AngleOffset, L::J4310 | L::J10422) => Some(0x38),
            (ModelReg::AngleOffset, L::J3507) => Some(0x36),

            // `x_off`, J10422 only. Note 0x36 is `x_off` here but `m_off`
            // under J3507: one address, two meanings.
            (ModelReg::OutputShaftAngleOffset, L::J10422) => Some(0x36),
            (ModelReg::OutputShaftAngleOffset, L::J4310 | L::J3507) => None,

            // Diagnostics block 0x3B-0x41, absent entirely on J3507.
            // 0x3B is `Imax` (a current *limit*) on J4310 but `IBase` (a
            // current *base/scale*) on J10422 — different quantities, so they
            // are distinct variants rather than one renamed register.
            (ModelReg::DriverCurrentLimit, L::J4310) => Some(0x3B),
            (ModelReg::DriverCurrentLimit, L::J3507 | L::J10422) => None,
            (ModelReg::PhaseCurrentBase, L::J10422) => Some(0x3B),
            (ModelReg::PhaseCurrentBase, L::J4310 | L::J3507) => None,
            (ModelReg::BusVoltage, L::J4310 | L::J10422) => Some(0x3C),
            (ModelReg::PcbTemperature, L::J4310 | L::J10422) => Some(0x3D),
            (ModelReg::MotorTemperature, L::J4310 | L::J10422) => Some(0x3E),
            (ModelReg::PhaseUCurrentOffset, L::J4310 | L::J10422) => Some(0x3F),
            (ModelReg::PhaseVCurrentOffset, L::J4310 | L::J10422) => Some(0x40),
            (ModelReg::PhaseWCurrentOffset, L::J4310 | L::J10422) => Some(0x41),
            (
                ModelReg::BusVoltage
                | ModelReg::PcbTemperature
                | ModelReg::MotorTemperature
                | ModelReg::PhaseUCurrentOffset
                | ModelReg::PhaseVCurrentOffset
                | ModelReg::PhaseWCurrentOffset,
                L::J3507,
            ) => None,

            // J3507-only registers.
            (ModelReg::PhaseUOffset, L::J3507) => Some(0x32),
            (ModelReg::PhaseVOffset, L::J3507) => Some(0x33),
            (ModelReg::CompensationK1, L::J3507) => Some(0x34),
            (ModelReg::CompensationK2, L::J3507) => Some(0x35),
            (
                ModelReg::PhaseUOffset
                | ModelReg::PhaseVOffset
                | ModelReg::CompensationK1
                | ModelReg::CompensationK2,
                L::J4310 | L::J10422,
            ) => None,
        }
    }

    /// The register's name as printed in the manuals' Register Map.
    pub const fn name(self) -> &'static str {
        match self {
            ModelReg::Direction => "dir",
            ModelReg::AngleOffset => "m_off",
            ModelReg::OutputShaftAngleOffset => "x_off",
            ModelReg::MotorPosition => "p_m",
            ModelReg::OutputShaftPosition => "xout",
            ModelReg::DriverCurrentLimit => "Imax",
            ModelReg::PhaseCurrentBase => "IBase",
            ModelReg::BusVoltage => "VBus",
            ModelReg::PcbTemperature => "Tpcb",
            ModelReg::MotorTemperature => "Tmtr",
            ModelReg::PhaseUCurrentOffset => "I_U_OFF",
            ModelReg::PhaseVCurrentOffset => "I_V_OFF",
            ModelReg::PhaseWCurrentOffset => "I_W_OFF",
            ModelReg::PhaseUOffset => "u_off",
            ModelReg::PhaseVOffset => "v_off",
            ModelReg::CompensationK1 => "k1",
            ModelReg::CompensationK2 => "k2",
        }
    }

    /// A short human-readable description (the manuals' Description column).
    pub const fn description(self) -> &'static str {
        match self {
            ModelReg::Direction => "direction",
            ModelReg::AngleOffset => "motor-side angle offset",
            ModelReg::OutputShaftAngleOffset => "output shaft angle offset",
            ModelReg::MotorPosition => "motor position (rad)",
            ModelReg::OutputShaftPosition => "output shaft position (rad)",
            ModelReg::DriverCurrentLimit => "driver current limit",
            ModelReg::PhaseCurrentBase => "phase current base peak value",
            ModelReg::BusVoltage => "bus voltage",
            ModelReg::PcbTemperature => "PCB temperature",
            ModelReg::MotorTemperature => "motor temperature",
            ModelReg::PhaseUCurrentOffset => "phase U current offset",
            ModelReg::PhaseVCurrentOffset => "phase V current offset",
            ModelReg::PhaseWCurrentOffset => "phase W current offset",
            ModelReg::PhaseUOffset => "U-phase offset",
            ModelReg::PhaseVOffset => "V-phase offset",
            ModelReg::CompensationK1 => "compensation coefficient 1",
            ModelReg::CompensationK2 => "compensation coefficient 2",
        }
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

    /// Pin [`Rid::is_int`] to the official manuals' "Type" column for every
    /// register in the documented `0x00`-`0x25` block. Transcribed from the
    /// DM-J4310-2EC / DM-J3507-2EC "Register Map" tables, which agree on the
    /// type of every row in this range.
    #[test]
    fn int_classification_matches_manual_type_column_for_whole_block() {
        // The only `uint32` rows in 0x00..=0x25; every other row is `float`.
        const INT_RIDS: &[u8] = &[
            Rid::MST_ID,   // 7
            Rid::ESC_ID,   // 8
            Rid::TIMEOUT,  // 9
            Rid::CTRL_MODE,// 10
            Rid::HW_VER,   // 13
            Rid::SW_VER,   // 14
            Rid::SN,       // 15
            Rid::NPP,      // 16
            Rid::CAN_BR,   // 35
            Rid::SUB_VER,  // 36
            Rid::BOOT_VER, // 37
        ];
        for rid in 0..=37u8 {
            let expected = INT_RIDS.contains(&rid);
            assert_eq!(
                Rid::is_int(rid),
                expected,
                "RID {rid} misclassified: manual says {}",
                if expected { "uint32" } else { "float" }
            );
        }
    }

    /// Every constant in the block must map to the address the manuals print,
    /// so a typo in one shifts nothing silently past its neighbours.
    #[test]
    fn register_addresses_match_manual() {
        let expected: &[(u8, &str)] = &[
            (Rid::UV_VALUE, "UV_Value"),
            (Rid::KT_VALUE, "KT_Value"),
            (Rid::OT_VALUE, "OT_Value"),
            (Rid::OC_VALUE, "OC_Value"),
            (Rid::ACC, "ACC"),
            (Rid::DEC, "DEC"),
            (Rid::MAX_SPD, "MAX_SPD"),
            (Rid::MST_ID, "MST_ID"),
            (Rid::ESC_ID, "ESC_ID"),
            (Rid::TIMEOUT, "TIMEOUT"),
            (Rid::CTRL_MODE, "CTRL_MODE"),
            (Rid::DAMP, "Damp"),
            (Rid::INERTIA, "Inertia"),
            (Rid::HW_VER, "hw_ver"),
            (Rid::SW_VER, "sw_ver"),
            (Rid::SN, "SN"),
            (Rid::NPP, "NPP"),
            (Rid::RS, "Rs"),
            (Rid::LS, "Ls"),
            (Rid::FLUX, "Flux"),
            (Rid::GR, "Gr"),
            (Rid::PMAX, "PMAX"),
            (Rid::VMAX, "VMAX"),
            (Rid::TMAX, "TMAX"),
            (Rid::I_BW, "I_BW"),
            (Rid::KP_ASR, "KP_ASR"),
            (Rid::KI_ASR, "KI_ASR"),
            (Rid::KP_APR, "KP_APR"),
            (Rid::KI_APR, "KI_APR"),
            (Rid::OV_VALUE, "OV_Value"),
            (Rid::GREF, "GREF"),
            (Rid::DETA, "Deta"),
            (Rid::V_BW, "V_BW"),
            (Rid::IQ_C1, "IQ_c1"),
            (Rid::VL_C1, "VL_c1"),
            (Rid::CAN_BR, "can_br"),
            (Rid::SUB_VER, "sub_ver"),
            (Rid::BOOT_VER, "Boot_ver"),
        ];
        // The manual's block is contiguous 0x00..=0x25, so the constants must
        // enumerate exactly that, in order, with no gaps or duplicates.
        for (i, (rid, name)) in expected.iter().enumerate() {
            assert_eq!(*rid as usize, i, "{name} is not at RID {i}");
        }
        assert_eq!(expected.len(), 0x26);
    }

    /// Pin every [`ModelReg`] address to the nine manuals' Register Map
    /// tables, one column per register layout.
    #[test]
    fn model_register_addresses_match_manuals() {
        use ModelReg as M;
        /// One row of the manuals' Register Map, as (register, J4310-layout
        /// RID, J3507-layout RID, J10422-layout RID).
        type Row = (ModelReg, Option<u8>, Option<u8>, Option<u8>);
        let expected: &[Row] = &[
            (M::PhaseUOffset, None, Some(0x32), None),
            (M::PhaseVOffset, None, Some(0x33), None),
            (M::CompensationK1, None, Some(0x34), None),
            (M::CompensationK2, None, Some(0x35), None),
            // 0x36 is `m_off` on J3507 but `x_off` on J10422 — one address,
            // two meanings, which is why these are separate variants.
            (M::AngleOffset, Some(0x38), Some(0x36), Some(0x38)),
            (M::OutputShaftAngleOffset, None, None, Some(0x36)),
            (M::Direction, Some(0x37), Some(0x37), Some(0x37)),
            // 0x3B is `Imax` on J4310 but `IBase` on J10422.
            (M::DriverCurrentLimit, Some(0x3B), None, None),
            (M::PhaseCurrentBase, None, None, Some(0x3B)),
            (M::BusVoltage, Some(0x3C), None, Some(0x3C)),
            (M::PcbTemperature, Some(0x3D), None, Some(0x3D)),
            (M::MotorTemperature, Some(0x3E), None, Some(0x3E)),
            (M::PhaseUCurrentOffset, Some(0x3F), None, Some(0x3F)),
            (M::PhaseVCurrentOffset, Some(0x40), None, Some(0x40)),
            (M::PhaseWCurrentOffset, Some(0x41), None, Some(0x41)),
            (M::MotorPosition, Some(0x50), Some(0x50), Some(0x50)),
            (M::OutputShaftPosition, Some(0x51), Some(0x51), Some(0x51)),
        ];
        for (reg, j4310, j3507, j10422) in expected {
            assert_eq!(reg.rid(MotorModel::Dm4310), *j4310, "{} J4310", reg.name());
            assert_eq!(reg.rid(MotorModel::Dm3507), *j3507, "{} J3507", reg.name());
            assert_eq!(
                reg.rid(MotorModel::Dm10422P),
                *j10422,
                "{} J10422",
                reg.name()
            );
        }
        // ALL must cover exactly the registers enumerated above.
        assert_eq!(ModelReg::ALL.len(), expected.len());
        for (reg, _, _, _) in expected {
            assert!(ModelReg::ALL.contains(reg), "{} missing from ALL", reg.name());
        }
    }

    /// Every model must resolve identically to the other members of its
    /// layout — that is the property that makes adding a model a one-liner.
    #[test]
    fn models_sharing_a_layout_resolve_identically() {
        for &model in MotorModel::ALL {
            let representative = match model.register_layout() {
                RegisterLayout::J4310 => MotorModel::Dm4310,
                RegisterLayout::J3507 => MotorModel::Dm3507,
                RegisterLayout::J10422 => MotorModel::Dm10422P,
            };
            for reg in ModelReg::ALL {
                assert_eq!(
                    reg.rid(model),
                    reg.rid(representative),
                    "{} disagrees with its layout representative {} for {}",
                    model.name(),
                    representative.name(),
                    reg.name()
                );
            }
        }
    }

    /// `Boot_ver` is the one non-universal register in the `0x00`-`0x25`
    /// block; it must track the layout, not the model list.
    #[test]
    fn boot_ver_presence_follows_layout() {
        for &model in MotorModel::ALL {
            let expected = !matches!(model.register_layout(), RegisterLayout::J3507);
            assert_eq!(
                model.has_boot_ver(),
                expected,
                "{} boot_ver presence",
                model.name()
            );
        }
        assert!(MotorModel::Dm4310.has_boot_ver());
        assert!(!MotorModel::Dm8009.has_boot_ver());
        assert!(MotorModel::Dm10422P.has_boot_ver());
    }

    /// `m_off` is the trap this enum exists to prevent: reading DM4310's
    /// address on a DM3507 would hit `dir` instead.
    #[test]
    fn angle_offset_address_differs_between_models() {
        let dm4310 = ModelReg::AngleOffset.rid(MotorModel::Dm4310).unwrap();
        let dm3507 = ModelReg::AngleOffset.rid(MotorModel::Dm3507).unwrap();
        assert_ne!(dm4310, dm3507);
        // DM4310's m_off address is DM3507's... nothing at all (0x38 is
        // undocumented there), while DM3507's m_off (0x36) is likewise
        // undocumented on DM4310 — so a flat constant is wrong either way.
        assert_eq!(dm4310, 0x38);
        assert_eq!(dm3507, 0x36);
    }

    /// Every model-specific register is `f32`, so `is_int` must reject them
    /// all — otherwise a dump would decode floats as integers.
    #[test]
    fn model_registers_are_all_float_typed() {
        for reg in ModelReg::ALL {
            for model in [MotorModel::Dm4310, MotorModel::Dm3507] {
                if let Some(rid) = reg.rid(model) {
                    assert!(
                        !Rid::is_int(rid),
                        "{} (RID {rid:#04X}) must decode as f32",
                        reg.name()
                    );
                }
            }
        }
    }
}
