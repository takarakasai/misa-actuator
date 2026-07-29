//! Command codes for the MG4005 RS485 protocol.
//!
//! These follow the LK-Tech V3 RS485 command set. Cross-check with your
//! firmware manual: older revisions and OEM variants sometimes shift codes.

/// Symbolic command codes.
///
/// Convert to the wire byte with `cmd as u8`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Command {
    /// Read PID parameters.
    ReadPid = 0x30,
    /// Write PID parameters to RAM.
    WritePidRam = 0x31,
    /// Write PID parameters to ROM.
    WritePidRom = 0x32,
    /// Read acceleration value.
    ReadAccel = 0x33,
    /// Write acceleration value to RAM.
    WriteAccelRam = 0x34,

    /// Read absolute encoder position.
    ReadEncoder = 0x90,
    /// Write encoder offset.
    WriteEncoderOffset = 0x91,
    /// Set the current encoder position as the zero offset (writes ROM).
    WriteCurrentPosAsZero = 0x19,

    /// Read multi-turn angle.
    ReadMultiTurnAngle = 0x92,
    /// Read single-turn angle.
    ReadSingleTurnAngle = 0x94,
    /// Clear stored motor angle (zero the multi-turn counter).
    ClearMotorAngle = 0x95,

    /// Read motor state 1 (temperature, voltage, error flags).
    ReadMotorState1 = 0x9A,
    /// Clear error flag.
    ClearError = 0x9B,
    /// Read motor state 2 (temperature, current, speed, position).
    ReadMotorState2 = 0x9C,
    /// Read motor state 3 (phase A/B/C currents).
    ReadMotorState3 = 0x9D,

    /// Power off the motor (clears the running flag).
    MotorOff = 0x80,
    /// Stop the motor (keeps the running flag set).
    MotorStop = 0x81,
    /// Resume motor operation after a stop.
    MotorRun = 0x88,

    /// Closed-loop torque control.
    TorqueClosedLoop = 0xA1,
    /// Closed-loop speed control.
    SpeedClosedLoop = 0xA2,
    /// Closed-loop position control 1 (multi-turn absolute).
    PositionClosedLoop1 = 0xA3,
    /// Closed-loop position control 2 (multi-turn absolute with speed limit).
    PositionClosedLoop2 = 0xA4,
    /// Closed-loop position control 3 (single-turn with direction).
    PositionClosedLoop3 = 0xA5,
    /// Closed-loop position control 4 (single-turn with direction and speed limit).
    PositionClosedLoop4 = 0xA6,
    /// Incremental position control 1 (relative move, direction from sign).
    IncrementalPosition1 = 0xA7,
    /// Incremental position control 2 (relative move with speed limit).
    IncrementalPosition2 = 0xA8,

    /// Read a control parameter (PID / limit / ramp). Param ID in `DATA[0]`.
    ReadControlParam = 0xC0,
    /// Write a control parameter to RAM (lost on power cycle).
    WriteControlParamRam = 0xC1,

    /// Read a setting parameter — a *different* parameter space from
    /// `ReadControlParam` (see [`SettingParamId`]'s doc comment for why the
    /// two must not be confused despite overlapping numeric sub-IDs).
    ReadSettingParam = 0x40,
    /// Write a setting parameter to RAM. Must be followed by
    /// [`Command::SaveSettingParam`] for the change to survive a power
    /// cycle (per the manual: "the Save setting parameters command must be
    /// sent before the data is written into ROM").
    WriteSettingParam = 0x42,
    /// Commit all `WriteSettingParam` RAM writes to ROM. Takes effect after
    /// a restart ([`Command::MotorRestart`]) or power cycle.
    SaveSettingParam = 0x44,
    /// Restart the motor (equivalent to a power cycle). No reply is sent.
    MotorRestart = 0x07,

    /// Engage/release the brake, or read its current state.
    BrakeControl = 0x8C,
}

/// `DATA[0]` selector for `ReadControlParam` (`0xC0`) / `WriteControlParamRam` (`0xC1`).
///
/// Each ID has a fixed 6-byte value layout in `DATA[1..7]` — see
/// [`crate::response::parse_control_param`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ControlParamId {
    /// Position-loop PID (Kp/Ki/Kd, each `u16` 0..=2000).
    PositionLoopPid = 0x0A,
    /// Speed-loop PID (Kp/Ki/Kd, each `u16` 0..=2000).
    SpeedLoopPid = 0x0B,
    /// Current-loop PID (Kp/Ki/Kd, each `u16` 0..=2000).
    CurrentLoopPid = 0x0C,
    /// Torque (current) limit, raw int16. MS: 0..=850, MF/MHF/MG: 0..=2000.
    TorqueLimit = 0x1E,
    /// Speed limit, int32 (0.01 deg/s units, 0..=600000).
    SpeedLimit = 0x20,
    /// Angle limit, int32 (0.01 deg units).
    AngleLimit = 0x22,
    /// Current ramp, int32 (0..=30000).
    CurrentRamp = 0x24,
    /// Speed ramp, int32 (1 dps/s units, 0..=600000).
    SpeedRamp = 0x26,
}

impl ControlParamId {
    /// Wire byte for this parameter selector.
    #[inline]
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// `DATA[1]` selector for `ReadSettingParam` (`0x40`) / `WriteSettingParam`
/// (`0x42`) — the "One Parameter Command" sub-format ("Setting Parameter
/// Table" in the manual). Every request/reply for this sub-format carries a
/// **fixed marker byte `0x05` at `DATA[0]`** before this selector — see
/// [`crate::request::encode_read_setting_param`].
///
/// **Do not confuse this with [`ControlParamId`]**: several numeric values
/// coincide (`0x0A`/`0x0B`/`0x0C` mean Driver ID/Bus Type/RS485 Baudrate
/// here, but Position/Speed/Current-loop PID under `ReadControlParam`) — the
/// two are disambiguated only by which command byte wraps them
/// (`0x40`/`0x42` vs `0xC0`/`0xC1`). This is a genuine quirk of the official
/// manual (`ref/can_protocol_en.md`/`rs485_protocol_en.md`), not a
/// transcription error.
///
/// The manual's "Multiple Parameter Command" sub-format (Position/Speed/
/// Current Loop PID persisted via this same Setting/ROM path, sub-IDs
/// `0xA0`/`0xA4`/`0xA8`) is **not implemented** — those gains are already
/// readable via [`ControlParamId`]; only the ROM-persistence path for them
/// is missing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SettingParamId {
    /// Driver ID (`u8`, 0..=32).
    DriverId = 0x0A,
    /// Bus type (`u8`): `0`=None, `1`=RS485, `2`=CAN.
    BusType = 0x0B,
    /// RS485 baud rate code (`u8`, 0..=10 — see the manual's baud-rate table).
    Rs485Baudrate = 0x0C,
    /// CAN baud rate code (`u8`, 0..=4 — see the manual's baud-rate table).
    CanBaudrate = 0x0D,
    /// Max power (`i16`). Range 0..=850 (MS) or 0..=2000 (MF/MHF/MG).
    MaxPower = 0xE0,
    /// Max speed (`i32`, 0.01 deg/s units, 0..=600000).
    MaxSpeed = 0xE2,
    /// Max angle (`i32`, 0.01 deg units).
    MaxAngle = 0xE4,
    /// Current ramp (`i16`, 0..=30000). **Note**: the manual declares this
    /// `int16` here but `int32` under the same name/range in
    /// [`ControlParamId::CurrentRamp`] — a discrepancy in the source
    /// document itself (see `doc/can-rs485-manual-analysis.md`), kept as
    /// printed rather than silently reconciled.
    CurrentRamp = 0xEA,
    /// Speed ramp (`i32`, 1 dps/s units, 0..=600000).
    SpeedRamp = 0xEC,
}

impl SettingParamId {
    /// Wire byte for this parameter selector.
    #[inline]
    pub const fn code(self) -> u8 {
        self as u8
    }
}

impl Command {
    /// Wire byte for this command.
    #[inline]
    pub const fn code(self) -> u8 {
        self as u8
    }
}

impl From<Command> for u8 {
    #[inline]
    fn from(c: Command) -> u8 {
        c as u8
    }
}
