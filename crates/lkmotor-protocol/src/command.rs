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

    /// Read a control parameter (PID / limit / ramp). Param ID in `DATA[0]`.
    ReadControlParam = 0xC0,
    /// Write a control parameter to RAM (lost on power cycle).
    WriteControlParamRam = 0xC1,
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
