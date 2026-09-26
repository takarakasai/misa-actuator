//! Command codes and small enumerations.
//!
//! Codes are the `CODE_*` constants of `UartServoManager` in the SDK's
//! `uservo.py`. The SDK defines a few codes it never sends (`5` is marked
//! "unused") — they are listed for completeness but have no encoder here.

/// Servo id that addresses every servo on the bus.
///
/// Used by the vendor teleop script (`stararm102_ro.py`) for the start-up
/// unlock: `stop_on_control_mode(0xff, 0x10, 0x00)`. Only use it for
/// fire-and-forget commands — every servo would answer a query at once and
/// the replies would collide on the half-duplex line.
///
/// **Not every command honours it.** Seeed's teleoperator notes that
/// `reset_multi_turn_angle(0xFF)` from the old SDK "is out of range" and
/// resets each id individually instead. Prefer per-id commands except for
/// the stop/unlock, where the vendor script shows broadcast works.
pub const BROADCAST_ID: u8 = 0xFF;

/// Single-turn angle clamp used by the SDK (`±1800` in 0.1° = `±180°`).
pub const SINGLE_TURN_ANGLE_LIMIT: i16 = 1800;

/// Multi-turn angle clamp used by the SDK (`±3_686_400` in 0.1° = `±1024`
/// turns).
pub const MTURN_ANGLE_LIMIT: i32 = 3_686_400;

/// Upper clamp on the multi-turn move interval used by the SDK (ms).
pub const MTURN_INTERVAL_MAX_MS: u32 = 4_096_000;

/// Command (function) codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Code {
    /// `CODE_PING`. Reply: `[id]`.
    Ping = 1,
    /// `CODE_RESET_USER_DATA` — factory-reset the user table. Reply:
    /// `[id, result]`.
    ResetUserData = 2,
    /// `CODE_READ_DATA` — read one memory-table entry. Reply:
    /// `[id, address, content..]`.
    ReadData = 3,
    /// `CODE_WRITE_DATA` — write one memory-table entry. Reply:
    /// `[id, address, result]`.
    WriteData = 4,
    /// `CODE_QUERY_SERVO_INFO` — marked "unused, no return value" in the SDK.
    QueryServoInfo = 5,
    /// `CODE_SET_SPIN` — wheel (continuous rotation) mode.
    SetSpin = 7,
    /// `CODE_SET_SERVO_ANGLE` — single-turn angle with a move time.
    SetAngle = 8,
    /// `CODE_SET_DAMPING` — damping mode with a holding power.
    SetDamping = 9,
    /// `CODE_QUERY_SERVO_ANGLE` — single-turn angle. Reply: `[id, i16]`.
    QueryAngle = 10,
    /// `CODE_SET_SERVO_ANGLE_BY_INTERVAL` — single-turn, move time + accel/decel.
    SetAngleByInterval = 11,
    /// `CODE_SET_SERVO_ANGLE_BY_VELOCITY` — single-turn, target speed + accel/decel.
    SetAngleByVelocity = 12,
    /// `CODE_SET_SERVO_ANGLE_MTURN` — multi-turn angle with a move time.
    SetAngleMturn = 13,
    /// `CODE_SET_SERVO_ANGLE_MTURN_BY_INTERVAL` — what the follower arm is
    /// driven with in the vendor teleop loop.
    SetAngleMturnByInterval = 14,
    /// `CODE_SET_SERVO_ANGLE_MTURN_BY_VELOCITY`.
    SetAngleMturnByVelocity = 15,
    /// `CODE_QUERY_SERVO_ANGLE_MTURN`. Reply: `[id, i32 angle, i16 turns]`.
    QueryAngleMturn = 16,
    /// `CODE_RESET_MULTI_TURN_ANGLE` — clear the turn counter.
    ResetMultiTurn = 17,
    /// `CODE_BEGIN_ASYNC` — start buffering commands.
    BeginAsync = 18,
    /// `CODE_END_ASYNC` — execute (or cancel) the buffered commands.
    EndAsync = 19,
    /// `CODE_QUERY_SERVO_MONITOR` — voltage/current/power/temp/status/angle.
    QueryMonitor = 22,
    /// `CODE_SET_ORIGIN_POINT` — **writes the zero into the servo's
    /// non-volatile memory.**
    SetOriginPoint = 23,
    /// `CODE_SET_STOP_ON_CONTROL` — stop and release / hold / damp.
    StopOnControl = 24,
    /// `CODE_SYNC_COMMAND` — one frame carrying the same command for N servos.
    SyncCommand = 25,
}

impl Code {
    /// Wire value.
    #[inline]
    pub const fn code(self) -> u8 {
        self as u8
    }

    /// Reverse lookup from a wire value.
    pub const fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            1 => Self::Ping,
            2 => Self::ResetUserData,
            3 => Self::ReadData,
            4 => Self::WriteData,
            5 => Self::QueryServoInfo,
            7 => Self::SetSpin,
            8 => Self::SetAngle,
            9 => Self::SetDamping,
            10 => Self::QueryAngle,
            11 => Self::SetAngleByInterval,
            12 => Self::SetAngleByVelocity,
            13 => Self::SetAngleMturn,
            14 => Self::SetAngleMturnByInterval,
            15 => Self::SetAngleMturnByVelocity,
            16 => Self::QueryAngleMturn,
            17 => Self::ResetMultiTurn,
            18 => Self::BeginAsync,
            19 => Self::EndAsync,
            22 => Self::QueryMonitor,
            23 => Self::SetOriginPoint,
            24 => Self::StopOnControl,
            25 => Self::SyncCommand,
            _ => return None,
        })
    }

    /// Whether the servo answers this command.
    ///
    /// Taken from `UartServoManager.response_handle_funcs`: those are the
    /// only codes the SDK can process on the way back, and it never waits
    /// after sending anything else.
    pub const fn has_reply(self) -> bool {
        matches!(
            self,
            Self::Ping
                | Self::ResetUserData
                | Self::ReadData
                | Self::WriteData
                | Self::QueryAngle
                | Self::QueryAngleMturn
                | Self::QueryMonitor
        )
    }
}

/// Mode byte of [`Code::StopOnControl`] (SDK `StopOptions.mode`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum StopMode {
    /// `0x10` "unlocked": stop and release torque. The leader arm is used
    /// like this so a human can move it freely.
    Release = 0x10,
    /// `0x11` "locked": stop and hold the current position.
    Hold = 0x11,
    /// `0x12` "damping": stop and resist motion proportionally to speed.
    Damping = 0x12,
}

impl StopMode {
    /// Wire value.
    #[inline]
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// Wheel-mode control method (`WHEEL_MODE_*`). The direction bit (`0x80` =
/// clockwise) is OR-ed in by the encoder.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum WheelMode {
    Stop = 0x00,
    /// Spin until told otherwise.
    Normal = 0x01,
    /// Spin a given number of turns.
    Turns = 0x02,
    /// Spin for a given time (ms).
    Timed = 0x03,
}

/// Memory-table addresses the SDK names (`ADDRESS_*`). Each holds a
/// little-endian `u16`, except [`DataAddress::STATUS`] which is one byte.
///
/// The same values are available more cheaply through
/// [`Code::QueryMonitor`], which returns all of them in one reply.
pub struct DataAddress;

impl DataAddress {
    /// Bus voltage, mV.
    pub const VOLTAGE: u8 = 1;
    /// Current, mA.
    pub const CURRENT: u8 = 2;
    /// Power, mW.
    pub const POWER: u8 = 3;
    /// Raw NTC ADC count (see the driver's temperature conversion).
    pub const TEMPERATURE: u8 = 4;
    /// Status byte, see [`status_bits`].
    pub const STATUS: u8 = 5;
}

/// Bits of the status byte, transcribed from the comment above
/// `UartServoManager.query_status` in `uservo.py`.
pub mod status_bits {
    /// Set while a command is executing, cleared when done.
    pub const EXECUTING: u8 = 1 << 0;
    /// Last command failed; cleared by the next successful one.
    pub const COMMAND_ERROR: u8 = 1 << 1;
    /// Stall detected; cleared once the stall is released.
    pub const STALL: u8 = 1 << 2;
    pub const OVER_VOLTAGE: u8 = 1 << 3;
    pub const UNDER_VOLTAGE: u8 = 1 << 4;
    pub const CURRENT_ERROR: u8 = 1 << 5;
    pub const POWER_ERROR: u8 = 1 << 6;
    pub const TEMPERATURE_ERROR: u8 = 1 << 7;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn code_round_trip() {
        for v in 0..=u8::MAX {
            if let Some(c) = Code::from_u8(v) {
                assert_eq!(c.code(), v);
            }
        }
        assert_eq!(Code::from_u8(6), None);
        assert_eq!(Code::from_u8(26), None);
    }

    #[test]
    fn only_queries_reply() {
        assert!(Code::QueryMonitor.has_reply());
        assert!(Code::Ping.has_reply());
        assert!(!Code::SetOriginPoint.has_reply());
        assert!(!Code::StopOnControl.has_reply());
        assert!(!Code::SyncCommand.has_reply());
    }
}
