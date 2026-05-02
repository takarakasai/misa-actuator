//! Unified error type shared by all actuator drivers.
//!
//! Driver-specific errors (RS485 framing, CAN socket, parameter parsing,
//! etc.) are funneled into the variants below. Each driver is expected to
//! provide `From<DriverError> for misa_actuator::Error` so its native error
//! type can be propagated through `?` from `Actuator` impls.

use thiserror::Error;

/// Unified actuator error.
#[derive(Debug, Error)]
pub enum Error {
    /// Underlying transport (serial, CAN socket, EtherCAT master, ...) failed.
    #[error("bus/transport: {0}")]
    Bus(String),

    /// Wire-protocol encode/decode/parse failure.
    #[error("protocol: {0}")]
    Protocol(String),

    /// No reply received from the motor within the configured deadline.
    #[error("timeout waiting for response from motor {motor_id}")]
    Timeout { motor_id: u8 },

    /// Motor is reporting a fault.
    #[error("motor {motor_id} reported a fault (code 0x{code:08X})")]
    MotorFault { motor_id: u8, code: u32 },

    /// A control command was issued before the motor was enabled.
    #[error("motor {motor_id} is not enabled — call enable() first")]
    NotEnabled { motor_id: u8 },

    /// `set_position` was called before establishing a position anchor.
    #[error("motor {motor_id}: position not anchored — call set_zero() first")]
    PositionNotAnchored { motor_id: u8 },

    /// Motor id outside the firmware-allowed range.
    #[error("invalid motor id {0}")]
    InvalidMotorId(u8),

    /// A driver does not support the requested operation (e.g. `set_zero()`
    /// on a closed-source firmware that lacks an absolute-zero command).
    #[error("operation not supported by this driver: {0}")]
    Unsupported(&'static str),

    /// Catch-all for driver-specific errors that don't map cleanly to one of
    /// the variants above. Keep this rare — prefer adding a typed variant.
    #[error("driver: {0}")]
    Other(String),
}

impl Error {
    /// Convenience constructor for [`Error::Bus`] from any displayable type.
    pub fn bus<E: core::fmt::Display>(e: E) -> Self {
        Self::Bus(e.to_string())
    }

    /// Convenience constructor for [`Error::Protocol`] from any displayable type.
    pub fn protocol<E: core::fmt::Display>(e: E) -> Self {
        Self::Protocol(e.to_string())
    }

    /// Convenience constructor for [`Error::Other`] from any displayable type.
    pub fn other<E: core::fmt::Display>(e: E) -> Self {
        Self::Other(e.to_string())
    }
}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Self::Bus(e.to_string())
    }
}

pub type Result<T> = std::result::Result<T, Error>;
