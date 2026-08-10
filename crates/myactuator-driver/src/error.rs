//! Driver error types for the MyActuator RMD family.

use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    /// Transparent so the transport's own message is not repeated once here
    /// and again as the `Caused by:` line — see robstride-driver's Error.
    #[error(transparent)]
    Bus(#[from] misa_can::Error),

    #[error("timeout waiting for response from motor {motor_id}")]
    Timeout { motor_id: u8 },

    #[error("invalid response: {0}")]
    InvalidResponse(String),

    #[error("invalid motor id {0} (must be 1..=32)")]
    InvalidMotorId(u8),

    #[error("invalid CAN frame: {0}")]
    InvalidFrame(&'static str),

    #[error("motor {motor_id}: position not anchored — call set_zero() first")]
    PositionNotAnchored { motor_id: u8 },
}

pub type Result<T> = std::result::Result<T, Error>;

impl From<Error> for misa_actuator::Error {
    fn from(e: Error) -> Self {
        use misa_actuator::Error as M;
        match e {
            Error::Bus(e) => M::Bus(e.to_string()),
            Error::Timeout { motor_id } => M::Timeout { motor_id },
            Error::InvalidResponse(msg) => M::Protocol(msg),
            Error::InvalidMotorId(id) => M::InvalidMotorId(id),
            Error::InvalidFrame(msg) => M::Protocol(msg.to_string()),
            Error::PositionNotAnchored { motor_id } => M::PositionNotAnchored { motor_id },
        }
    }
}
