//! Driver error types for the DAMIAO family.

use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("CAN socket error: {0}")]
    CanSocket(#[from] std::io::Error),

    #[error("timeout waiting for response from motor {motor_id}")]
    Timeout { motor_id: u8 },

    #[error("invalid response: {0}")]
    InvalidResponse(String),

    #[error("motor {motor_id} reported a fault (code 0x{code:X})")]
    MotorFault { motor_id: u8, code: u8 },

    #[error("motor {motor_id} is not enabled — call enable() before sending control commands")]
    NotEnabled { motor_id: u8 },

    #[error("mode switch to {requested} not confirmed (motor {motor_id} still reports {actual})")]
    ModeSwitchFailed {
        motor_id: u8,
        requested: i32,
        actual: i32,
    },

    #[error("invalid CAN frame: {0}")]
    InvalidFrame(&'static str),

    #[error("motor {motor_id} ({model}) does not support operation: {op}")]
    Unsupported {
        motor_id: u8,
        model: &'static str,
        op: &'static str,
    },
}

pub type Result<T> = std::result::Result<T, Error>;

impl From<Error> for misa_actuator::Error {
    fn from(e: Error) -> Self {
        use misa_actuator::Error as M;
        match e {
            Error::CanSocket(e) => M::Bus(e.to_string()),
            Error::Timeout { motor_id } => M::Timeout { motor_id },
            Error::InvalidResponse(msg) => M::Protocol(msg),
            Error::MotorFault { motor_id, code } => M::MotorFault {
                motor_id,
                code: code as u32,
            },
            Error::NotEnabled { motor_id } => M::NotEnabled { motor_id },
            Error::ModeSwitchFailed {
                motor_id,
                requested,
                actual,
            } => M::Other(format!(
                "motor {motor_id}: mode switch to {requested} failed (still {actual})"
            )),
            Error::InvalidFrame(msg) => M::Protocol(msg.to_string()),
            Error::Unsupported { op, .. } => M::Unsupported(op),
        }
    }
}
