//! Driver error types.

use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("CAN socket error: {0}")]
    CanSocket(#[from] std::io::Error),

    #[error("timeout waiting for response from motor {motor_id}")]
    Timeout { motor_id: u8 },

    #[error("invalid response: {0}")]
    InvalidResponse(String),

    #[error("motor fault reported (motor {motor_id}, raw extra=0x{extra_data:04X})")]
    MotorFault { motor_id: u8, extra_data: u16 },

    #[error("motor {motor_id} is not enabled — call enable() before sending control commands")]
    NotEnabled { motor_id: u8 },

    #[error("invalid CAN frame: {0}")]
    InvalidFrame(&'static str),
}

pub type Result<T> = std::result::Result<T, Error>;

impl From<Error> for misa_actuator::Error {
    fn from(e: Error) -> Self {
        use misa_actuator::Error as M;
        match e {
            Error::CanSocket(e) => M::Bus(e.to_string()),
            Error::Timeout { motor_id } => M::Timeout { motor_id },
            Error::InvalidResponse(msg) => M::Protocol(msg),
            Error::MotorFault {
                motor_id,
                extra_data,
            } => M::MotorFault {
                motor_id,
                code: extra_data as u32,
            },
            Error::NotEnabled { motor_id } => M::NotEnabled { motor_id },
            Error::InvalidFrame(msg) => M::Protocol(msg.to_string()),
        }
    }
}
