//! Error types for the LK Motor driver.

use lkmotor_protocol::frame::{DecodeError, EncodeError};
use lkmotor_protocol::response::ParseError;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("serial port: {0}")]
    SerialPort(#[from] serialport::Error),

    #[error("io: {0}")]
    Io(#[from] std::io::Error),

    #[error("encode: {0:?}")]
    Encode(EncodeError),

    #[error("decode: {0:?}")]
    Decode(DecodeError),

    #[error("parse: {0:?}")]
    Parse(ParseError),

    #[error("timeout waiting for response from motor {motor_id}")]
    Timeout { motor_id: u8 },

    #[error(
        "unexpected response: command=0x{command:02X} motor_id={motor_id} (expected motor {expected_motor_id})"
    )]
    UnexpectedResponse {
        command: u8,
        motor_id: u8,
        expected_motor_id: u8,
    },

    #[error("invalid motor id {0} (must be 1..=32)")]
    InvalidMotorId(u8),

    #[error(
        "Motor::set_position called before rezero() — absolute zero anchor not set for motor {motor_id}"
    )]
    PositionNotAnchored { motor_id: u8 },
}

impl From<EncodeError> for Error {
    fn from(e: EncodeError) -> Self {
        Error::Encode(e)
    }
}

impl From<DecodeError> for Error {
    fn from(e: DecodeError) -> Self {
        Error::Decode(e)
    }
}

impl From<ParseError> for Error {
    fn from(e: ParseError) -> Self {
        Error::Parse(e)
    }
}

pub type Result<T> = std::result::Result<T, Error>;

impl From<Error> for misa_actuator::Error {
    fn from(e: Error) -> Self {
        use misa_actuator::Error as M;
        match e {
            Error::SerialPort(e) => M::Bus(e.to_string()),
            Error::Io(e) => M::Bus(e.to_string()),
            Error::Encode(e) => M::Protocol(format!("{:?}", e)),
            Error::Decode(e) => M::Protocol(format!("{:?}", e)),
            Error::Parse(e) => M::Protocol(format!("{:?}", e)),
            Error::Timeout { motor_id } => M::Timeout { motor_id },
            Error::UnexpectedResponse {
                command,
                motor_id,
                expected_motor_id,
            } => M::Protocol(format!(
                "unexpected response cmd=0x{:02X} from motor {} (expected {})",
                command, motor_id, expected_motor_id
            )),
            Error::InvalidMotorId(id) => M::InvalidMotorId(id),
            Error::PositionNotAnchored { motor_id } => M::PositionNotAnchored { motor_id },
        }
    }
}
