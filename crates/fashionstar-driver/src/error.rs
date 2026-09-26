//! Error types for the FashionStar driver.

use fashionstar_protocol::{EncodeError, ParseError};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("serial port: {0}")]
    SerialPort(#[from] serialport::Error),

    #[error("io: {0}")]
    Io(#[from] std::io::Error),

    #[error("encode: {0:?}")]
    Encode(EncodeError),

    #[error("parse: {0:?}")]
    Parse(ParseError),

    #[error("timeout waiting for response from servo {servo_id}")]
    Timeout { servo_id: u8 },

    /// The servo answered with the monitor "invalid" sentinel (see
    /// [`fashionstar_protocol::MONITOR_INVALID_ANGLE`]), which the SDK also
    /// discards.
    #[error("servo {servo_id} returned an invalid monitor reading")]
    InvalidReading { servo_id: u8 },

    #[error("unexpected response from servo {servo_id}: {detail}")]
    UnexpectedResponse { servo_id: u8, detail: String },

    #[error("invalid argument: {0}")]
    InvalidArgument(&'static str),
}

impl From<EncodeError> for Error {
    fn from(e: EncodeError) -> Self {
        Error::Encode(e)
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
            Error::Encode(e) => M::Protocol(format!("{e:?}")),
            Error::Parse(e) => M::Protocol(format!("{e:?}")),
            Error::Timeout { servo_id } => M::Timeout { motor_id: servo_id },
            e @ Error::InvalidReading { .. } => M::Protocol(e.to_string()),
            e @ Error::UnexpectedResponse { .. } => M::Protocol(e.to_string()),
            e @ Error::InvalidArgument(_) => M::Other(e.to_string()),
        }
    }
}
