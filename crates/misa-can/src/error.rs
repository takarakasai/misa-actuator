//! Transport-level error type shared by every CAN backend.

use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    /// No frame arrived within the configured receive timeout. Drivers map
    /// this onto their own `Timeout` variant, so it must stay distinguishable
    /// from a real I/O failure.
    #[error("timeout waiting for a CAN frame")]
    Timeout,

    /// Underlying OS / driver I/O failure.
    #[error("CAN I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// The interface spec could not be parsed, or names a backend that is not
    /// compiled in on this platform.
    #[error("invalid CAN interface {spec:?}: {reason}")]
    BadSpec { spec: String, reason: String },

    /// The interface could not be brought up: the device is missing, the
    /// driver is absent, or the adapter rejected the requested bitrate / FD
    /// mode. The message is a complete sentence naming the device and what to
    /// check, so it is printed as-is rather than behind a generic prefix.
    #[error("{0}")]
    Config(String),

    /// The backend cannot do what was asked (e.g. CAN-FD over SLCAN).
    #[error("unsupported by this CAN backend: {0}")]
    Unsupported(&'static str),

    /// A vendor API call failed. `code` is the raw driver status.
    #[error("{api} failed: {message} (code 0x{code:08X})")]
    Driver {
        api: &'static str,
        message: String,
        code: u32,
    },

    /// A frame could not be built or decoded.
    #[error("invalid CAN frame: {0}")]
    Frame(&'static str),
}

pub type Result<T> = std::result::Result<T, Error>;

impl Error {
    /// `true` if this is the benign "nothing arrived in time" case.
    pub fn is_timeout(&self) -> bool {
        matches!(self, Error::Timeout)
    }

    pub(crate) fn bad_spec(spec: &str, reason: impl Into<String>) -> Self {
        Error::BadSpec {
            spec: spec.to_string(),
            reason: reason.into(),
        }
    }
}
