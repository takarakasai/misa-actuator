//! Newtype wrapper around the 1..=32 motor address space.

use crate::error::Error;

/// A validated motor identifier (1..=32).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MotorId(u8);

impl MotorId {
    /// Maximum legal motor ID on the bus (firmware limit).
    pub const MAX: u8 = 32;

    /// Construct a `MotorId`, returning `None` if `id` is outside `1..=32`.
    pub const fn new(id: u8) -> Option<Self> {
        if id == 0 || id > Self::MAX {
            None
        } else {
            Some(MotorId(id))
        }
    }

    /// Construct a `MotorId`, returning [`Error::InvalidMotorId`] on failure.
    pub fn try_from_u8(id: u8) -> Result<Self, Error> {
        Self::new(id).ok_or(Error::InvalidMotorId(id))
    }

    /// Wire byte representation.
    #[inline]
    pub const fn get(self) -> u8 {
        self.0
    }
}

impl From<MotorId> for u8 {
    #[inline]
    fn from(id: MotorId) -> u8 {
        id.0
    }
}
