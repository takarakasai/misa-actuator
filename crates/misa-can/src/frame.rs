//! A backend-independent CAN / CAN-FD frame.
//!
//! The payload lives in a fixed 64-byte buffer so the receive path never
//! allocates — drivers copy out into whatever shape their own bus trait
//! promises.

pub use embedded_can::{ExtendedId, Id, StandardId};

use crate::error::{Error, Result};

/// Largest payload a CAN-FD frame can carry.
pub const MAX_DATA_LEN: usize = 64;

/// Payload lengths a CAN-FD frame is allowed to have. Anything in between is
/// padded up to the next entry.
const FD_DLC_STEPS: [usize; 16] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64];

/// One CAN frame on the wire.
#[derive(Clone, Copy)]
pub struct Frame {
    id: Id,
    len: u8,
    fd: bool,
    brs: bool,
    data: [u8; MAX_DATA_LEN],
}

impl Frame {
    /// Build a classic-CAN frame (payload ≤ 8 bytes).
    pub fn new(id: impl Into<Id>, data: &[u8]) -> Result<Self> {
        if data.len() > 8 {
            return Err(Error::Frame("payload too long for a classic CAN frame"));
        }
        Ok(Self::build(id.into(), data, false, false))
    }

    /// Build a CAN-FD frame. `brs` requests bit-rate switching, i.e. the data
    /// phase runs at the interface's data bitrate.
    pub fn new_fd(id: impl Into<Id>, data: &[u8], brs: bool) -> Result<Self> {
        if data.len() > MAX_DATA_LEN {
            return Err(Error::Frame("payload too long for a CAN-FD frame"));
        }
        Ok(Self::build(id.into(), data, true, brs))
    }

    fn build(id: Id, data: &[u8], fd: bool, brs: bool) -> Self {
        let mut buf = [0u8; MAX_DATA_LEN];
        buf[..data.len()].copy_from_slice(data);
        Self {
            id,
            len: data.len() as u8,
            fd,
            brs,
            data: buf,
        }
    }

    /// Build a frame from a raw id, choosing standard vs. extended by whether
    /// the id fits in 11 bits. Only for backends that hand back a raw id plus
    /// a separate "was extended" flag.
    pub fn from_raw(raw_id: u32, extended: bool, data: &[u8], fd: bool, brs: bool) -> Result<Self> {
        let id: Id = if extended {
            ExtendedId::new(raw_id)
                .ok_or(Error::Frame("CAN id exceeds 29 bits"))?
                .into()
        } else {
            StandardId::new(raw_id as u16)
                .ok_or(Error::Frame("CAN id exceeds 11 bits"))?
                .into()
        };
        if data.len() > MAX_DATA_LEN {
            return Err(Error::Frame("payload longer than 64 bytes"));
        }
        Ok(Self::build(id, data, fd, brs))
    }

    pub fn id(&self) -> Id {
        self.id
    }

    /// The id as a plain integer, whatever its width.
    pub fn raw_id(&self) -> u32 {
        match self.id {
            Id::Standard(s) => s.as_raw() as u32,
            Id::Extended(e) => e.as_raw(),
        }
    }

    /// `Some(id)` only for 11-bit frames — the DAMIAO and MyActuator drivers
    /// use this to drop foreign extended traffic sharing the wire.
    pub fn standard_id(&self) -> Option<u16> {
        match self.id {
            Id::Standard(s) => Some(s.as_raw()),
            Id::Extended(_) => None,
        }
    }

    /// `Some(id)` only for 29-bit frames — the Robstride driver's filter.
    pub fn extended_id(&self) -> Option<u32> {
        match self.id {
            Id::Standard(_) => None,
            Id::Extended(e) => Some(e.as_raw()),
        }
    }

    pub fn is_extended(&self) -> bool {
        matches!(self.id, Id::Extended(_))
    }

    pub fn is_fd(&self) -> bool {
        self.fd
    }

    pub fn is_brs(&self) -> bool {
        self.brs
    }

    pub fn data(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    pub fn len(&self) -> usize {
        self.len as usize
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Payload padded up to the next legal CAN-FD length, for backends that
    /// must hand the driver a whole DLC step. Returns the classic payload
    /// unchanged for non-FD frames.
    pub fn fd_padded(&self) -> ([u8; MAX_DATA_LEN], usize) {
        let want = self.len as usize;
        let padded = if self.fd {
            FD_DLC_STEPS
                .iter()
                .copied()
                .find(|&step| step >= want)
                .unwrap_or(MAX_DATA_LEN)
        } else {
            want
        };
        (self.data, padded)
    }
}

impl core::fmt::Debug for Frame {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let kind = match (self.fd, self.brs) {
            (false, _) => "CAN",
            (true, false) => "FD",
            (true, true) => "FD/BRS",
        };
        write!(
            f,
            "{kind} {:#X} [{}] {:02X?}",
            self.raw_id(),
            self.len,
            self.data()
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classic_rejects_long_payloads() {
        assert!(Frame::new(StandardId::new(0x141).unwrap(), &[0u8; 9]).is_err());
        assert!(Frame::new(StandardId::new(0x141).unwrap(), &[0u8; 8]).is_ok());
    }

    #[test]
    fn raw_ids_round_trip() {
        let f = Frame::from_raw(0x1234_5678 & 0x1FFF_FFFF, true, &[1, 2, 3], false, false).unwrap();
        assert!(f.is_extended());
        assert_eq!(f.extended_id(), Some(0x1234_5678 & 0x1FFF_FFFF));
        assert_eq!(f.standard_id(), None);
        assert_eq!(f.data(), &[1, 2, 3]);
    }

    #[test]
    fn fd_padding_snaps_to_legal_dlc() {
        let f = Frame::new_fd(StandardId::new(0x01).unwrap(), &[0xAA; 9], true).unwrap();
        let (_, padded) = f.fd_padded();
        assert_eq!(padded, 12);
        // A classic frame is never padded.
        let c = Frame::new(StandardId::new(0x01).unwrap(), &[0xAA; 5]).unwrap();
        assert_eq!(c.fd_padded().1, 5);
    }
}
