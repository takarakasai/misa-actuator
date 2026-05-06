//! Bus abstraction for the DAMIAO CAN / CAN-FD servo motor family.
//!
//! DAMIAO traffic is carried as **standard 11-bit** CAN frames with an 8-byte
//! payload. Crucially, the application protocol is *identical* on classic CAN
//! and CAN-FD — same ids, same 8-byte payloads — so the only difference is the
//! physical layer. The [`DamiaoBus`] trait captures the wire I/O, and we
//! provide two concrete implementations:
//!
//! - [`SocketCanBus`] — classic CAN (1 Mbps) via `socketcan::CanSocket`.
//! - [`SocketCanFdBus`] — CAN-FD (1–5 Mbps) via `socketcan::CanFdSocket`,
//!   sending BRS-enabled FD frames.
//!
//! A single robot may run a classic-CAN bus and a CAN-FD bus side by side
//! (e.g. mixing motor lots that only speak one or the other); pick the matching
//! bus per interface. They are never mixed on the *same* wire.

use std::io;
use std::time::Duration;

use socketcan::{
    CanAnyFrame, CanFdFrame, CanFdSocket, CanSocket, EmbeddedFrame, Id, Socket, StandardId,
};

use crate::error::{Error, Result};

/// One CAN frame as seen by the bus layer.
#[derive(Debug, Clone)]
pub struct CanFrame {
    /// Standard 11-bit CAN id (0..=0x7FF).
    pub can_id: u16,
    /// Payload (0..=8 bytes for DAMIAO).
    pub data: Vec<u8>,
}

/// Application-level CAN transport for the DAMIAO protocol.
///
/// Implementations transmit a single standard-CAN frame on [`Self::send`] and
/// return the next received frame on [`Self::recv`]. The driver layer
/// ([`crate::driver::DamiaoMotor`]) builds the id-filter loop on top.
pub trait DamiaoBus {
    /// Transmit one frame to standard id `can_id`.
    fn send(&mut self, can_id: u16, data: &[u8]) -> Result<()>;

    /// Receive the next frame, or [`Error::Timeout`] (with `motor_id = 0`) if
    /// none arrives within the configured timeout.
    fn recv(&mut self) -> Result<CanFrame>;

    /// Set the per-receive timeout.
    fn set_timeout(&mut self, timeout: Duration) -> Result<()>;
}

/// Build a standard-id from a `u16`, rejecting ids above 0x7FF.
fn std_id(can_id: u16) -> Result<StandardId> {
    StandardId::new(can_id).ok_or(Error::InvalidFrame("CAN ID exceeds 11 bits"))
}

/// Extract the standard id and payload from a received frame, or `None` for
/// extended / non-data frames we don't care about.
fn decode_std(id: Id, data: &[u8]) -> Option<CanFrame> {
    match id {
        Id::Standard(s) => Some(CanFrame {
            can_id: s.as_raw(),
            data: data.to_vec(),
        }),
        Id::Extended(_) => None,
    }
}

/// SocketCAN-backed **classic CAN** implementation (Linux only).
pub struct SocketCanBus {
    socket: CanSocket,
    timeout: Duration,
}

impl SocketCanBus {
    /// Open a classic-CAN SocketCAN interface (e.g. `"can0"`).
    pub fn open(interface: &str) -> Result<Self> {
        let socket = CanSocket::open(interface)?;
        let timeout = Duration::from_millis(100);
        socket.set_read_timeout(timeout)?;
        Ok(Self { socket, timeout })
    }

    /// Borrow the underlying SocketCAN handle.
    pub fn socket(&self) -> &CanSocket {
        &self.socket
    }
}

impl DamiaoBus for SocketCanBus {
    fn send(&mut self, can_id: u16, data: &[u8]) -> Result<()> {
        let frame = socketcan::CanFrame::new(Id::Standard(std_id(can_id)?), data)
            .ok_or(Error::InvalidFrame("payload too long for classic CAN frame"))?;
        self.socket.write_frame(&frame)?;
        Ok(())
    }

    fn recv(&mut self) -> Result<CanFrame> {
        loop {
            match self.socket.read_frame() {
                Ok(frame) => {
                    if let Some(f) = decode_std(frame.id(), frame.data()) {
                        return Ok(f);
                    }
                    // extended frame — not DAMIAO, keep waiting
                }
                Err(ref e)
                    if e.kind() == io::ErrorKind::WouldBlock
                        || e.kind() == io::ErrorKind::TimedOut =>
                {
                    return Err(Error::Timeout { motor_id: 0 });
                }
                Err(e) => return Err(Error::CanSocket(e)),
            }
        }
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.socket.set_read_timeout(timeout)?;
        self.timeout = timeout;
        Ok(())
    }
}

/// SocketCAN-backed **CAN-FD** implementation (Linux only).
///
/// Sends BRS-enabled FD frames so the data phase runs at the bus's configured
/// data bitrate (e.g. 5 Mbps). The DAMIAO payload is still 8 bytes. A
/// `CanFdSocket` also receives classic frames, so feedback is handled
/// uniformly regardless of how the motor framed its reply.
///
/// The interface must be brought up in FD mode, e.g.:
/// ```text
/// sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on up
/// ```
pub struct SocketCanFdBus {
    socket: CanFdSocket,
    timeout: Duration,
}

impl SocketCanFdBus {
    /// Open a CAN-FD SocketCAN interface (e.g. `"can0"`). The interface must
    /// already be configured with `fd on`.
    pub fn open(interface: &str) -> Result<Self> {
        let socket = CanFdSocket::open(interface)?;
        let timeout = Duration::from_millis(100);
        socket.set_read_timeout(timeout)?;
        Ok(Self { socket, timeout })
    }

    /// Borrow the underlying CAN-FD socket handle.
    pub fn socket(&self) -> &CanFdSocket {
        &self.socket
    }
}

impl DamiaoBus for SocketCanFdBus {
    fn send(&mut self, can_id: u16, data: &[u8]) -> Result<()> {
        let mut frame = CanFdFrame::new(Id::Standard(std_id(can_id)?), data)
            .ok_or(Error::InvalidFrame("payload too long for CAN-FD frame"))?;
        // Bit-rate switching: run the data phase at the fast data bitrate.
        frame.set_brs(true);
        self.socket.write_frame(&frame)?;
        Ok(())
    }

    fn recv(&mut self) -> Result<CanFrame> {
        loop {
            match self.socket.read_frame() {
                Ok(any) => {
                    let decoded = match any {
                        CanAnyFrame::Normal(f) => decode_std(f.id(), f.data()),
                        CanAnyFrame::Fd(f) => decode_std(f.id(), f.data()),
                        // Remote / error frames are not DAMIAO data.
                        CanAnyFrame::Remote(_) | CanAnyFrame::Error(_) => None,
                    };
                    if let Some(f) = decoded {
                        return Ok(f);
                    }
                }
                Err(ref e)
                    if e.kind() == io::ErrorKind::WouldBlock
                        || e.kind() == io::ErrorKind::TimedOut =>
                {
                    return Err(Error::Timeout { motor_id: 0 });
                }
                Err(e) => return Err(Error::CanSocket(e)),
            }
        }
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.socket.set_read_timeout(timeout)?;
        self.timeout = timeout;
        Ok(())
    }
}
