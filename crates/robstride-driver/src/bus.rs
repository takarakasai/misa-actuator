//! Bus abstraction for the Robstride CAN servo motor family.
//!
//! All Robstride traffic is carried as 29-bit extended CAN frames with an
//! 8-byte payload. The [`RobstrideBus`] trait abstracts over the underlying
//! CAN transport so that the same `Motor` driver works on Linux SocketCAN
//! today and on USB-CAN serial adapters / EtherCAT-based gateways
//! tomorrow.

use std::io;
use std::time::Duration;

use socketcan::{CanSocket, EmbeddedFrame, ExtendedId, Id, Socket, StandardId};

use crate::error::{Error, Result};

/// One CAN frame as seen by the bus layer.
#[derive(Debug, Clone)]
pub struct CanFrame {
    /// Full 29-bit extended CAN id.
    pub can_id: u32,
    /// Payload (0..=8 bytes).
    pub data: Vec<u8>,
}

/// Application-level CAN transport for the Robstride protocol.
///
/// Implementations send a single extended-CAN frame on `send` and return
/// the next frame received on `recv`. The driver layer
/// ([`crate::driver::Motor`]) builds the frame-filter loop on top — bus
/// implementations only need to provide the wire I/O primitives.
pub trait RobstrideBus {
    /// Transmit one frame.
    fn send(&mut self, can_id: u32, data: &[u8]) -> Result<()>;

    /// Receive the next frame. Should return [`Error::Timeout`] (with
    /// `motor_id = 0`) if no frame arrives within the configured timeout.
    fn recv(&mut self) -> Result<CanFrame>;

    /// Set the per-receive timeout. Subsequent [`Self::recv`] calls return
    /// `Error::Timeout` after this duration of silence.
    fn set_timeout(&mut self, timeout: Duration) -> Result<()>;
}

/// SocketCAN-backed implementation of [`RobstrideBus`] (Linux only).
pub struct SocketCanBus {
    socket: CanSocket,
    timeout: Duration,
}

impl SocketCanBus {
    /// Open a SocketCAN interface (e.g. `"can0"`).
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

impl RobstrideBus for SocketCanBus {
    fn send(&mut self, can_id: u32, data: &[u8]) -> Result<()> {
        let ext_id = ExtendedId::new(can_id)
            .ok_or(Error::InvalidFrame("CAN ID exceeds 29 bits"))?;
        let frame = socketcan::CanFrame::new(Id::Extended(ext_id), data)
            .ok_or(Error::InvalidFrame("payload too long for CAN frame"))?;
        self.socket.write_frame(&frame)?;
        Ok(())
    }

    fn recv(&mut self) -> Result<CanFrame> {
        loop {
            match self.socket.read_frame() {
                Ok(frame) => {
                    if !frame.is_extended() {
                        continue;
                    }
                    let raw_id = match frame.id() {
                        Id::Standard(s) => StandardId::as_raw(&s) as u32,
                        Id::Extended(e) => ExtendedId::as_raw(&e),
                    };
                    return Ok(CanFrame {
                        can_id: raw_id,
                        data: frame.data().to_vec(),
                    });
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock
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
