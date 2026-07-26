//! Bus abstraction for the MyActuator RMD CAN protocol.
//!
//! V3 traffic is carried as **standard 11-bit** CAN frames with an 8-byte
//! payload at 1 Mbps. The [`MyActuatorBus`] trait captures the wire I/O so the
//! driver can also run over mocks (tests) or future transports (the protocol
//! also exists on RS485); [`SocketCanBus`] is the Linux SocketCAN
//! implementation.

use std::io;
use std::time::Duration;

use socketcan::{CanSocket, EmbeddedFrame, Id, Socket, StandardId};

use misa_actuator::Shared;

use crate::error::{Error, Result};

/// One CAN frame as seen by the bus layer.
#[derive(Debug, Clone)]
pub struct CanFrame {
    /// Standard 11-bit CAN id (0..=0x7FF).
    pub can_id: u16,
    /// Payload (8 bytes for MyActuator).
    pub data: Vec<u8>,
}

/// Application-level CAN transport for the MyActuator RMD protocol.
pub trait MyActuatorBus {
    /// Transmit one frame to standard id `can_id`.
    fn send(&mut self, can_id: u16, data: &[u8]) -> Result<()>;

    /// Receive the next frame, or [`Error::Timeout`] (with `motor_id = 0`) if
    /// none arrives within the configured timeout.
    fn recv(&mut self) -> Result<CanFrame>;

    /// Set the per-receive timeout.
    fn set_timeout(&mut self, timeout: Duration) -> Result<()>;
}

/// Share one bus across several [`crate::MyActuatorMotor`] handles on the same
/// wire: wrap the opened bus in [`misa_actuator::Shared`] and hand each motor a
/// clone. Access is serialized by the mutex — drive the motors from a single
/// control loop per bus.
///
/// Note: [`MyActuatorBus::set_timeout`] here affects the *shared* socket, so
/// the last value set wins across all motors on the bus.
impl<B: MyActuatorBus> MyActuatorBus for Shared<B> {
    fn send(&mut self, can_id: u16, data: &[u8]) -> Result<()> {
        self.lock().send(can_id, data)
    }

    fn recv(&mut self) -> Result<CanFrame> {
        self.lock().recv()
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.lock().set_timeout(timeout)
    }
}

/// Build a standard-id from a `u16`, rejecting ids above 0x7FF.
fn std_id(can_id: u16) -> Result<StandardId> {
    StandardId::new(can_id).ok_or(Error::InvalidFrame("CAN ID exceeds 11 bits"))
}

/// SocketCAN-backed classic-CAN implementation (Linux only).
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

impl MyActuatorBus for SocketCanBus {
    fn send(&mut self, can_id: u16, data: &[u8]) -> Result<()> {
        let frame = socketcan::CanFrame::new(Id::Standard(std_id(can_id)?), data)
            .ok_or(Error::InvalidFrame("payload too long for classic CAN frame"))?;
        self.socket.write_frame(&frame)?;
        Ok(())
    }

    fn recv(&mut self) -> Result<CanFrame> {
        loop {
            match self.socket.read_frame() {
                Ok(frame) => match frame.id() {
                    Id::Standard(s) => {
                        return Ok(CanFrame {
                            can_id: s.as_raw(),
                            data: frame.data().to_vec(),
                        });
                    }
                    // Extended frame — not MyActuator, keep waiting.
                    Id::Extended(_) => {}
                },
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
