//! Bus abstraction for the MyActuator RMD CAN protocol.
//!
//! V3 traffic is carried as **standard 11-bit** CAN frames with an 8-byte
//! payload at 1 Mbps. The [`MyActuatorBus`] trait captures the wire I/O so the
//! driver can also run over mocks (tests) or future transports (the protocol
//! also exists on RS485). [`CanBus`] adapts any [`misa_can`] transport —
//! Linux SocketCAN, a PEAK adapter on Windows, or an SLCAN dongle — to that
//! trait.

use std::time::Duration;

use misa_actuator::Shared;
use misa_can::{OpenOptions, StandardId};

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
/// Note: [`MyActuatorBus::set_timeout`] here affects the *shared* transport,
/// so the last value set wins across all motors on the bus.
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

/// Adapter presenting any [`misa_can::CanBus`] as a [`MyActuatorBus`].
pub struct CanBus<T: misa_can::CanBus> {
    inner: T,
}

/// The bus [`crate::MyActuatorMotor::open`] produces.
pub type AnyCanBus = CanBus<Box<dyn misa_can::CanBus>>;

impl AnyCanBus {
    /// Open whichever CAN transport `interface` names.
    ///
    /// | interface | transport |
    /// |-----------|-----------|
    /// | `can0` | Linux SocketCAN |
    /// | `pcan:usb1` | PEAK PCAN-Basic (Windows) |
    /// | `slcan:COM5` / `slcan:/dev/ttyACM0` | USB-CAN adapter |
    ///
    /// RMD V3 motors ship at 1 Mbit/s, which is the default.
    pub fn open(interface: &str) -> Result<Self> {
        Self::open_with(interface, &OpenOptions::classic())
    }

    /// Open with explicit transport options (timeout, CAN-FD).
    pub fn open_with(interface: &str, opts: &OpenOptions) -> Result<Self> {
        Ok(Self {
            inner: misa_can::open(interface, opts).map_err(map_err)?,
        })
    }
}

impl<T: misa_can::CanBus> CanBus<T> {
    /// Wrap an already-open transport.
    pub fn new(inner: T) -> Self {
        Self { inner }
    }

    /// Borrow the underlying transport.
    pub fn inner(&self) -> &T {
        &self.inner
    }

    /// What is actually open, e.g. `PCAN_USBBUS1 (CAN, 1000000 bit/s)`.
    pub fn description(&self) -> String {
        self.inner.description()
    }
}

impl<T: misa_can::CanBus> MyActuatorBus for CanBus<T> {
    fn send(&mut self, can_id: u16, data: &[u8]) -> Result<()> {
        let id = StandardId::new(can_id).ok_or(Error::InvalidFrame("CAN ID exceeds 11 bits"))?;
        let frame = misa_can::Frame::new(id, data).map_err(map_err)?;
        self.inner.send(&frame).map_err(map_err)
    }

    fn recv(&mut self) -> Result<CanFrame> {
        loop {
            let frame = self.inner.recv().map_err(map_err)?;
            // MyActuator only uses 11-bit ids; an extended frame belongs to
            // another family sharing the wire (e.g. RobStride).
            let Some(can_id) = frame.standard_id() else {
                continue;
            };
            return Ok(CanFrame {
                can_id,
                data: frame.data().to_vec(),
            });
        }
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.inner.set_timeout(timeout).map_err(map_err)
    }
}

/// Map a transport error onto the driver's own error type, keeping the
/// timeout case distinguishable — the driver's retry loops match on it.
fn map_err(e: misa_can::Error) -> Error {
    match e {
        misa_can::Error::Timeout => Error::Timeout { motor_id: 0 },
        other => Error::Bus(other),
    }
}
