//! Bus abstraction for the Robstride CAN servo motor family.
//!
//! All Robstride traffic is carried as 29-bit extended CAN frames with an
//! 8-byte payload. The [`RobstrideBus`] trait abstracts over the underlying
//! CAN transport so that the same `Motor` driver works on Linux SocketCAN, on
//! a PEAK adapter under Windows, and on a USB-CAN (SLCAN) dongle on either.
//!
//! The concrete transports live in [`misa_can`]; [`CanBus`] is the thin
//! adapter that presents one of them as a `RobstrideBus`. [`AnyCanBus`] is
//! what [`crate::Motor::open`] builds — the backend is chosen by the
//! interface string (`can0`, `pcan:usb1`, `slcan:COM5`, ...).

use std::time::Duration;

use misa_actuator::Shared;
use misa_can::{ExtendedId, OpenOptions};

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

/// Share one Robstride bus across several [`crate::Motor`] handles on the same
/// wire: wrap an opened bus in [`misa_actuator::Shared`] and hand each motor a
/// clone. Access is serialized by the mutex — drive the motors from a single
/// control loop per bus.
impl<B: RobstrideBus> RobstrideBus for Shared<B> {
    fn send(&mut self, can_id: u32, data: &[u8]) -> Result<()> {
        self.lock().send(can_id, data)
    }

    fn recv(&mut self) -> Result<CanFrame> {
        self.lock().recv()
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.lock().set_timeout(timeout)
    }
}

/// Adapter presenting any [`misa_can::CanBus`] as a [`RobstrideBus`].
pub struct CanBus<T: misa_can::CanBus> {
    inner: T,
}

/// The bus [`crate::Motor::open`] produces: a transport picked at run time
/// from the interface string.
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
    /// Robstride motors ship at 1 Mbit/s, which is the default; append
    /// `@500K` to the interface to override it.
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

impl<T: misa_can::CanBus> RobstrideBus for CanBus<T> {
    fn send(&mut self, can_id: u32, data: &[u8]) -> Result<()> {
        let id = ExtendedId::new(can_id).ok_or(Error::InvalidFrame("CAN ID exceeds 29 bits"))?;
        let frame = misa_can::Frame::new(id, data).map_err(map_err)?;
        self.inner.send(&frame).map_err(map_err)
    }

    fn recv(&mut self) -> Result<CanFrame> {
        loop {
            let frame = self.inner.recv().map_err(map_err)?;
            // Robstride only ever uses 29-bit ids; a standard-id frame belongs
            // to some other family sharing the wire.
            let Some(can_id) = frame.extended_id() else {
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
