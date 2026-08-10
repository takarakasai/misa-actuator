//! Bus abstraction for the DAMIAO CAN / CAN-FD servo motor family.
//!
//! DAMIAO traffic is carried as **standard 11-bit** CAN frames with an 8-byte
//! payload. Crucially, the application protocol is *identical* on classic CAN
//! and CAN-FD — same ids, same 8-byte payloads — so the only difference is the
//! physical layer.
//!
//! [`DamiaoBus`] captures the wire I/O and [`CanBus`] adapts any
//! [`misa_can`] transport to it. Which transport that is comes from the
//! interface string: `can0` (Linux SocketCAN), `pcan:usb1` (PEAK, Windows) or
//! `slcan:COM5` (USB-CAN dongle, classic CAN only).
//!
//! A single robot may run a classic-CAN bus and a CAN-FD bus side by side
//! (e.g. mixing motor lots that only speak one or the other); pick the
//! matching mode per interface. They are never mixed on the *same* wire.

use std::time::Duration;

use misa_actuator::Shared;
use misa_can::{OpenOptions, StandardId};

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

    /// How many times the transport reported frames lost before this process
    /// could read them.
    ///
    /// `0` where the transport cannot tell, which is every backend except
    /// PCAN — so **`0` means "none reported", not "none happened"**. See
    /// [`misa_can::CanBus::rx_overruns`].
    ///
    /// Carried up to here so a diagnostic run can separate "the motor did not
    /// answer" from "the answer was dropped on this side of the wire". Those
    /// look identical from a timeout, and telling them apart is the open
    /// question in the CAN-FD investigation (`doc/handover.md` §4).
    fn rx_overruns(&self) -> u64 {
        0
    }
}

/// Share one DAMIAO bus across several [`crate::DamiaoMotor`] handles on the
/// same wire: wrap an opened bus in [`misa_actuator::Shared`] and hand each
/// motor a clone. Access is serialized by the mutex — drive the motors from a
/// single control loop per bus (see the crate-level multi-motor docs).
///
/// Note: [`DamiaoBus::set_timeout`] here affects the *shared* transport, so
/// the last value set wins across all motors on the bus.
impl<B: DamiaoBus> DamiaoBus for Shared<B> {
    fn send(&mut self, can_id: u16, data: &[u8]) -> Result<()> {
        self.lock().send(can_id, data)
    }

    fn recv(&mut self) -> Result<CanFrame> {
        self.lock().recv()
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.lock().set_timeout(timeout)
    }

    fn rx_overruns(&self) -> u64 {
        self.lock().rx_overruns()
    }
}

/// Adapter presenting any [`misa_can::CanBus`] as a [`DamiaoBus`].
///
/// `fd` decides how *outgoing* frames are framed. Incoming frames are
/// accepted either way — a CAN-FD interface still receives classic frames, so
/// feedback is handled uniformly regardless of how the motor framed its reply.
pub struct CanBus<T: misa_can::CanBus> {
    inner: T,
    fd: bool,
}

/// The bus [`crate::DamiaoMotor::open`] and
/// [`crate::DamiaoMotor::open_fd`] produce.
pub type AnyCanBus = CanBus<Box<dyn misa_can::CanBus>>;

impl AnyCanBus {
    /// Open `interface` in **classic CAN** mode (1 Mbit/s by default).
    ///
    /// | interface | transport |
    /// |-----------|-----------|
    /// | `can0` | Linux SocketCAN |
    /// | `pcan:usb1` | PEAK PCAN-Basic (Windows) |
    /// | `slcan:COM5` | USB-CAN adapter |
    pub fn open(interface: &str) -> Result<Self> {
        Self::open_with(interface, &OpenOptions::classic())
    }

    /// Open `interface` in **CAN-FD** mode, sending BRS-enabled FD frames so
    /// the data phase runs at the data bitrate (5 Mbit/s by default; override
    /// with e.g. `pcan:usb1@1M,2M`).
    ///
    /// SLCAN adapters cannot do FD and are rejected here. On Linux the link
    /// must already be up with `fd on`:
    ///
    /// ```text
    /// sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on up
    /// ```
    pub fn open_fd(interface: &str) -> Result<Self> {
        Self::open_with(interface, &OpenOptions::fd())
    }

    /// Open the link in **CAN-FD** mode but send **classic** frames on it.
    ///
    /// A diagnostic, not a mode anyone should run a motor in. When a motor
    /// stops answering the moment you switch to `open_fd`, there are two
    /// explanations — the motor is not configured for CAN-FD, or our FD
    /// framing is wrong — and they call for completely different next steps.
    /// This separates them: the link is initialised exactly as `open_fd` does,
    /// so if the motor answers classic frames over it, the FD initialisation
    /// and bit timing are fine and the motor is simply not an FD node.
    pub fn open_fd_link_classic_frames(interface: &str) -> Result<Self> {
        Ok(Self {
            inner: misa_can::open(interface, &OpenOptions::fd()).map_err(map_err)?,
            fd: false,
        })
    }

    /// Open with explicit transport options.
    pub fn open_with(interface: &str, opts: &OpenOptions) -> Result<Self> {
        Ok(Self {
            inner: misa_can::open(interface, opts).map_err(map_err)?,
            fd: opts.fd,
        })
    }
}

impl<T: misa_can::CanBus> CanBus<T> {
    /// Wrap an already-open transport, framing sends as classic CAN.
    pub fn new(inner: T) -> Self {
        Self { inner, fd: false }
    }

    /// Wrap an already-open transport, framing sends as CAN-FD with BRS.
    pub fn new_fd(inner: T) -> Self {
        Self { inner, fd: true }
    }

    /// Borrow the underlying transport.
    pub fn inner(&self) -> &T {
        &self.inner
    }

    /// Whether outgoing frames are sent as CAN-FD.
    pub fn is_fd(&self) -> bool {
        self.fd
    }

    /// What is actually open, e.g. `PCAN_USBBUS1 (CAN-FD, ...)`.
    pub fn description(&self) -> String {
        self.inner.description()
    }
}

impl<T: misa_can::CanBus> DamiaoBus for CanBus<T> {
    fn send(&mut self, can_id: u16, data: &[u8]) -> Result<()> {
        let id = StandardId::new(can_id).ok_or(Error::InvalidFrame("CAN ID exceeds 11 bits"))?;
        let frame = if self.fd {
            // Bit-rate switching: run the data phase at the fast data bitrate.
            misa_can::Frame::new_fd(id, data, true)
        } else {
            misa_can::Frame::new(id, data)
        }
        .map_err(map_err)?;
        self.inner.send(&frame).map_err(map_err)
    }

    fn recv(&mut self) -> Result<CanFrame> {
        loop {
            let frame = self.inner.recv().map_err(map_err)?;
            // Extended frames belong to some other family on the wire.
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

    fn rx_overruns(&self) -> u64 {
        self.inner.rx_overruns()
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
