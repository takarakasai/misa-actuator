//! OS-agnostic CAN / CAN-FD transport for the misa-actuator drivers.
//!
//! Every motor driver in this workspace already abstracts its wire I/O behind
//! its own bus trait (`RobstrideBus`, `DamiaoBus`, `MyActuatorBus`). What was
//! missing was a transport those traits could sit on that is not Linux-only.
//! This crate is that layer: one [`CanBus`] trait, one [`Frame`] type, and a
//! backend per platform:
//!
//! | backend | platform | CAN-FD | notes |
//! |---------|----------|--------|-------|
//! | [`backend::socketcan`] | Linux | yes | the kernel CAN stack, unchanged behaviour |
//! | [`backend::pcan`] | Windows | yes | PEAK PCAN-Basic, `PCANBasic.dll` loaded at run time |
//! | [`backend::slcan`] | any | no | Lawicel ASCII over a USB serial port (CANable, ...) |
//!
//! Which one you get is decided by the `--interface` string — see
//! [`InterfaceSpec`] for the grammar. `can0` still means what it always meant;
//! `pcan:usb1` and `slcan:COM5` are the Windows equivalents.
//!
//! ```no_run
//! use misa_can::{OpenOptions, open};
//!
//! # fn main() -> Result<(), misa_can::Error> {
//! let mut bus = open("pcan:usb1@1M", &OpenOptions::default())?;
//! let frame = misa_can::Frame::new(misa_can::StandardId::new(0x141).unwrap(), &[0x9A; 8])?;
//! bus.send(&frame)?;
//! let reply = bus.recv()?;
//! println!("{reply:?}");
//! # Ok(())
//! # }
//! ```

pub mod backend;
pub mod error;
pub mod frame;
pub mod spec;

use std::time::Duration;

pub use error::{Error, Result};
pub use frame::{ExtendedId, Frame, Id, StandardId, MAX_DATA_LEN};
pub use spec::{
    Backend, InterfaceSpec, DEFAULT_BITRATE, DEFAULT_DATA_BITRATE, DEFAULT_SLCAN_SERIAL_BAUD,
};

/// Default per-receive timeout, matching what the drivers used to set on a
/// freshly opened SocketCAN socket.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(100);

/// A CAN transport: send a frame, receive the next one, bound the wait.
///
/// Implementations are expected to be *blocking* with a timeout, and to
/// return [`Error::Timeout`] — not an I/O error — when nothing arrives. The
/// drivers rely on that distinction to implement their retry loops.
pub trait CanBus: Send {
    /// Transmit one frame.
    fn send(&mut self, frame: &Frame) -> Result<()>;

    /// Block until a frame arrives or the timeout expires.
    ///
    /// Error, remote and status frames are swallowed by the backend — callers
    /// only ever see data frames.
    fn recv(&mut self) -> Result<Frame>;

    /// Set the per-receive timeout used by subsequent [`Self::recv`] calls.
    fn set_timeout(&mut self, timeout: Duration) -> Result<()>;

    /// The timeout currently in force.
    fn timeout(&self) -> Duration;

    /// Whether this bus can carry CAN-FD frames.
    fn supports_fd(&self) -> bool {
        false
    }

    /// How many times the transport has reported frames lost before this
    /// process could read them.
    ///
    /// Defaults to `0` on backends with no way to tell, so a caller can always
    /// ask. **`0` therefore means "none reported", not "none happened"** — say
    /// which when reporting it.
    ///
    /// Kept separate from the error path on purpose. A host-side receive-queue
    /// overrun loses data with every bus-health indicator clear: the frames
    /// arrived intact and were dropped above the wire. A missing reply is then
    /// indistinguishable from one the motor never sent, which is precisely the
    /// ambiguity left open in the DAMIAO CAN-FD investigation — where "the
    /// adapter reported no bus errors" was taken as evidence and could not
    /// have been. See `doc/handover.md` §4.
    fn rx_overruns(&self) -> u64 {
        0
    }

    /// Short backend name, for logs and error messages.
    fn backend_name(&self) -> &'static str;

    /// Human-readable description of what is actually open ("PCAN_USBBUS1 @ 1
    /// Mbit/s"), used by the CLIs when reporting a connection.
    fn description(&self) -> String;
}

impl CanBus for Box<dyn CanBus> {
    fn send(&mut self, frame: &Frame) -> Result<()> {
        (**self).send(frame)
    }

    fn recv(&mut self) -> Result<Frame> {
        (**self).recv()
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        (**self).set_timeout(timeout)
    }

    fn timeout(&self) -> Duration {
        (**self).timeout()
    }

    fn supports_fd(&self) -> bool {
        (**self).supports_fd()
    }

    fn rx_overruns(&self) -> u64 {
        (**self).rx_overruns()
    }

    fn backend_name(&self) -> &'static str {
        (**self).backend_name()
    }

    fn description(&self) -> String {
        (**self).description()
    }
}

/// How to bring the interface up.
#[derive(Debug, Clone)]
pub struct OpenOptions {
    /// Request CAN-FD. Backends that cannot do FD fail rather than silently
    /// degrading to classic CAN — a DAMIAO bus configured for FD would not
    /// answer a classic frame anyway.
    pub fd: bool,
    /// Initial receive timeout.
    pub timeout: Duration,
}

impl Default for OpenOptions {
    fn default() -> Self {
        Self {
            fd: false,
            timeout: DEFAULT_TIMEOUT,
        }
    }
}

impl OpenOptions {
    /// Classic CAN with the default timeout.
    pub fn classic() -> Self {
        Self::default()
    }

    /// CAN-FD with the default timeout.
    pub fn fd() -> Self {
        Self {
            fd: true,
            ..Self::default()
        }
    }

    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }
}

/// Open whatever `spec` names. See [`InterfaceSpec`] for the grammar.
pub fn open(spec: &str, opts: &OpenOptions) -> Result<Box<dyn CanBus>> {
    open_spec(&InterfaceSpec::parse(spec)?, opts)
}

/// Open an already-parsed spec.
pub fn open_spec(spec: &InterfaceSpec, opts: &OpenOptions) -> Result<Box<dyn CanBus>> {
    spec.require_available()?;
    log::debug!(
        "opening CAN interface {:?} via {} (fd={})",
        spec.raw,
        spec.backend.name(),
        opts.fd
    );
    match spec.backend {
        #[cfg(target_os = "linux")]
        Backend::SocketCan => Ok(Box::new(backend::socketcan::SocketCanBackend::open(
            spec, opts,
        )?)),
        #[cfg(windows)]
        Backend::Pcan => Ok(Box::new(backend::pcan::PcanBackend::open(spec, opts)?)),
        Backend::Slcan => Ok(Box::new(backend::slcan::SlcanBackend::open(spec, opts)?)),
        // `require_available` already rejected everything else; this arm only
        // exists so the match stays exhaustive on every target.
        #[allow(unreachable_patterns)]
        other => Err(Error::bad_spec(
            &spec.raw,
            format!("{} support is not compiled into this binary", other.name()),
        )),
    }
}

/// The `--interface` default that makes sense on this platform. Linux keeps
/// `can0`; Windows has no kernel CAN stack, so the first PEAK channel is the
/// closest thing to a conventional default.
pub const fn default_interface() -> &'static str {
    if cfg!(target_os = "linux") {
        "can0"
    } else {
        "pcan:usb1"
    }
}

/// One-line hint listing the interface forms this build accepts. CLIs put it
/// in their `--interface` help text so the right syntax is discoverable
/// without reading the docs.
pub fn interface_help() -> &'static str {
    if cfg!(target_os = "linux") {
        "SocketCAN interface (can0), or slcan:/dev/ttyACM0 for a USB-CAN adapter"
    } else {
        "PEAK channel (pcan:usb1) or USB-CAN adapter (slcan:COM5); \
         append @1M,5M to set the bitrates"
    }
}
