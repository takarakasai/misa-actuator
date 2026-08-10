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
pub mod discover;
pub mod error;
pub mod frame;
pub mod spec;

use std::time::Duration;

pub use discover::{format_list, list_interfaces, Discovered};
pub use error::{Error, Result};
pub use frame::{ExtendedId, Frame, Id, StandardId, MAX_DATA_LEN};
pub use spec::{
    Backend, InterfaceSpec, DEFAULT_BITRATE, DEFAULT_DATA_BITRATE, DEFAULT_SLCAN_SERIAL_BAUD,
    DEFAULT_USBCAN_SERIAL_BAUD,
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

    /// How many error / status / remote-request frames the backend has dropped
    /// on the way to the caller.
    ///
    /// Not data, so callers never see them, but counted because a frame
    /// discarded without a record cannot be ruled out as the explanation for
    /// anything. Normally zero: error frames are not even delivered unless
    /// [`OpenOptions::allow_error_frames`] asked for them.
    fn rx_skipped(&self) -> u64 {
        0
    }

    /// The adapter's own timestamp for the frame [`Self::recv`] just returned,
    /// in microseconds since the channel was opened.
    ///
    /// `None` where the backend cannot say. Worth asking for when the question
    /// is about wire timing: a timestamp taken in this process is taken after
    /// queueing, scheduling and formatting, so a burst drained from the receive
    /// queue looks spread out and two frames microseconds apart can be recorded
    /// milliseconds apart. That is a difference between "the motor answered
    /// twice" and "the adapter handed us a queued pair".
    fn last_rx_timestamp_us(&self) -> Option<u64> {
        None
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

    fn rx_skipped(&self) -> u64 {
        (**self).rx_skipped()
    }

    fn last_rx_timestamp_us(&self) -> Option<u64> {
        (**self).last_rx_timestamp_us()
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
    /// Receive without participating: no transmissions, and **no
    /// acknowledgement**.
    ///
    /// Required of anything watching a bus it is not part of. On CAN the
    /// acknowledge slot is asserted by *any* node that received the frame
    /// correctly, so an ordinary second node acknowledges on the real
    /// recipient's behalf — which is exactly the observation a bus monitor is
    /// there to make. Without this a monitor does not watch the bus, it
    /// changes it.
    ///
    /// Only the PCAN backend implements it; the others reject it rather than
    /// silently listening as a participant.
    pub listen_only: bool,
    /// Deliver error frames instead of discarding them.
    ///
    /// Off by default because the drivers want data frames and nothing else.
    /// Worth turning on when the question is whether the wire is corrupting
    /// traffic, since a controller that detects an error signals it and the
    /// sender retransmits — so corruption shows up as error frames and delay
    /// rather than as loss, and none of that is visible while these are
    /// dropped.
    pub allow_error_frames: bool,
}

impl Default for OpenOptions {
    fn default() -> Self {
        Self {
            fd: false,
            timeout: DEFAULT_TIMEOUT,
            listen_only: false,
            allow_error_frames: false,
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

    /// Watch the bus without joining it — see [`Self::listen_only`].
    pub fn listening_only(mut self) -> Self {
        self.listen_only = true;
        self
    }

    /// Also deliver error frames — see [`Self::allow_error_frames`].
    pub fn allowing_error_frames(mut self) -> Self {
        self.allow_error_frames = true;
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
    // Refuse rather than come up as a participant. A monitor that quietly
    // acknowledges frames is worse than no monitor: it changes the bus it was
    // opened to observe, and nothing downstream would say so.
    if opts.listen_only && !spec.backend.can_listen_only() {
        return Err(Error::bad_spec(
            &spec.raw,
            format!(
                "{} cannot listen without acknowledging; the PCAN and usbcan backends \
                 are the ones with a silent mode",
                spec.backend.name()
            ),
        ));
    }
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
        Backend::UsbCan => Ok(Box::new(backend::usbcan::UsbCanBackend::open(spec, opts)?)),
        // `require_available` already rejected everything else; this arm only
        // exists so the match stays exhaustive on every target.
        #[allow(unreachable_patterns)]
        other => Err(Error::bad_spec(
            &spec.raw,
            format!("{} support is not compiled into this binary", other.name()),
        )),
    }
}

/// **Is this adapter alive?** Send a frame in loopback and check it comes back.
///
/// The USB-CAN Analyzer acknowledges nothing — not even its settings command — so
/// silence has too many explanations: a loose wire, the wrong serial framing, a
/// wrong bitrate, a dead unit, or simply no motor at the id being probed. All of
/// those cost an evening on 2026-08-05/06.
///
/// This separates the adapter from everything beyond it. In loopback the frame
/// never reaches the bus, so a pass proves the serial link in both directions, the
/// baud and stop bits, that the settings command was understood, and that the
/// packet format is right — leaving only the CAN side to blame. A fail means stop
/// looking at the wiring.
///
/// One process, because one serial port has one owner: send and read here rather
/// than in two terminals.
pub fn loopback_selftest(spec: &str) -> Result<String> {
    let mut parsed = InterfaceSpec::parse(spec)?;
    if parsed.backend != Backend::UsbCan {
        return Err(Error::bad_spec(
            spec,
            format!(
                "only the usbcan backend has a loopback mode; {} would have to be \
                 tested against a second adapter",
                parsed.backend.name()
            ),
        ));
    }
    parsed.loopback = true;

    let mut bus = open_spec(&parsed, &OpenOptions::classic().with_timeout(DEFAULT_TIMEOUT))?;
    // Deliberately awkward values: an id and payload that no motor family uses,
    // so a genuine bus frame arriving mid-test cannot be mistaken for the echo.
    let sent = Frame::new(StandardId::new(0x123).expect("0x123 is 11-bit"), &[0xDE, 0xAD, 0xBE])?;
    bus.send(&sent)?;

    let got = bus.recv().map_err(|e| match e {
        Error::Timeout => Error::Config(format!(
            "{spec}: nothing came back in loopback. The adapter is not \
             understanding what it is being sent — check the serial framing \
             (`?serial-baud=`, `?stop-bits=`) before suspecting the wiring, which \
             loopback does not use."
        )),
        other => other,
    })?;

    if got.raw_id() != sent.raw_id() || got.data() != sent.data() {
        return Err(Error::Config(format!(
            "{spec}: loopback returned a different frame (sent id 0x{:X} {:02X?}, \
             got id 0x{:X} {:02X?}) — the packet format is wrong, not the wiring",
            sent.raw_id(),
            sent.data(),
            got.raw_id(),
            got.data()
        )));
    }

    Ok(format!(
        "{spec}: loopback returned the frame unchanged (id 0x{:X}, {} byte(s)). \
         The adapter, the serial link and the packet format are all good, so \
         anything still wrong is on the CAN side: wiring, termination, bitrate, \
         or no motor at the ids being probed.",
        got.raw_id(),
        got.len()
    ))
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
        "SocketCAN interface (can0), or slcan:/dev/ttyACM0 for an adapter running \
         slcan firmware"
    } else {
        "PEAK channel (pcan:usb1) or an adapter running slcan firmware (slcan:COM5); \
         append @1M,5M to set the bitrates"
    }
}
