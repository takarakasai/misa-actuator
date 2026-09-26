//! Bus abstraction and the serial implementation.
//!
//! [`FsBus`] is the transport-level primitive set: send a frame, or send a
//! frame and collect the replies it provokes. Typed commands live on
//! [`crate::FsCommands`], blanket-implemented for every [`FsBus`].
//!
//! Frames are passed **encoded**: FashionStar has one wire format whatever
//! carries it, and sync commands are built directly as frames by the
//! protocol crate, so there is nothing for a transport to re-frame.

use std::time::{Duration, Instant};

use fashionstar_protocol::{Response, ResponseParser};
use misa_actuator::Shared;

use crate::error::{Error, Result};
use crate::transport::Transport;

/// Request/response transport for one FashionStar bus.
pub trait FsBus {
    /// Send a frame that gets no reply (angle, stop, origin, reset, ...).
    fn send(&mut self, frame: &[u8]) -> Result<()>;

    /// Send a frame and return the reply with `reply_code` from servo `id`.
    ///
    /// Frames that are not that reply are dropped. Fails with
    /// [`Error::Timeout`] when nothing matching arrives within
    /// [`Self::timeout`].
    fn transact(&mut self, frame: &[u8], reply_code: u8, id: u8) -> Result<Response>;

    /// Send a frame and collect one `reply_code` reply from each of `ids`.
    ///
    /// Returns as soon as every id has answered, otherwise when
    /// [`Self::timeout`] (for the whole collection, not per reply) expires.
    /// Slot `i` holds the reply of `ids[i]`, or `None` if it did not arrive.
    /// Replies are matched by the id inside them, not by arrival order.
    fn transact_many(
        &mut self,
        frame: &[u8],
        reply_code: u8,
        ids: &[u8],
    ) -> Result<Vec<Option<Response>>>;

    /// Drop buffered input.
    fn flush_rx(&mut self) -> Result<()>;

    /// Reply deadline used by [`Self::transact`] / [`Self::transact_many`].
    fn timeout(&self) -> Duration;

    /// Change the reply deadline.
    fn set_timeout(&mut self, timeout: Duration);
}

/// Share one bus across several [`crate::FashionStarServo`] handles (all
/// seven servos of an arm sit on one wire). Each call locks once, so a
/// request and its replies are one atomic transaction even with several
/// threads — but prefer one loop per bus, ideally doing one
/// [`crate::FsCommands::sync_monitor`] for all servos instead of seven
/// separate `measure` calls.
impl<B: FsBus> FsBus for Shared<B> {
    fn send(&mut self, frame: &[u8]) -> Result<()> {
        self.lock().send(frame)
    }

    fn transact(&mut self, frame: &[u8], reply_code: u8, id: u8) -> Result<Response> {
        self.lock().transact(frame, reply_code, id)
    }

    fn transact_many(
        &mut self,
        frame: &[u8],
        reply_code: u8,
        ids: &[u8],
    ) -> Result<Vec<Option<Response>>> {
        self.lock().transact_many(frame, reply_code, ids)
    }

    fn flush_rx(&mut self) -> Result<()> {
        self.lock().flush_rx()
    }

    fn timeout(&self) -> Duration {
        self.lock().timeout()
    }

    fn set_timeout(&mut self, timeout: Duration) {
        self.lock().set_timeout(timeout)
    }
}

/// FashionStar bus over a serial port (or any [`Transport`]).
///
/// The default transport is a `serialport` handle, opened 8N1 by
/// [`FashionStarBus::open`]; tests use [`crate::MockTransport`].
pub struct FashionStarBus<T: Transport = Box<dyn serialport::SerialPort>> {
    io: T,
    parser: ResponseParser,
    timeout: Duration,
}

/// The servos' factory baud rate, and what the Star Arm 102 runs at.
pub const DEFAULT_BAUD: u32 = 1_000_000;

impl FashionStarBus {
    /// Open `device` at `baud`, 8N1, no flow control (as the SDK's
    /// `serial.Serial(..., parity=NONE, stopbits=1, bytesize=8)`).
    ///
    /// `timeout` is the reply deadline for one transaction. A single reply
    /// takes well under a millisecond on the wire at 1 Mbit/s; most of the
    /// budget is USB-serial latency, so 10–20 ms is plenty.
    pub fn open(device: &str, baud: u32, timeout: Duration) -> Result<Self> {
        let device = crate::serial::normalize_port_name(device);
        let port = serialport::new(&device, baud)
            .timeout(timeout)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .open()?;
        log::info!("fashionstar: opened {device} @ {baud} baud");
        let mut bus = Self::from_transport(port, timeout);
        // Stale bytes from a previous session would otherwise be the first
        // thing the parser sees.
        bus.flush_rx()?;
        Ok(bus)
    }
}

impl<T: Transport> FashionStarBus<T> {
    /// Wrap an already-open transport.
    pub fn from_transport(io: T, timeout: Duration) -> Self {
        Self {
            io,
            parser: ResponseParser::new(),
            timeout,
        }
    }

    /// The transport, e.g. to inspect a [`crate::MockTransport`] in a test.
    pub fn transport(&self) -> &T {
        &self.io
    }

    /// Mutable access to the transport.
    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.io
    }

    /// Garbage bytes skipped by the parser since the bus was opened: echoes,
    /// noise, corrupted frames. Rising steadily means a bad line.
    pub fn discarded_bytes(&self) -> usize {
        self.parser.discarded()
    }

    fn write(&mut self, frame: &[u8]) -> Result<()> {
        self.io.write_all(frame)?;
        log::trace!("fashionstar TX {frame:02X?}");
        Ok(())
    }

    /// Start of every transaction: drop anything already received.
    ///
    /// A reply that missed its own deadline would otherwise still be in the
    /// buffer and — carrying the right code and id — be taken as the answer
    /// to the *next* request. In a sync-monitor loop that makes a slow servo
    /// report its previous-cycle angle forever after a single hiccup.
    fn begin(&mut self) -> Result<()> {
        self.parser.clear();
        self.io.discard_input()?;
        Ok(())
    }

    /// Read until `deadline`, feeding each complete frame to `on_frame`
    /// until it returns `true` ("done"). Returns whether it finished.
    fn pump(
        &mut self,
        deadline: Instant,
        mut on_frame: impl FnMut(Response) -> bool,
    ) -> Result<bool> {
        let mut scratch = [0u8; 256];
        loop {
            while let Some(resp) = self.parser.next_frame() {
                log::trace!("fashionstar RX code={} {:02X?}", resp.code, resp.params);
                if on_frame(resp) {
                    return Ok(true);
                }
            }
            let now = Instant::now();
            if now >= deadline {
                return Ok(false);
            }
            match self.io.read(&mut scratch, deadline - now) {
                Ok(n) => self.parser.push(&scratch[..n]),
                Err(e)
                    if e.kind() == std::io::ErrorKind::TimedOut
                        || e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::Interrupted => {}
                Err(e) => return Err(e.into()),
            }
        }
    }
}

impl<T: Transport> FsBus for FashionStarBus<T> {
    fn send(&mut self, frame: &[u8]) -> Result<()> {
        self.write(frame)
    }

    fn transact(&mut self, frame: &[u8], reply_code: u8, id: u8) -> Result<Response> {
        self.begin()?;
        self.write(frame)?;
        let deadline = Instant::now() + self.timeout;
        let mut found = None;
        self.pump(deadline, |resp| {
            if resp.code == reply_code && resp.servo_id() == Some(id) {
                found = Some(resp);
                true
            } else {
                log::debug!(
                    "fashionstar: dropping unrelated frame code={} id={:?}",
                    resp.code,
                    resp.servo_id()
                );
                false
            }
        })?;
        found.ok_or(Error::Timeout { servo_id: id })
    }

    fn transact_many(
        &mut self,
        frame: &[u8],
        reply_code: u8,
        ids: &[u8],
    ) -> Result<Vec<Option<Response>>> {
        let mut out: Vec<Option<Response>> = vec![None; ids.len()];
        if ids.is_empty() {
            return Ok(out);
        }
        self.begin()?;
        self.write(frame)?;
        let deadline = Instant::now() + self.timeout;
        let mut remaining = ids.len();
        self.pump(deadline, |resp| {
            if resp.code != reply_code {
                return false;
            }
            let Some(id) = resp.servo_id() else {
                return false;
            };
            // Matched by id, not position: the SDK does the same (it indexes
            // its cache by the id inside each reply), and nothing documents
            // that servos answer in request order.
            match ids
                .iter()
                .zip(out.iter())
                .position(|(&want, slot)| want == id && slot.is_none())
            {
                Some(i) => {
                    out[i] = Some(resp);
                    remaining -= 1;
                    remaining == 0
                }
                None => {
                    log::debug!("fashionstar: dropping unrequested/duplicate reply from id {id}");
                    false
                }
            }
        })?;
        Ok(out)
    }

    fn flush_rx(&mut self) -> Result<()> {
        self.begin()
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }

    fn set_timeout(&mut self, timeout: Duration) {
        self.timeout = timeout;
    }
}
