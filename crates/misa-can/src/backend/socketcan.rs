//! Linux SocketCAN backend.
//!
//! Behaviourally identical to the per-driver `SocketCanBus` types this crate
//! replaced: a blocking read with an `SO_RCVTIMEO`-style timeout, mapping the
//! would-block/timed-out kinds onto [`Error::Timeout`].
//!
//! The bitrate in the interface spec is *not* applied here — on Linux the
//! netdev is configured out of band (`ip link set can0 type can bitrate
//! 1000000 up`), and silently reconfiguring a shared interface from inside a
//! motor CLI would be surprising. A mismatch is reported instead.

use std::io;
use std::time::Duration;

use socketcan::{CanAnyFrame, CanFdFrame, CanFdSocket, CanSocket, EmbeddedFrame, Socket};

use crate::error::{Error, Result};
use crate::frame::Frame;
use crate::spec::InterfaceSpec;
use crate::{CanBus, OpenOptions};

/// SocketCAN transport. Opens a `CanFdSocket` when FD is requested (which also
/// receives classic frames) and a plain `CanSocket` otherwise.
pub struct SocketCanBackend {
    socket: SocketKind,
    interface: String,
    timeout: Duration,
}

enum SocketKind {
    Classic(CanSocket),
    Fd(CanFdSocket),
}

impl SocketCanBackend {
    /// Open a SocketCAN interface (`can0`, `vcan0`, ...).
    pub fn open(spec: &InterfaceSpec, opts: &OpenOptions) -> Result<Self> {
        let socket = if opts.fd {
            SocketKind::Fd(CanFdSocket::open(&spec.target).map_err(|e| {
                Error::Config(format!(
                    "cannot open {} in CAN-FD mode ({e}); is the link up with `fd on`?",
                    spec.target
                ))
            })?)
        } else {
            SocketKind::Classic(CanSocket::open(&spec.target).map_err(|e| {
                Error::Config(format!("cannot open SocketCAN interface {} ({e})", spec.target))
            })?)
        };

        let mut bus = Self {
            socket,
            interface: spec.target.clone(),
            timeout: opts.timeout,
        };
        bus.set_timeout(opts.timeout)?;
        Ok(bus)
    }

    /// Borrow the classic socket, for callers that need a SocketCAN-specific
    /// knob this trait does not expose.
    pub fn classic_socket(&self) -> Option<&CanSocket> {
        match &self.socket {
            SocketKind::Classic(s) => Some(s),
            SocketKind::Fd(_) => None,
        }
    }
}

impl CanBus for SocketCanBackend {
    fn send(&mut self, frame: &Frame) -> Result<()> {
        match &mut self.socket {
            SocketKind::Classic(s) => {
                if frame.is_fd() {
                    return Err(Error::Unsupported(
                        "CAN-FD frame on a classic SocketCAN socket",
                    ));
                }
                let f = socketcan::CanFrame::new(frame.id(), frame.data())
                    .ok_or(Error::Frame("payload too long for a classic CAN frame"))?;
                s.write_frame(&f)?;
            }
            SocketKind::Fd(s) => {
                let (buf, len) = frame.fd_padded();
                let mut f = CanFdFrame::new(frame.id(), &buf[..len])
                    .ok_or(Error::Frame("payload too long for a CAN-FD frame"))?;
                f.set_brs(frame.is_brs());
                s.write_frame(&f)?;
            }
        }
        Ok(())
    }

    fn recv(&mut self) -> Result<Frame> {
        loop {
            let decoded = match &mut self.socket {
                SocketKind::Classic(s) => match s.read_frame() {
                    Ok(f) => Some(Frame::from_raw(
                        raw_id(f.id()),
                        f.is_extended(),
                        f.data(),
                        false,
                        false,
                    )?),
                    Err(e) => return Err(map_io(e)),
                },
                SocketKind::Fd(s) => match s.read_frame() {
                    Ok(CanAnyFrame::Normal(f)) => Some(Frame::from_raw(
                        raw_id(f.id()),
                        f.is_extended(),
                        f.data(),
                        false,
                        false,
                    )?),
                    Ok(CanAnyFrame::Fd(f)) => Some(Frame::from_raw(
                        raw_id(f.id()),
                        f.is_extended(),
                        f.data(),
                        true,
                        f.is_brs(),
                    )?),
                    // Remote and error frames carry no motor payload.
                    Ok(CanAnyFrame::Remote(_)) | Ok(CanAnyFrame::Error(_)) => None,
                    Err(e) => return Err(map_io(e)),
                },
            };
            if let Some(frame) = decoded {
                return Ok(frame);
            }
        }
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        match &self.socket {
            SocketKind::Classic(s) => s.set_read_timeout(timeout)?,
            SocketKind::Fd(s) => s.set_read_timeout(timeout)?,
        }
        self.timeout = timeout;
        Ok(())
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }

    fn supports_fd(&self) -> bool {
        matches!(self.socket, SocketKind::Fd(_))
    }

    fn backend_name(&self) -> &'static str {
        "socketcan"
    }

    fn description(&self) -> String {
        let mode = if self.supports_fd() { "CAN-FD" } else { "CAN" };
        format!("SocketCAN {} ({mode})", self.interface)
    }
}

fn raw_id(id: socketcan::Id) -> u32 {
    use socketcan::Id;
    match id {
        Id::Standard(s) => s.as_raw() as u32,
        Id::Extended(e) => e.as_raw(),
    }
}

/// A read that timed out is not a failure — the drivers poll on it.
fn map_io(e: io::Error) -> Error {
    match e.kind() {
        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut => Error::Timeout,
        _ => Error::Io(e),
    }
}
