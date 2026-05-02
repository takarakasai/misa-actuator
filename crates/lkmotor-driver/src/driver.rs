//! Synchronous serial (RS485) bus implementation for the LK Motor V3 family.
//!
//! `Rs485Driver` is a thin wrapper around `serialport::SerialPort` that
//! implements the [`LkBus`](crate::bus::LkBus) trait. Typed command helpers
//! (`read_state2`, `torque_control`, ...) live on the
//! [`LkCommands`](crate::bus::LkCommands) blanket trait — bring it into
//! scope to call them.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use lkmotor_protocol::frame::{DecodeError, MAX_FRAME, try_decode};

use crate::bus::{LkBus, Response};
use crate::error::{Error, Result};
use crate::motor_id::MotorId;

/// Default per-byte read timeout used when polling for a response.
const READ_POLL_TIMEOUT: Duration = Duration::from_millis(20);

/// RS485 (V3 protocol) bus for the LKMTech servo motor family.
pub struct Rs485Driver {
    port: Box<dyn serialport::SerialPort>,
    rx_buf: Vec<u8>,
    response_timeout: Duration,
}

impl Rs485Driver {
    /// Open the serial port and prepare the bus.
    pub fn open(device: &str, baud: u32, response_timeout: Duration) -> Result<Self> {
        let port = serialport::new(device, baud)
            .timeout(READ_POLL_TIMEOUT)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .open()?;
        log::info!("lkmotor RS485: opened {} @ {} baud", device, baud);
        Ok(Self {
            port,
            rx_buf: Vec::with_capacity(MAX_FRAME * 2),
            response_timeout,
        })
    }

    /// Wrap an already-open port (useful for tests or custom setups).
    pub fn from_port(port: Box<dyn serialport::SerialPort>, response_timeout: Duration) -> Self {
        Self {
            port,
            rx_buf: Vec::with_capacity(MAX_FRAME * 2),
            response_timeout,
        }
    }

    /// Per-request timeout used when waiting for a response.
    pub fn set_response_timeout(&mut self, timeout: Duration) {
        self.response_timeout = timeout;
    }

    /// Send a fully encoded frame on the bus.
    pub fn send_raw(&mut self, command: u8, motor_id: MotorId, data: &[u8]) -> Result<()> {
        let mut buf = [0u8; MAX_FRAME];
        let n = lkmotor_protocol::frame::encode(command, motor_id.get(), data, &mut buf)?;
        self.port.write_all(&buf[..n])?;
        self.port.flush()?;
        log::debug!(
            "lkmotor TX: cmd=0x{:02X} id={} len={}",
            command,
            motor_id.get(),
            data.len()
        );
        Ok(())
    }

    /// Send a frame and wait for the next response addressed to `motor_id`.
    pub fn transact(
        &mut self,
        command: u8,
        motor_id: MotorId,
        data: &[u8],
    ) -> Result<Response> {
        self.send_raw(command, motor_id, data)?;
        self.recv_for(motor_id)
    }

    /// Read until a frame addressed to `motor_id` arrives or the deadline elapses.
    pub fn recv_for(&mut self, motor_id: MotorId) -> Result<Response> {
        let deadline = Instant::now() + self.response_timeout;
        let mut scratch = [0u8; 64];

        loop {
            match try_decode(&self.rx_buf) {
                Ok((frame, used)) => {
                    let resp = Response {
                        command: frame.command,
                        motor_id: frame.motor_id,
                        data: frame.data.to_vec(),
                    };
                    self.rx_buf.drain(..used);
                    if resp.motor_id == motor_id.get() {
                        log::debug!(
                            "lkmotor RX: cmd=0x{:02X} id={} len={}",
                            resp.command,
                            resp.motor_id,
                            resp.data.len()
                        );
                        return Ok(resp);
                    } else {
                        log::warn!(
                            "lkmotor: dropping unsolicited frame cmd=0x{:02X} id={}",
                            resp.command,
                            resp.motor_id
                        );
                        continue;
                    }
                }
                Err(DecodeError::NeedMore { .. }) => {}
                Err(DecodeError::BadHeader { .. }) => {
                    // Resync by dropping one byte and retrying.
                    self.rx_buf.remove(0);
                    continue;
                }
                Err(e) => {
                    return Err(e.into());
                }
            }

            if Instant::now() >= deadline {
                return Err(Error::Timeout {
                    motor_id: motor_id.get(),
                });
            }

            match self.port.read(&mut scratch) {
                Ok(0) => {}
                Ok(n) => self.rx_buf.extend_from_slice(&scratch[..n]),
                Err(ref e)
                    if e.kind() == std::io::ErrorKind::TimedOut
                        || e.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(e) => return Err(e.into()),
            }
        }
    }

    /// Read raw bytes from the wire for `window`, regardless of whether they
    /// form a valid frame. Diagnostic-only — returns whatever shows up.
    pub fn read_raw_for(&mut self, window: Duration) -> Result<Vec<u8>> {
        let deadline = Instant::now() + window;
        let mut out = Vec::new();
        let mut scratch = [0u8; 64];
        if !self.rx_buf.is_empty() {
            out.extend_from_slice(&self.rx_buf);
            self.rx_buf.clear();
        }
        while Instant::now() < deadline {
            match self.port.read(&mut scratch) {
                Ok(0) => {}
                Ok(n) => out.extend_from_slice(&scratch[..n]),
                Err(ref e)
                    if e.kind() == std::io::ErrorKind::TimedOut
                        || e.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(e) => return Err(e.into()),
            }
        }
        Ok(out)
    }

    /// Discard any buffered bytes (queued + already-on-wire).
    pub fn flush_rx(&mut self) -> Result<()> {
        self.rx_buf.clear();
        let mut scratch = [0u8; 256];
        loop {
            match self.port.read(&mut scratch) {
                Ok(0) => break,
                Ok(_) => continue,
                Err(ref e)
                    if e.kind() == std::io::ErrorKind::TimedOut
                        || e.kind() == std::io::ErrorKind::WouldBlock =>
                {
                    break;
                }
                Err(e) => return Err(e.into()),
            }
        }
        Ok(())
    }
}

impl LkBus for Rs485Driver {
    fn transact(&mut self, command: u8, motor_id: MotorId, data: &[u8]) -> Result<Response> {
        Rs485Driver::transact(self, command, motor_id, data)
    }

    fn flush_rx(&mut self) -> Result<()> {
        Rs485Driver::flush_rx(self)
    }
}
