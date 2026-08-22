//! Synchronous serial (RS485) bus implementation for the LK Motor V3 family.
//!
//! `Rs485Driver` is a thin wrapper around `serialport::SerialPort` that
//! implements the [`LkBus`](crate::bus::LkBus) trait. Typed command helpers
//! (`read_state2`, `torque_control`, ...) live on the
//! [`LkCommands`](crate::bus::LkCommands) blanket trait — bring it into
//! scope to call them.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use lkmotor_protocol::broadcast::{
    BROADCAST_FRAME_LEN, BroadcastCommand, encode_broadcast_hybrid, encode_broadcast_values,
};
use lkmotor_protocol::frame::{DecodeError, MAX_FRAME, try_decode};

use crate::bus::{LkBus, Response};
use crate::error::{Error, Result};
use crate::motor_id::MotorId;

/// Initial read timeout applied when the port is opened. Each receive loop
/// then narrows it to the time left before its own deadline, so this is only
/// the value in effect before the first `response_timeout` window starts.
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
        // Windows needs `\\.\COM12` for two-digit ports; a bare name would
        // fail to open with a bare "not found".
        let device = crate::serial::normalize_port_name(device);
        let port = serialport::new(&device, baud)
            .timeout(READ_POLL_TIMEOUT)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .open()?;
        log::info!("lkmotor RS485: opened {} @ {} baud", device, baud);
        let mut driver = Self {
            port,
            rx_buf: Vec::with_capacity(MAX_FRAME * 2),
            response_timeout,
        };
        // Discard any stale bytes the kernel buffered from a previous session —
        // otherwise the first transaction decodes leftover garbage and fails
        // with a checksum error (and the failure can cascade).
        let _ = driver.flush_rx();
        Ok(driver)
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
                Err(_) => {
                    // Any other decode failure (bad header, bad checksum, ...)
                    // means the byte stream is misaligned — usually leftover or
                    // corrupted bytes. Resync by dropping one byte and retrying
                    // rather than aborting the whole transaction; the deadline
                    // below still bounds the search.
                    if !self.rx_buf.is_empty() {
                        self.rx_buf.remove(0);
                    }
                    continue;
                }
            }

            let now = Instant::now();
            if now >= deadline {
                return Err(Error::Timeout {
                    motor_id: motor_id.get(),
                });
            }
            // Shrink the poll(2) window to whatever is left before the
            // deadline. With the port stuck at READ_POLL_TIMEOUT, a silent bus
            // sleeps the full 20 ms no matter how small `response_timeout` is,
            // so the configured value never takes effect. `set_timeout` on a
            // TTYPort is a plain field assignment, so doing this per read costs
            // nothing, and a reply still wakes the poll immediately.
            let _ = self.port.set_timeout(deadline - now);

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

    /// Listen for another frame addressed to `motor_id` without transmitting.
    ///
    /// Used to catch two devices sharing one id: the first reply arrives
    /// through the normal transaction, and this picks up the second one.
    /// Returns `Ok(None)` if the window passes quietly, which is the normal
    /// case on a healthy bus.
    pub fn recv_extra_for(
        &mut self,
        motor_id: MotorId,
        window: Duration,
    ) -> Result<Option<Response>> {
        let deadline = Instant::now() + window;
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
                        return Ok(Some(resp));
                    }
                    // A frame from a different id is not what this is looking
                    // for, but it is still traffic — keep draining.
                    continue;
                }
                Err(DecodeError::NeedMore { .. }) => {}
                Err(_) => {
                    // Garbage here is expected: two motors answering at once
                    // collide on a half-duplex line. Resync a byte at a time
                    // rather than giving up, same as `recv_for`.
                    if !self.rx_buf.is_empty() {
                        self.rx_buf.remove(0);
                    }
                    continue;
                }
            }

            if Instant::now() >= deadline {
                return Ok(None);
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

    /// Send an already-encoded broadcast frame (see
    /// [`lkmotor_protocol::broadcast`]) raw on the wire, back-to-back with
    /// no gaps, as broadcast mode requires (the motor uses bus-idle time to
    /// find frame boundaries).
    fn send_broadcast_raw(&mut self, frame: &[u8]) -> Result<()> {
        self.port.write_all(frame)?;
        self.port.flush()?;
        log::debug!("lkmotor broadcast TX: {} bytes", frame.len());
        Ok(())
    }

    /// Collect up to `max_replies` broadcast-mode replies — ordinary
    /// single-motor frames, arriving one per responding motor in ascending
    /// ID order. Each reply gets its own `response_timeout` window; the
    /// method returns early with whatever was collected once a window
    /// elapses with nothing new, since a broadcast group need not have all
    /// 4 IDs populated.
    fn recv_broadcast_replies(&mut self, max_replies: usize) -> Result<Vec<Response>> {
        let mut replies = Vec::with_capacity(max_replies);
        let mut scratch = [0u8; 64];
        while replies.len() < max_replies {
            let deadline = Instant::now() + self.response_timeout;
            let mut got_one = false;
            loop {
                match try_decode(&self.rx_buf) {
                    Ok((frame, used)) => {
                        let resp = Response {
                            command: frame.command,
                            motor_id: frame.motor_id,
                            data: frame.data.to_vec(),
                        };
                        self.rx_buf.drain(..used);
                        replies.push(resp);
                        got_one = true;
                        break;
                    }
                    Err(DecodeError::NeedMore { .. }) => {}
                    Err(_) => {
                        if !self.rx_buf.is_empty() {
                            self.rx_buf.remove(0);
                        }
                        continue;
                    }
                }

                let now = Instant::now();
                if now >= deadline {
                    break;
                }
                // Same deadline-tracking as `recv_for`: without this the reply
                // window is floored at READ_POLL_TIMEOUT instead of
                // `response_timeout`.
                let _ = self.port.set_timeout(deadline - now);

                match self.port.read(&mut scratch) {
                    Ok(0) => {}
                    Ok(n) => self.rx_buf.extend_from_slice(&scratch[..n]),
                    Err(ref e)
                        if e.kind() == std::io::ErrorKind::TimedOut
                            || e.kind() == std::io::ErrorKind::WouldBlock => {}
                    Err(e) => return Err(e.into()),
                }
            }
            if !got_one {
                break;
            }
        }
        Ok(replies)
    }

    /// Send a frame and collect its broadcast-mode replies in one call.
    fn broadcast_transact(&mut self, frame: &[u8], max_replies: usize) -> Result<Vec<Response>> {
        self.send_broadcast_raw(frame)?;
        self.recv_broadcast_replies(max_replies)
    }

    /// Broadcast torque/open-loop control to up to 4 motors in one frame
    /// (`0x80`). `torque_raw[i]` is the signed raw torque-current value for
    /// motor `i+1` (range -2000..=2000 MF/MG, -850..=850 MS) — `0` for a
    /// motor that should not be commanded (broadcast mode always drives
    /// every listed slot; there is no "leave unchanged" value).
    /// `max_replies` bounds how many single-motor replies to wait for.
    pub fn broadcast_torque(
        &mut self,
        torque_raw: [i16; 4],
        max_replies: usize,
    ) -> Result<Vec<Response>> {
        let mut buf = [0u8; BROADCAST_FRAME_LEN];
        let n = encode_broadcast_values(BroadcastCommand::Torque, torque_raw, &mut buf)?;
        self.broadcast_transact(&buf[..n], max_replies)
    }

    /// Broadcast speed control to up to 4 motors in one frame (`0x81`).
    /// `speed_dps[i]` is dps for motor `i+1`; `0` to leave a slot uncommanded
    /// away from zero (see [`Self::broadcast_torque`]'s caveat).
    pub fn broadcast_speed(
        &mut self,
        speed_dps: [i16; 4],
        max_replies: usize,
    ) -> Result<Vec<Response>> {
        let mut buf = [0u8; BROADCAST_FRAME_LEN];
        let n = encode_broadcast_values(BroadcastCommand::Speed, speed_dps, &mut buf)?;
        self.broadcast_transact(&buf[..n], max_replies)
    }

    /// Broadcast absolute position control to up to 4 motors in one frame
    /// (`0x82`). `angle_centideg[i]` is `0.01 deg/LSB` for motor `i+1`,
    /// clamped to the wire field's `i16` range (`±327.67°`).
    pub fn broadcast_position(
        &mut self,
        angle_centideg: [i16; 4],
        max_replies: usize,
    ) -> Result<Vec<Response>> {
        let mut buf = [0u8; BROADCAST_FRAME_LEN];
        let n = encode_broadcast_values(BroadcastCommand::Position, angle_centideg, &mut buf)?;
        self.broadcast_transact(&buf[..n], max_replies)
    }

    /// Broadcast a distinct single-motor command byte per motor slot
    /// (`0x88`) — e.g. read state1/state2, on/off/stop (see
    /// [`lkmotor_protocol::broadcast::BroadcastCommand::Hybrid`]'s doc for
    /// the supported `motorCmd` bytes). `0x00` for an unused slot.
    pub fn broadcast_hybrid(
        &mut self,
        motor_cmds: [u8; 4],
        max_replies: usize,
    ) -> Result<Vec<Response>> {
        let mut buf = [0u8; BROADCAST_FRAME_LEN];
        let n = encode_broadcast_hybrid(motor_cmds, &mut buf)?;
        self.broadcast_transact(&buf[..n], max_replies)
    }
}

impl LkBus for Rs485Driver {
    fn transact(&mut self, command: u8, motor_id: MotorId, data: &[u8]) -> Result<Response> {
        Rs485Driver::transact(self, command, motor_id, data)
    }

    fn send_only(&mut self, command: u8, motor_id: MotorId, data: &[u8]) -> Result<()> {
        Rs485Driver::send_raw(self, command, motor_id, data)
    }

    fn flush_rx(&mut self) -> Result<()> {
        Rs485Driver::flush_rx(self)
    }

    fn recv_extra(&mut self, motor_id: MotorId, window: Duration) -> Result<Option<Response>> {
        Rs485Driver::recv_extra_for(self, motor_id, window)
    }

    fn can_detect_duplicate_ids(&self) -> bool {
        true
    }
}
