//! SLCAN (Lawicel ASCII) backend — works on any platform `serialport` does.
//!
//! This is the cheap route onto a CAN bus from Windows: a CANable /
//! candleLight / USBtin-class adapter running slcan firmware shows up as a
//! virtual COM port and speaks a line protocol:
//!
//! ```text
//! S8\r                      set 1 Mbit/s
//! O\r                       open the channel
//! t1418AABBCCDDEEFF0011\r   send std id 0x141, 8 bytes
//! T12345678 8 ...\r         send ext id (no spaces on the wire)
//! ```
//!
//! Only classic CAN exists in the protocol, so a CAN-FD request is rejected
//! rather than silently downgraded — a bus configured for FD would not answer
//! classic frames anyway.
//!
//! Throughput is bounded by the USB serial link, not by CAN: every 8-byte
//! frame costs 22 ASCII bytes each way. That is fine for status polling and
//! the quasi-static characterizations, and marginal for a high-rate chirp —
//! prefer a PEAK adapter when the loop rate matters.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use crate::error::{Error, Result};
use crate::frame::Frame;
use crate::spec::InterfaceSpec;
use crate::{CanBus, OpenOptions};

/// `S<n>` bitrate selectors defined by the Lawicel protocol.
const SLCAN_BITRATES: &[(u32, u8)] = &[
    (10_000, b'0'),
    (20_000, b'1'),
    (50_000, b'2'),
    (100_000, b'3'),
    (125_000, b'4'),
    (250_000, b'5'),
    (500_000, b'6'),
    (800_000, b'7'),
    (1_000_000, b'8'),
];

/// The adapter answers every command with CR (ok) or BEL (rejected).
const CR: u8 = b'\r';
const BEL: u8 = 0x07;

/// How long a single serial read is allowed to block. `recv` loops on this
/// until its own deadline, so the value only bounds how precisely a timeout
/// lands — reads return as soon as any byte arrives.
const READ_QUANTUM: Duration = Duration::from_millis(5);

/// Guard against a wedged adapter streaming garbage without a CR.
const MAX_LINE: usize = 64;

pub struct SlcanBackend {
    port: Box<dyn serialport::SerialPort>,
    port_name: String,
    bitrate: u32,
    timeout: Duration,
    /// Bytes received but not yet terminated by CR.
    partial: Vec<u8>,
    /// Raw read scratch buffer, kept alive so recv() never allocates.
    scratch: [u8; 256],
}

impl SlcanBackend {
    /// Open the serial port and bring the CAN channel up.
    pub fn open(spec: &InterfaceSpec, opts: &OpenOptions) -> Result<Self> {
        if opts.fd {
            return Err(Error::Unsupported(
                "SLCAN adapters speak classic CAN only — use a PEAK adapter (pcan:usb1) \
                 for CAN-FD",
            ));
        }

        let selector = SLCAN_BITRATES
            .iter()
            .find(|(rate, _)| *rate == spec.bitrate)
            .map(|(_, sel)| *sel)
            .ok_or_else(|| {
                let known: Vec<String> = SLCAN_BITRATES
                    .iter()
                    .map(|(r, _)| format!("{}", r / 1000))
                    .collect();
                Error::bad_spec(
                    &spec.raw,
                    format!(
                        "SLCAN supports only these bitrates (kbit/s): {}",
                        known.join(", ")
                    ),
                )
            })?;

        let port_name = normalize_port_name(&spec.target);
        let port = serialport::new(&port_name, spec.serial_baud)
            .timeout(READ_QUANTUM)
            .open()
            .map_err(|e| {
                Error::Config(format!(
                    "cannot open serial port {port_name} ({e}); \
                     check the port name in Device Manager and that nothing else has it open"
                ))
            })?;

        // Kept aside because the bring-up error message below is built while
        // `bus` is mutably borrowed.
        let named = port_name.clone();

        let mut bus = Self {
            port,
            port_name,
            bitrate: spec.bitrate,
            timeout: opts.timeout,
            partial: Vec::with_capacity(MAX_LINE),
            scratch: [0u8; 256],
        };

        // Opening the port proves only that *something* is on that COM
        // number — a Bluetooth stack and a modem open just as happily as a
        // CAN adapter. Everything that can distinguish them happens in
        // bring_up, so that is where the actionable message belongs.
        bus.bring_up(selector).map_err(|e| match e {
            // `command` already says which command was rejected and why.
            Error::Config(msg) => Error::Config(msg),
            other => Error::Config(format!(
                "SLCAN bring-up on {} failed ({other}); the port opened but does \
                 not behave like an slcan adapter — check the COM port number \
                 and that the adapter runs slcan firmware rather than \
                 candleLight/gs_usb",
                named
            )),
        })?;

        log::info!("SLCAN {} up at {} bit/s", bus.port_name, bus.bitrate);
        Ok(bus)
    }

    /// Close any channel left open by a previous run, set the bitrate, open.
    fn bring_up(&mut self, selector: u8) -> Result<()> {
        // Closing first is what makes the bitrate command take; a channel that
        // was already closed answers with BEL, so ignore the verdict.
        let _ = self.write_line(b"C");
        self.drain();

        self.command(&[b'S', selector])?;
        self.command(b"O")
    }

    fn write_line(&mut self, body: &[u8]) -> Result<()> {
        let mut line = Vec::with_capacity(body.len() + 1);
        line.extend_from_slice(body);
        line.push(CR);
        self.port.write_all(&line)?;
        self.port.flush()?;
        Ok(())
    }

    /// Send a setup command and wait for the adapter's CR / BEL verdict.
    /// Frames that arrive in the meantime are kept for [`Self::recv`].
    fn command(&mut self, body: &[u8]) -> Result<()> {
        self.write_line(body)?;
        let deadline = Instant::now() + Duration::from_millis(500);
        loop {
            match self.next_line(deadline)? {
                Some(line) if line.is_empty() => return Ok(()), // bare CR = ack
                Some(line) if line == [BEL] => {
                    return Err(Error::Config(format!(
                        "SLCAN adapter on {} rejected command {:?}",
                        self.port_name,
                        String::from_utf8_lossy(body)
                    )));
                }
                // Anything else is unsolicited traffic; keep waiting.
                Some(_) => continue,
                None => {
                    return Err(Error::Config(format!(
                        "SLCAN adapter on {} did not answer command {:?}; \
                         is it running slcan firmware?",
                        self.port_name,
                        String::from_utf8_lossy(body)
                    )));
                }
            }
        }
    }

    /// Throw away anything already buffered on the wire.
    fn drain(&mut self) {
        let deadline = Instant::now() + Duration::from_millis(50);
        while Instant::now() < deadline {
            match self.port.read(&mut self.scratch) {
                Ok(0) => break,
                Ok(_) => continue,
                Err(_) => break,
            }
        }
        self.partial.clear();
    }

    /// Read one CR-terminated line, or `None` once `deadline` passes.
    ///
    /// The returned line excludes the CR. A lone BEL is returned as a
    /// one-byte line so [`Self::command`] can see it.
    fn next_line(&mut self, deadline: Instant) -> Result<Option<Vec<u8>>> {
        loop {
            if let Some(pos) = self.partial.iter().position(|&b| b == CR) {
                let line: Vec<u8> = self.partial.drain(..=pos).take(pos).collect();
                return Ok(Some(line));
            }
            // A BEL is not CR-terminated on some firmwares.
            if self.partial.first() == Some(&BEL) {
                self.partial.remove(0);
                return Ok(Some(vec![BEL]));
            }
            if self.partial.len() > MAX_LINE {
                log::warn!(
                    "SLCAN {}: discarding {} bytes with no line terminator",
                    self.port_name,
                    self.partial.len()
                );
                self.partial.clear();
            }
            if Instant::now() >= deadline {
                return Ok(None);
            }
            match self.port.read(&mut self.scratch) {
                Ok(0) => continue,
                Ok(n) => {
                    let chunk = &self.scratch[..n];
                    self.partial.extend_from_slice(chunk);
                }
                Err(ref e) if is_timeout(e) => continue,
                Err(e) => return Err(Error::Io(e)),
            }
        }
    }
}

impl CanBus for SlcanBackend {
    fn send(&mut self, frame: &Frame) -> Result<()> {
        if frame.is_fd() {
            return Err(Error::Unsupported("CAN-FD frame on an SLCAN adapter"));
        }
        if frame.len() > 8 {
            return Err(Error::Frame("payload too long for a classic CAN frame"));
        }

        let mut line = Vec::with_capacity(2 + 8 + 1 + 16);
        match frame.extended_id() {
            Some(id) => {
                line.push(b'T');
                push_hex(&mut line, id, 8);
            }
            None => {
                line.push(b't');
                push_hex(&mut line, frame.raw_id(), 3);
            }
        }
        push_hex(&mut line, frame.len() as u32, 1);
        for byte in frame.data() {
            push_hex(&mut line, *byte as u32, 2);
        }
        self.write_line(&line)
    }

    fn recv(&mut self) -> Result<Frame> {
        let deadline = Instant::now() + self.timeout;
        loop {
            match self.next_line(deadline)? {
                None => return Err(Error::Timeout),
                Some(line) => {
                    if let Some(frame) = parse_frame_line(&line)? {
                        return Ok(frame);
                    }
                    // Acks, version strings, remote frames — not our payload.
                }
            }
        }
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.timeout = timeout;
        Ok(())
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }

    fn backend_name(&self) -> &'static str {
        "slcan"
    }

    fn description(&self) -> String {
        format!("SLCAN {} (CAN, {} bit/s)", self.port_name, self.bitrate)
    }
}

impl Drop for SlcanBackend {
    fn drop(&mut self) {
        // Leave the adapter closed so the next run can reconfigure it.
        let _ = self.write_line(b"C");
    }
}

/// Windows needs the `\\.\` device-namespace prefix for COM10 and above.
fn normalize_port_name(name: &str) -> String {
    if !cfg!(windows) || name.starts_with(r"\\.\") {
        return name.to_string();
    }
    let upper = name.to_ascii_uppercase();
    if let Some(digits) = upper.strip_prefix("COM") {
        if digits.chars().all(|c| c.is_ascii_digit()) && digits.len() > 1 {
            return format!(r"\\.\{upper}");
        }
    }
    name.to_string()
}

fn push_hex(out: &mut Vec<u8>, value: u32, digits: usize) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for shift in (0..digits).rev() {
        out.push(HEX[((value >> (shift * 4)) & 0xF) as usize]);
    }
}

fn hex_value(byte: u8) -> Option<u32> {
    match byte {
        b'0'..=b'9' => Some((byte - b'0') as u32),
        b'a'..=b'f' => Some((byte - b'a' + 10) as u32),
        b'A'..=b'F' => Some((byte - b'A' + 10) as u32),
        _ => None,
    }
}

fn hex_field(bytes: &[u8]) -> Option<u32> {
    let mut acc = 0u32;
    for &b in bytes {
        acc = acc.checked_mul(16)?.checked_add(hex_value(b)?)?;
    }
    Some(acc)
}

/// Decode one received SLCAN line. `Ok(None)` for anything that is not a data
/// frame (acks, remote frames, version banners).
fn parse_frame_line(line: &[u8]) -> Result<Option<Frame>> {
    let Some(&tag) = line.first() else {
        return Ok(None);
    };
    let (id_digits, extended) = match tag {
        b't' => (3usize, false),
        b'T' => (8usize, true),
        // Remote frames carry no payload; everything else is chatter.
        _ => return Ok(None),
    };

    let body = &line[1..];
    if body.len() < id_digits + 1 {
        log::warn!("SLCAN: truncated frame line {:?}", String::from_utf8_lossy(line));
        return Ok(None);
    }
    let Some(id) = hex_field(&body[..id_digits]) else {
        log::warn!("SLCAN: bad id in {:?}", String::from_utf8_lossy(line));
        return Ok(None);
    };
    let Some(len) = hex_value(body[id_digits]) else {
        log::warn!("SLCAN: bad length in {:?}", String::from_utf8_lossy(line));
        return Ok(None);
    };
    let len = len as usize;
    if len > 8 {
        log::warn!("SLCAN: length {len} exceeds classic CAN");
        return Ok(None);
    }

    // Some firmwares append a 4-hex-digit timestamp; take only the payload.
    let payload_hex = &body[id_digits + 1..];
    if payload_hex.len() < len * 2 {
        log::warn!(
            "SLCAN: frame line {:?} promises {len} bytes but carries {}",
            String::from_utf8_lossy(line),
            payload_hex.len() / 2
        );
        return Ok(None);
    }
    let mut data = [0u8; 8];
    for i in 0..len {
        let Some(v) = hex_field(&payload_hex[i * 2..i * 2 + 2]) else {
            log::warn!("SLCAN: bad payload in {:?}", String::from_utf8_lossy(line));
            return Ok(None);
        };
        data[i] = v as u8;
    }

    Ok(Some(Frame::from_raw(
        id,
        extended,
        &data[..len],
        false,
        false,
    )?))
}

fn is_timeout(e: &std::io::Error) -> bool {
    matches!(
        e.kind(),
        std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_standard_frame() {
        let f = parse_frame_line(b"t1418AABBCCDDEEFF0011")
            .unwrap()
            .unwrap();
        assert_eq!(f.standard_id(), Some(0x141));
        assert_eq!(f.data(), &[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11]);
        assert!(!f.is_extended());
    }

    #[test]
    fn parses_an_extended_frame() {
        let f = parse_frame_line(b"T1234567880011223344556677")
            .unwrap()
            .unwrap();
        assert_eq!(f.extended_id(), Some(0x1234_5678));
        assert_eq!(f.len(), 8);
        assert_eq!(f.data()[0], 0x00);
        assert_eq!(f.data()[7], 0x77);
    }

    #[test]
    fn ignores_non_frame_lines() {
        assert!(parse_frame_line(b"").unwrap().is_none());
        assert!(parse_frame_line(b"V1013").unwrap().is_none());
        assert!(parse_frame_line(b"r1410").unwrap().is_none());
        // Truncated payload is dropped, not misread.
        assert!(parse_frame_line(b"t1418AABB").unwrap().is_none());
    }

    #[test]
    fn tolerates_a_trailing_timestamp() {
        // `Z1` mode appends 4 hex digits after the payload.
        let f = parse_frame_line(b"t14181122334455667788ABCD")
            .unwrap()
            .unwrap();
        assert_eq!(f.data(), &[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);
    }

    #[test]
    fn encodes_hex_fields() {
        let mut out = Vec::new();
        push_hex(&mut out, 0x141, 3);
        push_hex(&mut out, 8, 1);
        assert_eq!(out, b"1418");
    }

    #[test]
    fn com_ports_above_nine_get_the_device_prefix() {
        if cfg!(windows) {
            assert_eq!(normalize_port_name("COM10"), r"\\.\COM10");
            assert_eq!(normalize_port_name("com12"), r"\\.\COM12");
            // Single-digit ports work either way; leave them alone.
            assert_eq!(normalize_port_name("COM5"), "COM5");
        } else {
            assert_eq!(normalize_port_name("/dev/ttyACM0"), "/dev/ttyACM0");
        }
    }
}
