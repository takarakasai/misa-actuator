//! "USB-CAN Analyzer" backend — the CH340 dongle's own binary protocol.
//!
//! Not an SLCAN adapter, though it looks like one from the outside. It answers
//! nothing at all to the Lawicel commands: confirmed 2026-08-05 with `V`+CR at
//! 1228800 / 2000000 / 115200 / 38400 / 9600 baud, plus a sync CR, lowercase
//! `v`, and `C`→`S8`→`V` — silent every time.
//!
//! # Provenance
//!
//! **Neither source is in this repository** — the licensing of some of the
//! material is unclear, so it is deliberately not committed. Everything needed to
//! maintain this backend is therefore written out below rather than cited.
//!
//! - **the vendor's product note.** Good for the hardware and the mode list. Its
//!   "send/receive data format" is *not* the data-frame format: the 20-byte
//!   checksummed layout it describes is the settings command. A backend built
//!   from the note alone cannot exchange a single frame.
//! - **`canusb.c`**, a Linux tool for this adapter (MIT, © 2018 Kjetil Erga). The
//!   authority for the wire format, because it is a description that has been run
//!   against hardware: the two packet shapes, the info byte's bits, the speed and
//!   mode codes, the checksum range, and the serial framing all come from it.
//!
//! Where they disagree, the working code wins:
//!
//! | | vendor note | `canusb.c` | here |
//! |---|---|---|---|
//! | data frame | 20 bytes, checksummed | variable, `0x55`-terminated | `canusb.c` |
//! | serial baud | 1,228,800 | 2,000,000 | `canusb.c`, `?serial-baud=` for the other |
//! | stop bits | 1 | 2 (`CSTOPB`) | `canusb.c`, `?stop-bits=` for the other |
//!
//! The framing row has **not** been settled by experiment. The adapter never
//! acknowledges its settings command, so a wrong choice raises no error and
//! simply receives nothing — the first thing to check on a bring-up, and the
//! reason both alternatives are reachable from the interface string.
//!
//! # Two packet shapes, one header byte
//!
//! Everything starts with `0xAA`, and the *second* byte says which shape it is.
//!
//! **Settings command — 20 bytes, fixed.** Sent once at open to set the bitrate
//! and mode.
//!
//! ```text
//!  0  0xAA
//!  1  0x55                 marks a command
//!  2  0x12                 "settings"
//!  3  speed code           0x01 = 1 Mbit/s … 0x0C = 5 kbit/s
//!  4  0x01 | 0x02          filter frame type: standard | extended
//!  5..8                    filter id  (unused, zero)
//!  9..12                   mask id    (unused, zero)
//! 13  mode                 0 normal, 1 loopback, 2 silent, 3 loopback+silent
//! 14  0x01
//! 15..18                   zero
//! 19  checksum             low byte of the sum of bytes 2..=18
//! ```
//!
//! **Data frame — variable length.** No checksum; a trailing `0x55` closes it.
//!
//! ```text
//!  0  0xAA
//!  1  0b11_E_R_LLLL        bits 7,6 always set; E = extended; R = remote;
//!                          LLLL = DLC
//!  2..                     id, low byte first — 2 bytes standard, 4 extended
//!  ..                      DLC payload bytes
//! last 0x55                end of frame
//! ```
//!
//! So a standard frame is `DLC + 5` bytes and an extended one `DLC + 7`.
//!
//! **`canusb.c` uses `DLC + 5` for both** (`frame_is_complete`), even though its
//! own send path writes four id bytes for an extended frame. It therefore
//! declares an extended frame complete two bytes early and desynchronises the
//! stream after the first one. That would bite immediately here — RobStride
//! addresses are 29-bit — so this implementation counts per frame type and also
//! checks the terminator, which turns a length mistake into a skipped packet
//! rather than a silently mangled reading. There is a test for exactly that
//! truncation.
//!
//! The serial link is 8 data bits, no parity, and the baud and stop bits from the
//! table above.
//!
//! # What this backend cannot tell you
//!
//! No receive-overrun count and no hardware timestamps — the protocol carries
//! neither, so [`CanBus::rx_overruns`] stays 0 and
//! [`CanBus::last_rx_timestamp_us`] stays `None`. Those two are what settled the
//! DAMIAO CAN-FD investigation (`doc/handover.md` §4), so prefer a PEAK adapter
//! for diagnostic work. Classic CAN only.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use crate::error::{Error, Result};
use crate::frame::Frame;
use crate::spec::InterfaceSpec;
use crate::{CanBus, OpenOptions};

/// Bitrate → speed code for the settings command.
const SPEED_CODES: &[(u32, u8)] = &[
    (1_000_000, 0x01),
    (800_000, 0x02),
    (500_000, 0x03),
    (400_000, 0x04),
    (250_000, 0x05),
    (200_000, 0x06),
    (125_000, 0x07),
    (100_000, 0x08),
    (50_000, 0x09),
    (20_000, 0x0A),
    (10_000, 0x0B),
    (5_000, 0x0C),
];

const PACKET_START: u8 = 0xAA;
const PACKET_END: u8 = 0x55;
/// Second byte of a settings/command packet (and its terminator value reused).
const COMMAND_MARK: u8 = 0x55;
const COMMAND_LEN: usize = 20;
const COMMAND_SETTINGS: u8 = 0x12;

/// Top two bits of a data frame's info byte.
const DATA_FRAME_MARK: u8 = 0xC0;
const INFO_EXTENDED: u8 = 0x20;
const INFO_REMOTE: u8 = 0x10;
const INFO_DLC: u8 = 0x0F;

const MODE_NORMAL: u8 = 0x00;
const MODE_SILENT: u8 = 0x02;

const FILTER_FRAME_EXTENDED: u8 = 0x02;

/// Longest packet either shape can be: an extended data frame with 8 bytes.
const MAX_DATA_PACKET: usize = 15;

/// How long a single serial read may block. `recv` loops on this until its own
/// deadline, so this only bounds how precisely a timeout lands.
const READ_QUANTUM: Duration = Duration::from_millis(5);

/// Cap on unparsed bytes held while looking for a packet, so a wedged adapter
/// streaming garbage cannot grow the buffer without limit.
const MAX_PARTIAL: usize = 8 * MAX_DATA_PACKET;

pub struct UsbCanBackend {
    port: Box<dyn serialport::SerialPort>,
    port_name: String,
    bitrate: u32,
    serial_baud: u32,
    listen_only: bool,
    timeout: Duration,
    /// Bytes read but not yet forming a complete packet.
    partial: Vec<u8>,
    scratch: [u8; 256],
    skipped: u64,
    warned_about_skips: bool,
}

impl UsbCanBackend {
    pub fn open(spec: &InterfaceSpec, opts: &OpenOptions) -> Result<Self> {
        if opts.fd {
            return Err(Error::Unsupported(
                "the USB-CAN Analyzer protocol carries at most 8 data bytes and no FD \
                 flags — use a PEAK adapter (pcan:usb1) for CAN-FD",
            ));
        }

        let speed = SPEED_CODES
            .iter()
            .find(|(rate, _)| *rate == spec.bitrate)
            .map(|(_, code)| *code)
            .ok_or_else(|| {
                let known: Vec<String> = SPEED_CODES
                    .iter()
                    .map(|(r, _)| format!("{}", r / 1000))
                    .collect();
                Error::bad_spec(
                    &spec.raw,
                    format!(
                        "the USB-CAN Analyzer supports only these bitrates (kbit/s): {}",
                        known.join(", ")
                    ),
                )
            })?;

        let port_name = normalize_port_name(&spec.target);
        let stop_bits = match spec.stop_bits {
            1 => serialport::StopBits::One,
            2 => serialport::StopBits::Two,
            other => {
                return Err(Error::bad_spec(
                    &spec.raw,
                    format!("stop-bits must be 1 or 2, not {other}"),
                ))
            }
        };
        let port = serialport::new(&port_name, spec.serial_baud)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(stop_bits)
            .flow_control(serialport::FlowControl::None)
            .timeout(READ_QUANTUM)
            .open()
            .map_err(|e| {
                Error::Config(format!(
                    "cannot open serial port {port_name} ({e}); \
                     check the port name in Device Manager and that nothing else has it \
                     open — the vendor's own tool holds the port while it is running"
                ))
            })?;

        let mut bus = Self {
            port,
            port_name,
            bitrate: spec.bitrate,
            serial_baud: spec.serial_baud,
            listen_only: opts.listen_only,
            timeout: opts.timeout,
            partial: Vec::with_capacity(MAX_PARTIAL),
            scratch: [0u8; 256],
            skipped: 0,
            warned_about_skips: false,
        };

        let mode = if opts.listen_only {
            MODE_SILENT
        } else {
            MODE_NORMAL
        };
        bus.send_settings(speed, mode)?;

        log::info!(
            "USB-CAN Analyzer {} up at {} bit/s ({} baud, {} stop bit(s){})",
            bus.port_name,
            bus.bitrate,
            bus.serial_baud,
            spec.stop_bits,
            if opts.listen_only { ", silent" } else { "" }
        );
        Ok(bus)
    }

    /// Send the settings command and clear whatever the adapter echoes.
    ///
    /// Nothing is waited for: the adapter does not acknowledge, so there is no
    /// verdict to read. That means **a wrong serial framing cannot be detected
    /// here** — the write succeeds into a device that never understood it, and
    /// the symptom arrives later as a receive timeout. Hence the loud logging of
    /// baud and stop bits at open.
    fn send_settings(&mut self, speed: u8, mode: u8) -> Result<()> {
        let packet = settings_packet(speed, mode);
        self.port.write_all(&packet)?;
        self.port.flush()?;
        // Anything already buffered predates the new settings.
        let deadline = Instant::now() + Duration::from_millis(50);
        while Instant::now() < deadline {
            match self.port.read(&mut self.scratch) {
                Ok(0) => break,
                Ok(_) => continue,
                Err(_) => break,
            }
        }
        self.partial.clear();
        Ok(())
    }

    fn note_skip(&mut self, why: &str) {
        self.skipped += 1;
        if !self.warned_about_skips {
            self.warned_about_skips = true;
            log::warn!(
                "USB-CAN Analyzer {}: discarding a packet ({why}). If nothing is ever \
                 received, check the serial framing first — the vendor note says \
                 1228800 baud / 1 stop bit while the working reference uses 2000000 / 2, \
                 and `?serial-baud=` and `?stop-bits=` switch between them.",
                self.port_name
            );
        } else {
            log::debug!(
                "USB-CAN Analyzer {}: discarding a packet ({why})",
                self.port_name
            );
        }
    }

    /// Pull the next complete packet out of `partial`, reading more if needed.
    ///
    /// Returns the packet with its framing intact, so [`decode`] can re-check the
    /// shape it was parsed as.
    fn next_packet(&mut self, deadline: Instant) -> Result<Option<Vec<u8>>> {
        loop {
            // Drop anything before a start byte.
            while !self.partial.is_empty() && self.partial[0] != PACKET_START {
                self.partial.remove(0);
                self.note_skip("byte before a packet start");
            }
            if self.partial.len() >= 2 {
                match packet_len(self.partial[1]) {
                    Some(len) => {
                        if self.partial.len() >= len {
                            let packet: Vec<u8> = self.partial.drain(..len).collect();
                            return Ok(Some(packet));
                        }
                    }
                    None => {
                        // Neither a command nor a data frame: resync.
                        self.partial.remove(0);
                        self.note_skip("unknown packet shape after the start byte");
                        continue;
                    }
                }
            }
            if self.partial.len() > MAX_PARTIAL {
                let n = self.partial.len();
                self.partial.clear();
                self.note_skip("buffer full with no complete packet");
                log::warn!(
                    "USB-CAN Analyzer {}: dropped {n} bytes that never formed a packet",
                    self.port_name
                );
            }
            if Instant::now() >= deadline {
                return Ok(None);
            }
            match self.port.read(&mut self.scratch) {
                Ok(0) => continue,
                Ok(n) => self.partial.extend_from_slice(&self.scratch[..n]),
                Err(ref e) if is_timeout(e) => continue,
                Err(e) => return Err(Error::Io(e)),
            }
        }
    }
}

impl CanBus for UsbCanBackend {
    fn send(&mut self, frame: &Frame) -> Result<()> {
        if self.listen_only {
            return Err(Error::Unsupported(
                "this channel was opened listen-only, so it must not transmit",
            ));
        }
        let packet = encode(frame)?;
        self.port.write_all(&packet)?;
        self.port.flush()?;
        Ok(())
    }

    fn recv(&mut self) -> Result<Frame> {
        let deadline = Instant::now() + self.timeout;
        loop {
            match self.next_packet(deadline)? {
                None => return Err(Error::Timeout),
                Some(packet) => match decode(&packet) {
                    Decoded::Data(frame) => return Ok(frame),
                    Decoded::Skip(why) => self.note_skip(why),
                },
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

    fn supports_fd(&self) -> bool {
        false
    }

    fn rx_skipped(&self) -> u64 {
        self.skipped
    }

    fn backend_name(&self) -> &'static str {
        "usbcan"
    }

    fn description(&self) -> String {
        format!(
            "USB-CAN Analyzer {} (CAN, {} bit/s{})",
            self.port_name,
            self.bitrate,
            if self.listen_only { ", silent" } else { "" }
        )
    }
}

/// Why a packet was not turned into a frame, or the frame itself.
enum Decoded {
    Data(Frame),
    Skip(&'static str),
}

/// Low byte of the sum of `data`. Used by the settings command only; data frames
/// carry no checksum.
fn checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, b| acc.wrapping_add(*b))
}

/// The 20-byte settings command: bitrate, mode, no filtering.
fn settings_packet(speed: u8, mode: u8) -> [u8; COMMAND_LEN] {
    let mut p = [0u8; COMMAND_LEN];
    p[0] = PACKET_START;
    p[1] = COMMAND_MARK;
    p[2] = COMMAND_SETTINGS;
    p[3] = speed;
    // The filter frame type with an all-zero filter and mask, which lets
    // everything through — we need standard *and* extended traffic on the same
    // wire (DAMIAO is 11-bit, RobStride is 29-bit).
    p[4] = FILTER_FRAME_EXTENDED;
    // 5..=12 filter and mask ids stay zero.
    p[13] = mode;
    p[14] = 0x01;
    // 15..=18 stay zero.
    p[19] = checksum(&p[2..=18]);
    p
}

/// Total packet length implied by the second byte, or `None` if it is neither
/// packet shape.
fn packet_len(info: u8) -> Option<usize> {
    if info == COMMAND_MARK {
        return Some(COMMAND_LEN);
    }
    if info & DATA_FRAME_MARK != DATA_FRAME_MARK {
        return None;
    }
    let dlc = (info & INFO_DLC) as usize;
    if dlc > 8 {
        return None;
    }
    let id_len = if info & INFO_EXTENDED != 0 { 4 } else { 2 };
    // start + info + id + payload + terminator
    Some(2 + id_len + dlc + 1)
}

/// Build the packet for a classic CAN frame.
fn encode(frame: &Frame) -> Result<Vec<u8>> {
    if frame.is_fd() {
        return Err(Error::Unsupported(
            "CAN-FD frame on a USB-CAN Analyzer, which only carries classic frames",
        ));
    }
    let dlc = frame.len();
    if dlc > 8 {
        return Err(Error::Frame("payload too long for a classic CAN frame"));
    }

    let extended = frame.is_extended();
    let mut packet = Vec::with_capacity(MAX_DATA_PACKET);
    packet.push(PACKET_START);
    let mut info = DATA_FRAME_MARK | dlc as u8;
    if extended {
        info |= INFO_EXTENDED;
    }
    packet.push(info);
    let id = frame.raw_id();
    packet.push((id & 0xFF) as u8);
    packet.push(((id >> 8) & 0xFF) as u8);
    if extended {
        packet.push(((id >> 16) & 0xFF) as u8);
        packet.push(((id >> 24) & 0xFF) as u8);
    }
    packet.extend_from_slice(frame.data());
    packet.push(PACKET_END);
    Ok(packet)
}

/// Turn one received packet into a frame, or say why not.
fn decode(packet: &[u8]) -> Decoded {
    if packet.len() < 2 {
        return Decoded::Skip("packet too short to have a shape");
    }
    let info = packet[1];

    if info == COMMAND_MARK {
        // The adapter echoes commands. Checking the checksum is still worth it:
        // a mismatch means the stream is misaligned, not that a command failed.
        if packet.len() == COMMAND_LEN && packet[19] != checksum(&packet[2..=18]) {
            return Decoded::Skip("command packet with a bad checksum");
        }
        return Decoded::Skip("command packet, not data");
    }

    let Some(expected) = packet_len(info) else {
        return Decoded::Skip("unknown packet shape");
    };
    if packet.len() != expected {
        return Decoded::Skip("packet length disagrees with its info byte");
    }
    // The only integrity check a data frame has. A wrong length — the reference
    // implementation's `+5` for extended frames, say — lands here rather than
    // producing a frame with a truncated id.
    if packet[packet.len() - 1] != PACKET_END {
        return Decoded::Skip("data frame without its 0x55 terminator");
    }
    if info & INFO_REMOTE != 0 {
        return Decoded::Skip("remote-request frame");
    }

    let extended = info & INFO_EXTENDED != 0;
    let dlc = (info & INFO_DLC) as usize;
    let id = if extended {
        u32::from_le_bytes([packet[2], packet[3], packet[4], packet[5]])
    } else {
        u16::from_le_bytes([packet[2], packet[3]]) as u32
    };
    let data_at = if extended { 6 } else { 4 };
    match Frame::from_raw(id, extended, &packet[data_at..data_at + dlc], false, false) {
        Ok(frame) => Decoded::Data(frame),
        // An id too wide for its own frame type: corruption, or a resync that
        // landed on a false start byte.
        Err(_) => Decoded::Skip("id out of range for the stated frame type"),
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

fn is_timeout(e: &std::io::Error) -> bool {
    matches!(
        e.kind(),
        std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip(id: u32, extended: bool, data: &[u8]) -> Frame {
        let frame = Frame::from_raw(id, extended, data, false, false).expect("build");
        let packet = encode(&frame).expect("encode");
        match decode(&packet) {
            Decoded::Data(f) => f,
            Decoded::Skip(why) => panic!("own packet rejected: {why}"),
        }
    }

    /// Pinned byte by byte against `ref/usb-can/canusb.c`'s `send_data_frame`,
    /// because a round trip passes just as happily with two fields transposed.
    #[test]
    fn a_standard_data_frame_matches_the_reference_layout() {
        let frame = Frame::from_raw(0x123, false, &[0xDE, 0xAD], false, false).unwrap();
        let p = encode(&frame).unwrap();
        assert_eq!(p[0], 0xAA, "start");
        // 0xC0 | dlc, standard so bit 5 clear, data so bit 4 clear.
        assert_eq!(p[1], 0xC2, "info byte");
        assert_eq!(&p[2..4], &[0x23, 0x01], "id, low byte first");
        assert_eq!(&p[4..6], &[0xDE, 0xAD], "payload");
        assert_eq!(p[6], 0x55, "terminator");
        assert_eq!(p.len(), 2 + 2 + 2 + 1, "DLC + 5 for a standard frame");
    }

    /// The extended case is where the reference tool is wrong (`DLC + 5` for
    /// every frame), and where RobStride lives.
    #[test]
    fn an_extended_data_frame_is_two_bytes_longer() {
        let frame = Frame::from_raw(0x1234_5678, true, &[1, 2, 3], false, false).unwrap();
        let p = encode(&frame).unwrap();
        assert_eq!(p[1], 0xC0 | 0x20 | 3, "info byte carries the extended bit");
        assert_eq!(&p[2..6], &[0x78, 0x56, 0x34, 0x12], "four id bytes, LSB first");
        assert_eq!(&p[6..9], &[1, 2, 3]);
        assert_eq!(p[9], 0x55);
        assert_eq!(p.len(), 3 + 7, "DLC + 7 for an extended frame");
        assert_eq!(packet_len(p[1]), Some(p.len()));
    }

    #[test]
    fn ids_and_payloads_survive_a_round_trip() {
        let f = roundtrip(0x7FF, false, &[1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(f.raw_id(), 0x7FF);
        assert!(!f.is_extended());
        assert_eq!(f.data(), &[1, 2, 3, 4, 5, 6, 7, 8]);

        let f = roundtrip(0x1FFF_FFFF, true, &[]);
        assert_eq!(f.raw_id(), 0x1FFF_FFFF);
        assert!(f.is_extended());
        assert_eq!(f.data(), &[] as &[u8]);

        // Every DLC, both widths — the length arithmetic is the part most likely
        // to be off by the id width.
        for dlc in 0..=8usize {
            let data: Vec<u8> = (0..dlc as u8).collect();
            assert_eq!(roundtrip(0x555, false, &data).data(), &data[..]);
            assert_eq!(roundtrip(0x1AB_CDEF, true, &data).data(), &data[..]);
        }
    }

    /// A payload may contain the framing bytes; nothing escapes them, so staying
    /// in step is the only protection and the length must come from the info
    /// byte rather than from a search for `0x55`.
    #[test]
    fn framing_bytes_inside_a_payload_do_not_end_the_frame() {
        let f = roundtrip(0x100, false, &[0x55, 0xAA, 0x55, 0xAA]);
        assert_eq!(f.data(), &[0x55, 0xAA, 0x55, 0xAA]);
    }

    #[test]
    fn the_settings_command_matches_the_reference() {
        // 1 Mbit/s, normal mode.
        let p = settings_packet(0x01, MODE_NORMAL);
        assert_eq!(&p[..4], &[0xAA, 0x55, 0x12, 0x01]);
        assert_eq!(p[4], 0x02, "filter frame type");
        assert_eq!(&p[5..13], &[0u8; 8], "no filter, no mask");
        assert_eq!(p[13], 0x00, "normal mode");
        assert_eq!(p[14], 0x01);
        assert_eq!(&p[15..19], &[0u8; 4]);
        assert_eq!(p[19], checksum(&p[2..=18]), "checksum over bytes 2..=18");
        assert_eq!(p.len(), 20);

        // Silent mode is what makes listen-only possible on this adapter.
        assert_eq!(settings_packet(0x01, MODE_SILENT)[13], 0x02);
    }

    #[test]
    fn a_data_frame_without_its_terminator_is_refused() {
        let frame = Frame::from_raw(0x321, false, &[9], false, false).unwrap();
        let mut p = encode(&frame).unwrap();
        *p.last_mut().unwrap() = 0x00;
        assert!(matches!(
            decode(&p),
            Decoded::Skip("data frame without its 0x55 terminator")
        ));
    }

    /// The failure mode of `canusb.c`'s length bug, spelled out: an extended
    /// frame cut to `DLC + 5` must not decode as anything.
    #[test]
    fn an_extended_frame_truncated_the_way_the_reference_would_is_refused() {
        let frame = Frame::from_raw(0x1234_5678, true, &[7, 7, 7], false, false).unwrap();
        let full = encode(&frame).unwrap();
        let short = &full[..3 + 5];
        assert!(matches!(
            decode(short),
            Decoded::Skip("packet length disagrees with its info byte")
        ));
    }

    #[test]
    fn remote_and_unknown_shapes_are_skipped_rather_than_guessed_at() {
        let frame = Frame::from_raw(0x200, false, &[1], false, false).unwrap();

        let mut remote = encode(&frame).unwrap();
        remote[1] |= INFO_REMOTE;
        assert!(matches!(decode(&remote), Decoded::Skip("remote-request frame")));

        // Second byte that is neither 0x55 nor 0b11xxxxxx.
        assert_eq!(packet_len(0x00), None);
        assert_eq!(packet_len(0x80), None);
        // DLC above 8 is not a shape either.
        assert_eq!(packet_len(0xC9), None);
        // ...but every legal DLC is.
        for dlc in 0..=8u8 {
            assert_eq!(packet_len(0xC0 | dlc), Some(dlc as usize + 5));
            assert_eq!(packet_len(0xC0 | INFO_EXTENDED | dlc), Some(dlc as usize + 7));
        }
        assert_eq!(packet_len(COMMAND_MARK), Some(20));
    }

    /// A command echo is not data, and a misaligned one is worth telling apart
    /// from a command that simply arrived.
    #[test]
    fn command_echoes_are_skipped_and_checked() {
        let good = settings_packet(0x01, MODE_NORMAL);
        assert!(matches!(decode(&good), Decoded::Skip("command packet, not data")));

        let mut bad = good;
        bad[19] ^= 0xFF;
        assert!(matches!(
            decode(&bad),
            Decoded::Skip("command packet with a bad checksum")
        ));
    }

    #[test]
    fn an_id_too_wide_for_its_frame_type_is_refused() {
        let frame = Frame::from_raw(0x123, false, &[], false, false).unwrap();
        let mut p = encode(&frame).unwrap();
        p[2] = 0x34;
        p[3] = 0x12; // 0x1234 needs more than 11 bits
        assert!(matches!(
            decode(&p),
            Decoded::Skip("id out of range for the stated frame type")
        ));
    }

    #[test]
    fn fd_frames_are_refused_rather_than_truncated() {
        let fd =
            Frame::new_fd(crate::frame::StandardId::new(0x10).unwrap(), &[0u8; 16], true).unwrap();
        assert!(encode(&fd).is_err());
    }
}
