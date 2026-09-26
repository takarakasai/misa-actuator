//! Streaming response scanner.
//!
//! A serial read returns whatever bytes have arrived: half a frame, three
//! frames back to back (a sync monitor answers with one frame per servo),
//! or garbage — an echo of our own request on a half-duplex adapter, line
//! noise, the tail of a reply to an earlier request that timed out.
//! [`scan_response`] finds the next valid response frame in such a buffer and
//! says how much of the buffer before it can be thrown away.
//!
//! # Resync policy
//!
//! The SDK's `PacketBuffer` hunts for the `05 1C` header, commits to it, and
//! on a checksum failure throws the whole candidate away. That has two weak
//! spots, both fixed here:
//!
//! - After a bad checksum it restarts *after* the bogus frame, so a real
//!   header that happened to sit inside it is lost. Here the search resumes
//!   one byte after the false header.
//! - A false `05 1C` whose "length" byte is large makes it wait for bytes
//!   that belong to the next real frames. Here, while a candidate is
//!   incomplete, the rest of the buffer is also searched: if a later header
//!   already yields a complete valid frame, the incomplete candidate is
//!   treated as garbage. A genuine frame that is still arriving is only
//!   skipped if its own payload contains a valid-checksum frame, which for
//!   16-byte monitor payloads is on the order of one in a million.

use crate::frame::{try_decode, DecodeError, Frame, HEADER_RESPONSE};

/// Result of [`scan_response`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scan<'a> {
    /// A valid frame occupies `input[start..end]`. Everything before `start`
    /// is garbage; drop `..end` once the frame has been handled.
    Frame {
        frame: Frame<'a>,
        start: usize,
        end: usize,
    },
    /// No complete frame yet. `input[..discard]` can never become part of
    /// one and may be dropped; keep the rest and read more.
    NeedMore { discard: usize },
}

/// Position of the next response header at or after `from`. A lone `05` as
/// the very last byte also counts — its `1C` may be the next byte to arrive.
fn find_header(input: &[u8], from: usize) -> Option<usize> {
    let mut i = from;
    while i < input.len() {
        if input[i] == HEADER_RESPONSE[0]
            && (i + 1 == input.len() || input[i + 1] == HEADER_RESPONSE[1])
        {
            return Some(i);
        }
        i += 1;
    }
    None
}

/// Find the next complete, checksum-valid response frame in `input`.
pub fn scan_response(input: &[u8]) -> Scan<'_> {
    let mut from = 0;
    loop {
        let Some(p) = find_header(input, from) else {
            return Scan::NeedMore {
                discard: input.len(),
            };
        };
        match try_decode(HEADER_RESPONSE, &input[p..]) {
            Ok((frame, used)) => {
                return Scan::Frame {
                    frame,
                    start: p,
                    end: p + used,
                }
            }
            Err(DecodeError::NeedMore { .. }) => {
                // See the module docs: prefer a complete frame further on
                // over waiting on a candidate that may be a false header.
                if let Some(later) = complete_frame_after(input, p + 1) {
                    return later;
                }
                return Scan::NeedMore { discard: p };
            }
            Err(DecodeError::BadHeader { .. }) | Err(DecodeError::Checksum { .. }) => {
                from = p + 1;
            }
        }
    }
}

/// The first complete, valid frame starting at or after `from`, if any.
fn complete_frame_after(input: &[u8], mut from: usize) -> Option<Scan<'_>> {
    while let Some(p) = find_header(input, from) {
        if let Ok((frame, used)) = try_decode(HEADER_RESPONSE, &input[p..]) {
            return Some(Scan::Frame {
                frame,
                start: p,
                end: p + used,
            });
        }
        from = p + 1;
    }
    None
}

/// An owned response frame.
#[cfg(feature = "alloc")]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Response {
    pub code: u8,
    pub params: alloc::vec::Vec<u8>,
}

#[cfg(feature = "alloc")]
impl Response {
    /// First param byte — the servo id for every reply this crate parses.
    pub fn servo_id(&self) -> Option<u8> {
        self.params.first().copied()
    }
}

/// Byte-stream accumulator around [`scan_response`]: push whatever the port
/// returned, pop frames until `None`.
#[cfg(feature = "alloc")]
#[derive(Debug, Default)]
pub struct ResponseParser {
    buf: alloc::vec::Vec<u8>,
    discarded: usize,
}

#[cfg(feature = "alloc")]
impl ResponseParser {
    pub fn new() -> Self {
        Self::default()
    }

    /// Append received bytes.
    pub fn push(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    /// Next complete frame, dropping any garbage in front of it.
    pub fn next_frame(&mut self) -> Option<Response> {
        match scan_response(&self.buf) {
            Scan::Frame { frame, start, end } => {
                let resp = Response {
                    code: frame.code,
                    params: frame.params.to_vec(),
                };
                self.discarded += start;
                self.buf.drain(..end);
                Some(resp)
            }
            Scan::NeedMore { discard } => {
                self.discarded += discard;
                self.buf.drain(..discard);
                None
            }
        }
    }

    /// Bytes still buffered (a partial frame, usually).
    pub fn pending(&self) -> usize {
        self.buf.len()
    }

    /// Total garbage bytes dropped so far — a cheap line-quality indicator.
    pub fn discarded(&self) -> usize {
        self.discarded
    }

    /// Forget everything buffered.
    pub fn clear(&mut self) {
        self.buf.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ping reply id 3 and a query-angle reply, both derived in response.rs.
    const PING3: [u8; 6] = [0x05, 0x1C, 0x01, 0x01, 0x03, 0x26];
    const ANGLE3: [u8; 8] = [0x05, 0x1C, 0x0A, 0x03, 0x03, 0x2E, 0xFB, 0x5A];

    fn frames(input: &[u8]) -> ([usize; 8], usize) {
        // Walk all frames with the no_std API, returning their start offsets.
        let mut starts = [0usize; 8];
        let mut n = 0;
        let mut base = 0;
        while let Scan::Frame { start, end, .. } = scan_response(&input[base..]) {
            starts[n] = base + start;
            n += 1;
            base += end;
        }
        (starts, n)
    }

    #[test]
    fn back_to_back() {
        let mut buf = [0u8; 14];
        buf[..6].copy_from_slice(&PING3);
        buf[6..].copy_from_slice(&ANGLE3);
        let (starts, n) = frames(&buf);
        assert_eq!(n, 2);
        assert_eq!(&starts[..2], &[0, 6]);
    }

    #[test]
    fn leading_garbage_and_echo() {
        // An echoed request (12 4C ...) followed by noise, then a reply.
        let mut buf = [0u8; 6 + 3 + 6];
        buf[..6].copy_from_slice(&[0x12, 0x4C, 0x01, 0x01, 0x03, 0x63]);
        buf[6..9].copy_from_slice(&[0xAA, 0x05, 0x00]);
        buf[9..].copy_from_slice(&PING3);
        match scan_response(&buf) {
            Scan::Frame { frame, start, .. } => {
                assert_eq!(start, 9);
                assert_eq!(frame.params, &[3]);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn corrupted_frame_then_good_one() {
        let mut buf = [0u8; 12];
        buf[..6].copy_from_slice(&PING3);
        buf[5] ^= 0xFF; // break its checksum
        buf[6..].copy_from_slice(&PING3);
        let (starts, n) = frames(&buf);
        assert_eq!(n, 1);
        assert_eq!(starts[0], 6);
    }

    #[test]
    fn false_header_with_huge_length_does_not_stall() {
        // `05 1C 16 FF` claims 255 params; the real frame follows at once.
        let mut buf = [0u8; 4 + 6];
        buf[..4].copy_from_slice(&[0x05, 0x1C, 0x16, 0xFF]);
        buf[4..].copy_from_slice(&PING3);
        match scan_response(&buf) {
            Scan::Frame { start, .. } => assert_eq!(start, 4),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn partial_frame_keeps_its_bytes() {
        assert_eq!(scan_response(&PING3[..4]), Scan::NeedMore { discard: 0 });
        // A lone trailing 0x05 may be the start of a header.
        assert_eq!(scan_response(&[0xAA, 0xBB, 0x05]), Scan::NeedMore { discard: 2 });
        assert_eq!(scan_response(&[0xAA, 0xBB]), Scan::NeedMore { discard: 2 });
        assert_eq!(scan_response(&[]), Scan::NeedMore { discard: 0 });
    }

    #[cfg(feature = "alloc")]
    #[test]
    fn parser_byte_at_a_time() {
        let mut p = ResponseParser::new();
        let mut stream = alloc::vec::Vec::new();
        stream.extend_from_slice(&[0x00, 0x05, 0x05]);
        stream.extend_from_slice(&PING3);
        stream.extend_from_slice(&ANGLE3);
        let mut got = alloc::vec::Vec::new();
        for b in &stream {
            p.push(core::slice::from_ref(b));
            while let Some(r) = p.next_frame() {
                got.push(r);
            }
        }
        assert_eq!(got.len(), 2);
        assert_eq!(got[0].code, 1);
        assert_eq!(got[1].code, 10);
        assert_eq!(p.discarded(), 3);
        assert_eq!(p.pending(), 0);
    }
}
