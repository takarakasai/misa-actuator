//! Low-level frame encode/decode (SDK `Packet` class in `uservo.py`).

/// Header of every host → servo frame (`Packet.HEADERS[PKT_TYPE_REQUEST]`).
pub const HEADER_REQUEST: [u8; 2] = [0x12, 0x4C];

/// Header of every servo → host frame (`Packet.HEADERS[PKT_TYPE_RESPONSE]`).
///
/// Using a different header per direction is what lets the receiver ignore
/// an echo of its own request on a half-duplex adapter: the echo starts with
/// `12 4C` and is simply skipped as garbage.
pub const HEADER_RESPONSE: [u8; 2] = [0x05, 0x1C];

/// Bytes of framing around the params: header(2) + code + len + checksum.
pub const OVERHEAD: usize = 5;

/// Maximum parameter length (the `len` field is one byte).
pub const MAX_PARAMS: usize = u8::MAX as usize;

/// Upper bound on a fully encoded frame.
pub const MAX_FRAME: usize = OVERHEAD + MAX_PARAMS;

/// Errors returned from the encoders.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// Output buffer is smaller than the encoded frame.
    BufferTooSmall { needed: usize, got: usize },
    /// Params exceed [`MAX_PARAMS`] bytes (e.g. a sync command for too many
    /// servos).
    ParamsTooLong { got: usize },
}

/// Errors returned from [`try_decode`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// Input does not yet contain a full frame; at least `needed` more bytes.
    NeedMore { needed: usize },
    /// First two bytes are not the expected header.
    BadHeader { found: [u8; 2] },
    /// Checksum mismatch.
    Checksum { expected: u8, found: u8 },
}

/// Size of a frame with `params_len` parameter bytes.
#[inline]
pub const fn encoded_size(params_len: usize) -> usize {
    OVERHEAD + params_len
}

/// `sum(bytes) mod 256` — `Packet.calc_checksum`, which sums the header,
/// code, size and params (everything before the checksum byte).
#[inline]
pub fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
}

fn encode_with_header(
    header: [u8; 2],
    code: u8,
    params: &[u8],
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    if params.len() > MAX_PARAMS {
        return Err(EncodeError::ParamsTooLong { got: params.len() });
    }
    let needed = encoded_size(params.len());
    if out.len() < needed {
        return Err(EncodeError::BufferTooSmall {
            needed,
            got: out.len(),
        });
    }
    out[0] = header[0];
    out[1] = header[1];
    out[2] = code;
    out[3] = params.len() as u8;
    out[4..4 + params.len()].copy_from_slice(params);
    out[needed - 1] = checksum(&out[..needed - 1]);
    Ok(needed)
}

/// Encode a request frame (`Packet.pack`). Returns the bytes written.
pub fn encode(code: u8, params: &[u8], out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_with_header(HEADER_REQUEST, code, params, out)
}

/// Encode a *response* frame, as a servo would send it.
///
/// Not needed to talk to a servo — it exists so drivers and applications can
/// build canned replies for tests and simulators without re-deriving the
/// checksum by hand.
pub fn encode_response(code: u8, params: &[u8], out: &mut [u8]) -> Result<usize, EncodeError> {
    encode_with_header(HEADER_RESPONSE, code, params, out)
}

/// A decoded frame borrowing its params from the input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame<'a> {
    pub code: u8,
    pub params: &'a [u8],
}

impl Frame<'_> {
    /// First param byte, which is the servo id for every command that
    /// carries one.
    #[inline]
    pub fn servo_id(&self) -> Option<u8> {
        self.params.first().copied()
    }
}

/// Decode one frame that starts exactly at `input[0]` with `header`.
///
/// On success returns the frame and the number of bytes consumed. Use
/// [`crate::stream::scan_response`] instead when the input may start with
/// garbage or hold several frames.
pub fn try_decode(header: [u8; 2], input: &[u8]) -> Result<(Frame<'_>, usize), DecodeError> {
    // The header is checked byte by byte so a lone correct first byte asks
    // for more data rather than failing — a stream reader may have received
    // only half the header so far.
    if let Some(&b0) = input.first() {
        if b0 != header[0] {
            return Err(DecodeError::BadHeader {
                found: [b0, input.get(1).copied().unwrap_or(0)],
            });
        }
    }
    if let Some(&b1) = input.get(1) {
        if b1 != header[1] {
            return Err(DecodeError::BadHeader {
                found: [input[0], b1],
            });
        }
    }
    if input.len() < 4 {
        return Err(DecodeError::NeedMore {
            needed: 4 - input.len(),
        });
    }
    let total = encoded_size(input[3] as usize);
    if input.len() < total {
        return Err(DecodeError::NeedMore {
            needed: total - input.len(),
        });
    }
    let expected = checksum(&input[..total - 1]);
    let found = input[total - 1];
    if expected != found {
        return Err(DecodeError::Checksum { expected, found });
    }
    Ok((
        Frame {
            code: input[2],
            params: &input[4..total - 1],
        },
        total,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ping_request_bytes() {
        // uservo.py: send_request(CODE_PING, struct.pack('<B', servo_id))
        // Packet.pack: 12 4C | code=01 | size=01 | params=00 | checksum
        // checksum = 0x12 + 0x4C + 0x01 + 0x01 + 0x00 = 0x60
        let mut buf = [0u8; MAX_FRAME];
        let n = encode(1, &[0x00], &mut buf).unwrap();
        assert_eq!(&buf[..n], &[0x12, 0x4C, 0x01, 0x01, 0x00, 0x60]);
    }

    #[test]
    fn empty_params_frame() {
        // begin_async: send_request(CODE_BEGIN_ASYNC, b'')
        // 12 4C 12 00 | checksum = 0x12 + 0x4C + 0x12 + 0x00 = 0x70
        let mut buf = [0u8; MAX_FRAME];
        let n = encode(18, &[], &mut buf).unwrap();
        assert_eq!(&buf[..n], &[0x12, 0x4C, 0x12, 0x00, 0x70]);
    }

    #[test]
    fn checksum_wraps() {
        // The header is included in the sum, so it wraps with modest params:
        // stop_on_control_mode(0xFF, 0x10, 0) = 12 4C 18 04 FF 10 00 00
        // 0x12+0x4C+0x18+0x04+0xFF+0x10 = 0x189 -> 0x89
        let mut buf = [0u8; MAX_FRAME];
        let n = encode(24, &[0xFF, 0x10, 0x00, 0x00], &mut buf).unwrap();
        assert_eq!(buf[n - 1], 0x89);
    }

    #[test]
    fn response_round_trip() {
        // A ping reply for id 3: 05 1C 01 01 03 | 0x05+0x1C+0x01+0x01+0x03 = 0x26
        let mut buf = [0u8; MAX_FRAME];
        let n = encode_response(1, &[3], &mut buf).unwrap();
        assert_eq!(&buf[..n], &[0x05, 0x1C, 0x01, 0x01, 0x03, 0x26]);
        let (f, used) = try_decode(HEADER_RESPONSE, &buf[..n]).unwrap();
        assert_eq!(used, n);
        assert_eq!(f.code, 1);
        assert_eq!(f.params, &[3]);
        assert_eq!(f.servo_id(), Some(3));
    }

    #[test]
    fn decode_partial_header_needs_more() {
        assert_eq!(
            try_decode(HEADER_RESPONSE, &[0x05]),
            Err(DecodeError::NeedMore { needed: 3 })
        );
        assert_eq!(
            try_decode(HEADER_RESPONSE, &[]),
            Err(DecodeError::NeedMore { needed: 4 })
        );
    }

    #[test]
    fn decode_rejects_request_header() {
        let err = try_decode(HEADER_RESPONSE, &[0x12, 0x4C, 0x01, 0x01, 0x00, 0x60]).unwrap_err();
        assert!(matches!(err, DecodeError::BadHeader { .. }));
    }

    #[test]
    fn decode_needs_params() {
        let err = try_decode(HEADER_RESPONSE, &[0x05, 0x1C, 0x16, 0x10, 0x02]).unwrap_err();
        assert_eq!(err, DecodeError::NeedMore { needed: 16 });
    }

    #[test]
    fn decode_checksum_mismatch() {
        let err = try_decode(HEADER_RESPONSE, &[0x05, 0x1C, 0x01, 0x01, 0x03, 0x27]).unwrap_err();
        assert_eq!(
            err,
            DecodeError::Checksum {
                expected: 0x26,
                found: 0x27
            }
        );
    }

    #[test]
    fn encode_errors() {
        let mut small = [0u8; 5];
        assert_eq!(
            encode(1, &[0], &mut small),
            Err(EncodeError::BufferTooSmall { needed: 6, got: 5 })
        );
        let big = [0u8; 256];
        let mut buf = [0u8; 300];
        assert_eq!(
            encode(25, &big, &mut buf),
            Err(EncodeError::ParamsTooLong { got: 256 })
        );
    }
}
