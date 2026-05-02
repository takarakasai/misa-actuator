//! Low-level frame encode/decode for the MG4005 RS485 protocol.

/// Frame start byte.
pub const HEADER: u8 = 0x3E;

/// Bytes consumed by the fixed header (`HEADER`, cmd, id, len, header checksum).
pub const HEADER_SIZE: usize = 5;

/// Maximum data payload size (the `len` field is a single byte).
pub const MAX_DATA: usize = u8::MAX as usize;

/// Upper bound on a fully encoded frame.
pub const MAX_FRAME: usize = HEADER_SIZE + MAX_DATA + 1;

/// Errors returned from [`encode`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// Output buffer is smaller than the encoded frame.
    BufferTooSmall { needed: usize, got: usize },
    /// Data payload exceeds [`MAX_DATA`] bytes.
    DataTooLong { got: usize },
}

/// Errors returned from [`try_decode`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// Input slice does not yet contain a full frame; pull at least `needed` more bytes.
    NeedMore { needed: usize },
    /// First byte is not [`HEADER`].
    BadHeader { found: u8 },
    /// Header checksum failed.
    HeaderChecksum { expected: u8, found: u8 },
    /// Data checksum failed.
    DataChecksum { expected: u8, found: u8 },
}

/// Number of bytes required to encode a frame with `data_len` payload bytes.
pub const fn encoded_size(data_len: usize) -> usize {
    if data_len == 0 {
        HEADER_SIZE
    } else {
        HEADER_SIZE + data_len + 1
    }
}

#[inline]
fn checksum_u8(bytes: &[u8]) -> u8 {
    let mut acc: u8 = 0;
    let mut i = 0;
    while i < bytes.len() {
        acc = acc.wrapping_add(bytes[i]);
        i += 1;
    }
    acc
}

/// Encode a request frame into `out`. Returns the number of bytes written.
///
/// The encoded frame layout matches the diagram in the crate root.
pub fn encode(
    command: u8,
    motor_id: u8,
    data: &[u8],
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    if data.len() > MAX_DATA {
        return Err(EncodeError::DataTooLong { got: data.len() });
    }
    let needed = encoded_size(data.len());
    if out.len() < needed {
        return Err(EncodeError::BufferTooSmall {
            needed,
            got: out.len(),
        });
    }

    let len_byte = data.len() as u8;
    out[0] = HEADER;
    out[1] = command;
    out[2] = motor_id;
    out[3] = len_byte;
    out[4] = HEADER
        .wrapping_add(command)
        .wrapping_add(motor_id)
        .wrapping_add(len_byte);

    if !data.is_empty() {
        out[5..5 + data.len()].copy_from_slice(data);
        out[5 + data.len()] = checksum_u8(data);
    }
    Ok(needed)
}

/// A decoded frame view borrowing from the input buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame<'a> {
    pub command: u8,
    pub motor_id: u8,
    pub data: &'a [u8],
}

/// Try to decode a single frame starting at `input[0]`.
///
/// On success, returns the decoded frame and the number of bytes consumed
/// (so the caller can advance its receive buffer).
pub fn try_decode(input: &[u8]) -> Result<(Frame<'_>, usize), DecodeError> {
    if input.len() < HEADER_SIZE {
        return Err(DecodeError::NeedMore {
            needed: HEADER_SIZE - input.len(),
        });
    }
    if input[0] != HEADER {
        return Err(DecodeError::BadHeader { found: input[0] });
    }

    let command = input[1];
    let motor_id = input[2];
    let len_byte = input[3];
    let data_len = len_byte as usize;
    let header_sum = input[4];
    let expected_header_sum = HEADER
        .wrapping_add(command)
        .wrapping_add(motor_id)
        .wrapping_add(len_byte);
    if header_sum != expected_header_sum {
        return Err(DecodeError::HeaderChecksum {
            expected: expected_header_sum,
            found: header_sum,
        });
    }

    let total = encoded_size(data_len);
    if input.len() < total {
        return Err(DecodeError::NeedMore {
            needed: total - input.len(),
        });
    }

    let data = &input[5..5 + data_len];
    if data_len > 0 {
        let expected_data_sum = checksum_u8(data);
        let found_data_sum = input[5 + data_len];
        if found_data_sum != expected_data_sum {
            return Err(DecodeError::DataChecksum {
                expected: expected_data_sum,
                found: found_data_sum,
            });
        }
    }

    Ok((
        Frame {
            command,
            motor_id,
            data,
        },
        total,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encoded_size_no_payload() {
        assert_eq!(encoded_size(0), HEADER_SIZE);
    }

    #[test]
    fn encoded_size_with_payload() {
        assert_eq!(encoded_size(8), HEADER_SIZE + 8 + 1);
    }

    #[test]
    fn encode_no_payload() {
        let mut buf = [0u8; MAX_FRAME];
        let n = encode(0x80, 0x01, &[], &mut buf).unwrap();
        assert_eq!(n, HEADER_SIZE);
        assert_eq!(&buf[..n], &[0x3E, 0x80, 0x01, 0x00, 0xBF]);
    }

    #[test]
    fn encode_with_payload() {
        let mut buf = [0u8; MAX_FRAME];
        let data = [0x10, 0x20, 0x30, 0x40];
        let n = encode(0xA2, 0x01, &data, &mut buf).unwrap();
        // Header sum = 0x3E + 0xA2 + 0x01 + 0x04 = 0xE5
        // Data sum   = 0x10 + 0x20 + 0x30 + 0x40 = 0xA0
        assert_eq!(
            &buf[..n],
            &[0x3E, 0xA2, 0x01, 0x04, 0xE5, 0x10, 0x20, 0x30, 0x40, 0xA0]
        );
    }

    #[test]
    fn encode_buffer_too_small() {
        let mut buf = [0u8; 4];
        let err = encode(0x80, 0x01, &[], &mut buf).unwrap_err();
        assert_eq!(
            err,
            EncodeError::BufferTooSmall {
                needed: HEADER_SIZE,
                got: 4
            }
        );
    }

    #[test]
    fn round_trip_no_payload() {
        let mut buf = [0u8; MAX_FRAME];
        let n = encode(0x9A, 0x05, &[], &mut buf).unwrap();
        let (frame, used) = try_decode(&buf[..n]).unwrap();
        assert_eq!(used, n);
        assert_eq!(frame.command, 0x9A);
        assert_eq!(frame.motor_id, 0x05);
        assert_eq!(frame.data, &[][..]);
    }

    #[test]
    fn round_trip_with_payload() {
        let mut buf = [0u8; MAX_FRAME];
        let data = [0xDE, 0xAD, 0xBE, 0xEF];
        let n = encode(0xA1, 0x02, &data, &mut buf).unwrap();
        let (frame, used) = try_decode(&buf[..n]).unwrap();
        assert_eq!(used, n);
        assert_eq!(frame.command, 0xA1);
        assert_eq!(frame.motor_id, 0x02);
        assert_eq!(frame.data, &data[..]);
    }

    #[test]
    fn decode_need_more_for_header() {
        let err = try_decode(&[0x3E, 0x80]).unwrap_err();
        assert!(matches!(err, DecodeError::NeedMore { .. }));
    }

    #[test]
    fn decode_need_more_for_payload() {
        // header says len=4, but only 2 payload bytes present
        let buf = [0x3E, 0xA2, 0x01, 0x04, 0xE5, 0x10, 0x20];
        let err = try_decode(&buf).unwrap_err();
        assert!(matches!(err, DecodeError::NeedMore { needed: 3 }));
    }

    #[test]
    fn decode_bad_header() {
        let buf = [0x00, 0x80, 0x01, 0x00, 0xBF];
        let err = try_decode(&buf).unwrap_err();
        assert_eq!(err, DecodeError::BadHeader { found: 0x00 });
    }

    #[test]
    fn decode_header_checksum_mismatch() {
        let buf = [0x3E, 0x80, 0x01, 0x00, 0xFF];
        let err = try_decode(&buf).unwrap_err();
        assert_eq!(
            err,
            DecodeError::HeaderChecksum {
                expected: 0xBF,
                found: 0xFF
            }
        );
    }

    #[test]
    fn decode_data_checksum_mismatch() {
        let buf = [0x3E, 0xA2, 0x01, 0x04, 0xE5, 0x10, 0x20, 0x30, 0x40, 0x00];
        let err = try_decode(&buf).unwrap_err();
        assert_eq!(
            err,
            DecodeError::DataChecksum {
                expected: 0xA0,
                found: 0x00
            }
        );
    }

    #[test]
    fn decode_consumes_only_one_frame() {
        let mut buf = [0u8; MAX_FRAME * 2];
        let n1 = encode(0x80, 0x01, &[], &mut buf).unwrap();
        let n2 = encode(0x81, 0x01, &[], &mut buf[n1..]).unwrap();
        let (_, used) = try_decode(&buf[..n1 + n2]).unwrap();
        assert_eq!(used, n1);
        let (frame2, _) = try_decode(&buf[used..n1 + n2]).unwrap();
        assert_eq!(frame2.command, 0x81);
    }
}
