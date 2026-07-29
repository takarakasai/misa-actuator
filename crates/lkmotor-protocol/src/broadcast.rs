//! RS485 broadcast-mode frame encoding (up to 4 motors per frame).
//!
//! This is a distinct wire format from the per-motor [`crate::frame`]
//! module: `Head(0x02) | CMD | data[8] | checksum`, always 11 bytes, with a
//! single trailing checksum covering every prior byte (not the separate
//! header/data checksum split the addressed protocol uses). See
//! `ref/rs485_broadcast_protocol_en.md`.
//!
//! The CAN-bus variant of broadcast mode (frame IDs `0x280`/`0x281`/`0x282`/
//! `0x288`, see `ref/can_broadcast_protocol_en.md`) carries the identical
//! 8-byte data layout directly as a CAN frame's DLC=8 payload, with no
//! Head/CMD/checksum bytes — the CAN frame ID selects the command and CAN's
//! own frame integrity replaces the checksum byte. This module only encodes
//! the RS485 form; no CAN transport exists for this driver in this codebase.
//!
//! Replies are ordinary single-motor frames (per [`crate::frame`]), one per
//! responding motor, sent back in ascending motor-ID order — decode them
//! with [`crate::frame::try_decode`] as usual.

use crate::frame::EncodeError;

/// RS485 broadcast frame header byte (distinct from the per-motor `0x3E`).
pub const BROADCAST_HEAD: u8 = 0x02;

/// Total encoded length of every broadcast frame (fixed, no variable data).
pub const BROADCAST_FRAME_LEN: usize = 11;

/// CMD byte selecting the broadcast command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BroadcastCommand {
    /// Torque / open-loop control (`torqueValue`, `i16`, per-motor slot).
    Torque = 0x80,
    /// Speed control (`speedValue`, `i16` dps, per-motor slot).
    Speed = 0x81,
    /// Absolute position control (`angleValue`, `i16`, 0.01 deg/LSB, per-motor slot).
    Position = 0x82,
    /// Hybrid — a distinct single-motor command byte per motor slot.
    Hybrid = 0x88,
}

impl BroadcastCommand {
    /// Wire byte for this broadcast command.
    #[inline]
    pub const fn code(self) -> u8 {
        self as u8
    }
}

#[inline]
fn checksum(bytes: &[u8]) -> u8 {
    let mut acc: u8 = 0;
    let mut i = 0;
    while i < bytes.len() {
        acc = acc.wrapping_add(bytes[i]);
        i += 1;
    }
    acc
}

/// Encode a Torque/Speed/Position broadcast command: one `i16` slot per
/// motor (index 0 = motor #1 .. index 3 = motor #4), little-endian.
///
/// Broadcast mode always drives every listed slot — there is no "leave
/// unchanged" value, so a motor that should not move must be sent `0`
/// explicitly (matching the manual's own worked examples).
pub fn encode_broadcast_values(
    cmd: BroadcastCommand,
    values: [i16; 4],
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    if out.len() < BROADCAST_FRAME_LEN {
        return Err(EncodeError::BufferTooSmall {
            needed: BROADCAST_FRAME_LEN,
            got: out.len(),
        });
    }
    out[0] = BROADCAST_HEAD;
    out[1] = cmd.code();
    for (i, v) in values.iter().enumerate() {
        out[2 + i * 2..4 + i * 2].copy_from_slice(&v.to_le_bytes());
    }
    out[10] = checksum(&out[0..10]);
    Ok(BROADCAST_FRAME_LEN)
}

/// Encode a Hybrid broadcast command: one single-motor command byte per
/// motor slot (index 0 = motor #1 .. index 3 = motor #4); the odd byte of
/// each slot is fixed padding (`0x00`, per the manual).
pub fn encode_broadcast_hybrid(motor_cmds: [u8; 4], out: &mut [u8]) -> Result<usize, EncodeError> {
    if out.len() < BROADCAST_FRAME_LEN {
        return Err(EncodeError::BufferTooSmall {
            needed: BROADCAST_FRAME_LEN,
            got: out.len(),
        });
    }
    out[0] = BROADCAST_HEAD;
    out[1] = BroadcastCommand::Hybrid.code();
    for (i, &c) in motor_cmds.iter().enumerate() {
        out[2 + i * 2] = c;
        out[3 + i * 2] = 0x00;
    }
    out[10] = checksum(&out[0..10]);
    Ok(BROADCAST_FRAME_LEN)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Manual worked example: torque 100 -> motor #1, torque -100 -> motor #3.
    #[test]
    fn torque_matches_manual_example() {
        let mut buf = [0u8; BROADCAST_FRAME_LEN];
        let n = encode_broadcast_values(BroadcastCommand::Torque, [100, 0, -100, 0], &mut buf)
            .unwrap();
        assert_eq!(n, BROADCAST_FRAME_LEN);
        assert_eq!(
            buf,
            [0x02, 0x80, 0x64, 0x00, 0x00, 0x00, 0x9C, 0xFF, 0x00, 0x00, 0x81]
        );
    }

    /// Manual worked example: speed 360 dps -> motor #2, speed -720 dps -> motor #4.
    #[test]
    fn speed_matches_manual_example() {
        let mut buf = [0u8; BROADCAST_FRAME_LEN];
        let n =
            encode_broadcast_values(BroadcastCommand::Speed, [0, 360, 0, -720], &mut buf).unwrap();
        assert_eq!(n, BROADCAST_FRAME_LEN);
        assert_eq!(
            buf,
            [0x02, 0x81, 0x00, 0x00, 0x68, 0x01, 0x00, 0x00, 0x30, 0xFD, 0x19]
        );
    }

    /// Manual worked example: angle 180deg -> motor #1, angle -90deg -> motor #4.
    #[test]
    fn position_matches_manual_example() {
        let mut buf = [0u8; BROADCAST_FRAME_LEN];
        let n = encode_broadcast_values(
            BroadcastCommand::Position,
            [18_000, 0, 0, -9_000],
            &mut buf,
        )
        .unwrap();
        assert_eq!(n, BROADCAST_FRAME_LEN);
        assert_eq!(
            buf,
            [0x02, 0x82, 0x50, 0x46, 0x00, 0x00, 0x00, 0x00, 0xD8, 0xDC, 0xCE]
        );
    }

    /// Manual worked example: "read status 2" -> motor #1, "stop" -> motor #4.
    ///
    /// **Note**: the manual's own worked example gives checksum `0xCF`, but
    /// recomputing `head + CMD + data[0..7]` (low byte of the sum) over its
    /// own listed bytes yields `0xA7` — a typo in the source document (see
    /// `ref/rs485_broadcast_protocol_en.md`). This test asserts the correct,
    /// algorithm-derived checksum rather than the vendor's mistyped one.
    #[test]
    fn hybrid_matches_manual_example_with_corrected_checksum() {
        let mut buf = [0u8; BROADCAST_FRAME_LEN];
        let n = encode_broadcast_hybrid([0x9C, 0, 0, 0x81], &mut buf).unwrap();
        assert_eq!(n, BROADCAST_FRAME_LEN);
        assert_eq!(
            buf,
            [0x02, 0x88, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0x00, 0xA7]
        );
    }

    #[test]
    fn buffer_too_small() {
        let mut buf = [0u8; 3];
        let err =
            encode_broadcast_values(BroadcastCommand::Torque, [0, 0, 0, 0], &mut buf).unwrap_err();
        assert_eq!(
            err,
            EncodeError::BufferTooSmall {
                needed: BROADCAST_FRAME_LEN,
                got: 3
            }
        );
    }
}
