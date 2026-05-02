//! Protocol layer for the LKMTech V3 servo motor family (MG / MS / RMD-X).
//!
//! This crate is `no_std` and performs no I/O. It encodes request frames into a
//! caller-supplied byte buffer and decodes response frames from a borrowed
//! slice. Wire I/O is the responsibility of the consumer (see the `lkmotor`
//! crate for a `serialport`-backed RS485 driver).
//!
//! # Wire format
//!
//! All frames share the same envelope (LK-Tech V3 RS485 single-motor command):
//!
//! ```text
//! offset  0       1     2     3      4         5..5+N      5+N
//!         +-------+-----+-----+------+---------+-----------+----------+
//!         | 0x3E  | cmd | id  | len  | hdr_sum | data[..N] | data_sum |
//!         +-------+-----+-----+------+---------+-----------+----------+
//! ```
//!
//! - `hdr_sum  = (0x3E + cmd + id + len)         & 0xFF`
//! - `data_sum = (sum of data bytes)             & 0xFF`  (omitted if `len == 0`)
//!
//! Verify command codes against your specific motor's firmware manual before
//! relying on them in production — the codes in [`Command`] follow the widely
//! published V3 protocol but may differ on older or customised firmware.

#![no_std]

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod command;
pub mod frame;
pub mod request;
pub mod response;

pub use command::{Command, ControlParamId};
pub use frame::{
    DecodeError, EncodeError, Frame, HEADER, HEADER_SIZE, MAX_DATA, MAX_FRAME, encode,
    encoded_size, try_decode,
};
