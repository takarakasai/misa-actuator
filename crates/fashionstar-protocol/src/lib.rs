//! Protocol layer for FashionStar UART bus servos (the servos inside the
//! Star Arm 102 / reBot Arm 102 leader and follower arms).
//!
//! This crate is `no_std` and performs no I/O. It encodes request frames into
//! a caller-supplied byte buffer and decodes response frames from a borrowed
//! slice. Wire I/O is the responsibility of the consumer (see the
//! `fashionstar-driver` crate for a `serialport`-backed driver).
//!
//! Everything here is in **raw wire units** (0.1°, mV, mA, mW, raw NTC ADC
//! counts). SI conversions need `f32::ln` / rounding, which `core` lacks, so
//! they live in the driver crate.
//!
//! # Sources
//!
//! There is no vendor manual in this repository. The ground truth is the
//! vendor's own Python SDK, read line by line:
//!
//! - `fashionstar-uart-sdk` v1.3.12, `fashionstar_uart_sdk/uservo.py` —
//!   `Packet` (framing, checksum), `PacketBuffer` (receive state machine),
//!   `UartServoManager` (command codes, `struct.pack` layouts of every
//!   request, `struct.unpack` layouts of every response, the NTC formula and
//!   the monitor "invalid" sentinel).
//! - same package, `uart_pocket_handler.py` — `StopOptions` (stop modes) and
//!   the `quick_update` read loop.
//! - Star-Arm-102 `Python_SDK/stararm102_ro.py` — the vendor teleop loop:
//!   broadcast unlock + broadcast multi-turn reset at start-up, then
//!   `send_sync_servo_monitor([0..=6])` per cycle, forwarding the monitor
//!   angle into `SET_SERVO_ANGLE_MTURN_BY_INTERVAL` on the follower.
//! - Seeed's LeRobot teleoperator for the "reBot Arm 102" (= Star Arm 102),
//!   `rebot_arm_102_leader.py` — calibration via `SET_ORIGIN_POINT` and a
//!   host-side unwrap of the multi-turn angle into a window around the
//!   joint range.
//!
//! Byte vectors in the unit tests were derived by hand from those
//! `struct.pack` calls and cross-checked by running the SDK's own
//! `Packet.pack` on the same arguments.
//!
//! # Wire format
//!
//! ```text
//! offset  0    1    2      3     4 .. 4+N     4+N
//!         +----+----+------+-----+-----------+----------+
//!         | h0 | h1 | code | len | params[N] | checksum |
//!         +----+----+------+-----+-----------+----------+
//! ```
//!
//! - request header `12 4C`, response header `05 1C`
//! - `len` = number of parameter bytes (so at most 255)
//! - `checksum = (sum of every preceding byte, header included) mod 256`
//! - all multi-byte fields little-endian
//!
//! The servo id lives **inside** `params` (first byte), not in the envelope,
//! so matching a reply to a request means looking at `code` *and*
//! `params[0]`.
//!
//! Commands that change state (angle, stop, origin, multi-turn reset, ...)
//! are fire-and-forget: the SDK never waits for a reply to them, and has no
//! handler that would accept one. Only the query commands listed in
//! [`Code::has_reply`] answer.

#![no_std]

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod command;
pub mod frame;
pub mod request;
pub mod response;
pub mod stream;

pub use command::{
    status_bits, Code, DataAddress, StopMode, WheelMode, BROADCAST_ID, MTURN_ANGLE_LIMIT,
    MTURN_INTERVAL_MAX_MS, SINGLE_TURN_ANGLE_LIMIT,
};
pub use frame::{
    checksum, encode, encode_response, encoded_size, try_decode, DecodeError, EncodeError,
    Frame, HEADER_REQUEST, HEADER_RESPONSE, MAX_FRAME, MAX_PARAMS, OVERHEAD,
};
pub use request::{
    AngleByInterval, AngleByVelocity, AngleCommand, MturnAngle, MturnAngleByInterval,
    MturnAngleByVelocity, SetAngle,
};
pub use response::{
    parse_angle, parse_angle_mturn, parse_monitor, parse_ping, parse_read_data,
    parse_reset_user_data, parse_write_data, AngleReading, DataRead, Monitor, MultiTurnAngle,
    ParseError, WriteAck, MONITOR_INVALID_ANGLE, MONITOR_PARAMS_LEN,
};
pub use stream::{scan_response, Scan};
#[cfg(feature = "alloc")]
pub use stream::{Response, ResponseParser};
