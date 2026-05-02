//! Protocol layer for the Robstride CAN servo motor family.
//!
//! This crate is `no_std` and performs no I/O. It encodes request frames into
//! the fixed 8-byte payload of an extended (29-bit) CAN frame, and decodes
//! response frames from a borrowed slice. Wire I/O is the responsibility of
//! the consumer (see the `robstride` crate for a `socketcan`-backed driver).
//!
//! # Wire format
//!
//! All Robstride frames are CAN extended (29-bit) frames. The 29-bit ID
//! carries the command type, an auxiliary 16-bit field, and the target
//! device id:
//!
//! ```text
//! bits  28..24       23..8           7..0
//!      +---------+----------------+-----------+
//!      | comm(5) |  extra_data(16)| dev_id(8) |
//!      +---------+----------------+-----------+
//! ```
//!
//! The 8 data bytes carry per-command payloads; for MIT-mode control they
//! are big-endian `[pos_u16, vel_u16, kp_u16, kd_u16]`, and the torque
//! feedforward rides in `extra_data` of the CAN ID itself.
//!
//! # Motor models
//!
//! [`MotorModel`] enumerates the published RS-00..RS-06 family (the Edulite
//! series uses these same motors, e.g. `Edulite05` == [`MotorModel::Rs05`]).
//! Each model has its own MIT scaling table — see [`MitScales::for_model`].
//!
//! Verify command codes and parameter indices against your motor's firmware
//! manual before relying on them in production.

#![no_std]

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(any(test, feature = "alloc"))]
extern crate alloc as _alloc_for_tests;

pub mod can_id;
pub mod comm_type;
pub mod feedback;
pub mod frame;
pub mod mit;
pub mod model;
pub mod param;

pub use can_id::{build_can_id, build_can_id_raw, parse_can_id};
pub use comm_type::{CommType, RunMode};
pub use feedback::{MotorFeedback, MotorStatusBits, parse_param_response, parse_status_frame};
pub use frame::{
    DATA_LEN, build_disable_frame, build_enable_frame, build_mit_frame, build_ping_frame,
    build_read_param_frame, build_run_mode_frame, build_set_zero_frame, build_write_param_f32_frame,
    build_write_param_i8_frame,
};
pub use mit::{decode_mit_signed, decode_mit_unsigned, encode_mit_signed, encode_mit_unsigned};
pub use model::{MitScales, MotorModel};
pub use param::ParamIndex;

/// Default host CAN ID. Robstride recommends this be greater than every
/// motor id on the bus for optimal scheduling on the controller side.
pub const DEFAULT_HOST_ID: u8 = 0xFD;
