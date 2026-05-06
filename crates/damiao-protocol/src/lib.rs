//! Pure protocol layer for the DAMIAO (达妙) CAN / CAN-FD servo motor family.
//!
//! This crate is `no_std` and allocation-free: every frame builder returns a
//! fixed `[u8; 8]` payload plus its standard 11-bit CAN id. It knows nothing
//! about the transport — classic CAN and CAN-FD carry the *identical* 8-byte
//! payload, so the driver crate ([`damiao-driver`](../damiao_driver/index.html))
//! picks the socket type while this crate stays bus-agnostic.
//!
//! ## Modules
//!
//! - [`pack`] — fixed-point float ⇄ uint quantization (the SDK formulas).
//! - [`can_id`] — per-mode standard CAN id layout.
//! - [`limits`] — per-model quantization ranges ([`MotorModel`] / [`Limits`]).
//! - [`frame`] — MIT / POS_VEL / VEL / special-command builders and feedback
//!   parsing.
//! - [`register`] — `0x7FF` register read/write/save protocol and
//!   [`ControlMode`].
//! - [`feedback`] — decoded [`Feedback`] and [`ErrorCode`].
//!
//! Protocol constants are taken from the DM-J4310-2EC V1.1 manual and the
//! official DAMIAO SDK (`DM_CAN.py` / `damiao.h`).

#![no_std]

pub mod can_id;
pub mod feedback;
pub mod frame;
pub mod limits;
pub mod pack;
pub mod register;

pub use can_id::{force_pos_id, is_feedback_from, mit_id, pos_vel_id, vel_id, REGISTER_ID};
pub use feedback::{ErrorCode, Feedback};
pub use frame::{
    build_clear_error_frame, build_disable_frame, build_enable_frame, build_mit_frame,
    build_pos_vel_frame, build_set_zero_frame, build_vel_frame, parse_feedback, DATA_LEN,
};
pub use limits::{Limits, MotorModel};
pub use pack::{float_to_uint, uint_to_float};
pub use register::{
    build_read_reg, build_save_all, build_write_reg_f32, build_write_reg_int, parse_reg_reply,
    ControlMode, RegReply, Rid,
};

/// Default Master ID (feedback CAN id) for a factory-fresh DAMIAO motor.
///
/// `MST_ID` is a full 11-bit standard CAN id, so it is represented as a `u16`.
/// The factory default is `0`; the driver treats a master id of `0` as
/// "accept any responder" and disambiguates motors by the id nibble carried in
/// the feedback payload.
pub const DEFAULT_MASTER_ID: u16 = 0;
