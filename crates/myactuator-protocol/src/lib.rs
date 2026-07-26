//! Pure protocol layer for the MyActuator RMD servo motor family
//! (RMD-X / RMD-L, CAN protocol V3).
//!
//! This crate is `no_std` and allocation-free: every frame builder returns a
//! fixed `[u8; 8]` payload, and the CAN ids are computed by the [`can_id`]
//! helpers. It knows nothing about the transport — the driver crate
//! (`myactuator-driver`) owns the SocketCAN plumbing.
//!
//! ## Wire format (V3)
//!
//! - Standard (11-bit) data frames, DLC 8, 1 Mbps.
//! - Single-motor commands go to `0x140 + ID` (ID 1..=32); the motor replies
//!   on `0x240 + ID` with `data[0]` echoing the command byte.
//! - Motion-mode (MIT) commands go to `0x400 + ID`; replies arrive on
//!   `0x500 + ID` (see [`motion`]).
//!
//! ## Units
//!
//! V3 speaks **output-shaft** units on the wire: positions in 0.01°/LSB,
//! speeds in 1 dps or 0.01 dps/LSB, torque as `iq` current in 0.01 A/LSB.
//! This crate keeps those raw integer units; SI conversion lives in the
//! driver.
//!
//! Protocol constants are taken from the MYACTUATOR "Servo Motor Control
//! Protocol" V3.9 manual.

#![no_std]

pub mod can_id;
pub mod feedback;
pub mod frame;
pub mod motion;

pub use can_id::{
    command_id, is_valid_motor_id, motion_id, motion_reply_id, reply_id, MAX_MOTOR_ID,
    MULTI_MOTOR_ID,
};
pub use feedback::{
    ErrorState, PidGains, PidIndex, RunMode, SingleTurnEncoder, Status1, Status2, Status3,
};
pub use frame::{
    build_brake_lock, build_brake_release, build_function_control, build_position_control,
    build_read_acceleration, build_read_motor_model, build_read_motor_power,
    build_read_multi_turn_angle, build_read_multi_turn_encoder, build_read_multi_turn_encoder_raw,
    build_read_multi_turn_zero_offset, build_read_pid, build_read_run_mode,
    build_read_single_turn_angle, build_read_single_turn_encoder, build_read_status1,
    build_read_status2, build_read_status3, build_read_uptime, build_read_version_date,
    build_set_zero_rom, build_shutdown, build_speed_control, build_stop, build_system_reset,
    build_torque_control, parse_acceleration, parse_motor_model, parse_motor_power,
    parse_multi_turn_angle, parse_multi_turn_encoder, parse_multi_turn_encoder_raw,
    parse_multi_turn_zero_offset, parse_pid_value, parse_run_mode, parse_single_turn_angle,
    parse_single_turn_encoder, parse_status1, parse_status2, parse_status3, parse_uptime,
    parse_version_date, AccelIndex, Cmd, DATA_LEN,
};
pub use motion::{
    build_motion_control, parse_motion_reply, MotionFeedback, KD_MAX, KP_MAX, P_MAX, T_MAX, V_MAX,
};
