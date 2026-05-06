//! CAN ID layout for the DAMIAO protocol.
//!
//! DAMIAO uses **standard 11-bit** CAN identifiers (unlike Robstride's 29-bit
//! extended IDs). Each control mode is addressed by a different ID derived from
//! the motor's configured `CAN_ID` (a.k.a. ESC_ID / slave id):
//!
//! | Mode / channel      | CAN ID            |
//! |---------------------|-------------------|
//! | MIT control         | `CAN_ID`          |
//! | Position-Velocity   | `0x100 + CAN_ID`  |
//! | Velocity            | `0x200 + CAN_ID`  |
//! | Force-Position      | `0x300 + CAN_ID`  |
//! | Register read/write | `0x7FF` (broadcast config channel) |
//!
//! Feedback frames are sent back on the motor's configured **Master ID**
//! (`MST_ID`, default `0`), *not* on the command ID.

/// MIT-mode command channel = the motor's CAN_ID directly.
pub const fn mit_id(can_id: u8) -> u16 {
    can_id as u16
}

/// Position-Velocity command channel.
pub const fn pos_vel_id(can_id: u8) -> u16 {
    0x100 + can_id as u16
}

/// Velocity command channel.
pub const fn vel_id(can_id: u8) -> u16 {
    0x200 + can_id as u16
}

/// Force-Position command channel (newer firmware only).
pub const fn force_pos_id(can_id: u8) -> u16 {
    0x300 + can_id as u16
}

/// Broadcast configuration channel for register read/write/save frames.
pub const REGISTER_ID: u16 = 0x7FF;

/// Whether a received frame's `can_id` matches a motor's Master ID.
///
/// `mst_id` is a full 11-bit standard id. This is an exact match; the
/// "`0` = accept any" policy lives in the driver, which also cross-checks the
/// id nibble in the feedback payload.
pub const fn is_feedback_from(can_id: u16, mst_id: u16) -> bool {
    can_id == mst_id
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn channel_offsets() {
        assert_eq!(mit_id(1), 0x001);
        assert_eq!(pos_vel_id(1), 0x101);
        assert_eq!(vel_id(1), 0x201);
        assert_eq!(force_pos_id(1), 0x301);
        assert_eq!(REGISTER_ID, 0x7FF);
    }

    #[test]
    fn feedback_matches_master_id() {
        assert!(is_feedback_from(0x00, 0)); // default MST_ID
        assert!(is_feedback_from(0x11, 0x11));
        assert!(is_feedback_from(0x305, 0x305)); // 11-bit master id
        assert!(!is_feedback_from(0x01, 0));
    }
}
