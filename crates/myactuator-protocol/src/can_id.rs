//! Standard-CAN id layout for the MyActuator RMD V3 protocol.

/// Highest motor id addressable on one bus (spec: `0x140 + ID(1~32)`).
pub const MAX_MOTOR_ID: u8 = 32;

/// Base id for single-motor commands (`0x140 + ID`).
pub const COMMAND_BASE: u16 = 0x140;

/// Base id for single-motor replies (`0x240 + ID`).
pub const REPLY_BASE: u16 = 0x240;

/// Broadcast id shared by all motors for multi-motor commands.
pub const MULTI_MOTOR_ID: u16 = 0x280;

/// Base id for motion-mode (MIT) commands (`0x400 + ID`).
pub const MOTION_BASE: u16 = 0x400;

/// Base id for motion-mode replies (`0x500 + ID`).
pub const MOTION_REPLY_BASE: u16 = 0x500;

/// `true` when `motor_id` is in the firmware-addressable range 1..=32.
#[inline]
pub const fn is_valid_motor_id(motor_id: u8) -> bool {
    motor_id >= 1 && motor_id <= MAX_MOTOR_ID
}

/// CAN id a single-motor command to `motor_id` is sent on.
#[inline]
pub const fn command_id(motor_id: u8) -> u16 {
    COMMAND_BASE + motor_id as u16
}

/// CAN id `motor_id` replies to single-motor commands on.
#[inline]
pub const fn reply_id(motor_id: u8) -> u16 {
    REPLY_BASE + motor_id as u16
}

/// CAN id a motion-mode command to `motor_id` is sent on.
#[inline]
pub const fn motion_id(motor_id: u8) -> u16 {
    MOTION_BASE + motor_id as u16
}

/// CAN id `motor_id` replies to motion-mode commands on.
#[inline]
pub const fn motion_reply_id(motor_id: u8) -> u16 {
    MOTION_REPLY_BASE + motor_id as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn id_layout_matches_manual_examples() {
        // Manual examples address motor 1 as 0x141 and see replies on 0x241.
        assert_eq!(command_id(1), 0x141);
        assert_eq!(reply_id(1), 0x241);
        assert_eq!(motion_id(1), 0x401);
        assert_eq!(motion_reply_id(1), 0x501);
    }

    #[test]
    fn motor_id_range() {
        assert!(!is_valid_motor_id(0));
        assert!(is_valid_motor_id(1));
        assert!(is_valid_motor_id(32));
        assert!(!is_valid_motor_id(33));
    }
}
