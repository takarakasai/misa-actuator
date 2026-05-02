//! 29-bit CAN ID encoding and decoding.

use crate::comm_type::CommType;

/// Build a 29-bit extended CAN ID.
///
/// Layout: `[comm_type(5)][extra_data(16)][device_id(8)]`.
pub fn build_can_id(comm_type: CommType, extra_data: u16, device_id: u8) -> u32 {
    build_can_id_raw(comm_type as u8, extra_data, device_id)
}

/// Like [`build_can_id`] but takes the raw 5-bit comm type — useful when
/// reconstructing an ID from an already-parsed integer.
pub fn build_can_id_raw(comm_type: u8, extra_data: u16, device_id: u8) -> u32 {
    (((comm_type & 0x1F) as u32) << 24)
        | ((extra_data as u32) << 8)
        | (device_id as u32)
}

/// Decode a 29-bit extended CAN ID into `(comm_type, extra_data, device_id)`.
pub fn parse_can_id(id: u32) -> (u8, u16, u8) {
    let comm_type = ((id >> 24) & 0x1F) as u8;
    let extra_data = ((id >> 8) & 0xFFFF) as u16;
    let device_id = (id & 0xFF) as u8;
    (comm_type, extra_data, device_id)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let id = build_can_id(CommType::OperationControl, 0x7FFF, 0x01);
        assert_eq!(parse_can_id(id), (CommType::OperationControl as u8, 0x7FFF, 0x01));
    }

    #[test]
    fn enable_layout() {
        let id = build_can_id(CommType::Enable, 0xFD, 0x01);
        assert_eq!(id, (3 << 24) | (0xFD << 8) | 1);
    }
}
