//! Response parsers for status, fault, and parameter-read frames.

use crate::can_id::parse_can_id;
use crate::comm_type::CommType;
use crate::mit::decode_mit_signed;
use crate::model::MitScales;

/// Status bits decoded from the `extra_data` field of an `OperationStatus` frame.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct MotorStatusBits {
    /// Motor mode (2 bits).
    pub mode: u8,
    /// Encoder uncalibrated.
    pub uncalibrated: bool,
    /// Motor stalled.
    pub stall: bool,
    /// Magnetic encoder fault.
    pub magnetic_encoder_fault: bool,
    /// Over-temperature.
    pub overtemperature: bool,
    /// Over-current.
    pub overcurrent: bool,
    /// Under-voltage.
    pub undervoltage: bool,
    /// Device ID echoed in the lower 8 bits of `extra_data`.
    pub device_id: u8,
}

/// Decoded motor feedback frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorFeedback {
    /// Motor CAN ID (decoded from the status bits, not the CAN ID device field).
    pub motor_id: u8,
    /// Position in radians.
    pub position: f32,
    /// Velocity in rad/s.
    pub velocity: f32,
    /// Torque in Nm.
    pub torque: f32,
    /// Motor temperature in °C.
    pub temperature: f32,
    /// Decoded status bits.
    pub status: MotorStatusBits,
}

fn parse_status_bits(extra_data: u16) -> MotorStatusBits {
    MotorStatusBits {
        mode: ((extra_data >> 14) & 0x03) as u8,
        uncalibrated: ((extra_data >> 13) & 0x01) != 0,
        stall: ((extra_data >> 12) & 0x01) != 0,
        magnetic_encoder_fault: ((extra_data >> 11) & 0x01) != 0,
        overtemperature: ((extra_data >> 10) & 0x01) != 0,
        overcurrent: ((extra_data >> 9) & 0x01) != 0,
        undervoltage: ((extra_data >> 8) & 0x01) != 0,
        device_id: (extra_data & 0xFF) as u8,
    }
}

/// Parse an `OperationStatus` (or `FaultReport`) frame into [`MotorFeedback`].
///
/// Returns `None` if the frame's comm type is something else, or the data
/// slice is shorter than 8 bytes.
pub fn parse_status_frame(can_id: u32, data: &[u8], scales: &MitScales) -> Option<MotorFeedback> {
    if data.len() < 8 {
        return None;
    }
    let (comm_type, extra_data, _) = parse_can_id(can_id);
    if comm_type != CommType::OperationStatus as u8 && comm_type != CommType::FaultReport as u8 {
        return None;
    }

    let status = parse_status_bits(extra_data);
    let pos_u = u16::from_be_bytes([data[0], data[1]]);
    let vel_u = u16::from_be_bytes([data[2], data[3]]);
    let trq_u = u16::from_be_bytes([data[4], data[5]]);
    let temp_u = u16::from_be_bytes([data[6], data[7]]);

    Some(MotorFeedback {
        motor_id: status.device_id,
        position: decode_mit_signed(pos_u, scales.position),
        velocity: decode_mit_signed(vel_u, scales.velocity),
        torque: decode_mit_signed(trq_u, scales.torque),
        temperature: temp_u as f32 * 0.1,
        status,
    })
}

/// Parse the payload of a `READ_PARAMETER` response into `(index, value)`.
pub fn parse_param_response(data: &[u8]) -> Option<(u16, f32)> {
    if data.len() < 8 {
        return None;
    }
    let idx = u16::from_le_bytes([data[0], data[1]]);
    let val = f32::from_le_bytes([data[4], data[5], data[6], data[7]]);
    Some((idx, val))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::can_id::build_can_id_raw;
    use crate::model::MotorModel;

    #[test]
    fn status_bits_round_trip() {
        let extra = (1u16 << 8) | 7;
        let bits = parse_status_bits(extra);
        assert!(bits.undervoltage);
        assert_eq!(bits.device_id, 7);
        assert!(!bits.overcurrent);
    }

    #[test]
    fn ignore_non_status_frames() {
        let scales = MitScales::for_model(MotorModel::Rs05);
        let id = build_can_id_raw(CommType::Enable as u8, 0, 1);
        assert!(parse_status_frame(id, &[0u8; 8], &scales).is_none());
    }

    #[test]
    fn parse_status_zero_payload() {
        let scales = MitScales::for_model(MotorModel::Rs05);
        let id = build_can_id_raw(CommType::OperationStatus as u8, 0x0001, 0xFD);
        let payload = [0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x00, 0xFA];
        let fb = parse_status_frame(id, &payload, &scales).unwrap();
        assert_eq!(fb.motor_id, 0x01);
        assert!(fb.position.abs() < 0.01);
        assert!(fb.velocity.abs() < 0.01);
        assert!((fb.temperature - 25.0).abs() < 0.01);
    }
}
