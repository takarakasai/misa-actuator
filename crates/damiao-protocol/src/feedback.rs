//! Decoded DAMIAO feedback frame and error/status code.

/// Decoded motor feedback (one reply frame).
///
/// All quantities are in motor output-frame SI units (rad, rad/s, N·m). The
/// two temperatures are reported as raw integer °C by the firmware.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Feedback {
    /// Motor id echoed in the low nibble of byte 0.
    pub motor_id: u8,
    /// Position in **rad**.
    pub position: f32,
    /// Velocity in **rad/s**.
    pub velocity: f32,
    /// Torque in **N·m**.
    pub torque: f32,
    /// MOSFET (driver) average temperature in **°C**.
    pub t_mos: f32,
    /// Rotor / coil average temperature in **°C**.
    pub t_rotor: f32,
    /// Decoded controller status / error code.
    pub err: ErrorCode,
}

/// The status/error code carried in the high nibble of feedback byte 0.
///
/// `0x0`/`0x1` are normal (disabled / enabled); `0x8`..=`0xE` are faults.
/// Values are taken from the DM-J4310-2EC manual and the official SDK.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorCode {
    /// `0x0` — motor disabled (closed-loop off).
    Disabled,
    /// `0x1` — motor enabled, running normally.
    Enabled,
    /// `0x8` — over-voltage.
    OverVoltage,
    /// `0x9` — under-voltage.
    UnderVoltage,
    /// `0xA` — over-current.
    OverCurrent,
    /// `0xB` — MOSFET over-temperature.
    MosOverTemp,
    /// `0xC` — coil / rotor over-temperature.
    CoilOverTemp,
    /// `0xD` — communication lost (watchdog auto-disable).
    CommLost,
    /// `0xE` — overload.
    Overload,
    /// Any code not in the table above.
    Unknown(u8),
}

impl ErrorCode {
    /// Decode the 4-bit status nibble.
    pub const fn from_nibble(n: u8) -> Self {
        match n & 0x0F {
            0x0 => ErrorCode::Disabled,
            0x1 => ErrorCode::Enabled,
            0x8 => ErrorCode::OverVoltage,
            0x9 => ErrorCode::UnderVoltage,
            0xA => ErrorCode::OverCurrent,
            0xB => ErrorCode::MosOverTemp,
            0xC => ErrorCode::CoilOverTemp,
            0xD => ErrorCode::CommLost,
            0xE => ErrorCode::Overload,
            other => ErrorCode::Unknown(other),
        }
    }

    /// The raw nibble value.
    pub const fn raw(self) -> u8 {
        match self {
            ErrorCode::Disabled => 0x0,
            ErrorCode::Enabled => 0x1,
            ErrorCode::OverVoltage => 0x8,
            ErrorCode::UnderVoltage => 0x9,
            ErrorCode::OverCurrent => 0xA,
            ErrorCode::MosOverTemp => 0xB,
            ErrorCode::CoilOverTemp => 0xC,
            ErrorCode::CommLost => 0xD,
            ErrorCode::Overload => 0xE,
            ErrorCode::Unknown(v) => v,
        }
    }

    /// `true` for `0x8`..=`0xE` (an actual fault, not the normal
    /// disabled/enabled states).
    pub const fn is_fault(self) -> bool {
        matches!(
            self,
            ErrorCode::OverVoltage
                | ErrorCode::UnderVoltage
                | ErrorCode::OverCurrent
                | ErrorCode::MosOverTemp
                | ErrorCode::CoilOverTemp
                | ErrorCode::CommLost
                | ErrorCode::Overload
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nibble_round_trip() {
        for n in 0u8..=0xF {
            let c = ErrorCode::from_nibble(n);
            assert_eq!(c.raw(), n, "nibble {n:#X}");
        }
    }

    #[test]
    fn fault_classification() {
        assert!(!ErrorCode::Disabled.is_fault());
        assert!(!ErrorCode::Enabled.is_fault());
        assert!(ErrorCode::CommLost.is_fault());
        assert!(ErrorCode::Overload.is_fault());
    }
}
