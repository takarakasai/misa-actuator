//! Communication-type and run-mode enums used throughout the Robstride
//! protocol.

/// Communication type encoded in bits 28..=24 of the 29-bit CAN ID.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CommType {
    /// Get device ID and 64-bit MCU unique identifier.
    GetDeviceId = 0,
    /// MIT mode operation control (pos/vel/kp/kd in data, torque in extra_data).
    OperationControl = 1,
    /// Motor status feedback frame.
    OperationStatus = 2,
    /// Enable the motor.
    Enable = 3,
    /// Disable the motor.
    Disable = 4,
    /// Set the current position as the mechanical zero.
    SetZeroPosition = 6,
    /// Set the device CAN ID.
    SetDeviceId = 7,
    /// Read a parameter.
    ReadParameter = 17,
    /// Write a parameter.
    WriteParameter = 18,
    /// Fault report feedback.
    FaultReport = 21,
    /// Save all parameters to flash.
    SaveParameters = 22,
    /// Set CAN baudrate.
    SetBaudrate = 23,
    /// Motor active report.
    ActiveReport = 24,
    /// Set the protocol type.
    SetProtocol = 25,
}

impl CommType {
    /// Try to convert a 5-bit raw comm type back into the enum.
    pub fn from_u8(value: u8) -> Option<Self> {
        Some(match value {
            0 => Self::GetDeviceId,
            1 => Self::OperationControl,
            2 => Self::OperationStatus,
            3 => Self::Enable,
            4 => Self::Disable,
            6 => Self::SetZeroPosition,
            7 => Self::SetDeviceId,
            17 => Self::ReadParameter,
            18 => Self::WriteParameter,
            21 => Self::FaultReport,
            22 => Self::SaveParameters,
            23 => Self::SetBaudrate,
            24 => Self::ActiveReport,
            25 => Self::SetProtocol,
            _ => return None,
        })
    }
}

/// Motor run modes selectable through [`crate::ParamIndex::RunMode`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RunMode {
    /// MIT mode — position, velocity, torque, and PD gains all set in one frame.
    Mit = 0,
    /// Position mode (target position via parameter write).
    Position = 1,
    /// Velocity mode (target velocity via parameter write).
    Velocity = 2,
    /// Torque (current) mode (target Iq via parameter write).
    Torque = 3,
}
