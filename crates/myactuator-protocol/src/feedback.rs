//! Decoded reply payloads: Status1 (`0x9A`) and Status2 (`0x9C` and every
//! closed-loop control reply).

/// Error-state bitfield reported in Status1 (`0x9A`).
///
/// Bit values follow the V3.9 manual's `System_errorState` table.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ErrorState(pub u16);

impl ErrorState {
    pub const STALL: u16 = 0x0002;
    pub const UNDER_VOLTAGE: u16 = 0x0004;
    pub const OVER_VOLTAGE: u16 = 0x0008;
    pub const OVER_CURRENT: u16 = 0x0010;
    pub const POWER_OVERRUN: u16 = 0x0040;
    pub const CALIBRATION_WRITE_ERROR: u16 = 0x0080;
    pub const OVER_SPEED: u16 = 0x0100;
    pub const MOTOR_OVER_TEMPERATURE: u16 = 0x1000;
    pub const ENCODER_CALIBRATION_ERROR: u16 = 0x2000;

    /// Raw bitfield as reported by the motor.
    #[inline]
    pub const fn raw(self) -> u16 {
        self.0
    }

    /// `true` when any error bit is set.
    #[inline]
    pub const fn any(self) -> bool {
        self.0 != 0
    }

    #[inline] pub const fn stall(self)             -> bool { self.0 & Self::STALL != 0 }
    #[inline] pub const fn under_voltage(self)     -> bool { self.0 & Self::UNDER_VOLTAGE != 0 }
    #[inline] pub const fn over_voltage(self)      -> bool { self.0 & Self::OVER_VOLTAGE != 0 }
    #[inline] pub const fn over_current(self)      -> bool { self.0 & Self::OVER_CURRENT != 0 }
    #[inline] pub const fn power_overrun(self)     -> bool { self.0 & Self::POWER_OVERRUN != 0 }
    #[inline] pub const fn calibration_write_error(self) -> bool {
        self.0 & Self::CALIBRATION_WRITE_ERROR != 0
    }
    #[inline] pub const fn over_speed(self)        -> bool { self.0 & Self::OVER_SPEED != 0 }
    #[inline] pub const fn motor_over_temperature(self) -> bool {
        self.0 & Self::MOTOR_OVER_TEMPERATURE != 0
    }
    #[inline] pub const fn encoder_calibration_error(self) -> bool {
        self.0 & Self::ENCODER_CALIBRATION_ERROR != 0
    }
}

/// Reply to `0x9A` — temperature, brake state, bus voltage, error flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status1 {
    /// Motor temperature, 1 °C/LSB.
    pub temperature_c: i8,
    /// `true` = last brake control command was "release".
    pub brake_released: bool,
    /// Bus voltage, 0.1 V/LSB.
    pub voltage_dv: u16,
    /// Error flag bitfield.
    pub error: ErrorState,
}

impl Status1 {
    /// Bus voltage in volts.
    #[inline]
    pub fn voltage_v(&self) -> f32 {
        self.voltage_dv as f32 * 0.1
    }
}

/// Reply to `0x9C` and to every closed-loop control command
/// (`0xA1`/`0xA2`/`0xA4`/...): temperature, iq, output-shaft speed and angle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status2 {
    /// Motor temperature, 1 °C/LSB.
    pub temperature_c: i8,
    /// Torque current `iq`, 0.01 A/LSB.
    pub iq_centi_amps: i16,
    /// Output-shaft speed, 1 dps/LSB.
    pub speed_dps: i16,
    /// Output-shaft multi-turn angle, 1 °/LSB (wraps at ±32767°).
    pub angle_deg: i16,
}

impl Status2 {
    /// Torque current in amps.
    #[inline]
    pub fn current_a(&self) -> f32 {
        self.iq_centi_amps as f32 * 0.01
    }
}

/// Reply to `0x30` — current/speed/position-loop PID gains. Each value is a
/// `uint8` (0-255) normalized unit, not a physical gain: per the manual, the
/// firmware maps the model-specific gain range onto 256 equal steps, so the
/// same raw byte means a different real gain on different motor models.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PidGains {
    pub current_kp: u8,
    pub current_ki: u8,
    pub speed_kp: u8,
    pub speed_ki: u8,
    pub position_kp: u8,
    pub position_ki: u8,
}
