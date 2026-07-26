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
    /// Added in the X4-36 V4.3 firmware generation (absent from the plain
    /// V3.9 manual's table).
    pub const COMPONENT_OVER_TEMPERATURE: u16 = 0x0800;
    pub const MOTOR_OVER_TEMPERATURE: u16 = 0x1000;
    pub const ENCODER_CALIBRATION_ERROR: u16 = 0x2000;
    /// Added in the X4-36 V4.3 firmware generation (absent from the plain
    /// V3.9 manual's table).
    pub const ENCODER_DATA_ERROR: u16 = 0x4000;

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
    #[inline] pub const fn component_over_temperature(self) -> bool {
        self.0 & Self::COMPONENT_OVER_TEMPERATURE != 0
    }
    #[inline] pub const fn encoder_data_error(self) -> bool {
        self.0 & Self::ENCODER_DATA_ERROR != 0
    }
}

/// Reply to `0x9A` — temperature, brake state, bus voltage, error flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status1 {
    /// Motor temperature, 1 °C/LSB.
    pub temperature_c: i8,
    /// MOSFET/driver-stage temperature, 1 °C/LSB. X4-36 V4.3+ firmware only
    /// (`data[2]` is `NULL` on plain V3.9 firmware, which decodes as `0`).
    pub mos_temperature_c: i8,
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

/// Which PID gain `0x30`/`0x31`/`0x32` reads/writes (V4.2+ indexed-`Float`
/// protocol, confirmed against real X4-36 firmware). Pre-V4.2 firmware used a
/// different, index-less "all six gains as `uint8`" layout — this crate
/// targets the current indexed protocol since that's what shipping hardware
/// (X4-36, firmware ≥ 2024.5) actually speaks; a bare `0x30` probe on such
/// firmware returns an all-zero float rather than six gain bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PidIndex {
    CurrentKp = 0x01,
    CurrentKi = 0x02,
    SpeedKp = 0x04,
    SpeedKi = 0x05,
    PositionKp = 0x07,
    PositionKi = 0x08,
    PositionKd = 0x09,
}

impl PidIndex {
    /// Every index, in the order [`PidGains`] presents them.
    pub const ALL: [PidIndex; 7] = [
        PidIndex::CurrentKp,
        PidIndex::CurrentKi,
        PidIndex::SpeedKp,
        PidIndex::SpeedKi,
        PidIndex::PositionKp,
        PidIndex::PositionKi,
        PidIndex::PositionKd,
    ];
}

/// Current/speed/position-loop PID gains, gathered from one `0x30` read per
/// [`PidIndex`]. Real (not normalized) `Float` gain values.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidGains {
    pub current_kp: f32,
    pub current_ki: f32,
    pub speed_kp: f32,
    pub speed_ki: f32,
    pub position_kp: f32,
    pub position_ki: f32,
    pub position_kd: f32,
}

/// Reply to `0x90` — single-turn encoder position (direct-drive models).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SingleTurnEncoder {
    /// Position after subtracting the zero offset.
    pub encoder: i16,
    /// Raw position, zero offset not applied.
    pub encoder_raw: i16,
    /// The zero-offset value itself.
    pub encoder_offset: i16,
}

/// Reply to `0x9D` — per-phase motor current.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status3 {
    /// Motor temperature, 1 °C/LSB.
    pub temperature_c: i8,
    /// Phase A/B/C current, 0.01 A/LSB.
    pub phase_a_centi_amps: i16,
    pub phase_b_centi_amps: i16,
    pub phase_c_centi_amps: i16,
}

/// Reply to `0x70` — current run mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunMode {
    CurrentLoop,
    SpeedLoop,
    PositionLoop,
    /// A value the manual doesn't document (its own text says "one of three
    /// states" but hints at a fourth without naming it).
    Unknown(u8),
}

impl RunMode {
    #[inline]
    pub const fn from_raw(raw: u8) -> Self {
        match raw {
            0x01 => Self::CurrentLoop,
            0x02 => Self::SpeedLoop,
            0x03 => Self::PositionLoop,
            other => Self::Unknown(other),
        }
    }
}
