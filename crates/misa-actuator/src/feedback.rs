//! Common feedback / status / mode types in output-frame SI units.

/// Per-cycle measurement returned by every control / measure call.
///
/// All quantities are in the **output frame** (after the motor's reduction
/// gearbox, when one is configured) except `current_a`, which is the raw
/// motor-frame `iq` (always in motor-electrical amps).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorFeedback {
    /// Continuous (multi-turn where supported) position in **radians**.
    /// Reference frame depends on the driver: most expose position relative
    /// to the last `set_zero()` anchor.
    pub position_rad: f32,
    /// Velocity in **rad/s**.
    pub velocity_rad_per_s: f32,
    /// Output-frame torque in **N·m**.
    pub torque_nm: f32,
    /// Motor-frame quadrature current `iq` in **A**.
    pub current_a: f32,
    /// Motor temperature in **°C** (NaN if not available in this frame).
    pub temperature_c: f32,
}

impl MotorFeedback {
    /// All-zero feedback — useful as a placeholder before the first read.
    pub const fn zero() -> Self {
        Self {
            position_rad: 0.0,
            velocity_rad_per_s: 0.0,
            torque_nm: 0.0,
            current_a: 0.0,
            temperature_c: f32::NAN,
        }
    }
}

/// Slow-changing status (bus voltage, motor temperature, error flags).
///
/// On most motors this requires a separate transaction from the per-cycle
/// feedback frame, so it should be polled at a lower rate (e.g. 1 Hz).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorStatus {
    /// DC bus voltage in **V** (NaN if not reported by this driver).
    pub voltage_v: f32,
    /// Motor temperature in **°C** (NaN if not reported by this driver).
    pub temperature_c: f32,
    /// Decoded error / fault bits.
    pub error: ErrorFlags,
}

/// Common motor fault / warning flags.
///
/// Each driver maps its native fault bitfield onto these common bits and
/// stashes the original raw value in [`Self::raw`] for inspection.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ErrorFlags {
    bits: u32,
    raw: u32,
}

impl ErrorFlags {
    /// Bit positions in [`Self::bits`].
    pub const UNDER_VOLTAGE: u32 = 1 << 0;
    pub const OVER_VOLTAGE: u32 = 1 << 1;
    pub const OVER_CURRENT: u32 = 1 << 2;
    pub const MOTOR_OVERHEAT: u32 = 1 << 3;
    pub const DRIVER_OVERHEAT: u32 = 1 << 4;
    pub const STALL: u32 = 1 << 5;
    pub const MOTOR_SHORT: u32 = 1 << 6;
    pub const SIGNAL_TIMEOUT: u32 = 1 << 7;
    pub const ENCODER_FAULT: u32 = 1 << 8;
    pub const UNCALIBRATED: u32 = 1 << 9;

    /// Construct from common bits + driver-native raw value.
    pub const fn new(bits: u32, raw: u32) -> Self {
        Self { bits, raw }
    }

    /// Common (cross-driver) flag bitset.
    #[inline]
    pub const fn bits(self) -> u32 {
        self.bits
    }

    /// Raw driver-native bitfield, for diagnostics.
    #[inline]
    pub const fn raw(self) -> u32 {
        self.raw
    }

    /// `true` if any common flag is set.
    #[inline]
    pub const fn any(self) -> bool {
        self.bits != 0
    }

    #[inline] pub const fn under_voltage(self)   -> bool { self.bits & Self::UNDER_VOLTAGE   != 0 }
    #[inline] pub const fn over_voltage(self)    -> bool { self.bits & Self::OVER_VOLTAGE    != 0 }
    #[inline] pub const fn over_current(self)    -> bool { self.bits & Self::OVER_CURRENT    != 0 }
    #[inline] pub const fn motor_overheat(self)  -> bool { self.bits & Self::MOTOR_OVERHEAT  != 0 }
    #[inline] pub const fn driver_overheat(self) -> bool { self.bits & Self::DRIVER_OVERHEAT != 0 }
    #[inline] pub const fn stall(self)           -> bool { self.bits & Self::STALL           != 0 }
    #[inline] pub const fn motor_short(self)     -> bool { self.bits & Self::MOTOR_SHORT     != 0 }
    #[inline] pub const fn signal_timeout(self)  -> bool { self.bits & Self::SIGNAL_TIMEOUT  != 0 }
    #[inline] pub const fn encoder_fault(self)   -> bool { self.bits & Self::ENCODER_FAULT   != 0 }
    #[inline] pub const fn uncalibrated(self)    -> bool { self.bits & Self::UNCALIBRATED    != 0 }
}

/// Hint about which control mode a high-level driver should configure
/// before issuing per-mode commands.
///
/// Drivers that don't model an explicit run-mode (e.g. lkmotor V3 which
/// takes a different command per mode) will treat this as a no-op.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunMode {
    /// MIT-style position+velocity+torque feedforward with PD gains.
    Mit,
    /// Closed-loop position control.
    Position,
    /// Closed-loop velocity control.
    Velocity,
    /// Closed-loop torque (current) control.
    Torque,
}
