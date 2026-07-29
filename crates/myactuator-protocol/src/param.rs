//! Named indices for the `0xC0`/`0xC1` generic parameter read/write space.
//!
//! Reverse-engineered from a live CAN capture of MyActuator's Windows
//! "Setup Software" (V4.0) talking to a real X4-36 (FW `2026042402`) —
//! not documented in any official manual obtained so far. Full writeup,
//! confidence levels, and open questions: `doc/setup-software-c0-param-protocol.md`.
//!
//! Every index carries a single `f32` value (little-endian), read via
//! [`crate::frame::build_read_param`] or written via
//! [`crate::frame::build_write_param`] + [`crate::frame::build_commit_params`].
//!
//! Two indices observed on the wire (`0x3A`, `0x3C`) are deliberately **not**
//! included here — their meaning is unconfirmed and inventing a placeholder
//! name would misrepresent them as understood (see doc §1.1, §7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ParamIndex {
    // -- Motor Information (Basic Parameters tab) --
    MotorNumber = 0x01,
    /// Doc §1.1: the wire read is correct (`260408` matches the on-screen
    /// value); the Setup Software's own "Export parameters" feature writes
    /// `0` for this field instead — a bug in that export path, not here.
    FactoryTime = 0x02,
    ReductionRatio = 0x03,

    // -- PID Parameters panel (doc §1.2) --
    // Distinct from the `PidIndex`/`0x30` gains elsewhere in this crate —
    // that command only covers Current/Speed/Position Kp/Ki; this one
    // additionally exposes Kd/R(Slope)/T(Filter) per loop, read by the
    // "PID Parameters -> Read" button. D-Axis Current has no independent
    // indices here — the GUI just mirrors Q-Axis Current onto the D-Axis
    // display without a separate wire read (doc §1.2).
    QAxisCurrentKp = 0x04,
    QAxisCurrentKi = 0x05,
    SpeedLoopKp = 0x06,
    SpeedLoopKi = 0x07,
    PositionLoopKp = 0x08,
    PositionLoopKi = 0x09,
    PositionLoopKd = 0x0A,
    QAxisCurrentKd = 0x2C,
    SpeedLoopKd = 0x2D,
    QAxisCurrentRSlope = 0x2E,
    QAxisCurrentTFilter = 0x2F,
    SpeedLoopTFilter = 0x30,
    PositionLoopTFilter = 0x31,

    // -- Encoder / calibration misc (doc §1.4, confirmed via
    // `memo_myactuator_001.xlsx` field-name export) --
    EnabledPowerdownSaveMultiTurn = 0x0B,
    /// xlsx export spells this "Pole-Paris" (typo for Pole-Pairs).
    PolePairs = 0x0C,
    /// Encoder resolution in pulses (observed `16384` = 2^14).
    SingleResolutionPulses = 0x0D,
    CalibrateCurrent = 0x0E,
    ChangeMotorDirection = 0x0F,
    /// Doc §1.4: xlsx export leaves this field blank, so the pairing with
    /// [`Self::EncoderCalibrateValue`] is inferred from value shape only
    /// (boolean-looking `1.0`), not confirmed by name.
    ExchangePhase = 0x10,
    /// Doc §1.4: same caveat as [`Self::ExchangePhase`] — blank in the xlsx
    /// export. The observed value (`6734`) also appears via the unrelated
    /// top-level `cmd 0x19` (doc §6), which is weak corroborating evidence
    /// but not a confirmation.
    EncoderCalibrateValue = 0x11,

    // -- Protect Parameters panel (doc §1.1, §1.6) --
    OverVoltage = 0x13,
    LowVoltage = 0x14,
    StallTimeLimit = 0x15,
    EBrakeStartDutyCycle = 0x16,
    CurrentSampleRes = 0x17,
    EBrakeHoldDutyCycle = 0x18,
    /// Confirmed via differential test (doc §1.6): enum `0.0` = E-Brake,
    /// `1.0` = Resistor — the GUI dropdown has exactly these two options.
    BrakeMode = 0x19,

    // -- Plan Parameters panel (doc §1.1) --
    MaxPositivePosition = 0x1A,
    MinNegativePosition = 0x1B,
    PositionPlanMaxAcc = 0x1C,
    PositionPlanMaxDec = 0x1D,
    PositionPlanMaxSpeed = 0x1E,
    SpeedPlanMaxAcc = 0x1F,
    SpeedPlanMaxDec = 0x20,
    MotorPositionZero = 0x21,
    KtOut = 0x3E,

    // -- Motor Parameters panel (doc §1.1, §1.5) --
    RatedCurrent = 0x22,
    MaxCurrent = 0x23,
    StallCurrent = 0x24,
    ShutdownTemp = 0x25,
    ResumeTemp = 0x26,
    MaxSpeed = 0x27,
    NominalSpeed = 0x28,
    EnableEtherCat = 0x29,
    /// Doc §1.5: **read-only mirror**. The actual enable/disable write goes
    /// through the dedicated `cmd 0x20` (`function_control` with index `2`,
    /// see `myactuator-driver`'s `function_control` docs) — writing this
    /// index via `0xC0` is not expected to take effect.
    EnableCanFilter = 0x32,
    Enable2ndEncoder = 0x3B,
    /// `0.0` observed = "Thermistor1" selected (only option seen so far).
    SelectThermistor = 0x3D,
    Encoder2AbnormalValue = 0x3F,
    Encoder2AbnormalSpeed = 0x40,
    AutomaticErrorRecovery = 0x41,

    // -- Output-encoder mapping (doc §1.4; names confirmed via xlsx export,
    // exact per-field semantics — polarity/offset/gain? — not reverse
    // engineered further) --
    OutEncoder = 0x33,
    OutEncoder1 = 0x34,
    OutEncoder2 = 0x35,
    OutEncoder3 = 0x36,
    OutEncoder2_1 = 0x37,
    OutEncoder2_2 = 0x38,
    OutEncoder2_3 = 0x39,
}

impl ParamIndex {
    /// Wire byte for this index.
    #[inline]
    pub const fn code(self) -> u8 {
        self as u8
    }
}

impl From<ParamIndex> for u8 {
    #[inline]
    fn from(i: ParamIndex) -> u8 {
        i as u8
    }
}
