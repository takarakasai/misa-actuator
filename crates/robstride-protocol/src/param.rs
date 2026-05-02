//! Parameter indices for the read/write parameter commands.

/// Robstride parameter indices.
///
/// Confirm against your firmware revision — older units occasionally renumber
/// or omit entries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum ParamIndex {
    MechOffset = 0x2005,
    MeasuredPosition = 0x3016,
    MeasuredVelocity = 0x3017,
    MeasuredTorque = 0x302C,
    RunMode = 0x7005,
    IqRef = 0x7006,
    SpdRef = 0x700A,
    LimitTorque = 0x700B,
    CurKp = 0x7010,
    CurKi = 0x7011,
    CurFiltGain = 0x7014,
    LocRef = 0x7016,
    LimitSpd = 0x7017,
    LimitCur = 0x7018,
    MechPos = 0x7019,
    IqFilt = 0x701A,
    MechVel = 0x701B,
    Vbus = 0x701C,
    LocKp = 0x701E,
    SpdKp = 0x701F,
    SpdKi = 0x7020,
    SpdFiltGain = 0x7021,
    AccRad = 0x7022,
    VelMax = 0x7024,
    AccSet = 0x7025,
    CanTimeout = 0x7028,
    ZeroState = 0x7029,
}

impl ParamIndex {
    pub fn raw(self) -> u16 {
        self as u16
    }
}
