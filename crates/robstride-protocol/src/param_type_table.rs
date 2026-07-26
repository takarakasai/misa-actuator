//! Official parameter type table for the extended `FunctionCode` space
//! (`0x0000`–`0x30xx`) read via [`crate::param_table`].
//!
//! Transcribed from the official user manuals (`ref/rs04_manual_en.md` /
//! `ref/el05_manual_en.md` — the "Full parameter table" under "Parameter
//! Settings"), converted from the vendor PDFs. Spot-checked identical
//! between RS04 and EL05 for the rows compared; treated as one shared table
//! for the RS family. This resolves the ambiguity in
//! [`crate::param_table`]'s reverse-engineered wire format, which carries no
//! type tag of its own — the earlier approach of showing every plausible
//! width side by side is superseded by this table wherever a `FunctionCode`
//! is listed here.

/// A parameter's declared wire type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamType {
    /// ASCII string (the `0x0000`/`0x0001` identity fields and the
    /// `0x1000`–`0x1007` boot/app info block).
    String,
    U8,
    U16,
    I16,
    U32,
    I32,
    F32,
}

/// One row of the official parameter table.
#[derive(Debug, Clone, Copy)]
pub struct ParamTypeInfo {
    /// `(page << 8) | row`.
    pub function_code: u16,
    /// The manual's parameter name (matches `motorstudio`'s `Name` column).
    pub name: &'static str,
    pub ty: ParamType,
}

/// Look up a `FunctionCode`'s official name and type. Returns `None` for
/// codes not in the manual's table (e.g. unlisted/reserved rows observed on
/// the wire).
///
/// ⚠ A live capture against a real RS04 (2026-07-26) found this unit's
/// firmware **does not** match the manual's `FunctionCode`↔name mapping
/// exactly: `0x2006`/`0x2007` reported names `chasu_offset`/`status1` on the
/// wire where the manual lists `MechPos_init`/`limit_torque` (while
/// neighboring `0x2005`/`0x2008` did match). So this row-by-row remapping
/// isn't a uniform offset — some firmware builds apparently repurpose
/// specific rows. Prefer [`lookup_param_type_by_name`] against a name
/// obtained from the wire itself (e.g. [`crate::param_table`]'s bulk
/// stream, which self-reports each row's name) when precision matters;
/// this function is a reasonable default only when no wire-reported name is
/// available (e.g. the single-row `read_param` path).
pub fn lookup_param_type(function_code: u16) -> Option<ParamTypeInfo> {
    PARAM_TYPE_TABLE
        .iter()
        .find(|e| e.function_code == function_code)
        .copied()
}

/// Look up a parameter's type by its **name**, as self-reported by the
/// motor on the wire (see the caveat on [`lookup_param_type`] for why this
/// is the more reliable lookup when a name is available). Exact,
/// case-sensitive match against the manual's `Name` column. `vel_max`
/// appears twice in the manual (`0x201F` and `0x3035`) but both are `float`,
/// so the ambiguity doesn't affect the returned type.
pub fn lookup_param_type_by_name(name: &str) -> Option<ParamTypeInfo> {
    PARAM_TYPE_TABLE.iter().find(|e| e.name == name).copied()
}

/// The full RS04/EL05 parameter table, from the official manuals in `ref/`.
pub static PARAM_TYPE_TABLE: &[ParamTypeInfo] = &[
    ParamTypeInfo { function_code: 0x0000, name: "Name", ty: ParamType::String },
    ParamTypeInfo { function_code: 0x0001, name: "BarCode", ty: ParamType::String },
    ParamTypeInfo { function_code: 0x1000, name: "BootCodeVersion", ty: ParamType::String },
    ParamTypeInfo { function_code: 0x1001, name: "BootBuildDate", ty: ParamType::String },
    ParamTypeInfo { function_code: 0x1002, name: "BootBuildTime", ty: ParamType::String },
    ParamTypeInfo { function_code: 0x1003, name: "AppCodeVersion", ty: ParamType::String },
    ParamTypeInfo { function_code: 0x1004, name: "AppGitVersion", ty: ParamType::String },
    ParamTypeInfo { function_code: 0x1005, name: "AppBuildDate", ty: ParamType::String },
    ParamTypeInfo { function_code: 0x1006, name: "AppBuildTime", ty: ParamType::String },
    ParamTypeInfo { function_code: 0x1007, name: "AppCodeName", ty: ParamType::String },
    ParamTypeInfo { function_code: 0x2000, name: "echoPara1", ty: ParamType::U16 },
    ParamTypeInfo { function_code: 0x2001, name: "echoPara2", ty: ParamType::U16 },
    ParamTypeInfo { function_code: 0x2002, name: "echoPara3", ty: ParamType::U16 },
    ParamTypeInfo { function_code: 0x2003, name: "echoPara4", ty: ParamType::U16 },
    ParamTypeInfo { function_code: 0x2004, name: "echoFreHz", ty: ParamType::U32 },
    ParamTypeInfo { function_code: 0x2005, name: "MechOffset", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x2006, name: "MechPos_init", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x2007, name: "limit_torque", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x2008, name: "I_FW_MAX", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x2009, name: "motor_baud", ty: ParamType::U8 },
    ParamTypeInfo { function_code: 0x200A, name: "CAN_ID", ty: ParamType::U8 },
    ParamTypeInfo { function_code: 0x200B, name: "CAN_MASTER", ty: ParamType::U8 },
    ParamTypeInfo { function_code: 0x200C, name: "CAN_TIMEOUT", ty: ParamType::U32 },
    ParamTypeInfo { function_code: 0x200D, name: "status2", ty: ParamType::I16 },
    ParamTypeInfo { function_code: 0x200E, name: "status3", ty: ParamType::U32 },
    ParamTypeInfo { function_code: 0x200F, name: "status1", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x2010, name: "Status6", ty: ParamType::U8 },
    ParamTypeInfo { function_code: 0x2011, name: "cur_filt_gain", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x2012, name: "cur_kp", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x2013, name: "cur_ki", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x2014, name: "spd_kp", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x2015, name: "spd_ki", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x2016, name: "loc_kp", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x2017, name: "spd_filt_gain", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x2018, name: "limit_spd", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x2019, name: "limit_cur", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x201A, name: "loc_ref_filt_gain", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x201B, name: "limit_loc", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x201C, name: "position_offset", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x201D, name: "chasu_angle_offset", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x201E, name: "spd_step_value", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x201F, name: "vel_max", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x2020, name: "acc_set", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x2021, name: "zero_sta", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3000, name: "timeUse0", ty: ParamType::U16 },
    ParamTypeInfo { function_code: 0x3001, name: "timeUse1", ty: ParamType::U16 },
    ParamTypeInfo { function_code: 0x3002, name: "timeUse2", ty: ParamType::U16 },
    ParamTypeInfo { function_code: 0x3003, name: "timeUse3", ty: ParamType::U16 },
    ParamTypeInfo { function_code: 0x3004, name: "encoderRaw", ty: ParamType::I16 },
    ParamTypeInfo { function_code: 0x3005, name: "mcuTemp", ty: ParamType::I16 },
    ParamTypeInfo { function_code: 0x3006, name: "motorTemp", ty: ParamType::I16 },
    ParamTypeInfo { function_code: 0x3007, name: "vBus(mv)", ty: ParamType::U16 },
    ParamTypeInfo { function_code: 0x3008, name: "adc1Offset", ty: ParamType::I32 },
    ParamTypeInfo { function_code: 0x3009, name: "adc2Offset", ty: ParamType::I32 },
    ParamTypeInfo { function_code: 0x300A, name: "adc1Raw", ty: ParamType::U16 },
    ParamTypeInfo { function_code: 0x300B, name: "adc2Raw", ty: ParamType::U16 },
    ParamTypeInfo { function_code: 0x300C, name: "VBUS", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x300D, name: "cmdId", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x300E, name: "cmdIq", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x300F, name: "cmdlocref", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3010, name: "cmdspdref", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3011, name: "cmdTorque", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3012, name: "cmdPos", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3013, name: "cmdVel", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3014, name: "rotation", ty: ParamType::I16 },
    ParamTypeInfo { function_code: 0x3015, name: "modPos", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3016, name: "mechPos", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3017, name: "mechVel", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3018, name: "elecPos", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3019, name: "ia", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x301A, name: "ib", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x301B, name: "ic", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x301C, name: "timeout", ty: ParamType::U32 },
    ParamTypeInfo { function_code: 0x301D, name: "phaseOrder", ty: ParamType::U8 },
    ParamTypeInfo { function_code: 0x301E, name: "iqf", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x301F, name: "boardTemp", ty: ParamType::I16 },
    ParamTypeInfo { function_code: 0x3020, name: "iq", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3021, name: "id", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3022, name: "faultSta", ty: ParamType::U32 },
    ParamTypeInfo { function_code: 0x3023, name: "warnSta", ty: ParamType::U32 },
    ParamTypeInfo { function_code: 0x3024, name: "drv_fault", ty: ParamType::U16 },
    ParamTypeInfo { function_code: 0x3025, name: "drv_temp", ty: ParamType::I16 },
    ParamTypeInfo { function_code: 0x3026, name: "Uq", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3027, name: "Ud", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3028, name: "dtc_u", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3029, name: "dtc_v", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x302A, name: "dtc_w", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x302B, name: "v_bus", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x302C, name: "torque_fdb", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x302D, name: "rated_i", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x302E, name: "limit_i", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x302F, name: "spd_ref", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3030, name: "spd_reff", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3031, name: "zero_fault", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3032, name: "chasu_coder_raw", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3033, name: "chasu_angle", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3034, name: "as_angle", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3035, name: "vel_max", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3036, name: "judge", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3037, name: "fault1", ty: ParamType::U32 },
    ParamTypeInfo { function_code: 0x3038, name: "fault2", ty: ParamType::U32 },
    ParamTypeInfo { function_code: 0x3039, name: "fault3", ty: ParamType::U32 },
    ParamTypeInfo { function_code: 0x303A, name: "fault4", ty: ParamType::U32 },
    ParamTypeInfo { function_code: 0x303B, name: "fault5", ty: ParamType::U32 },
    ParamTypeInfo { function_code: 0x303C, name: "fault6", ty: ParamType::U32 },
    ParamTypeInfo { function_code: 0x303D, name: "fault7", ty: ParamType::U32 },
    ParamTypeInfo { function_code: 0x303E, name: "fault8", ty: ParamType::U32 },
    ParamTypeInfo { function_code: 0x303F, name: "ElecOffset", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3040, name: "mcOverTemp", ty: ParamType::I16 },
    ParamTypeInfo { function_code: 0x3041, name: "Kt_Nm/Amp", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3042, name: "Tqcali_Type", ty: ParamType::U8 },
    ParamTypeInfo { function_code: 0x3043, name: "low_position", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3044, name: "theta_mech_1", ty: ParamType::F32 },
    ParamTypeInfo { function_code: 0x3045, name: "instep", ty: ParamType::F32 },
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn looks_up_known_codes() {
        assert_eq!(lookup_param_type(0x1003).unwrap().ty, ParamType::String);
        assert_eq!(lookup_param_type(0x1003).unwrap().name, "AppCodeVersion");
        assert_eq!(lookup_param_type(0x2005).unwrap().ty, ParamType::F32);
        assert_eq!(lookup_param_type(0x200A).unwrap().ty, ParamType::U8);
        assert_eq!(lookup_param_type(0x200D).unwrap().ty, ParamType::I16);
        assert_eq!(lookup_param_type(0x200E).unwrap().ty, ParamType::U32);
        assert_eq!(lookup_param_type(0x3008).unwrap().ty, ParamType::I32);
    }

    #[test]
    fn unknown_code_returns_none() {
        assert!(lookup_param_type(0x0404).is_none());
    }

    #[test]
    fn table_has_no_duplicate_function_codes() {
        for (i, a) in PARAM_TYPE_TABLE.iter().enumerate() {
            for b in &PARAM_TYPE_TABLE[i + 1..] {
                assert_ne!(
                    a.function_code, b.function_code,
                    "duplicate FunctionCode 0x{:04X} ({} vs {})",
                    a.function_code, a.name, b.name
                );
            }
        }
    }
}
