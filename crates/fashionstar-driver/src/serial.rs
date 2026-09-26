//! Serial-port naming helpers.
//!
//! Same idea as `lkmotor_driver::serial`, duplicated rather than shared so
//! this driver does not depend on another vendor's crate.

/// A sensible default port name for this platform. The Star Arm 102 leader
/// enumerates as a CH340 (`1a86:7523`) → `/dev/ttyUSB*` on Linux.
pub const fn default_serial_port() -> &'static str {
    if cfg!(windows) {
        "COM3"
    } else {
        "/dev/ttyUSB0"
    }
}

/// Windows only opens `COM10` and above through the device namespace
/// (`\\.\COM12`); everything else passes through untouched.
pub fn normalize_port_name(name: &str) -> String {
    if !cfg!(windows) || name.starts_with(r"\\.\") {
        return name.to_string();
    }
    let upper = name.to_ascii_uppercase();
    if let Some(digits) = upper.strip_prefix("COM") {
        if digits.len() > 1 && digits.chars().all(|c| c.is_ascii_digit()) {
            return format!(r"\\.\{upper}");
        }
    }
    name.to_string()
}
