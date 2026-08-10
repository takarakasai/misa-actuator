//! Serial-port naming helpers.
//!
//! RS485 works the same on Linux and Windows through the `serialport` crate;
//! only the *names* differ (`/dev/ttyUSB0` vs. `COM5`). These helpers keep
//! that difference out of the driver and the CLIs.

/// The `--interface` / `--device` default that makes sense on this platform.
///
/// There is no reliable "first RS485 adapter" on Windows, so `COM3` is a
/// convention rather than a discovery — run [`list_ports`] (or
/// `lkmotor-cli ports`) to see what is actually attached.
pub const fn default_serial_port() -> &'static str {
    if cfg!(windows) {
        "COM3"
    } else {
        "/dev/ttyUSB0"
    }
}

/// Fix up a port name so `serialport` can open it.
///
/// Windows only exposes `COM10` and above through the device namespace, so a
/// bare `COM12` fails to open while `\\.\COM12` works. Everything else is
/// passed through untouched.
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

/// Serial ports the OS currently reports, as `(name, description)` pairs.
///
/// Handy on Windows, where the RS485 adapter's COM number is assigned by the
/// OS and changes between machines.
pub fn list_ports() -> Vec<(String, String)> {
    serialport::available_ports()
        .unwrap_or_default()
        .into_iter()
        .map(|p| {
            let detail = match &p.port_type {
                serialport::SerialPortType::UsbPort(info) => {
                    // The VID:PID is what identifies the adapter chipset
                    // (1a86:7523 = CH340, 0403:6001 = FTDI, ...).
                    let product = info.product.clone().unwrap_or_default();
                    let manufacturer = info.manufacturer.clone().unwrap_or_default();
                    format!(
                        "USB {:04x}:{:04x} {manufacturer} {product}",
                        info.vid, info.pid
                    )
                    .trim_end()
                    .to_string()
                }
                // Bluetooth / PCI / unknown, plus anything a newer serialport
                // release adds — the Debug form is informative enough.
                other => format!("{other:?}"),
            };
            (p.port_name, detail)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn windows_needs_the_device_prefix_above_com9() {
        if cfg!(windows) {
            assert_eq!(normalize_port_name("COM10"), r"\\.\COM10");
            assert_eq!(normalize_port_name("com23"), r"\\.\COM23");
            assert_eq!(normalize_port_name("COM3"), "COM3");
            // Already-prefixed names are left alone.
            assert_eq!(normalize_port_name(r"\\.\COM10"), r"\\.\COM10");
        } else {
            assert_eq!(normalize_port_name("/dev/ttyUSB0"), "/dev/ttyUSB0");
        }
    }
}
