//! Parsing of the `--interface` string into a backend + target.
//!
//! One flag has to name a Linux SocketCAN interface, a PEAK PCAN channel and
//! a USB-CAN adapter on a COM port, so the spec is a tiny URI-ish grammar:
//!
//! ```text
//! [<backend>:]<target>[@<bitrate>[,<data-bitrate>]][?<key>=<value>&...]
//! ```
//!
//! | spec                        | meaning                                          |
//! |-----------------------------|--------------------------------------------------|
//! | `can0`                      | SocketCAN `can0` (Linux)                          |
//! | `socketcan:can1`            | SocketCAN, stated explicitly                      |
//! | `pcan:usb1`                 | PEAK `PCAN_USBBUS1` @ 1 Mbit/s                    |
//! | `PCAN_USBBUS1`              | same, using PEAK's own channel name               |
//! | `pcan:usb1@1M,5M`           | PCAN, 1 Mbit/s arbitration, 5 Mbit/s FD data      |
//! | `slcan:COM5`                | SLCAN adapter on COM5 @ 1 Mbit/s                  |
//! | `slcan:COM5@500K?serial-baud=2000000` | 500 kbit/s CAN, 2 Mbaud on the USB link |
//! | `slcan:/dev/ttyACM0`        | the same adapter on Linux                         |
//!
//! A bare target with no backend prefix is resolved by platform: SocketCAN on
//! Linux, and on Windows `COM*` → SLCAN, `PCAN_*` → PCAN.

use crate::error::{Error, Result};

/// Default arbitration bitrate. Every motor family in this workspace ships
/// configured for 1 Mbit/s.
pub const DEFAULT_BITRATE: u32 = 1_000_000;

/// Default CAN-FD data-phase bitrate (DAMIAO's documented FD setup).
pub const DEFAULT_DATA_BITRATE: u32 = 5_000_000;

/// Default baud of the USB serial link an SLCAN adapter presents. Real
/// USB-CDC adapters ignore it, but the value still has to be legal.
pub const DEFAULT_SLCAN_SERIAL_BAUD: u32 = 115_200;

/// Which transport implementation a spec selects.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Backend {
    /// Linux kernel CAN stack.
    SocketCan,
    /// PEAK-System PCAN-Basic (Windows).
    Pcan,
    /// Lawicel/SLCAN ASCII protocol over a serial port (any platform).
    Slcan,
}

impl Backend {
    pub fn name(self) -> &'static str {
        match self {
            Backend::SocketCan => "socketcan",
            Backend::Pcan => "pcan",
            Backend::Slcan => "slcan",
        }
    }

    /// Whether this backend is compiled into the current binary.
    pub fn is_available(self) -> bool {
        match self {
            Backend::SocketCan => cfg!(target_os = "linux"),
            Backend::Pcan => cfg!(windows),
            Backend::Slcan => true,
        }
    }
}

/// A parsed `--interface` value.
#[derive(Debug, Clone)]
pub struct InterfaceSpec {
    pub backend: Backend,
    /// Backend-specific device name (`can0`, `usb1`, `COM5`, ...).
    pub target: String,
    /// Arbitration bitrate in bit/s.
    pub bitrate: u32,
    /// CAN-FD data-phase bitrate in bit/s. Ignored unless FD is requested.
    pub data_bitrate: u32,
    /// SLCAN only: baud of the host↔adapter serial link.
    pub serial_baud: u32,
    /// The original string, kept for error messages.
    pub raw: String,
}

impl InterfaceSpec {
    /// Parse an `--interface` value. See the module docs for the grammar.
    pub fn parse(spec: &str) -> Result<Self> {
        let raw = spec.trim();
        if raw.is_empty() {
            return Err(Error::bad_spec(spec, "empty interface"));
        }

        // Split off the query string first so a `?` can never be mistaken for
        // part of a device name.
        let (head, query) = match raw.split_once('?') {
            Some((h, q)) => (h, Some(q)),
            None => (raw, None),
        };

        // ...then the bitrate suffix.
        let (locator, rates) = match head.split_once('@') {
            Some((l, r)) => (l, Some(r)),
            None => (head, None),
        };

        let (backend, target) = split_backend(locator, raw)?;
        if target.is_empty() {
            return Err(Error::bad_spec(raw, "no device named after the backend"));
        }

        let (bitrate, data_bitrate) = match rates {
            None => (DEFAULT_BITRATE, DEFAULT_DATA_BITRATE),
            Some(r) => {
                let (nominal, data) = match r.split_once(',') {
                    Some((n, d)) => (n, Some(d)),
                    None => (r, None),
                };
                let nominal = parse_bitrate(nominal, raw)?;
                let data = match data {
                    Some(d) => parse_bitrate(d, raw)?,
                    None => DEFAULT_DATA_BITRATE.max(nominal),
                };
                (nominal, data)
            }
        };

        let mut serial_baud = DEFAULT_SLCAN_SERIAL_BAUD;
        if let Some(query) = query {
            for pair in query.split('&').filter(|p| !p.is_empty()) {
                let (key, value) = pair
                    .split_once('=')
                    .ok_or_else(|| Error::bad_spec(raw, format!("option {pair:?} needs a value")))?;
                match key {
                    "serial-baud" | "serial_baud" => {
                        serial_baud = value.parse().map_err(|_| {
                            Error::bad_spec(raw, format!("bad serial baud {value:?}"))
                        })?;
                    }
                    other => {
                        return Err(Error::bad_spec(raw, format!("unknown option {other:?}")));
                    }
                }
            }
        }

        Ok(Self {
            backend,
            target: target.to_string(),
            bitrate,
            data_bitrate,
            serial_baud,
            raw: raw.to_string(),
        })
    }

    /// Reject a spec whose backend was not compiled in, with a message that
    /// says what to use instead rather than just "unsupported".
    pub fn require_available(&self) -> Result<()> {
        if self.backend.is_available() {
            return Ok(());
        }
        let hint = match self.backend {
            Backend::SocketCan => {
                "SocketCAN is Linux-only. On Windows use a PEAK adapter \
                 (`pcan:usb1`) or an SLCAN adapter (`slcan:COM5`)."
            }
            Backend::Pcan => "PCAN-Basic is Windows-only. On Linux use SocketCAN (`can0`).",
            Backend::Slcan => "SLCAN needs a serial port this platform can open.",
        };
        Err(Error::bad_spec(&self.raw, hint))
    }
}

/// Pull an explicit `backend:` prefix off the locator, or infer one.
fn split_backend<'a>(locator: &'a str, raw: &str) -> Result<(Backend, &'a str)> {
    // A Windows device path (`\\.\COM10`) has no colon, so any colon here is
    // genuinely a scheme separator.
    if let Some((scheme, rest)) = locator.split_once(':') {
        let backend = match scheme.to_ascii_lowercase().as_str() {
            "socketcan" | "can" => Some(Backend::SocketCan),
            "pcan" | "peak" => Some(Backend::Pcan),
            "slcan" | "lawicel" | "serial" => Some(Backend::Slcan),
            _ => None,
        };
        if let Some(backend) = backend {
            return Ok((backend, rest));
        }
        return Err(Error::bad_spec(
            raw,
            format!("unknown backend {scheme:?} (known: socketcan, pcan, slcan)"),
        ));
    }

    Ok((infer_backend(locator, raw)?, locator))
}

/// Guess the backend for a bare device name.
fn infer_backend(target: &str, raw: &str) -> Result<Backend> {
    let lower = target.to_ascii_lowercase();
    if lower.starts_with("pcan_") {
        return Ok(Backend::Pcan);
    }
    if is_com_port(&lower) || lower.starts_with("/dev/tty") {
        return Ok(Backend::Slcan);
    }
    // Kernel netdev names are recognised on *every* platform, not just Linux,
    // so that a command line copied from the Linux docs answers "SocketCAN is
    // Linux-only, use pcan:usb1" instead of "I can't tell what this is".
    // `require_available` produces that message.
    if is_can_netdev(&lower) {
        return Ok(Backend::SocketCan);
    }
    if cfg!(target_os = "linux") {
        return Ok(Backend::SocketCan);
    }
    Err(Error::bad_spec(
        raw,
        "cannot tell which CAN backend this is; prefix it, e.g. \
         `pcan:usb1` for a PEAK adapter or `slcan:COM5` for a USB-CAN adapter",
    ))
}

/// `com5`, `com12` — a Windows serial port.
fn is_com_port(lower: &str) -> bool {
    matches!(lower.strip_prefix("com"), Some(n) if !n.is_empty() && n.chars().all(|c| c.is_ascii_digit()))
}

/// `can0`, `vcan0`, `slcan0` — a Linux CAN network device.
fn is_can_netdev(lower: &str) -> bool {
    ["vcan", "slcan", "can"].iter().any(|prefix| {
        matches!(lower.strip_prefix(prefix), Some(n) if !n.is_empty() && n.chars().all(|c| c.is_ascii_digit()))
    })
}

/// Accept `1000000`, `1M`, `500k`, `125K`.
fn parse_bitrate(text: &str, raw: &str) -> Result<u32> {
    let t = text.trim();
    if t.is_empty() {
        return Err(Error::bad_spec(raw, "empty bitrate"));
    }
    let (digits, scale) = match t.chars().last().unwrap() {
        'k' | 'K' => (&t[..t.len() - 1], 1_000),
        'm' | 'M' => (&t[..t.len() - 1], 1_000_000),
        _ => (t, 1),
    };
    let value: f64 = digits
        .parse()
        .map_err(|_| Error::bad_spec(raw, format!("bad bitrate {text:?}")))?;
    let bits = value * scale as f64;
    if !(1_000.0..=10_000_000.0).contains(&bits) {
        return Err(Error::bad_spec(
            raw,
            format!("bitrate {text:?} is outside 1 kbit/s..10 Mbit/s"),
        ));
    }
    Ok(bits as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn explicit_backends() {
        let s = InterfaceSpec::parse("pcan:usb1").unwrap();
        assert_eq!(s.backend, Backend::Pcan);
        assert_eq!(s.target, "usb1");
        assert_eq!(s.bitrate, DEFAULT_BITRATE);

        let s = InterfaceSpec::parse("socketcan:can1").unwrap();
        assert_eq!(s.backend, Backend::SocketCan);
        assert_eq!(s.target, "can1");

        let s = InterfaceSpec::parse("slcan:/dev/ttyACM0").unwrap();
        assert_eq!(s.backend, Backend::Slcan);
        assert_eq!(s.target, "/dev/ttyACM0");
    }

    #[test]
    fn bare_names_are_inferred() {
        assert_eq!(
            InterfaceSpec::parse("COM5").unwrap().backend,
            Backend::Slcan
        );
        assert_eq!(
            InterfaceSpec::parse("COM12").unwrap().backend,
            Backend::Slcan
        );
        assert_eq!(
            InterfaceSpec::parse("PCAN_USBBUS1").unwrap().backend,
            Backend::Pcan
        );
        // Kernel netdev names resolve to SocketCAN everywhere, so that the
        // "Linux-only" message can be the one the user sees on Windows.
        for name in ["can0", "can1", "vcan0", "slcan0"] {
            assert_eq!(
                InterfaceSpec::parse(name).unwrap().backend,
                Backend::SocketCan,
                "{name}"
            );
        }
    }

    #[test]
    fn a_linux_interface_is_rejected_off_linux_with_an_actionable_message() {
        let spec = InterfaceSpec::parse("can0").unwrap();
        let verdict = spec.require_available();
        if cfg!(target_os = "linux") {
            assert!(verdict.is_ok());
        } else {
            let msg = verdict.unwrap_err().to_string();
            assert!(msg.contains("Linux-only"), "{msg}");
            assert!(msg.contains("pcan:usb1"), "{msg}");
        }
    }

    #[test]
    fn a_name_that_is_neither_is_rejected_off_linux() {
        // Not a netdev, not a COM port, not a PEAK channel.
        let parsed = InterfaceSpec::parse("mystery0");
        if cfg!(target_os = "linux") {
            assert_eq!(parsed.unwrap().backend, Backend::SocketCan);
        } else {
            assert!(parsed.is_err());
        }
    }

    #[test]
    fn bitrates_accept_suffixes() {
        let s = InterfaceSpec::parse("pcan:usb1@1M,5M").unwrap();
        assert_eq!(s.bitrate, 1_000_000);
        assert_eq!(s.data_bitrate, 5_000_000);

        let s = InterfaceSpec::parse("slcan:COM5@500K").unwrap();
        assert_eq!(s.bitrate, 500_000);

        assert!(InterfaceSpec::parse("pcan:usb1@nonsense").is_err());
        assert!(InterfaceSpec::parse("pcan:usb1@50").is_err());
    }

    #[test]
    fn serial_baud_option() {
        let s = InterfaceSpec::parse("slcan:COM5@1M?serial-baud=2000000").unwrap();
        assert_eq!(s.serial_baud, 2_000_000);
        assert_eq!(s.bitrate, 1_000_000);
        assert_eq!(s.target, "COM5");

        assert!(InterfaceSpec::parse("slcan:COM5?bogus=1").is_err());
    }

    #[test]
    fn rejects_unknown_scheme_and_empty() {
        assert!(InterfaceSpec::parse("kvaser:0").is_err());
        assert!(InterfaceSpec::parse("").is_err());
        assert!(InterfaceSpec::parse("pcan:").is_err());
    }
}
