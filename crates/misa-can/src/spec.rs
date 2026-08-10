//! Parsing of the `--interface` string into a backend + target.
//!
//! One flag has to name a Linux SocketCAN interface, a PEAK PCAN channel and
//! an SLCAN adapter on a COM port, so the spec is a tiny URI-ish grammar:
//!
//! **`slcan:` means the Lawicel ASCII protocol, not "any USB-CAN dongle".**
//! Adapters sold under that name speak several different protocols, and the
//! hardware looks the same from the outside:
//!
//! | protocol | prefix | notes |
//! |----------|--------|-------|
//! | Lawicel ASCII (slcan firmware: CANable, USBtin) | `slcan:` | answers `V` + CR |
//! | "USB-CAN Analyzer" binary (`0xAA` … `0x55`) | `usbcan:` | answers nothing to `V` + CR |
//! | RobStride's own module, `41 54` … `0D 0A` | — | no backend; `robstride-protocol/ref/rs04_manual_en.md` |
//!
//! A bare `COM5` is taken as `slcan:` for compatibility, so a USB-CAN Analyzer
//! must be named explicitly. Nothing in a port name distinguishes them; the
//! test that does is to send `V` + CR and see whether ASCII comes back.
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
//! | `slcan:COM5`                | slcan-firmware adapter on COM5 @ 1 Mbit/s         |
//! | `slcan:COM5@500K?serial-baud=2000000` | 500 kbit/s CAN, 2 Mbaud on the USB link |
//! | `slcan:/dev/ttyACM0`        | the same adapter on Linux                         |
//! | `usbcan:COM1`               | USB-CAN Analyzer, 1 Mbit/s CAN, 2 Mbaud / 2 stop bits |
//! | `usbcan:COM1@250K?stop-bits=1&serial-baud=1228800` | the framing the vendor note describes |
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

/// Default baud for a USB-CAN Analyzer.
///
/// From `canusb.c`, i.e. from code that has run against the hardware; the
/// vendor's note says 1,228,800 instead. The working value wins the default and
/// `?serial-baud=` reaches the other. Neither source is committed here — see
/// [`crate::backend::usbcan`], which carries the whole format.
///
/// Unlike a CDC adapter this one is a real UART behind a CH340, so the value is
/// not decorative — at the wrong baud the adapter simply never answers, and it
/// never acknowledges anything either, so there is no error to read.
pub const DEFAULT_USBCAN_SERIAL_BAUD: u32 = 2_000_000;

/// Which transport implementation a spec selects.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Backend {
    /// Linux kernel CAN stack.
    SocketCan,
    /// PEAK-System PCAN-Basic (Windows).
    Pcan,
    /// Lawicel/SLCAN ASCII protocol over a serial port (any platform).
    Slcan,
    /// "USB-CAN Analyzer" binary protocol over a serial port.
    ///
    /// A different protocol from SLCAN, on hardware that looks identical from
    /// the outside — see the module docs. Never inferred from a bare `COM*`,
    /// because SLCAN has that inference and the two cannot be told apart from
    /// the port name.
    UsbCan,
}

impl Backend {
    pub fn name(self) -> &'static str {
        match self {
            Backend::SocketCan => "socketcan",
            Backend::Pcan => "pcan",
            Backend::Slcan => "slcan",
            Backend::UsbCan => "usbcan",
        }
    }

    /// Whether this backend is compiled into the current binary.
    pub fn is_available(self) -> bool {
        match self {
            Backend::SocketCan => cfg!(target_os = "linux"),
            Backend::Pcan => cfg!(windows),
            Backend::Slcan | Backend::UsbCan => true,
        }
    }

    /// Whether this backend can watch a bus without acknowledging on it.
    ///
    /// PCAN has a listen-only mode, and the USB-CAN Analyzer has a silent mode
    /// in its settings command. The others must refuse rather than come up as a
    /// participant: a monitor that quietly acknowledges frames changes the bus
    /// it was opened to observe.
    pub fn can_listen_only(self) -> bool {
        matches!(self, Backend::Pcan | Backend::UsbCan)
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
    /// Serial backends only: baud of the host↔adapter link. Defaults per
    /// backend — see [`DEFAULT_SLCAN_SERIAL_BAUD`] and
    /// [`DEFAULT_USBCAN_SERIAL_BAUD`].
    pub serial_baud: u32,
    /// Serial backends only: stop bits on the host↔adapter link.
    ///
    /// Two by default for the USB-CAN Analyzer, because the working reference
    /// sets `CSTOPB`; the vendor's note says one. The adapter does not
    /// acknowledge its settings command, so a wrong value shows up only as a
    /// receive timeout — which is why this is switchable.
    pub stop_bits: u8,
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

        // Per backend, because the two serial protocols disagree: an slcan
        // adapter is usually CDC and ignores the value, while a USB-CAN
        // Analyzer is a real UART that answers at exactly one baud.
        let mut serial_baud = match backend {
            Backend::UsbCan => DEFAULT_USBCAN_SERIAL_BAUD,
            _ => DEFAULT_SLCAN_SERIAL_BAUD,
        };
        let mut stop_bits = match backend {
            Backend::UsbCan => 2,
            _ => 1,
        };
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
                    "stop-bits" | "stop_bits" => {
                        stop_bits = match value {
                            "1" => 1,
                            "2" => 2,
                            other => {
                                return Err(Error::bad_spec(
                                    raw,
                                    format!("stop-bits must be 1 or 2, not {other:?}"),
                                ))
                            }
                        };
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
            stop_bits,
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
            Backend::UsbCan => "The USB-CAN Analyzer backend needs a serial port this \
                                platform can open.",
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
            "usbcan" | "usb-can" | "canalyzer" => Some(Backend::UsbCan),
            _ => None,
        };
        if let Some(backend) = backend {
            return Ok((backend, rest));
        }
        return Err(Error::bad_spec(
            raw,
            format!("unknown backend {scheme:?} (known: socketcan, pcan, slcan, usbcan)"),
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
         `pcan:usb1` for a PEAK adapter or `slcan:COM5` for one running slcan firmware",
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

    /// The two serial backends must stay tellable apart, and a bare `COM*` must
    /// keep meaning SLCAN — nothing in a port name distinguishes the protocols,
    /// so silently changing that inference would break existing command lines
    /// and mislead new ones.
    #[test]
    fn the_usb_can_analyzer_is_a_separate_backend_that_is_never_inferred() {
        for name in ["usbcan:COM1", "usb-can:COM1", "canalyzer:COM1"] {
            let s = InterfaceSpec::parse(name).unwrap();
            assert_eq!(s.backend, Backend::UsbCan, "{name}");
            assert_eq!(s.target, "COM1");
        }
        // Bare COM ports still mean SLCAN.
        assert_eq!(
            InterfaceSpec::parse("COM1").unwrap().backend,
            Backend::Slcan
        );
        assert_eq!(
            InterfaceSpec::parse("slcan:COM1").unwrap().backend,
            Backend::Slcan
        );
    }

    /// The baud is not decorative here: the adapter is a real UART behind a
    /// CH340 and answers at exactly one rate, so the default has to be its own.
    #[test]
    fn the_usb_can_analyzer_defaults_to_its_documented_baud() {
        let s = InterfaceSpec::parse("usbcan:COM1").unwrap();
        assert_eq!(s.serial_baud, DEFAULT_USBCAN_SERIAL_BAUD);
        assert_eq!(s.serial_baud, 2_000_000);
        // SLCAN keeps its own default...
        assert_eq!(
            InterfaceSpec::parse("slcan:COM1").unwrap().serial_baud,
            DEFAULT_SLCAN_SERIAL_BAUD
        );
        // ...and an explicit value still wins on either.
        assert_eq!(
            InterfaceSpec::parse("usbcan:COM1?serial-baud=115200")
                .unwrap()
                .serial_baud,
            115_200
        );
    }

    /// The two sources disagree on the serial framing — the working reference
    /// uses two stop bits, the vendor note one — and a wrong choice is invisible
    /// (the adapter never acknowledges), so both have to be reachable.
    #[test]
    fn the_usb_can_analyzer_defaults_to_the_reference_framing() {
        let s = InterfaceSpec::parse("usbcan:COM1").unwrap();
        assert_eq!(s.stop_bits, 2, "canusb.c sets CSTOPB");
        // The vendor note's combination, spelled out in one spec.
        let s = InterfaceSpec::parse("usbcan:COM1?stop-bits=1&serial-baud=1228800").unwrap();
        assert_eq!(s.stop_bits, 1);
        assert_eq!(s.serial_baud, 1_228_800);
        // Everything else stays on one stop bit.
        assert_eq!(InterfaceSpec::parse("slcan:COM1").unwrap().stop_bits, 1);
        assert!(InterfaceSpec::parse("usbcan:COM1?stop-bits=3").is_err());
    }

    /// Silent mode is in this adapter's settings command, so it can monitor
    /// without acknowledging. Everything except PCAN and it must still refuse.
    #[test]
    fn listen_only_is_offered_by_exactly_the_backends_that_have_it() {
        assert!(Backend::Pcan.can_listen_only());
        assert!(Backend::UsbCan.can_listen_only());
        assert!(!Backend::Slcan.can_listen_only());
        assert!(!Backend::SocketCan.can_listen_only());
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
