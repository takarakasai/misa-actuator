//! What interfaces are attached right now.
//!
//! For picking one from a list instead of remembering `pcan:usb1`. The awkward
//! part is that the answer has two different kinds of certainty, and a list that
//! blurs them is worse than no list:
//!
//! - **A PCAN channel or a SocketCAN netdev is a fact.** The driver says the
//!   device is there, and the spec that opens it follows from its name.
//! - **A serial port is a port, not a protocol.** Nothing about `COM1` says
//!   whether the thing behind it speaks SLCAN, the USB-CAN Analyzer's binary
//!   protocol, RobStride's `AT` framing, or is a Bluetooth link that has nothing
//!   to do with CAN. Guessing from the USB vendor id gets you a CH340, which all
//!   three of the CAN candidates use.
//!
//! So a serial port is listed **once**, carrying every spec that could open it,
//! and [`Discovered::certain`] says which kind of entry it is. A UI shows the
//! port and lets the operator choose the protocol; it must not print
//! `slcan:COM1` as though that were known to work. Getting this wrong is not
//! hypothetical — an afternoon went into a dongle labelled "USB-CAN" that turned
//! out not to speak SLCAN at all (`doc/windows.md`).
//!
//! Enumeration is never the only way in: every CLI and the GUI still accept a
//! typed interface string, because a device this misses is a device that still
//! needs opening.

use crate::spec::Backend;

/// One thing that might be a CAN interface.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Discovered {
    /// Interface strings that could open it, best candidate first. More than one
    /// only when the protocol is unknown.
    pub specs: Vec<String>,
    /// Which backend the first spec uses.
    pub backend: Backend,
    /// Short name for a list: `pcan:usb1`, `COM1`, `can0`.
    pub label: String,
    /// What is known about it — the adapter's product name, USB ids, whether the
    /// channel is busy. Shown so the operator can tell two ports apart.
    pub detail: String,
    /// Whether this is a device the driver confirmed, as opposed to a port whose
    /// protocol is a guess. **A `false` here must reach the operator**; it is the
    /// difference between "this is your adapter" and "something is on this port".
    pub certain: bool,
    /// Already open, by us or by another program. Still listed: "there but busy"
    /// and "not there" need different fixes.
    pub busy: bool,
}

/// Everything attached, across every backend compiled into this binary.
///
/// Cheap enough to call whenever a UI opens a dropdown: PCAN enumeration reads a
/// parameter per channel without initialising any, and the serial scan is a
/// registry/sysfs walk. Nothing here opens a device or puts a byte on a wire.
pub fn list_interfaces() -> Vec<Discovered> {
    let mut out = Vec::new();
    out.extend(socketcan_interfaces());
    out.extend(pcan_interfaces());
    out.extend(serial_interfaces());
    out
}

#[cfg(windows)]
fn pcan_interfaces() -> Vec<Discovered> {
    use crate::backend::pcan::{available_channels, ChannelState};
    available_channels()
        .into_iter()
        .map(|(target, name, state)| {
            let busy = state == ChannelState::Occupied;
            let name = if name.is_empty() {
                "PEAK adapter".to_string()
            } else {
                name
            };
            Discovered {
                specs: vec![format!("pcan:{target}")],
                backend: Backend::Pcan,
                label: format!("pcan:{target}"),
                detail: if busy {
                    format!("{name}, already in use — one owner per channel")
                } else {
                    format!("{name}, CAN and CAN-FD")
                },
                certain: true,
                busy,
            }
        })
        .collect()
}

#[cfg(not(windows))]
fn pcan_interfaces() -> Vec<Discovered> {
    Vec::new()
}

/// CAN netdevs, from the kernel's own list.
///
/// Read from `/sys/class/net/*/uevent`, which names the device type; a plain
/// directory listing would also offer `eth0` and `lo`.
#[cfg(target_os = "linux")]
fn socketcan_interfaces() -> Vec<Discovered> {
    let Ok(entries) = std::fs::read_dir("/sys/class/net") else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().to_string();
        let uevent = entry.path().join("uevent");
        let Ok(text) = std::fs::read_to_string(&uevent) else {
            continue;
        };
        // DEVTYPE=can for vcan and for real controllers alike.
        if !text.lines().any(|l| l.trim() == "DEVTYPE=can") {
            continue;
        }
        let operstate = std::fs::read_to_string(entry.path().join("operstate"))
            .map(|s| s.trim().to_string())
            .unwrap_or_default();
        out.push(Discovered {
            specs: vec![name.clone()],
            backend: Backend::SocketCan,
            label: name,
            detail: if operstate.is_empty() {
                "SocketCAN interface".to_string()
            } else {
                format!("SocketCAN interface, {operstate}")
            },
            certain: true,
            busy: false,
        });
    }
    out.sort_by(|a, b| a.label.cmp(&b.label));
    out
}

#[cfg(not(target_os = "linux"))]
fn socketcan_interfaces() -> Vec<Discovered> {
    Vec::new()
}

/// Serial ports, each with every protocol that could be behind it.
///
/// Bluetooth links are listed last and said to be unlikely rather than filtered
/// out: an RS485 adapter or a CAN dongle on an unexpected port type is still
/// worth being able to pick.
fn serial_interfaces() -> Vec<Discovered> {
    let Ok(ports) = serialport::available_ports() else {
        return Vec::new();
    };
    let mut out: Vec<Discovered> = ports
        .into_iter()
        .map(|port| {
            let (detail, likely) = describe_port(&port.port_type);
            Discovered {
                // SLCAN first because a bare `COM*` already means slcan:, so the
                // order here matches what typing the port name alone would do.
                specs: vec![
                    format!("slcan:{}", port.port_name),
                    format!("usbcan:{}", port.port_name),
                ],
                backend: Backend::Slcan,
                label: port.port_name.clone(),
                detail: format!("{detail} — protocol unknown: slcan or usbcan"),
                certain: false,
                busy: false,
            }
            .with_priority(likely)
        })
        .collect();
    out.sort_by(|a, b| b.certain.cmp(&a.certain).then(a.label.cmp(&b.label)));
    out
}

impl Discovered {
    /// Bluetooth and unknown port types keep their entry but lose the claim to
    /// being a plausible adapter.
    fn with_priority(mut self, likely: bool) -> Self {
        if !likely {
            self.detail = format!("{} (unlikely to be a CAN adapter)", self.detail);
        }
        self
    }

    /// The spec a UI should offer first.
    pub fn primary_spec(&self) -> &str {
        self.specs.first().map(String::as_str).unwrap_or("")
    }
}

/// A human description of a port, and whether it looks like a candidate at all.
fn describe_port(kind: &serialport::SerialPortType) -> (String, bool) {
    match kind {
        serialport::SerialPortType::UsbPort(info) => {
            let mut parts = Vec::new();
            if let Some(product) = info.product.as_deref() {
                parts.push(product.to_string());
            }
            parts.push(format!("USB {:04x}:{:04x}", info.vid, info.pid));
            if let Some(serial) = info.serial_number.as_deref() {
                parts.push(format!("s/n {serial}"));
            }
            (parts.join(", "), true)
        }
        serialport::SerialPortType::BluetoothPort => ("Bluetooth serial link".to_string(), false),
        serialport::SerialPortType::PciPort => ("PCI serial port".to_string(), true),
        serialport::SerialPortType::Unknown => ("serial port".to_string(), false),
    }
}

/// One line per interface, for a CLI.
///
/// Marks the uncertain entries rather than leaving them to look like the others.
pub fn format_list(found: &[Discovered]) -> String {
    if found.is_empty() {
        return "no CAN interfaces found. A typed interface string still works — \
                see `--help` for the forms."
            .to_string();
    }
    let width = found.iter().map(|d| d.label.len()).max().unwrap_or(0);
    let mut out = String::new();
    for d in found {
        let mark = if d.certain { " " } else { "?" };
        out.push_str(&format!(
            "{mark} {:width$}  {}\n",
            d.label,
            d.detail,
            width = width
        ));
        if d.specs.len() > 1 || !d.certain {
            out.push_str(&format!("  {:width$}  → {}\n", "", d.specs.join("  or  "), width = width));
        }
    }
    out.push_str(
        "\n? = a port, not a confirmed adapter: nothing about a serial port says which\n\
         protocol is behind it, so both candidates are shown. Send `V`+CR to it — an\n\
         slcan adapter answers, a USB-CAN Analyzer does not.\n",
    );
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Enumeration must never be the reason something cannot be opened, so the
    /// only hard requirement is that it does not panic or hang on any machine.
    #[test]
    fn listing_works_on_a_machine_with_no_adapters() {
        let found = list_interfaces();
        for d in &found {
            assert!(!d.specs.is_empty(), "an entry with no way to open it");
            assert!(!d.label.is_empty());
            // Every offered spec must parse, or the list would hand a UI
            // something it cannot use.
            for spec in &d.specs {
                assert!(
                    crate::InterfaceSpec::parse(spec).is_ok(),
                    "offered an unparseable spec {spec:?}"
                );
            }
        }
    }

    /// A serial port is a port. Both protocols have to be offered, and the entry
    /// has to say it is a guess — this is the distinction the module exists for.
    #[test]
    fn a_serial_port_offers_both_protocols_and_admits_the_guess() {
        for d in serial_interfaces() {
            assert!(!d.certain, "{}: a serial port cannot be certain", d.label);
            assert_eq!(d.specs.len(), 2, "{}: both protocols", d.label);
            assert!(d.specs[0].starts_with("slcan:"));
            assert!(d.specs[1].starts_with("usbcan:"));
            assert!(
                d.detail.contains("protocol unknown"),
                "{}: detail must say so, was {:?}",
                d.label,
                d.detail
            );
        }
    }

    #[test]
    fn the_empty_list_still_tells_you_what_to_do() {
        let text = format_list(&[]);
        assert!(text.contains("typed interface string still works"), "{text}");
    }

    #[test]
    fn an_uncertain_entry_is_marked_in_the_listing() {
        let certain = Discovered {
            specs: vec!["pcan:usb1".into()],
            backend: Backend::Pcan,
            label: "pcan:usb1".into(),
            detail: "PEAK adapter".into(),
            certain: true,
            busy: false,
        };
        let guess = Discovered {
            specs: vec!["slcan:COM1".into(), "usbcan:COM1".into()],
            backend: Backend::Slcan,
            label: "COM1".into(),
            detail: "USB-SERIAL CH340 — protocol unknown: slcan or usbcan".into(),
            certain: false,
            busy: false,
        };
        let text = format_list(&[certain, guess]);
        assert!(text.contains("? COM1"), "{text}");
        assert!(text.contains("  pcan:usb1"), "{text}");
        // Both candidates offered for the guess, neither for the certain one.
        assert!(text.contains("slcan:COM1  or  usbcan:COM1"), "{text}");
    }
}
