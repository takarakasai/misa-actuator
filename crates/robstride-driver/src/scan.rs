//! Bus scanning and passive monitoring helpers.
//!
//! These convenience helpers build an [`AnyCanBus`] internally. To scan
//! over a bus you already hold open, call [`scan_bus_on`].

use std::ops::RangeInclusive;
use std::time::{Duration, Instant};

use robstride_protocol::{build_ping_frame, parse_can_id};

use crate::bus::{AnyCanBus, RobstrideBus};
use crate::error::{Error, Result};

/// Result of probing a single motor id.
#[derive(Debug, Clone)]
pub struct ScanResult {
    pub motor_id: u8,
    /// 8-byte response payload (typically the MCU UUID for Robstride motors).
    pub payload: Vec<u8>,
}

/// Progress callback signature for [`scan_bus`]. Reports `(index, total, motor_id)`.
pub type ScanProgress<'a> = &'a mut dyn FnMut(usize, usize, u8);

/// Scan a SocketCAN interface for motors that respond to `GET_DEVICE_ID`.
pub fn scan_bus(
    interface: &str,
    host_id: u8,
    id_range: RangeInclusive<u8>,
    timeout_per_id: Duration,
    on_progress: Option<ScanProgress<'_>>,
) -> Result<Vec<ScanResult>> {
    let mut bus = AnyCanBus::open(interface)?;
    scan_bus_on(&mut bus, host_id, id_range, timeout_per_id, on_progress)
}

/// Scan over an already-open [`RobstrideBus`].
pub fn scan_bus_on<B: RobstrideBus>(
    bus: &mut B,
    host_id: u8,
    id_range: RangeInclusive<u8>,
    timeout_per_id: Duration,
    mut on_progress: Option<ScanProgress<'_>>,
) -> Result<Vec<ScanResult>> {
    bus.set_timeout(timeout_per_id)?;
    let ids: Vec<u8> = id_range.collect();
    let total = ids.len();
    let mut found = Vec::new();

    for (idx, &motor_id) in ids.iter().enumerate() {
        if let Some(cb) = on_progress.as_mut() {
            cb(idx, total, motor_id);
        }

        let (can_id, data) = build_ping_frame(host_id, motor_id);
        if bus.send(can_id, &data).is_err() {
            continue;
        }

        let start = Instant::now();
        while start.elapsed() < timeout_per_id {
            match bus.recv() {
                Ok(frame) => {
                    let (ct, extra, dev_id) = parse_can_id(frame.can_id);
                    let resp_motor_id = (extra & 0xFF) as u8;
                    // Skip our own TX echo (gs_usb ECHO flag re-emits the same frame).
                    if dev_id == motor_id && ct == 0 {
                        continue;
                    }
                    if resp_motor_id == motor_id || dev_id == motor_id {
                        found.push(ScanResult {
                            motor_id,
                            payload: frame.data,
                        });
                        break;
                    }
                }
                Err(Error::Timeout { .. }) => break,
                Err(e) => return Err(e),
            }
        }
    }

    if let Some(cb) = on_progress.as_mut() {
        cb(total, total, 0);
    }
    Ok(found)
}

/// Passively listen on the bus and return every frame seen during `duration`.
pub fn dump_bus(interface: &str, duration: Duration) -> Result<Vec<(u32, Vec<u8>)>> {
    let mut frames = Vec::new();
    dump_bus_streaming(interface, duration, &|| false, &mut |_, id, data| {
        frames.push((id, data.to_vec()))
    })?;
    Ok(frames)
}

/// Listen passively and hand each frame to `on_frame` as it arrives, with the
/// time since capture started.
///
/// Streaming rather than collect-then-return because the reason to run this is
/// usually to watch what another tool puts on the bus while you drive its UI —
/// output that appears only after the capture window closes is useless for
/// correlating "I pressed the button" with "these frames appeared".
///
/// `should_stop` is polled between frames so a Ctrl-C can end the capture
/// early without losing what was already printed.
pub fn dump_bus_streaming(
    interface: &str,
    duration: Duration,
    should_stop: &dyn Fn() -> bool,
    on_frame: &mut dyn FnMut(Duration, u32, &[u8]),
) -> Result<usize> {
    let mut bus = AnyCanBus::open(interface)?;
    // Short poll so `should_stop` and the deadline are both honoured promptly
    // on a quiet bus.
    bus.set_timeout(Duration::from_millis(50))?;
    let mut count = 0usize;
    let start = Instant::now();
    while start.elapsed() < duration && !should_stop() {
        match bus.recv() {
            Ok(frame) => {
                on_frame(start.elapsed(), frame.can_id, &frame.data);
                count += 1;
            }
            Err(Error::Timeout { .. }) => continue,
            Err(e) => return Err(e),
        }
    }
    Ok(count)
}
