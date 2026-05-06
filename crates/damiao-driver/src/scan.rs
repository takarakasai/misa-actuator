//! Bus scanning for DAMIAO motors.
//!
//! DAMIAO has no broadcast "ping" that every motor answers. The reliable probe
//! (matching the reference `robstride_sandbox` driver, which works on real
//! DM-J4310 hardware) is:
//!
//! 1. send a DM **enable** frame (`FF..FC`) to the candidate `CAN_ID`;
//! 2. listen briefly for *any* standard-id frame whose payload byte-0 low
//!    nibble equals `CAN_ID`'s low nibble (the motor-id field) — this works even
//!    though the feedback arrives on an unknown `MST_ID`;
//! 3. send a DM **disable** frame so a responding motor returns to a safe
//!    (coasting) state.
//!
//! No MIT command is ever sent between enable and disable, so no torque is
//! commanded — a probed motor twitches at most.

use std::ops::RangeInclusive;
use std::thread;
use std::time::{Duration, Instant};

use damiao_protocol::{build_disable_frame, build_enable_frame, DATA_LEN};

use crate::bus::DamiaoBus;
use crate::error::{Error, Result};

/// Optional progress callback: `(index, total, can_id)` before each probe.
pub type ScanProgress<'a> = &'a mut dyn FnMut(usize, usize, u8);

/// Probe each id in `id_range` and return those that answered.
pub fn scan_bus_on<B: DamiaoBus>(
    bus: &mut B,
    id_range: RangeInclusive<u8>,
    timeout_per_id: Duration,
    mut on_progress: Option<ScanProgress<'_>>,
) -> Result<Vec<u8>> {
    drain(bus);
    bus.set_timeout(timeout_per_id)?;

    let ids: Vec<u8> = id_range.collect();
    let total = ids.len();
    let mut found = Vec::new();

    for (idx, &can_id) in ids.iter().enumerate() {
        if let Some(cb) = on_progress.as_mut() {
            cb(idx, total, can_id);
        }
        if probe_one(bus, can_id, timeout_per_id)? {
            found.push(can_id);
        }
    }
    Ok(found)
}

/// Probe a single CAN_ID with an enable frame; returns whether the motor
/// answered. Always sends a disable frame afterward.
pub fn probe_one<B: DamiaoBus>(bus: &mut B, can_id: u8, timeout: Duration) -> Result<bool> {
    let (id, enable) = build_enable_frame(can_id);
    if bus.send(id, &enable).is_err() {
        return Ok(false);
    }

    let start = Instant::now();
    let mut answered = false;
    while start.elapsed() < timeout {
        match bus.recv() {
            Ok(frame) => {
                // Match the motor-id nibble in feedback byte 0; the feedback
                // CAN id (MST_ID) is unknown during a scan.
                if frame.data.len() >= DATA_LEN && (frame.data[0] & 0x0F) == (can_id & 0x0F) {
                    answered = true;
                    break;
                }
            }
            Err(Error::Timeout { .. }) => break,
            Err(_) => break,
        }
    }

    // Always return the motor to a safe state.
    let (did, disable) = build_disable_frame(can_id);
    let _ = bus.send(did, &disable);
    // Small gap so the motor processes the disable before the next probe.
    thread::sleep(Duration::from_millis(2));

    Ok(answered)
}

/// Drain any frames already buffered on the socket so they don't contaminate
/// the first probe. Uses a short timeout and stops at the first silence.
fn drain<B: DamiaoBus>(bus: &mut B) {
    let _ = bus.set_timeout(Duration::from_millis(2));
    for _ in 0..64 {
        if bus.recv().is_err() {
            break;
        }
    }
}
