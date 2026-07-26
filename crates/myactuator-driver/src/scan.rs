//! Bus scanning for MyActuator RMD motors.
//!
//! The probe is a side-effect-free Status1 read (`0x9A`) to `0x140 + id`; a
//! present motor answers on `0x240 + id`. No control command is ever sent, so
//! a probed motor never moves.

use std::ops::RangeInclusive;
use std::time::{Duration, Instant};

use myactuator_protocol::{build_read_status1, can_id, DATA_LEN};

use crate::bus::MyActuatorBus;
use crate::error::{Error, Result};

/// Optional progress callback: `(index, total, motor_id)` before each probe.
pub type ScanProgress<'a> = &'a mut dyn FnMut(usize, usize, u8);

/// Probe each id in `id_range` and return those that answered. Ids outside
/// 1..=32 are skipped (never probed, never reported).
pub fn scan_bus_on<B: MyActuatorBus>(
    bus: &mut B,
    id_range: RangeInclusive<u8>,
    timeout_per_id: Duration,
    mut on_progress: Option<ScanProgress<'_>>,
) -> Result<Vec<u8>> {
    drain(bus);
    bus.set_timeout(timeout_per_id)?;

    let ids: Vec<u8> = id_range.filter(|&id| can_id::is_valid_motor_id(id)).collect();
    let total = ids.len();
    let mut found = Vec::new();

    for (idx, &motor_id) in ids.iter().enumerate() {
        if let Some(cb) = on_progress.as_mut() {
            cb(idx, total, motor_id);
        }
        if probe_one(bus, motor_id, timeout_per_id)? {
            found.push(motor_id);
        }
    }
    Ok(found)
}

/// Probe a single motor id with a Status1 read; returns whether it answered.
pub fn probe_one<B: MyActuatorBus>(bus: &mut B, motor_id: u8, timeout: Duration) -> Result<bool> {
    if !can_id::is_valid_motor_id(motor_id) {
        return Ok(false);
    }
    let payload = build_read_status1();
    if bus.send(can_id::command_id(motor_id), &payload).is_err() {
        return Ok(false);
    }

    let start = Instant::now();
    while start.elapsed() < timeout {
        match bus.recv() {
            Ok(frame) => {
                if frame.can_id == can_id::reply_id(motor_id)
                    && frame.data.len() >= DATA_LEN
                    && frame.data[0] == payload[0]
                {
                    return Ok(true);
                }
            }
            Err(Error::Timeout { .. }) => break,
            Err(_) => break,
        }
    }
    Ok(false)
}

/// Drain any frames already buffered on the socket so they don't contaminate
/// the first probe. Uses a short timeout and stops at the first silence.
fn drain<B: MyActuatorBus>(bus: &mut B) {
    let _ = bus.set_timeout(Duration::from_millis(2));
    for _ in 0..64 {
        if bus.recv().is_err() {
            break;
        }
    }
}
