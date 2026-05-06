//! [`Shared<T>`] — an `Arc<Mutex<T>>` wrapper for sharing one transport across
//! several actuator handles on the same physical bus.
//!
//! Each motor family owns its bus *by value* (`Motor<B>` / `DamiaoMotor<B>`,
//! generic over a per-family bus trait). To drive several motors that
//! physically share one CAN / RS485 wire, wrap the opened bus in a `Shared<_>`
//! and hand each motor a [`clone`](Shared::clone) — every clone points at the
//! same underlying transport.
//!
//! `Shared<T>` is transport- and family-agnostic: it lives here in the common
//! crate, and each driver crate provides the one impl that makes it usable as
//! *its* bus, e.g.
//!
//! ```ignore
//! // in damiao-driver:
//! impl<B: DamiaoBus> DamiaoBus for Shared<B> {
//!     fn send(&mut self, can_id: u16, data: &[u8]) -> Result<()> {
//!         self.lock().send(can_id, data)
//!     }
//!     // ...
//! }
//! ```
//!
//! (Implementing a crate-local bus trait for the foreign `Shared` type is
//! allowed by the orphan rule.) The same `Shared` therefore works for every
//! family — CAN-based (`DamiaoBus`, `RobstrideBus`) or RS485 (`LkBus`).
//!
//! # Threading model
//!
//! Access is serialized by the mutex, so the intended pattern is **one control
//! loop per bus** iterating over its motors: each transaction briefly locks the
//! bus for its own `send` + `recv`. Different buses (e.g. `can0` and `can1`)
//! use independent `Shared` instances and run fully in parallel. Per-motor
//! threads on the *same* bus work too, but because the bus trait's `send` and
//! `recv` are separate calls, a request and its reply are not locked as one
//! atomic transaction — prefer the single-loop-per-bus pattern.

use std::sync::{Arc, Mutex, MutexGuard};

/// A clonable handle to a single shared transport `T`.
///
/// Cloning yields another handle to the *same* `T` (the inner `Arc` is cloned,
/// not `T`), so `T` need not be `Clone`.
pub struct Shared<T>(Arc<Mutex<T>>);

impl<T> Shared<T> {
    /// Wrap a transport for sharing.
    pub fn new(inner: T) -> Self {
        Self(Arc::new(Mutex::new(inner)))
    }

    /// Lock the shared transport for exclusive access.
    ///
    /// Recovers from mutex poisoning (a panic while another handle held the
    /// lock): a CAN socket / serial port is not left in a logically invalid
    /// state by a panic, so the guard is returned rather than propagating the
    /// poison.
    pub fn lock(&self) -> MutexGuard<'_, T> {
        self.0.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// Number of handles currently sharing this transport.
    pub fn handle_count(&self) -> usize {
        Arc::strong_count(&self.0)
    }
}

impl<T> Clone for Shared<T> {
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

impl<T: core::fmt::Debug> core::fmt::Debug for Shared<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_tuple("Shared").field(&*self.lock()).finish()
    }
}
