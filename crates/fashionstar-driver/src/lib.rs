//! Driver for FashionStar UART bus servos — the servos of the Star Arm 102
//! (a.k.a. reBot Arm 102) leader and follower arms.
//!
//! Layers, following `lkmotor-driver`:
//!
//! - [`Transport`] — a byte pipe: a `serialport` handle, or [`MockTransport`]
//!   for tests.
//! - [`FsBus`] — send a frame / send and collect replies. [`FashionStarBus`]
//!   implements it over any [`Transport`]; [`misa_actuator::Shared`] wraps it
//!   for several handles on one port.
//! - [`FsCommands`] — typed commands (`ping`, `sync_monitor`,
//!   `set_angle_multiturn_by_interval`, ...) on every [`FsBus`], in SI units
//!   with `_raw` variants.
//! - [`FashionStarServo`] — one servo as a [`misa_actuator::Actuator`].
//!
//! Wire format and sources: see [`fashionstar_protocol`].
//!
//! # Star Arm 102 (LD leader)
//!
//! 1 Mbit/s 8N1 through a CH340 USB-serial adapter; servo ids `0..=6` are
//! joints 1–6 and the gripper handle. The vendor teleop loop, reproduced:
//!
//! ```no_run
//! use std::time::Duration;
//! use fashionstar_driver::{FashionStarBus, FsCommands, StopMode, DEFAULT_BAUD};
//!
//! let mut bus = FashionStarBus::open("/dev/ttyUSB0", DEFAULT_BAUD, Duration::from_millis(10))?;
//! let ids = [0, 1, 2, 3, 4, 5, 6];
//! for &id in &ids {
//!     bus.stop(id, StopMode::Release)?; // limp, so a human can move it
//!     bus.reset_multi_turn(id)?;        // fold the angle back into one turn
//! }
//! loop {
//!     for (id, m) in ids.iter().zip(bus.sync_monitor(&ids)?) {
//!         match m {
//!             Some(m) => println!("{id}: {:.1}°", m.angle_deg()),
//!             None => println!("{id}: no reply"),
//!         }
//!     }
//! }
//! # Ok::<(), fashionstar_driver::Error>(())
//! ```

pub mod bus;
pub mod commands;
pub mod error;
pub mod serial;
pub mod servo;
pub mod transport;
pub mod units;

pub use bus::{FashionStarBus, FsBus, DEFAULT_BAUD};
pub use commands::{mturn_by_interval, FsCommands, Monitor, RawMonitor, MIN_ACCEL_TIME};
pub use error::{Error, Result};
pub use fashionstar_protocol::{StopMode, BROADCAST_ID};
pub use serial::{default_serial_port, normalize_port_name};
pub use servo::FashionStarServo;
pub use transport::{MockTransport, Responder, Transport};
pub use units::{ntc_to_celsius, unwrap_to_window};

/// Re-export of the protocol crate for raw frames.
pub use fashionstar_protocol as protocol;

#[cfg(test)]
mod tests;
