//! High-level driver for the LKMTech V3 servo motor family (MG / MS / RMD-X).
//!
//! Layered architecture:
//!
//! - [`bus::LkBus`] — application-level request/response trait. Concrete
//!   implementations exist for RS485 ([`Rs485Driver`]); future CAN
//!   transports plug in here.
//! - [`bus::LkCommands`] — typed command helpers (`read_state2`,
//!   `torque_control`, ...) automatically available on any `LkBus`.
//! - [`Motor`] — owns per-motor state (encoder turn tracker, gear ratio,
//!   torque-constant, position anchor). Methods take `&mut B` where
//!   `B: LkBus`, so the same `Motor` works over any transport.
//! - [`LkMotor`] — owns a bus + a `Motor`, implements
//!   [`misa_actuator::Actuator`] for use through the unified interface.
//!
//! ```no_run
//! use std::time::Duration;
//! use lkmotor_driver::{LkMotor, MotorConfig, MotorId, Rs485Driver};
//! use misa_actuator::Actuator;
//!
//! let bus = Rs485Driver::open("/dev/ttyUSB0", 1_000_000, Duration::from_millis(50)).unwrap();
//! let mut act = LkMotor::new(bus, MotorId::new(1).unwrap(), MotorConfig::current_units(10.0));
//! act.set_zero().unwrap();
//! act.set_position(0.5, 5.0).unwrap();
//! ```

pub mod bus;
pub mod driver;
pub mod error;
pub mod lk_motor;
pub mod motor;
pub mod motor_id;

pub use bus::{LkBus, LkCommands, Response, parse_state2_from_response};
pub use driver::Rs485Driver;
pub use error::{Error, Result};
pub use lk_motor::{LkMotor, PositionAnchor};
pub use motor::{ErrorFlags, Motor, MotorConfig, MotorFeedback, MotorStatus};
pub use motor_id::MotorId;

/// Re-export of the protocol crate for users who want to drop down to raw frames.
pub use lkmotor_protocol as protocol;
