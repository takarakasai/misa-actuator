//! `misa-actuator` — common motor-control interface shared by all drivers in
//! this workspace.
//!
//! The [`Actuator`] trait is the SI-unit motor-control surface that
//! applications and the debug TUI speak. Every driver crate
//! (`lkmotor-driver`, `robstride-driver`, future `myactuator-driver`)
//! provides a high-level type that implements this trait so that downstream
//! code can be written generically over any backend.
//!
//! ## Layered architecture
//!
//! - **Bus / transport (per family)**: each motor family defines its own
//!   bus trait (e.g. `LkBus`, `RobstrideBus`) because the wire formats
//!   differ. Concrete implementations exist for RS485, SocketCAN, and in
//!   the future EtherCAT / USB-CAN.
//! - **Driver (per family)**: `LkMotor<B: LkBus>`, `RobstrideMotor<B: RobstrideBus>`
//!   etc. — generic over the bus, hold the per-motor state (id, gear ratio,
//!   torque constant, MIT scales, position anchor, ...).
//! - **`Actuator` trait (this crate)**: the bus-independent API surface.
//!
//! ## Units
//!
//! Every quantity exposed through [`Actuator`] is in **output-frame SI**:
//! - position in **rad**
//! - velocity in **rad/s**
//! - torque in **N·m**
//! - current in **A** (motor frame, before gear reduction)
//! - voltage in **V**, temperature in **°C**

pub mod error;
pub mod feedback;
pub mod shared;
pub mod traits;

pub use error::{Error, Result};
pub use feedback::{ErrorFlags, MotorFeedback, MotorStatus, RunMode};
pub use shared::Shared;
pub use traits::Actuator;
