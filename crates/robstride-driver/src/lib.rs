//! Driver for the Robstride CAN servo motor family.
//!
//! Layered architecture:
//!
//! - [`bus::RobstrideBus`] — CAN-frame send/recv abstraction. Concrete
//!   implementations: [`SocketCanBus`] (Linux SocketCAN); future
//!   USB-CAN serial adapters plug in here.
//! - [`Motor<B>`] — high-level driver, generic over the bus. Defaults to
//!   `Motor<SocketCanBus>` so the original `Motor::open("can0", ...)`
//!   ergonomics still work.
//! - `impl Actuator for Motor<B>` — the unified [`misa_actuator::Actuator`]
//!   surface for use through the trait-object TUI.
//!
//! ```no_run
//! use robstride_driver::{Motor, MotorModel, RunMode};
//!
//! let mut motor = Motor::open("can0", 1, MotorModel::Rs05)?;
//! motor.enable()?;
//! motor.set_run_mode(RunMode::Position)?;
//! motor.set_position(3.14)?;
//! let fb = motor.read_status()?;
//! println!("position = {:.3} rad", fb.position);
//! motor.disable()?;
//! # Ok::<(), robstride_driver::Error>(())
//! ```

pub mod actuator;
pub mod bus;
pub mod driver;
pub mod error;
pub mod scan;

pub use bus::{CanFrame, RobstrideBus, SocketCanBus};
pub use driver::Motor;
pub use error::{Error, Result};
pub use scan::{ScanProgress, ScanResult, dump_bus, scan_bus, scan_bus_on};

/// Re-export of the protocol crate so consumers can drop down to raw frames.
pub use robstride_protocol as protocol;

pub use robstride_protocol::{
    DEFAULT_HOST_ID, MitScales, MotorFeedback, MotorModel, MotorStatusBits, ParamIndex, RunMode,
};
