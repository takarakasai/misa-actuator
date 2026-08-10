//! Session plumbing shared by every front end in this workspace.
//!
//! `misa-actuator` gives you a blocking, `&mut self` [`Actuator`]. That is the
//! right shape for a driver and the wrong shape for a user interface: a single
//! transaction costs about a millisecond, and a UI that blocks on one drops
//! frames. This crate is the layer in between — a worker thread that owns the
//! actuator, and a handle a UI drives it through without ever waiting on the
//! bus.
//!
//! Deliberately free of any UI dependency. The GUI, the TUI and a future
//! headless server all sit on top of the same [`Session`], and the tests here
//! run it against [`misa_actuator_sim`] with no hardware present.
//!
//! ```no_run
//! use std::time::Duration;
//! use misa_actuator_core::{Command, ControlMode, DriverConfig, DriverKind, Session, Setpoint};
//!
//! # fn main() -> anyhow::Result<()> {
//! let session = Session::connect(&DriverConfig {
//!     kind: DriverKind::Sim,
//!     model: "rs04".into(),
//!     ..DriverConfig::default()
//! })?;
//!
//! session.send(Command::Enable)?;
//! session.set_setpoint(Setpoint {
//!     mode: ControlMode::Position,
//!     position_rad: 0.5,
//!     max_speed_rad_s: 2.0,
//!     ..Setpoint::default()
//! });
//! session.send(Command::SetStreaming { on: true })?;
//!
//! // ...then once per UI frame:
//! session.heartbeat();
//! for sample in session.drain_samples() {
//!     println!("{:+.3} rad", sample.position_rad);
//! }
//! # Ok(())
//! # }
//! ```
//!
//! # Safety model
//!
//! Three mechanisms, in order of how much they are relied on:
//!
//! 1. [`Session::stop`] sets a flag the worker checks before every
//!    transaction. It is not a queued command, so it cannot end up behind a
//!    backlog.
//! 2. The **watchdog** disables the motor if the UI stops calling
//!    [`Session::heartbeat`] while streaming. This is what covers a hung
//!    renderer or a crashed front-end process — cases where a stop button, by
//!    definition, will never be pressed.
//! 3. Dropping the [`Session`] stops the motor and waits for the worker to
//!    confirm, so a closing window cannot outlive a driven motor.

pub mod factory;
pub mod multi;
pub mod multi_session;
pub mod protocol;
pub(crate) mod safety;
pub mod session;
pub(crate) mod worker;

pub use factory::{
    build_actuator, build_actuator_checked, validate_driver_args, BusKind, DriverConfig, DriverKind,
    IdentityReport, DEFAULT_SIM_BUS_IDS, MODEL_UNSPECIFIED,
};
pub use protocol::{
    BodeData, ChirpJob, Command, ConnectionState, ControlMode, Event, ExcitationKind, JobSpec,
    LogLevel, Sample, Setpoint, StatusSnapshot, StopReason, TelemetryBatch, DEFAULT_RATE_HZ,
    DEFAULT_WATCHDOG, MAX_RATE_HZ, MIN_RATE_HZ,
};
pub use session::Session;
