//! Shared library surface for the `misa-actuator-tui` crate's binaries.
//!
//! Both `misa-actuator-tui` (single-motor command console) and
//! `misa-actuator-monitor` (multi-motor read-only dashboard) build on the
//! same driver-agnostic actuator factory.
//!
//! That factory now lives in [`misa_actuator_core`], because the GUI needs it
//! too and a GUI has no business depending on the TUI crate. It is re-exported
//! here so `misa_actuator_tui::factory::…` keeps working.

pub use misa_actuator_core::factory;
