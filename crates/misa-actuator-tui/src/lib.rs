//! Shared library surface for the `misa-actuator-tui` crate's binaries.
//!
//! Both `misa-actuator-tui` (single-motor command console) and
//! `misa-actuator-monitor` (multi-motor read-only dashboard) build on the
//! same driver-agnostic actuator factory.

pub mod factory;
