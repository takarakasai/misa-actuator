//! Concrete CAN transports.
//!
//! Each submodule is gated to the platforms it can exist on, so a Windows
//! build never compiles (or even resolves) the SocketCAN path and a Linux
//! build never references `PCANBasic.dll`.

pub mod slcan;
pub mod usbcan;

#[cfg(target_os = "linux")]
pub mod socketcan;

#[cfg(windows)]
pub mod pcan;
