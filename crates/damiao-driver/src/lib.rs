//! Driver for the DAMIAO (达妙) CAN / CAN-FD servo motor family.
//!
//! Layered architecture:
//!
//! - [`bus::DamiaoBus`] — standard-CAN frame send/recv abstraction. Concrete
//!   implementations: [`SocketCanBus`] (classic CAN, 1 Mbps) and
//!   [`SocketCanFdBus`] (CAN-FD, 1–5 Mbps). The DAMIAO protocol is identical on
//!   both, so the choice is purely the physical layer.
//! - [`DamiaoMotor<B>`] — high-level driver, generic over the bus. Defaults to
//!   `DamiaoMotor<SocketCanBus>` so `DamiaoMotor::open("can0", 1, model)` works
//!   out of the box; use `open_fd` for a CAN-FD bus.
//! - `impl Actuator for DamiaoMotor<B>` — the unified
//!   [`misa_actuator::Actuator`] surface for the trait-object TUI.
//!
//! ```no_run
//! use damiao_driver::{DamiaoMotor, MotorModel};
//! use misa_actuator::{Actuator, RunMode};
//!
//! // Classic CAN bus (1 Mbps):
//! let mut motor = DamiaoMotor::open("can0", 1, MotorModel::Dm4310)?;
//! motor.set_run_mode(RunMode::Position)?;
//! motor.enable()?;
//! motor.set_position(1.57, 5.0)?;     // rad, max rad/s
//! motor.disable()?;
//! # Ok::<(), misa_actuator::Error>(())
//! ```
//!
//! ## Multi-motor buses and `MST_ID`
//!
//! DAMIAO feedback is returned on the motor's **Master ID** (`MST_ID`), which
//! ships as `0` on every motor. If several motors share one bus with `MST_ID =
//! 0` they all reply on CAN id `0x000` — frames collide during arbitration and
//! the driver can only tell them apart by the 4-bit id nibble in the payload.
//! So **give every motor on a shared bus a unique `MST_ID`.** The convention
//! used here is **`MST_ID = 0x10 + CAN_ID`** (send id `0x01..0x0N`, receive id
//! `0x11..0x1N`).
//!
//! Assign IDs **one motor at a time** (fresh motors all share `CAN_ID = 1`, so
//! you cannot address them individually until they differ):
//!
//! 1. connect a single motor;
//! 2. `damiao-cli -i can0 -m 1 set-id --new-can-id <N>` (MST_ID auto = `0x10+N`);
//! 3. power-cycle the motor (CAN_ID / MST_ID changes apply only after that);
//! 4. repeat for the next motor.
//!
//! ### Several motors on one bus
//!
//! Each motor handle owns its bus, so to put several motors on one wire wrap the
//! opened bus in [`Shared`] (an `Arc<Mutex<_>>` from `misa-actuator`) and give
//! each motor a clone. Drive them from one control loop per bus; different buses
//! (`can0`, `can1`) use independent `Shared` instances and run in parallel.
//!
//! ```no_run
//! use damiao_driver::{DamiaoMotor, MotorModel, Shared, SocketCanBus};
//! use misa_actuator::{Actuator, RunMode};
//!
//! let bus0 = Shared::new(SocketCanBus::open("can0")?);
//! // CAN_ID 1 / 2, MST_ID 0x11 / 0x12 (the 0x10 + CAN_ID convention):
//! let mut m1 = DamiaoMotor::with_bus_and_master(bus0.clone(), 1, 0x11, MotorModel::Dm4310);
//! let mut m2 = DamiaoMotor::with_bus_and_master(bus0.clone(), 2, 0x12, MotorModel::Dm4310);
//! for m in [&mut m1, &mut m2] {
//!     m.set_run_mode(RunMode::Mit)?;
//!     m.enable()?;
//! }
//! // single loop: each transaction briefly locks the shared bus
//! loop {
//!     m1.mit_control(0.0, 0.0, 0.0, 0.5, 0.0)?;
//!     m2.mit_control(0.0, 0.0, 0.0, 0.5, 0.0)?;
//! #   break;
//! }
//! # Ok::<(), misa_actuator::Error>(())
//! ```
//!
//! See `examples/multi_motor.rs` for a two-bus version. Per-motor *threads* on
//! the same bus also work but don't lock a request+reply as one transaction, so
//! the single-loop-per-bus pattern is preferred.

pub mod actuator;
pub mod bus;
pub mod driver;
pub mod error;
pub mod scan;

pub use bus::{CanFrame, DamiaoBus, SocketCanBus, SocketCanFdBus};
/// Re-exported for sharing one bus across motors — see the multi-motor docs.
pub use misa_actuator::Shared;
pub use driver::DamiaoMotor;
pub use error::{Error, Result};
pub use scan::{probe_one, scan_bus_on, ScanProgress};

/// Re-export of the protocol crate so consumers can drop down to raw frames.
pub use damiao_protocol as protocol;

pub use damiao_protocol::{
    ControlMode, ErrorCode, Feedback, Limits, MotorModel, Rid, DEFAULT_MASTER_ID,
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::CanFrame;
    use damiao_protocol::{build_mit_frame, ControlMode, Rid};
    use std::collections::VecDeque;
    use std::time::Duration;

    /// In-memory [`DamiaoBus`] that records every transmitted frame and replays
    /// a queue of canned replies, so the driver's wire behaviour can be tested
    /// without hardware.
    #[derive(Default)]
    struct MockBus {
        sent: Vec<CanFrame>,
        replies: VecDeque<CanFrame>,
    }

    impl MockBus {
        fn push_reply(&mut self, can_id: u16, data: [u8; 8]) {
            self.replies.push_back(CanFrame {
                can_id,
                data: data.to_vec(),
            });
        }
    }

    impl DamiaoBus for MockBus {
        fn send(&mut self, can_id: u16, data: &[u8]) -> Result<()> {
            self.sent.push(CanFrame {
                can_id,
                data: data.to_vec(),
            });
            Ok(())
        }
        fn recv(&mut self) -> Result<CanFrame> {
            self.replies
                .pop_front()
                .ok_or(Error::Timeout { motor_id: 0 })
        }
        fn set_timeout(&mut self, _t: Duration) -> Result<()> {
            Ok(())
        }
    }

    /// Build a feedback payload (master-id frame) the mock can replay, encoding
    /// `pos` (rad) in the position field.
    fn feedback_at(id: u8, err_nibble: u8, pos: f32) -> [u8; 8] {
        let (_, mit) = build_mit_frame(id, &MotorModel::Dm4310.limits(), pos, 0.0, 0.0, 0.0, 0.0);
        [
            (err_nibble << 4) | (id & 0x0F),
            mit[0], // pos high
            mit[1], // pos low
            mit[2], // vel high
            mit[3], // vel/torque packed
            mit[7], // torque low
            40,     // T_MOS
            50,     // T_Rotor
        ]
    }

    /// Feedback at the origin.
    fn feedback_payload(id: u8, err_nibble: u8) -> [u8; 8] {
        feedback_at(id, err_nibble, 0.0)
    }

    #[test]
    fn enable_then_mit_sends_expected_frames_and_parses_feedback() {
        let mut motor = DamiaoMotor::with_bus(MockBus::default(), 1, MotorModel::Dm4310);

        // enable() may consume one feedback reply; provide one.
        motor.bus().push_reply(0x00, feedback_payload(1, 0x1));
        let _ = DamiaoMotor::enable(&mut motor).unwrap();

        // MIT control: queue a feedback reply on the master id (0).
        motor.bus().push_reply(0x00, feedback_payload(1, 0x1));
        let fb = DamiaoMotor::mit_control(&mut motor, 0.0, 0.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(fb.motor_id, 1);
        assert_eq!(fb.t_mos, 40.0);
        assert_eq!(fb.t_rotor, 50.0);

        // The enable frame and a MIT frame should both have gone out on CAN_ID=1.
        let sent = &motor.bus().sent;
        assert_eq!(sent[0].can_id, 0x001);
        assert_eq!(sent[0].data, vec![0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]);
        assert_eq!(sent[1].can_id, 0x001); // MIT command id == CAN_ID
    }

    #[test]
    fn mit_control_requires_enable() {
        let mut motor = DamiaoMotor::with_bus(MockBus::default(), 1, MotorModel::Dm4310);
        let err = DamiaoMotor::mit_control(&mut motor, 0.0, 0.0, 0.0, 0.0, 0.0).unwrap_err();
        assert!(matches!(err, Error::NotEnabled { motor_id: 1 }));
    }

    #[test]
    fn soft_zero_offsets_commands_and_feedback() {
        let mut motor = DamiaoMotor::with_bus(MockBus::default(), 1, MotorModel::Dm4310);
        motor.bus().push_reply(0x00, feedback_payload(1, 0x1));
        DamiaoMotor::enable(&mut motor).unwrap();

        // Pretend the motor is physically at +1.0 rad and re-zero there.
        motor.set_soft_zero(1.0);
        motor.bus().push_reply(0x00, feedback_at(1, 0x1, 1.0)); // raw pos = 1.0
        let fb = DamiaoMotor::mit_control(&mut motor, 0.0, 0.0, 0.0, 0.0, 0.0).unwrap();
        // Reported position is raw - soft_zero = 0.
        assert!(fb.position.abs() < 0.05, "reported {}", fb.position);
    }

    #[test]
    fn set_zero_nvm_sends_magic_frame_and_clears_soft_zero() {
        let mut motor = DamiaoMotor::with_bus(MockBus::default(), 1, MotorModel::Dm4310);
        motor.set_soft_zero(2.0);
        motor.bus().push_reply(0x00, feedback_payload(1, 0x0)); // try_recv_feedback
        DamiaoMotor::set_zero_nvm(&mut motor).unwrap();
        assert_eq!(motor.soft_zero(), 0.0);
        let last = motor.bus().sent.last().unwrap();
        assert_eq!(last.can_id, 0x001);
        assert_eq!(last.data, vec![0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]);
    }

    #[test]
    fn switch_mode_writes_ctrl_mode_register_and_verifies() {
        let mut motor = DamiaoMotor::with_bus(MockBus::default(), 1, MotorModel::Dm4310);
        // The motor replies on its Master ID (0x000 by default), NOT on 0x7FF.
        // write_register_int drains the write echo; read_register matches the
        // read reply by content (rid=10, value=PosVel(2)).
        motor.bus().push_reply(0x000, [0x01, 0x00, 0x55, Rid::CTRL_MODE, 0x02, 0, 0, 0]); // write echo
        motor.bus().push_reply(0x000, [0x01, 0x00, 0x33, Rid::CTRL_MODE, 0x02, 0, 0, 0]); // read reply
        DamiaoMotor::switch_mode(&mut motor, ControlMode::PosVel).unwrap();
        assert_eq!(motor.mode(), ControlMode::PosVel);

        // The first 0x7FF frame sent must be a CTRL_MODE(=10) write of value 2.
        let write = motor
            .bus()
            .sent
            .iter()
            .find(|f| f.can_id == 0x7FF && f.data[2] == 0x55)
            .expect("a register write was sent");
        assert_eq!(write.data[3], Rid::CTRL_MODE);
        assert_eq!(write.data[4], 0x02);
    }
}
