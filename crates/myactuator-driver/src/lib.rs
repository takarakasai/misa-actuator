//! CAN driver for MyActuator RMD servo motors (CAN protocol V3),
//! implementing [`misa_actuator::Actuator`].
//!
//! The transport comes from the interface string: `can0` (Linux SocketCAN),
//! `pcan:usb1` (PEAK adapter on Windows) or `slcan:COM5` (USB-CAN dongle).
//!
//! ```no_run
//! use myactuator_driver::{MotorConfig, MyActuatorMotor};
//! use misa_actuator::Actuator;
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! // RMD-X6 V3 on can0, motor id 1, Kt from the datasheet.
//! let mut motor = MyActuatorMotor::open("can0", 1, MotorConfig::new(0.83))?;
//! motor.enable()?;
//! motor.set_position(1.57, 2.0)?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Multi-motor buses
//!
//! Wrap one opened [`AnyCanBus`] in [`misa_actuator::Shared`] and hand each
//! [`MyActuatorMotor`] a clone; replies are disambiguated by the per-motor
//! reply id (`0x240 + ID`).

pub mod actuator;
pub mod bus;
pub mod driver;
pub mod error;
pub mod scan;

pub use bus::{AnyCanBus, CanBus, CanFrame, MyActuatorBus};
pub use driver::{MotorConfig, MotorFeedback, MotorStatus, MyActuatorMotor};
pub use error::{Error, Result};
pub use myactuator_protocol::{ErrorState, MotionFeedback, ParamIndex, Status1, Status2};
pub use scan::{dump_bus_on, probe_one, scan_bus_on};

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;
    use std::time::Duration;

    use misa_actuator::Actuator;
    use myactuator_protocol::{can_id, DATA_LEN};

    use crate::bus::{CanFrame, MyActuatorBus};
    use crate::driver::{MotorConfig, MyActuatorMotor};
    use crate::error::{Error, Result};

    /// In-memory bus: records every sent frame and synthesizes a reply the way
    /// a V3 motor would (echoing the command byte on `0x240 + id`).
    #[derive(Default)]
    struct MockBus {
        sent: Vec<(u16, [u8; DATA_LEN])>,
        rx: VecDeque<CanFrame>,
        /// Angle returned for `0x92` reads, 0.01°/LSB.
        multi_turn_centideg: i32,
    }

    impl MyActuatorBus for MockBus {
        fn send(&mut self, tx_id: u16, data: &[u8]) -> Result<()> {
            let mut payload = [0u8; DATA_LEN];
            payload[..data.len()].copy_from_slice(data);
            self.sent.push((tx_id, payload));

            // Synthesize the reply.
            let motor_id = if (0x141..=0x160).contains(&tx_id) {
                (tx_id - 0x140) as u8
            } else if (0x401..=0x420).contains(&tx_id) {
                let id = (tx_id - 0x400) as u8;
                // Motion-mode reply: data[0] is the replying CAN address,
                // then mid-scale codes ≈ zero pos/vel/torque (see
                // myactuator_protocol::motion for the reply's byte layout,
                // shifted one byte later than the command's).
                self.rx.push_back(CanFrame {
                    can_id: can_id::motion_reply_id(id),
                    data: vec![id, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 0, 0],
                });
                return Ok(());
            } else {
                return Ok(());
            };

            let mut reply = [0u8; DATA_LEN];
            reply[0] = data[0];
            if data[0] == 0x92 {
                reply[4..8].copy_from_slice(&self.multi_turn_centideg.to_le_bytes());
            }
            self.rx.push_back(CanFrame {
                can_id: can_id::reply_id(motor_id),
                data: reply.to_vec(),
            });
            Ok(())
        }

        fn recv(&mut self) -> Result<CanFrame> {
            self.rx.pop_front().ok_or(Error::Timeout { motor_id: 0 })
        }

        fn set_timeout(&mut self, _timeout: Duration) -> Result<()> {
            Ok(())
        }
    }

    fn motor() -> MyActuatorMotor<MockBus> {
        MyActuatorMotor::with_bus(MockBus::default(), 1, MotorConfig::current_units()).unwrap()
    }

    fn sent_cmds(m: &mut MyActuatorMotor<MockBus>) -> Vec<u8> {
        m.bus().sent.iter().map(|(_, d)| d[0]).collect()
    }

    #[test]
    fn rejects_out_of_range_motor_id() {
        assert!(MyActuatorMotor::with_bus(MockBus::default(), 0, MotorConfig::current_units())
            .is_err());
        assert!(MyActuatorMotor::with_bus(MockBus::default(), 33, MotorConfig::current_units())
            .is_err());
    }

    #[test]
    fn set_velocity_encodes_centi_dps() {
        let mut m = motor();
        // 1 rad/s = 57.2958 dps = 5729 centi-dps (truncated).
        m.set_velocity(1.0).unwrap();
        let (tx_id, frame) = m.bus().sent[0];
        assert_eq!(tx_id, 0x141);
        assert_eq!(frame[0], 0xA2);
        let centi_dps = i32::from_le_bytes(frame[4..8].try_into().unwrap());
        assert!((centi_dps - 5729).abs() <= 1, "centi_dps={centi_dps}");
    }

    #[test]
    fn set_torque_in_current_units_sends_amps() {
        let mut m = motor();
        // current_units: Kt = 1 → 2.5 "N·m" = 2.5 A = 250 centi-amps.
        m.set_torque(2.5).unwrap();
        let (_, frame) = m.bus().sent[0];
        assert_eq!(frame[0], 0xA1);
        let centi_amps = i16::from_le_bytes(frame[4..6].try_into().unwrap());
        assert_eq!(centi_amps, 250);
    }

    #[test]
    /// A control reply's position is 1°/LSB unless the fine reading is asked
    /// for, and asking for it costs a round trip.
    ///
    /// This is what made a shaft appear to jump a degree when streaming started
    /// and back when it stopped: `measure` substitutes the 0.01° `0x92` value
    /// and control replies did not, so the same stationary shaft read
    /// differently depending on which path the caller was on (2026-08-08).
    #[test]
    fn fine_position_costs_a_read_and_buys_resolution() {
        let mut m = motor();
        m.bus().multi_turn_centideg = 0;
        m.rezero().unwrap();

        // The multi-turn register says 17.19° (0.3 rad); the synthesized
        // control reply carries whatever coarse angle the mock inlines, which
        // is a different number.
        m.bus().multi_turn_centideg = 1719;

        let coarse = MyActuatorMotor::set_position(&mut m, 0.3, 1.0).unwrap();
        let sent_before = m.bus().sent.len();

        MyActuatorMotor::set_fine_position(&mut m, true);
        let fine = MyActuatorMotor::set_position(&mut m, 0.3, 1.0).unwrap();
        let sent_after = m.bus().sent.len();

        // The fine reading is the multi-turn value: 17.19° = 0.3 rad.
        assert!(
            (fine.position_rad - 0.3).abs() < 1e-3,
            "fine position {} should be the 0x92 reading",
            fine.position_rad
        );
        // And it is not what the control reply alone said.
        assert!(
            (fine.position_rad - coarse.position_rad).abs() > 1e-3,
            "coarse {} and fine {} should differ",
            coarse.position_rad,
            fine.position_rad
        );

        // Two frames for the fine command against one for the coarse: the cost
        // this is off by default for.
        assert_eq!(
            sent_after - sent_before,
            2,
            "the fine reading should cost one extra transaction"
        );
    }

    #[test]
    fn set_position_requires_anchor_then_offsets_by_zero() {
        let mut m = motor();
        m.bus().multi_turn_centideg = 18_000; // motor sits at +180.00°
        assert!(matches!(
            MyActuatorMotor::set_position(&mut m, 0.0, 1.0),
            Err(Error::PositionNotAnchored { .. })
        ));

        m.rezero().unwrap();
        // Target 0 rad → absolute 18000 centideg (the anchor).
        MyActuatorMotor::set_position(&mut m, 0.0, 1.0).unwrap();
        let (_, frame) = *m.bus().sent.last().unwrap();
        assert_eq!(frame[0], 0xA4);
        let centideg = i32::from_le_bytes(frame[4..8].try_into().unwrap());
        assert_eq!(centideg, 18_000);
        // Speed cap 1 rad/s = 57 dps.
        let dps = u16::from_le_bytes(frame[2..4].try_into().unwrap());
        assert_eq!(dps, 57);
    }

    #[test]
    fn actuator_set_position_auto_anchors_on_first_use() {
        let mut m = motor();
        m.bus().multi_turn_centideg = 9_000;
        let fb = Actuator::set_position(&mut m, 0.0, 1.0).unwrap();
        let cmds = sent_cmds(&mut m);
        assert!(cmds.contains(&0x92), "rezero read expected: {cmds:02X?}");
        assert!(cmds.contains(&0xA4));
        // Mock replies angle_deg = 0 → 0° raw − 90° zero = −π/2 rad.
        assert!((fb.position_rad + std::f32::consts::FRAC_PI_2).abs() < 1e-3);
    }

    #[test]
    fn enable_anchors_and_holds_position() {
        let mut m = motor();
        m.enable().unwrap();
        let cmds = sent_cmds(&mut m);
        assert!(cmds.contains(&0x92));
        assert!(cmds.contains(&0xA4));
        assert!(m.is_enabled_hint());
    }

    #[test]
    fn disable_sends_stop() {
        let mut m = motor();
        m.disable().unwrap();
        assert!(sent_cmds(&mut m).contains(&0x81));
        assert!(!m.is_enabled_hint());
    }

    #[test]
    fn measure_combines_92_and_9c() {
        let mut m = motor();
        m.bus().multi_turn_centideg = 36_000; // +360.00°
        m.rezero().unwrap();
        m.bus().multi_turn_centideg = 54_000; // moved to +540.00°
        let fb = MyActuatorMotor::measure(&mut m).unwrap();
        // +180° from the anchor = π rad.
        assert!((fb.position_rad - std::f32::consts::PI).abs() < 1e-3);
        let cmds = sent_cmds(&mut m);
        assert!(cmds.contains(&0x9C));
    }

    #[test]
    fn mit_control_uses_motion_channel() {
        let mut m = motor();
        let fb = m.mit_control(0.0, 0.0, 10.0, 1.0, 0.0).unwrap();
        let (tx_id, _) = m.bus().sent[0];
        assert_eq!(tx_id, 0x401);
        // Mid-scale mock reply ≈ zero everywhere.
        assert!(fb.position_rad.abs() < 1e-3);
        assert!(fb.velocity_rad_per_s.abs() < 0.02);
        assert!(fb.torque_nm.abs() < 0.01);
    }

    #[test]
    fn scan_finds_only_valid_ids() {
        let mut m = motor();
        let found = m.scan_bus(1..=3, Duration::from_millis(5)).unwrap();
        assert_eq!(found, vec![1, 2, 3]);
        // Out-of-range ids are skipped silently.
        let mut m = motor();
        let found = m.scan_bus(30..=40, Duration::from_millis(5)).unwrap();
        assert_eq!(found, vec![30, 31, 32]);
    }

    #[test]
    fn read_status_maps_voltage() {
        let mut m = motor();
        let st = Actuator::read_status(&mut m).unwrap();
        assert_eq!(st.voltage_v, 0.0);
        assert!(!st.error.any());
    }

    #[test]
    fn read_pid_sends_one_0x30_per_index() {
        use myactuator_protocol::PidIndex;
        let mut m = motor();
        m.read_pid().unwrap();
        let sent = m.bus().sent.clone();
        for index in PidIndex::ALL {
            assert!(
                sent.iter().any(|(_, d)| d[0] == 0x30 && d[1] == index as u8),
                "missing 0x30 read for index {index:?}"
            );
        }
    }

    #[test]
    fn read_acceleration_carries_index() {
        use myactuator_protocol::AccelIndex;
        let mut m = motor();
        m.read_acceleration(AccelIndex::SpeedDecel).unwrap();
        let (_, frame) = m.bus().sent[0];
        assert_eq!(frame[0], 0x42);
        assert_eq!(frame[1], AccelIndex::SpeedDecel as u8);
    }

    #[test]
    fn read_multi_turn_encoder_sends_0x60() {
        let mut m = motor();
        m.read_multi_turn_encoder().unwrap();
        assert!(sent_cmds(&mut m).contains(&0x60));
    }

    #[test]
    fn read_run_mode_sends_0x70() {
        let mut m = motor();
        m.read_run_mode().unwrap();
        assert!(sent_cmds(&mut m).contains(&0x70));
    }

    #[test]
    fn read_status3_sends_0x9d() {
        let mut m = motor();
        m.read_status3().unwrap();
        assert!(sent_cmds(&mut m).contains(&0x9D));
    }

    #[test]
    fn read_uptime_sends_0xb1() {
        let mut m = motor();
        m.read_uptime_ms().unwrap();
        assert!(sent_cmds(&mut m).contains(&0xB1));
    }

    #[test]
    fn read_param_sends_0xc0_with_index_and_read_flag() {
        use myactuator_protocol::ParamIndex;
        let mut m = motor();
        m.read_param(ParamIndex::OverVoltage).unwrap();
        let (_, frame) = m.bus().sent[0];
        assert_eq!(frame[0], 0xC0);
        assert_eq!(frame[2], ParamIndex::OverVoltage as u8);
        assert_eq!(frame[3], 0x01);
    }

    #[test]
    fn write_param_sends_0xc0_with_write_flag_and_value() {
        use myactuator_protocol::ParamIndex;
        let mut m = motor();
        m.write_param(ParamIndex::BrakeMode, 1.0).unwrap();
        let (_, frame) = m.bus().sent[0];
        assert_eq!(frame[0], 0xC0);
        assert_eq!(frame[2], ParamIndex::BrakeMode as u8);
        assert_eq!(frame[3], 0x00);
        let v = f32::from_le_bytes(frame[4..8].try_into().unwrap());
        assert_eq!(v, 1.0);
    }

    #[test]
    fn commit_params_sends_0xc1() {
        let mut m = motor();
        m.commit_params().unwrap();
        assert!(sent_cmds(&mut m).contains(&0xC1));
    }
}
