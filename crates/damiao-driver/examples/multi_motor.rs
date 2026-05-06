//! Drive several DAMIAO motors split across two buses from a single loop.
//!
//! Wiring assumed: `can0` and `can1` each carry two DM-J4310 motors with
//! `CAN_ID` 1 and 2 and the `MST_ID = 0x10 + CAN_ID` convention (so MST_IDs
//! `0x11` / `0x12`). Assign IDs first with `damiao-cli set-id` (one motor at a
//! time), then:
//!
//! ```text
//! cargo run -p damiao-driver --example multi_motor
//! ```
//!
//! Each bus is wrapped in a `Shared<_>` so its two motors share one socket;
//! transactions are serialized by the mutex. The two buses are independent.

use std::error::Error;
use std::thread;
use std::time::Duration;

use damiao_driver::{DamiaoMotor, MotorModel, Shared, SocketCanBus};
use misa_actuator::{Actuator, RunMode};

fn main() -> Result<(), Box<dyn Error>> {
    // One shared bus per physical interface.
    let bus0 = Shared::new(SocketCanBus::open("can0")?);
    let bus1 = Shared::new(SocketCanBus::open("can1")?);

    // Several motors per bus; each gets a clone of its bus and a unique MST_ID.
    let mut motors: Vec<Box<dyn Actuator + Send>> = vec![
        Box::new(DamiaoMotor::with_bus_and_master(bus0.clone(), 1, 0x11, MotorModel::Dm4310)),
        Box::new(DamiaoMotor::with_bus_and_master(bus0.clone(), 2, 0x12, MotorModel::Dm4310)),
        Box::new(DamiaoMotor::with_bus_and_master(bus1.clone(), 1, 0x11, MotorModel::Dm4310)),
        Box::new(DamiaoMotor::with_bus_and_master(bus1.clone(), 2, 0x12, MotorModel::Dm4310)),
    ];

    for m in motors.iter_mut() {
        m.set_run_mode(RunMode::Mit)?;
        m.enable()?;
    }

    // Single control loop: each tick, command every motor and read its
    // feedback. A motor's transaction locks only its own bus, so the two buses
    // run independently and same-bus motors are cleanly serialized.
    for _tick in 0..500 {
        for (i, m) in motors.iter_mut().enumerate() {
            // Light damping hold (kp=0, kd=0.5, no feed-forward torque).
            let fb = m.mit_control(0.0, 0.0, 0.0, 0.5, 0.0)?;
            if _tick % 100 == 0 {
                println!("motor[{i}] pos={:+.3} rad", fb.position_rad);
            }
        }
        thread::sleep(Duration::from_millis(2)); // ~2 ms loop period
    }

    for m in motors.iter_mut() {
        m.disable()?;
    }
    Ok(())
}
