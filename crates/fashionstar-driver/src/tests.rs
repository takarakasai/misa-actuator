//! Bus-level tests against [`MockTransport`] — no hardware.
//!
//! A small fake servo bank answers requests the way the SDK expects servos
//! to: one frame per reply, the id inside the params, a sync monitor
//! answered by one ordinary monitor frame per listed id.

use std::collections::HashMap;
use std::time::{Duration, Instant};

use fashionstar_protocol::{
    encode_response, request, try_decode, Code, Monitor as RawMonitor, StopMode,
    HEADER_REQUEST, MAX_FRAME, MONITOR_INVALID_ANGLE,
};
use misa_actuator::{Actuator, Error as MisaError, RunMode, Shared};

use crate::*;

fn response(code: Code, params: &[u8]) -> Vec<u8> {
    let mut buf = [0u8; MAX_FRAME];
    let n = encode_response(code.code(), params, &mut buf).unwrap();
    buf[..n].to_vec()
}

fn monitor_params(m: &RawMonitor) -> Vec<u8> {
    let mut p = vec![m.id];
    p.extend(m.voltage_mv.to_le_bytes());
    p.extend(m.current_ma.to_le_bytes());
    p.extend(m.power_mw.to_le_bytes());
    p.extend(m.temp_raw.to_le_bytes());
    p.push(m.status);
    p.extend(m.angle.to_le_bytes());
    p.extend(m.turns.to_le_bytes());
    p
}

fn raw_monitor(id: u8, angle: i32) -> RawMonitor {
    RawMonitor {
        id,
        voltage_mv: 12_000,
        current_ma: 150,
        power_mw: 1_800,
        temp_raw: 2048,
        status: 0,
        angle,
        turns: 0,
    }
}

/// A bank of fake servos keyed by id, each with a monitor state.
#[derive(Clone, Default)]
struct Bank {
    servos: HashMap<u8, RawMonitor>,
    /// Answer sync-monitor replies in reverse id order.
    reverse: bool,
    /// Prepend an echo of the request (half-duplex adapter behaviour).
    echo: bool,
}

impl Bank {
    fn arm(ids: &[u8]) -> Self {
        let servos = ids
            .iter()
            .map(|&id| (id, raw_monitor(id, id as i32 * 100)))
            .collect();
        Self { servos, ..Default::default() }
    }

    fn respond(&self, req: &[u8]) -> Vec<u8> {
        let (f, _) = try_decode(HEADER_REQUEST, req).expect("host sent a malformed frame");
        let mut out = if self.echo { req.to_vec() } else { Vec::new() };
        let monitor = |id: u8| self.servos.get(&id).map(|m| response(Code::QueryMonitor, &monitor_params(m)));
        match Code::from_u8(f.code).unwrap() {
            Code::Ping => {
                if self.servos.contains_key(&f.params[0]) {
                    out.extend(response(Code::Ping, &f.params[..1]));
                }
            }
            Code::QueryMonitor => out.extend(monitor(f.params[0]).unwrap_or_default()),
            Code::QueryAngleMturn => {
                if let Some(m) = self.servos.get(&f.params[0]) {
                    let mut p = vec![m.id];
                    p.extend(m.angle.to_le_bytes());
                    p.extend(m.turns.to_le_bytes());
                    out.extend(response(Code::QueryAngleMturn, &p));
                }
            }
            Code::QueryAngle => {
                if let Some(m) = self.servos.get(&f.params[0]) {
                    let mut p = vec![m.id];
                    p.extend((m.angle as i16).to_le_bytes());
                    out.extend(response(Code::QueryAngle, &p));
                }
            }
            Code::SyncCommand => {
                assert_eq!(&f.params[..2], &[22, 1], "only sync monitor is answered");
                let n = f.params[2] as usize;
                let mut ids: Vec<u8> = f.params[3..3 + n].to_vec();
                if self.reverse {
                    ids.reverse();
                }
                for id in ids {
                    out.extend(monitor(id).unwrap_or_default());
                }
            }
            _ => {} // fire-and-forget commands
        }
        out
    }

    fn into_bus(self, timeout: Duration) -> FashionStarBus<MockTransport> {
        let mut t = MockTransport::with_responder(move |req| self.respond(req));
        t.set_max_read(3); // force partial frames through the parser
        FashionStarBus::from_transport(t, timeout)
    }
}

const T: Duration = Duration::from_millis(20);
const LEADER: [u8; 7] = [0, 1, 2, 3, 4, 5, 6];

#[test]
fn ping_present_and_absent() {
    let mut bus = Bank::arm(&[3]).into_bus(T);
    assert!(bus.ping(3).unwrap());
    assert!(!bus.ping(4).unwrap());
    // PING request from the SDK for id 3: 12 4C 01 01 03 63
    assert_eq!(bus.transport().written()[0], vec![0x12, 0x4C, 0x01, 0x01, 0x03, 0x63]);
}

#[test]
fn angle_queries_convert_to_rad() {
    let mut bank = Bank::arm(&[2]);
    bank.servos.get_mut(&2).unwrap().angle = -900; // -90.0°
    bank.servos.get_mut(&2).unwrap().turns = -1;
    let mut bus = bank.into_bus(T);
    assert!((bus.query_angle(2).unwrap() + std::f32::consts::FRAC_PI_2).abs() < 1e-6);
    let mt = bus.query_angle_multiturn_raw(2).unwrap();
    assert_eq!((mt.angle, mt.turns), (-900, -1));
    assert!((bus.query_angle_multiturn(2).unwrap() + std::f32::consts::FRAC_PI_2).abs() < 1e-6);
}

#[test]
fn monitor_si_units() {
    let mut bus = Bank::arm(&[1]).into_bus(T);
    let m = bus.monitor(1).unwrap();
    assert_eq!(m.id, 1);
    assert!((m.voltage_v - 12.0).abs() < 1e-6);
    assert!((m.current_a - 0.15).abs() < 1e-6);
    assert!((m.power_w - 1.8).abs() < 1e-6);
    assert!((m.temperature_c - 25.0).abs() < 1e-3);
    assert!((m.angle_deg() - 10.0).abs() < 1e-6);
    assert!((m.angle_rad - 10f32.to_radians()).abs() < 1e-6);
}

#[test]
fn monitor_sentinel_is_an_error() {
    let mut bank = Bank::arm(&[1]);
    bank.servos.get_mut(&1).unwrap().angle = MONITOR_INVALID_ANGLE;
    let mut bus = bank.into_bus(T);
    assert!(matches!(bus.monitor(1), Err(Error::InvalidReading { servo_id: 1 })));
    // ...but the raw path still returns it.
    assert!(!bus.monitor_raw(1).unwrap().is_valid());
}

#[test]
fn sync_monitor_whole_arm_any_order_with_echo() {
    let mut bank = Bank::arm(&LEADER);
    bank.reverse = true;
    bank.echo = true;
    let mut bus = bank.into_bus(Duration::from_millis(500));
    let t0 = Instant::now();
    let got = bus.sync_monitor(&LEADER).unwrap();
    // Returns as soon as the 7th reply lands, not at the deadline.
    assert!(t0.elapsed() < Duration::from_millis(250));
    assert_eq!(got.len(), 7);
    for (i, m) in got.iter().enumerate() {
        let m = m.expect("every servo answered");
        assert_eq!(m.id, LEADER[i], "slot follows the request, not arrival order");
        assert_eq!(m.raw.angle, i as i32 * 100);
    }
    // One request frame: the SDK's sync monitor for ids 0..=6.
    assert_eq!(bus.transport().written().len(), 1);
    assert_eq!(
        bus.transport().written()[0],
        vec![0x12, 0x4C, 0x19, 0x0A, 0x16, 0x01, 0x07, 0, 1, 2, 3, 4, 5, 6, 0xB4]
    );
    // The echoed request was skipped as garbage.
    assert_eq!(bus.discarded_bytes(), 15);
}

#[test]
fn sync_monitor_chunked_splits_requests_and_keeps_order() {
    let mut bank = Bank::arm(&LEADER);
    bank.reverse = true;
    let mut bus = bank.into_bus(Duration::from_millis(500));
    let got = bus.sync_monitor_chunked(&LEADER, 3).unwrap();
    assert_eq!(got.len(), 7);
    for (i, m) in got.iter().enumerate() {
        assert_eq!(m.expect("every servo answered").id, LEADER[i]);
    }
    // 3 + 3 + 1: three sync requests, each listing its own ids.
    let written = bus.transport().written();
    assert_eq!(written.len(), 3);
    assert_eq!(&written[0][6..10], &[3, 0, 1, 2]);
    assert_eq!(&written[1][6..10], &[3, 3, 4, 5]);
    assert_eq!(&written[2][6..8], &[1, 6]);
}

#[test]
fn sync_monitor_missing_and_invalid_become_none() {
    let mut bank = Bank::arm(&LEADER);
    bank.servos.remove(&4);
    bank.servos.get_mut(&5).unwrap().angle = MONITOR_INVALID_ANGLE;
    let mut bus = bank.into_bus(T);
    let t0 = Instant::now();
    let got = bus.sync_monitor(&LEADER).unwrap();
    // A missing reply costs the full timeout — the bus cannot know it
    // is not merely late.
    assert!(t0.elapsed() >= T);
    let present: Vec<bool> = got.iter().map(Option::is_some).collect();
    assert_eq!(present, [true, true, true, true, false, false, true]);
}

#[test]
fn stale_input_is_not_taken_as_the_answer() {
    let mut bus = Bank::arm(&[2]).into_bus(T);
    // A late monitor reply from a previous cycle, with a different angle.
    let stale = response(Code::QueryMonitor, &monitor_params(&raw_monitor(2, 9999)));
    bus.transport_mut().inject(&stale);
    assert_eq!(bus.monitor(2).unwrap().raw.angle, 200);
}

#[test]
fn unrelated_frames_are_skipped() {
    let mut replies = response(Code::Ping, &[7]); // someone else's reply
    replies.extend([0xDE, 0xAD]); // noise
    replies.extend(response(Code::QueryMonitor, &monitor_params(&raw_monitor(1, 42))));
    let t = MockTransport::scripted(vec![replies]);
    let mut bus = FashionStarBus::from_transport(t, T);
    assert_eq!(bus.monitor(1).unwrap().raw.angle, 42);
}

#[test]
fn timeout_is_reported() {
    let mut bus = FashionStarBus::from_transport(MockTransport::silent(), T);
    assert!(matches!(bus.monitor(3), Err(Error::Timeout { servo_id: 3 })));
    assert!(bus.sync_monitor(&[0, 1]).unwrap().iter().all(Option::is_none));
    assert!(bus.sync_monitor(&[]).unwrap().is_empty());
}

#[test]
fn fire_and_forget_bytes_match_sdk() {
    let mut bus = FashionStarBus::from_transport(MockTransport::silent(), T);
    bus.stop(BROADCAST_ID, StopMode::Release).unwrap();
    bus.reset_multi_turn(4).unwrap();
    bus.set_origin_point(5).unwrap();
    bus.set_angle(0, -std::f32::consts::FRAC_PI_2, Duration::from_millis(1000))
        .unwrap();
    bus.set_angle_multiturn_by_interval(
        2,
        360f32.to_radians(),
        Duration::from_millis(100),
        Duration::from_millis(50),
        Duration::from_millis(50),
    )
    .unwrap();
    let w = bus.transport().written();
    assert_eq!(w[0], vec![0x12, 0x4C, 0x18, 0x04, 0xFF, 0x10, 0x00, 0x00, 0x89]);
    assert_eq!(w[1], vec![0x12, 0x4C, 0x11, 0x01, 0x04, 0x74]);
    assert_eq!(w[2], vec![0x12, 0x4C, 0x17, 0x02, 0x05, 0x00, 0x7C]);
    assert_eq!(w[3], vec![0x12, 0x4C, 0x08, 0x07, 0x00, 0x7C, 0xFC, 0xE8, 0x03, 0x00, 0x00, 0xD0]);
    assert_eq!(
        w[4],
        vec![
            0x12, 0x4C, 0x0E, 0x0F, 0x02, 0x10, 0x0E, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x32,
            0x00, 0x32, 0x00, 0x00, 0x00, 0x63
        ]
    );
}

#[test]
fn sdk_clamps_are_applied() {
    // t_acc/t_dec below 20 ms are raised; interval raised to t_acc + t_dec.
    let c = mturn_by_interval(1, 0.0, Duration::ZERO, Duration::from_millis(5), Duration::ZERO);
    assert_eq!((c.t_acc_ms, c.t_dec_ms, c.interval_ms), (20, 20, 40));
    // Angle beyond ±1024 turns is clamped.
    let c = mturn_by_interval(1, 1e9, Duration::from_secs(1), MIN_ACCEL_TIME, MIN_ACCEL_TIME);
    assert_eq!(c.angle, 3_686_400);
    // Single-turn target past 180° is clamped to 1800.
    let mut bus = FashionStarBus::from_transport(MockTransport::silent(), T);
    bus.set_angle(0, 4.0, Duration::ZERO).unwrap();
    let w = &bus.transport().written()[0];
    assert_eq!(i16::from_le_bytes([w[5], w[6]]), 1800);
}

#[test]
fn sync_follower_command() {
    let mut bus = FashionStarBus::from_transport(MockTransport::silent(), T);
    let cmds: Vec<_> = [(0u8, 10.0f32), (1, -10.0)]
        .iter()
        .map(|&(id, deg)| {
            mturn_by_interval(id, deg.to_radians(), Duration::from_millis(100), Duration::from_millis(50), Duration::from_millis(50))
        })
        .collect();
    bus.sync_set_angle_multiturn_by_interval(&cmds).unwrap();
    let mut expect = [0u8; MAX_FRAME];
    let n = request::encode_sync(&cmds, &mut expect).unwrap();
    assert_eq!(bus.transport().written()[0], expect[..n].to_vec());
    // Matches the SDK vector for (0, +100) / (1, -100) in 0.1°.
    assert_eq!(expect[n - 1], 0x45);
}

#[test]
fn read_and_write_data() {
    let t = MockTransport::scripted(vec![
        response(Code::ReadData, &[1, 1, 0xE0, 0x2E]),
        response(Code::WriteData, &[1, 33, 1]),
        response(Code::ReadData, &[1, 2, 0x00, 0x00]),
    ]);
    let mut bus = FashionStarBus::from_transport(t, T);
    assert_eq!(bus.read_data(1, 1).unwrap(), vec![0xE0, 0x2E]);
    assert!(bus.write_data(1, 33, &[1]).unwrap());
    assert!(matches!(bus.read_data(1, 1), Err(Error::UnexpectedResponse { .. })));
}

#[test]
fn actuator_over_shared_bus() {
    let bus = Shared::new(Bank::arm(&[0, 1]).into_bus(T));
    let mut j0 = FashionStarServo::new(bus.clone(), 0);
    let mut j1 = FashionStarServo::new(bus.clone(), 1);

    let fb = j1.enable().unwrap();
    assert!(j1.is_enabled_hint());
    assert!((fb.position_rad - 10f32.to_radians()).abs() < 1e-6);

    // From 10° to 1 rad at 2 rad/s: (1 - 0.17453) / 2 = 0.41273 s -> 412 ms.
    j1.set_position(1.0, 2.0).unwrap();
    j0.disable().unwrap();

    let written = bus.lock().transport().written().to_vec();
    // enable: stop(hold, power 0) + monitor
    assert_eq!(written[0], vec![0x12, 0x4C, 0x18, 0x04, 0x01, 0x11, 0x00, 0x00, 0x8C]);
    // set_position: monitor, then SET_SERVO_ANGLE_MTURN (13)
    let mv = &written[3];
    assert_eq!(mv[2], 13);
    assert_eq!(i32::from_le_bytes([mv[5], mv[6], mv[7], mv[8]]), 573); // 1 rad = 57.3°
    assert_eq!(u32::from_le_bytes([mv[9], mv[10], mv[11], mv[12]]), 412);
    // disable: stop(release) for id 0
    assert_eq!(written[4], vec![0x12, 0x4C, 0x18, 0x04, 0x00, 0x10, 0x00, 0x00, 0x8A]);

    assert!(matches!(j0.set_torque(0.1), Err(MisaError::Unsupported(_))));
    assert!(matches!(j0.set_velocity(0.1), Err(MisaError::Unsupported(_))));
    assert!(matches!(j0.mit_control(0.0, 0.0, 1.0, 0.1, 0.0), Err(MisaError::Unsupported(_))));
    assert!(j0.set_run_mode(RunMode::Position).is_ok());
    assert!(j0.set_run_mode(RunMode::Mit).is_err());
    assert!(j0.set_position(0.0, 0.0).is_err());

    let st = j0.read_status().unwrap();
    assert!((st.voltage_v - 12.0).abs() < 1e-6);
    assert!(!st.error.any());
}

#[test]
fn scan_bus_restores_timeout() {
    let bus = Shared::new(Bank::arm(&[0, 2, 5]).into_bus(T));
    let mut j = FashionStarServo::new(bus.clone(), 0);
    let found = j.scan_bus(0..=6, Duration::from_millis(2)).unwrap();
    assert_eq!(found, vec![0, 2, 5]);
    assert_eq!(bus.timeout(), T);
    assert!(j.probe_motor(2, Duration::from_millis(2)).unwrap());
}

#[test]
fn set_zero_sends_origin_point() {
    let bus = Shared::new(FashionStarBus::from_transport(MockTransport::silent(), T));
    let mut j = FashionStarServo::new(bus.clone(), 5);
    j.set_zero().unwrap();
    assert_eq!(
        bus.lock().transport().written()[0],
        vec![0x12, 0x4C, 0x17, 0x02, 0x05, 0x00, 0x7C]
    );
}
