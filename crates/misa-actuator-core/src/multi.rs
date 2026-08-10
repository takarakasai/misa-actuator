//! Several motors on **one** wire.
//!
//! [`Session`](crate::Session) is one motor per worker thread, and each worker
//! opens its own transport. That does not extend by running two of them: a PCAN
//! channel has a single owner, so a second `CAN_Initialize` on the same channel
//! fails with `PCAN_ERROR_NETINUSE`. Sharing the wire has to happen below the
//! drivers, not above them.
//!
//! So the bus is opened once, wrapped in [`SharedCanBus`], and each motor gets
//! its own driver over a clone of that handle. Every driver crate already
//! supports this - `CanBus::new` is generic over any [`misa_can::CanBus`] - and
//! `doc/handover.md` section 3 names one control loop per bus as the intended
//! pattern.
//!
//! # What sharing costs
//!
//! - **A transaction is not atomic.** The vendor bus traits send and receive as
//!   separate calls, so the lock is released between them. Talking to one motor
//!   at a time is what keeps that safe here: nothing else is mid-exchange.
//!   Per-motor threads on one wire would not be.
//! - **The timeout is shared.** It belongs to the transport, so setting it for
//!   one motor sets it for all.
//! - **A mixed-vendor wire has no router.** A frame goes to whichever driver is
//!   reading, and one that does not recognise it discards it, so a reply can be
//!   consumed by the wrong reader and lost. Sequential access keeps that window
//!   small; it does not close it.
//! - **Addressing is only as strong as the protocol makes it.** An earlier
//!   version of this note claimed each driver checks that a reply is addressed
//!   to it, so the worst case was a timeout rather than wrong data. That was
//!   wrong on both families checked against hardware:
//!   - RobStride accepted any status frame of the right kind, ignoring the
//!     sender id in `extra_data`. Fixed 2026-08-03; it now filters on it.
//!   - DAMIAO carries the motor id in **four bits** of the feedback payload
//!     (the high nibble is the status code), so ids agreeing in those bits are
//!     indistinguishable unless each motor has a unique non-zero `MST_ID`.
//!     [`build_multi`] refuses such a pair rather than reporting one motor's
//!     position as another's.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{bail, Context, Result};
use misa_actuator::Actuator;
use serde::{Deserialize, Serialize};

use crate::factory::{
    BusKind, DriverKind, IdentityReport, DEFAULT_SIM_BUS_IDS, MODEL_UNSPECIFIED,
};

/// One [`misa_can::CanBus`] shared by several drivers.
///
/// A local newtype rather than `misa_actuator::Shared`, because implementing
/// `misa_can::CanBus` for a type owned by another crate is not allowed - and
/// every vendor adapter wants something that *is* a `CanBus`.
#[derive(Clone)]
pub struct SharedCanBus(Arc<Mutex<Box<dyn misa_can::CanBus>>>);

impl SharedCanBus {
    pub fn new(bus: Box<dyn misa_can::CanBus>) -> Self {
        Self(Arc::new(Mutex::new(bus)))
    }

    /// Recovers from poisoning: a panic elsewhere does not leave a CAN socket in
    /// a logically invalid state, and refusing to talk to the motors after one
    /// is worse than carrying on. Matches `misa_actuator::Shared`.
    fn lock(&self) -> std::sync::MutexGuard<'_, Box<dyn misa_can::CanBus>> {
        self.0.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Discard whatever is already queued, and say how much there was.
    ///
    /// Called immediately before a transaction, so the next frame read is the
    /// one this transaction asked for. Without it a leftover frame is taken as
    /// the answer: a register reply is caught by its RID, but **a feedback frame
    /// carries no sequence number**, so a duplicate or late one is accepted as
    /// the current position. Duplicated feedback frames were observed on the
    /// bench on 2026-08-03 (`doc/handover.md` section 4), so this is not
    /// hypothetical.
    ///
    /// Non-blocking: the timeout is set to zero, which makes `recv` attempt one
    /// read and return. Bounded, so a busy wire cannot hold the worker here.
    ///
    /// The count is returned rather than logged per frame. A discard nobody can
    /// see is a discard nobody can rule out, and per-frame logging on a busy
    /// wire becomes the noise it was meant to expose.
    pub fn drain_stale(&mut self) -> u64 {
        const MAX_DRAIN: usize = 64;
        let mut bus = self.lock();
        let restore = bus.timeout();
        if bus.set_timeout(Duration::ZERO).is_err() {
            return 0;
        }
        let mut drained = 0;
        for _ in 0..MAX_DRAIN {
            match bus.recv() {
                Ok(_) => drained += 1,
                Err(_) => break,
            }
        }
        let _ = bus.set_timeout(restore);
        drained
    }
}

impl misa_can::CanBus for SharedCanBus {
    fn send(&mut self, frame: &misa_can::Frame) -> misa_can::Result<()> {
        self.lock().send(frame)
    }

    fn recv(&mut self) -> misa_can::Result<misa_can::Frame> {
        self.lock().recv()
    }

    fn set_timeout(&mut self, timeout: Duration) -> misa_can::Result<()> {
        self.lock().set_timeout(timeout)
    }

    fn timeout(&self) -> Duration {
        self.lock().timeout()
    }

    fn supports_fd(&self) -> bool {
        self.lock().supports_fd()
    }

    fn backend_name(&self) -> &'static str {
        self.lock().backend_name()
    }

    fn description(&self) -> String {
        self.lock().description()
    }

    fn rx_overruns(&self) -> u64 {
        self.lock().rx_overruns()
    }

    fn rx_skipped(&self) -> u64 {
        self.lock().rx_skipped()
    }

    fn last_rx_timestamp_us(&self) -> Option<u64> {
        self.lock().last_rx_timestamp_us()
    }
}

/// One motor's share of a multi-motor configuration.
///
/// The driver kind is per motor, not per bus: three of the four families speak
/// CAN and can share a wire, so a RobStride and a DAMIAO on one channel is a
/// real configuration rather than a mistake.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MotorSpec {
    pub driver: DriverKind,
    pub motor_id: u8,
    /// Required for RobStride, where it sets the MIT quantisation range.
    /// Optional elsewhere.
    #[serde(default)]
    pub model: String,
    /// RobStride host id. The driver's own default is used when zero.
    #[serde(default)]
    pub host_id: u8,
    /// Torque constant, for the families that derive torque from current.
    #[serde(default)]
    pub kt: f32,
    /// Gear ratio, where the driver needs telling.
    #[serde(default)]
    pub gear_ratio: f32,
    /// A label for the UI. The motor id is used when empty.
    #[serde(default)]
    pub name: String,
}

impl MotorSpec {
    /// What to call this motor on screen.
    pub fn label(&self) -> String {
        if self.name.trim().is_empty() {
            format!("id {}", self.motor_id)
        } else {
            self.name.clone()
        }
    }
}

/// Everything needed to open one wire with several motors on it.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MultiConfig {
    /// The one interface every motor here is reached through.
    #[serde(default)]
    pub interface: String,
    /// Classic or FD for the **whole channel**. A DAMIAO configured as an FD
    /// node forces FD for everything on the wire; the other families send
    /// classic frames, which an FD-initialised channel carries.
    #[serde(default = "default_bus_kind")]
    pub bus: BusKind,
    #[serde(default = "default_timeout_ms")]
    pub timeout_ms: u64,
    pub motors: Vec<MotorSpec>,
}

fn default_bus_kind() -> BusKind {
    BusKind::Can
}

fn default_timeout_ms() -> u64 {
    100
}

impl MultiConfig {
    pub fn timeout(&self) -> Duration {
        Duration::from_millis(self.timeout_ms)
    }
}

/// A motor built for a multi-motor session, with the spec it came from.
pub struct BuiltMotor {
    pub spec: MotorSpec,
    pub actuator: Box<dyn Actuator + Send>,
    /// What the motor said about itself, where the protocol allows asking.
    pub identity: IdentityReport,
}

// By hand because `dyn Actuator` is not `Debug`, and printing the spec and what
// the motor said about itself is the useful part anyway.
impl std::fmt::Debug for BuiltMotor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BuiltMotor")
            .field("spec", &self.spec)
            .field("identity", &self.identity)
            .finish_non_exhaustive()
    }
}

/// What [`build_multi`] produces: the motors, and the wire they share.
///
/// The bus comes back because the session needs it for
/// [`SharedCanBus::drain_stale`] — the drivers each hold a clone, but none of
/// them owns the wire on everyone's behalf, and draining is a property of the
/// wire rather than of any one motor. `None` when every motor is simulated.
pub struct BuiltBus {
    pub motors: Vec<BuiltMotor>,
    pub bus: Option<SharedCanBus>,
}

// By hand, because a `dyn CanBus` is not `Debug`. The motors and whether a wire
// was opened are the parts worth printing.
impl std::fmt::Debug for BuiltBus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BuiltBus")
            .field("motors", &self.motors)
            .field("bus", &self.bus.as_ref().map(|_| "shared"))
            .finish()
    }
}

/// Open one wire and build every motor on it.
///
/// Simulated motors are built without touching the bus, so a mixed
/// configuration is allowed and a fully simulated one needs no interface at all.
/// That is deliberate: a multi-motor UI has to be developable without four
/// motors on a bench.
pub fn build_multi(cfg: &MultiConfig) -> Result<BuiltBus> {
    if cfg.motors.is_empty() {
        bail!("no motors were listed");
    }

    // A duplicate id on one wire cannot be told apart in the replies, and the
    // symptom - one motor's feedback appearing under another's name - is the
    // kind that gets blamed on the hardware.
    let mut seen: Vec<(DriverKind, u8)> = Vec::with_capacity(cfg.motors.len());
    for m in &cfg.motors {
        let key = (m.driver, m.motor_id);
        if seen.contains(&key) {
            bail!(
                "motor id {} is listed twice for the same driver; replies from the two \
                 would be indistinguishable",
                m.motor_id
            );
        }
        seen.push(key);
        // Before the wire, not after: a mistyped model is a configuration error,
        // and finding it should not require the adapter to be plugged in.
        check_spec(m)?;
    }
    check_damiao_addressing(&cfg.motors)?;

    // Only open the wire if something on it needs one.
    let needs_bus = cfg.motors.iter().any(|m| m.driver != DriverKind::Sim);
    let shared = if needs_bus {
        let opts = match cfg.bus {
            BusKind::Can => misa_can::OpenOptions::classic(),
            BusKind::CanFd => misa_can::OpenOptions::fd(),
        }
        .with_timeout(cfg.timeout());
        let bus = misa_can::open(&cfg.interface, &opts).with_context(|| {
            format!(
                "failed to open {} for {} motors",
                cfg.interface,
                cfg.motors.len()
            )
        })?;
        log::info!(
            "multi: {} shared by {} motors",
            bus.description(),
            cfg.motors.len()
        );
        Some(SharedCanBus::new(bus))
    } else {
        None
    };

    let mut built = Vec::with_capacity(cfg.motors.len());
    for spec in &cfg.motors {
        built.push(build_one(spec, cfg, shared.as_ref())?);
    }
    Ok(BuiltBus {
        motors: built,
        bus: shared,
    })
}

/// One motor found by [`scan_for_motors`].
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ScanHit {
    pub driver: DriverKind,
    pub motor_id: u8,
    /// The model, where the protocol lets us ask. `None` means **unknown**, not
    /// "the default": on RobStride an unrecognised firmware major returns no
    /// answer rather than a guess, and a guess there would pick a MIT range that
    /// differs 21-fold across the family.
    pub model: Option<String>,
    /// What the identification was based on, for the operator to judge.
    pub evidence: Option<String>,
}

/// Which vendors a scan should probe, and how far.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ScanRequest {
    pub interface: String,
    #[serde(default = "default_bus_kind")]
    pub bus: BusKind,
    /// Families to probe. Empty means every CAN family.
    #[serde(default)]
    pub drivers: Vec<DriverKind>,
    #[serde(default = "default_scan_from")]
    pub from: u8,
    #[serde(default = "default_scan_to")]
    pub to: u8,
    /// Per-id timeout. Short, because a scan pays it for every silent id.
    #[serde(default = "default_scan_timeout_ms")]
    pub timeout_ms: u64,
}

fn default_scan_from() -> u8 {
    1
}

fn default_scan_to() -> u8 {
    32
}

fn default_scan_timeout_ms() -> u64 {
    30
}

/// Whether probing this family energises a motor.
///
/// **DAMIAO does.** It has no broadcast ping that every motor answers, so the
/// probe is an enable frame, a listen, and a disable frame. No MIT command is
/// sent, so no torque is commanded, but the motor is briefly energised and may
/// twitch — and `doc/handover.md` section 2 records that `disable` is not a
/// latched safe state on a DM-J4310. The other two families read and nothing
/// more.
///
/// Exposed so a UI can say this **before** the button is pressed rather than
/// after.
pub fn probe_energises_motor(driver: DriverKind) -> bool {
    matches!(driver, DriverKind::Damiao)
}

/// Everything a scan can offer, in probe order: harmless first.
///
/// Order matters on a mixed bus. Doing the probes that touch nothing first means
/// a scan that is interrupted, or that the operator stops after seeing enough,
/// has energised nothing.
///
/// The simulator is first because it opens no bus at all. It is here so the
/// multi-motor UI has a way in without hardware — the alternative was a tab that
/// could only be exercised by energising somebody's motors, which is a poor way
/// to develop one.
pub const SCANNABLE: &[DriverKind] = &[
    DriverKind::Sim,
    DriverKind::Robstride,
    DriverKind::Myactuator,
    DriverKind::Damiao,
];

/// Probe one wire for motors of several families.
///
/// Opens the bus, probes, and closes it again — so it must not run while a
/// [`crate::multi_session::MultiSession`] holds the same channel. A PCAN channel
/// has one owner and the second open fails with `PCAN_ERROR_NETINUSE`, which is
/// the same reason two sessions cannot share a wire.
///
/// The vendor is always reported. The model is reported only when the protocol
/// answers, because the alternative is presenting a guess as a reading.
pub fn scan_for_motors(req: &ScanRequest) -> Result<Vec<ScanHit>> {
    if req.to < req.from {
        bail!("scan range is inverted: {}..{}", req.from, req.to);
    }
    let drivers: Vec<DriverKind> = if req.drivers.is_empty() {
        SCANNABLE.to_vec()
    } else {
        // Keep the caller's selection but impose the safe order.
        SCANNABLE
            .iter()
            .copied()
            .filter(|d| req.drivers.contains(d))
            .collect()
    };
    if drivers.is_empty() {
        bail!("no scannable families were selected (LK Motor is RS485, not CAN)");
    }

    let mut hits = Vec::new();

    // The simulator answers without a wire, so it is handled before anything is
    // opened — and a simulator-only scan opens nothing at all.
    if drivers.contains(&DriverKind::Sim) {
        for &id in DEFAULT_SIM_BUS_IDS.iter().filter(|&&id| (req.from..=req.to).contains(&id)) {
            hits.push(ScanHit {
                driver: DriverKind::Sim,
                motor_id: id,
                model: Some("ideal".into()),
                evidence: Some("simulated; no bus was opened".into()),
            });
        }
    }
    let can: Vec<DriverKind> = drivers
        .into_iter()
        .filter(|&d| d != DriverKind::Sim)
        .collect();
    if can.is_empty() {
        return Ok(hits);
    }

    let opts = match req.bus {
        BusKind::Can => misa_can::OpenOptions::classic(),
        BusKind::CanFd => misa_can::OpenOptions::fd(),
    }
    .with_timeout(Duration::from_millis(req.timeout_ms));
    let bus = misa_can::open(&req.interface, &opts)
        .with_context(|| format!("failed to open {} to scan", req.interface))?;
    let shared = SharedCanBus::new(bus);
    let timeout = Duration::from_millis(req.timeout_ms);
    let range = req.from..=req.to;

    for driver in can {
        match driver {
            DriverKind::Robstride => {
                let mut adapter = robstride_driver::CanBus::new(shared.clone());
                let found = robstride_driver::scan_bus_on(
                    &mut adapter,
                    robstride_driver::DEFAULT_HOST_ID,
                    range.clone(),
                    timeout,
                    None,
                )
                .unwrap_or_default();
                for r in found {
                    hits.push(robstride_hit(&shared, r.motor_id, timeout));
                }
            }
            DriverKind::Myactuator => {
                let mut adapter = myactuator_driver::CanBus::new(shared.clone());
                let found = myactuator_driver::scan_bus_on(
                    &mut adapter,
                    range.clone(),
                    timeout,
                    None,
                )
                .unwrap_or_default();
                for id in found {
                    hits.push(ScanHit {
                        driver: DriverKind::Myactuator,
                        motor_id: id,
                        model: None,
                        evidence: Some("answered a Status1 read".into()),
                    });
                }
            }
            DriverKind::Damiao => {
                let mut adapter = match req.bus {
                    BusKind::Can => damiao_driver::CanBus::new(shared.clone()),
                    BusKind::CanFd => damiao_driver::CanBus::new_fd(shared.clone()),
                };
                let found =
                    damiao_driver::scan_bus_on(&mut adapter, range.clone(), timeout, None)
                        .unwrap_or_default();
                for id in found {
                    hits.push(damiao_hit(&shared, req.bus, id, timeout));
                }
            }
            // Not reachable: filtered by SCANNABLE above.
            DriverKind::Sim | DriverKind::Lkmotor => {}
        }
    }
    Ok(hits)
}

/// Identify a RobStride that answered, from its firmware version.
///
/// The motor has to be constructed with *some* model before it can be asked
/// anything, and the model is what we are trying to find out. RS-04 is used as a
/// placeholder and **only `read_version` is called on it** — that read carries no
/// scaled quantity, so the placeholder cannot corrupt the answer. Anything that
/// scales torque or position must wait until the real model is known.
fn robstride_hit(shared: &SharedCanBus, motor_id: u8, timeout: Duration) -> ScanHit {
    let mut motor = robstride_driver::Motor::with_bus(
        robstride_driver::CanBus::new(shared.clone()),
        motor_id,
        robstride_driver::MotorModel::Rs04,
    );
    let _ = motor.set_timeout(timeout);

    match motor.read_version() {
        Ok(v) => {
            let named = robstride_driver::MotorModel::from_firmware_version(v.version);
            match named {
                Some((model, line)) => ScanHit {
                    driver: DriverKind::Robstride,
                    motor_id,
                    model: Some(line.catalogue_name(model).to_string()),
                    evidence: Some(format!("firmware {v}")),
                },
                // Deliberately no fallback. An unrecognised major means the
                // version-to-model table has moved on, and naming a model here
                // would be a guess about the MIT range.
                None => ScanHit {
                    driver: DriverKind::Robstride,
                    motor_id,
                    model: None,
                    evidence: Some(format!(
                        "firmware {v}, which no known model claims - pick the model by hand"
                    )),
                },
            }
        }
        Err(e) => ScanHit {
            driver: DriverKind::Robstride,
            motor_id,
            model: None,
            evidence: Some(format!("answered a ping, but the version read failed: {e}")),
        },
    }
}

/// Identify a DAMIAO from its reduction ratio, which names the model exactly.
fn damiao_hit(
    shared: &SharedCanBus,
    bus: BusKind,
    motor_id: u8,
    timeout: Duration,
) -> ScanHit {
    let adapter = match bus {
        BusKind::Can => damiao_driver::CanBus::new(shared.clone()),
        BusKind::CanFd => damiao_driver::CanBus::new_fd(shared.clone()),
    };
    // A placeholder model again, and again only a register read is issued: the
    // gear ratio is not a scaled quantity.
    let mut motor = damiao_driver::DamiaoMotor::with_bus(
        adapter,
        motor_id,
        damiao_driver::MotorModel::Dm4310,
    );
    let _ = motor.set_timeout(timeout);

    // Two independent channels, like RobStride: the reduction ratio names the
    // model, and `sw_ver` names the series *and the hardware generation* — which
    // the ratio cannot, because a 4310 reads GR 10 whether it is V2, V3 or 48 V.
    let by_ratio = motor.identify_model().ok();
    let firmware = motor
        .read_register(damiao_driver::protocol::Rid::SW_VER)
        .ok()
        .map(|r| r.as_i32() as u32)
        .and_then(damiao_driver::MotorModel::from_firmware_version);

    let model = by_ratio
        .map(|m| m.name().to_string())
        // The firmware code is a real answer even where the ratio read failed,
        // and for a series this crate has no variant for it is the *only*
        // answer.
        .or_else(|| firmware.and_then(|f| f.model).map(|m| m.name().to_string()))
        .or_else(|| firmware.map(|f| format!("{} ({})", f.series, f.hardware.name())));

    let evidence = match (by_ratio, firmware) {
        (Some(r), Some(f)) => {
            let agree = f.model.is_none_or(|m| m == r);
            if agree {
                Some(format!(
                    "reduction ratio {}:1 and firmware {}{:02} ({} {}) agree",
                    r.gear_ratio(),
                    f.code,
                    f.build,
                    f.series,
                    f.hardware.name()
                ))
            } else {
                // Say it rather than pick one. Two channels disagreeing means
                // one of the tables is stale, and neither answer is safe to
                // build a MIT range on.
                Some(format!(
                    "DISAGREEMENT: reduction ratio says {}, firmware {}{:02} says {} {}. \
                     Trust neither until this is resolved.",
                    r.name(),
                    f.code,
                    f.build,
                    f.series,
                    f.hardware.name()
                ))
            }
        }
        (Some(r), None) => Some(format!("reduction ratio {}:1 (RID 20)", r.gear_ratio())),
        (None, Some(f)) => Some(format!(
            "firmware {}{:02} ({} {}); the gear-ratio read did not answer",
            f.code,
            f.build,
            f.series,
            f.hardware.name()
        )),
        (None, None) => Some("answered a probe, but neither identification read came back".into()),
    };

    ScanHit {
        driver: DriverKind::Damiao,
        motor_id,
        model,
        evidence,
    }
}

/// Refuse DAMIAO pairs whose feedback cannot be attributed.
///
/// The id in a DAMIAO feedback frame is four bits wide, so two motors agreeing
/// in those bits are separable only by their Master IDs — and the default Master
/// ID of `0` means "accept any responder", which switches that check off. Such a
/// pair does not fail loudly: each driver reports the other's position as its
/// own. Refusing beats measuring the wrong shaft.
fn check_damiao_addressing(motors: &[MotorSpec]) -> Result<()> {
    let damiao: Vec<&MotorSpec> = motors
        .iter()
        .filter(|m| m.driver == DriverKind::Damiao)
        .collect();

    for (i, a) in damiao.iter().enumerate() {
        for b in &damiao[i + 1..] {
            // `master_id` is not in `MotorSpec` yet, so every DAMIAO here uses
            // the driver default of 0. Passing it explicitly keeps this honest
            // about what is being compared.
            if !damiao_driver::DamiaoMotor::<damiao_driver::AnyCanBus>::feedback_is_distinguishable(
                (a.motor_id, 0),
                (b.motor_id, 0),
            ) {
                bail!(
                    "DAMIAO motors {} and {} cannot be told apart on one bus: the feedback \
                     frame carries only the low four bits of the id ({:#X} for both), and \
                     neither has a unique Master ID. Each driver would report the other's \
                     position as its own. Give them ids that differ in the low nibble, or \
                     assign a unique MST_ID to each (damiao-cli reg-write 7 <id> --save, \
                     then power-cycle).",
                    a.motor_id,
                    b.motor_id,
                    a.motor_id & 0x0F
                );
            }
        }
    }
    Ok(())
}

/// Everything about one spec that can be judged without a bus.
///
/// Kept separate so [`build_multi`] can reject a bad configuration before
/// opening anything. The model checks are the ones that matter: RobStride has no
/// safe default, and a typo elsewhere should not read as "the adapter is
/// missing".
fn check_spec(spec: &MotorSpec) -> Result<()> {
    match spec.driver {
        DriverKind::Robstride => {
            // No fallback, for the reason `build_actuator_checked` gives: the
            // MIT scales span 21-fold across the family, so a guess is how you
            // send twenty times the torque you meant to.
            if spec.model.trim().is_empty() {
                bail!(
                    "motor id {} is a RobStride and needs a model: it sets the MIT \
                     quantisation range, which differs 21-fold across the family \
                     (RS-05 is +-5.5 N-m, RS-04 +-120)",
                    spec.motor_id
                );
            }
            robstride_model(spec).map(|_| ())
        }
        DriverKind::Damiao => damiao_model(spec).map(|_| ()),
        DriverKind::Sim => sim_preset(spec).map(|_| ()),
        DriverKind::Myactuator => Ok(()),
        DriverKind::Lkmotor => bail!(
            "motor id {} is an LK Motor, which speaks RS485 rather than CAN and cannot \
             share this bus. Open it as its own single-motor session.",
            spec.motor_id
        ),
    }
}

fn robstride_model(spec: &MotorSpec) -> Result<robstride_driver::MotorModel> {
    robstride_driver::MotorModel::from_name(&spec.model).with_context(|| {
        format!(
            "unknown RobStride model {:?} for motor {}",
            spec.model, spec.motor_id
        )
    })
}

/// An untouched shared default means "the family default", as it does in
/// `build_actuator_checked`; a genuinely wrong override still fails.
fn damiao_model(spec: &MotorSpec) -> Result<damiao_driver::MotorModel> {
    match damiao_driver::MotorModel::from_name(&spec.model) {
        Some(m) => Ok(m),
        None if spec.model.trim().is_empty() || spec.model == MODEL_UNSPECIFIED => {
            Ok(damiao_driver::MotorModel::Dm4310)
        }
        None => bail!(
            "unknown DAMIAO model {:?} for motor {}",
            spec.model,
            spec.motor_id
        ),
    }
}

fn sim_preset(spec: &MotorSpec) -> Result<misa_actuator_sim::SimConfig> {
    match misa_actuator_sim::SimConfig::from_name(&spec.model) {
        Some(p) => Ok(p),
        None if spec.model.trim().is_empty() || spec.model == MODEL_UNSPECIFIED => {
            Ok(misa_actuator_sim::SimConfig::ideal())
        }
        None => bail!(
            "unknown sim preset {:?} for motor {} (known: {})",
            spec.model,
            spec.motor_id,
            misa_actuator_sim::SimConfig::PRESETS.join(", ")
        ),
    }
}

fn build_one(
    spec: &MotorSpec,
    cfg: &MultiConfig,
    shared: Option<&SharedCanBus>,
) -> Result<BuiltMotor> {
    let bus = || -> Result<SharedCanBus> {
        shared
            .cloned()
            .ok_or_else(|| anyhow::anyhow!("internal error: no bus was opened"))
    };

    let (actuator, identity): (Box<dyn Actuator + Send>, _) = match spec.driver {
        DriverKind::Sim => {
            let sim =
                misa_actuator_sim::SimActuator::new(sim_preset(spec)?).with_motor_id(spec.motor_id);
            (Box::new(sim), IdentityReport::default())
        }
        DriverKind::Robstride => {
            let model = robstride_model(spec)?;
            let host_id = if spec.host_id == 0 {
                robstride_driver::DEFAULT_HOST_ID
            } else {
                spec.host_id
            };
            let mut m = robstride_driver::Motor::with_bus_and_host(
                robstride_driver::CanBus::new(bus()?),
                spec.motor_id,
                host_id,
                model,
            );
            m.set_timeout(cfg.timeout())
                .context("failed to set the bus timeout")?;
            let report = crate::factory::robstride_identity(&mut m, model, &spec.model);
            (Box::new(m), report)
        }
        DriverKind::Damiao => {
            let model = damiao_model(spec)?;
            // Frame the sends to match the channel. An FD-initialised channel
            // still carries classic frames, but a DAMIAO configured as an FD
            // node will not answer them.
            let adapter = match cfg.bus {
                BusKind::Can => damiao_driver::CanBus::new(bus()?),
                BusKind::CanFd => damiao_driver::CanBus::new_fd(bus()?),
            };
            let mut m = damiao_driver::DamiaoMotor::with_bus(adapter, spec.motor_id, model);
            m.set_timeout(cfg.timeout())
                .context("failed to set the bus timeout")?;
            let report = crate::factory::damiao_identity(&mut m, model);
            (Box::new(m), report)
        }
        DriverKind::Myactuator => {
            // Zero means current units: the wire carries amps, and no Kt is
            // invented to dress them up as torque.
            let mya_cfg = if spec.kt > 0.0 {
                myactuator_driver::MotorConfig::new(spec.kt)
            } else {
                myactuator_driver::MotorConfig::current_units()
            };
            let m = myactuator_driver::MyActuatorMotor::with_bus(
                myactuator_driver::CanBus::new(bus()?),
                spec.motor_id,
                mya_cfg,
            )
            .with_context(|| format!("failed to bind MyActuator motor {}", spec.motor_id))?;
            (Box::new(m), IdentityReport::default())
        }
        DriverKind::Lkmotor => unreachable!("rejected in build_multi"),
    };

    Ok(BuiltMotor {
        spec: spec.clone(),
        actuator,
        identity,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sim(id: u8) -> MotorSpec {
        MotorSpec {
            driver: DriverKind::Sim,
            motor_id: id,
            model: "ideal".into(),
            host_id: 0,
            kt: 0.0,
            gear_ratio: 0.0,
            name: String::new(),
        }
    }

    fn cfg(motors: Vec<MotorSpec>) -> MultiConfig {
        MultiConfig {
            interface: String::new(),
            bus: BusKind::Can,
            timeout_ms: 100,
            motors,
        }
    }

    /// A fully simulated set opens no interface at all, so the multi-motor UI
    /// can be built and exercised without hardware.
    #[test]
    fn simulated_motors_need_no_bus() {
        let built = build_multi(&cfg(vec![sim(1), sim(2), sim(3)])).unwrap();
        assert_eq!(built.motors.len(), 3);
        assert_eq!(built.motors[2].spec.motor_id, 3);
        // Each motor answers as itself, not as the preset's own id.
        assert_eq!(built.motors[0].actuator.motor_id(), 1);
        assert_eq!(built.motors[1].actuator.motor_id(), 2);
        assert!(
            built.bus.is_none(),
            "a simulated set must not have opened a wire"
        );
    }

    /// Two motors answering to one id produce feedback that cannot be
    /// attributed, which is worse than refusing to start.
    #[test]
    fn a_duplicate_id_is_refused() {
        let err = build_multi(&cfg(vec![sim(1), sim(1)]))
            .unwrap_err()
            .to_string();
        assert!(err.contains("twice"), "{err}");
    }

    /// RS485 cannot share a CAN wire. Saying so beats a confusing open failure
    /// against an interface name that is really a COM port.
    #[test]
    fn lkmotor_is_refused_with_a_reason() {
        let mut lk = sim(1);
        lk.driver = DriverKind::Lkmotor;
        let err = build_multi(&cfg(vec![lk])).unwrap_err().to_string();
        assert!(err.contains("RS485"), "{err}");
    }

    /// RobStride without a model is refused rather than defaulted, because the
    /// default would silently pick a quantisation range.
    #[test]
    fn robstride_still_refuses_to_guess_a_model() {
        let mut rs = sim(1);
        rs.driver = DriverKind::Robstride;
        rs.model = String::new();
        let err = build_multi(&cfg(vec![rs])).unwrap_err().to_string();
        assert!(err.contains("needs a model"), "{err}");
        // Refused before the bus is touched: no interface was configured.
        assert!(!err.contains("failed to open"), "{err}");
    }

    fn damiao(id: u8) -> MotorSpec {
        MotorSpec {
            driver: DriverKind::Damiao,
            motor_id: id,
            model: "DM4310".into(),
            host_id: 0,
            kt: 0.0,
            gear_ratio: 0.0,
            name: String::new(),
        }
    }

    /// The id in a DAMIAO feedback frame is four bits wide, so 1 and 17 look
    /// identical on the wire. Each driver would report the other's position as
    /// its own, so this has to be refused rather than measured.
    #[test]
    fn damiao_ids_that_alias_in_four_bits_are_refused() {
        let err = build_multi(&cfg(vec![damiao(1), damiao(17)]))
            .unwrap_err()
            .to_string();
        assert!(err.contains("cannot be told apart"), "{err}");
        // Refused before the bus is opened, so no adapter is needed to find it.
        assert!(!err.contains("failed to open"), "{err}");
    }

    /// 16 is the id on the bench unit and its low nibble is zero, which collides
    /// with 32 -- worth pinning, because it is the pair most likely to be hit.
    #[test]
    fn sixteen_and_thirty_two_are_refused() {
        let err = build_multi(&cfg(vec![damiao(16), damiao(32)]))
            .unwrap_err()
            .to_string();
        assert!(err.contains("cannot be told apart"), "{err}");
    }

    /// Ids differing in the low nibble are fine, and must still reach the bus
    /// rather than being rejected by the addressing check.
    #[test]
    fn damiao_ids_differing_in_the_low_nibble_are_allowed() {
        let err = build_multi(&cfg(vec![damiao(1), damiao(2)]))
            .unwrap_err()
            .to_string();
        assert!(
            !err.contains("cannot be told apart"),
            "addressing is fine here: {err}"
        );
        // It fails for the only remaining reason: there is no interface.
        assert!(err.contains("failed to open"), "{err}");
    }

    /// A single DAMIAO has nothing to be confused with.
    #[test]
    fn one_damiao_is_never_ambiguous() {
        let err = build_multi(&cfg(vec![damiao(16)])).unwrap_err().to_string();
        assert!(!err.contains("cannot be told apart"), "{err}");
    }

    #[test]
    fn an_empty_list_is_refused() {
        assert!(build_multi(&cfg(Vec::new())).is_err());
    }

    /// The order is a safety property, not a preference: the read-only probes
    /// run before the one that energises a motor, so a scan stopped early has
    /// energised nothing.
    #[test]
    fn the_energising_probe_runs_last() {
        let damiao_at = SCANNABLE.iter().position(|&d| d == DriverKind::Damiao);
        assert_eq!(damiao_at, Some(SCANNABLE.len() - 1));
        for &d in &SCANNABLE[..SCANNABLE.len() - 1] {
            assert!(!probe_energises_motor(d), "{d:?} was expected to be read-only");
        }
        assert!(probe_energises_motor(DriverKind::Damiao));
    }

    /// LK Motor speaks RS485 and cannot be found on a CAN wire, so offering it
    /// would produce a scan that cannot work.
    #[test]
    fn lkmotor_is_not_scannable() {
        assert!(!SCANNABLE.contains(&DriverKind::Lkmotor));
    }

    /// A simulator-only scan opens nothing, which is what makes the multi-motor
    /// UI developable without hardware — and without energising anybody's motor
    /// to see a table.
    #[test]
    fn a_simulated_scan_needs_no_interface() {
        let req = ScanRequest {
            interface: String::new(),
            bus: BusKind::Can,
            drivers: vec![DriverKind::Sim],
            from: 1,
            to: 32,
            timeout_ms: 10,
        };
        let hits = scan_for_motors(&req).expect("a simulated scan cannot fail on a missing bus");
        assert!(!hits.is_empty());
        assert!(hits.iter().all(|h| h.driver == DriverKind::Sim));
        // Every simulated hit is usable as-is: a preset is named, so the model
        // box is not left for the operator to guess at.
        assert!(hits.iter().all(|h| h.model.is_some()));
    }

    /// The range still applies to simulated ids, or the from/to boxes would
    /// silently mean nothing for the one driver you can try without hardware.
    #[test]
    fn a_simulated_scan_respects_the_range() {
        let req = ScanRequest {
            interface: String::new(),
            bus: BusKind::Can,
            drivers: vec![DriverKind::Sim],
            from: 1,
            to: 2,
            timeout_ms: 10,
        };
        let hits = scan_for_motors(&req).unwrap();
        assert!(hits.iter().all(|h| h.motor_id <= 2), "{hits:?}");
    }

    /// The simulator must not be listed as energising: it has no motor to
    /// energise, and a spurious warning trains people to ignore the real one.
    #[test]
    fn the_simulator_does_not_warn_about_energising() {
        assert!(!probe_energises_motor(DriverKind::Sim));
    }

    /// Selecting nothing scannable is refused rather than returning an empty
    /// list, which would read as "no motors on the bus".
    #[test]
    fn selecting_no_scannable_family_is_an_error() {
        let req = ScanRequest {
            interface: "pcan:usb1".into(),
            bus: BusKind::Can,
            drivers: vec![DriverKind::Lkmotor],
            from: 1,
            to: 8,
            timeout_ms: 10,
        };
        let err = scan_for_motors(&req).unwrap_err().to_string();
        assert!(err.contains("no scannable families"), "{err}");
        // Refused before the bus is touched.
        assert!(!err.contains("failed to open"), "{err}");
    }

    #[test]
    fn an_inverted_range_is_refused() {
        let req = ScanRequest {
            interface: "pcan:usb1".into(),
            bus: BusKind::Can,
            drivers: Vec::new(),
            from: 20,
            to: 3,
            timeout_ms: 10,
        };
        let err = scan_for_motors(&req).unwrap_err().to_string();
        assert!(err.contains("inverted"), "{err}");
    }

    #[test]
    fn a_label_falls_back_to_the_id() {
        assert_eq!(sim(7).label(), "id 7");
        let mut named = sim(7);
        named.name = "shoulder".into();
        assert_eq!(named.label(), "shoulder");
    }
}
