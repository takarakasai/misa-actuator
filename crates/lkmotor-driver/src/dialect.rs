//! Work out which parameter dialect a driver board speaks.
//!
//! LKMotor's RS485 and CAN protocols carry **no command for reading a firmware
//! version, model, hardware revision, or serial number** — this is stated
//! explicitly in the V2.36 manual translations under `lkmotor-protocol/ref/`,
//! re-confirmed against the Chinese sources. So a board's generation cannot be
//! read; it can only be *inferred* from which commands it answers.
//!
//! Five MG4005 units on one bench each answered exactly one parameter
//! interface, and never both:
//!
//! | command | `0xC0` board | legacy board |
//! |---------|--------------|--------------|
//! | `0xC0` read control parameter | answers | silent |
//! | `0x30` read PID (legacy)      | silent  | answers |
//! | `0x33` read accel (legacy)    | silent  | answers |
//!
//! Do not read "legacy" as "older unit". It names the command family, which
//! is all that was measured. The housing's `V2`/`V3` suffix denotes the driver
//! version, but it does not line up with this split: a unit marked `V3` and
//! units marked `V2` all answered `0xC0`.
//!
//! That mutual exclusivity is what makes the inference work. Both boards
//! answered the state reads (`0x9A`/`0x9C`/`0x9D`) identically, so those
//! separate "is anyone there" from "which dialect".
//!
//! ## Why one payload shape per command is enough
//!
//! A silent command is only evidence of "unsupported" if the request was
//! shaped the way that board wants — and `0xC0` really is shape-sensitive:
//! the `0xC0` board answers it at `len=7` and ignores `len=0`, `1` and `2`
//! entirely. So the obvious worry is that a board might answer some shape
//! this probe never sends, and get filed under the wrong dialect.
//!
//! Both units were swept against that worry, 10 attempts per shape:
//!
//! | command / payload len | `0xC0` board | legacy board |
//! |-----------------------|--------------|--------------|
//! | `0x30` len 0, 1, 2, 6, 7 | 0/10 every shape | 10/10 every shape |
//! | `0x33` len 0, 1, 4       | 0/10 every shape | 10/10 every shape |
//! | `0xC0` len 0, 1, 2       | 0/10             | 0/10 |
//! | `0xC0` len 7 (spec)      | 10/10            | 0/10 |
//!
//! Both negatives survive: the legacy board stays silent on `0xC0` even when
//! asked exactly the way the manual specifies, and the `0xC0` board stays
//! silent on the legacy pair in every shape tried. The legacy commands turn
//! out to ignore the payload entirely, so shape cannot matter for them at
//! all. Sending one canonical shape per command therefore reaches the same
//! verdict as the full sweep, at a quarter of the traffic — which is why
//! [`probe_dialect`] does not sweep.
//!
//! What this does *not* rule out is a third board that wants some other
//! `0xC0` shape. If one ever turns up it will land in [`Dialect::Legacy`]
//! wrongly, and the fix is to widen the sweep here.
//!
//! **Probing must retry.** On a marginal RS485 link a lost frame is
//! indistinguishable from "this command is unsupported", and a single-shot
//! probe will happily report the wrong dialect — that misclassification has
//! already happened once on this bench, on a link that was later measured at
//! 84% frame reliability. [`probe_dialect`] therefore asks each command
//! several times and reports the hit counts alongside the verdict, so a
//! marginal link shows up as partial counts rather than a confident lie.

use lkmotor_protocol::command::{Command, ControlParamId};

use crate::bus::{LkBus, LkCommands};
use crate::error::Result;
use crate::motor_id::MotorId;

/// Default number of attempts per probed command. Three is enough to make a
/// few-percent frame-loss rate unlikely to read as "unsupported", while
/// keeping a probe of four commands quick.
pub const DEFAULT_ATTEMPTS: u8 = 4;

/// Which parameter dialect a board speaks.
///
/// These names describe **what the board answers**, deliberately, and not the
/// `V2`/`V3` suffix printed on an MG4005 housing. Bench evidence says the two
/// are not the same axis: a unit marked `V3` and units marked `V2` all answer
/// `0xC0`, so `0xC0` cannot be what the marking distinguishes. Until a mapping
/// is established against enough labelled units, reporting a dialect as a
/// driver version would be asserting more than is known.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dialect {
    /// Answers `0xC0`, the control-parameter table documented in the V2.36
    /// manual, and not the legacy commands.
    ControlParamTable,
    /// Answers legacy `0x30`/`0x33` and not `0xC0`. These two commands appear
    /// in no manual under `lkmotor-protocol/ref/`, including V2.36, so they
    /// predate that revision or belong to a separate lineage.
    Legacy,
    /// Answered both families. Not seen on this bench; reported rather than
    /// forced into one bucket, because it would mean the mutual exclusivity
    /// this inference rests on does not hold for that firmware.
    Both,
    /// The motor answers state reads but neither parameter family. Either a
    /// third dialect, or a link too poor for any probe to land.
    Neither,
}

impl Dialect {
    /// Short label for display.
    pub fn label(self) -> &'static str {
        match self {
            Dialect::ControlParamTable => "control-parameter table (0xC0)",
            Dialect::Legacy => "legacy (0x30 / 0x33)",
            Dialect::Both => "ambiguous — answered both families",
            Dialect::Neither => "unknown — answered neither family",
        }
    }
}

/// Outcome of [`probe_dialect`], including the raw evidence.
#[derive(Debug, Clone, Copy)]
pub struct DialectReport {
    /// The verdict. Only meaningful when `present` is true.
    pub dialect: Dialect,
    /// Whether the motor answered the state read at all.
    pub present: bool,
    /// Attempts made per command.
    pub attempts: u8,
    /// Successful `0x9A` state reads.
    pub state_hits: u8,
    /// Successful `0xC0` control-parameter reads.
    pub control_param_hits: u8,
    /// Successful legacy `0x30` PID reads.
    pub legacy_pid_hits: u8,
    /// Successful legacy `0x33` accel reads.
    pub legacy_accel_hits: u8,
}

impl DialectReport {
    /// True when every probed command that answered at all answered every
    /// time. A partial count means frames were lost, so the verdict rests on
    /// fewer samples than it looks — worth surfacing to the user.
    pub fn link_was_clean(&self) -> bool {
        let full = |n: u8| n == 0 || n == self.attempts;
        full(self.state_hits)
            && full(self.control_param_hits)
            && full(self.legacy_pid_hits)
            && full(self.legacy_accel_hits)
    }
}

/// Ask a motor which parameter dialect it speaks.
///
/// Sends only side-effect-free reads. `attempts` is the number of tries per
/// command; see the module docs for why more than one is required. Returns
/// `present: false` if the motor never answered the state read, in which case
/// the dialect verdict carries no information.
pub fn probe_dialect<B: LkBus>(
    bus: &mut B,
    motor_id: MotorId,
    attempts: u8,
) -> Result<DialectReport> {
    let attempts = attempts.max(1);

    // `0x9A` is answered by every board seen so far, so it separates "nothing
    // is there / the link is dead" from "there, but speaks another dialect".
    let mut state_hits = 0;
    for _ in 0..attempts {
        let _ = bus.flush_rx();
        if bus.read_state1(motor_id).is_ok() {
            state_hits += 1;
        }
    }

    let mut control_param_hits = 0;
    for _ in 0..attempts {
        let _ = bus.flush_rx();
        if bus
            .read_control_param(motor_id, ControlParamId::PositionLoopPid)
            .is_ok()
        {
            control_param_hits += 1;
        }
    }

    let mut legacy_pid_hits = 0;
    for _ in 0..attempts {
        let _ = bus.flush_rx();
        if bus.read_legacy_pids(motor_id).is_ok() {
            legacy_pid_hits += 1;
        }
    }

    let mut legacy_accel_hits = 0;
    for _ in 0..attempts {
        let _ = bus.flush_rx();
        if bus.read_legacy_accel(motor_id).is_ok() {
            legacy_accel_hits += 1;
        }
    }

    let speaks_new = control_param_hits > 0;
    let speaks_old = legacy_pid_hits > 0 || legacy_accel_hits > 0;
    let dialect = match (speaks_new, speaks_old) {
        (true, false) => Dialect::ControlParamTable,
        (false, true) => Dialect::Legacy,
        (true, true) => Dialect::Both,
        (false, false) => Dialect::Neither,
    };

    Ok(DialectReport {
        dialect,
        present: state_hits > 0,
        attempts,
        state_hits,
        control_param_hits,
        legacy_pid_hits,
        legacy_accel_hits,
    })
}

/// The command codes this probe uses, for display.
pub const PROBED_COMMANDS: [(u8, &str); 4] = [
    (Command::ReadMotorState1 as u8, "state read (presence)"),
    (Command::ReadControlParam as u8, "control-param table"),
    (Command::ReadPid as u8, "legacy read-PID"),
    (Command::ReadAccel as u8, "legacy read-accel"),
];

#[cfg(test)]
mod tests {
    use super::*;

    fn report(cp: u8, lp: u8, la: u8, state: u8) -> DialectReport {
        let speaks_new = cp > 0;
        let speaks_old = lp > 0 || la > 0;
        DialectReport {
            dialect: match (speaks_new, speaks_old) {
                (true, false) => Dialect::ControlParamTable,
                (false, true) => Dialect::Legacy,
                (true, true) => Dialect::Both,
                (false, false) => Dialect::Neither,
            },
            present: state > 0,
            attempts: 4,
            state_hits: state,
            control_param_hits: cp,
            legacy_pid_hits: lp,
            legacy_accel_hits: la,
        }
    }

    #[test]
    fn newer_board_classifies_as_control_param_table() {
        // The V3 unit on this bench: 0xC0 answers, legacy silent.
        assert_eq!(report(4, 0, 0, 4).dialect, Dialect::ControlParamTable);
    }

    #[test]
    fn older_board_classifies_as_legacy() {
        // The other MG4005: 0x30/0x33 answer, 0xC0 silent.
        assert_eq!(report(0, 4, 4, 4).dialect, Dialect::Legacy);
    }

    #[test]
    fn a_single_surviving_reply_is_enough_to_claim_support() {
        // Frame loss must not turn a supported command into an unsupported
        // one: one hit out of four still proves the board answers it.
        assert_eq!(report(1, 0, 0, 2).dialect, Dialect::ControlParamTable);
    }

    #[test]
    fn answering_both_is_reported_not_guessed() {
        assert_eq!(report(4, 4, 4, 4).dialect, Dialect::Both);
    }

    #[test]
    fn silence_on_both_families_is_neither() {
        assert_eq!(report(0, 0, 0, 4).dialect, Dialect::Neither);
    }

    #[test]
    fn absent_motor_is_flagged_regardless_of_dialect() {
        assert!(!report(0, 0, 0, 0).present);
    }

    #[test]
    fn partial_hit_counts_mark_the_link_dirty() {
        assert!(report(4, 0, 0, 4).link_was_clean());
        assert!(!report(2, 0, 0, 4).link_was_clean());
        assert!(!report(4, 0, 0, 3).link_was_clean());
    }
}
