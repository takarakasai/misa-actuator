//! Bulk parameter-table read (comm type `19`).
//!
//! # ⚠ Reverse-engineered, not from the manual or the community reference
//!
//! This mechanism is what RobStride's official `motorstudio` tool uses to
//! populate its "Parameter Setting" table (`Name`, `BootCodeVersion`,
//! `AppCodeVersion`, ...) — it is **not** documented in the community
//! reference implementation this crate is otherwise built from
//! (`robstride_sandbox`), which only covers comm types `0`–`25`. Everything
//! here was reverse-engineered from a single live CAN-FD capture of
//! `motorstudio` talking to a real RS04 (2026-07-26); treat it as best-effort
//! until corroborated by more captures or official documentation.
//!
//! It is **read-only** (queries the table; never writes), so a wrong guess
//! in [`build_read_param_table_frame`]'s exact trigger payload just times
//! out rather than risking device state — unlike a write/config command,
//! there is no plausible failure mode here worse than "no reply".
//!
//! ## Wire shape, once triggered
//!
//! An earlier revision of this module guessed the trigger was a bare
//! `GetDeviceId`-shaped frame with an all-zero payload; that was tried live
//! against a real RS04 and got no response. The frame captured on the wire
//! (comm type `19`, `extra_data = host_id`, `device_id = target`, payload =
//! the 8-byte MCU UID) is now understood to be the **request itself**, not
//! a reply: a bare CAN id can only address 255 devices, so this protocol
//! disambiguates by UID instead, meaning the host must already know the
//! target's UID (e.g. from a prior classic `GetDeviceId` query) before it
//! can trigger a table read. [`build_read_param_table_frame`] takes that UID
//! accordingly. This reading fits the evidence (identical bytes on every
//! trigger — the UID never changes — and no separate request frame is ever
//! observed) better than the original guess, but is still unconfirmed by
//! anything beyond one live capture; validate against your own hardware.
//!
//! 1. Host sends comm type `19`, `extra_data = host_id`, `device_id =
//!    target`, payload = the target's 8-byte MCU UID.
//! 2. The motor then streams every row of its parameter table: comm type
//!    `19`, `extra_data = (slot << 8) | motor_id`, `device_id = host_id`
//!    (**roles swapped** from the request: the reply's addressee — the
//!    host — now sits in the `device_id` slot). Payload is
//!    `[row, page, ...6 data bytes]`, where `(page << 8) | row` is the
//!    `FunctionCode` (e.g. `0x1003` = `AppCodeVersion`, matching
//!    `motorstudio`'s own column). `slot` `0..=2` carry three consecutive
//!    6-byte fragments of the parameter's **name** (ASCII, NUL-padded);
//!    slot `6..=9` carry fragments of its **value** (ASCII for
//!    string-typed rows, raw little-endian bytes for numeric-typed rows —
//!    this crate does not know a row's type, only its raw bytes). The
//!    stream covers every row across every page and **wraps back** to
//!    `FunctionCode 0x0000` when it finishes, which is the only observed
//!    end-of-stream signal (no explicit terminator frame).
//!
//! ## Single-row read (comm type `9`) — cheaper than the bulk table
//!
//! A separate, much cheaper mechanism (comm type `9`) reads **one**
//! `FunctionCode`'s value directly, without streaming the whole table.
//! Reverse-engineered from the same 2026-07-26 capture (see
//! [`build_read_param_frame_ext`] / [`parse_read_param_reply`]):
//!
//! 1. Host sends comm type `9`, `extra_data = host_id`, `device_id =
//!    target`, payload = `[row, page, 0, 0, 0, 0, 0, 0]`.
//! 2. The motor replies with **exactly 4** frames: comm type `9`,
//!    `extra_data = motor_id` (no slot bits this time — the sub-index rides
//!    in the payload instead), `device_id = host_id`. Payload is
//!    `[row, page, 0x0A, sub_index, ...4 data bytes]`, `sub_index` `0..=3`.
//!    Concatenating the four 4-byte chunks in `sub_index` order gives the
//!    same 16 bytes the bulk stream's value fragments carry (verified
//!    against `BootCodeVersion` / `BootBuildDate` — identical bytes either
//!    way). The `0x0A` byte's meaning is unconfirmed (possibly a type tag);
//!    it was constant across every capture and is not currently validated.

use crate::can_id::{build_can_id_raw, parse_can_id};

/// The comm type used for the bulk-table stream (see module docs).
pub const READ_PARAM_TABLE: u8 = 19;

/// Build the trigger frame. `device_uid` is the target's 8-byte MCU UID
/// (from a prior classic `GetDeviceId` query) — see the module docs for why
/// the request carries it instead of being a bare probe.
pub fn build_read_param_table_frame(
    host_id: u8,
    device_id: u8,
    device_uid: [u8; 8],
) -> (u32, [u8; 8]) {
    (
        build_can_id_raw(READ_PARAM_TABLE, host_id as u16, device_id),
        device_uid,
    )
}

/// The comm type used for the single-row read (see module docs).
pub const READ_PARAM: u8 = 9;

/// Build a single-row read request for `function_code` (`(page << 8) | row`).
pub fn build_read_param_frame_ext(host_id: u8, device_id: u8, function_code: u16) -> (u32, [u8; 8]) {
    let row = (function_code & 0xFF) as u8;
    let page = (function_code >> 8) as u8;
    (
        build_can_id_raw(READ_PARAM, host_id as u16, device_id),
        [row, page, 0, 0, 0, 0, 0, 0],
    )
}

/// One of the 4 reply frames to a single-row read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReadParamReply {
    /// `(page << 8) | row` — the `FunctionCode` this reply is for.
    pub function_code: u16,
    /// Sub-index `0..=3` — concatenate in order for the full 16-byte value.
    pub sub_index: u8,
    /// The id of the motor that sent this reply.
    pub motor_id: u8,
    /// 4 raw data bytes for this sub-frame.
    pub data: [u8; 4],
}

/// Parse a raw frame as a single-row-read reply. Returns `None` for frames
/// on a different comm type.
pub fn parse_read_param_reply(can_id: u32, data: &[u8]) -> Option<ReadParamReply> {
    if data.len() < 8 {
        return None;
    }
    let (comm_type, extra, _device_id) = parse_can_id(can_id);
    if comm_type != READ_PARAM {
        return None;
    }
    let mut chunk = [0u8; 4];
    chunk.copy_from_slice(&data[4..8]);
    Some(ReadParamReply {
        function_code: ((data[1] as u16) << 8) | data[0] as u16,
        sub_index: data[3],
        motor_id: (extra & 0xFF) as u8,
        data: chunk,
    })
}

/// One decoded fragment from the parameter-table stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParamTableFrame {
    /// `(page << 8) | row` — the `FunctionCode` this fragment belongs to.
    pub function_code: u16,
    /// Fragment slot: `0..=2` = name fragment, `6..=9` = value fragment.
    pub slot: u8,
    /// The id of the motor that sent this fragment.
    pub motor_id: u8,
    /// 6 raw payload bytes for this fragment.
    pub data: [u8; 6],
}

impl ParamTableFrame {
    /// `true` if this fragment belongs to the parameter's name (vs. value).
    pub const fn is_name_fragment(&self) -> bool {
        self.slot <= 2
    }
}

/// Parse a raw frame as a parameter-table stream fragment. Returns `None`
/// for frames on a different comm type (including our own request echoing
/// back, which shares comm type `19` but carries the UID payload rather
/// than this `[row, page, ...]` shape — harmless to misclassify since a
/// real UID's first two bytes essentially never collide with a valid
/// `(page, row)` pair the caller is looking for, but callers matching a
/// specific `function_code` are unaffected either way).
pub fn parse_param_table_frame(can_id: u32, data: &[u8]) -> Option<ParamTableFrame> {
    if data.len() < 8 {
        return None;
    }
    let (comm_type, extra, _device_id) = parse_can_id(can_id);
    if comm_type != READ_PARAM_TABLE {
        return None;
    }
    let slot = (extra >> 8) as u8;
    let motor_id = (extra & 0xFF) as u8;
    let mut fragment = [0u8; 6];
    fragment.copy_from_slice(&data[2..8]);
    Some(ParamTableFrame {
        function_code: ((data[1] as u16) << 8) | data[0] as u16,
        slot,
        motor_id,
        data: fragment,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trigger_frame_carries_the_uid_payload() {
        // Captured: 1300FD01 → C3 5C 3B C0 00 20 30 0F (RS04, motor id 1) —
        // now understood to be the request itself, not a reply.
        let uid = [0xC3, 0x5C, 0x3B, 0xC0, 0x00, 0x20, 0x30, 0x0F];
        let (can_id, data) = build_read_param_table_frame(0xFD, 1, uid);
        assert_eq!(can_id, (READ_PARAM_TABLE as u32) << 24 | (0xFDu32 << 8) | 1);
        assert_eq!(data, uid);
    }

    #[test]
    fn read_param_request_layout() {
        // BootBuildDate = FunctionCode 0x1001 → row=0x01, page=0x10.
        let (can_id, data) = build_read_param_frame_ext(0xFD, 1, 0x1001);
        assert_eq!(can_id, (READ_PARAM as u32) << 24 | (0xFDu32 << 8) | 1);
        assert_eq!(data, [0x01, 0x10, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn read_param_reply_reassembles_boot_build_date() {
        // Captured 4-frame reply for FunctionCode 0x1001 (BootBuildDate),
        // motor id 1: concatenated data bytes spell "Mar 26 2024\0".
        let can_id = 0x090001FD;
        let frames = [
            [0x01, 0x10, 0x0A, 0x00, b'M', b'a', b'r', b' '],
            [0x01, 0x10, 0x0A, 0x01, b'2', b'6', b' ', b'2'],
            [0x01, 0x10, 0x0A, 0x02, b'0', b'2', b'4', 0x00],
        ];
        let mut value = [0u8; 12];
        for (i, f) in frames.iter().enumerate() {
            let reply = parse_read_param_reply(can_id, f).unwrap();
            assert_eq!(reply.function_code, 0x1001);
            assert_eq!(reply.motor_id, 1);
            value[i * 4..i * 4 + 4].copy_from_slice(&reply.data);
        }
        assert_eq!(&value[..11], b"Mar 26 2024");
    }

    #[test]
    fn read_param_reply_ignores_other_comm_types() {
        let can_id = (2u32 << 24) | (1u32 << 8) | 0xFD;
        assert_eq!(parse_read_param_reply(can_id, &[0u8; 8]), None);
    }

    #[test]
    fn stream_fragment_decodes_function_code_and_slot() {
        // Captured: 130001FD → 03 10 41 70 70 43 6F 64 ("AppCod", name frag 0
        // of FunctionCode 0x1003 = AppCodeVersion, motor id 1).
        let can_id = 0x130001FD;
        let data = [0x03, 0x10, 0x41, 0x70, 0x70, 0x43, 0x6F, 0x64];
        let frag = parse_param_table_frame(can_id, &data).unwrap();
        assert_eq!(frag.function_code, 0x1003);
        assert_eq!(frag.slot, 0);
        assert_eq!(frag.motor_id, 1);
        assert_eq!(&frag.data, b"AppCod");
        assert!(frag.is_name_fragment());
    }

    #[test]
    fn value_fragment_slot_6_is_not_a_name_fragment() {
        // Captured: 130601FD → 03 10 30 2E 34 2E 31 2E ("0.4.1.", value frag
        // 0 of AppCodeVersion).
        let can_id = 0x130601FD;
        let data = [0x03, 0x10, 0x30, 0x2E, 0x34, 0x2E, 0x31, 0x2E];
        let frag = parse_param_table_frame(can_id, &data).unwrap();
        assert_eq!(frag.function_code, 0x1003);
        assert_eq!(frag.slot, 6);
        assert_eq!(&frag.data, b"0.4.1.");
        assert!(!frag.is_name_fragment());
    }

    #[test]
    fn ignores_frames_on_other_comm_types() {
        // A normal OperationStatus frame (comm_type=2) must not be mistaken
        // for a param-table fragment.
        let can_id = (2u32 << 24) | (0xFDu32 << 8) | 1;
        assert_eq!(parse_param_table_frame(can_id, &[0u8; 8]), None);
    }
}
