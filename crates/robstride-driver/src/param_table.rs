//! RobStride's undocumented bulk/single "parameter table" reads (`Name`,
//! `BootCodeVersion`, `AppCodeVersion`, tunable gains, telemetry, ...) —
//! the same data `motorstudio`'s "Parameter Setting" tab shows.
//!
//! Split out of `driver.rs` because it's a distinct, separately
//! reverse-engineered protocol layered on top of the classic private
//! protocol (see [`robstride_protocol::param_table`] for the wire format
//! and [`robstride_protocol::param_type_table`] for the official
//! FunctionCode→type mapping this module decodes against). The classic
//! motor-control methods in `driver.rs` don't depend on any of this.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use robstride_protocol::{
    build_read_param_frame_ext, build_read_param_table_frame, lookup_param_type,
    lookup_param_type_by_name, parse_param_table_frame, parse_read_param_reply, ParamType,
};

use crate::bus::RobstrideBus;
use crate::driver::Motor;
use crate::error::{Error, Result};

impl<B: RobstrideBus> Motor<B> {
    /// Read a single row's raw value from the extended `FunctionCode` space
    /// (e.g. `0x1003` = `AppCodeVersion`) without streaming the whole bulk
    /// table — see [`robstride_protocol::param_table`]'s "single-row read"
    /// section. Much cheaper than [`Self::read_param_table`]: a handful of
    /// reply frames instead of the full ~130-row stream.
    ///
    /// String-typed rows (the `0x1000`–`0x1007` boot/app info block) reply
    /// with 4 sub-frames (16 bytes). Numeric-typed rows (`0x2000+`) were
    /// observed replying with just **1** sub-frame carrying `CurrentValue`
    /// directly (not the `[min][max][?][current]` layout the bulk stream
    /// uses) — so the sub-frame count genuinely varies by row, and this
    /// collects whatever arrives within a quiet period rather than waiting
    /// for a fixed count (which would hang until `timeout` for rows that
    /// only ever send one frame).
    ///
    /// Returns the raw concatenated bytes (sub-frames in `sub_index`
    /// order); interpret as a NUL-padded ASCII string for string-typed rows
    /// (trim yourself, or use [`Self::read_firmware_version`] for the
    /// common case) or as raw little-endian bytes for numeric-typed rows —
    /// the exact numeric layout via this path is less confirmed than the
    /// bulk stream's, so prefer [`Self::read_param_table`]'s
    /// [`ParamTableEntry`] accessors when precision matters.
    pub fn read_single_param(&mut self, function_code: u16, timeout: Duration) -> Result<Vec<u8>> {
        let (id, data) = build_read_param_frame_ext(self.host_id(), self.motor_id(), function_code);
        self.send(id, &data)?;

        let mut chunks: BTreeMap<u8, [u8; 4]> = BTreeMap::new();
        let poll_step = Duration::from_millis(50);
        self.bus().set_timeout(poll_step)?;
        let deadline = Instant::now() + timeout;

        loop {
            if Instant::now() >= deadline {
                break;
            }
            let frame = match self.bus().recv() {
                Ok(f) => f,
                Err(Error::Timeout { .. }) => {
                    if chunks.is_empty() {
                        continue; // still within the overall deadline — keep waiting
                    }
                    break; // quiet period after data arrived — reply is done
                }
                Err(e) => return Err(e),
            };
            let Some(reply) = parse_read_param_reply(frame.can_id, &frame.data) else {
                continue;
            };
            if reply.function_code != function_code || reply.motor_id != self.motor_id() {
                continue;
            }
            chunks.insert(reply.sub_index, reply.data);
        }
        let default_timeout = self.timeout();
        self.bus().set_timeout(default_timeout)?;

        if chunks.is_empty() {
            return Err(Error::Timeout {
                motor_id: self.motor_id(),
            });
        }
        Ok(chunks.into_values().flatten().collect())
    }

    /// Read RobStride's undocumented bulk "parameter table" (`Name`,
    /// `BootCodeVersion`, `AppCodeVersion`, ... — the same table
    /// `motorstudio`'s "Parameter Setting" tab shows). See
    /// [`robstride_protocol::param_table`] for how this was reverse-engineered
    /// and its caveats — in particular, the trigger addresses the motor by
    /// MCU UID rather than being a bare probe, so this first does a classic
    /// `GetDeviceId` ping to learn it.
    ///
    /// Prefer [`Self::read_single_param`] / [`Self::read_firmware_version`]
    /// when you only need one row — this streams the entire table.
    ///
    /// Sends the trigger frame, then collects fragments until the stream
    /// wraps back to `FunctionCode 0x0000` (the only observed end marker) or
    /// `timeout` elapses without a new fragment arriving, whichever comes
    /// first — so a generous `timeout` (the whole table takes a noticeable
    /// fraction of a second to stream) is safer than a tight one that could
    /// cut the last few rows off.
    pub fn read_param_table(&mut self, timeout: Duration) -> Result<Vec<ParamTableEntry>> {
        // The trigger addresses the motor by UID (see
        // `robstride_protocol::param_table`), so fetch it first via the
        // classic GetDeviceId ping.
        let (_uid_extra, uid_payload) = self.ping()?;
        let mut uid = [0u8; 8];
        let n = uid_payload.len().min(8);
        uid[..n].copy_from_slice(&uid_payload[..n]);

        let (id, data) = build_read_param_table_frame(self.host_id(), self.motor_id(), uid);
        self.send(id, &data)?;

        let mut names: BTreeMap<u16, Vec<(u8, [u8; 6])>> = BTreeMap::new();
        let mut values: BTreeMap<u16, Vec<(u8, [u8; 6])>> = BTreeMap::new();
        let mut order: Vec<u16> = Vec::new();
        let mut first_fc: Option<u16> = None;

        // Poll in short steps so a quiet period (stream ended without
        // wrapping) is detected quickly, while `deadline` bounds the total
        // time spent regardless of how many fragments keep arriving.
        let poll_step = Duration::from_millis(50);
        self.bus().set_timeout(poll_step)?;
        let deadline = Instant::now() + timeout;

        let result = loop {
            if Instant::now() >= deadline {
                break if order.is_empty() {
                    Err(Error::Timeout {
                        motor_id: self.motor_id(),
                    })
                } else {
                    Ok(())
                };
            }
            let frame = match self.bus().recv() {
                Ok(f) => f,
                Err(Error::Timeout { .. }) => {
                    if order.is_empty() {
                        continue; // still within the overall deadline — keep waiting
                    }
                    break Ok(()); // quiet period after data arrived — stream ended
                }
                Err(e) => break Err(e),
            };
            let Some(frag) = parse_param_table_frame(frame.can_id, &frame.data) else {
                continue;
            };
            // Reject our own request's TX echo (some adapters loop it back
            // to the local socket) and any stray traffic from other motors —
            // both share comm type 19 with genuine stream fragments, but a
            // real fragment always carries *this* motor's id.
            if frag.motor_id != self.motor_id() {
                continue;
            }
            match first_fc {
                None => first_fc = Some(frag.function_code),
                Some(fc) if fc == frag.function_code && order.len() > 1 => break Ok(()),
                _ => {}
            }
            if !order.contains(&frag.function_code) {
                order.push(frag.function_code);
            }
            let bucket = if frag.is_name_fragment() {
                names.entry(frag.function_code).or_default()
            } else {
                values.entry(frag.function_code).or_default()
            };
            bucket.push((frag.slot, frag.data));
        };
        let default_timeout = self.timeout();
        self.bus().set_timeout(default_timeout)?;
        result?;

        Ok(order
            .into_iter()
            .map(|fc| ParamTableEntry {
                function_code: fc,
                name: reassemble_string(names.get(&fc)),
                value_raw: reassemble_raw(values.get(&fc)),
            })
            .collect())
    }

    /// Convenience wrapper: read just `AppCodeVersion` (`FunctionCode
    /// 0x1003`) as a string (e.g. `"0.4.1.32"`) via the cheap single-row
    /// read — does not stream the whole table.
    pub fn read_firmware_version(&mut self, timeout: Duration) -> Result<String> {
        const APP_CODE_VERSION: u16 = 0x1003;
        let raw = self.read_single_param(APP_CODE_VERSION, timeout)?;
        let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
        Ok(String::from_utf8_lossy(&raw[..end]).trim().to_string())
    }
}

/// One reassembled row of RobStride's undocumented bulk parameter table
/// (see [`robstride_protocol::param_table`] for how this is read).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParamTableEntry {
    /// `(page << 8) | row`, e.g. `0x1003` = `AppCodeVersion`.
    pub function_code: u16,
    /// The row's name (e.g. `"AppCodeVersion"`), reassembled from its ASCII
    /// name fragments and trimmed of NUL padding.
    pub name: String,
    /// Raw value bytes reassembled from the value fragments, in slot order.
    /// ASCII for string-typed rows (see [`Self::value_raw_as_string`]);
    /// interpret as raw little-endian bytes (e.g. via `f32::from_le_bytes`)
    /// for numeric-typed rows — the wire format has no type tag, so the
    /// caller must know which a given `function_code` is.
    pub value_raw: Vec<u8>,
}

impl ParamTableEntry {
    /// Value bytes trimmed at the first NUL and decoded as UTF-8 (lossy).
    /// Only meaningful for string-typed rows (`function_code < 0x2000` in
    /// every row observed so far — `Name`, `BarCode`, and the `0x1000`–
    /// `0x1007` boot/app info block).
    pub fn value_raw_as_string(&self) -> String {
        let end = self
            .value_raw
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(self.value_raw.len());
        String::from_utf8_lossy(&self.value_raw[..end])
            .trim()
            .to_string()
    }

    /// `MinValue`, decoded as `i16` (reverse-engineered from a live capture
    /// against an RS04, cross-checked against `motorstudio`'s displayed
    /// `MinValue`/`MaxValue`/`CurrentValue` columns for ~12 rows spanning
    /// `uint8`/`uint16`/`int16`/`uint32`/`float` types — see
    /// [`robstride_protocol::param_table`]). Only meaningful for
    /// numeric-typed rows (`function_code >= 0x2000`); layout is
    /// `[min_i16][max_i16][4 unidentified bytes][current, native width]`.
    /// Very large `uint32` ranges (e.g. `CAN_TIMEOUT`) may not fit in `i16`
    /// and will not decode correctly here.
    pub fn value_min_i16(&self) -> Option<i16> {
        self.value_raw
            .get(0..2)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
    }

    /// `MaxValue`, decoded as `i16`. See [`Self::value_min_i16`] for caveats.
    pub fn value_max_i16(&self) -> Option<i16> {
        self.value_raw
            .get(2..4)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
    }

    /// `CurrentValue` as `uint8` — for rows whose `motorstudio` type column
    /// reads `uint8`.
    pub fn value_current_u8(&self) -> Option<u8> {
        self.value_raw.get(8).copied()
    }

    /// `CurrentValue` as `int16` — for `int16`-typed rows.
    pub fn value_current_i16(&self) -> Option<i16> {
        self.value_raw
            .get(8..10)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
    }

    /// `CurrentValue` as `uint16` — for `uint16`-typed rows.
    pub fn value_current_u16(&self) -> Option<u16> {
        self.value_raw
            .get(8..10)
            .map(|b| u16::from_le_bytes([b[0], b[1]]))
    }

    /// `CurrentValue` as `uint32` — for `uint32`-typed rows.
    pub fn value_current_u32(&self) -> Option<u32> {
        self.value_raw
            .get(8..12)
            .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    /// `CurrentValue` as `int32` — for `int32`-typed rows. Same bytes as
    /// [`Self::value_current_u32`], reinterpreted as signed.
    pub fn value_current_i32(&self) -> Option<i32> {
        self.value_raw
            .get(8..12)
            .map(|b| i32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    /// `CurrentValue` as `float` — for `float`-typed rows. Verified
    /// bit-exact against `motorstudio` for several rows (e.g. `limit_spd`
    /// = `1.0` → `0x3F800000`, `limit_cur` = `97.0` → `0x42C20000`).
    pub fn value_current_f32(&self) -> Option<f32> {
        self.value_raw
            .get(8..12)
            .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    /// Decode `CurrentValue` using this row's declared type from the
    /// RS04/EL05 manual's parameter table (see
    /// [`robstride_protocol::param_type_table`]) — the authoritative
    /// alternative to guessing from [`Self::value_current_u8`] and friends.
    /// Matches by this row's wire-reported `name` first (falling back to
    /// `function_code`), since a live capture found at least one firmware
    /// build repurposes specific `FunctionCode`s relative to the manual
    /// while keeping names consistent — see `lookup_param_type`'s doc
    /// comment. Returns `None` if neither the name nor the code is in the
    /// table, or if `value_raw` is too short for the looked-up type.
    pub fn value_current_typed(&self) -> Option<TypedValue> {
        // Prefer matching by the wire-reported name (this row's actual
        // identity on this device) over the FunctionCode, which a live
        // capture found can be repurposed on some firmware builds (see
        // `lookup_param_type`'s doc comment).
        let info = lookup_param_type_by_name(&self.name)
            .or_else(|| lookup_param_type(self.function_code))?;
        Some(match info.ty {
            ParamType::String => TypedValue::String(self.value_raw_as_string()),
            ParamType::U8 => TypedValue::U8(self.value_current_u8()?),
            ParamType::U16 => TypedValue::U16(self.value_current_u16()?),
            ParamType::I16 => TypedValue::I16(self.value_current_i16()?),
            ParamType::U32 => TypedValue::U32(self.value_current_u32()?),
            ParamType::I32 => TypedValue::I32(self.value_current_i32()?),
            ParamType::F32 => TypedValue::F32(self.value_current_f32()?),
        })
    }
}

/// A [`ParamTableEntry`]'s `CurrentValue`, decoded per its official type
/// (see [`ParamTableEntry::value_current_typed`]).
#[derive(Debug, Clone, PartialEq)]
pub enum TypedValue {
    String(String),
    U8(u8),
    U16(u16),
    I16(i16),
    U32(u32),
    I32(i32),
    F32(f32),
}

impl std::fmt::Display for TypedValue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TypedValue::String(s) => write!(f, "{s:?}"),
            TypedValue::U8(v) => write!(f, "{v}"),
            TypedValue::U16(v) => write!(f, "{v}"),
            TypedValue::I16(v) => write!(f, "{v}"),
            TypedValue::U32(v) => write!(f, "{v}"),
            TypedValue::I32(v) => write!(f, "{v}"),
            TypedValue::F32(v) => write!(f, "{v}"),
        }
    }
}

/// Concatenate a row's fragments (sorted by slot) and decode as a trimmed
/// UTF-8 string. Used for both name and (string-typed) value reassembly.
fn reassemble_string(fragments: Option<&Vec<(u8, [u8; 6])>>) -> String {
    let raw = reassemble_raw(fragments);
    let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
    String::from_utf8_lossy(&raw[..end]).trim().to_string()
}

/// Concatenate a row's fragments (sorted by slot) into raw bytes.
fn reassemble_raw(fragments: Option<&Vec<(u8, [u8; 6])>>) -> Vec<u8> {
    let Some(fragments) = fragments else {
        return Vec::new();
    };
    let mut sorted = fragments.clone();
    sorted.sort_by_key(|(slot, _)| *slot);
    sorted.into_iter().flat_map(|(_, bytes)| bytes).collect()
}

#[cfg(test)]
mod tests {
    use super::{ParamTableEntry, TypedValue};

    fn entry(function_code: u16, value_raw: Vec<u8>) -> ParamTableEntry {
        ParamTableEntry {
            function_code,
            name: String::new(),
            value_raw,
        }
    }

    // All raw bytes below are live captures against a real RS04
    // (2026-07-26), cross-checked against motorstudio's displayed
    // MinValue/MaxValue/CurrentValue for the same FunctionCode.

    #[test]
    fn float_current_value_matches_motorstudio_exactly() {
        // limit_spd (0x2018): float, Current = 1.0.
        let e = entry(0x2018, vec![0, 0, 200, 0, 0, 102, 0, 6, 0, 0, 128, 63]);
        assert_eq!(e.value_current_f32(), Some(1.0));

        // limit_cur (0x2019): float, Current = 97.0.
        let e = entry(0x2019, vec![0, 0, 150, 0, 0, 102, 0, 6, 0, 0, 194, 66]);
        assert_eq!(e.value_current_f32(), Some(97.0));

        // cur_filt_gain (0x2011): float, Current = 0.1.
        let e = entry(0x2011, vec![0, 0, 1, 0, 0, 102, 0, 6, 205, 204, 204, 61]);
        assert!((e.value_current_f32().unwrap() - 0.1).abs() < 1e-6);
    }

    #[test]
    fn min_max_decode_as_i16() {
        // MechOffset (0x2005): float, Min = -50, Max = 50.
        let e = entry(
            0x2005,
            vec![206, 255, 50, 0, 0, 102, 206, 6, 171, 160, 120, 64],
        );
        assert_eq!(e.value_min_i16(), Some(-50));
        assert_eq!(e.value_max_i16(), Some(50));
        assert!((e.value_current_f32().unwrap() - 3.884_806).abs() < 1e-3);

        // status1 (0x2007): float, Min = -10, Max = 10.
        let e = entry(0x2007, vec![246, 255, 10, 0, 0, 6, 246, 6, 0, 0, 0, 0]);
        assert_eq!(e.value_min_i16(), Some(-10));
        assert_eq!(e.value_max_i16(), Some(10));

        // status2 (0x200C): int16, Min = -200, Max = 1500.
        let e = entry(0x200C, vec![56, 255, 220, 5, 0, 3, 56, 3, 0, 0, 0, 0]);
        assert_eq!(e.value_min_i16(), Some(-200));
        assert_eq!(e.value_max_i16(), Some(1500));
    }

    #[test]
    fn current_value_decodes_at_native_width() {
        // CAN_ID (0x2009): uint8, Min=0, Max=127, Current=1.
        let e = entry(0x2009, vec![0, 0, 127, 0, 0, 96, 0, 0, 1, 253, 0, 0]);
        assert_eq!(e.value_min_i16(), Some(0));
        assert_eq!(e.value_max_i16(), Some(127));
        assert_eq!(e.value_current_u8(), Some(1));

        // echoFreHz (0x2004): uint32, Min=1, Max=10000, Current=1000.
        let e = entry(0x2004, vec![1, 0, 16, 39, 0, 36, 1, 4, 232, 3, 0, 0]);
        assert_eq!(e.value_min_i16(), Some(1));
        assert_eq!(e.value_max_i16(), Some(10000));
        assert_eq!(e.value_current_u32(), Some(1000));

        // echoPara1 (0x2000): uint16, Min=5, Max=120, Current=5.
        let e = entry(0x2000, vec![5, 0, 120, 0, 0, 34, 5, 2, 5, 0, 5, 0]);
        assert_eq!(e.value_current_u16(), Some(5));
    }

    #[test]
    fn short_value_raw_returns_none_instead_of_panicking() {
        // 3 bytes: enough for value_min_i16 (needs 2), not for anything
        // reading past byte 3 — must return None, not panic on an
        // out-of-bounds slice.
        let e = entry(0x2000, vec![1, 2, 3]);
        assert_eq!(e.value_min_i16(), Some(i16::from_le_bytes([1, 2])));
        assert_eq!(e.value_max_i16(), None);
        assert_eq!(e.value_current_u8(), None);
        assert_eq!(e.value_current_f32(), None);

        let empty = entry(0x2000, vec![]);
        assert_eq!(empty.value_min_i16(), None);
    }

    #[test]
    fn value_current_typed_uses_the_official_manual_type() {
        // AppCodeVersion (0x1003): String per the manual — decodes even
        // though value_raw here is the 16-byte bulk-stream shape, not the
        // single-param-read's 4/12-byte shape (value_raw_as_string trims at
        // the first NUL either way).
        let e = entry(0x1003, b"0.4.1.32\0\0\0\0\0\0\0\0".to_vec());
        assert_eq!(
            e.value_current_typed(),
            Some(TypedValue::String("0.4.1.32".to_string()))
        );

        // MechOffset (0x2005): float per the manual.
        let e = entry(
            0x2005,
            vec![206, 255, 50, 0, 0, 102, 206, 6, 171, 160, 120, 64],
        );
        assert_eq!(e.value_current_typed(), Some(TypedValue::F32(3.884_806_4)));

        // CAN_ID (0x200A): uint8 per the manual.
        let e = entry(0x200A, vec![0, 0, 127, 0, 0, 96, 0, 0, 1, 253, 0, 0]);
        assert_eq!(e.value_current_typed(), Some(TypedValue::U8(1)));

        // status2 (0x200D): int16 per the manual.
        let e = entry(0x200D, vec![56, 255, 220, 5, 0, 3, 56, 3, 0, 0, 0, 0]);
        assert_eq!(e.value_current_typed(), Some(TypedValue::I16(0)));

        // Unlisted FunctionCode: no type known.
        let e = entry(0x0404, vec![2, 8, 45, 0, 75, 0]);
        assert_eq!(e.value_current_typed(), None);
    }

    #[test]
    fn value_current_typed_prefers_wire_reported_name_over_function_code() {
        // Live capture against a real RS04 (2026-07-26) found this unit's
        // firmware reports name "chasu_offset" at FunctionCode 0x2006,
        // where the manual lists "MechPos_init" — some firmware builds
        // repurpose specific codes. Simulate the mismatch with an
        // unmistakable case: an entry at an unlisted FunctionCode (so a
        // function_code-only lookup would fail) but whose wire-reported
        // name matches a real manual entry — the name match must still
        // resolve the type.
        let e = ParamTableEntry {
            function_code: 0x9999,
            name: "MechOffset".to_string(),
            // Bulk-stream shape: [min:2][max:2][unknown:4][current, 1.0f32].
            value_raw: vec![0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 63],
        };
        assert_eq!(e.value_current_typed(), Some(TypedValue::F32(1.0)));
    }

    #[test]
    fn typed_value_display_formats_each_variant() {
        assert_eq!(TypedValue::String("x".into()).to_string(), "\"x\"");
        assert_eq!(TypedValue::U8(1).to_string(), "1");
        assert_eq!(TypedValue::U16(2).to_string(), "2");
        assert_eq!(TypedValue::I16(-3).to_string(), "-3");
        assert_eq!(TypedValue::U32(4).to_string(), "4");
        assert_eq!(TypedValue::I32(-5).to_string(), "-5");
        assert_eq!(TypedValue::F32(1.5).to_string(), "1.5");
    }
}
