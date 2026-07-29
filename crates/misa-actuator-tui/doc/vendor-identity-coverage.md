# Model name / firmware version coverage across vendors (`misa-actuator-identify`)

`misa-actuator-identify` fetches model name and firmware version for each id
that responds to a vendor's `scan_bus`. Coverage differs per vendor because
it differs in the protocols themselves, not just this tool's implementation.
Checked against the official/local manuals available in this workspace.

| Vendor | Model name | Firmware version | Source checked |
|---|---|---|---|
| MyActuator | ✅ `0xB5` (chunked ASCII, e.g. `RMD-X4-P36-36`) | ✅ `0xB2` (plain u32, e.g. `2026042402`) | `myactuator-protocol/ref/` official manuals + live capture (`doc/setup-software-c0-param-protocol.md`) |
| RobStride | ❌ no wire read found | ✅ `read_firmware_version()` (comm_type 9, e.g. `0.4.1.32`) | `robstride-protocol/ref/rs04_manual_en.md`, `el05_manual_en.md` |
| DAMIAO | ❌ no ASCII/string register exists (confirmed against the full official Register Map — none of the ~50 registers is string-typed) | ✅ `Rid::SW_VER` (register 0x0E/14) — **updated**: official manuals now available confirm this, not `Rid::SUB_VER` (register 36), which this tool used before the manuals were added. See `damiao-protocol/doc/dm4310-dm3507-manual-analysis.md` | `damiao-protocol/ref/dm4310_manual_en.md`, `dm3507_manual_en.md` (official manuals added 2026-07-30) |
| LKMotor | ❌ not documented | ❌ not documented | `/home/takara/work/dp/lkmotor-driver/ref/can_protocol_desc_en.md` (V2.36, all 29 CAN commands) and `rs485_protocol_desc_en.md` (all 24 RS485 commands) — neither manual has a "read device info" / model / firmware command of any kind |

## Notes

- RobStride's model is only known because the caller passes it in
  (`--model`) at `open()` time — it isn't read back from the device, so
  there's no way to *verify* it against what's actually mounted on that id.
  Same for DAMIAO.
- LKMotor's gap is a genuine protocol limitation, not a driver oversight —
  both the CAN and RS485 command lists were read in full and neither has
  anything resembling a device-info read. (LKMotor manuals were also added to
  this workspace on 2026-07-30 — re-check if this needs revisiting once
  they're converted/reviewed.)
- DAMIAO's official manuals were added 2026-07-30 (`ref/DM-J4310-2EC ...pdf`,
  `ref/DM-J3507-2EC ...pdf`) — this table's DAMIAO row was corrected
  accordingly; see `damiao-protocol/doc/dm4310-dm3507-manual-analysis.md` for
  the full register-map cross-check.
