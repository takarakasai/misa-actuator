# LKMotor (Shanghai Lingkong Technology) — Motor CAN Bus Communication Protocol

**Document:** 电机 CAN 总线通讯协议 ("Motor CAN Bus Communication Protocol")
**Publisher:** Shanghai Lingkong Technology Co., Ltd. ("Lingkong Technology" / K-TECH / LKMotor)
**Version:** V2.36
**Source:** `can_protocol.pdf` (Chinese original, 23 pages), translated and independently verified page-by-page against the PDF for this document.

---

## Summary

This is a complete, faithful translation of the Chinese-language LKMotor "Motor CAN Bus Communication Protocol V2.36" manual, covering the disclaimer, CAN bus parameters, and all 29 single-motor CAN commands (status/error reads, on/off/stop, brake control, open-loop/torque/speed/position closed-loop control variants, control-parameter and setting-parameter read/write with their full parameter tables, encoder calibration/zeroing, angle reads, and save/restart).

**Firmware/software version, model/hardware version, and serial-number read commands: absent.** This document was read directly, page by page, and cross-checked against the raw `pdftotext` extraction and against the pre-existing English translation at `lkmotor-driver/ref/can_protocol_desc_en.md`. None of the 29 single-motor commands, and neither the Motor Control Parameter Table nor the Setting Parameter Table, expose any way to read a firmware/software version, a model name/number, a hardware revision, or a serial number. This independently re-confirms (against the Chinese source directly, not just the existing English translation) that LKMotor's CAN protocol V2.36 genuinely has no such command — it is not merely missing from the existing translation.

**Note on internal source inconsistency found during verification:** the "Current Ramp" / "电流斜率" parameter is declared `int32` (4 data bytes, command byte `0x24`) in the **Motor Control Parameter Table** (§19) but `int16` (2 data bytes, upper bytes reserved as `0x00`, sub-ID `0xEA`) in the **Setting Parameter Table** (§27) — both list the identical range 0–30000, which fits in 16 bits. This inconsistency exists in the Chinese PDF itself (confirmed on both the direct PDF page images and the raw-text extraction) and was not flagged in the pre-existing `can_protocol_desc_en.md` translation. See the note under §27's parameter table below.

No other numeric discrepancies were found between this translation, the direct PDF re-read, and the pre-existing `can_protocol_desc_en.md` translation — all command bytes, sub-IDs, ranges, and byte-layouts cross-check exactly (including two already-documented source-internal quirks: the 64-bit `motorAngle` value in §23 that is only carried in 7 of its 8 bytes over the wire, and the `maxSpeed` field in §17 that the prose calls `uint32_t` but the frame only reserves 2 bytes for).

---

## Table of Contents

- [Disclaimer](#disclaimer)
- [CAN Bus Parameters](#can-bus-parameters)
- [Single-Motor Commands](#single-motor-commands)
  1. [Read motor status 1 and error flags](#1-read-motor-status-1-and-error-flags)
  2. [Clear motor error flags](#2-clear-motor-error-flags)
  3. [Read motor status 2](#3-read-motor-status-2)
  4. [Read motor status 3](#4-read-motor-status-3)
  5. [Motor off](#5-motor-off)
  6. [Motor run](#6-motor-run)
  7. [Motor stop](#7-motor-stop)
  8. [Brake control and state read](#8-brake-control-and-state-read)
  9. [Open-loop control (MS motors only; no effect on others)](#9-open-loop-control-ms-motors-only-no-effect-on-others)
  10. [Torque closed-loop control (MF / MH / MG motors only)](#10-torque-closed-loop-control-mf--mh--mg-motors-only)
  11. [Speed closed-loop control](#11-speed-closed-loop-control)
  12. [Multi-turn position closed-loop control 1](#12-multi-turn-position-closed-loop-control-1)
  13. [Multi-turn position closed-loop control 2](#13-multi-turn-position-closed-loop-control-2)
  14. [Single-turn position closed-loop control 1](#14-single-turn-position-closed-loop-control-1)
  15. [Single-turn position closed-loop control 2](#15-single-turn-position-closed-loop-control-2)
  16. [Incremental position closed-loop control 1](#16-incremental-position-closed-loop-control-1)
  17. [Incremental position closed-loop control 2](#17-incremental-position-closed-loop-control-2)
  18. [Read control parameters](#18-read-control-parameters)
  19. [Write control parameters](#19-write-control-parameters)
  20. [Read motor encoder data](#20-read-motor-encoder-data)
  21. [Calibrate encoder](#21-calibrate-encoder)
  22. [Set current position as motor zero (write to ROM, persistent)](#22-set-current-position-as-motor-zero-write-to-rom-persistent)
  23. [Read multi-turn angle](#23-read-multi-turn-angle)
  24. [Read single-turn angle](#24-read-single-turn-angle)
  25. [Set current position as zero (write to RAM)](#25-set-current-position-as-zero-write-to-ram)
  26. [Read setting parameters](#26-read-setting-parameters)
  27. [Write setting parameters](#27-write-setting-parameters)
  28. [Save setting parameters](#28-save-setting-parameters)
  29. [Motor restart](#29-motor-restart)

---

## Disclaimer

Thank you for purchasing the integrated motor-drive control system from Shanghai Lingkong Technology Co., Ltd. Before use, please read this disclaimer carefully. Once you use the product, you are deemed to have acknowledged and accepted all of its content. Strictly observe the product manual, control protocol, and the relevant laws, regulations, policies, and guidelines when installing and using the product. While using the product, the user undertakes responsibility for their own actions and all consequences arising from them. Lingkong Technology assumes no legal responsibility for any losses caused by the user's improper use, installation, or modification.

"Lingkong Technology" is a trademark of Shanghai Lingkong Technology Co., Ltd. and its affiliates. The product names and brands referred to in this document are trademarks or registered trademarks of their respective companies.

This product and its manual are copyrighted by Shanghai Lingkong Technology Co., Ltd. They may not be reproduced or reprinted in any form without permission. The right of final interpretation of this disclaimer belongs to the company.

---

## CAN Bus Parameters

- **Bus interface:** CAN
- **Baud rate (normal mode, single-motor commands):**
  - 1 Mbps (default)
  - 500 kbps
  - 250 kbps
  - 125 kbps
  - 100 kbps
- **Baud rate (broadcast mode, multi-motor commands):**
  - 1 Mbps
  - 500 kbps

---

## Single-Motor Commands

Up to 32 drives may be attached to the same bus (depending on bus loading). To prevent bus collisions, each drive must be assigned a unique ID.

The host sends a single-motor command on the bus. The motor with the matching ID executes the command on receipt and, within a short time (≤ 0.25 ms), sends a reply back to the host. The command and reply message formats are as follows:

- **Command identifier:** `0x140 + ID` (ID = 1–32)
- **Reply identifier:** `0x140 + ID` (ID = 1–32)
- **Frame format:** Data frame
- **Frame type:** Standard frame
- **DLC:** 8 bytes

---

### 1. Read motor status 1 and error flags

Reads the motor's current temperature, voltage, and error-status flags.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9A` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply:** The motor replies to the host on receipt. The reply frame contains:

1. `temperature` — motor temperature (`int8_t`, 1 ℃/LSB).
2. `voltage` — bus voltage (`int16_t`, 0.01 V/LSB).
3. `current` — bus current (`int16_t`, 0.01 A/LSB).
4. `motorState` — motor state (`uint8_t`; each bit represents a distinct motor state).
5. `errorState` — error flags (`uint8_t`; each bit represents a distinct motor error state).

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9A` |
| `DATA[1]` | Motor temperature | `DATA[1] = *(uint8_t *)(&temperature)` |
| `DATA[2]` | Bus voltage low byte | `DATA[2] = *(uint8_t *)(&voltage)` |
| `DATA[3]` | Bus voltage high byte | `DATA[3] = *((uint8_t *)(&voltage)+1)` |
| `DATA[4]` | Bus current low byte | `DATA[4] = *(uint8_t *)(&current)` |
| `DATA[5]` | Bus current high byte | `DATA[5] = *((uint8_t *)(&current)+1)` |
| `DATA[6]` | Motor state byte | `DATA[6] = motorState` |
| `DATA[7]` | Error state byte | `DATA[7] = errorState` |

**Notes:**

1. `motorState = 0x00`: motor is in the on state; `motorState = 0x10`: motor is in the off state.
2. `errorState` bit definitions:

| `errorState` bit | Meaning | `0` | `1` |
| --- | --- | --- | --- |
| 0 | Low-voltage state | Normal | Low-voltage protection |
| 1 | High-voltage state | Normal | High-voltage protection |
| 2 | Drive temperature state | Normal | Drive over-temperature |
| 3 | Motor temperature state | Normal | Motor over-temperature |
| 4 | Motor current state | Normal | Motor over-current |
| 5 | Motor short-circuit state | Normal | Motor short circuit |
| 6 | Stall state | Normal | Motor stalled |
| 7 | Input-signal state | Normal | Input signal lost / timed out |

---

### 2. Clear motor error flags

Clears the motor's current error state; the motor replies on receipt.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9B` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply:** The motor replies to the host on receipt. Reply data layout is identical to **Read motor status 1 and error flags** (only the command byte `DATA[0]` differs — here `0x9B`).

**Notes:**

1. The error flags cannot be cleared while the motor state has not yet returned to normal.

---

### 3. Read motor status 2

Reads the motor's current temperature, torque current (MF, MG) / output power (MS), speed, and encoder position.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9C` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply:** The motor replies to the host on receipt. The reply frame contains:

1. `temperature` — motor temperature (`int8_t`, 1 ℃/LSB).
2. `iq` (MF, MG motors) or `power` (MS motors), `int16_t`. MG `iq` resolution is (66/4096 A)/LSB; MF `iq` resolution is (33/4096 A)/LSB. MS `power` ranges −1000 to 1000.
3. `speed` — motor speed (`int16_t`, 1 dps/LSB).
4. `encoder` — encoder value (`uint16_t`; 14-bit encoder range 0–16383, 15-bit encoder range 0–32767, 16-bit encoder range 0–65535).

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9C` |
| `DATA[1]` | Motor temperature | `DATA[1] = *(uint8_t *)(&temperature)` |
| `DATA[2]` | Torque current low byte / output power low byte (MS series) | `DATA[2] = *(uint8_t *)(&iq)` / `DATA[2] = *(uint8_t *)(&power)` |
| `DATA[3]` | Torque current high byte / output power high byte (MS series) | `DATA[3] = *((uint8_t *)(&iq)+1)` / `DATA[3] = *((uint8_t *)(&power)+1)` |
| `DATA[4]` | Motor speed low byte | `DATA[4] = *(uint8_t *)(&speed)` |
| `DATA[5]` | Motor speed high byte | `DATA[5] = *((uint8_t *)(&speed)+1)` |
| `DATA[6]` | Encoder position low byte | `DATA[6] = *(uint8_t *)(&encoder)` |
| `DATA[7]` | Encoder position high byte | `DATA[7] = *((uint8_t *)(&encoder)+1)` |

---

### 4. Read motor status 3

MS motors do not sample phase currents, so this command has no effect on MS motors.

Reads the motor's current temperature and 3-phase current data.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9D` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply:** The motor replies to the host on receipt. The reply frame contains:

1. `temperature` — motor temperature (`int8_t`, 1 ℃/LSB).
2. `iA`, `iB`, `iC` — phase current data, `int16_t`. MG phase-current resolution is (66/4096 A)/LSB; MF phase-current resolution is (33/4096 A)/LSB.

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9D` |
| `DATA[1]` | Motor temperature | `DATA[1] = *(uint8_t *)(&temperature)` |
| `DATA[2]` | Phase A current low byte | `DATA[2] = *(uint8_t *)(&iA)` |
| `DATA[3]` | Phase A current high byte | `DATA[3] = *((uint8_t *)(&iA)+1)` |
| `DATA[4]` | Phase B current low byte | `DATA[4] = *(uint8_t *)(&iB)` |
| `DATA[5]` | Phase B current high byte | `DATA[5] = *((uint8_t *)(&iB)+1)` |
| `DATA[6]` | Phase C current low byte | `DATA[6] = *(uint8_t *)(&iC)` |
| `DATA[7]` | Phase C current high byte | `DATA[7] = *((uint8_t *)(&iC)+1)` |

---

### 5. Motor off

Switches the motor from the on state (the default after power-up) to the off state; clears the accumulated turn count and any previously received control commands. The LED changes from solid on to a slow blink. While off, the motor still replies to commands but does not perform any motion.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x80` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply:** Identical to the host frame.

---

### 6. Motor run

Switches the motor from the off state to the on state; the LED changes from a slow blink to solid on. Subsequent control commands will then drive the motor.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x88` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply:** Identical to the host frame.

---

### 7. Motor stop

Stops the motor without clearing its run state. Sending a control command again will resume motor motion.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x81` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply (1 frame):** Identical to the host frame.

---

### 8. Brake control and state read

Controls engagement/release of the brake, or reads the brake's current state.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x8C` |
| `DATA[1]` | Brake state control/read byte | `0x00`: brake de-energized, brake engaged<br>`0x01`: brake energized, brake released<br>`0x10`: read brake state |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply (1 frame):**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x8C` |
| `DATA[1]` | Brake state byte | `0x00`: brake is in the de-energized state, brake engaged<br>`0x01`: brake is in the energized state, brake released |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

---

### 9. Open-loop control (MS motors only; no effect on others)

The host sends this command to control the open-loop voltage output to the motor. The control value `powerControl` is `int16_t`, range −850 to 850 (motor current and torque vary by motor).

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA0` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Open-loop control value low byte | `DATA[4] = *(uint8_t *)(&powerControl)` |
| `DATA[5]` | Open-loop control value high byte | `DATA[5] = *((uint8_t *)(&powerControl)+1)` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Notes:**

1. The control value `powerControl` in this command is not bounded by the upper computer's **Max Power** value.

**Drive reply (1 frame):** The motor replies to the host on receipt. Reply data layout is identical to **Read motor status 2** (only the command byte `DATA[0]` differs — here `0xA0`).

---

### 10. Torque closed-loop control (MF / MH / MG motors only)

The host sends this command to control the motor's torque-current output. The control value `iqControl` is `int16_t`, range −2048 to 2048, corresponding to an actual torque-current range of −16.5 A to 16.5 A for MF motors and −33 A to 33 A for MG motors. Bus current and the motor's actual torque vary by motor.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA1` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Torque current control value low byte | `DATA[4] = *(uint8_t *)(&iqControl)` |
| `DATA[5]` | Torque current control value high byte | `DATA[5] = *((uint8_t *)(&iqControl)+1)` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Notes:**

1. The control value `iqControl` in this command is not bounded by the upper computer's **Max Torque Current** value.

**Drive reply:** The motor replies to the host on receipt. Reply data layout is identical to **Read motor status 2** (only the command byte `DATA[0]` differs — here `0xA1`).

---

### 11. Speed closed-loop control

The host sends this command to control the motor's speed, with a simultaneous torque limit. The control value `speedControl` is `int32_t`, corresponding to an actual speed of 0.01 dps/LSB; the control value `iqControl` is `int16_t`, range −2048 to 2048, corresponding to an actual torque-current range of −16.5 A to 16.5 A for MF motors and −33 A to 33 A for MG motors. Bus current and the motor's actual torque vary by motor.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA2` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | Torque current limit value low byte | `DATA[2] = *(uint8_t *)(&iqControl)` |
| `DATA[3]` | Torque current limit value high byte | `DATA[3] = *((uint8_t *)(&iqControl)+1)` |
| `DATA[4]` | Speed control low byte | `DATA[4] = *(uint8_t *)(&speedControl)` |
| `DATA[5]` | Speed control | `DATA[5] = *((uint8_t *)(&speedControl)+1)` |
| `DATA[6]` | Speed control | `DATA[6] = *((uint8_t *)(&speedControl)+2)` |
| `DATA[7]` | Speed control high byte | `DATA[7] = *((uint8_t *)(&speedControl)+3)` |

**Notes:**

1. The motor's `speedControl` under this command is bounded by the upper computer's **Max Speed** value.
2. In this control mode, the motor's maximum acceleration is bounded by the upper computer's **Max Acceleration** value.

**Drive reply:** The motor replies to the host on receipt. Reply data layout is identical to **Read motor status 2** (only the command byte `DATA[0]` differs — here `0xA2`).

---

### 12. Multi-turn position closed-loop control 1

The host sends this command to control the motor's position (multi-turn angle). The control value `angleControl` is `int32_t`, corresponding to an actual position of 0.01 degree/LSB (i.e. 36000 represents 360°). The motor's rotation direction is determined by the difference between the target position and the current position.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA3` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Position control low byte | `DATA[4] = *(uint8_t *)(&angleControl)` |
| `DATA[5]` | Position control | `DATA[5] = *((uint8_t *)(&angleControl)+1)` |
| `DATA[6]` | Position control | `DATA[6] = *((uint8_t *)(&angleControl)+2)` |
| `DATA[7]` | Position control high byte | `DATA[7] = *((uint8_t *)(&angleControl)+3)` |

**Notes:**

1. The control value `angleControl` under this command is bounded by the upper computer's **Max Angle** value.
2. The motor's maximum speed under this command is bounded by the upper computer's **Max Speed** value.
3. In this control mode, the motor's maximum acceleration is bounded by the upper computer's **Max Acceleration** value.
4. In this control mode, the maximum torque current of MF, MH, MG motors is bounded by the upper computer's **Max Torque Current** value; the maximum power of MS motors is bounded by the upper computer's **Max Power** value.

**Drive reply:** The motor replies to the host on receipt. Reply data layout is identical to **Read motor status 2** (only the command byte `DATA[0]` differs — here `0xA3`).

---

### 13. Multi-turn position closed-loop control 2

The host sends this command to control the motor's position (multi-turn angle).

1. The control value `angleControl` is `int32_t`, corresponding to an actual position of 0.01 degree/LSB (i.e. 36000 represents 360°). The motor's rotation direction is determined by the difference between the target position and the current position.
2. The control value `maxSpeed` limits the motor's maximum rotation speed; it is `uint16_t`, corresponding to an actual speed of 1 dps/LSB (i.e. 360 represents 360 dps).

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA4` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | Speed limit low byte | `DATA[2] = *(uint8_t *)(&maxSpeed)` |
| `DATA[3]` | Speed limit high byte | `DATA[3] = *((uint8_t *)(&maxSpeed)+1)` |
| `DATA[4]` | Position control low byte | `DATA[4] = *(uint8_t *)(&angleControl)` |
| `DATA[5]` | Position control | `DATA[5] = *((uint8_t *)(&angleControl)+1)` |
| `DATA[6]` | Position control | `DATA[6] = *((uint8_t *)(&angleControl)+2)` |
| `DATA[7]` | Position control high byte | `DATA[7] = *((uint8_t *)(&angleControl)+3)` |

**Notes:**

1. The control value `angleControl` under this command is bounded by the upper computer's **Max Angle** value.
2. In this control mode, the motor's maximum acceleration is bounded by the upper computer's **Max Acceleration** value.
3. In this control mode, the maximum torque current of MF, MH, MG motors is bounded by the upper computer's **Max Torque Current** value; the maximum power of MS motors is bounded by the upper computer's **Max Power** value.

**Drive reply (1 frame):** The motor replies to the host on receipt. Reply data layout is identical to **Read motor status 2** (only the command byte `DATA[0]` differs — here `0xA4`).

---

### 14. Single-turn position closed-loop control 1

The host sends this command to control the motor's position (single-turn angle).

1. The control value `spinDirection` sets the motor's rotation direction; it is `uint8_t`, `0x00` for clockwise, `0x01` for counterclockwise.
2. The control value `angleControl` is `uint32_t`, corresponding to an actual position of 0.01 degree/LSB (i.e. 36000 represents 360°).

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA5` |
| `DATA[1]` | Rotation direction byte | `DATA[1] = spinDirection` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Position control byte 1 (bit 0 : bit 7) | `DATA[4] = *(uint8_t *)(&angleControl)` |
| `DATA[5]` | Position control byte 2 (bit 8 : bit 15) | `DATA[5] = *((uint8_t *)(&angleControl)+1)` |
| `DATA[6]` | Position control byte 3 (bit 16 : bit 23) | `DATA[6] = *((uint8_t *)(&angleControl)+2)` |
| `DATA[7]` | Position control byte 4 (bit 24 : bit 31) | `DATA[7] = *((uint8_t *)(&angleControl)+3)` |

**Notes:**

1. The motor's maximum speed under this command is bounded by the upper computer's **Max Speed** value.
2. In this control mode, the motor's maximum acceleration is bounded by the upper computer's **Max Acceleration** value.
3. In this control mode, the maximum torque current of MF, MH, MG motors is bounded by the upper computer's **Max Torque Current** value; the maximum power of MS motors is bounded by the upper computer's **Max Power** value.

**Drive reply:** The motor replies to the host on receipt. Reply data layout is identical to **Read motor status 2** (only the command byte `DATA[0]` differs — here `0xA5`).

---

### 15. Single-turn position closed-loop control 2

The host sends this command to control the motor's position (single-turn angle).

1. The control value `spinDirection` sets the motor's rotation direction; it is `uint8_t`, `0x00` for clockwise, `0x01` for counterclockwise.
2. `angleControl` is `uint32_t`, corresponding to an actual position of 0.01 degree/LSB (i.e. 36000 represents 360°).
3. The speed control value `maxSpeed` limits the motor's maximum rotation speed; it is `uint16_t`, corresponding to an actual speed of 1 dps/LSB (i.e. 360 represents 360 dps).

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA6` |
| `DATA[1]` | Rotation direction byte | `DATA[1] = spinDirection` |
| `DATA[2]` | Speed limit byte 1 (bit 0 : bit 7) | `DATA[2] = *(uint8_t *)(&maxSpeed)` |
| `DATA[3]` | Speed limit byte 2 (bit 8 : bit 15) | `DATA[3] = *((uint8_t *)(&maxSpeed)+1)` |
| `DATA[4]` | Position control byte 1 (bit 0 : bit 7) | `DATA[4] = *(uint8_t *)(&angleControl)` |
| `DATA[5]` | Position control byte 2 (bit 8 : bit 15) | `DATA[5] = *((uint8_t *)(&angleControl)+1)` |
| `DATA[6]` | Position control byte 3 (bit 16 : bit 23) | `DATA[6] = *((uint8_t *)(&angleControl)+2)` |
| `DATA[7]` | Position control byte 4 (bit 24 : bit 31) | `DATA[7] = *((uint8_t *)(&angleControl)+3)` |

**Notes:**

1. In this control mode, the motor's maximum acceleration is bounded by the upper computer's **Max Acceleration** value.
2. In this control mode, the maximum torque current of MF, MH, MG motors is bounded by the upper computer's **Max Torque Current** value; the maximum power of MS motors is bounded by the upper computer's **Max Power** value.

**Drive reply (1 frame):** The motor replies to the host on receipt. Reply data layout is identical to **Read motor status 2** (only the command byte `DATA[0]` differs — here `0xA6`).

---

### 16. Incremental position closed-loop control 1

The host sends this command to control the motor's position increment. The control value `angleIncrement` is `int32_t`, corresponding to an actual position of 0.01 degree/LSB (i.e. 36000 represents 360°). The motor's rotation direction is determined by the sign of this parameter.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA7` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Position control low byte | `DATA[4] = *(uint8_t *)(&angleIncrement)` |
| `DATA[5]` | Position control | `DATA[5] = *((uint8_t *)(&angleIncrement)+1)` |
| `DATA[6]` | Position control | `DATA[6] = *((uint8_t *)(&angleIncrement)+2)` |
| `DATA[7]` | Position control high byte | `DATA[7] = *((uint8_t *)(&angleIncrement)+3)` |

**Notes:**

1. The motor's maximum speed under this command is bounded by the upper computer's **Max Speed** value.
2. In this control mode, the motor's maximum acceleration is bounded by the upper computer's **Max Acceleration** value.
3. In this control mode, the maximum torque current of MF, MH, MG motors is bounded by the upper computer's **Max Torque Current** value; the maximum power of MS motors is bounded by the upper computer's **Max Power** value.

**Drive reply:** The motor replies to the host on receipt. Reply data layout is identical to **Read motor status 2** (only the command byte `DATA[0]` differs — here `0xA7`).

---

### 17. Incremental position closed-loop control 2

The host sends this command to control the motor's position increment.

1. The control value `angleIncrement` is `int32_t`, corresponding to an actual position of 0.01 degree/LSB (i.e. 36000 represents 360°). The motor's rotation direction is determined by the sign of this parameter.
2. The control value `maxSpeed` limits the motor's maximum rotation speed; per the source text it is `uint32_t`, corresponding to an actual speed of 1 dps/LSB (i.e. 360 represents 360 dps).

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA8` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | Speed limit low byte | `DATA[2] = *(uint8_t *)(&maxSpeed)` |
| `DATA[3]` | Speed limit high byte | `DATA[3] = *((uint8_t *)(&maxSpeed)+1)` |
| `DATA[4]` | Position control low byte | `DATA[4] = *(uint8_t *)(&angleIncrement)` |
| `DATA[5]` | Position control | `DATA[5] = *((uint8_t *)(&angleIncrement)+1)` |
| `DATA[6]` | Position control | `DATA[6] = *((uint8_t *)(&angleIncrement)+2)` |
| `DATA[7]` | Position control high byte | `DATA[7] = *((uint8_t *)(&angleIncrement)+3)` |

**Note:** the source prose declares `maxSpeed` as `uint32_t`, but the frame layout only reserves 2 bytes for it (`DATA[2]`/`DATA[3]`) — unlike command 13 (`0xA4`), which declares the equivalent field `uint16_t` and likewise uses 2 bytes. This is an internal inconsistency in the source document; the effective wire range matches a `uint16_t`, not a full `uint32_t`. Verify against firmware/observed traffic before relying on a 32-bit range.

**Notes:**

1. In this control mode, the motor's maximum acceleration is bounded by the upper computer's **Max Acceleration** value.
2. In this control mode, the maximum torque current of MF, MH, MG motors is bounded by the upper computer's **Max Torque Current** value; the maximum power of MS motors is bounded by the upper computer's **Max Power** value.

**Drive reply:** The motor replies to the host on receipt. Reply data layout is identical to **Read motor status 2** (only the command byte `DATA[0]` differs — here `0xA8`).

---

### 18. Read control parameters

The host sends this command to read the current control parameter from RAM. The parameter read is selected by the index `controlParamID` — see the **Motor Control Parameter Table** below.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xC0` |
| `DATA[1]` | Control parameter index | `DATA[1] = controlParamID` |
| `DATA[2]` | NULL | `DATA[2] = 0x00` |
| `DATA[3]` | NULL | `DATA[3] = 0x00` |
| `DATA[4]` | NULL | `DATA[4] = 0x00` |
| `DATA[5]` | NULL | `DATA[5] = 0x00` |
| `DATA[6]` | NULL | `DATA[6] = 0x00` |
| `DATA[7]` | NULL | `DATA[7] = 0x00` |

**Drive reply:** The reply data contains the read parameter value — see the **Motor Control Parameter Table**.

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xC0` |
| `DATA[1]` | Control parameter index | `DATA[1] = controlParamID` |
| `DATA[2]` | Control parameter byte 1 | Control Parameter |
| `DATA[3]` | Control parameter byte 2 | Control Parameter |
| `DATA[4]` | Control parameter byte 3 | Control Parameter |
| `DATA[5]` | Control parameter byte 4 | Control Parameter |
| `DATA[6]` | Control parameter byte 5 | Control Parameter |
| `DATA[7]` | Control parameter byte 6 | Control Parameter |

---

### 19. Write control parameters

The host sends this command to write a control parameter into RAM — it takes effect immediately, but is lost on power-down. Control parameters and their indices are given in the **Motor Control Parameter Table** below.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xC1` |
| `DATA[1]` | Control parameter index | `DATA[1] = controlParamID` |
| `DATA[2]` | Control parameter byte 1 | Control Parameter |
| `DATA[3]` | Control parameter byte 2 | Control Parameter |
| `DATA[4]` | Control parameter byte 3 | Control Parameter |
| `DATA[5]` | Control parameter byte 4 | Control Parameter |
| `DATA[6]` | Control parameter byte 5 | Control Parameter |
| `DATA[7]` | Control parameter byte 6 | Control Parameter |

**Drive reply:** The reply data contains the parameter value after the write — see the **Motor Control Parameter Table**.

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xC1` |
| `DATA[1]` | Control parameter index | `DATA[1] = controlParamID` |
| `DATA[2]` | Control parameter byte 1 | Control Parameter |
| `DATA[3]` | Control parameter byte 2 | Control Parameter |
| `DATA[4]` | Control parameter byte 3 | Control Parameter |
| `DATA[5]` | Control parameter byte 4 | Control Parameter |
| `DATA[6]` | Control parameter byte 5 | Control Parameter |
| `DATA[7]` | Control parameter byte 6 | Control Parameter |

#### Motor Control Parameter Table

**Position Loop PID** — Data Range: 0–2000, Data Type: `uint16`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x0A` |
| `DATA[2]` | Position Loop Kp Bit 7:0 |
| `DATA[3]` | Position Loop Kp Bit 15:8 |
| `DATA[4]` | Position Loop Ki Bit 7:0 |
| `DATA[5]` | Position Loop Ki Bit 15:8 |
| `DATA[6]` | Position Loop Kd Bit 7:0 |
| `DATA[7]` | Position Loop Kd Bit 15:8 |

**Speed Loop PID** — Data Range: 0–2000, Data Type: `uint16`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x0B` |
| `DATA[2]` | Speed Loop Kp Bit 7:0 |
| `DATA[3]` | Speed Loop Kp Bit 15:8 |
| `DATA[4]` | Speed Loop Ki Bit 7:0 |
| `DATA[5]` | Speed Loop Ki Bit 15:8 |
| `DATA[6]` | Speed Loop Kd Bit 7:0 |
| `DATA[7]` | Speed Loop Kd Bit 15:8 |

**Current Loop PID** — Data Range: 0–2000, Data Type: `uint16`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x0C` |
| `DATA[2]` | Current Loop Kp Bit 7:0 |
| `DATA[3]` | Current Loop Kp Bit 15:8 |
| `DATA[4]` | Current Loop Ki Bit 7:0 |
| `DATA[5]` | Current Loop Ki Bit 15:8 |
| `DATA[6]` | Current Loop Kd Bit 7:0 |
| `DATA[7]` | Current Loop Kd Bit 15:8 |

**Torque Limit** — Data Range: 0–850 (MS series); 0–2000 (MF, MHF, MG series), Data Type: `int16`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x1E` |
| `DATA[2]` | `0x00` |
| `DATA[3]` | `0x00` |
| `DATA[4]` | Torque Limit Bit 7:0 |
| `DATA[5]` | Torque Limit Bit 15:8 |
| `DATA[6]` | `0x00` |
| `DATA[7]` | `0x00` |

**Speed Limit** — Data Range: 0–600000, Unit: 0.01 dps, Data Type: `int32`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x20` |
| `DATA[2]` | `0x00` |
| `DATA[3]` | `0x00` |
| `DATA[4]` | Speed Limit Bit 7:0 |
| `DATA[5]` | Speed Limit Bit 15:8 |
| `DATA[6]` | Speed Limit Bit 23:16 |
| `DATA[7]` | Speed Limit Bit 31:24 |

**Angle Limit** — Data Range: 0–(2³¹ − 1), Unit: 0.01 deg, Data Type: `int32`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x22` |
| `DATA[2]` | `0x00` |
| `DATA[3]` | `0x00` |
| `DATA[4]` | Angle Limit Bit 7:0 |
| `DATA[5]` | Angle Limit Bit 15:8 |
| `DATA[6]` | Angle Limit Bit 23:16 |
| `DATA[7]` | Angle Limit Bit 31:24 |

**Current Ramp** — Data Range: 0–30000, Data Type: `int32`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x24` |
| `DATA[2]` | `0x00` |
| `DATA[3]` | `0x00` |
| `DATA[4]` | Current Ramp Bit 7:0 |
| `DATA[5]` | Current Ramp Bit 15:8 |
| `DATA[6]` | Current Ramp Bit 23:16 |
| `DATA[7]` | Current Ramp Bit 31:24 |

**Note**: this entry declares Current Ramp as `int32` using all 4 data bytes. Compare with the **Current Ramp** entry in the [Setting Parameter Table](#27-write-setting-parameters) below (sub-ID `0xEA`), which declares the *same* parameter name with the *same* range (0–30000) as `int16`, using only 2 data bytes (`DATA[6]`/`DATA[7]` fixed at `0x00`). This is an inconsistency present in the Chinese source PDF itself (confirmed directly on both the source page images and the raw-text extraction, independent of the pre-existing `can_protocol_desc_en.md` translation, which did not call this out). Since 30000 fits within a 16-bit range, the Setting Parameter Table's `int16`/2-byte form is plausible as the authoritative layout, and this table's declared 4-byte layout may be a documentation copy-paste error from the neighboring Speed Ramp entry — but this cannot be confirmed without testing against real hardware/firmware.

**Speed Ramp** — Data Range: 0–600000, Unit: 1 dps/s, Data Type: `int32`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x26` |
| `DATA[2]` | `0x00` |
| `DATA[3]` | `0x00` |
| `DATA[4]` | Speed Ramp Bit 7:0 |
| `DATA[5]` | Speed Ramp Bit 15:8 |
| `DATA[6]` | Speed Ramp Bit 23:16 |
| `DATA[7]` | Speed Ramp Bit 31:24 |

---

### 20. Read motor encoder data

The host sends this command to read the encoder's current position.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x90` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply:** The motor replies to the host on receipt. The reply frame contains:

1. `encoder` — encoder position (`uint16_t`, 14-bit encoder range 0–16383); equal to the raw encoder position minus the encoder zero offset.
2. `encoderRaw` — raw encoder position (`uint16_t`, 14-bit encoder range 0–16383).
3. `encoderOffset` — encoder zero offset (`uint16_t`, 14-bit encoder range 0–16383); this point serves as the motor's 0° angle.

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x90` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | Encoder position low byte | `DATA[2] = *(uint8_t *)(&encoder)` |
| `DATA[3]` | Encoder position high byte | `DATA[3] = *((uint8_t *)(&encoder)+1)` |
| `DATA[4]` | Raw encoder position low byte | `DATA[4] = *(uint8_t *)(&encoderRaw)` |
| `DATA[5]` | Raw encoder position high byte | `DATA[5] = *((uint8_t *)(&encoderRaw)+1)` |
| `DATA[6]` | Encoder zero offset low byte | `DATA[6] = *(uint8_t *)(&encoderOffset)` |
| `DATA[7]` | Encoder zero offset high byte | `DATA[7] = *((uint8_t *)(&encoderOffset)+1)` |

---

### 21. Calibrate encoder

Calibrates the encoder; only needs to be performed once, and the calibration value is stored in ROM, permanently effective.

> **Caution:**
>
> 1. This command writes calibration-related parameters to ROM. Repeated writes will affect chip lifespan — frequent use is not recommended.
> 2. The motor should be unloaded or lightly loaded during calibration.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x18` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply:** The motor performs calibration on receipt of this command — during calibration the motor rotates back and forth for several seconds, then replies to the host once calibration completes. The reply frame contains:

1. `AlignValue` — calibration value, `uint32_t`.
2. `AlignRatio` — calibration ratio, `uint16_t`; should be near 1000 — the closer to 1000, the better the calibration result.
3. `AlignState` — calibration status byte. Bit 4 indicates the detected phase sequence (`0`: forward; `1`: reversed). Bit 0 indicates whether calibration succeeded (`0`: failure; `1`: success).

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x18` |
| `DATA[1]` | Calibration value byte 1 | `DATA[1] = *(uint8_t *)(&AlignValue)` |
| `DATA[2]` | Calibration value byte 2 | `DATA[2] = *((uint8_t *)(&AlignValue)+1)` |
| `DATA[3]` | Calibration value byte 3 | `DATA[3] = *((uint8_t *)(&AlignValue)+2)` |
| `DATA[4]` | Calibration value byte 4 | `DATA[4] = *((uint8_t *)(&AlignValue)+3)` |
| `DATA[5]` | Calibration ratio low byte | `DATA[5] = *(uint8_t *)(&AlignRatio)` |
| `DATA[6]` | Calibration ratio high byte | `DATA[6] = *((uint8_t *)(&AlignRatio)+1)` |
| `DATA[7]` | Calibration status byte | `DATA[7] = AlignState` |

---

### 22. Set current position as motor zero (write to ROM, persistent)

Sets the raw encoder value at the motor's current position as the motor's initial zero point after power-up.

> **Caution:**
>
> 1. This command only takes effect after the motor is powered up again.
> 2. This command writes the zero point to the drive's ROM. Repeated writes will affect chip lifespan — frequent use is not recommended.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x19` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply:** The motor replies to the host on receipt; `encoderOffset` in the reply data is the configured zero offset.

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x19` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Encoder zero offset low byte 1 | `DATA[4] = *(uint8_t *)(&encoderOffset)` |
| `DATA[5]` | Encoder zero offset byte 2 | `DATA[5] = *((uint8_t *)(&encoderOffset)+1)` |
| `DATA[6]` | Encoder zero offset byte 3 | `DATA[6] = *((uint8_t *)(&encoderOffset)+2)` |
| `DATA[7]` | Encoder zero offset high byte 4 | `DATA[7] = *((uint8_t *)(&encoderOffset)+3)` |

---

### 23. Read multi-turn angle

The host sends this command to read the motor's current multi-turn absolute angle value.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x92` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply:** The motor replies to the host on receipt. The reply frame contains:

1. `motorAngle` — motor angle, `int64_t`; positive values indicate accumulated clockwise angle, negative values indicate accumulated counterclockwise angle, unit 0.01°/LSB.

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x92` |
| `DATA[1]` | Angle low byte 1 | `DATA[1] = *(uint8_t *)(&motorAngle)` |
| `DATA[2]` | Angle byte 2 | `DATA[2] = *((uint8_t *)(&motorAngle)+1)` |
| `DATA[3]` | Angle byte 3 | `DATA[3] = *((uint8_t *)(&motorAngle)+2)` |
| `DATA[4]` | Angle byte 4 | `DATA[4] = *((uint8_t *)(&motorAngle)+3)` |
| `DATA[5]` | Angle byte 5 | `DATA[5] = *((uint8_t *)(&motorAngle)+4)` |
| `DATA[6]` | Angle byte 6 | `DATA[6] = *((uint8_t *)(&motorAngle)+5)` |
| `DATA[7]` | Angle byte 7 | `DATA[7] = *((uint8_t *)(&motorAngle)+6)` |

**Note:** as written, the source declares `motorAngle` as `int64_t` (8 bytes), but the frame table only carries 7 bytes of it (`DATA[1]`–`DATA[7]`, i.e. bytes 0 through 6 of the value) — the most-significant byte (byte 7 / bits 56–63) is never placed in the frame in this table. This is exactly as printed in the source PDF (confirmed directly against the page image, not only the raw-text extraction). Verify against firmware/observed traffic before assuming the full 64-bit range is available over CAN.

---

### 24. Read single-turn angle

The host sends this command to read the motor's current single-turn angle.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x94` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply:** The motor replies to the host on receipt. The reply frame contains:

1. `circleAngle` — motor single-turn angle, `uint32_t`; the encoder's zero point is the starting point, increasing clockwise, rolling back to 0 upon returning to the zero point again, unit 0.01°/LSB, value range 0 to 36000 × gear-ratio − 1.

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x94` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Single-turn angle low byte 1 | `DATA[4] = *(uint8_t *)(&circleAngle)` |
| `DATA[5]` | Single-turn angle byte 2 | `DATA[5] = *((uint8_t *)(&circleAngle)+1)` |
| `DATA[6]` | Single-turn angle byte 3 | `DATA[6] = *((uint8_t *)(&circleAngle)+2)` |
| `DATA[7]` | Single-turn angle high byte 4 | `DATA[7] = *((uint8_t *)(&circleAngle)+3)` |

---

### 25. Set current position as zero (write to RAM)

The host sends this command to set the motor's current position as the zero point, writing it to RAM. After the command is sent, the motor switches to the `motor stop` state. This zero point is lost after the motor is powered up again.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x95` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply:** The motor replies to the host on receipt; the reply frame data is identical to the host frame.

---

### 26. Read setting parameters

The host sends this command to read a setting parameter. See the **Setting Parameter Table** below for the settings that can be read.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x40` |
| `DATA[1]` | Parameter byte 1 | `DATA[1]` |
| `DATA[2]` | Parameter byte 2 | `DATA[2]` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply:** The reply data contains the read setting parameter value — see the **Setting Parameter Table**.

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x40` |
| `DATA[1]` | Parameter byte 1 | `DATA[1]` |
| `DATA[2]` | Parameter byte 2 | `DATA[2]` |
| `DATA[3]` | Parameter byte 3 | `DATA[3]` |
| `DATA[4]` | Parameter byte 4 | `DATA[4]` |
| `DATA[5]` | Parameter byte 5 | `DATA[5]` |
| `DATA[6]` | Parameter byte 6 | `DATA[6]` |
| `DATA[7]` | Parameter byte 7 | `DATA[7]` |

---

### 27. Write setting parameters

The host sends this command to write a setting parameter. See the **Setting Parameter Table** below for the settings that can be written.

**Notes:**

1. After writing setting parameters, the **Save setting parameters** command must be sent before the data is written into ROM.
2. Multiple setting parameters may be written before sending the **Save setting parameters** command.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x42` |
| `DATA[1]` | Parameter byte 1 | `DATA[1]` |
| `DATA[2]` | Parameter byte 2 | `DATA[2]` |
| `DATA[3]` | Parameter byte 3 | `DATA[3]` |
| `DATA[4]` | Parameter byte 4 | `DATA[4]` |
| `DATA[5]` | Parameter byte 5 | `DATA[5]` |
| `DATA[6]` | Parameter byte 6 | `DATA[6]` |
| `DATA[7]` | Parameter byte 7 | `DATA[7]` |

**Drive reply:** The reply data contains the parameter value after the write — see the **Setting Parameter Table**.

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x42` |
| `DATA[1]` | Parameter byte 1 | `DATA[1]` |
| `DATA[2]` | Parameter byte 2 | `DATA[2]` |
| `DATA[3]` | Parameter byte 3 | `DATA[3]` |
| `DATA[4]` | Parameter byte 4 | `DATA[4]` |
| `DATA[5]` | Parameter byte 5 | `DATA[5]` |
| `DATA[6]` | Parameter byte 6 | `DATA[6]` |
| `DATA[7]` | Parameter byte 7 | `DATA[7]` |

#### Setting Parameter Table

##### One Parameter Command

**Driver ID** — Data Range: 0–32, Data Type: `uint8`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x05` |
| `DATA[2]` | `0x0A` |
| `DATA[3]` | `0x00` |
| `DATA[4]` | Driver ID Bit 7:0 |
| `DATA[5]` | `0x00` |
| `DATA[6]` | `0x00` |
| `DATA[7]` | `0x00` |

**Bus Type** — Data Range: 0–2 (`0`: None, `1`: RS485, `2`: CAN), Data Type: `uint8`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x05` |
| `DATA[2]` | `0x0B` |
| `DATA[3]` | `0x00` |
| `DATA[4]` | Bus Type Bit 7:0 |
| `DATA[5]` | `0x00` |
| `DATA[6]` | `0x00` |
| `DATA[7]` | `0x00` |

**RS485 Baudrate** — Data Range: 0–10, Data Type: `uint8`

| Code | Baud rate |
| --- | --- |
| 0 | 9600 bps |
| 1 | 19200 bps |
| 2 | 38400 bps |
| 3 | 57600 bps |
| 4 | 115200 bps |
| 5 | 230400 bps |
| 6 | 460800 bps |
| 7 | 921600 bps |
| 8 | 1000000 bps |
| 9 | 2000000 bps |
| 10 | 4000000 bps |

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x05` |
| `DATA[2]` | `0x0C` |
| `DATA[3]` | `0x00` |
| `DATA[4]` | RS485 Baudrate Bit 7:0 |
| `DATA[5]` | `0x00` |
| `DATA[6]` | `0x00` |
| `DATA[7]` | `0x00` |

**CAN Baudrate** — Data Range: 0–4, Data Type: `uint8`

| Code | Baud rate |
| --- | --- |
| 0 | 100 kbps |
| 1 | 125 kbps |
| 2 | 250 kbps |
| 3 | 500 kbps |
| 4 | 1 Mbps |

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x05` |
| `DATA[2]` | `0x0D` |
| `DATA[3]` | `0x00` |
| `DATA[4]` | CAN Baudrate Bit 7:0 |
| `DATA[5]` | `0x00` |
| `DATA[6]` | `0x00` |
| `DATA[7]` | `0x00` |

**Max Power** — Data Range: 0–850 (MS series); 0–2000 (MF, MHF, MG series), Data Type: `int16`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x05` |
| `DATA[2]` | `0xE0` |
| `DATA[3]` | `0x00` |
| `DATA[4]` | Max Power Bit 7:0 |
| `DATA[5]` | Max Power Bit 15:8 |
| `DATA[6]` | `0x00` |
| `DATA[7]` | `0x00` |

**Max Speed** — Data Range: 0–600000, Data Type: `int32`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x05` |
| `DATA[2]` | `0xE2` |
| `DATA[3]` | `0x00` |
| `DATA[4]` | Max Speed Bit 7:0 |
| `DATA[5]` | Max Speed Bit 15:8 |
| `DATA[6]` | Max Speed Bit 23:16 |
| `DATA[7]` | Max Speed Bit 31:24 |

**Max Angle** — Data Range: 0–(2³¹ − 1), Data Type: `int32`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x05` |
| `DATA[2]` | `0xE4` |
| `DATA[3]` | `0x00` |
| `DATA[4]` | Max Angle Bit 7:0 |
| `DATA[5]` | Max Angle Bit 15:8 |
| `DATA[6]` | Max Angle Bit 23:16 |
| `DATA[7]` | Max Angle Bit 31:24 |

**Current Ramp** — Data Range: 0–30000, Data Type: `int16`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x05` |
| `DATA[2]` | `0xEA` |
| `DATA[3]` | `0x00` |
| `DATA[4]` | Current Ramp Bit 7:0 |
| `DATA[5]` | Current Ramp Bit 15:8 |
| `DATA[6]` | `0x00` |
| `DATA[7]` | `0x00` |

**Note**: see the discrepancy flagged under the **Motor Control Parameter Table**'s Current Ramp entry above (§19) — this entry is `int16`/2 bytes for the same parameter name and range that the control-parameter version of Current Ramp (sub-ID `0x24`) declares as `int32`/4 bytes.

**Speed Ramp** — Data Range: 0–600000, Data Type: `int32`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0x05` |
| `DATA[2]` | `0xEC` |
| `DATA[3]` | `0x00` |
| `DATA[4]` | Speed Ramp Bit 7:0 |
| `DATA[5]` | Speed Ramp Bit 15:8 |
| `DATA[6]` | Speed Ramp Bit 23:16 |
| `DATA[7]` | Speed Ramp Bit 31:24 |

##### Multiple Parameter Command

**Position Loop PID** — Data Range: 0–2000, Data Type: `uint16`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0xA0` |
| `DATA[2]` | Position Loop Kp Bit 7:0 |
| `DATA[3]` | Position Loop Kp Bit 15:8 |
| `DATA[4]` | Position Loop Ki Bit 7:0 |
| `DATA[5]` | Position Loop Ki Bit 15:8 |
| `DATA[6]` | Position Loop Kd Bit 7:0 |
| `DATA[7]` | Position Loop Kd Bit 15:8 |

**Speed Loop PID** — Data Range: 0–2000, Data Type: `uint16`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0xA4` |
| `DATA[2]` | Speed Loop Kp Bit 7:0 |
| `DATA[3]` | Speed Loop Kp Bit 15:8 |
| `DATA[4]` | Speed Loop Ki Bit 7:0 |
| `DATA[5]` | Speed Loop Ki Bit 15:8 |
| `DATA[6]` | Speed Loop Kd Bit 7:0 |
| `DATA[7]` | Speed Loop Kd Bit 15:8 |

**Current Loop PID** — Data Range: 0–2000, Data Type: `uint16`

| Byte | Value |
| --- | --- |
| `DATA[1]` | `0xA8` |
| `DATA[2]` | Current Loop Kp Bit 7:0 |
| `DATA[3]` | Current Loop Kp Bit 15:8 |
| `DATA[4]` | Current Loop Ki Bit 7:0 |
| `DATA[5]` | Current Loop Ki Bit 15:8 |
| `DATA[6]` | Current Loop Kd Bit 7:0 |
| `DATA[7]` | Current Loop Kd Bit 15:8 |

---

### 28. Save setting parameters

The host sends this command to save the setting parameters to ROM. After saving, the motor must be power-cycled or sent the restart command for the changes to take effect.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x44` |
| `DATA[1]` | Fixed value | `DATA[1] = 0x05` |
| `DATA[2]` | Fixed value | `DATA[2] = 0xFA` |
| `DATA[3]` | NULL | `DATA[3] = 0x00` |
| `DATA[4]` | NULL | `DATA[4] = 0x00` |
| `DATA[5]` | NULL | `DATA[5] = 0x00` |
| `DATA[6]` | NULL | `DATA[6] = 0x00` |
| `DATA[7]` | NULL | `DATA[7] = 0x00` |

**Drive reply:** The reply data contains the save status.

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x44` |
| `DATA[1]` | Fixed value | `DATA[1] = 0x05` |
| `DATA[2]` | Save flag | `DATA[2] = 0x01`: parameter save succeeded<br>`DATA[2] = 0x00`: parameter save failed |
| `DATA[3]` | NULL | `DATA[3] = 0x00` |
| `DATA[4]` | NULL | `DATA[4] = 0x00` |
| `DATA[5]` | NULL | `DATA[5] = 0x00` |
| `DATA[6]` | NULL | `DATA[6] = 0x00` |
| `DATA[7]` | NULL | `DATA[7] = 0x00` |

---

### 29. Motor restart

Restarts the motor, equivalent to power-cycling the motor. The motor does not reply to this command on receipt.

**Command:**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x07` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Drive reply (1 frame):** None.
