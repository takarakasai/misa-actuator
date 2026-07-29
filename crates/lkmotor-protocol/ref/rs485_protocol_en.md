# Shanghai Lingkong Technology — Motor RS485 Bus Communication Protocol

**Version:** V2.36

---

## Summary

This document is a complete, independently-verified translation of the Chinese-original LKMotor (Shanghai Lingkong Technology / 上海瓴控科技) RS485 bus protocol manual (V2.36), transcribed and translated directly from the source PDF (`rs485_protocol.pdf`, 20 pages), cross-checked against the sibling reference translation at `lkmotor-driver/ref/rs485_protocol_desc_en.md`.

**Version / model / serial-number read command:** This protocol document contains **no command anywhere** for reading a firmware/software version, model name or number, hardware version, or serial number. All 24 single-motor commands are enumerated in the Table of Contents below, and none of them expose device identity or firmware-version information — the closest thing to an identifying read is the **Read control parameters** command (`0xC0`), which reads *tunable control parameters* (PID gains, limits, ramps) by numeric ID, not identity/version data. This independently re-confirms, directly against the Chinese source, the finding already reflected in the existing English translation: **LKMotor's RS485 protocol genuinely has no version/model/serial-read command.** (Note: this is a separate protocol from LKMotor's CAN protocol, which is out of scope here — the open question referenced in project memory concerns whether *any* LKMotor protocol variant exposes this, and this document at least confirms RS485 V2.36 does not.)

**Discrepancies found vs. the existing translation** (`lkmotor-driver/ref/rs485_protocol_desc_en.md`): the source PDF has three internal errors where a table's declared byte count doesn't match the number of rows actually listed (commands 18 and 20, detailed in notes at each site below). The existing translation silently "corrected" one of these (command 18's request byte count, quietly changed from the source's "3 byte" to "8 byte" with no flag) but left the other two (command 20's request/reply byte counts) unflagged and unmodified, faithfully matching the source's incorrect header text. This document instead preserves the source's literal (evidently wrong) header text in all three places and flags each with a **Note**, so implementers aren't misled by a silent correction and aren't left to rediscover the mismatch themselves. All other content (command byte values, ranges, units, the two known `+1`-instead-of-`+0`/`+6`-instead-of-`+7` byte-index bugs in commands 17 and 22, and the DATA_SUM range bug in command 24) was independently re-verified against the PDF and matches the existing translation.

---

## Table of Contents

- [Disclaimer](#disclaimer)
- [RS485 Bus Parameters](#rs485-bus-parameters)
- [Single-Motor Commands](#single-motor-commands)
  1. [Read motor status 1 and error flags](#1-read-motor-status-1-and-error-flags)
  2. [Clear motor error flags](#2-clear-motor-error-flags)
  3. [Read motor status 2](#3-read-motor-status-2)
  4. [Read motor status 3](#4-read-motor-status-3)
  5. [Motor off](#5-motor-off)
  6. [Motor run](#6-motor-run)
  7. [Motor stop](#7-motor-stop)
  8. [Brake state control and read](#8-brake-state-control-and-read)
  9. [Open-loop control (MS motors only)](#9-open-loop-control-ms-motors-only)
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
  20. [Read encoder](#20-read-encoder)
  21. [Set current position as motor zero (write to ROM)](#21-set-current-position-as-motor-zero-write-to-rom)
  22. [Read multi-turn angle](#22-read-multi-turn-angle)
  23. [Read single-turn angle](#23-read-single-turn-angle)
  24. [Set current position as zero (write to RAM)](#24-set-current-position-as-zero-write-to-ram)

---

## Disclaimer

Thank you for purchasing the integrated motor-drive control system from Shanghai Lingkong Technology Co., Ltd. Before use, please read this disclaimer carefully. Once you use the product, you are deemed to have acknowledged and accepted the entirety of this disclaimer. Strictly comply with the product manual, control protocol, and relevant laws, regulations, policies, and guidelines when installing and using this product. While using the product, the user undertakes responsibility for their own actions and all consequences arising from them. Lingkong Technology will bear no legal liability for any loss caused by the user's improper use, installation, or modification.

"Lingkong Technology" is a trademark of Shanghai Lingkong Technology Co., Ltd. and its affiliated companies. Product names and brands appearing in this document are the trademarks or registered trademarks of their respective owning companies.

This product and its manual are copyrighted by Shanghai Lingkong Technology Co., Ltd. No part may be reproduced or reprinted in any form without permission. The right of final interpretation of this disclaimer belongs to the company.

---

## RS485 Bus Parameters

- **Bus interface:** RS485
- **Baud rate (normal mode, single-motor commands):**
  - 9600 bps
  - 19200 bps
  - 38400 bps
  - 57600 bps
  - 115200 bps (default)
  - 230400 bps
  - 460800 bps
  - 1 Mbps
  - 2 Mbps
  - 4 Mbps
- **Baud rate (broadcast mode, multi-motor commands):**
  - 1 Mbps
  - 2 Mbps
  - 4 Mbps
- **Data bits:** 8
- **Parity:** None
- **Stop bit:** 1

---

## Single-Motor Commands

Up to 32 drives may be attached to the same bus (subject to bus loading). To prevent bus collisions, each drive must be configured with a distinct ID in the range 1–32.

The host sends a single-motor command frame on the bus; the motor with the matching ID executes the command upon receipt and, a short time later (within 0.25 ms), sends a reply frame with the same ID back to the host. The command frame and reply frame share the same message format: **frame command + frame data (optional)**, described in detail below:

| | Data description | Data length (bytes) | Notes |
| --- | --- | --- | --- |
| **Frame command** | Frame header | 1 | Frame header identifier, `0x3E` |
| | Command | 1 | `CMD` |
| | ID | 1 | 1–32, corresponding motor ID |
| | Data length | 1 | Length of the data appended to the frame command; depends on the specific command |
| | Frame command checksum byte | 1 | `CMD_SUM`, checksum (sum) of all frame-command bytes; keep the low 8 bits, discard the high bits |
| **Frame data** | Data | 0–100 | Data appended to the frame command |
| | Frame data checksum byte | 0 or 1 | `DATA_SUM`, checksum (sum) of all frame-data bytes; keep the low 8 bits, discard the high bits |

---

### 1. Read motor status 1 and error flags

This command reads the motor's current temperature, voltage, and error-status flags.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x9A` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x00` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Drive reply**

The motor replies to the host upon receipt of the command. The reply frame data contains the following parameters:

1. Motor temperature `temperature` (`int8_t` type, unit 1 °C/LSB).
2. Bus voltage `voltage` (`int16_t` type, unit 0.01 V/LSB).
3. Bus current `current` (`int16_t` type, unit 0.01 A/LSB).
4. Motor state `motorState` (`uint8_t` type; each bit represents a different motor state).
5. Error flags `errorState` (`uint8_t` type; each bit represents a different motor error state).

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x9A` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x07` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (8 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Motor temperature | `DATA[0] = *(uint8_t *)(&temperature)` |
| `DATA[1]` | Bus voltage low byte | `DATA[1] = *(uint8_t *)(&voltage)` |
| `DATA[2]` | Bus voltage high byte | `DATA[2] = *((uint8_t *)(&voltage)+1)` |
| `DATA[3]` | Bus current low byte | `DATA[3] = *(uint8_t *)(&current)` |
| `DATA[4]` | Bus current high byte | `DATA[4] = *((uint8_t *)(&current)+1)` |
| `DATA[5]` | Motor state byte | `DATA[5] = motorState` |
| `DATA[6]` | Error state byte | `DATA[6] = errorState` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[6]` |

**Notes:**

1. `motorState = 0x00`: motor is in the on (enabled) state; `motorState = 0x10`: motor is in the off (disabled) state.
2. The individual bits of `errorState` are as follows:

| `errorState` bit | Meaning | 0 | 1 |
| --- | --- | --- | --- |
| 0 | Low-voltage state | Normal | Low-voltage protection |
| 1 | High-voltage state | Normal | High-voltage protection |
| 2 | Drive temperature state | Normal | Drive over-temperature |
| 3 | Motor temperature state | Normal | Motor over-temperature |
| 4 | Motor current state | Normal | Motor over-current |
| 5 | Motor short-circuit state | Normal | Motor short circuit |
| 6 | Stall state | Normal | Motor stalled |
| 7 | Input signal state | Normal | Input signal lost / timed out |

---

### 2. Clear motor error flags

This command clears the motor's current error state; the motor returns a reply upon receipt.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x9B` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x00` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Drive reply**

The motor replies to the host upon receipt. The reply data is identical to that of **Read motor status 1 and error flags** (only the command byte `CMD[1]` differs — here it is `0x9B`).

**Notes:**

1. The error flags cannot be cleared while the motor state has not yet returned to normal.

---

### 3. Read motor status 2

This command reads the motor's current temperature, torque current (MF, MG) / output power (MS), rotational speed, and encoder position.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x9C` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x00` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Drive reply**

The motor replies to the host upon receipt of the command. The reply frame data contains the following parameters:

1. Motor temperature `temperature` (`int8_t` type, 1 °C/LSB).
2. For MF, MG motors: torque-current value `iq`; for MS motors: output-power value `power` — both `int16_t` type. MG motor `iq` resolution is (66/4096 A)/LSB; MF motor `iq` resolution is (33/4096 A)/LSB. MS motor `power` range is −1000 to 1000.
3. Motor speed `speed` (`int16_t` type, 1 dps/LSB).
4. Encoder value `encoder` (`uint16_t` type; a 14-bit encoder's value range is 0–16383, a 15-bit encoder's value range is 0–32767, a 16-bit encoder's value range is 0–65535).

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x9C` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x07` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (8 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Motor temperature | `DATA[0] = *(uint8_t *)(&temperature)` |
| `DATA[1]` | Torque current low byte / Output power low byte (MS series) | `DATA[1] = *(uint8_t *)(&iq)` / `DATA[1] = *(uint8_t *)(&power)` |
| `DATA[2]` | Torque current high byte / Output power high byte (MS series) | `DATA[2] = *((uint8_t *)(&iq)+1)` / `DATA[2] = *((uint8_t *)(&power)+1)` |
| `DATA[3]` | Motor speed low byte | `DATA[3] = *(uint8_t *)(&speed)` |
| `DATA[4]` | Motor speed high byte | `DATA[4] = *((uint8_t *)(&speed)+1)` |
| `DATA[5]` | Encoder position low byte | `DATA[5] = *(uint8_t *)(&encoder)` |
| `DATA[6]` | Encoder position high byte | `DATA[6] = *((uint8_t *)(&encoder)+1)` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[6]` |

---

### 4. Read motor status 3

Because MS motors have no phase-current sampling, this command has no effect on MS motors.

This command reads the motor's current temperature and 3-phase current data.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x9D` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x00` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Drive reply (13 bytes)**

The motor replies to the host upon receipt of the command. The reply frame data contains the following data:

1. Motor temperature `temperature` (`int8_t` type, 1 °C/LSB).
2. Phase-current data `iA`, `iB`, `iC` (`int16_t` type). MG motor phase-current resolution is (66/4096 A)/LSB; MF motor phase-current resolution is (33/4096 A)/LSB.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x9D` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x07` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (8 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[5]` | Motor temperature | `DATA[5] = *(uint8_t *)(&temperature)` |
| `DATA[6]` | Phase-A current low byte | `DATA[6] = *(uint8_t *)(&iA)` |
| `DATA[7]` | Phase-A current high byte | `DATA[7] = *((uint8_t *)(&iA)+1)` |
| `DATA[8]` | Phase-B current low byte | `DATA[8] = *(uint8_t *)(&iB)` |
| `DATA[9]` | Phase-B current high byte | `DATA[9] = *((uint8_t *)(&iB)+1)` |
| `DATA[10]` | Phase-C current low byte | `DATA[10] = *(uint8_t *)(&iC)` |
| `DATA[11]` | Phase-C current high byte | `DATA[11] = *((uint8_t *)(&iC)+1)` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[6]` |

> **Note:** As printed in the source, this table's byte indices start at `DATA[5]` and run to `DATA[11]` (7 data entries), yet the checksum description still reads "checksum of `DATA[0]`–`DATA[6]`" and the reply is stated to be 13 bytes overall (5-byte CMD portion + 8-byte data portion). The indices are almost certainly meant to be `DATA[0]`–`DATA[6]` (temperature + 3×2-byte phase currents = 7 bytes, plus `DATA_SUM` = 8 bytes total, matching `CMD[3] = 0x07`). Reproduced verbatim from the source; treat the byte *offsets* (temperature first, then iA/iB/iC low/high pairs, in that order) as authoritative rather than the literal index numbers.

---

### 5. Motor off

Switches the motor from the on state (the default state after power-up) to the off state; the LED changes from solid-on to slow-blink. While off, the motor still replies to commands but does not execute any motion.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x80` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x00` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Drive reply:** Identical to what the host sent.

---

### 6. Motor run

Switches the motor from the off state to the on state; the LED changes from slow-blink to solid-on. Control commands sent from this point on will drive the motor.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x88` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x00` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Drive reply:** Identical to what the host sent.

---

### 7. Motor stop

Stops the motor, but does not clear the motor's run state. Sending a control command again will resume control of the motor's motion.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x81` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x00` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Drive reply:** Identical to what the host sent.

---

### 8. Brake state control and read

Controls engagement/release of the brake, or reads the current brake state.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x8C` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x01` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |
| `DATA[0]` — Brake control/read byte | `0x00`: brake de-energized, brake engaged<br>`0x01`: brake energized, brake released<br>`0x10`: read brake state |
| `DATA_SUM` — Data checksum byte | Checksum of `DATA[0]` |

**Drive reply**

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x8C` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x01` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |
| `DATA[0]` — Brake state byte | `0x00`: brake is in the de-energized state, brake engaged<br>`0x01`: brake is in the energized state, brake released |
| `DATA_SUM` — Data checksum byte | Checksum of `DATA[0]` |

---

### 9. Open-loop control (this command is implemented only on MS motors)

The host sends this command to control the open-loop voltage output to the motor. The control value `powerControl` is `int16_t` type, value range −850 to 850 (motor current and torque vary by motor).

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0xA0` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x02` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (3 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Open-loop control value low byte | `DATA[0] = *(uint8_t *)(&powerControl)` |
| `DATA[1]` | Open-loop control value high byte | `DATA[1] = *((uint8_t *)(&powerControl)+1)` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`CMD[1]` *(sic — printed this way in the source; almost certainly means `DATA[0]`–`DATA[1]`)* |

**Drive reply**

The motor replies to the host upon receipt of the command. The reply data is identical to that of **Read motor status 2** (only the command byte `CMD[1]` differs — here it is `0xA0`).

---

### 10. Torque closed-loop control (this command is implemented only on MF, MH, MG motors)

The host sends this command to control the motor's torque-current output. The control value `iqControl` is `int16_t` type, value range −2048 to 2048, corresponding to an actual torque-current range of −16.5 A to 16.5 A for MF motors and −33 A to 33 A for MG motors. The bus current and the motor's actual torque vary by motor.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0xA1` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x02` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (3 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Torque-current control value low byte | `DATA[0] = *(uint8_t *)(&iqControl)` |
| `DATA[1]` | Torque-current control value high byte | `DATA[1] = *((uint8_t *)(&iqControl)+1)` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[1]` |

**Drive reply**

The motor replies to the host upon receipt of the command. The reply data is identical to that of **Read motor status 2** (only the command byte `CMD[1]` differs — here it is `0xA1`).

---

### 11. Speed closed-loop control

The host sends this command to control the motor's speed. The control value `speedControl` is `int32_t` type, corresponding to an actual speed of 0.01 dps/LSB. This command has two lengths — the data-length-6 form additionally carries a torque limit.

**Speed control 1:**

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0xA2` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x04` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (5 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Motor speed low byte | `DATA[0] = *(uint8_t *)(&speedControl)` |
| `DATA[1]` | Motor speed | `DATA[1] = *((uint8_t *)(&speedControl)+1)` |
| `DATA[2]` | Motor speed | `DATA[2] = *((uint8_t *)(&speedControl)+2)` |
| `DATA[3]` | Motor speed high byte | `DATA[3] = *((uint8_t *)(&speedControl)+3)` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[3]` |

**Speed control 2 (with torque limit):**

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0xA2` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x06` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (7 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Motor speed low byte | `DATA[0] = *(uint8_t *)(&speedControl)` |
| `DATA[1]` | Motor speed | `DATA[1] = *((uint8_t *)(&speedControl)+1)` |
| `DATA[2]` | Motor speed | `DATA[2] = *((uint8_t *)(&speedControl)+2)` |
| `DATA[3]` | Motor speed high byte | `DATA[3] = *((uint8_t *)(&speedControl)+3)` |
| `DATA[4]` | Torque-current limit low byte | `DATA[4] = *(uint8_t *)(&iqControl)` |
| `DATA[5]` | Torque-current limit high byte | `DATA[5] = *((uint8_t *)(&iqControl)+1)` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[5]` |

**Notes:**

1. Under this command, the motor's `speedControl` is bounded by the **Max Speed** value in the host software (upper computer).
2. In this control mode, the motor's maximum acceleration is bounded by the **Max Acceleration** value in the host software.

**Drive reply**

The motor replies to the host upon receipt of the command. The reply data is identical to that of **Read motor status 2** (only the command byte `CMD[1]` differs — here it is `0xA2`).

---

### 12. Multi-turn position closed-loop control 1

The host sends this command to control the motor's position (multi-turn angle). The control value `angleControl` is `int64_t` type, corresponding to an actual position of 0.01 degree/LSB — i.e. 36000 represents 360°. The motor's direction of rotation is determined by the difference between the target position and the current position.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0xA3` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x08` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (9 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Position control byte 1 (low) | `DATA[0] = *(uint8_t *)(&angleControl)` |
| `DATA[1]` | Position control byte 2 | `DATA[1] = *((uint8_t *)(&angleControl)+1)` |
| `DATA[2]` | Position control byte 3 | `DATA[2] = *((uint8_t *)(&angleControl)+2)` |
| `DATA[3]` | Position control byte 4 | `DATA[3] = *((uint8_t *)(&angleControl)+3)` |
| `DATA[4]` | Position control byte 5 | `DATA[4] = *((uint8_t *)(&angleControl)+4)` |
| `DATA[5]` | Position control byte 6 | `DATA[5] = *((uint8_t *)(&angleControl)+5)` |
| `DATA[6]` | Position control byte 7 | `DATA[6] = *((uint8_t *)(&angleControl)+6)` |
| `DATA[7]` | Position control byte 8 (high) | `DATA[7] = *((uint8_t *)(&angleControl)+7)` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[7]` |

**Notes:**

1. Under this command, the control value `angleControl` is bounded by the **Max Angle** value in the host software.
2. Under this command, the motor's maximum speed is bounded by the **Max Speed** value in the host software.
3. In this control mode, the motor's maximum acceleration is bounded by the **Max Acceleration** value in the host software.
4. In this control mode, MF, MH, MG motors' maximum torque current is bounded by the **Max Torque Current** value in the host software; MS motors' maximum power is bounded by the **Max Power** value in the host software.

**Drive reply**

The motor replies to the host upon receipt of the command. The reply data is identical to that of **Read motor status 2** (only the command byte `CMD[1]` differs — here it is `0xA3`).

---

### 13. Multi-turn position closed-loop control 2

The host sends this command to control the motor's position (multi-turn angle):

1. The control value `angleControl` is `int64_t` type, corresponding to an actual position of 0.01 degree/LSB — i.e. 36000 represents 360°. The motor's direction of rotation is determined by the difference between the target position and the current position.
2. The control value `maxSpeed` limits the motor's maximum rotation speed; it is `uint32_t` type, corresponding to an actual speed of 0.01 dps/LSB — i.e. 36000 represents 360 dps.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0xA4` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x0C` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (13 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Position control byte 1 (low) | `DATA[0] = *(uint8_t *)(&angleControl)` |
| `DATA[1]` | Position control byte 2 | `DATA[1] = *((uint8_t *)(&angleControl)+1)` |
| `DATA[2]` | Position control byte 3 | `DATA[2] = *((uint8_t *)(&angleControl)+2)` |
| `DATA[3]` | Position control byte 4 | `DATA[3] = *((uint8_t *)(&angleControl)+3)` |
| `DATA[4]` | Position control byte 5 | `DATA[4] = *((uint8_t *)(&angleControl)+4)` |
| `DATA[5]` | Position control byte 6 | `DATA[5] = *((uint8_t *)(&angleControl)+5)` |
| `DATA[6]` | Position control byte 7 | `DATA[6] = *((uint8_t *)(&angleControl)+6)` |
| `DATA[7]` | Position control byte 8 (high) | `DATA[7] = *((uint8_t *)(&angleControl)+7)` |
| `DATA[8]` | Speed limit byte 1 (low) | `DATA[8] = *(uint8_t *)(&maxSpeed)` |
| `DATA[9]` | Speed limit byte 2 | `DATA[9] = *((uint8_t *)(&maxSpeed)+1)` |
| `DATA[10]` | Speed limit byte 3 | `DATA[10] = *((uint8_t *)(&maxSpeed)+2)` |
| `DATA[11]` | Speed limit byte 4 (high) | `DATA[11] = *((uint8_t *)(&maxSpeed)+3)` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[11]` |

**Notes:**

1. Under this command, the control value `angleControl` is bounded by the **Max Angle** value in the host software.
2. In this control mode, the motor's maximum acceleration is bounded by the **Max Acceleration** value in the host software.
3. In this control mode, MF, MH, MG motors' maximum torque current is bounded by the **Max Torque Current** value in the host software; MS motors' maximum power is bounded by the **Max Power** value in the host software.

**Drive reply**

The motor replies to the host upon receipt of the command. The reply data is identical to that of **Read motor status 2** (only the command byte `CMD[1]` differs — here it is `0xA4`).

---

### 14. Single-turn position closed-loop control 1

The host sends this command to control the motor's position (single-turn angle):

1. The control value `spinDirection` sets the motor's direction of rotation; it is `uint8_t` type — `0x00` represents clockwise, `0x01` represents counterclockwise.
2. The angle control value `angleControl` is `uint32_t` type but occupies only 3 bytes, unit 0.01 degree/LSB. For direct-drive motors, the value range is 0–35999; for geared motors, the value range is 0–(36000 × gear ratio − 1).

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0xA5` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x04` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (5 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Spin-direction byte | `DATA[0] = spinDirection` |
| `DATA[1]` | Position control byte 1 | `DATA[1] = *(uint8_t *)(&angleControl)` |
| `DATA[2]` | Position control byte 2 | `DATA[2] = *((uint8_t *)(&angleControl)+1)` |
| `DATA[3]` | Position control byte 3 | `DATA[3] = *((uint8_t *)(&angleControl)+2)` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[3]` |

**Notes:**

1. Under this command, the motor's maximum speed is bounded by the **Max Speed** value in the host software.
2. In this control mode, the motor's maximum acceleration is bounded by the **Max Acceleration** value in the host software.
3. In this control mode, MF, MH, MG motors' maximum torque current is bounded by the **Max Torque Current** value in the host software; MS motors' maximum power is bounded by the **Max Power** value in the host software.

**Drive reply**

The motor replies to the host upon receipt of the command. The reply data is identical to that of **Read motor status 2** (only the command byte `CMD[1]` differs — here it is `0xA5`).

---

### 15. Single-turn position closed-loop control 2

The host sends this command to control the motor's position (single-turn angle):

1. The control value `spinDirection` sets the motor's direction of rotation; it is `uint8_t` type — `0x00` represents clockwise, `0x01` represents counterclockwise.
2. The angle control value `angleControl` is `uint32_t` type but occupies only 3 bytes, unit 0.01 degree/LSB. For direct-drive motors, the value range is 0–35999; for geared motors, the value range is 0–(36000 × gear ratio − 1).
3. The speed control value `maxSpeed` limits the motor's maximum rotation speed; it is `uint32_t` type, corresponding to an actual speed of 0.01 dps/LSB — i.e. 36000 represents 360 dps.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0xA6` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x08` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (9 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Spin-direction byte | `DATA[0] = spinDirection` |
| `DATA[1]` | Position control byte 1 | `DATA[1] = *(uint8_t *)(&angleControl)` |
| `DATA[2]` | Position control byte 2 | `DATA[2] = *((uint8_t *)(&angleControl)+1)` |
| `DATA[3]` | Position control byte 3 | `DATA[3] = *((uint8_t *)(&angleControl)+2)` |
| `DATA[4]` | Speed limit byte 1 (low) | `DATA[4] = *(uint8_t *)(&maxSpeed)` |
| `DATA[5]` | Speed limit byte 2 | `DATA[5] = *((uint8_t *)(&maxSpeed)+1)` |
| `DATA[6]` | Speed limit byte 3 | `DATA[6] = *((uint8_t *)(&maxSpeed)+2)` |
| `DATA[7]` | Speed limit byte 4 (high) | `DATA[7] = *((uint8_t *)(&maxSpeed)+3)` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[7]` |

**Notes:**

1. In this control mode, the motor's maximum acceleration is bounded by the **Max Acceleration** value in the host software.
2. In this control mode, MF, MH, MG motors' maximum torque current is bounded by the **Max Torque Current** value in the host software; MS motors' maximum power is bounded by the **Max Power** value in the host software.

**Drive reply**

The motor replies to the host upon receipt of the command. The reply data is identical to that of **Read motor status 2** (only the command byte `CMD[1]` differs — here it is `0xA6`).

---

### 16. Incremental position closed-loop control 1

The host sends this command to control the motor's incremental position. The control value `angleIncrement` is `int32_t` type, corresponding to an actual position of 0.01 degree/LSB — i.e. 36000 represents 360°; the motor's direction of rotation is determined by the sign of this parameter.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0xA7` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x04` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (5 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Incremental-position control low byte 1 | `DATA[0] = *(uint8_t *)(&angleIncrement)` |
| `DATA[1]` | Incremental-position control byte 2 | `DATA[1] = *((uint8_t *)(&angleIncrement)+1)` |
| `DATA[2]` | Incremental-position control byte 3 | `DATA[2] = *((uint8_t *)(&angleIncrement)+2)` |
| `DATA[3]` | Incremental-position control high byte 4 | `DATA[3] = *((uint8_t *)(&angleIncrement)+3)` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[3]` |

**Notes:**

1. Under this command, the motor's maximum speed is bounded by the **Max Speed** value in the host software.
2. In this control mode, the motor's maximum acceleration is bounded by the **Max Acceleration** value in the host software.
3. In this control mode, MF, MH, MG motors' maximum torque current is bounded by the **Max Torque Current** value in the host software; MS motors' maximum power is bounded by the **Max Power** value in the host software.

**Drive reply**

The motor replies to the host upon receipt of the command. The reply data is identical to that of **Read motor status 2** (only the command byte `CMD[1]` differs — here it is `0xA7`).

---

### 17. Incremental position closed-loop control 2

The host sends this command to control the motor's incremental position:

1. The control value `angleIncrement` is `int32_t` type, corresponding to an actual position of 0.01 degree/LSB — i.e. 36000 represents 360°; the motor's direction of rotation is determined by the sign of this parameter.
2. The control value `maxSpeed` limits the motor's maximum rotation speed; it is `uint32_t` type, corresponding to an actual speed of 0.01 dps/LSB — i.e. 36000 represents 360 dps.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0xA8` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x08` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (9 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Incremental-position control low byte 1 | `DATA[0] = *(uint8_t *)(&angleIncrement)` |
| `DATA[1]` | Incremental-position control byte 2 | `DATA[1] = *((uint8_t *)(&angleIncrement)+1)` |
| `DATA[2]` | Incremental-position control byte 3 | `DATA[2] = *((uint8_t *)(&angleIncrement)+2)` |
| `DATA[3]` | Incremental-position control high byte 4 | `DATA[3] = *((uint8_t *)(&angleIncrement)+3)` |
| `DATA[4]` | Speed limit byte 1 | `DATA[4] = *((uint8_t *)(&maxSpeed)+1)` |
| `DATA[5]` | Speed limit byte 2 | `DATA[5] = *((uint8_t *)(&maxSpeed)+2)` |
| `DATA[6]` | Speed limit high byte 3 | `DATA[6] = *((uint8_t *)(&maxSpeed)+3)` |
| `DATA[7]` | Speed limit byte 4 | `DATA[7] = *((uint8_t *)(&maxSpeed)+1)` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[7]` |

> **Note:** The `maxSpeed` byte mapping above is transcribed verbatim from the source, which is internally inconsistent: `DATA[4]`–`DATA[6]` map to `maxSpeed+1`, `+2`, `+3` (never `+0`), and `DATA[7]` repeats `+1` instead of following on with `+4`/wrapping correctly. No byte in the table maps to `*(uint8_t *)(&maxSpeed)` (offset `+0`, the true low byte). This looks like a copy/paste error in the original manual (most likely each `+N` was meant to be one less, i.e. `DATA[4]=+0, DATA[5]=+1, DATA[6]=+2, DATA[7]=+3`, mirroring commands 13/15's `maxSpeed` layout). Verify against firmware/CAN-protocol sibling before relying on this exact byte order.

**Notes:**

1. In this control mode, the motor's maximum acceleration is bounded by the **Max Acceleration** value in the host software.
2. In this control mode, MF, MH, MG motors' maximum torque current is bounded by the **Max Torque Current** value in the host software; MS motors' maximum power is bounded by the **Max Power** value in the host software.

**Drive reply**

The motor replies to the host upon receipt of the command. The reply data is identical to that of **Read motor status 2** (only the command byte `CMD[1]` differs — here it is `0xA8`).

---

### 18. Read control parameters

The host sends this command to read the motor's current control parameters; the parameter read is determined by the index `controlParamID` — see the [Motor Control Parameter Table](#motor-control-parameter-table).

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0xC0` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x07` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (3 bytes incl. checksum):**

| Byte | Description | Value |
| --- | --- | --- |
| `DATA[0]` | Parameter index | `DATA[0] = controlParamID` |
| `DATA[1]` | Parameter byte 1 | `DATA[1] = 0x00` |
| `DATA[2]` | Parameter byte 2 | `DATA[2] = 0x00` |
| `DATA[3]` | Parameter byte 3 | `DATA[3] = 0x00` |
| `DATA[4]` | Parameter byte 4 | `DATA[4] = 0x00` |
| `DATA[5]` | Parameter byte 5 | `DATA[5] = 0x00` |
| `DATA[6]` | Parameter byte 6 | `DATA[6] = 0x00` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[6]` |

> **Note:** The source manual labels this table "帧数据（3byte，含校验）" ("frame data, 3 bytes incl. checksum"), but the table itself lists `DATA[0]`–`DATA[6]` plus `DATA_SUM` — 8 bytes, consistent with `CMD[3] = 0x07` (7 data bytes) + 1 checksum byte = 8. The "3 byte" header is almost certainly a typo in the original document for "8 byte"; the byte-by-byte content (`DATA[0]`–`DATA[6]`, all zero-filled placeholders except `DATA[0]` which carries the parameter index) is unambiguous and reproduced faithfully above. The existing sibling translation (`lkmotor-driver/ref/rs485_protocol_desc_en.md`) silently prints "8 bytes" here with no note — i.e. it quietly corrects this typo rather than flagging it.

**Drive reply**

The reply data from the drive contains the read parameter value; see the [Motor Control Parameter Table](#motor-control-parameter-table) for specifics.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0xC0` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x07` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (8 bytes incl. checksum):**

| Byte | Description |
| --- | --- |
| `DATA[0]` | Parameter index (`DATA[0] = controlParamID`) |
| `DATA[1]` | Parameter byte 1 |
| `DATA[2]` | Parameter byte 2 |
| `DATA[3]` | Parameter byte 3 — Control Parameter |
| `DATA[4]` | Parameter byte 4 — Control Parameter |
| `DATA[5]` | Parameter byte 5 |
| `DATA[6]` | Parameter byte 6 |
| `DATA_SUM` | Data checksum byte — Checksum of `DATA[0]`–`DATA[6]` |

---

### 19. Write control parameters

The host sends this command to write a control parameter into RAM; it takes effect immediately but is lost on power-down. The parameter written and its index `controlParamID` are given in the [Motor Control Parameter Table](#motor-control-parameter-table).

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0xC1` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x07` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (8 bytes incl. checksum):**

| Byte | Description |
| --- | --- |
| `DATA[0]` | Parameter index (`DATA[0] = controlParamID`) |
| `DATA[1]` | Parameter byte 1 |
| `DATA[2]` | Parameter byte 2 |
| `DATA[3]` | Parameter byte 3 — Control Parameter |
| `DATA[4]` | Parameter byte 4 — Control Parameter |
| `DATA[5]` | Parameter byte 5 |
| `DATA[6]` | Parameter byte 6 |
| `DATA_SUM` | Data checksum byte — Checksum of `DATA[0]`–`DATA[6]` |

**Drive reply**

The reply data from the drive contains the parameter value after the write; see the [Motor Control Parameter Table](#motor-control-parameter-table) for specifics.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0xC1` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x07` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (8 bytes incl. checksum):**

| Byte | Description |
| --- | --- |
| `DATA[0]` | Parameter index (`DATA[0] = controlParamID`) |
| `DATA[1]` | Parameter byte 1 |
| `DATA[2]` | Parameter byte 2 |
| `DATA[3]` | Parameter byte 3 — Control Parameter |
| `DATA[4]` | Parameter byte 4 — Control Parameter |
| `DATA[5]` | Parameter byte 5 |
| `DATA[6]` | Parameter byte 6 |
| `DATA_SUM` | Data checksum byte — Checksum of `DATA[0]`–`DATA[6]` |

#### Motor Control Parameter Table

**Position-loop PID** (`0x0A`) — Data type: `uint16`; Data range: 0–2000

| Byte | Value |
| --- | --- |
| `DATA[0]` | `0x0A` |
| `DATA[1]` | Position Loop Kp, bit 7:0 |
| `DATA[2]` | Position Loop Kp, bit 15:8 |
| `DATA[3]` | Position Loop Ki, bit 7:0 |
| `DATA[4]` | Position Loop Ki, bit 15:8 |
| `DATA[5]` | Position Loop Kd, bit 7:0 |
| `DATA[6]` | Position Loop Kd, bit 15:8 |

**Speed-loop PID** (`0x0B`) — Data type: `uint16`; Data range: 0–2000

| Byte | Value |
| --- | --- |
| `DATA[0]` | `0x0B` |
| `DATA[1]` | Speed Loop Kp, bit 7:0 |
| `DATA[2]` | Speed Loop Kp, bit 15:8 |
| `DATA[3]` | Speed Loop Ki, bit 7:0 |
| `DATA[4]` | Speed Loop Ki, bit 15:8 |
| `DATA[5]` | Speed Loop Kd, bit 7:0 |
| `DATA[6]` | Speed Loop Kd, bit 15:8 |

**Current-loop PID** (`0x0C`) — Data type: `uint16`; Data range: 0–2000

| Byte | Value |
| --- | --- |
| `DATA[0]` | `0x0C` |
| `DATA[1]` | Current Loop Kp, bit 7:0 |
| `DATA[2]` | Current Loop Kp, bit 15:8 |
| `DATA[3]` | Current Loop Ki, bit 7:0 |
| `DATA[4]` | Current Loop Ki, bit 15:8 |
| `DATA[5]` | Current Loop Kd, bit 7:0 |
| `DATA[6]` | Current Loop Kd, bit 15:8 |

**Torque-current limit** (`0x1E`) — Data type: `int16`; Data range: 0–850 (MS motors); 0–2000 (MF, MHF, MG motors)

| Byte | Value |
| --- | --- |
| `DATA[0]` | `0x1E` |
| `DATA[1]` | `0x00` |
| `DATA[2]` | `0x00` |
| `DATA[3]` | Torque Limit, bit 7:0 |
| `DATA[4]` | Torque Limit, bit 15:8 |
| `DATA[5]` | `0x00` |
| `DATA[6]` | `0x00` |

> **Note:** the source writes "MHF" here for the third motor family, whereas every other command in this document (e.g. commands 10, 12–17) refers to the same family as "MH" (as in "MF、MH、MG"). Reproduced verbatim; likely the same family, with "MHF" being either a typo or an alternate/older model-family label.

**Speed limit** (`0x20`) — Data type: `int32`; Data range: 0–600000; Unit: 0.01 dps

| Byte | Value |
| --- | --- |
| `DATA[0]` | `0x20` |
| `DATA[1]` | `0x00` |
| `DATA[2]` | `0x00` |
| `DATA[3]` | Speed Limit, bit 7:0 |
| `DATA[4]` | Speed Limit, bit 15:8 |
| `DATA[5]` | Speed Limit, bit 23:16 |
| `DATA[6]` | Speed Limit, bit 31:24 |

**Angle limit** (`0x22`) — Data type: `int32`; Data range: 0–(2³¹ − 1); Unit: 0.01 deg

| Byte | Value |
| --- | --- |
| `DATA[0]` | `0x22` |
| `DATA[1]` | `0x00` |
| `DATA[2]` | `0x00` |
| `DATA[3]` | Angle Limit, bit 7:0 |
| `DATA[4]` | Angle Limit, bit 15:8 |
| `DATA[5]` | Angle Limit, bit 23:16 |
| `DATA[6]` | Angle Limit, bit 31:24 |

**Current ramp** (`0x24`) — Data type: `int32`; Data range: 0–30000

| Byte | Value |
| --- | --- |
| `DATA[0]` | `0x24` |
| `DATA[1]` | `0x00` |
| `DATA[2]` | `0x00` |
| `DATA[3]` | Current Ramp, bit 7:0 |
| `DATA[4]` | Current Ramp, bit 15:8 |
| `DATA[5]` | Current Ramp, bit 23:16 |
| `DATA[6]` | Current Ramp, bit 31:24 |

**Speed ramp** (`0x26`) — Data type: `int32`; Data range: 0–600000; Unit: 1 dps/s

| Byte | Value |
| --- | --- |
| `DATA[0]` | `0x26` |
| `DATA[1]` | `0x00` |
| `DATA[2]` | `0x00` |
| `DATA[3]` | Speed Ramp, bit 7:0 |
| `DATA[4]` | Speed Ramp, bit 15:8 |
| `DATA[5]` | Speed Ramp, bit 23:16 |
| `DATA[6]` | Speed Ramp, bit 31:24 |

---

### 20. Read encoder

The host sends this command to read the encoder's current position.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x90` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x07` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (5 bytes incl. checksum):**

| Byte | Description |
| --- | --- |
| `DATA[0]` | Encoder data type — `0`: read encoder data; `1`: read raw encoder data |
| `DATA[1]` | `NULL`, `0x00` |
| `DATA[2]` | `NULL`, `0x00` |
| `DATA[3]` | `NULL`, `0x00` |
| `DATA[4]` | `NULL`, `0x00` |
| `DATA[5]` | `NULL`, `0x00` |
| `DATA[6]` | `NULL`, `0x00` |
| `DATA_SUM` | Data checksum byte — Checksum of `DATA[0]`–`DATA[6]` |

> **Note:** As in command 18, the source manual's header here reads "帧数据（5byte，含校验）" ("frame data, 5 bytes incl. checksum"), but the table lists `DATA[0]`–`DATA[6]` plus `DATA_SUM` (8 bytes total), consistent with `CMD[3] = 0x07` + 1 checksum byte. This is almost certainly the same class of typo as in command 18 (likely meant "8 byte"). The sibling translation reproduces the same "5 bytes" header text without flagging the mismatch; this document flags it explicitly.

**Drive reply**

The motor replies to the host upon receipt of the command; the reply data contains the following parameters:

1. Encoder data type — `DATA[0] = 0`: encoder data (zero-offset removed); `DATA[0] = 1`: raw encoder data (zero-offset not removed).
2. Encoder data or raw encoder data, `uint32` type.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x90` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x07` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (7 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Encoder data type | `0`: returns encoder data; `1`: returns raw encoder data |
| `DATA[1]` | `NULL` | `0x00` |
| `DATA[2]` | `NULL` | `0x00` |
| `DATA[3]` | Encoder byte 1 | `encoder` (bit 7:0) |
| `DATA[4]` | Encoder byte 2 | `encoder` (bit 15:8) |
| `DATA[5]` | Encoder byte 3 | `encoder` (bit 23:16) |
| `DATA[6]` | Encoder byte 4 | `encoder` (bit 31:24) |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[6]` |

> **Note:** Again the source header ("帧数据（7byte，含校验）") undercounts by one relative to the 8-row table (`DATA[0]`–`DATA[6]` + `DATA_SUM`), for the same reason as above — the true total, consistent with `CMD[3] = 0x07` + 1 checksum byte, is 8 bytes. Flagged here (not silently corrected) since the sibling translation reproduces the same "7 bytes" text without comment.

**Notes:**

1. A 14-bit-resolution encoder's value range is 0–16383; a 15-bit-resolution encoder's value range is 0–32767; an 18-bit-resolution encoder's value range is 0–262143.

---

### 21. Set current position as motor zero (write to ROM)

Sets the encoder's raw value at the motor's current position as the motor's initial zero point after power-up.

**Caution:**

1. This command writes the zero point to the drive's FLASH; repeated writes will affect chip lifespan — frequent use is not recommended.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x19` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x00` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Drive reply**

The motor replies to the host upon receipt of the command; the reply data contains the following parameter:

1. The raw encoder value at the current position, `encoderZero`.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x19` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x02` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (3 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Zero-point raw encoder value, low byte | `DATA[0] = *(uint8_t *)(&encoderZero)` |
| `DATA[1]` | Zero-point raw encoder value, high byte | `DATA[1] = *((uint8_t *)(&encoderZero)+1)` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[1]` |

---

### 22. Read multi-turn angle

The host sends this command to read the motor's current multi-turn absolute angle value.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x92` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x00` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Drive reply**

The motor replies to the host upon receipt of the command; the reply frame data contains the following parameter:

1. Motor angle `motorAngle`, `int64_t` type data. A positive value represents cumulative clockwise angle; a negative value represents cumulative counterclockwise angle. Unit 0.01°/LSB.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x92` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x08` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (9 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Angle low byte 1 | `DATA[0] = *(uint8_t *)(&motorAngle)` |
| `DATA[1]` | Angle byte 2 | `DATA[1] = *((uint8_t *)(&motorAngle)+1)` |
| `DATA[2]` | Angle byte 3 | `DATA[2] = *((uint8_t *)(&motorAngle)+2)` |
| `DATA[3]` | Angle byte 4 | `DATA[3] = *((uint8_t *)(&motorAngle)+3)` |
| `DATA[4]` | Angle byte 5 | `DATA[4] = *((uint8_t *)(&motorAngle)+4)` |
| `DATA[5]` | Angle byte 6 | `DATA[5] = *((uint8_t *)(&motorAngle)+5)` |
| `DATA[6]` | Angle byte 7 | `DATA[6] = *((uint8_t *)(&motorAngle)+6)` |
| `DATA[7]` | Angle high byte 8 | `DATA[7] = *((uint8_t *)(&motorAngle)+6)` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[7]` |

> **Note:** `DATA[7]`'s mapping is printed as `*((uint8_t *)(&motorAngle)+6)` in the source — identical to `DATA[6]`'s mapping, and one byte-offset short of covering the full 8-byte `int64_t`. This is almost certainly meant to be `+7`. Reproduced verbatim; this same typo is also present in the sibling translation, so it is confirmed to originate in the source manual rather than being a translation error.

---

### 23. Read single-turn angle

The host sends this command to read the motor's current single-turn absolute angle value.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x94` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x00` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Drive reply**

The motor replies to the host upon receipt of the command; the reply frame data contains the following parameter:

1. Motor single-turn angle `circleAngle`, `uint32_t` type data. Starting from the encoder's zero point, it increases clockwise and rolls back to 0 upon reaching the zero point again. Unit 0.01°/LSB, value range 0–(36000−1).

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x94` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x04` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (5 bytes incl. checksum):**

| Byte | Description | Mapping |
| --- | --- | --- |
| `DATA[0]` | Single-turn angle low byte 1 | `DATA[0] = *(uint8_t *)(&circleAngle)` |
| `DATA[1]` | Single-turn angle byte 2 | `DATA[1] = *((uint8_t *)(&circleAngle)+1)` |
| `DATA[2]` | Single-turn angle byte 3 | `DATA[2] = *((uint8_t *)(&circleAngle)+2)` |
| `DATA[3]` | Single-turn angle high byte 4 | `DATA[3] = *((uint8_t *)(&circleAngle)+3)` |
| `DATA_SUM` | Data checksum byte | Checksum of `DATA[0]`–`DATA[3]` |

---

### 24. Set current position as zero (write to RAM)

The host sends this command to set the motor's current position as the zero point of the multi-turn position. The zero point is written to RAM only — it is temporarily effective and is lost after the next power-up.

**Frame command (5 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `CMD[0]` — Frame header | `0x3E` |
| `CMD[1]` — Command | `0x95` |
| `CMD[2]` — ID | `0x01`–`0x20` |
| `CMD[3]` — Data length | `0x07` |
| `CMD_SUM` — Frame command checksum byte | Checksum of `CMD[0]`–`CMD[3]` |

**Frame data (8 bytes incl. checksum):**

| Byte | Value |
| --- | --- |
| `DATA[0]` | `0x00` |
| `DATA[1]` | `0x00` |
| `DATA[2]` | `0x00` |
| `DATA[3]` | `0x00` |
| `DATA[4]` | `0x00` |
| `DATA[5]` | `0x00` |
| `DATA[6]` | `0x00` |
| `DATA_SUM` — Data checksum byte | Checksum of `DATA[0]`–`DATA[3]` |

> **Note:** The source manual states the checksum covers only `DATA[0]`–`DATA[3]`, even though the frame carries 7 zero-filled data bytes (`DATA[0]`–`DATA[6]`) per `CMD[3] = 0x07`. This is almost certainly meant to read `DATA[0]`–`DATA[6]`, consistent with every other command's `DATA_SUM` description in this document. Reproduced verbatim from the source. The sibling translation independently flags this identical discrepancy, corroborating that it originates in the source manual.

**Drive reply (8 bytes):** Identical to what the host sent.
