# MyActuator Servo Motor Control Protocol

**Applicable driver:** V3
**Version:** V4.4
**Date:** 2025.09

Wire protocol reference for the RMD-X series motors (CAN bus and RS485 bus command sets). This is
the newest consolidated protocol manual as of 2026-07-26, superseding both
[`RMD-X-Servo-Motor-Control-Protocol-V3.91.pdf`](RMD-X-Servo-Motor-Control-Protocol-V3.91.pdf) and
[`X4-36_Motor_Motion_Protocol.pdf`](X4-36_Motor_Motion_Protocol.pdf) (V4.3) — see
`x4-36-v4.3-analysis.md` and `doc/data-map.md` for the differences already found between those
older manuals and real hardware behavior, several of which are now explained by changes documented
here (see the erratum notes throughout, especially §2.12 and §8).

Converted from the official bilingual (Chinese/English) PDF `CAN BUS Motor Motion Protocol V4.4
260425.pdf`; only the English text was kept. Every worked communication example is reproduced with
exact byte values. Where the source manual itself contains an internal inconsistency (a translation
typo, a stale cross-reference, a table/prose mismatch), it is preserved verbatim with an inline
**Erratum** note rather than silently "corrected" — several were found during this conversion (see
§2.12, §2.24–2.25, §2.31–2.36, §4, §8).

---

## Table of Contents

- [Disclaimer](#disclaimer)
- [1. Communication Bus Parameters and Message Format](#1-communication-bus-parameters-and-message-format)
  - [1.1. CAN Bus](#11-can-bus)
  - [1.2. RS485 Bus](#12-rs485-bus)
- [2. Single Motor Command Description](#2-single-motor-command-description)
  - [2.1. Read PID Parameter Command (0x30)](#21-read-pid-parameter-command-0x30)
  - [2.2. Write PID Parameters to RAM Command (0x31)](#22-write-pid-parameters-to-ram-command-0x31)
  - [2.3. Write PID Parameters to ROM Command (0x32)](#23-write-pid-parameters-to-rom-command-0x32)
  - [2.4. Read Acceleration Command (0x42)](#24-read-acceleration-command-0x42)
  - [2.5. Write Acceleration to RAM and ROM Command (0x43)](#25-write-acceleration-to-ram-and-rom-command-0x43)
  - [2.6. Read Multi-Turn Encoder Position Data Command (0x60)](#26-read-multi-turn-encoder-position-data-command-0x60)
  - [2.7. Read Multi-Turn Encoder Original Position Data Command (0x61)](#27-read-multi-turn-encoder-original-position-data-command-0x61)
  - [2.8. Read Multi-Turn Encoder Zero Offset Data Command (0x62)](#28-read-multi-turn-encoder-zero-offset-data-command-0x62)
  - [2.9. Write Encoder Multi-Turn Value to ROM as Motor Zero Command (0x63)](#29-write-encoder-multi-turn-value-to-rom-as-motor-zero-command-0x63)
  - [2.10. Write the Current Multi-Turn Position of the Encoder to the ROM as the Motor Zero Command (0x64)](#210-write-the-current-multi-turn-position-of-the-encoder-to-the-rom-as-the-motor-zero-command-0x64)
  - [2.11. Read Multi-Turn Angle Command (0x92)](#211-read-multi-turn-angle-command-0x92)
  - [2.12. Read Single-Turn Angle Command (0x94)](#212-read-single-turn-angle-command-0x94)
  - [2.13. Read Motor Status 1 and Error Flag Command (0x9A)](#213-read-motor-status-1-and-error-flag-command-0x9a)
  - [2.14. Read Motor Status 2 Command (0x9C)](#214-read-motor-status-2-command-0x9c)
  - [2.15. Read Motor Status 3 Command (0x9D)](#215-read-motor-status-3-command-0x9d)
  - [2.16. Motor Shutdown Command (0x80)](#216-motor-shutdown-command-0x80)
  - [2.17. Motor Stop Command (0x81)](#217-motor-stop-command-0x81)
  - [2.18. Torque Closed-Loop Control Command (0xA1)](#218-torque-closed-loop-control-command-0xa1)
  - [2.19. Speed Closed-Loop Control Command (0xA2)](#219-speed-closed-loop-control-command-0xa2)
  - [2.20. Absolute Position Closed-Loop Control Command (0xA4)](#220-absolute-position-closed-loop-control-command-0xa4)
  - [2.21. Single-Turn Position Control Command (0xA6)](#221-single-turn-position-control-command-0xa6)
  - [2.22. Incremental Position Closed-Loop Control Command (0xA8)](#222-incremental-position-closed-loop-control-command-0xa8)
  - [2.23. Force Control Position Closed-Loop Command (0xA9)](#223-force-control-position-closed-loop-command-0xa9)
  - [2.24. SF Command (0x72, Position Control with Speed Feedforward)](#224-sf-command-0x72-position-control-with-speed-feedforward)
  - [2.25. TF Command (0x73, Position Control with Torque Feedforward)](#225-tf-command-0x73-position-control-with-torque-feedforward)
  - [2.26. System Operating Mode Acquisition (0x70)](#226-system-operating-mode-acquisition-0x70)
  - [2.27. System Reset Command (0x76)](#227-system-reset-command-0x76)
  - [2.28. System Brake Release Command (0x77)](#228-system-brake-release-command-0x77)
  - [2.29. System Brake Lock Command (0x78)](#229-system-brake-lock-command-0x78)
  - [2.30. System Runtime Read Command (0xB1)](#230-system-runtime-read-command-0xb1)
  - [2.31. System Software Version Date Read Command (0xB2)](#231-system-software-version-date-read-command-0xb2)
  - [2.32. Communication Interruption Protection Time Setting Command (0xB3)](#232-communication-interruption-protection-time-setting-command-0xb3)
  - [2.33. Communication Baud Rate Setting Command (0xB4)](#233-communication-baud-rate-setting-command-0xb4)
  - [2.34. Motor Model Reading Command (0xB5)](#234-motor-model-reading-command-0xb5)
  - [2.35. Active Reply Function Command (0xB6)](#235-active-reply-function-command-0xb6)
  - [2.36. Function Control Command (0x20)](#236-function-control-command-0x20)
- [3. CAN Multi-Motor Command (0x280 + Command)](#3-can-multi-motor-command-0x280--command)
- [4. Motion Mode Control Command_CAN (0x400 + ID)](#4-motion-mode-control-command_can-0x400--id)
- [5. RS485 Multi-Motor Command (0xCD + Command)](#5-rs485-multi-motor-command-0xcd--command)
- [6. RS485 Motion Mode Control Command](#6-rs485-motion-mode-control-command)
- [7. Indicator Light Description](#7-indicator-light-description)
- [8. Version Revision Information](#8-version-revision-information)

---

## Disclaimer

Thanks for choosing MYACTUATOR. Please read this statement carefully before using. Once used, this statement is deemed to be approved and accepted. Please install and use this product strictly in accordance with the manual, product description, and relevant laws, regulations, policies, and guidelines. In the process of using the product, the user undertakes to be responsible for their own behavior and all consequences arising therefrom. MYACTUATOR will not be liable for any loss caused by improper use, installation, or modification by the user.

"MYACTUATOR" is the trademark of Suzhou Micro Actuator Technology Co., Ltd. and its affiliates. Product names, brands, etc. appearing in this document are trademarks or registered trademarks of their respective companies.

This product and manual are copyrighted by MYACTUATOR. Reproduction in any form is not permitted without permission. The final interpretation right of the disclaimer belongs to MYACTUATOR.

---

## 1. Communication Bus Parameters and Message Format

### 1.1. CAN Bus

#### 1.1.1. Parameters

- **Bus interface:** CAN
- **Baud rate:** 1 Mbps

#### 1.1.2. Message Format

- **Identifier:**
  - Single-motor command send: `0x140 + ID` (ID = 1–32)
  - Multi-motor command send: `0x280`
  - Reply: `0x240 + ID` (ID = 1–32)
- **Frame format:** Data frame
- **Frame type:** Standard frame
- **DLC:** 8 bytes

### 1.2. RS485 Bus

#### 1.2.1. Parameters

- **Bus interface:** RS485
- **Baud rate:** 115200 bps, 500 Kbps, 1 Mbps, 1.5 Mbps, 2.5 Mbps
- **Serial port configuration:** 8 data bits, 1 stop bit, no parity bit

#### 1.2.2. Message Format

| Type | Data Definition | Bytes | Description |
| --- | --- | --- | --- |
| Frame header | `0x3E` | 1 | Communication frame header, used for identification. |
| ID | 1–32 | 1 | Device address, corresponding to the ID number of each motor. |
| Data length | Data length | 1 | The length of the data field. In the standard protocol, the length is fixed at 8 bytes. |
| Data field | Data content | According to the length | The content of the data field in the standard protocol is exactly the same as that of the CAN. |
| Check | CRC check | 2 | CRC16 check, low byte first, high byte last. |

## 2. Single Motor Command Description

The host addresses a single motor with `0x140 + ID` on CAN (`ID` = 1–32) and receives its reply on `0x240 + ID`. On RS485, the same 8-byte data-field content is wrapped in a `0x3E` frame with the device's ID byte, a length byte, and a trailing CRC16. Each command below is described once; the CAN and RS485 encodings of `DATA[0]`–`DATA[7]` are identical.

### 2.1. Read PID Parameter Command (0x30)

#### 2.1.1. Instruction Description

This command can read the PID parameters of the current, speed, and position loops. The data type is `Float`, determined by the index value — see the index description table in 2.1.4.

#### 2.1.2. Send Data Field Definition

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x30` |
| `DATA[1]` | Parameter index | `DATA[1] = (uint8_t)index` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

#### 2.1.3. Reply Data Field Definition

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x30` |
| `DATA[1]` | Function index | `DATA[1] = (uint8_t)index` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Parameter low byte 1 | `DATA[4] = (uint8_t)(Value)` |
| `DATA[5]` | Parameter byte 2 | `DATA[5] = (uint8_t)(Value>>8)` |
| `DATA[6]` | Parameter byte 3 | `DATA[6] = (uint8_t)(Value>>16)` |
| `DATA[7]` | Parameter byte 4 | `DATA[7] = (uint8_t)(Value>>24)` |

#### 2.1.4. Function Index Description

| Index | Parameter |
| --- | --- |
| `0x01` | Current loop KP parameter |
| `0x02` | Current loop KI parameter |
| `0x04` | Speed loop KP parameter |
| `0x05` | Speed loop KI parameter |
| `0x07` | Position loop KP parameter |
| `0x08` | Position loop KI parameter |
| `0x09` | Position loop KD parameter |

#### 2.1.5. Communication Example

**Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x30` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x30` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[1] = 0x01`; according to the index value table, this means the current loop KP, indicating a read of the current loop KP parameter.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x30` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0x80` | `0x3F` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x30` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0x80` | `0x3F` | CRC16L | CRC16H |

Description: In the returned frame data, `Data[1] = 0x01`, meaning the current loop KP. `Data[4]` through `Data[7]` form a 32-bit value (`Data[4]` is the lowest byte, `Data[7]` is the highest byte) of `0x3F800000`. The data type is `Float`; converted to decimal this is `1.0`, meaning the motor's current loop KP parameter is currently `1.0`.

An online conversion tool can be used, e.g. IEEE-754 hex/float converters such as the one at speedfly.cn.

### 2.2. Write PID Parameters to RAM Command (0x31)

#### 2.2.1. Instruction Description

This command writes the current, speed, and position loop KP/KI parameters to RAM in a single operation; the values are not retained after power-off. The data type is `Float`, determined by the index value — see the index description table in 2.2.4. Be careful to avoid writing parameters while the motor has just started or is in motion.

#### 2.2.2. Send Data Field Definition

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x31` |
| `DATA[1]` | Function index | `DATA[1] = (uint8_t)index` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Parameter low byte 1 | `DATA[4] = (uint8_t)(Value)` |
| `DATA[5]` | Parameter byte 2 | `DATA[5] = (uint8_t)(Value>>8)` |
| `DATA[6]` | Parameter byte 3 | `DATA[6] = (uint8_t)(Value>>16)` |
| `DATA[7]` | Parameter byte 4 | `DATA[7] = (uint8_t)(Value>>24)` |

#### 2.2.3. Reply Data Field Definition

The content of the reply data is the same as the sent data.

#### 2.2.4. Function Index Description

| Index | Parameter |
| --- | --- |
| `0x01` | Current loop KP parameter |
| `0x02` | Current loop KI parameter |
| `0x04` | Speed loop KP parameter |
| `0x05` | Speed loop KI parameter |
| `0x07` | Position loop KP parameter |
| `0x08` | Position loop KI parameter |
| `0x09` | Position loop KD parameter |

#### 2.2.5. Communication Example

**Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x31` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0xC0` | `0x3F` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x31` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0xC0` | `0x3F` | CRC16L | CRC16H |

Description: `Data[1] = 0x01`; according to the index value table, this parameter is the current loop KP. `Data[4]` through `Data[7]` form a 32-bit value (`Data[4]` is the lowest byte, `Data[7]` is the highest byte) of `0x3FC00000`. The data type is `Float`; converted to decimal this is `1.5` (an online IEEE-754 hex/float conversion tool, such as the one at speedfly.cn, can be used). This means the motor's current loop KP parameter is set to `1.5` and written to the motor drive's RAM; the parameter is not retained after power-off.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x31` | `0x01`* | `0x00` | `0x00` | `0x00` | `0x00` | `0xC0` | `0x3F` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x31` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0xC0` | `0x3F` | CRC16L | CRC16H |

*Note: the source manual's CAN reply row for this example omits a `Data[1]` column header/value (its table only lists `Data[0]`, `Data[2]`–`Data[7]`), unlike the RS485 row which includes `D1 = 0x01`. Reproduced here with `Data[1] = 0x01` filled in for consistency with the RS485 encoding and with 2.1's reply, since the command replies with the same content as the send.

### 2.3. Write PID Parameters to ROM Command (0x32)

#### 2.3.1. Instruction Description

This command writes the current, speed, and position loop KP/KI parameters to ROM in a single operation; the values are retained after power-off. The data type is `Float`, determined by the index value — see the index description table in 2.2.4 (the manual cross-references the 2.2.4 table rather than repeating a separate 2.3.4 numbering internally, though a 2.3.4 table is also given below). Be careful to avoid writing parameters while the motor has just started or is in motion. Parameters can only be successfully saved to ROM while the motor is disabled.

#### 2.3.2. Send Data Field Definition

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x32` |
| `DATA[1]` | Function index | `DATA[1] = (uint8_t)index` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Parameter low byte 1 | `DATA[4] = (uint8_t)(Value)` |
| `DATA[5]` | Parameter byte 2 | `DATA[5] = (uint8_t)(Value>>8)` |
| `DATA[6]` | Parameter byte 3 | `DATA[6] = (uint8_t)(Value>>16)` |
| `DATA[7]` | Parameter byte 4 | `DATA[7] = (uint8_t)(Value>>24)` |

#### 2.3.3. Reply Data Field Definition

The content of the reply data is the same as the sent data.

#### 2.3.4. Function Index Description

| Index | Parameter |
| --- | --- |
| `0x01` | Current loop KP parameter |
| `0x02` | Current loop KI parameter |
| `0x04` | Speed loop KP parameter |
| `0x05` | Speed loop KI parameter |
| `0x07` | Position loop KP parameter |
| `0x08` | Position loop KI parameter |
| `0x09` | Position loop KD parameter |

#### 2.3.5. Communication Example

**Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x32` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0xC0` | `0x3F` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x32` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0xC0` | `0x3F` | CRC16L | CRC16H |

Description: `Data[1] = 0x01`; according to the index value table, this parameter is the current loop KP. `Data[4]` through `Data[7]` form a 32-bit value (`Data[4]` is the lowest byte, `Data[7]` is the highest byte) of `0x3FC00000`. The data type is `Float`; converted to decimal this is `1.5` (an online IEEE-754 hex/float conversion tool, such as the one at speedfly.cn, can be used). This means the motor's current loop KP parameter is set to `1.5` and written to the motor drive's ROM; the parameter is retained after power-off.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x32` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0xC0` | `0x3F` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x32` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0xC0` | `0x3F` | CRC16L | CRC16H |

### 2.4. Read Acceleration Command (0x42)

#### 2.4.1. Instruction Description

The host sends this command to read the current acceleration parameters of the motor.

#### 2.4.2. Send Data Field Definition

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x42` |
| `DATA[1]` | Function index | `DATA[1] = (uint8_t)index` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

#### 2.4.3. Reply Data Field Definition

The drive's reply data contains the acceleration parameter. Acceleration data `Accel` is `int32_t`, in units of 1 dps/s, with a parameter range of 100–60000.

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x42` |
| `DATA[1]` | Function index | `DATA[1] = (uint8_t)index` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Acceleration low byte 1 | `DATA[4] = (uint8_t)(Accel)` |
| `DATA[5]` | Acceleration byte 2 | `DATA[5] = (uint8_t)(Accel>>8)` |
| `DATA[6]` | Acceleration byte 3 | `DATA[6] = (uint8_t)(Accel>>16)` |
| `DATA[7]` | Acceleration byte 4 | `DATA[7] = (uint8_t)(Accel>>24)` |

#### 2.4.4. Function Index Description

| Index value | Command name | Function description |
| --- | --- | --- |
| `0x00` | Position planning acceleration | Acceleration value from initial velocity to maximum velocity in position planning |
| `0x01` | Position planning deceleration | Deceleration value from maximum velocity to standstill in position planning |
| `0x02` | Speed planning acceleration | The acceleration value from the current speed to the target speed, including acceleration in the positive and negative directions |
| `0x03` | Speed planning deceleration | The deceleration value to decelerate from the current velocity to the target velocity in the same direction |

#### 2.4.5. Communication Example

**Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x42` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x42` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: Send a command to read the position planning acceleration.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x42` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x42` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[1]` is `0x00`, indicating the position planning acceleration value. `Data[4]` through `Data[7]` form a 32-bit value (`Data[4]` is the lowest byte, `Data[7]` is the highest byte) of `0x00002710`, which is `10000` in decimal. This means the acceleration of the motor's position loop is `10000` dps/s.

**Example 2:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x42` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x42` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: Send a command to read the position planning deceleration.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x42` | `0x01` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x42` | `0x01` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[1]` is `0x01`, indicating the position planning deceleration value. `Data[4]` through `Data[7]` form a 32-bit value (`Data[4]` is the lowest byte, `Data[7]` is the highest byte) of `0x00002710`, which is `10000` in decimal. This means the deceleration of the motor's position loop is `10000` dps/s.

### 2.5. Write Acceleration to RAM and ROM Command (0x43)

#### 2.5.1. Instruction Description

The host sends this command to write acceleration/deceleration values into RAM and ROM; the values are retained after power-off. Acceleration data `Accel` is `uint32_t`, in units of 1 dps/s, with a parameter range of 100–60000. The command covers the acceleration and deceleration values used in both position and velocity planning, selected by the index value — see the index description table in 2.5.4. Be careful to avoid writing parameters while the motor has just started or is in motion. Parameters can only be successfully saved to ROM while the motor is disabled.

#### 2.5.2. Send Data Field Definition

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x43` |
| `DATA[1]` | Function index | `DATA[1] = (uint8_t)index` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Acceleration low byte 1 | `DATA[4] = (uint8_t)(Accel)` |
| `DATA[5]` | Acceleration byte 2 | `DATA[5] = (uint8_t)(Accel>>8)` |
| `DATA[6]` | Acceleration byte 3 | `DATA[6] = (uint8_t)(Accel>>16)` |
| `DATA[7]` | Acceleration byte 4 | `DATA[7] = (uint8_t)(Accel>>24)` |

#### 2.5.3. Reply Data Field Definition

The motor replies to the host after receiving the command, and the reply command is the same as the received command.

#### 2.5.4. Function Index Description

| Index value | Command name | Function description |
| --- | --- | --- |
| `0x00` | Position planning acceleration | Acceleration value from initial velocity to maximum velocity in position planning |
| `0x01` | Position planning deceleration | Deceleration value from maximum speed to stop in position planning |
| `0x02` | Speed planning acceleration | The acceleration value from the current speed to the target speed, including acceleration in the forward and reverse directions |
| `0x03` | Speed planning deceleration | In the same direction, the deceleration value from the current speed to the target speed |

#### 2.5.5. Communication Example

**Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x43` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x43` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[1]` is `0x00`, indicating the position planning acceleration value. `Data[4]` through `Data[7]` form a 32-bit value (`Data[4]` is the lowest byte, `Data[7]` is the highest byte) of `0x00002710`, which is `10000` in decimal. This writes a position planning acceleration of `10000` dps/s to the motor drive, and the value is retained after power-off.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x43` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x43` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: The motor replies to the host after receiving the command, and the reply command is the same as the received command.

**Example 2:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x43` | `0x01` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x43` | `0x01` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[1]` is `0x01`, indicating the position planning deceleration value. `Data[4]` through `Data[7]` form a 32-bit value (`Data[4]` is the lowest byte, `Data[7]` is the highest byte) of `0x00002710`, which is `10000` in decimal. This writes a position planning deceleration of `10000` dps/s to the motor drive, and the value is retained after power-off.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x43` | `0x01` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x43` | `0x01` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: The motor replies to the host computer after receiving the command, and the reply command is the same as the received command.

**Example 3:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x43` | `0x02` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x43` | `0x02` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[1]` is `0x02`, indicating the speed planning acceleration value. `Data[4]` through `Data[7]` form a 32-bit value (`Data[4]` is the lowest byte, `Data[7]` is the highest byte) of `0x00002710`, which is `10000` in decimal. This writes a speed planning acceleration of `10000` dps/s to the motor drive, and the value is retained after power-off.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x43` | `0x02` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x43` | `0x02` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: The motor replies to the host after receiving the command, and the reply command is the same as the received command.

**Example 4:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x43` | `0x03` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x43` | `0x03` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[1]` is `0x03`, indicating the speed planning deceleration value. `Data[4]` through `Data[7]` form a 32-bit value (`Data[4]` is the lowest byte, `Data[7]` is the highest byte) of `0x00002710`, which is `10000` in decimal. This writes a speed planning deceleration of `10000` dps/s to the motor drive, and the value is retained after power-off.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x43` | `0x03` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x43` | `0x03` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: The motor replies to the host after receiving the command, and the reply command is the same as the received command.

### 2.6. Read Multi-Turn Encoder Position Data Command (0x60)

#### 2.6.1. Instruction Description

The host sends this command to read the multi-turn position of the encoder, representing the rotation angle of the motor output shaft.

#### 2.6.2. Send Data Field Definition

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x60` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

#### 2.6.3. Reply Data Field Definition

The motor replies to the host after receiving the command, and the frame data contains the following parameter: encoder multi-turn position `encoder` (`int32_t` type, value range of the multi-turn encoder, 4 bytes of valid data), which is the value obtained by subtracting the encoder's multi-turn zero offset (initial position) from the encoder's raw position.

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x60` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Encoder position low byte 1 | `DATA[4] = (uint8_t)(encoder)` |
| `DATA[5]` | Encoder position byte 2 | `DATA[5] = (uint8_t)(encoder>>8)` |
| `DATA[6]` | Encoder position byte 3 | `DATA[6] = (uint8_t)(encoder>>16)` |
| `DATA[7]` | Encoder position byte 4 | `DATA[7] = (uint8_t)(encoder>>24)` |

#### 2.6.4. Communication Example

**Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x60` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x60` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: The host sends this command to read the multi-turn position of the encoder.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x60` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x60` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[4]` through `Data[7]` form a 32-bit value (`Data[4]` is the lowest byte, `Data[7]` is the highest byte) of `0x00002710`, which is `10000` in decimal. This represents a multi-turn encoder value, relative to the current multi-turn zero offset (initial position), of `10000` pulses.

### 2.7. Read Multi-Turn Encoder Original Position Data Command (0x61)

#### 2.7.1. Instruction Description

The host sends this command to read the multi-turn encoder's raw (home) position, i.e. the multi-turn encoder value without the zero offset (home position) applied.

#### 2.7.2. Send Data Field Definition

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x61` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

#### 2.7.3. Reply Data Field Definition

The motor replies to the host after receiving the command, and the frame data contains the following parameter: encoder multi-turn raw position `encoderRaw` (`int32_t` type, 4 bytes of valid data).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x61` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Encoder original position byte 1 | `DATA[4] = (uint8_t)(encoderRaw)` |
| `DATA[5]` | Encoder original position byte 2 | `DATA[5] = (uint8_t)(encoderRaw>>8)` |
| `DATA[6]` | Encoder original position byte 3 | `DATA[6] = (uint8_t)(encoderRaw>>16)` |
| `DATA[7]` | Encoder original position byte 4 | `DATA[7] = (uint8_t)(encoderRaw>>24)` |

#### 2.7.4. Communication Example

**Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x61` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x61` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: The host sends this command to read the multi-turn encoder's raw position.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x61` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x61` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[4]` through `Data[7]` form a 32-bit value (`Data[4]` is the lowest byte, `Data[7]` is the highest byte) of `0x00002710`, which is `10000` in decimal. This means the motor's current multi-turn encoder value is `10000` pulses, excluding the zero offset (initial position).

### 2.8. Read Multi-Turn Encoder Zero Offset Data Command (0x62)

#### 2.8.1. Instruction Description

The host sends this command to read the multi-turn zero offset value (initial position) of the encoder.

#### 2.8.2. Send Data Field Definition

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x62` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

#### 2.8.3. Reply Data Field Definition

The motor replies to the host after receiving the command, and the frame data contains the following parameter: encoder multi-turn zero offset `encoderOffset` (`int32_t` type, 4 bytes of valid data).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x62` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Encoder offset byte 1 | `DATA[4] = (uint8_t)(encoderOffset)` |
| `DATA[5]` | Encoder offset byte 2 | `DATA[5] = (uint8_t)(encoderOffset>>8)` |
| `DATA[6]` | Encoder offset byte 3 | `DATA[6] = (uint8_t)(encoderOffset>>16)` |
| `DATA[7]` | Encoder offset byte 4 | `DATA[7] = (uint8_t)(encoderOffset>>24)` |

#### 2.8.4. Communication Example

**Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x62` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x62` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: The host sends this command to read the multi-turn zero offset value of the encoder.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x62` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x62` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[4]` through `Data[7]` form a 32-bit value (`Data[4]` is the lowest byte, `Data[7]` is the highest byte) of `0x00002710`, which is `10000` in decimal. This means the motor's current multi-turn encoder zero offset value is `10000` pulses.

### 2.9. Write Encoder Multi-Turn Value to ROM as Motor Zero Command (0x63)

#### 2.9.1. Instruction Description

The host sends this command to set the zero offset (initial position) of the encoder, where the multi-turn encoder value to be written, `encoderOffset`, is `int32_t` type (4 bytes of valid data). Be careful to avoid writing parameters while the motor has just started or is in motion. Parameters can only be successfully saved to ROM while the motor is disabled.

Note: After writing the new zero-point position, the motor must be restarted for it to take effect. Because the zero offset has changed, target positions should subsequently be set relative to the new zero offset (initial position).

#### 2.9.2. Send Data Field Definition

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x63` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Encoder zero offset low byte 1 | `DATA[4] = (uint8_t)(encoderOffset)` |
| `DATA[5]` | Encoder zero offset byte 2 | `DATA[5] = (uint8_t)(encoderOffset>>8)` |
| `DATA[6]` | Encoder zero offset byte 3 | `DATA[6] = (uint8_t)(encoderOffset>>8)` |
| `DATA[7]` | Encoder zero offset byte 4 | `DATA[7] = (uint8_t)(encoderOffset>>8)` |

Note: the source manual gives the shift for `DATA[6]` and `DATA[7]` as `>>8` (matching `DATA[5]`), rather than `>>16` and `>>24` as used consistently elsewhere in this document (e.g. 2.6–2.8, 2.10). This is reproduced verbatim as it appears to be an error in the original manual; the intended shifts are almost certainly `>>16` and `>>24` respectively, consistent with every other multi-byte field in this document.

#### 2.9.3. Reply Data Field Definition

The motor replies to the host after receiving the command, and the frame data is the same as the command sent by the host.

#### 2.9.4. Communication Example

**Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x63` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x63` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[4]` through `Data[7]` form a 32-bit value (`Data[4]` is the lowest byte, `Data[7]` is the highest byte) of `0x00002710`, which is `10000` in decimal. This writes `10000` pulses as the multi-turn encoder zero offset.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x63` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x63` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: The motor replies to the host after receiving the command, and the frame data is the same as the command sent by the host.

### 2.10. Write the Current Multi-Turn Position of the Encoder to the ROM as the Motor Zero Command (0x64)

#### 2.10.1. Instruction Description

Writes the motor's current encoder position into ROM as the multi-turn encoder zero offset (initial position). Be careful to avoid writing parameters while the motor has just started or is in motion. Parameters can only be successfully saved to ROM while the motor is disabled.

Note: After writing the new zero-point position, `0x76` (System Reset Command) must be sent to restart the system for it to take effect. Because the zero offset has changed, target positions should subsequently be set relative to the new zero offset (initial position).

#### 2.10.2. Send Data Field Definition

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x64` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | NULL | `0x00` |
| `DATA[5]` | NULL | `0x00` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

#### 2.10.3. Reply Data Field Definition

The motor replies to the host after receiving the command; `encoderOffset` in the data is the resulting zero offset value.

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x64` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Encoder zero offset low byte 1 | `DATA[4] = (uint8_t)(encoderOffset)` |
| `DATA[5]` | Encoder zero offset byte 2 | `DATA[5] = (uint8_t)(encoderOffset>>8)` |
| `DATA[6]` | Encoder zero offset byte 3 | `DATA[6] = (uint8_t)(encoderOffset>>16)` |
| `DATA[7]` | Encoder zero offset byte 4 | `DATA[7] = (uint8_t)(encoderOffset>>24)` |

#### 2.10.4. Communication Example

**Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x64` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x64` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: After sending the `0x64` command, the motor writes the current multi-turn encoder value into ROM as the zero offset (initial position).

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x64` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x64` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[4]` through `Data[7]` form a 32-bit value (`Data[4]` is the lowest byte, `Data[7]` is the highest byte) of `0x00002710`, which is `10000` in decimal. This means the multi-turn zero offset value (initial position) written to the motor is `10000` pulses.

### 2.11. Read Multi-Turn Angle Command (0x92)

**Instruction description:** The host sends this command to read the current multi-turn absolute angle value of the motor.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x92` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor replies to the host after receiving the command, and the frame data contains the following parameter: motor angle `motorAngle` (`int32_t` type, valid data 4 bytes), unit 0.01°/LSB.

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x92` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Angle low byte 1 | `DATA[4] = (uint8_t)(motorAngle)` |
| `DATA[5]` | Angle byte 2 | `DATA[5] = (uint8_t)(motorAngle>>8)` |
| `DATA[6]` | Angle byte 3 | `DATA[6] = (uint8_t)(motorAngle>>16)` |
| `DATA[7]` | Angle byte 4 | `DATA[7] = (uint8_t)(motorAngle>>24)` |

**Communication example — Example 1:**

Send command:

CAN: `0x141 0x92 0x00 0x00 0x00 0x00 0x00 0x00 0x00`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0x92 0x00 0x00 0x00 0x00 0x00 0x00 0x00`, `CRC16L CRC16H`.

Description: After sending the 0x92 command, it will return the absolute angle of the motor output shaft.

Reply command:

CAN: `0x241 0x92 0x00 0x00 0x00 0xA0 0x8C 0x00 0x00`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0x92 0x00 0x00 0x00 0xA0 0x8C 0x00 0x00`, `CRC16L CRC16H`.

Description: `Data[4]` through `Data[7]` form one 32-bit value (`Data[4]` is the lowest bit, `Data[7]` is the highest bit) equal to `0x00008CA0`, which is `36000` in decimal. Scaled down by the 0.01°/LSB unit (÷100), that is `36000 × 0.01 = 360°`. This indicates that the motor output shaft moves `360°` in the positive direction relative to the zero position.

---

### 2.12. Read Single-Turn Angle Command (0x94)

**Instruction description:** The host sends this command to read the current single-turn angle of the motor. Note that this command is used as a single-turn data read command for direct-drive motors. The single-turn angle range of the motor is **-180° to +180°**.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x94` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor replies to the host after receiving the command, and the frame data contains the following parameter: motor output shaft single-turn angle `Single circle angle` (`int32_t` type, valid data 4 bytes), unit 0.01°/LSB.

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x94` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Single-turn angle low byte 1 | `DATA[4] = (uint8_t)(Single circle angle)` |
| `DATA[5]` | Single-turn angle byte 2 | `DATA[5] = (uint8_t)(Single circle angle>>8)` |
| `DATA[6]` | Single-turn angle byte 3 | `DATA[6] = (uint8_t)(Single circle angle>>16)` |
| `DATA[7]` | Single-turn angle byte 4 | `DATA[7] = (uint8_t)(Single circle angle>>24)` |

**Communication example — Example 1:**

Send command:

CAN: `0x141 0x94 0x00 0x00 0x00 0x00 0x00 0x00 0x00`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0x94 0x00 0x00 0x00 0x00 0x00 0x00 0x00`, `CRC16L CRC16H`.

Description: After sending the 0x94 command, it will return the motor single-turn angle.

Reply command:

CAN: `0x241 0x94 0x00 0x00 0x00 0x4C 0x1D 0x00 0x00`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0x94 0x00 0x00 0x00 0x4C 0x1D 0x00 0x00`, `CRC16L CRC16H`.

Description: `Data[4]` through `Data[7]` form a 32-bit value (where `Data[4]` is the LSB and `Data[7]` is the MSB). A value of `0x00001D4C` corresponds to `7500` in decimal. Applying the scaling factor of 0.01°/LSB (`7500 × 0.01`), the result is `75°`. This indicates that the motor output shaft's single-turn angle is `75°`.

---

### 2.13. Read Motor Status 1 and Error Flag Command (0x9A)

**Instruction description:** This command reads the current motor temperature, voltage, and error status flags.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9A` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor replies to the host after receiving the command, and the frame data contains the following parameters:

1. Motor temperature `temperature` (`int8_t` type, unit 1°C/LSB).
2. Brake control command: indicates the state of the brake control command — `1` represents the brake release command, `0` represents the brake lock command.
3. Voltage `voltage` (`uint16_t` type, unit 0.1V/LSB).
4. Error flag `errorState` (`uint16_t` type, each bit represents a different motor state).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9A` |
| `DATA[1]` | Motor temperature | `DATA[1] = (uint8_t)(temperature)` |
| `DATA[2]` | MOS temperature | `DATA[2] = (uint8_t)(motorMOStemperature)` |
| `DATA[3]` | Brake release command | `DATA[3] = (uint8_t)(RlyCtrlRslt)` |
| `DATA[4]` | Voltage low byte | `DATA[4] = (uint8_t)(voltage)` |
| `DATA[5]` | Voltage high byte | `DATA[5] = (uint8_t)(voltage>>8)` |
| `DATA[6]` | Error status low byte 1 | `DATA[6] = (uint8_t)(errorState)` |
| `DATA[7]` | Error status byte 2 | `DATA[7] = (uint8_t)(errorState>>8)` |

**Remark:** System abnormal state value `System_errorState` state table 1 is as follows:

| `System_errorState` | Status description |
| --- | --- |
| `0x0002` | **Motor stall.** A Motor Stall Error is triggered if the motor current exceeds the "Stall Current" parameter for a duration longer than the "Stall Time" parameter. This error is non-auto-recoverable. |
| `0x0004` | **Low voltage.** A Low Voltage Error is triggered when the bus voltage drops below the "Undervoltage Protection Voltage" parameter. This error automatically clears once the bus voltage rises above the "Undervoltage Protection Voltage" threshold. |
| `0x0008` | **Over voltage.** An Overvoltage Error is triggered when the bus voltage exceeds the "Overvoltage Protection Voltage" parameter. This error automatically recovers when the bus voltage drops below the "Overvoltage Protection Voltage" threshold. |
| `0x0010` | **Over current.** A Phase Overcurrent Error is triggered when the motor current exceeds the "Maximum Current" parameter. This error is latched and cannot be automatically recovered. |
| `0x0040` | **Power overrun.** |
| `0x0080` | **Calibration parameter writing error.** |
| `0x0100` | **Overspeed.** An Overspeed Error is triggered when the motor input speed exceeds the "Maximum Speed" parameter. This error automatically recovers when the input speed drops below the "Maximum Speed" threshold. |
| `0x0800` | **Component overtemperature.** A Component Overtemperature Error is triggered when the PCB temperature exceeds 110°C. This error automatically recovers once the PCB temperature drops below 90°C. |
| `0x1000` | **Motor temperature over temperature.** A Motor Overtemperature Error is triggered when the motor stator temperature exceeds the "Overtemperature Protection Temperature" parameter. This error automatically recovers when the stator temperature drops below the "Overtemperature Recovery Temperature" parameter. |
| `0x2000` | **Encoder calibration error.** An Encoder Calibration Error is triggered if the motor calibration fails. This error is non-auto-recoverable. |
| `0x4000` | **Encoder data error.** An Encoder Data Error is triggered if the motor encoder data is abnormal. This error is non-auto-recoverable. |

When multiple errors occur at the same time, the error status bits are displayed superimposed (additively). For example, if the value `0x0016` appears, it means `0x2 + 0x4 + 0x10` added together — i.e. motor stall, low voltage, and phase overcurrent are present simultaneously.

**Communication example — Example 1:**

Send command:

CAN: `0x141 0x9A 0x00 0x00 0x00 0x00 0x00 0x00 0x00`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0x9A 0x00 0x00 0x00 0x00 0x00 0x00 0x00`, `CRC16L CRC16H`.

Description: After sending the 0x9A command, the temperature, voltage, and error status flags of the motor will be returned.

Reply command:

CAN: `0x241 0x9A 0x32 0x00 0x01 0xE5 0x01 0x04 0x00`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0x9A 0x32 0x00 0x01 0xE5 0x01 0x04 0x00`, `CRC16L CRC16H`.

Description: `Data[1] = 0x32` is `50` in decimal, meaning the motor temperature is `50°C` at the moment. `Data[3]` indicates the state of the brake control command — `1` represents the brake release command, `0` represents the brake lock command — so `0x01` indicates that the brake release command has been executed. `Data[4]` and `Data[5]` (`Data[4]` low, `Data[5]` high) form `0x01E5`, which is `485` in decimal; scaled down by 10 per the 0.1V/LSB unit, `485 × 0.1 = 48.5V`, meaning the current motor supply voltage is `48.5V`. `Data[6]` and `Data[7]` (`Data[6]` low, `Data[7]` high) form `0x0004`, which per the `System_errorState` table indicates a low-voltage error.

---

### 2.14. Read Motor Status 2 Command (0x9C)

**Instruction description:** This command reads the temperature, speed, and encoder position of the current motor.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9C` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor replies to the host after receiving the command, and the frame data contains the following parameters:

1. Motor temperature `temperature` (`int8_t` type, 1°C/LSB).
2. Motor torque current value `iq` (`int16_t` type, 0.01A/LSB).
3. Motor output shaft speed `speed` (`int16_t` type, 1dps/LSB).
4. Motor output shaft angle (`int16_t` type, 1degree/LSB, maximum range ±32767 degree).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9C` |
| `DATA[1]` | Motor temperature | `DATA[1] = (uint8_t)(temperature)` |
| `DATA[2]` | Torque current low byte | `DATA[2] = (uint8_t)(iq)` |
| `DATA[3]` | Torque current high byte | `DATA[3] = (uint8_t)(iq>>8)` |
| `DATA[4]` | Motor speed low byte | `DATA[4] = (uint8_t)(speed)` |
| `DATA[5]` | Motor speed high byte | `DATA[5] = (uint8_t)(speed>>8)` |
| `DATA[6]` | Motor angle low byte | `DATA[6] = (uint8_t)(degree)` |
| `DATA[7]` | Motor angle high byte | `DATA[7] = (uint8_t)(degree>>8)` |

**Communication example — Example 1:**

Send command:

CAN: `0x141 0x9C 0x00 0x00 0x00 0x00 0x00 0x00 0x00`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0x9C 0x00 0x00 0x00 0x00 0x00 0x00 0x00`, `CRC16L CRC16H`.

Description: This command reads the current temperature, speed, and encoder position of the motor.

Reply command:

CAN: `0x241 0x9C 0x32 0x64 0x00 0xF4 0x01 0x2D 0x00`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0x9C 0x32 0x64 0x00 0xF4 0x01 0x2D 0x00`, `CRC16L CRC16H`.

Description: `Data[1] = 0x32` is `50` in decimal, meaning the motor temperature is `50°C` at the moment. The composite data of `Data[2]` and `Data[3]`, `0x0064`, is `100` in decimal; scaled down by 100 (0.01A/LSB), `100 × 0.01 = 1A`, meaning the actual current of the motor is `1A`. The composite data of `Data[4]` and `Data[5]`, `0x01F4`, is `500` in decimal, meaning the motor output shaft speed is `500dps`. There is a reduction-ratio relationship between the motor output shaft speed and the motor speed — if the reduction ratio is 6, the motor speed is 6 times higher than the output shaft speed. The composite data of `Data[6]` and `Data[7]`, `0x002D`, is `45` in decimal, meaning the motor output shaft moves `45°` in the positive direction relative to the zero position. The output shaft position is related to the number of motor encoder lines and the reduction ratio — for example, if the encoder has 16384 lines and the reduction ratio is 6, then 360° of the motor output shaft corresponds to `16384 × 6 = 98304` pulses.

---

### 2.15. Read Motor Status 3 Command (0x9D)

**Instruction description:** This command reads the current motor temperature and phase current data.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9D` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor replies to the host after receiving the command, and the frame data contains the following parameters:

1. Motor temperature `temperature` (`int8_t` type, 1°C/LSB).
2. Phase A current data (`int16_t` type; the corresponding actual phase current unit is 0.01A/LSB).
3. Phase B current data (`int16_t` type; the corresponding actual phase current unit is 0.01A/LSB).
4. Phase C current data (`int16_t` type; the corresponding actual phase current unit is 0.01A/LSB).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9D` |
| `DATA[1]` | Motor temperature | `DATA[1] = (uint8_t)(temperature)` |
| `DATA[2]` | Phase A current low byte | `DATA[2] = (uint8_t)(iA)` |
| `DATA[3]` | Phase A current high byte | `DATA[3] = (uint8_t)(iA>>8)` |
| `DATA[4]` | Phase B current low byte | `DATA[4] = (uint8_t)(iB)` |
| `DATA[5]` | Phase B current high byte | `DATA[5] = (uint8_t)(iB>>8)` |
| `DATA[6]` | Phase C current low byte | `DATA[6] = (uint8_t)(iC)` |
| `DATA[7]` | Phase C current high byte | `DATA[7] = (uint8_t)(iC>>8)` |

**Communication example — Example 1:**

Send command:

CAN: `0x141 0x9D 0x00 0x00 0x00 0x00 0x00 0x00 0x00`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0x9D 0x00 0x00 0x00 0x00 0x00 0x00 0x00`, `CRC16L CRC16H`.

Description: This command reads the current motor temperature and phase current data.

Reply command:

CAN: `0x241 0x9D 0x32 0xC2 0x0B 0x10 0xFA 0xC0 0xF9`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0x9D 0x32 0xC2 0x0B 0x10 0xFA 0xC0 0xF9`, `CRC16L CRC16H`.

Description: `Data[1] = 0x32` is `50` in decimal, meaning the motor temperature is `50°C` at the moment. The composite data of `Data[2]` and `Data[3]`, `0x0BC2`, is `3010` in decimal; scaled down by 100, `3010 × 0.01 = 30.1A`, meaning the actual phase-A current is `30.1A`. The composite data of `Data[4]` and `Data[5]`, `0xFA10`, is `-1520` in decimal; scaled down by 100, `-1520 × 0.01 = -15.2A`, meaning the actual phase-B current is `-15.2A`. The composite data of `Data[6]` and `Data[7]`, `0xF9C0`, is `-1600` in decimal; scaled down by 100, `-1600 × 0.01 = -16A`, meaning the actual phase-C current is `-16A`.

---

### 2.16. Motor Shutdown Command (0x80)

**Instruction description:** Turns off the motor output and also clears the motor running state; the motor is not in any closed-loop mode.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x80` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor replies to the host after receiving the command, and the frame data is the same as that sent by the host.

---

### 2.17. Motor Stop Command (0x81)

**Instruction description:** Stops the motor — brings the motor speed to zero and holds the motor in place, without changing the closed-loop mode the motor is running in; it just stops the motor speed.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x81` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor replies to the host after receiving the command, and the frame data is the same as that sent by the host.

---

### 2.18. Torque Closed-Loop Control Command (0xA1)

**Instruction description:** This command is a control command, which can be run when the motor is not faulty. The host sends this command to control the torque current output of the motor. The control value `iqControl` is of type `int16_t` and the unit is 0.01A/LSB.

For safety reasons, this command cannot open the brake directly. Use the 0x77 command to open the brake first, then use the 0xA1 command.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA1` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Torque current control value low byte | `DATA[4] = (uint8_t)(iqControl)` |
| `DATA[5]` | Torque current control value high byte | `DATA[5] = (uint8_t)(iqControl>>8)` |
| `DATA[6]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor replies to the host after receiving the command, and the frame data contains the following parameters:

1. Motor temperature `temperature` (`int8_t` type, 1°C/LSB).
2. Motor torque current value `iq` (`int16_t` type, 0.01A/LSB).
3. Motor output shaft speed `speed` (`int16_t` type, 1dps/LSB).
4. Motor output shaft angle (`int16_t` type, 1degree/LSB, maximum range ±32767 degree).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA1` |
| `DATA[1]` | Motor temperature | `DATA[1] = (uint8_t)(temperature)` |
| `DATA[2]` | Torque current low byte | `DATA[2] = (uint8_t)(iq)` |
| `DATA[3]` | Torque current high byte | `DATA[3] = (uint8_t)(iq>>8)` |
| `DATA[4]` | Motor speed low byte | `DATA[4] = (uint8_t)(speed)` |
| `DATA[5]` | Motor speed high byte | `DATA[5] = (uint8_t)(speed>>8)` |
| `DATA[6]` | Motor angle low byte | `DATA[6] = (uint8_t)(degree)` |
| `DATA[7]` | Motor angle high byte | `DATA[7] = (uint8_t)(degree>>8)` |

**Communication example — Example 1:**

Send command:

CAN: `0x141 0xA1 0x00 0x00 0x00 0x64 0x00 0x00 0x00`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0xA1 0x00 0x00 0x00 0x64 0x00 0x00 0x00`, `CRC16L CRC16H`.

Description: `Data[4]` and `Data[5]` represent the data (`Data[4]` = `0x64` is the low byte, `Data[5]` = `0x00` is the high byte). The actual data is `0x0064`, i.e. `100` decimal; scaled down per 0.01A/LSB, `100 × 0.01 = 1A`. The driver runs with `1A` as the target current.

Reply command:

CAN: `0x241 0xA1 0x32 0x64 0x00 0xF4 0x01 0x2D 0x00`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0xA1 0x32 0x64 0x00 0xF4 0x01 0x2D 0x00`, `CRC16L CRC16H`.

Description: `Data[1] = 0x32` is `50` in decimal, meaning the motor temperature is `50°C` at the moment. The composite data of `Data[2]` and `Data[3]`, `0x0064`, is `100` in decimal; scaled down by 100, `100 × 0.01 = 1A`, meaning the actual current of the motor is `1A`. The composite data of `Data[4]` and `Data[5]`, `0x01F4`, is `500` in decimal, meaning the motor output shaft speed is `500dps`. There is a reduction-ratio relationship between the motor output shaft speed and the motor speed — if the reduction ratio is 6, the motor speed is 6 times higher than the output shaft speed. The composite data of `Data[6]` and `Data[7]`, `0x002D`, is `45` in decimal, meaning the motor output shaft moves `45°` in the positive direction relative to the zero position. The output shaft position is related to the number of motor encoder lines and the reduction ratio — for example, if the encoder has 16384 lines and the reduction ratio is 6, then 360° of the motor output shaft corresponds to `16384 × 6 = 98304` pulses.

**Communication example — Example 2:**

Send command:

CAN: `0x141 0xA1 0x00 0x00 0x00 0x9C 0xFF 0x00 0x00`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0xA1 0x00 0x00 0x00 0x9C 0xFF 0x00 0x00`, `CRC16L CRC16H`.

Description: `Data[4]` and `Data[5]` represent the data (`Data[4]` = `0x9C` is the low byte, `Data[5]` = `0xFF` is the high byte). The actual data is `0xFF9C`, i.e. `-100` decimal; scaled down per 0.01A/LSB, `-100 × 0.01 = -1A`. The driver runs with `-1A` as the target current.

Reply command:

CAN: `0x241 0xA1 0x32 0x9C 0xFF 0x0C 0xFE 0xD3 0xFF`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0xA1 0x32 0x9C 0xFF 0x0C 0xFE 0xD3 0xFF`, `CRC16L CRC16H`.

Description: `Data[1] = 0x32` is `50` in decimal, meaning the motor temperature is `50°C` at the moment. The composite data of `Data[2]` and `Data[3]`, `0xFF9C`, is `-100` in decimal; scaled down by 100, `-100 × 0.01 = -1A`, meaning the actual current of the motor is `-1A`. The composite data of `Data[4]` and `Data[5]`, `0xFE0C`, is `-500` in decimal, meaning the motor output shaft speed is `-500dps`. There is a reduction-ratio relationship between the motor output shaft speed and the motor speed — if the reduction ratio is 6, the motor speed is 6 times higher than the output shaft speed. The composite data of `Data[6]` and `Data[7]`, `0xFFD3`, is `-45` in decimal, meaning the motor output shaft moves in the opposite direction by `-45°` relative to the zero position. The output shaft position is related to the number of motor encoder lines and the reduction ratio — for example, if the encoder has 16384 lines and the reduction ratio is 6, then 360° of the motor output shaft corresponds to `16384 × 6 = 98304` pulses.

---

### 2.19. Speed Closed-Loop Control Command (0xA2)

**Instruction description:** This command is a control command, which can be run when the motor is not faulty. The host sends this command to control the speed of the motor output shaft. The control value `speedControl` is `int32_t` type, and the corresponding actual speed unit is 0.01dps/LSB. The control value `maxTorque` limits the maximum torque of the motor output shaft; it is `uint8_t` type, with a value range of 0 to 255. The unit is a percentage of the rated current, specifically 1% of rated current per LSB. If the given current is 0 or greater than the stall current, force control mode will not be activated — the maximum torque current of the motor is then limited by the motor stall current value set in the host/setup software.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA2` |
| `DATA[1]` | Max torque | `DATA[1] = (uint8_t)(maxTorque)` |
| `DATA[2]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Speed control low byte | `DATA[4] = (uint8_t)(speedControl)` |
| `DATA[5]` | Speed control byte 2 | `DATA[5] = (uint8_t)(speedControl>>8)` |
| `DATA[6]` | Speed control byte 3 | `DATA[6] = (uint8_t)(speedControl>>16)` |
| `DATA[7]` | Speed control high byte | `DATA[7] = (uint8_t)(speedControl>>24)` |

**Remark:**

1. The maximum torque current of the motor under this command is limited by the Max Torque Current value in the host computer.
2. In this control mode, the maximum acceleration of the motor is limited by the Max Acceleration value in the host computer.
3. When the speed-loop acceleration value is 0, the speed-loop acceleration is limited by the maximum current output capability.

**Reply data field:** The motor replies to the host after receiving the command, and the frame data contains the following parameters:

1. Motor temperature `temperature` (`int8_t` type, 1°C/LSB).
2. Motor torque current value `iq` (`int16_t` type, 0.01A/LSB).
3. Motor output shaft speed `speed` (`int16_t` type, 1dps/LSB).
4. Motor output shaft angle (`int16_t` type, 1degree/LSB, maximum range ±32767 degree).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA2` |
| `DATA[1]` | Motor temperature | `DATA[1] = (uint8_t)(temperature)` |
| `DATA[2]` | Torque current low byte | `DATA[2] = (uint8_t)(iq)` |
| `DATA[3]` | Torque current high byte | `DATA[3] = (uint8_t)(iq>>8)` |
| `DATA[4]` | Motor speed low byte | `DATA[4] = (uint8_t)(speed)` |
| `DATA[5]` | Motor speed high byte | `DATA[5] = (uint8_t)(speed>>8)` |
| `DATA[6]` | Motor angle low byte | `DATA[6] = (uint8_t)(degree)` |
| `DATA[7]` | Motor angle high byte | `DATA[7] = (uint8_t)(degree>>8)` |

**Communication example — Example 1:**

Send command:

CAN: `0x141 0xA2 0x00 0x00 0x00 0x10 0x27 0x00 0x00`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0xA2 0x00 0x00 0x00 0x10 0x27 0x00 0x00`, `CRC16L CRC16H`.

Description: `Data[4]` through `Data[7]` form one 32-bit value (`Data[4]` is the lowest bit, `Data[7]` is the highest bit) equal to `0x00002710`, which is `10000` in decimal. The sending command is scaled down by 100 per 0.01dps/LSB, that is `10000 × 0.01 = 100dps`. The drive operates at a target speed of `100dps` of the motor output shaft.

Reply command:

CAN: `0x241 0xA2 0x32 0x64 0x00 0xF4 0x01 0x2D 0x00`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0xA2 0x32 0x64 0x00 0xF4 0x01 0x2D 0x00`, `CRC16L CRC16H`.

Description: `Data[1] = 0x32` is `50` in decimal, meaning the motor temperature is `50°C` at the moment. The composite data of `Data[2]` and `Data[3]`, `0x0064`, is `100` in decimal; scaled down by 100, `100 × 0.01 = 1A`, meaning the actual current of the motor is `1A`. The composite data of `Data[4]` and `Data[5]`, `0x01F4`, is `500` in decimal, meaning the motor output shaft speed is `500dps`. There is a reduction-ratio relationship between the motor output shaft speed and the motor speed — if the reduction ratio is 6, the motor speed is 6 times higher than the output shaft speed. The composite data of `Data[6]` and `Data[7]`, `0x002D`, is `45` in decimal, meaning the motor output shaft moves `45°` in the positive direction relative to the zero position. The output shaft position is related to the number of motor encoder lines and the reduction ratio — for example, if the encoder has 16384 lines and the reduction ratio is 6, then 360° of the motor output shaft corresponds to `16384 × 6 = 98304` pulses.

**Communication example — Example 2:**

Send command:

CAN: `0x141 0xA2 0x00 0x00 0x00 0xF0 0xD8 0xFF 0xFF`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0xA2 0x00 0x00 0x00 0xF0 0xD8 0xFF 0xFF`, `CRC16L CRC16H`.

Description: `Data[4]` through `Data[7]` form one 32-bit value (`Data[4]` is the lowest bit, `Data[7]` is the highest bit) equal to `0xFFFFD8F0`, which is `-10000` in decimal. The sending command is scaled down by 100 per 0.01dps/LSB, that is `-10000 × 0.01 = -100dps`. The drive runs at a target speed of `-100dps` of the motor output shaft.

Reply command:

CAN: `0x241 0xA2 0x32 0x9C 0xFF 0x0C 0xFE 0xD3 0xFF`

RS485: frame header `0x3E`, ID `0x01`, Length `0x08`, `0xA2 0x32 0x9C 0xFF 0x0C 0xFE 0xD3 0xFF`, `CRC16L CRC16H`.

Description: `Data[1] = 0x32` is `50` in decimal, meaning the motor temperature is `50°C` at the moment. The composite data of `Data[2]` and `Data[3]`, `0xFF9C`, is `-100` in decimal; scaled down by 100, `-100 × 0.01 = -1A`, meaning the actual current of the motor is `-1A`. The composite data of `Data[4]` and `Data[5]`, `0xFE0C`, is `-500` in decimal, meaning the motor output shaft speed is `-500dps`. There is a reduction-ratio relationship between the motor output shaft speed and the motor speed — if the reduction ratio is 6, the motor speed is 6 times higher than the output shaft speed. The composite data of `Data[6]` and `Data[7]`, `0xFFD3`, is `-45` in decimal, meaning the motor output shaft moves in the opposite direction by `-45°` relative to the zero position. The output shaft position is related to the number of motor encoder lines and the reduction ratio — for example, if the encoder has 16384 lines and the reduction ratio is 6, then 360° of the motor output shaft corresponds to `16384 × 6 = 98304` pulses.

### 2.20. Absolute Position Closed-Loop Control Command (0xA4)

**Instruction description:** This command is a control command, which can be run when the motor is not faulty. The host sends this command to control the position of the motor (multi-turn angle). The control value `angleControl` is `int32_t` type, and the corresponding actual position is 0.01degree/LSB, that is, 36000 represents 360°, and the rotation direction of the motor is determined by the difference between the target position and the current position. The control value `maxSpeed` limits the maximum speed of the motor output shaft rotation, which is of type `uint16_t`, corresponding to the actual speed of 1dps/LSB.

According to the position planning acceleration value set by the system, different operating modes will result:

1. If the position loop acceleration is 0, then the position loop will enter direct tracking mode, and directly track the target position through the PI controller. Among them, `maxSpeed` limits the maximum speed during the position operation process. If the `maxSpeed` value is 0, then it is completely output by the calculation result of the PI controller (see Figure 2-1, Block Diagram of Position Tracking Mode with Speed Limit).
2. If the position loop acceleration is non-zero, the motor will operate in a velocity-profiled motion mode, where the motor handles the acceleration and deceleration phases. In this mode, `maxSpeed` limits the peak velocity during movement, while the actual acceleration is determined by the position loop's acceleration setting.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA4` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | Speed limit low byte | `DATA[2] = (uint8_t)(maxSpeed)` |
| `DATA[3]` | Speed limit high byte | `DATA[3] = (uint8_t)(maxSpeed>>8)` |
| `DATA[4]` | Position control low byte | `DATA[4] = (uint8_t)(angleControl)` |
| `DATA[5]` | Position control | `DATA[5] = (uint8_t)(angleControl>>8)` |
| `DATA[6]` | Position control | `DATA[6] = (uint8_t)(angleControl>>16)` |
| `DATA[7]` | Position control high byte | `DATA[7] = (uint8_t)(angleControl>>24)` |

**Reply data field:** The motor replies to the host after receiving the command, and the frame data contains the following parameters:

1. Motor temperature `temperature` (`int8_t` type, 1°C/LSB).
2. The torque current value `iq` of the motor (`int16_t` type, 0.01A/LSB).
3. Motor output shaft speed (`int16_t` type, 1dps/LSB).
4. Motor output shaft angle (`int16_t` type, 1degree/LSB, maximum range ±32767 degree).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA4` |
| `DATA[1]` | Motor temperature | `DATA[1] = (uint8_t)(temperature)` |
| `DATA[2]` | Torque current low byte | `DATA[2] = (uint8_t)(iq)` |
| `DATA[3]` | Torque current high byte | `DATA[3] = (uint8_t)(iq>>8)` |
| `DATA[4]` | Motor speed low byte | `DATA[4] = (uint8_t)(speed)` |
| `DATA[5]` | Motor speed high byte | `DATA[5] = (uint8_t)(speed>>8)` |
| `DATA[6]` | Motor angle low byte | `DATA[6] = (uint8_t)(degree)` |
| `DATA[7]` | Motor angle high byte | `DATA[7] = (uint8_t)(degree>>8)` |

**Communication example — Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0xA4` | `0x00` | `0xF4` | `0x01` | `0xA0` | `0x8C` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA4` | `0x00` | `0xF4` | `0x01` | `0xA0` | `0x8C` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: Data[2] and Data[3] form one (Data[2] is low, Data[3] is high) 16-bit data, `0x01F4`, indicating the decimal 500dps motor output shaft speed. The drive will run the position loop at this speed as the maximum speed.

Data[4] to Data[7] form a (Data[4] is the lowest byte, Data[7] is the highest byte) 32-bit data, `0x00008CA0`, which means 36000 in decimal. The sending command is scaled by 0.01degree/LSB, that is, 36000*0.01=360°. The motor will move forward 360° with the output shaft relative to the zero position.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0xA4` | `0x32` | `0x64` | `0x00` | `0xF4` | `0x01` | `0x2D` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA4` | `0x32` | `0x64` | `0x00` | `0xF4` | `0x01` | `0x2D` | `0x00` | CRC16L | CRC16H |

Description: `Data[1] = 0x32` is 50 in decimal, which means the motor temperature is 50 degrees at the moment.

The composite data of Data[2] and Data[3], `0x0064`, is 100 in decimal, which is 100*0.01=1A according to the 100-fold reduction, which means that the actual current of the motor is 1A.

The composite data of Data[4] and Data[5], `0x01F4`, is 500 in decimal, which means the motor output shaft speed is 500dps. There is a reduction ratio relationship between the motor output shaft speed and the motor speed. If the reduction ratio is 6, the motor speed is 6 times higher than the output shaft speed.

The composite data of Data[6] and Data[7], `0x002D`, is 45 in decimal, which means that the motor output shaft moves forward by 45 degrees relative to the zero position. The position of the motor output shaft is related to the number of lines of the motor encoder and the reduction ratio. For example, the number of lines of the motor encoder is 16384 and the reduction ratio is 6. Then 360 degrees of the motor output shaft corresponds to 16384*6 = 98304 pulses.

**Communication example — Example 2:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0xA4` | `0x00` | `0xF4` | `0x01` | `0x60` | `0x73` | `0xFF` | `0xFF` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA4` | `0x00` | `0xF4` | `0x01` | `0x60` | `0x73` | `0xFF` | `0xFF` | CRC16L | CRC16H |

Description: Data[2] and Data[3] form one (Data[2] is low, Data[3] is high) 16-bit data, `0x01F4`, indicating the decimal 500dps motor output shaft speed. The drive will run the position loop at this speed as the maximum speed.

Data[4] to Data[7] form a (Data[4] is the lowest byte, Data[7] is the highest byte) 32-bit data, `0xFFFF7360`, which means -36000 in decimal. The sending command is scaled by 0.01degree/LSB, that is, -36000*0.01=-360°. The motor will move -360° in reverse with respect to the zero position of the output shaft.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0xA4` | `0x32` | `0x9C` | `0xFF` | `0x0C` | `0xFE` | `0xD3` | `0xFF` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA4` | `0x32` | `0x9C` | `0xFF` | `0x0C` | `0xFE` | `0xD3` | `0xFF` | CRC16L | CRC16H |

Description: `Data[1] = 0x32` is 50 in decimal, which means the motor temperature is 50 degrees at the moment.

Data[2] and Data[3] synthesized data, `0xFF9C`, is -100 in decimal, which is -100*0.01=-1A when scaled down by 100 times, which means the actual current of the motor is -1A.

The composite data of Data[4] and Data[5], `0xFE0C`, is -500 in decimal, which means that the motor output shaft speed is -500dps. There is a reduction ratio relationship between the motor output shaft speed and the motor speed. If the reduction ratio is 6, the motor speed is 6 times higher than the output shaft speed.

The composite data of Data[6] and Data[7], `0xFFD3`, is -45 in decimal, which means that the output shaft of the motor moves backward by -45 degrees relative to the zero position. The position of the motor output shaft is related to the number of lines of the motor encoder and the reduction ratio. For example, the number of lines of the motor encoder is 16384 and the reduction ratio is 6. Then 360 degrees of the motor output shaft corresponds to 16384*6 = 98304 pulses.

### 2.21. Single-Turn Position Control Command (0xA6)

**Instruction description:** The host sends this command to control the position of the motor (single-turn angle). When the multi-turn save function is turned off, the default is single-turn mode. This instruction can be used in single-turn mode, and is mainly applied to direct-drive motors.

1. The angle control value `angleControl` is of `uint16_t` type, the value range is 0~35999, and the corresponding actual position is 0.01degree/LSB, that is, the actual angle range is 0°~359.99°.
2. `spinDirection` sets the direction of motor rotation, which is `uint8_t` type; `0x00` means clockwise, and `0x01` means counterclockwise.
3. `maxSpeed` limits the maximum speed of motor rotation, which is of `uint16_t` type, corresponding to the actual speed of 1dps/LSB.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA6` |
| `DATA[1]` | Rotation direction byte | `DATA[1] = spinDirection` |
| `DATA[2]` | Speed limit low byte | `DATA[2] = (uint8_t)(maxSpeed)` |
| `DATA[3]` | Speed limit high byte | `DATA[3] = (uint8_t)(maxSpeed>>8)` |
| `DATA[4]` | Position control low byte | `DATA[4] = (uint8_t)(angleControl)` |
| `DATA[5]` | Position control high byte | `DATA[5] = (uint8_t)(angleControl>>8)` |
| `DATA[6]` | NULL | `0x00` |
| `DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor replies to the host after receiving the command, and the frame data contains the following parameters:

1. Motor temperature `temperature` (`int8_t` type, 1°C/LSB).
2. The torque current value `iq` of the motor (`int16_t` type, 0.01A/LSB).
3. Motor output shaft speed (`int16_t` type, 1dps/LSB).
4. Encoder position value `encoder` (`uint16_t` type, the value range of the encoder is determined by the number of bits of the encoder).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA6` |
| `DATA[1]` | Motor temperature | `DATA[1] = (uint8_t)(temperature)` |
| `DATA[2]` | Torque current low byte | `DATA[2] = (uint8_t)(iq)` |
| `DATA[3]` | Torque current high byte | `DATA[3] = (uint8_t)(iq>>8)` |
| `DATA[4]` | Motor speed low byte | `DATA[4] = (uint8_t)(speed)` |
| `DATA[5]` | Motor speed high byte | `DATA[5] = (uint8_t)(speed>>8)` |
| `DATA[6]` | Encoder value low byte | `DATA[6] = (uint8_t)(encoder)` |
| `DATA[7]` | Encoder value high byte | `DATA[7] = (uint8_t)(encoder>>8)` |

**Communication example — Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0xA6` | `0x00` | `0xF4` | `0x01` | `0xA0` | `0x8C` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA6` | `0x00` | `0xF4` | `0x01` | `0xA0` | `0x8C` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[1]` is 0, which means the motor will rotate clockwise.

Data[2] and Data[3] form one (Data[2] is low, Data[3] is high) 16-bit data, `0x01F4`, which means the decimal 500dps motor speed. The drive will run the position loop at this speed as the maximum speed.

Data[4] to Data[7] form a (Data[4] is the lowest byte, Data[7] is the highest byte) 32-bit data, `0x8CA0`, which means that the decimal value is 36000, and the unit is 0.01degree. The motor will move 360° clockwise. Since the 360-degree and 0-degree positions in the single-turn position coincide, the position may also read 0 degrees at this time.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0xA6` | `0x32` | `0x64` | `0x00` | `0xF4` | `0x01` | `0xE8` | `0x03` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA6` | `0x32` | `0x64` | `0x00` | `0xF4` | `0x01` | `0xE8` | `0x03` | CRC16L | CRC16H |

Description: `Data[1] = 0x32` is 50 in decimal, which means the motor temperature is 50 degrees at the moment.

The composite data of Data[2] and Data[3], `0x0064`, is 100 in decimal, which is 100*0.01=1A according to the 100-fold reduction, which means that the actual current of the motor is 1A.

Data[4] and Data[5] synthesized data, `0x01F4`, is 500 in decimal, which means the motor speed is 500dps.

The composite data of Data[6] and Data[7], `0x03E8`, is 1000 in decimal, which means that the value of the motor encoder relative to the zero position is 1000 pulses.

**Communication example — Example 2:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0xA6` | `0x01` | `0xF4` | `0x01` | `0xA0` | `0x8C` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA6` | `0x01` | `0xF4` | `0x01` | `0xA0` | `0x8C` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[1]` is 1, which means the motor will rotate counterclockwise.

Data[2] and Data[3] form one (Data[2] is low, Data[3] is high) 16-bit data, `0x01F4`, which means the decimal 500dps motor speed. The drive will run the position loop at this speed as the maximum speed.

Data[4] to Data[7] form a (Data[4] is the lowest byte, Data[7] is the highest byte) 32-bit data, `0x8CA0`, which means that the decimal value is 36000, and the unit is 0.01degree. The motor will move 360° counterclockwise. Since the 360-degree and 0-degree positions in the single-turn position coincide, the position may also read 0 degrees at this time.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0xA6` | `0x32` | `0x64` | `0x00` | `0xF4` | `0x01` | `0xE8` | `0x03` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA6` | `0x32` | `0x64` | `0x00` | `0xF4` | `0x01` | `0xE8` | `0x03` | CRC16L | CRC16H |

Description: `Data[1] = 0x32` is 50 in decimal, which means the motor temperature is 50 degrees at the moment.

The composite data of Data[2] and Data[3], `0x0064`, is 100 in decimal, which is 100*0.01=1A according to the 100-fold reduction, which means that the actual current of the motor is 1A.

Data[4] and Data[5] synthesized data, `0x01F4`, is 500 in decimal, which means the motor speed is 500dps.

The composite data of Data[6] and Data[7], `0x03E8`, is 1000 in decimal, which means that the value of the motor encoder relative to the zero position is 1000 pulses.

### 2.22. Incremental Position Closed-Loop Control Command (0xA8)

**Instruction description:** This command is a control command, which can be run when the motor is not faulty. The host sends this command to control the incremental position (multi-turn angle) of the motor, and runs the input position increment with the current position as the starting point. The control value `angleControl` is of type `int32_t`, and the corresponding actual position is 0.01degree/LSB, that is, 36000 represents 360°, and the rotation direction of the motor is determined by the sign of the incremental position. The control value `maxSpeed` limits the maximum speed of the motor output shaft rotation, which is of type `uint16_t`, corresponding to the actual speed of 1dps/LSB.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA8` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | Speed limit low byte | `DATA[2] = (uint8_t)(maxSpeed)` |
| `DATA[3]` | Speed limit high byte | `DATA[3] = (uint8_t)(maxSpeed>>8)` |
| `DATA[4]` | Position control low byte | `DATA[4] = (uint8_t)(angleControl)` |
| `DATA[5]` | Position control | `DATA[5] = (uint8_t)(angleControl>>8)` |
| `DATA[6]` | Position control | `DATA[6] = (uint8_t)(angleControl>>16)` |
| `DATA[7]` | Position control high byte | `DATA[7] = (uint8_t)(angleControl>>24)` |

**Reply data field:** The motor replies to the host after receiving the command, and the frame data contains the following parameters:

1. Motor temperature `temperature` (`int8_t` type, 1°C/LSB).
2. The torque current value `iq` of the motor (`int16_t` type, 0.01A/LSB).
3. Motor output shaft speed (`int16_t` type, 1dps/LSB).
4. Motor output shaft angle (`int16_t` type, 1degree/LSB, maximum range ±32767 degree).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA8` |
| `DATA[1]` | Motor temperature | `DATA[1] = (uint8_t)(temperature)` |
| `DATA[2]` | Torque current low byte | `DATA[2] = (uint8_t)(iq)` |
| `DATA[3]` | Torque current high byte | `DATA[3] = (uint8_t)(iq>>8)` |
| `DATA[4]` | Motor speed low byte | `DATA[4] = (uint8_t)(speed)` |
| `DATA[5]` | Motor speed high byte | `DATA[5] = (uint8_t)(speed>>8)` |
| `DATA[6]` | Motor angle low byte | `DATA[6] = (uint8_t)(degree)` |
| `DATA[7]` | Motor angle high byte | `DATA[7] = (uint8_t)(degree>>8)` |

**Communication example — Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0xA8` | `0x00` | `0xF4` | `0x01` | `0xA0` | `0x8C` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA8` | `0x00` | `0xF4` | `0x01` | `0xA0` | `0x8C` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: Data[2] and Data[3] form one (Data[2] is low, Data[3] is high) 16-bit data, `0x01F4`, indicating the decimal 500dps motor output shaft speed. The drive will run the position loop at this speed as the maximum speed.

Data[4] to Data[7] form one (Data[4] is the lowest byte, Data[7] is the highest byte) 32-bit data, `0x00008CA0`, which means 36000 in decimal. The sending command is scaled by 0.01degree/LSB, that is, 36000*0.01=360°. The motor will move 360° in the positive direction with the output shaft relative to the current position.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0xA8` | `0x32` | `0x64` | `0x00` | `0xF4` | `0x01` | `0x2D` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA8` | `0x32` | `0x64` | `0x00` | `0xF4` | `0x01` | `0x2D` | `0x00` | CRC16L | CRC16H |

Description: `Data[1] = 0x32` is 50 in decimal, which means the motor temperature is 50 degrees at the moment.

The composite data of Data[2] and Data[3], `0x0064`, is 100 in decimal, and it is 100*0.01=1A when scaled down by 100 times, which means that the actual current of the motor is 1A.

The composite data of Data[4] and Data[5], `0x01F4`, is 500 in decimal, which means the motor output shaft speed is 500dps. There is a reduction ratio relationship between the motor output shaft speed and the motor speed. If the reduction ratio is 6, then the motor speed is 6 times higher than the output shaft speed.

The composite data of Data[6] and Data[7], `0x002D`, is 45 in decimal, which means that the motor output shaft moves 45 degrees in the positive direction relative to the zero position. The position of the motor output shaft is related to the number of lines of the motor encoder and the reduction ratio. For example, if the number of lines of the motor encoder is 16384 and the reduction ratio is 6, then 360 degrees of the motor output shaft corresponds to 16384*6 = 98304 pulses.

**Communication example — Example 2:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0xA8` | `0x00` | `0xF4` | `0x01` | `0x60` | `0x73` | `0xFF` | `0xFF` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA8` | `0x00` | `0xF4` | `0x01` | `0x60` | `0x73` | `0xFF` | `0xFF` | CRC16L | CRC16H |

Description: Data[2] and Data[3] form one (Data[2] is low, Data[3] is high) 16-bit data, `0x01F4`, which means the decimal 500dps motor output shaft speed. The drive will run the position loop at this speed as the maximum speed.

Data[4] to Data[7] form one (Data[4] is the lowest byte, Data[7] is the highest byte) 32-bit data, `0xFFFF7360`, which means -36000 in decimal. The sending command is scaled by 0.01degree/LSB, i.e. -36000*0.01=-360°. The motor will move -360° in the opposite direction relative to the current position with the output shaft.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0xA8` | `0x32` | `0x9C` | `0xFF` | `0x0C` | `0xFE` | `0xD3` | `0xFF` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA8` | `0x32` | `0x9C` | `0xFF` | `0x0C` | `0xFE` | `0xD3` | `0xFF` | CRC16L | CRC16H |

Description: `Data[1] = 0x32` is 50 in decimal, which means the motor temperature is 50 degrees at the moment.

The composite data of Data[2] and Data[3], `0xFF9C`, is -100 in decimal, and it is -100*0.01=-1A when scaled down by 100 times, which means that the actual current of the motor is -1A.

The composite data of Data[4] and Data[5], `0xFE0C`, is -500 in decimal, which means the motor output shaft speed is -500dps. There is a reduction ratio relationship between the motor output shaft speed and the motor speed. If the reduction ratio is 6, then the motor speed is 6 times higher than the output shaft speed.

The composite data of Data[6] and Data[7], `0xFFD3`, is -45 in decimal, which means that the motor output shaft moves in the opposite direction by -45 degrees relative to the zero position. The position of the motor output shaft is related to the number of lines of the motor encoder and the reduction ratio. For example, if the number of lines of the motor encoder is 16384 and the reduction ratio is 6, then 360 degrees of the motor output shaft corresponds to 16384*6 = 98304 pulses.

### 2.23. Force Control Position Closed-Loop Command (0xA9)

**Instruction description:** This command is a control command that can be executed when there are no faults in the motor. The host sends this command to control the position (multi-turn angle) of the motor. The control value `angleControl` is of type `int32_t`, corresponding to an actual position of 0.01degree/LSB. For example, 36000 represents 360°. The direction of motor rotation is determined by the difference between the target position and the current position. The control value `maxSpeed` limits the maximum rotational speed of the motor output shaft; it is of type `uint16_t`, corresponding to an actual speed of 1dps/LSB (degrees per second). The control value `maxTorque` limits the maximum torque of the motor output shaft; it is of type `uint8_t`, with a value range of 0 to 255, representing the percentage of the rated current, specifically 1%*rated current per LSB. If the given current exceeds the stall current, force control mode is not activated, and the maximum torque current of the motor is instead limited by the motor stall current value configured in the setup software.

> This command was added in V4.3 and refined in V4.4 with the addition of the `maxTorque` limiting field described above.

1. If the position loop acceleration is set to 0, the position loop enters direct tracking mode, where the target position is tracked directly via a PI controller. In this mode, `maxSpeed` defines the maximum velocity limit during operation (see Figure 2-1, Block Diagram of Position Tracking Mode with Velocity Limiting).
2. If the position loop acceleration is non-zero, the motor operates in a profiled motion mode. In this mode, the motor autonomously manages the acceleration and deceleration phases. The `maxSpeed` parameter caps the peak velocity during movement, while the acceleration rate is determined by the position loop's acceleration setting.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA9` |
| `DATA[1]` | Max torque | `DATA[1] = (uint8_t)(maxTorque)` |
| `DATA[2]` | Speed limit low byte | `DATA[2] = (uint8_t)(maxSpeed)` |
| `DATA[3]` | Speed limit high byte | `DATA[3] = (uint8_t)(maxSpeed>>8)` |
| `DATA[4]` | Position control low byte | `DATA[4] = (uint8_t)(angleControl)` |
| `DATA[5]` | Position control | `DATA[5] = (uint8_t)(angleControl>>8)` |
| `DATA[6]` | Position control | `DATA[6] = (uint8_t)(angleControl>>16)` |
| `DATA[7]` | Position control high byte | `DATA[7] = (uint8_t)(angleControl>>24)` |

**Reply data field:** After receiving the command, the motor replies to the host. The data frame includes the following parameters:

1. Motor temperature `temperature` (type `int8_t`, 1°C/LSB).
2. Motor torque current value `iq` (type `int16_t`, 0.01A/LSB).
3. Motor output shaft speed `speed` (type `int16_t`, 1dps/LSB).
4. Motor output shaft angle (type `int16_t`, 1degree/LSB, maximum range ±32767 degrees).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA9` |
| `DATA[1]` | Motor temperature | `DATA[1] = (uint8_t)(temperature)` |
| `DATA[2]` | Torque current low byte | `DATA[2] = (uint8_t)(iq)` |
| `DATA[3]` | Torque current high byte | `DATA[3] = (uint8_t)(iq>>8)` |
| `DATA[4]` | Motor speed low byte | `DATA[4] = (uint8_t)(speed)` |
| `DATA[5]` | Motor speed high byte | `DATA[5] = (uint8_t)(speed>>8)` |
| `DATA[6]` | Motor angle low byte | `DATA[6] = (uint8_t)(degree)` |
| `DATA[7]` | Motor angle high byte | `DATA[7] = (uint8_t)(degree>>8)` |

**Communication example — Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0xA9` | `0x3C` | `0xF4` | `0x01` | `0xA0` | `0x8C` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA9` | `0x3C` | `0xF4` | `0x01` | `0xA0` | `0x8C` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[1]` is an 8-bit data value `0x3C`, which represents 60*1%*rated current in decimal. Data[2] and Data[3] form a 16-bit data value (Data[2] as the low byte and Data[3] as the high byte), `0x01F4`, which represents 500dps (degrees per second) in decimal for the motor output shaft speed. The drive will operate with a maximum torque of 60% rated torque and a maximum speed of 500dps in the position loop.

Data[4] to Data[7] form a 32-bit data value (Data[4] as the lowest byte and Data[7] as the highest byte), `0x00008CA0`, which represents 36000 in decimal. The command is scaled down by a factor of 100 according to 0.01degree/LSB, i.e., 36000*0.01 = 360°. The motor will move the output shaft positively by 360° relative to the zero position.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0xA9` | `0x32` | `0x64` | `0x00` | `0xF4` | `0x01` | `0x2D` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA9` | `0x32` | `0x64` | `0x00` | `0xF4` | `0x01` | `0x2D` | `0x00` | CRC16L | CRC16H |

Description: `Data[1] = 0x32` is 50 in decimal, indicating that the current motor temperature is 50°C.

Data[2] and Data[3] form the data `0x0064`, which is 100 in decimal. According to the scaling factor of 100 times, this translates to 100*0.01 = 1A. Therefore, it represents that the actual current of the motor at this moment is 1A.

Data[4] and Data[5] form the data `0x01F4`, which is 500 in decimal, representing the motor output shaft speed as 500dps (degrees per second). There is a gear ratio relationship between the motor output shaft speed and the motor speed. If the gear ratio is 6, then the motor speed is 6 times higher than the output shaft speed.

Data[6] and Data[7] form the data `0x002D`, which is 45 in decimal, indicating that the motor output shaft has moved positively by 45 degrees relative to the zero position. The position of the motor output shaft is related to the motor encoder lines and the gear ratio. For example, if the motor encoder has 16384 lines and the gear ratio is 6, then 360 degrees of the motor output shaft corresponds to 16384*6 = 98304 pulses.

**Communication example — Example 2:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0xA9` | `0x3C` | `0xF4` | `0x01` | `0x60` | `0x73` | `0xFF` | `0xFF` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA9` | `0x3C` | `0xF4` | `0x01` | `0x60` | `0x73` | `0xFF` | `0xFF` | CRC16L | CRC16H |

Description: `Data[1]` is an 8-bit data value `0x3C`, which represents 60*1%*rated current in decimal. Data[2] and Data[3] form a 16-bit data value (Data[2] as the low byte and Data[3] as the high byte), `0x01F4`, which represents 500dps (degrees per second) in decimal for the motor output shaft speed. The drive will operate with a maximum torque of 60%*rated torque and a maximum speed of 500dps in the position loop.

Data[4] to Data[7] form a 32-bit data value (Data[4] as the lowest byte and Data[7] as the highest byte), `0xFFFF7360`, which represents -36000 in decimal. The command is scaled down by a factor of 100 according to 0.01degree/LSB, i.e., -36000*0.01 = -360°. The motor will move the output shaft negatively by -360° relative to the zero position.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0xA9` | `0x32` | `0x9C` | `0xFF` | `0x0C` | `0xFE` | `0xD3` | `0xFF` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0xA9` | `0x32` | `0x9C` | `0xFF` | `0x0C` | `0xFE` | `0xD3` | `0xFF` | CRC16L | CRC16H |

Description: `Data[1] = 0x32` is 50 in decimal, indicating that the motor temperature at this moment is 50 degrees.

Data[2] and Data[3] form the data `0xFF9C`, which is -100 in decimal. After scaling down by a factor of 100, it becomes -100*0.01 = -1A, representing that the actual current of the motor at present is -1A.

Data[4] and Data[5] form the data `0xFE0C`, which is -500 in decimal, indicating that the motor output shaft speed is -500dps (degrees per second). There is a gear ratio relationship between the motor output shaft speed and the motor speed. If the gear ratio is 6, then the motor speed is 6 times higher than the output shaft speed.

Data[6] and Data[7] form the data `0xFFD3`, which is -45 in decimal, indicating that the motor output shaft has moved -45 degrees in the reverse direction relative to the zero position. The position of the motor output shaft is related to the motor encoder lines and the gear ratio. For example, if the motor encoder lines are 16384 and the gear ratio is 6, then 360 degrees of the motor output shaft corresponds to 16384*6 = 98304 pulses.

### 2.24. SF Command (0x72, Position Control with Speed Feedforward)

**Instruction description:** This is a motion control command that can be executed as long as the motor is in a fault-free state. The host controller issues this command to control the motor's position (multi-turn angle). The control value `angleControl` (`int32_t`) specifies the target position with a resolution of 0.01°/LSB (e.g., a value of 36,000 represents 360°); the rotation direction is determined by the difference between the target and current positions. The `maxSpeed` parameter (`uint16_t`) limits the peak velocity of the motor output shaft with a resolution of 1 dps/LSB. The `Feedforward Speed` value (`int8_t`) sets the feedforward speed for the output shaft, ranging from -128 to 127, where each unit represents 1% of rated speed per LSB.

1. When the position-loop acceleration is set to 0, the system enters **Direct Tracking Mode**, where the target position is tracked directly via a PI controller. In this mode, `maxSpeed` defines the maximum velocity limit during operation (see Figure 2-1: Block Diagram of Position Tracking Mode with Velocity Limiting). The `Feedforward Speed` control value only takes effect in this tracking mode.
2. When the position-loop acceleration is non-zero, the system operates in **Profiled Motion Mode**, where the motor autonomously manages the acceleration/deceleration ramps. `maxSpeed` defines the peak velocity limit for the motion, while the acceleration rate is determined by the value configured in the position loop. In Profiled Motion Mode the `Feedforward Speed` control value does not take effect, and the command behaves exactly the same as the Absolute Position Closed-Loop Control Command (0xA4).

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x72` |
| `DATA[1]` | Feedforward Speed | `DATA[1] = (uint8_t)(Feedforward Speed)` |
| `DATA[2]` | Max Speed (low byte) | `DATA[2] = (uint8_t)(maxSpeed)` |
| `DATA[3]` | Max Speed (high byte) | `DATA[3] = (uint8_t)(maxSpeed>>8)` |
| `DATA[4]` | Position Control (byte 1 / LSB) | `DATA[4] = (uint8_t)(angleControl)` |
| `DATA[5]` | Position Control (byte 2) | `DATA[5] = (uint8_t)(angleControl>>8)` |
| `DATA[6]` | Position Control (byte 3) | `DATA[6] = (uint8_t)(angleControl>>16)` |
| `DATA[7]` | Position Control (byte 4 / MSB) | `DATA[7] = (uint8_t)(angleControl>>24)` |

> Erratum: the source manual prints the `DATA[1]` formula as `DATA[2] = (uint8_t)(Feedforward Speed)`; corrected above to `DATA[1] =` to match the field it defines.

**Reply data field:** After receiving the command, the motor replies to the host. The data frame contains:
1. Motor temperature (`int8_t`, 1°C/LSB);
2. Motor torque current value `iq` (`int16_t`, 0.01 A/LSB);
3. Motor output shaft speed `speed` (`int16_t`, 1 dps/LSB);
4. Motor output shaft angle (`int16_t`, 1 degree/LSB, maximum range ±32767 degrees).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x72` |
| `DATA[1]` | Motor temperature | `DATA[1] = (uint8_t)(temperature)` |
| `DATA[2]` | Torque current low byte | `DATA[2] = (uint8_t)(iq)` |
| `DATA[3]` | Torque current high byte | `DATA[3] = (uint8_t)(iq>>8)` |
| `DATA[4]` | Motor speed low byte | `DATA[4] = (uint8_t)(speed)` |
| `DATA[5]` | Motor speed high byte | `DATA[5] = (uint8_t)(speed>>8)` |
| `DATA[6]` | Motor angle low byte | `DATA[6] = (uint8_t)(degree)` |
| `DATA[7]` | Motor angle high byte | `DATA[7] = (uint8_t)(degree>>8)` |

**Communication example — Example 1:**

Send: CAN `0x141 0x73 0x3C 0xF4 0x01 0xA0 0x8C 0x00 0x00`.

> Erratum: the source manual prints `Data[0] = 0x73` (the TF command byte) in this SF worked example instead of `0x72`. Both SF worked examples reuse byte-for-byte the same payloads that appear under TF (section 2.25) — this looks like a copy-paste artifact in the manual itself rather than a real 0x72 frame. Treat the field-layout tables above as authoritative and verify this exact frame against hardware before using it as a test fixture.

Description: `Data[1]` is an 8-bit value `0x3C`, representing decimal 60 × 1% × rated speed. `Data[2]`/`Data[3]` form a 16-bit value (`Data[2]` low, `Data[3]` high) of `0x01F4`, decimal 500 dps output-shaft speed — the drive runs the position loop with 60% of rated speed as feedforward speed and 500 dps as the maximum speed. `Data[4]`–`Data[7]` form a 32-bit value (`Data[4]` lowest, `Data[7]` highest) of `0x00008CA0`, decimal 36000; scaled by 0.01 degree/LSB this is 36000 × 0.01 = 360°. The motor moves forward 360° relative to the output shaft's zero position.

Reply: CAN `0x241 0x73 0x32 0x64 0x00 0xF4 0x01 0x2D 0x00`.

Description: `Data[1] = 0x32` (decimal 50) — motor temperature is 50°C. `Data[2]`/`Data[3]` synthesize `0x0064` (decimal 100); scaled by 0.01 A/LSB this is 1 A actual motor current. `Data[4]`/`Data[5]` synthesize `0x01F4` (decimal 500) — motor output shaft speed of 500 dps (a reduction-ratio relationship exists between output-shaft speed and motor speed; e.g. with a 6:1 ratio, motor speed is 6× the output-shaft speed). `Data[6]`/`Data[7]` synthesize `0x002D` (decimal 45) — the output shaft has moved forward 45° relative to the zero position (output-shaft position relates to encoder line count and reduction ratio; e.g. a 16384-line encoder with a 6:1 ratio gives 16384 × 6 = 98,304 pulses per 360° of output-shaft rotation).

**Communication example — Example 2:**

Send: CAN `0x141 0x73 0x3C 0xF4 0x01 0x60 0x73 0xFF 0xFF`.

Description: `Data[1] = 0x3C`, decimal 60 × 1% × rated speed. `Data[2]`/`Data[3]` form `0x01F4`, decimal 500 dps.

> Erratum: the manual's own description text for this example reads "the drive runs the position loop with 60% of rated *torque* as feedforward *torque* and 500 dps as maximum speed" — reusing TF's wording verbatim. For an SF (0x72) frame this should read "60% of rated **speed** as feedforward **speed**," consistent with Example 1 above and with the command's own field definition.

`Data[4]`–`Data[7]` form `0xFFFF7360`, decimal -36000; scaled by 0.01 degree/LSB this is -36000 × 0.01 = -360°. The motor moves backward 360° relative to the output shaft's zero position.

Reply: CAN `0x241 0x73 0x32 0x9C 0xFF 0x0C 0xFE 0xD3 0xFF`.

Description: `Data[1] = 0x32` (50) — motor temperature 50°C. `Data[2]`/`Data[3]` synthesize `0xFF9C` (decimal -100); scaled by 0.01 A/LSB this is -1 A actual current. `Data[4]`/`Data[5]` synthesize `0xFE0C` (decimal -500) — output shaft speed of -500 dps. `Data[6]`/`Data[7]` synthesize `0xFFD3` (decimal -45) — the output shaft has moved backward 45° relative to the zero position.

### 2.25. TF Command (0x73, Position Control with Torque Feedforward)

**Instruction description:** This is a motion control command that can be executed as long as the motor is in a fault-free state. The host controller issues this command to control the motor's position (multi-turn angle). The control value `angleControl` (`int32_t`) specifies the target position with a resolution of 0.01°/LSB (e.g., a value of 36,000 represents 360°); the rotation direction is determined by the difference between the target and current positions. The `maxSpeed` parameter (`uint16_t`) limits the peak velocity of the motor output shaft with a resolution of 1 dps/LSB. The `Feedforward Torque` value (`int8_t`) sets the feedforward torque for the output shaft, ranging from -128 to 127, where each unit represents 1% of rated current per LSB.

1. When the position-loop acceleration is set to 0, the system enters **Direct Tracking Mode**, where the target position is tracked directly via a PI controller. In this mode, `maxSpeed` defines the maximum velocity limit during operation (see Figure 2-1: Block Diagram of Position Tracking Mode with Velocity Limiting).
2. When the position-loop acceleration is non-zero, the system operates in **Profiled Motion Mode**, where the motor autonomously manages the acceleration/deceleration ramps. `maxSpeed` defines the peak velocity limit for the motion, while the acceleration rate is determined by the value configured in the position loop.

> Note: unlike the SF command's description (2.24.1), the manual's TF description does not explicitly restate that `Feedforward Torque` only takes effect in Direct Tracking Mode, nor that Profiled Motion Mode behaves identically to the Absolute Position Closed-Loop Control Command (0xA4). Given the two commands otherwise share identical wording, the same behavior likely applies to TF, but this is not stated verbatim in the source for this command.

**Confirms the changelog claim:** the `Feedforward Torque` control value is `int8_t`, range -128 to 127 (1% of rated current per LSB) — this matches the V4.4 changelog note that this field's data type was changed to `int8_t`.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x73` |
| `DATA[1]` | Feedforward Torque | `DATA[1] = (uint8_t)(Feedforward Torque)` |
| `DATA[2]` | Max Speed (low byte) | `DATA[2] = (uint8_t)(maxSpeed)` |
| `DATA[3]` | Max Speed (high byte) | `DATA[3] = (uint8_t)(maxSpeed>>8)` |
| `DATA[4]` | Position Control (byte 1 / LSB) | `DATA[4] = (uint8_t)(angleControl)` |
| `DATA[5]` | Position Control (byte 2) | `DATA[5] = (uint8_t)(angleControl>>8)` |
| `DATA[6]` | Position Control (byte 3) | `DATA[6] = (uint8_t)(angleControl>>16)` |
| `DATA[7]` | Position Control (byte 4 / MSB) | `DATA[7] = (uint8_t)(angleControl>>24)` |

> Erratum: the source manual prints the `DATA[1]` formula as `DATA[2] = (uint8_t)(Feedforward Torque)`; corrected above to `DATA[1] =` to match the field it defines (same erratum as the SF command).

**Reply data field:** After receiving the command, the motor replies to the host. The data frame contains:
1. Motor temperature (`int8_t`, 1°C/LSB);
2. Motor torque current value `iq` (`int16_t`, 0.01 A/LSB);
3. Motor output shaft speed `speed` (`int16_t`, 1 dps/LSB);
4. Motor output shaft angle (`int16_t`, 1 degree/LSB, maximum range ±32767 degrees).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x73` |
| `DATA[1]` | Motor temperature | `DATA[1] = (uint8_t)(temperature)` |
| `DATA[2]` | Torque current low byte | `DATA[2] = (uint8_t)(iq)` |
| `DATA[3]` | Torque current high byte | `DATA[3] = (uint8_t)(iq>>8)` |
| `DATA[4]` | Motor speed low byte | `DATA[4] = (uint8_t)(speed)` |
| `DATA[5]` | Motor speed high byte | `DATA[5] = (uint8_t)(speed>>8)` |
| `DATA[6]` | Motor angle low byte | `DATA[6] = (uint8_t)(degree)` |
| `DATA[7]` | Motor angle high byte | `DATA[7] = (uint8_t)(degree>>8)` |

**Communication example — Example 1:**

Send: CAN `0x141 0x73 0x3C 0xF4 0x01 0xA0 0x8C 0x00 0x00`.

Description: `Data[1]` is an 8-bit value `0x3C`, representing decimal 60 × 1% × rated current. `Data[2]`/`Data[3]` form a 16-bit value (`Data[2]` low, `Data[3]` high) of `0x01F4`, decimal 500 dps output-shaft speed — the drive runs the position loop with 60% of rated torque as feedforward torque and 500 dps as the maximum speed. `Data[4]`–`Data[7]` form a 32-bit value (`Data[4]` lowest, `Data[7]` highest) of `0x00008CA0`, decimal 36000; scaled by 0.01 degree/LSB this is 36000 × 0.01 = 360°. The motor moves forward 360° relative to the output shaft's zero position.

Reply: CAN `0x241 0x73 0x32 0x64 0x00 0xF4 0x01 0x2D 0x00`.

Description: `Data[1] = 0x32` (decimal 50) — motor temperature is 50°C. `Data[2]`/`Data[3]` synthesize `0x0064` (decimal 100); scaled by 0.01 A/LSB this is 1 A actual motor current. `Data[4]`/`Data[5]` synthesize `0x01F4` (decimal 500) — motor output shaft speed of 500 dps. `Data[6]`/`Data[7]` synthesize `0x002D` (decimal 45) — the output shaft has moved forward 45° relative to the zero position.

**Communication example — Example 2:**

Send: CAN `0x141 0x73 0x3C 0xF4 0x01 0x60 0x73 0xFF 0xFF`.

Description: `Data[1] = 0x3C`, decimal 60 × 1% × rated current. `Data[2]`/`Data[3]` form `0x01F4`, decimal 500 dps — the drive runs the position loop with 60% of rated torque as feedforward torque and 500 dps as maximum speed. `Data[4]`–`Data[7]` form `0xFFFF7360`, decimal -36000; scaled by 0.01 degree/LSB this is -36000 × 0.01 = -360°. The motor moves backward 360° relative to the output shaft's zero position.

Reply: CAN `0x241 0x73 0x32 0x9C 0xFF 0x0C 0xFE 0xD3 0xFF`.

Description: `Data[1] = 0x32` (50) — motor temperature 50°C. `Data[2]`/`Data[3]` synthesize `0xFF9C` (decimal -100); scaled by 0.01 A/LSB this is -1 A actual current. `Data[4]`/`Data[5]` synthesize `0xFE0C` (decimal -500) — output shaft speed of -500 dps. `Data[6]`/`Data[7]` synthesize `0xFFD3` (decimal -45) — the output shaft has moved backward 45° relative to the zero position.

### 2.26. System Operating Mode Acquisition (0x70)

**Instruction description:** Reads the current motor running mode.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x70` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor replies to the host after receiving the command; the reply data contains the parameter `runmode` (`uint8_t`), the running state. The motor operation mode has 3 states: `0x01` current-loop mode, `0x02` speed-loop mode, `0x03` position-loop mode.

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x70` |
| `DATA[1]`–`DATA[6]` | NULL | `0x00` |
| `DATA[7]` | Motor operating mode | `DATA[7] = (uint8_t)(runmode)` |

**Communication example — Example 1:**

Send: CAN `0x141 0x70 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Description: This command reads the current motor running mode.

Reply: CAN `0x241 0x70 0x00 0x00 0x00 0x00 0x00 0x00 0x03`.

Description: `Data[7] = 0x03` — per the reply-frame definition, the current system is in position-loop mode.

### 2.27. System Reset Command (0x76)

**Instruction description:** This command is used to reset the system program.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x76` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor resets after receiving the command and does not send a reply.

**Communication example — Example 1:**

Send: CAN `0x141 0x76 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Description: After sending the command, the system resets and the program runs again.

### 2.28. System Brake Release Command (0x77)

**Instruction description:** This command is used to open the system brake. The system releases the holding brake, and the motor is placed in a movable state, unrestricted by the holding brake.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x77` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor replies to the host after receiving the command, and the reply frame data is identical to the command sent by the host.

> Note: the source manual does not provide a worked communication example for this command.

### 2.29. System Brake Lock Command (0x78)

**Instruction description:** This command is used to close the system holding brake. The holding brake locks the motor, and the motor can no longer run. The holding brake is also in this state after the system is powered off.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x78` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor replies to the host after receiving the command, and the reply frame data is identical to the command sent by the host.

> Note: the source manual does not provide a worked communication example for this command.

### 2.30. System Runtime Read Command (0xB1)

**Instruction description:** This command is used to obtain the system running time, in ms.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB1` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor replies to the host after receiving the command; the reply data contains the system running time `SysRunTime` (`uint32_t`, unit ms).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB1` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | SysRunTime low byte 1 | `DATA[4] = (uint8_t)(SysRunTime)` |
| `DATA[5]` | SysRunTime byte 2 | `DATA[5] = (uint8_t)(SysRunTime>>8)` |
| `DATA[6]` | SysRunTime byte 3 | `DATA[6] = (uint8_t)(SysRunTime>>16)` |
| `DATA[7]` | SysRunTime byte 4 | `DATA[7] = (uint8_t)(SysRunTime>>24)` |

> Note: the source manual labels the `DATA[1]`–`DATA[3]` rows `DATA[0]` (a table-layout artifact); corrected above.

**Communication example — Example 1:**

Send: CAN `0x141 0xB1 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Description: This command reads the running time of the current system.

Reply: CAN `0x241 0xB1 0x00 0x00 0x00 0x00 0x00 0x00 0x10`.

Description: `Data[4]`–`Data[7]` (`Data[4]` low, `Data[7]` high) = `0x10000000`, decimal 268435456 — the system has run for 268,435,456 ms since restarting or resetting, about 74 hours.

### 2.31. System Software Version Date Read Command (0xB2)

**Instruction description:** This command is used to get the update date of the system software version.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB2` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor replies to the host after receiving the command; the reply data contains the latest system software version date `VersionDate` (`uint32_t`), formatted as year-month-day, e.g. `20211126`.

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB2` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | VersionDate low byte 1 | `DATA[4] = (uint8_t)(VersionDate)` |
| `DATA[5]` | VersionDate byte 2 | `DATA[5] = (uint8_t)(VersionDate>>8)` |
| `DATA[6]` | VersionDate byte 3 | `DATA[6] = (uint8_t)(VersionDate>>16)` |
| `DATA[7]` | VersionDate byte 4 | `DATA[7] = (uint8_t)(VersionDate>>24)` |

> Note: the source manual labels the `DATA[1]`–`DATA[3]` rows `DATA[0]` (a table-layout artifact); corrected above.

**Communication example — Example 1:**

Send: CAN `0x141 0xB2 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Description: This command reads the current software version date.

Reply: CAN `0x241 0xB2 0x00 0x00 0x00 0x2E 0x89 0x34 0x01`.

Description: `Data[4]`–`Data[7]` (`Data[4]` low, `Data[7]` high) = `0x0134892E`, decimal 20220206 — the software version date is February 6, 2022.

### 2.32. Communication Interruption Protection Time Setting Command (0xB3)

**Instruction description:** This command is used to set the communication interruption protection time, in ms. If communication is interrupted for longer than the set time, the drive cuts off the output and locks the brake. To run again, stable and continuous communication must first be re-established. Writing 0 disables the communication interruption protection function. Take care to avoid writing this parameter right when the motor has just started or is in motion.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB3` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | CanRecvTime_MS low byte 1 | `DATA[4] = (uint8_t)(CanRecvTime_MS)` |
| `DATA[5]` | CanRecvTime_MS byte 2 | `DATA[5] = (uint8_t)(CanRecvTime_MS>>8)` |
| `DATA[6]` | CanRecvTime_MS byte 3 | `DATA[6] = (uint8_t)(CanRecvTime_MS>>16)` |
| `DATA[7]` | CanRecvTime_MS byte 4 | `DATA[7] = (uint8_t)(CanRecvTime_MS>>24)` |

**Reply data field:** The motor replies to the host after receiving the command, and the reply frame data is identical to the command sent by the host.

**Communication example — Example 1:**

Send: CAN `0x141 0xB3 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Description: All data values are 0, meaning the communication interruption protection function is not enabled. If communication is interrupted, the motor continues to execute the current command.

Reply: CAN `0x241 0xB3 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Description: The frame data is identical to the command sent by the host.

**Communication example — Example 2:**

Send: CAN `0x141 0xB3 0x00 0x00 0x00 0xE8 0x03 0x00 0x00`.

Description: `Data[4]`–`Data[7]` (`Data[4]` low, `Data[7]` high) form `0x000003E8`, decimal 1000 ms — this sets the communication interruption protection time to 1000 ms, which is stored in ROM and retained after power loss. If the communication interval then exceeds 1000 ms, the interruption protection triggers, cutting the output and locking the brake; normal operation resumes once the communication interval is restored to within 1000 ms.

Reply: CAN `0x241 0xB3 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Description: The manual states "the frame data is identical to the command sent by the host," but the reply bytes shown are all-zero even though the sent command had `0xE8 0x03` in `Data[4]`/`Data[5]`. This looks like the Example-2 reply row was copy-pasted from Example 1 without updating the echoed bytes — flagged here since it contradicts the stated echo behavior.

### 2.33. Communication Baud Rate Setting Command (0xB4)

**Instruction description:** This command sets the communication baud rate of the CAN and RS485 buses. The parameter is saved in ROM after being set, persists after power-off, and the drive runs at the modified baud rate the next time it is powered on. The parameter can only be successfully saved to ROM while the motor is disabled.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB4` |
| `DATA[1]`–`DATA[6]` | NULL | `0x00` |
| `DATA[7]` | baudrate | `DATA[7] = (uint8_t)(baudrate)` |

**Function index description:**

| Index | baudrate |
| --- | --- |
| `0x00` | CAN baud rate: 1M; RS485 baud rate: 115200 |
| `0x01` | CAN baud rate: 500K; RS485 baud rate: 5M |
| `0x02` | CAN baud rate: 250K |

> Note: the source table lists only "CAN baud rate: 250K" for index `0x02`, with no RS485 rate given. This conflicts with Example 3 below, whose description states index `0x02` sets the RS485 baud rate to 1 Mbps and makes CAN "invalid." The two statements cannot both be correct as printed; flagged for verification against hardware/firmware notes rather than silently reconciled.

**Reply data field:** Since the communication baud rate has been changed, the reply command content is arbitrary/random and does not need to be processed.

**Communication example — Example 1:**

Send: CAN `0x141 0xB4 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Description: `Data[7] = 0`, meaning the RS485 baud rate changes to 115200 bps and the CAN baud rate changes to 1 Mbps.

**Communication example — Example 2:**

Send: CAN `0x141 0xB4 0x00 0x00 0x00 0x00 0x00 0x00 0x01`.

Description: `Data[7] = 1`, meaning the RS485 baud rate changes to 5 Mbps and the CAN baud rate changes to 500 Kbps.

**Communication example — Example 3:**

Send: CAN `0x141 0xB4 0x00 0x00 0x00 0x00 0x00 0x00 0x02`.

Description: `Data[7] = 2`, meaning the RS485 baud rate changes to 1 Mbps, and CAN is invalid (not applicable).

### 2.34. Motor Model Reading Command (0xB5)

**Instruction description:** This command is used to read the motor model. The motor model contains up to 15 characters. Using an index, 5 characters can be read at a time, so three messages must be sent to retrieve the full model string.

> Note on the changelog's "more detailed usage instructions" for 0xB5: within this slice of the manual, the added detail is limited to the chunked-read mechanism above (index selects which group of 5 characters is returned) plus the worked example decoding "X8S2V" below. No mention of NVM/persistence behavior or of a minimum firmware version requirement was found anywhere in the 0xB5 section.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB5` |
| `DATA[1]` | Flag bit | Fixed to `0x01` |
| `DATA[2]` | Index | `DATA[2] = (uint8_t)(index)` |
| `DATA[3]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB5` |
| `DATA[1]` | Flag bit | Fixed to `0x01` |
| `DATA[2]` | Index | `DATA[2] = (uint8_t)(index)` |
| `DATA[3]` | Character 1 | `Char1` |
| `DATA[4]` | Character 2 | `Char2` |
| `DATA[5]` | Character 3 | `Char3` |
| `DATA[6]` | Character 4 | `Char4` |
| `DATA[7]` | Character 5 | `Char5` |

**Index description:**

| Index value | Function description |
| --- | --- |
| `0x01` | Read the first to fifth characters of the motor model. |
| `0x02` | Read the sixth to tenth characters of the motor model. |
| `0x03` | Read the eleventh to fifteenth characters of the motor model. |

**Communication example — Example 1:**

Send: CAN `0x141 0xB5 0x01 0x01 0x00 0x00 0x00 0x00 0x00`.

Description: Sends a command to read the first to fifth characters of the motor model.

Reply: CAN `0x241 0xB5 0x01 0x01 0x58 0x38 0x53 0x32 0x56`.

Description: This reply returns 5 ASCII codes. Decoded via the ASCII table, the first to fifth characters of the motor model are: `X8S2V`. Repeating this process three times (indices `0x01`, `0x02`, `0x03`) yields the complete motor model string.

### 2.35. Active Reply Function Command (0xB6)

**Instruction description:** This command selects specified commands for active, timed replies; more than one command can be specified, and different commands are replied to cyclically/alternately according to the set interval. Once an active-reply command is configured, the motor no longer replies to that command when it is received in the normal request/response fashion. This function is valid only for the CAN version — the RS485 version does not support it.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB6` |
| `DATA[1]` | Command to configure for active reply | Supported reply commands: `0x60`, `0x61`, `0x62`, `0x92`, `0x9A`, `0x9C`, `0x9D`, `0x9E` |
| `DATA[2]` | Unsolicited reply enable bit | `0`: disable active reply for this command; `1`: enable active reply for this command |
| `DATA[3]` | Reply interval, low byte | Reply interval time, unit 10 ms/LSB. When multiple commands are configured, they are replied to in an alternating cyclic loop. |
| `DATA[4]` | Reply interval, high byte | (see `DATA[3]`) |
| `DATA[5]`–`DATA[7]` | NULL | — |

**Reply data field:** After this function is enabled, no data is returned in direct reply; instead, the motor actively transmits the selected command's content at the configured interval.

**Communication example — Example 1:**

Send: CAN `0x141 0xB6 0x60 0x01 0x01 0x00 0x00 0x00 0x00`.

Description: Enables active reply for command `0x60` with an interval of 10 ms (`DATA[3] = 0x01` × 10 ms/LSB). After this command is sent, the motor no longer replies when command `0x60` is received normally; instead it cyclically transmits the `0x60` reply content every 10 ms.

> Erratum: the manual's own English translation states the interval is "20ms" in the first sentence of this description, then "10ms" in the second — self-contradictory. The Chinese source and the `DATA[3] = 0x01` byte value both agree on 10 ms, which is used above.

### 2.36. Function Control Command (0x20)

**Instruction description:** This command is used for a number of specific functions; it is a compound function instruction that can contain multiple function-control operations, selected by index. Take care to avoid writing parameters right when the motor has just started or is in motion. The parameter can only be successfully saved to ROM while the motor is disabled.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x20` |
| `DATA[1]` | Function index | `DATA[1] = (uint8_t)(index)` |
| `DATA[2]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Input parameter low byte 1 | `DATA[4] = (uint8_t)(Value)` |
| `DATA[5]` | Input parameter byte 2 | `DATA[5] = (uint8_t)(Value>>8)` |
| `DATA[6]` | Input parameter byte 3 | `DATA[6] = (uint8_t)(Value>>16)` |
| `DATA[7]` | Input parameter byte 4 | `DATA[7] = (uint8_t)(Value>>24)` |

**Reply data field:** The motor replies to the host after receiving the command, and the reply frame data is identical to the command sent by the host.

**Function index description:**

| Index value | Command name | Function description |
| --- | --- | --- |
| `0x01` | Clear multi-turn value | Clears the motor's multi-turn value, updates the zero point, and saves it. Takes effect after restart. |
| `0x02` | CANID filter enable | `Value = 1` enables the CANID filter, which improves motor send/receive efficiency on the CAN bus. `Value = 0` disables the CANID filter, which is required when the multi-motor control commands `0x280`/`0x300` are needed. This value is saved in FLASH; the written value is retained after power-off. |
| `0x03` | Error status transmission enable | `Value = 1` enables this function: after the motor enters an error state, it actively sends status command `0x9A` to the bus with a 100 ms sending period, and stops sending once the error status clears. `Value = 0` disables this function. |
| `0x04` | Multi-turn value save-on-power-off enable | `Value = 1` enables this function: the motor saves its current multi-turn value before losing power. `Value = 0` disables this function; the system then defaults to single-turn mode. Takes effect after restart. |
| `0x05` | Set CANID | The value is the CANID number to change to; it is saved to ROM and takes effect after reboot. |
| `0x06` | Set the maximum positive angle for position operation mode | The value is the maximum positive angle for position operation mode; it is saved to ROM and takes effect immediately. |
| `0x07` | Set the maximum negative angle for position operation mode | The value is the maximum negative angle for position operation mode; it is saved to ROM and takes effect immediately. |
| `0x08` | Error State Auto-Recovery Enable | `Value = 1` enables this function: the three error states of overvoltage, undervoltage, and overspeed will automatically recover once their recovery conditions are met. `Value = 0` disables this function: those three error states will not automatically recover. Other error states are not controlled by this function. |

> `0x08` (Error State Auto-Recovery Enable) is the function index newly added in V4.4 per the changelog.

**Communication example — Example 1:**

Send: CAN `0x141 0x20 0x01 0x00 0x00 0x00 0x00 0x00 0x00`.

Description: `Data[1] = 0x01` — per the function index table, this selects "clear multi-turn value."

Reply: CAN `0x241 0x20 0x01 0x00 0x00 0x00 0x00 0x00 0x00`.

Description: The frame data is identical to the command sent by the host.

**Communication example — Example 2:**

Send: CAN `0x141 0x20 0x02 0x00 0x00 0x01 0x00 0x00 0x00`.

Description: `Data[1] = 0x02` — per the function index table, this selects "CANID filter enable," with `Value = 1` (from `Data[4] = 0x01`). Note that once enabled, the `0x280` multi-motor command cannot be used; the CANID filter must be disabled again before using the `0x280` command.

> Erratum: the source manual's description text says "Data[1] = 0x01" here, but the example's own send table shows `Data[1] = 0x02`; corrected above to match the actual byte value in the frame.

Reply: CAN `0x241 0x20 0x02 0x00 0x00 0x01 0x00 0x00 0x00`.

Description: The frame data is identical to the command sent by the host.

## 3. CAN Multi-Motor Command (0x280 + Command)

### 3.1. Instruction Description

The ID number 0x280 means that multiple motors respond to the same command at the same time. The content and function of the command are the same as those of the corresponding single-motor command; see the single-motor command descriptions for details.

### 3.2. Communication Example

Suppose there are 4 motors on the CAN bus, with ID numbers 141, 142, 143, and 144 respectively.

**Example 1:**

Send command:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
|---|---|---|---|---|---|---|---|---|
| `0x280` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

Description: All 4 motors receive the 0x80 motor shutdown command at the same time (see §2.30 for details), and then all 4 motors immediately execute the motor shutdown command.

Reply command: all 4 motors reply at the same time, each with its own ID number as the reply ID. The reply order depends on each motor's respective delay on the bus.

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
|---|---|---|---|---|---|---|---|---|
| `0x241` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

Description: the motor with ID number 0x241 returns the corresponding command.

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
|---|---|---|---|---|---|---|---|---|
| `0x242` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

Description: the motor with ID number 0x242 returns the corresponding command.

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
|---|---|---|---|---|---|---|---|---|
| `0x243` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

Description: the motor with ID number 0x243 returns the corresponding command.

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
|---|---|---|---|---|---|---|---|---|
| `0x244` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

Description: the motor with ID number 0x244 returns the corresponding command.

**Example 2:**

Send command:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
|---|---|---|---|---|---|---|---|---|
| `0x280` | `0x60` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

Description: all 4 motors receive the 0x60 read multi-turn encoder position data command at the same time (see §2.21 for details), and then the 4 motors each reply with their own multi-turn encoder position data.

Reply command: all 4 motors reply at the same time, each with its own ID number as the reply ID. The reply order depends on each motor's respective delay on the bus.

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
|---|---|---|---|---|---|---|---|---|
| `0x241` | `0x60` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

Description: in the reply data of the motor with ID number 0x241, Data[4] through Data[7] form a 32-bit value (Data[4] is the lowest byte, Data[7] is the highest byte) of `0x00002710`, which is decimal 10000. This represents the motor's current multi-turn encoder value, relative to the multi-turn zero offset (initial position), of 10000 pulses.

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
|---|---|---|---|---|---|---|---|---|
| `0x242` | `0x60` | `0x00` | `0x00` | `0x00` | `0x20` | `0x4E` | `0x00` | `0x00` |

Description: in the reply data of the motor with ID number 0x242, Data[4] through Data[7] form a 32-bit value (Data[4] is the lowest byte, Data[7] is the highest byte) of `0x00004E20`, which is decimal 20000. This represents the motor's current multi-turn encoder value, relative to the multi-turn zero offset (initial position), of 20000 pulses.

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
|---|---|---|---|---|---|---|---|---|
| `0x243` | `0x60` | `0x00` | `0x00` | `0x00` | `0x30` | `0x75` | `0x00` | `0x00` |

Description: in the reply data of the motor with ID number 0x243, Data[4] through Data[7] form a 32-bit value (Data[4] is the lowest byte, Data[7] is the highest byte) of `0x00007530`, which is decimal 30000. This represents the motor's current multi-turn encoder value, relative to the multi-turn zero offset (initial position), of 30000 pulses.

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
|---|---|---|---|---|---|---|---|---|
| `0x244` | `0x60` | `0x00` | `0x00` | `0x00` | `0x40` | `0x9C` | `0x00` | `0x00` |

Description: in the response data of the motor with ID 0x244, Data[4] through Data[7] form a 32-bit value (Data[4] is the lowest byte, Data[7] is the highest byte) of `0x00009C40`, which represents decimal 40000. This indicates that the current multi-turn encoder value of the motor relative to the multi-turn zero offset (initial position) is 40,000 pulses.

## 4. Motion Mode Control Command_CAN (0x400 + ID)

### 4.1. Instruction Description

The command consists of 5 input parameters: p_des (desired position), v_des (desired velocity), t_ff (feedforward torque), kp (position deviation coefficient), and kd (velocity deviation coefficient).

Each parameter has a preset range:

- **p_des**: -12.566 to 12.566 rad. Data type is `uint16_t`, with a value range of 0 to 65535. Here 0 represents -12.566 and 65535 represents 12.566; all values between 0 and 65535 are mapped proportionally to the range -12.566 to 12.566.
- **v_des**: -45 to 45 rad/s. Data type is a 12-bit unsigned integer, with a value range of 0 to 4095. Here 0 represents -45 and 4095 represents 45; all values between 0 and 4095 are mapped proportionally to the range -45 to 45.
- **kp**: 0 to 1000. Data type is a 12-bit unsigned integer, with a value range of 0 to 4095. Here 0 represents 0 and 4095 represents 1000; all values between 0 and 4095 are mapped proportionally to the range 0 to 1000.
- **kd**: 0 to 5. Data type is a 12-bit unsigned integer, with a value range of 0 to 4095. Here 0 represents 0 and 4095 represents 5; all values between 0 and 4095 are mapped proportionally to the range 0 to 5.
- **t_ff**: -Motor Max Torque to +Motor Max Torque, in Nm (the Max Torque parameter can be viewed in the host software). Data type is a 12-bit unsigned integer, with a value range of 0 to 4095. Here 0 represents -Motor Max Torque and 4095 represents +Motor Max Torque; all values between 0 and 4095 are mapped proportionally to the range -Motor Max Torque to +Motor Max Torque.

> **Note on internal inconsistency in this document:** the worked example in §4.4 below computes kp using a range of 0–500 (not 0–1000 as stated in this paragraph's English text — the Chinese source text for this paragraph also says 0–500, so "1000" above appears to be a translation error), and computes kd using a range of 0–50 (not 0–5 as stated in this paragraph — matching the §8 V4.4 changelog entry that the kd range was widened from 0–5 to 0–50, apparently not carried through to this descriptive paragraph). The worked numeric example should be treated as authoritative for kp and kd ranges; see the flagged values in §4.4.

Function expression:

```
IqRef = [kp*(p_des - p_fb_actual_position) + kd*(v_des - v_fb_actual_speed) + t_ff] / KT_OUT (torque coefficient)
```

IqRef is the final output current given to the motor.

### 4.2. Send Data Field Definition (Big-endian byte order)

| Data field | Bits | Data combination | Data definition | Data range |
|---|---|---|---|---|
| DATA[0] | 7-0 | `p_des[15:8]` | p_des upper 8-bit data | 16-bit range |
| DATA[1] | 7-0 | `p_des[7:0]` | p_des lower 8-bit data | 16-bit range |
| DATA[2] | 7-0 | `v_des[11:4]` | v_des upper 8-bit data | 12-bit range |
| DATA[3] | bits 4-7 | `v_des[3:0]` | v_des lower 4-bit data | 12-bit range |
| DATA[3] | bits 0-3 | `kp[11:8]` | kp upper 4-bit data | 12-bit range |
| DATA[4] | 7-0 | `kp[7:0]` | kp lower 8-bit data | 12-bit range |
| DATA[5] | 7-0 | `kd[11:4]` | kd upper 8-bit data | 12-bit range |
| DATA[6] | bits 4-7 | `kd[3:0]` | kd lower 4-bit data | 12-bit range |
| DATA[6] | bits 0-3 | `t_ff[11:8]` | t_ff upper 4-bit data | 12-bit range |
| DATA[7] | 7-0 | `t_ff[7:0]` | t_ff lower 8-bit data | 12-bit range |

### 4.3. Reply Data Field Definition (Big-endian byte order)

| Data field | Bits | Data combination | Data definition | Data range |
|---|---|---|---|---|
| DATA[0] | bits 0-7 | `CANID[7:0]` | device CAN address number | 8-bit range |
| DATA[1] | 7-0 | `p[15:8]` | current position p, upper 8-bit data | 16-bit range |
| DATA[2] | 7-0 | `p[7:0]` | current position p, lower 8-bit data | 16-bit range |
| DATA[3] | 7-0 | `v[11:4]` | current velocity v, upper 8-bit data | 12-bit range |
| DATA[4] | bits 4-7 | `v[3:0]` | current velocity v, lower 4-bit data | 12-bit range |
| DATA[4] | bits 0-3 | `t[11:8]` | current torque t, upper 4-bit data | 12-bit range |
| DATA[5] | 7-0 | `t[7:0]` | current torque t, lower 8-bit data | 12-bit range |
| DATA[6] | bits 4-7 / 0-3 | NULL | NULL | NULL |
| DATA[7] | bits 4-7 / 0-3 | NULL | NULL | NULL |

### 4.4. Communication Example

**Example 1:**

Send command: ID number `0x401`

| Data field | Data | Data partition | Data definition | Data range | Calculation |
|---|---|---|---|---|---|
| DATA[0] | `0xE6` | bits 4-7 = `0xE`, bits 0-3 = `0x6` | p_des value is `0xE666`, decimal 58982 | -12.5 rad ~ 12.5 rad, total 25 rad | `p_des = (58982/65535)*25 + (-12.5) = 9.99 rad` |
| DATA[1] | `0x66` | bits 4-7 = `0x6`, bits 0-3 = `0x6` | (continuation of p_des) | (continuation) | (continuation) |
| DATA[2] | `0x82` | bits 4-7 = `0x8`, bits 0-3 = `0x2` | v_des value is `0x82E`, decimal 2094 | -45 rad/s ~ 45 rad/s, total 90 rad/s | `v_des = (2094/4095)*90 + (-45) = 1.021 rad/s` |
| DATA[3] | `0xE0` | bits 4-7 = `0xE`, bits 0-3 = `0x0` | (continuation of v_des) | (continuation) | (continuation) |
| DATA[4] | `0x52` | bits 4-7 = `0x5`, bits 0-3 = `0x2` | kp value is `0x052`, decimal 82 | 0 ~ 500, total 500 | `kp = (82/4095)*500 + 0 = 10.012` |
| DATA[5] | `0x33` | bits 4-7 = `0x3`, bits 0-3 = `0x3` | kd value is `0x333`, decimal 819 | 0 ~ 50, total 50 | `kd = (819/4095)*50 = 10` |
| DATA[6] | `0x3B` | bits 4-7 = `0x3`, bits 0-3 = `0xB` | t_ff value is `0xB55`, decimal 2901 | -Motor Max Torque ~ +Motor Max Torque, total 2 × Motor Max Torque | `t_ff = (2901/4095) * 2 * Motor Max Torque + (-Motor Max Torque) = 0.416 * Motor Max Torque` |
| DATA[7] | `0x55` | bits 4-7 = `0x5`, bits 0-3 = `0x5` | (continuation of t_ff) | (continuation) | (continuation) |

> Note: this worked example computes p_des using a range of -12.5 rad to 12.5 rad (total 25 rad), which conflicts with §4.1's stated p_des range of -12.566 to 12.566 rad — this appears to be a leftover from an earlier manual revision that was not updated when the p_des range was widened. Likewise this example's kp/kd ranges (0–500 and 0–50) conflict with the prose ranges stated in §4.1 (0–1000 and 0–5); see the note under §4.1.

Reply command: ID number `0x501`

| Data field | Data | Data partition | Data definition | Data range | Calculation |
|---|---|---|---|---|---|
| DATA[0] | `0x01` | bits 0-7 = `0x1` | CANID | 0-32 (device address ID number) | — |
| DATA[1] | `0xE6` | bits 4-7 = `0xE`, bits 0-3 = `0x6` | p value is `0xE666`, decimal 58982 | -12.5 rad ~ 12.5 rad, total 25 rad | `p_des = (58982/65535)*25 + (-12.5) = 9.99 rad` |
| DATA[2] | `0x66` | bits 4-7 = `0x6`, bits 0-3 = `0x6` | (continuation of p) | (continuation) | (continuation) |
| DATA[3] | `0x82` | bits 4-7 = `0x8`, bits 0-3 = `0x2` | v value is `0x82E`, decimal 2094 | -45 rad/s ~ 45 rad/s, total 90 rad/s | `v_des = (2094/4095)*90 + (-45) = 1.021 rad/s` |
| DATA[4] | `0xEB` | bits 4-7 = `0xE`, bits 0-3 = `0xB` | (continuation of v; also upper nibble of t) | (continuation) | (continuation) |
| DATA[5] | `0x55` | bits 4-7 = `0x5`, bits 0-3 = `0x5` | t value is `0xB55`, decimal 2901 | -Motor Max Torque ~ +Motor Max Torque, total 2 × Motor Max Torque | `t_ff = (2901/4095) * 2 * Motor Max Torque + (-Motor Max Torque) = 0.416 * Motor Max Torque` |
| DATA[6] | `0x0` | bits 4-7 / 0-3 = NULL | NULL | NULL | NULL |
| DATA[7] | `0x0` | bits 4-7 / 0-3 = NULL | NULL | NULL | NULL |

> Note: DATA[2] of the reply frame is printed as `0x65` in the source table, but the accompanying prose states the combined 12-byte-pair value is `0xE666` (decimal 58982), and 0xE665 would equal decimal 58981, not 58982. This byte is transcribed above as `0x66` to match the stated combined value; the `0x65` appears to be a text-extraction artifact of the source PDF rather than an intentional different value.

## 5. RS485 Multi-Motor Command (0xCD + Command)

### 5.1. Instruction Description

The ID number 0xCD means that multiple motors respond to the same command at the same time. The content and function of the command are the same as those of the corresponding single-motor command; see the single-motor command descriptions for details.

### 5.2. Communication Example

Suppose there are 4 motors on the RS485 bus, with ID numbers 01, 02, 03, and 04 respectively.

**Example 1:**

Send command:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `0x3E` | `0xCD` | `0x08` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: all 4 motors receive the 0x80 motor shutdown command at the same time (see §2.30 for details), and then all 4 motors immediately execute the motor shutdown command.

Reply command: all 4 motors reply at the same time, each with its own ID number as the reply ID. The reply order depends on each motor's respective delay on the bus.

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `0x3E` | `0x01` | `0x08` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: the motor with ID number 0x01 returns the corresponding command.

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `0x3E` | `0x02` | `0x08` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: the motor with ID number 0x02 returns the corresponding command.

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `0x3E` | `0x03` | `0x08` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: the motor with ID number 0x03 returns the corresponding command.

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `0x3E` | `0x04` | `0x08` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: the motor with ID number 0x04 returns the corresponding command.

**Example 2:**

Send command:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `0x3E` | `0xCD` | `0x08` | `0x60` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: all 4 motors receive the 0x60 read multi-turn encoder position data command at the same time (see §2.21 for details), and then the 4 motors each reply with their own multi-turn encoder position data.

Reply command: all 4 motors reply at the same time, each with its own ID number as the reply ID. The reply order depends on each motor's respective delay on the bus.

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `0x3E` | `0x01` | `0x08` | `0x60` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: in the reply data of the motor with ID number 0x01, Data[4] through Data[7] form a 32-bit value (Data[4] is the lowest byte, Data[7] is the highest byte) of `0x00002710`, which is decimal 10000. This represents the motor's current multi-turn encoder value, relative to the multi-turn zero offset (initial position), of 10000 pulses.

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `0x3E` | `0x02` | `0x08` | `0x60` | `0x00` | `0x00` | `0x00` | `0x20` | `0x4E` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: in the reply data of the motor with ID number 0x02, Data[4] through Data[7] form a 32-bit value (Data[4] is the lowest byte, Data[7] is the highest byte) of `0x00004E20`, which is decimal 20000. This represents the motor's current multi-turn encoder value, relative to the multi-turn zero offset (initial position), of 20000 pulses.

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `0x3E` | `0x03` | `0x08` | `0x60` | `0x00` | `0x00` | `0x00` | `0x30` | `0x75` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: in the reply data of the motor with ID number 0x03, Data[4] through Data[7] form a 32-bit value (Data[4] is the lowest byte, Data[7] is the highest byte) of `0x00007530`, which is decimal 30000. This represents the motor's current multi-turn encoder value, relative to the multi-turn zero offset (initial position), of 30000 pulses.

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `0x3E` | `0x04` | `0x08` | `0x60` | `0x00` | `0x00` | `0x00` | `0x40` | `0x9C` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: in the reply data of the motor with ID number 0x04, Data[4] through Data[7] form a 32-bit value (Data[4] is the lowest byte, Data[7] is the highest byte) of `0x00009C40`, which is decimal 40000. This represents the motor's current multi-turn encoder value, relative to the multi-turn zero offset (initial position), of 40000 pulses.

## 6. RS485 Motion Mode Control Command

### 6.1. Instruction Description

This command consists of five input parameters: p_des (desired position), v_des (desired velocity), t_ff (feedforward torque), kp (position error coefficient), and kd (velocity error coefficient). All of the above parameters are described in terms of the joint output side, i.e. after the reducer.

Functional expression:

```
IqRef = [kp * (p_des - p_fb_actual_position) + kd * (v_des - v_fb_actual_velocity) + t_ff] / KT_OUT (torque coefficient)
```

IqRef is the final output current magnitude supplied to the motor.

### 6.2. Transmit Data Field Definition (Little-endian byte order)

| Byte | Variable name | Description |
|---|---|---|
| 1 | Start0 | Frame header, fixed to `0xFE` |
| 2 | Start1 | Frame header, fixed to `0xEE` |
| 3 | Motor_ID | Motor ID, can be `0x01`, `0x02`, `0x03`, or `0xBB` (`0xBB` indicates broadcast to all motors) |
| 4 | Mode | Motor operating mode, can be `0x00` (stop) or `0x0A` (closed-loop servo control) |
| 5-8 | t_ff | Motor feedforward torque, `float` data type, unit Nm |
| 9-12 | v_des | Motor target speed, `float` data type, unit rad/s |
| 13-16 | p_des | Motor target position, `float` data type, unit rad |
| 17-20 | kp | Motor position stiffness kp, `float` data type |
| 21-24 | kd | Motor position stiffness kd, `float` data type |
| 25-28 | CRC_Data | CRC32 checksum |

### 6.3. Response Data Field Definition (Little-endian byte order)

| Byte | Variable name | Description |
|---|---|---|
| 1 | Start0 | Frame header, fixed to `0xFE` |
| 2 | Start1 | Frame header, fixed to `0xEE` |
| 3 | Motor_ID | Motor ID, can be `0x01`, `0x02`, or `0x03` (motors do not return status in broadcast mode) |
| 4 | Mode | Motor's current operating mode |
| 5 | Temp | Motor's current temperature, data type `int8_t`, unit °C |
| 6-7 | Motor_Err | Error status (little-endian in the message; described below in big-endian order) |
| 8-9 | Reserved | Reserved |
| 10-13 | T | Motor's current output torque, `float` data type, unit Nm |
| 14-17 | W | Motor's current actual speed, `float` data type, unit rad/s |
| 18-21 | Pos | Motor's current actual position, `float` data type, unit rad |
| 22-25 | CRC_Data | CRC32 checksum |

> Note: the source PDF's per-byte row numbering becomes inconsistent across the page break between the T/W/Pos/CRC_Data rows (the printed final byte index is 23, not 25). The byte ranges above for T, W, Pos, and CRC_Data are reconstructed assuming each `float` field and the CRC32 field are 4 bytes wide — consistent with the equivalent send-table structure in §6.2 — rather than taken literally from the garbled row numbers in the extracted text.

Motor_Err bit flags (multiple simultaneous errors are OR-added together; e.g. a value of `0x0016` indicates `0x2 + 0x4 + 0x10`, meaning the motor has three simultaneous errors: stall, undervoltage, and phase-current overcurrent):

- `0x0002`: Motor stall
- `0x0004`: Undervoltage
- `0x0008`: Overvoltage
- `0x0010`: Phase current overcurrent
- `0x0080`: Component overtemperature
- `0x1000`: Motor overtemperature
- `0x2000`: Encoder calibration error

## 7. Indicator Light Description

### 7.1. Status Description

- When the indicator light is solid on, the motor is running normally.
- Slow flashing indicates that the motor has a secondary (Level 2) error. If the recovery condition is reached, the motor automatically returns to normal operation and the indicator light becomes solid on again.
- Fast flashing indicates that the motor has a first-level (Level 1) error, from which the motor cannot recover automatically. The motor fault must be checked and the motor restarted before it can continue to run.

### 7.2. Failure Description Table

| Fault name | Description | Error level |
|---|---|---|
| Hardware over-current | If the motor current exceeds the limit value, there may be a short circuit, phase loss, loss of control, motor damage, etc. | Level 1 |
| Stall error | After the current reaches the stall current, the speed is very low and continues for a period of time. This indicates that the motor load is too large. | Level 1 |
| Under-voltage error | The power input is lower than the set undervoltage value. | Level 2 |
| Over-voltage error | The power input is higher than the set overvoltage value. | Level 2 |
| Phase-current over-current | The software detects that the motor current exceeds the limit value; there may be a short circuit, phase loss, loss of control, motor damage, etc. | Level 1 |
| Power overrun error | If the input current of the power supply exceeds the limit value, there may be a situation where the load is too large or the speed is too high. | Level 2 |
| Calibration parameter read error | Failed to write parameters, causing parameter loss. | Level 1 |
| Over-speed error | The motor's running speed exceeds the limit value; there may be over-pressure or drag use. | Level 2 |
| Motor over-temperature error | If the motor temperature exceeds the set value, there may be a short circuit, parameter error, or long-term overload use. | Level 2 |
| Encoder calibration error | The encoder calibration result deviates too much from the standard value. | Level 2 |

## 8. Version Revision Information

#### Version V3.1

- Revised the reply-data definition in the 5.0 motion-control command.

Revision date: 2022.6.23

#### Version V3.2

- Added indicator light description.

Revision date: 2022.7.27

#### Version V3.3

- Added function control command 0x20: clear multi-turn value function, and CAN filter enable/disable control function.

Revision date: 2022.7.31

#### Version V3.4

- Added position tracking command 0xA3.
- Added, in command 0x43, the setting of 4 values for position-planning and speed-planning acceleration and deceleration.

Revision date: 2022.8.17

#### Version V3.5

- Added position tracking command with speed limit, 0xA5.
- Added function control command 0x20: error-status sending and multi-turn-value power-down save selection function.
- Added command 0xB5 to read the motor model.

Revision date: 2022.9.05

#### Version V3.6

- Added RS485 broadcast command description, 0xCD.

Revision date: 2022.10.13

#### Version V3.7

- Removed command 0xA3.
- Merged 0xA5 into 0xA4.
- Added command 0xA6, single-turn position command.
- Added command 0x90, read single-turn encoder.
- Added command 0x94, read motor single-turn angle.

Revision date: 2022.11.26

#### Version V3.8

- Changed the RS485 protocol baud rate from 2 Mbps to 2.5 Mbps.

Revision date: 2022.11.26

#### Version V3.9

- Added 485 serial-port configuration instructions.
- Added a function index to command 0x42, so the acceleration and deceleration values of position and speed can be read via index.
- Added the 0xB6 active-reply function.

Revision date: 2023.3.11

#### Version V4.0

- Added the function of setting the CAN ID in command 0x20.
- Added the maximum positive angle limit value in command 0x20.
- Added the maximum negative angle limit value in command 0x20.

Revision date: 2023.10.16

#### Version V4.1

- Changed the motor's single-turn angle (circleAngle) to `uint16_t` type data.

Revision date: 2024.2.13

#### Version V4.2

- Modified the Read PID Parameters command (0x30) to read the PID parameters of the current, speed, and position loops using an index.
- Modified the Write PID Parameters to RAM command (0x31), using an index to write the PID parameters of the current loop, velocity loop, and position loop to RAM.
- Modified the Write PID Parameters to ROM command (0x32), using an index to write the PID parameters of the current loop, speed loop, and position loop to ROM.

Revision date: 2024.5.28

#### Version V4.3

- Added the force-control position closed-loop control command (0xA9).
- Included force-control mode in the speed closed-loop control command (0xA2).
- Improved the MIT command.
- Added encoder-data-abnormal error.

Revision date: 2025.5.12

> Note: the English text in the source PDF for this entry states the revision date as "2024.5.12," which would place it before the preceding V4.2 entry (2024.5.28). The Chinese source text for this same entry states "2025.5.12," which is chronologically consistent (after V4.2's 2024.5.28 and before V4.4's 2026.03.25). The date above uses the Chinese-confirmed value; the English date appears to be a translation typo.

#### Version V4.4

- Added the TF command (0x73, position control command with feedforward torque).
- Improved the force-controlled position closed-loop control command (0xA9).
- Expanded the value range of the kd parameter in the MIT command from 0-5 to 0-50.
- Changed the value range of the t_ff parameter in the MIT command from -24 Nm~+24 Nm to -Motor Max Torque~+Motor Max Torque.
- Changed the single-turn value returned by the read single-turn angle command (0x94) to a range of ±180°, data type `int32_t`, 4 valid bytes, unit 0.01°/LSB.
- Added more detailed usage instructions for the motor model read command (0xB5).
- Added error-state auto-recovery enable to the function control command (0x20).
- Removed the read single-turn encoder command (0x90) and the CAN ID setting command (0x79).
- Added motor error trigger and recovery conditions to the "Read Motor Status 1 and Error Flags" command (0x9A).
- Changed the data type of the Feedforward Torque control value in the TF command to `int8_t`, with a value range of -128 to 127.
- Changed the p_des value range in the CAN-communication MIT command from -12.5 to 12.5 rad to -12.566 to 12.566 rad.
- Added the SF command (0x72, position control command with feedforward speed).

  > Erratum: the source manual (both Chinese and English) literally says "feedforward torque" for
  > this changelog entry, but §2.24 itself titles 0x72 "Position Control with Speed Feedforward"
  > and its `Feedforward Speed` field is 1%-of-rated-**speed**/LSB — "torque" here is a copy-paste
  > error from the neighboring TF (0x73) entry, corrected above.

Revision date: 2026.03.25
