# MyActuator Servo Motor Control Protocol

**Applicable driver:** V3
**Version:** V4.3
**Date:** 2025.05

Wire protocol reference for the RMD-X4-36 actuator and other RMD-X series motors, covering both the CAN bus and RS485 bus command sets.

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
  - [2.11. Read Single-Turn Encoder Command (0x90)](#211-read-single-turn-encoder-command-0x90)
  - [2.12. Read Multi-Turn Angle Command (0x92)](#212-read-multi-turn-angle-command-0x92)
  - [2.13. Read Single-Turn Angle Command (0x94)](#213-read-single-turn-angle-command-0x94)
  - [2.14. Read Motor Status 1 and Error Flag Command (0x9A)](#214-read-motor-status-1-and-error-flag-command-0x9a)
  - [2.15. Read Motor Status 2 Command (0x9C)](#215-read-motor-status-2-command-0x9c)
  - [2.16. Read Motor Status 3 Command (0x9D)](#216-read-motor-status-3-command-0x9d)
  - [2.17. Motor Shutdown Command (0x80)](#217-motor-shutdown-command-0x80)
  - [2.18. Motor Stop Command (0x81)](#218-motor-stop-command-0x81)
  - [2.19. Torque Closed-Loop Control Command (0xA1)](#219-torque-closed-loop-control-command-0xa1)
  - [2.20. Speed Closed-Loop Control Command (0xA2)](#220-speed-closed-loop-control-command-0xa2)
  - [2.21. Absolute Position Closed-Loop Control Command (0xA4)](#221-absolute-position-closed-loop-control-command-0xa4)
  - [2.22. Single-Turn Position Control Command (0xA6)](#222-single-turn-position-control-command-0xa6)
  - [2.23. Incremental Position Closed-Loop Control Command (0xA8)](#223-incremental-position-closed-loop-control-command-0xa8)
  - [2.24. Force Control Position Closed-Loop Command (0xA9)](#224-force-control-position-closed-loop-command-0xa9)
  - [2.25. System Operating Mode Acquisition (0x70)](#225-system-operating-mode-acquisition-0x70)
  - [2.26. System Reset Command (0x76)](#226-system-reset-command-0x76)
  - [2.27. System Brake Release Command (0x77)](#227-system-brake-release-command-0x77)
  - [2.28. System Brake Lock Command (0x78)](#228-system-brake-lock-command-0x78)
  - [2.29. System Runtime Read Command (0xB1)](#229-system-runtime-read-command-0xb1)
  - [2.30. System Software Version Date Read Command (0xB2)](#230-system-software-version-date-read-command-0xb2)
  - [2.31. Communication Interruption Protection Time Setting Command (0xB3)](#231-communication-interruption-protection-time-setting-command-0xb3)
  - [2.32. Communication Baud Rate Setting Command (0xB4)](#232-communication-baud-rate-setting-command-0xb4)
  - [2.33. Motor Model Reading Command (0xB5)](#233-motor-model-reading-command-0xb5)
  - [2.34. Active Reply Function Command (0xB6)](#234-active-reply-function-command-0xb6)
  - [2.35. Function Control Command (0x20)](#235-function-control-command-0x20)
- [3. CAN Multi-Motor Command (0x280 + Command)](#3-can-multi-motor-command-0x280--command)
- [4. CANID Setting Command (0x79)](#4-canid-setting-command-0x79)
- [5. Motion Mode Control Command_CAN (0x400 + ID)](#5-motion-mode-control-command_can-0x400--id)
- [6. RS485 Multi-Motor Command (0xCD + Command)](#6-rs485-multi-motor-command-0xcd--command)
- [7. RS485-ID Setting Command (0x79)](#7-rs485-id-setting-command-0x79)
- [8. Indicator Light Description](#8-indicator-light-description)
- [9. Version Revision Information](#9-version-revision-information)

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
| ID | `1`–`32` | 1 | Device address, corresponding to the ID number of each motor. |
| Data Length | Data Length | 1 | The length of the data field. In the standard protocol, the length is fixed to 8 bytes. |
| Data field | Data content | According to the length | The content of the data field in the standard protocol is exactly the same as that of the CAN frame. |
| Check | CRC Check | 2 | CRC16 check, low byte first, high byte last. |

---

## 2. Single Motor Command Description

The host addresses a single motor with `0x140 + ID` on CAN (`ID` = 1–32) and receives its reply on `0x240 + ID`. On RS485, the same 8-byte data-field content is wrapped in a `0x3E` frame with the device's ID byte, a length byte, and a trailing CRC16. Each command below is described once; the CAN and RS485 encodings of `DATA[0]`–`DATA[7]` are identical.

### 2.1. Read PID Parameter Command (0x30)

**Instruction description:** This command can read the PID parameters of the current, speed, and position loops. The data type is `Float`, and the specific parameter is selected by the index value — see the function index table below.

**Send data field:**

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

**Reply data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x30` |
| `DATA[1]` | Function index | `DATA[1] = (uint8_t)index` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Parameter byte 1 (low) | `DATA[4] = (uint8_t)(Value)` |
| `DATA[5]` | Parameter byte 2 | `DATA[5] = (uint8_t)(Value>>8)` |
| `DATA[6]` | Parameter byte 3 | `DATA[6] = (uint8_t)(Value>>16)` |
| `DATA[7]` | Parameter byte 4 | `DATA[7] = (uint8_t)(Value>>24)` |

**Function index description:**

| Index | Parameter |
| --- | --- |
| `0x01` | Current loop KP parameter |
| `0x02` | Current loop KI parameter |
| `0x04` | Speed loop KP parameter |
| `0x05` | Speed loop KI parameter |
| `0x07` | Position loop KP parameter |
| `0x08` | Position loop KI parameter |
| `0x09` | Position loop KD parameter |

**Communication example — Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x30` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x30` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: According to the index value table, `Data[1] = 0x01` means the current loop KP, indicating a read of the current loop KP parameter.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x30` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0x80` | `0x3F` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x30` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0x80` | `0x3F` | CRC16L | CRC16H |

Description: `Data[1] = 0x01` indicates the current loop KP parameter. `Data[4]`–`Data[7]` (`Data[4]` lowest byte, `Data[7]` highest byte) form the 32-bit value `0x3F800000`, of type `Float`. Converted to decimal this is `1.0`, i.e. the current loop KP parameter is currently `1.0`. (An online hex/float converter such as http://www.speedfly.cn/tools/hexconvert/ can be used for this conversion.)

### 2.2. Write PID Parameters to RAM Command (0x31)

**Instruction description:** Writes the current, speed, and position loop KP/KI parameters to RAM in one shot; the value is **not** retained after power-off. The data type is `Float`, selected by index — see 2.1's function index table. Avoid writing parameters immediately after motor start-up while it is in motion.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x31` |
| `DATA[1]` | Function index | `DATA[1] = (uint8_t)index` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Parameter byte 1 (low) | `DATA[4] = (uint8_t)(Value)` |
| `DATA[5]` | Parameter byte 2 | `DATA[5] = (uint8_t)(Value>>8)` |
| `DATA[6]` | Parameter byte 3 | `DATA[6] = (uint8_t)(Value>>16)` |
| `DATA[7]` | Parameter byte 4 | `DATA[7] = (uint8_t)(Value>>24)` |

**Reply data field:** Same content as the sent data.

**Function index description:** Same table as [2.1](#21-read-pid-parameter-command-0x30).

**Communication example — Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x31` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0xC0` | `0x3F` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x31` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0xC0` | `0x3F` | CRC16L | CRC16H |

Description: `Data[1] = 0x01` is the current loop KP parameter. `Data[4]`–`Data[7]` form the 32-bit `Float` value `0x3FC00000`, which is `1.5` in decimal — the current loop KP parameter is set to `1.5` and written to RAM; it is not retained after power-off.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x31` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0xC0` | `0x3F` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x31` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0xC0` | `0x3F` | CRC16L | CRC16H |

### 2.3. Write PID Parameters to ROM Command (0x32)

**Instruction description:** Writes the current, speed, and position loop KP/KI parameters to ROM in one shot; the value **is** retained after power-off. The data type is `Float`, selected by index — see 2.1's function index table. Avoid writing parameters immediately after motor start-up while it is in motion.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x32` |
| `DATA[1]` | Function index | `DATA[1] = (uint8_t)index` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Parameter byte 1 (low) | `DATA[4] = (uint8_t)(Value)` |
| `DATA[5]` | Parameter byte 2 | `DATA[5] = (uint8_t)(Value>>8)` |
| `DATA[6]` | Parameter byte 3 | `DATA[6] = (uint8_t)(Value>>16)` |
| `DATA[7]` | Parameter byte 4 | `DATA[7] = (uint8_t)(Value>>24)` |

**Reply data field:** Same content as the sent data.

**Function index description:** Same table as [2.1](#21-read-pid-parameter-command-0x30).

**Communication example — Example 1:**

Send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x32` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0xC0` | `0x3F` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x32` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0xC0` | `0x3F` | CRC16L | CRC16H |

Description: `Data[1] = 0x01` is the current loop KP parameter. `Data[4]`–`Data[7]` form the 32-bit `Float` value `0x3FC00000` (`1.5` decimal) — the current loop KP parameter is set to `1.5` and written to ROM, so it **is** retained after power-off.

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

**Instruction description:** The host sends this command to read an acceleration/deceleration parameter of the current motor.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x42` |
| `DATA[1]` | Function index | `DATA[1] = (uint8_t)index` |
| `DATA[2]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The acceleration parameter `Accel` (`int32_t`, unit 1 dps/s, range 100–60000) is returned:

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x42` |
| `DATA[1]` | Function index | `DATA[1] = (uint8_t)index` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Acceleration byte 1 (low) | `DATA[4] = (uint8_t)(Accel)` |
| `DATA[5]` | Acceleration byte 2 | `DATA[5] = (uint8_t)(Accel>>8)` |
| `DATA[6]` | Acceleration byte 3 | `DATA[6] = (uint8_t)(Accel>>16)` |
| `DATA[7]` | Acceleration byte 4 | `DATA[7] = (uint8_t)(Accel>>24)` |

**Function index description:**

| Index value | Command name | Function description |
| --- | --- | --- |
| `0x00` | Position planning acceleration | Acceleration value from initial velocity to maximum velocity in position planning |
| `0x01` | Position planning deceleration | Deceleration value from maximum velocity to standstill in position planning |
| `0x02` | Speed planning acceleration | Acceleration value from the current speed to the target speed, including both positive and negative directions |
| `0x03` | Speed planning deceleration | Deceleration value to go from the current velocity to the target velocity in the same direction |

**Communication example:**

Example 1 — send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x42` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x42` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: Reads the position planning acceleration.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x42` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x42` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[1] = 0x00` indicates position planning acceleration. `Data[4]`–`Data[7]` (`Data[4]` low, `Data[7]` high) form the 32-bit value `0x00002710` = `10000` decimal — the position-loop acceleration is `10000 dps/s`.

Example 2 — send command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x42` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x42` | `0x01` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: Reads the position planning deceleration.

Reply command:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x42` | `0x01` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x42` | `0x01` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[1] = 0x01` indicates position planning deceleration. `Data[4]`–`Data[7]` form `0x00002710` = `10000` decimal — the position-loop deceleration is `10000 dps/s`.

### 2.5. Write Acceleration to RAM and ROM Command (0x43)

**Instruction description:** Writes an acceleration/deceleration value to both RAM and ROM (retained after power-off). `Accel` is `uint32_t`, unit 1 dps/s, range 100–60000. The index selects which of the four acceleration/deceleration values (position or speed planning) is written — see the function index table below. Avoid writing parameters immediately after motor start-up while it is in motion.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x43` |
| `DATA[1]` | Function index | `DATA[1] = (uint8_t)index` |
| `DATA[2]` | NULL | `0x00` |
| `DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Acceleration byte 1 (low) | `DATA[4] = (uint8_t)(Accel)` |
| `DATA[5]` | Acceleration byte 2 | `DATA[5] = (uint8_t)(Accel>>8)` |
| `DATA[6]` | Acceleration byte 3 | `DATA[6] = (uint8_t)(Accel>>16)` |
| `DATA[7]` | Acceleration byte 4 | `DATA[7] = (uint8_t)(Accel>>24)` |

**Reply data field:** Same content as the sent data.

**Function index description:**

| Index value | Command name | Function description |
| --- | --- | --- |
| `0x00` | Position planning acceleration | Acceleration value from initial velocity to maximum velocity in position planning |
| `0x01` | Position planning deceleration | Deceleration value from maximum speed to stop in position planning |
| `0x02` | Speed planning acceleration | Acceleration value from the current speed to the target speed, including forward and reverse directions |
| `0x03` | Speed planning deceleration | Deceleration value from the current speed to the target speed in the same direction |

**Communication example:**

Example 1 — position planning acceleration:

CAN:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x141` | `0x43` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` |

RS485:

| Frame header | ID | Length | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | CRC16L | CRC16H |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x3E` | `0x01` | `0x08` | `0x43` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | CRC16L | CRC16H |

Description: `Data[1] = 0x00` selects position planning acceleration. `Data[4]`–`Data[7]` form `0x00002710` = `10000` decimal — writes `10000 dps/s` position planning acceleration to the drive, saved across power cycles.

Reply command (same for all four examples in this section — echoes the sent frame):

CAN: `0x241 0x43 0x00 0x00 0x00 0x10 0x27 0x00 0x00`

Example 2 — position planning deceleration: `Data[1] = 0x01`, `Data[4..7] = 0x00002710` (`10000 dps/s`). Send: `0x141 0x43 0x01 0x00 0x00 0x10 0x27 0x00 0x00`. Reply echoes the sent frame with ID `0x241`.

Example 3 — speed planning acceleration: `Data[1] = 0x02`, `Data[4..7] = 0x00002710` (`10000 dps/s`). Send: `0x141 0x43 0x02 0x00 0x00 0x10 0x27 0x00 0x00`. Reply echoes the sent frame with ID `0x241`.

Example 4 — speed planning deceleration: `Data[1] = 0x03`, `Data[4..7] = 0x00002710` (`10000 dps/s`). Send: `0x141 0x43 0x03 0x00 0x00 0x10 0x27 0x00 0x00`. Reply echoes the sent frame with ID `0x241`.

### 2.6. Read Multi-Turn Encoder Position Data Command (0x60)

**Instruction description:** Reads the multi-turn position of the encoder, representing the rotation angle of the motor output shaft.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x60` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** `encoder` — multi-turn encoder position (`int32_t`, 4 bytes valid), equal to the raw multi-turn encoder value minus the encoder's multi-turn zero offset (home position).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x60` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Encoder position byte 1 (low) | `DATA[4] = (uint8_t)(encoder)` |
| `DATA[5]` | Encoder position byte 2 | `DATA[5] = (uint8_t)(encoder>>8)` |
| `DATA[6]` | Encoder position byte 3 | `DATA[6] = (uint8_t)(encoder>>16)` |
| `DATA[7]` | Encoder position byte 4 | `DATA[7] = (uint8_t)(encoder>>24)` |

**Communication example — Example 1:**

Send: CAN `0x141 0x60 0x00 0x00 0x00 0x00 0x00 0x00 0x00`. RS485: `0x3E 0x01 0x08 0x60 0x00 0x00 0x00 0x00 0x00 0x00 0x00 CRC16L CRC16H`.

Reply: CAN `0x241 0x60 0x00 0x00 0x00 0x10 0x27 0x00 0x00`. RS485: `0x3E 0x01 0x08 0x60 0x00 0x00 0x00 0x10 0x27 0x00 0x00 CRC16L CRC16H`.

Description: `Data[4]`–`Data[7]` form the 32-bit value `0x00002710` = `10000` decimal — the multi-turn encoder value relative to the zero offset (home position) is `10000` pulses.

### 2.7. Read Multi-Turn Encoder Original Position Data Command (0x61)

**Instruction description:** Reads the multi-turn encoder's raw position, i.e. the encoder value **without** the zero offset (home position) subtracted.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x61` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** `encoderRaw` — raw multi-turn encoder position (`int32_t`, 4 bytes valid).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x61` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Raw position byte 1 (low) | `DATA[4] = (uint8_t)(encoderRaw)` |
| `DATA[5]` | Raw position byte 2 | `DATA[5] = (uint8_t)(encoderRaw>>8)` |
| `DATA[6]` | Raw position byte 3 | `DATA[6] = (uint8_t)(encoderRaw>>16)` |
| `DATA[7]` | Raw position byte 4 | `DATA[7] = (uint8_t)(encoderRaw>>24)` |

**Communication example — Example 1:**

Send: CAN `0x141 0x61 0x00 0x00 0x00 0x00 0x00 0x00 0x00`. RS485: `0x3E 0x01 0x08 0x61 0x00 0x00 0x00 0x00 0x00 0x00 0x00 CRC16L CRC16H`.

Reply: CAN `0x241 0x61 0x00 0x00 0x00 0x10 0x27 0x00 0x00`. RS485: `0x3E 0x01 0x08 0x61 0x00 0x00 0x00 0x10 0x27 0x00 0x00 CRC16L CRC16H`.

Description: `Data[4]`–`Data[7]` form `0x00002710` = `10000` decimal — the raw multi-turn encoder value (excluding zero offset) is `10000` pulses.

### 2.8. Read Multi-Turn Encoder Zero Offset Data Command (0x62)

**Instruction description:** Reads the multi-turn zero offset value (home position) of the encoder.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x62` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** `encoderOffset` — multi-turn zero offset (`int32_t`, 4 bytes valid).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x62` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Offset byte 1 (low) | `DATA[4] = (uint8_t)(encoderOffset)` |
| `DATA[5]` | Offset byte 2 | `DATA[5] = (uint8_t)(encoderOffset>>8)` |
| `DATA[6]` | Offset byte 3 | `DATA[6] = (uint8_t)(encoderOffset>>16)` |
| `DATA[7]` | Offset byte 4 | `DATA[7] = (uint8_t)(encoderOffset>>24)` |

**Communication example — Example 1:**

Send: CAN `0x141 0x62 0x00 0x00 0x00 0x00 0x00 0x00 0x00`. RS485: `0x3E 0x01 0x08 0x62 0x00 0x00 0x00 0x00 0x00 0x00 0x00 CRC16L CRC16H`.

Reply: CAN `0x241 0x62 0x00 0x00 0x00 0x10 0x27 0x00 0x00`. RS485: `0x3E 0x01 0x08 0x62 0x00 0x00 0x00 0x10 0x27 0x00 0x00 CRC16L CRC16H`.

Description: `Data[4]`–`Data[7]` form `0x00002710` = `10000` decimal — the current multi-turn zero-offset value is `10000` pulses.

### 2.9. Write Encoder Multi-Turn Value to ROM as Motor Zero Command (0x63)

**Instruction description:** Sets the encoder's multi-turn zero offset (home position) to the given value `encoderOffset` (`int32_t`, 4 bytes valid), written to ROM. Avoid writing parameters immediately after motor start-up while it is in motion.

> **Note:** The motor must be restarted for the new zero point to take effect. Because the zero offset changes, subsequent target-position commands should be referenced against the new zero offset (home position).

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x63` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Offset byte 1 (low) | `DATA[4] = (uint8_t)(encoderOffset)` |
| `DATA[5]` | Offset byte 2 | `DATA[5] = (uint8_t)(encoderOffset>>8)` |
| `DATA[6]` | Offset byte 3 | `DATA[6] = (uint8_t)(encoderOffset>>16)` |
| `DATA[7]` | Offset byte 4 | `DATA[7] = (uint8_t)(encoderOffset>>24)` |

> **Source note:** the manual's byte table literally repeats `(uint8_t)(encoderOffset>>8)` for `DATA[6]` and `DATA[7]` (rather than `>>16`/`>>24`). This is an apparent copy-paste error in MyActuator's manual — the identical command in [2.10 (0x64)](#210-write-the-current-multi-turn-position-of-the-encoder-to-the-rom-as-the-motor-zero-command-0x64) uses the correct `>>16`/`>>24` shifts for the same field layout, and the shifts above have been corrected to match. Verify against firmware/PDF page 21–22 if in doubt.

**Reply data field:** Same content as the sent data.

**Communication example — Example 1:**

Send: CAN `0x141 0x63 0x00 0x00 0x00 0x10 0x27 0x00 0x00`. RS485: `0x3E 0x01 0x08 0x63 0x00 0x00 0x00 0x10 0x27 0x00 0x00 CRC16L CRC16H`.

Description: `Data[4]`–`Data[7]` form `0x00002710` = `10000` decimal — writes `10000` pulses as the multi-turn encoder zero offset.

Reply: echoes the sent frame, CAN `0x241 0x63 0x00 0x00 0x00 0x10 0x27 0x00 0x00`.

### 2.10. Write the Current Multi-Turn Position of the Encoder to the ROM as the Motor Zero Command (0x64)

**Instruction description:** Writes the motor's *current* encoder position as the multi-turn zero offset (home position) into ROM.

> **Note:** After writing the new zero point, send `0x76` (System Reset Command) to restart the system for it to take effect. Because the zero offset changes, subsequent target-position commands should be referenced against the new zero offset.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x64` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** `encoderOffset` — the zero offset value that was set.

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x64` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Offset byte 1 (low) | `DATA[4] = (uint8_t)(encoderOffset)` |
| `DATA[5]` | Offset byte 2 | `DATA[5] = (uint8_t)(encoderOffset>>8)` |
| `DATA[6]` | Offset byte 3 | `DATA[6] = (uint8_t)(encoderOffset>>16)` |
| `DATA[7]` | Offset byte 4 | `DATA[7] = (uint8_t)(encoderOffset>>24)` |

**Communication example — Example 1:**

Send: CAN `0x141 0x64 0x00 0x00 0x00 0x00 0x00 0x00 0x00`. RS485: `0x3E 0x01 0x08 0x64 0x00 0x00 0x00 0x00 0x00 0x00 0x00 CRC16L CRC16H`.

Description: After sending `0x64`, the motor writes its current multi-turn encoder value as the zero offset (home position) into ROM.

Reply: CAN `0x241 0x64 0x00 0x00 0x00 0x10 0x27 0x00 0x00`.

Description: `Data[4]`–`Data[7]` form `0x00002710` = `10000` decimal — the multi-turn zero offset written is `10000` pulses.

### 2.11. Read Single-Turn Encoder Command (0x90)

**Instruction description:** Reads the current position of the encoder. Note: this command is used as the single-turn data-read command for direct-drive motors.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x90` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** Three values are returned:
1. `encoder` — encoder position after subtracting the zero offset from the raw position.
2. `encoderRaw` — raw encoder position.
3. `encoderOffset` — encoder zero offset (the motor's angle zero point).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x90` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | Encoder position byte 1 (low) | `DATA[2] = (uint8_t)(encoder)` |
| `DATA[3]` | Encoder position byte 2 (high) | `DATA[3] = (uint8_t)(encoder>>8)` |
| `DATA[4]` | Raw position byte 1 (low) | `DATA[4] = (uint8_t)(encoderRaw)` |
| `DATA[5]` | Raw position byte 2 (high) | `DATA[5] = (uint8_t)(encoderRaw>>8)` |
| `DATA[6]` | Zero offset byte 1 (low) | `DATA[6] = (uint8_t)(encoderOffset)` |
| `DATA[7]` | Zero offset byte 2 (high) | `DATA[7] = (uint8_t)(encoderOffset>>8)` |

**Communication example — Example 1:**

Send: CAN `0x141 0x90 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Reply: CAN `0x241 0x90 0x00 0x33 0x08 0xBE 0x2C 0x8B 0x24`.

Description: `Data[2]`–`Data[3]` form the 16-bit value `0x0833` = `2099` decimal — the current encoder position relative to the zero offset is `2099` pulses. `Data[4]`–`Data[5]` form `0x2CBE` = `11454` decimal — the current raw encoder position is `11454` pulses. `Data[6]`–`Data[7]` form `0x248B` = `9355` decimal — the zero offset position is `9355` pulses.

### 2.12. Read Multi-Turn Angle Command (0x92)

**Instruction description:** Reads the current multi-turn absolute angle value of the motor.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x92` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** `motorAngle` (`int32_t`, 4 bytes valid, unit 0.01°/LSB).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x92` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Angle byte 1 (low) | `DATA[4] = (uint8_t)(motorAngle)` |
| `DATA[5]` | Angle byte 2 | `DATA[5] = (uint8_t)(motorAngle>>8)` |
| `DATA[6]` | Angle byte 3 | `DATA[6] = (uint8_t)(motorAngle>>16)` |
| `DATA[7]` | Angle byte 4 | `DATA[7] = (uint8_t)(motorAngle>>24)` |

**Communication example — Example 1:**

Send: CAN `0x141 0x92 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Reply: CAN `0x241 0x92 0x00 0x00 0x00 0xA0 0x8C 0x00 0x00`.

Description: `Data[4]`–`Data[7]` form `0x00008CA0` = `36000` decimal, reduced by the 0.01°/LSB unit → `36000 × 0.01 = 360°`. The motor output shaft has moved `360°` in the positive direction relative to the zero position.

### 2.13. Read Single-Turn Angle Command (0x94)

**Instruction description:** Reads the current single-turn angle of the motor.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x94` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** `circleAngle` — single-turn angle (`int16_t`, unit 0.01°/LSB, range 0–35999), starting from the encoder zero point, increasing clockwise, and wrapping to 0 upon returning to zero.

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x94` |
| `DATA[1]`–`DATA[5]` | NULL | `0x00` |
| `DATA[6]` | Single-turn angle byte 1 (low) | `DATA[6] = (uint8_t)(circleAngle)` |
| `DATA[7]` | Single-turn angle byte 2 (high) | `DATA[7] = (uint8_t)(circleAngle>>8)` |

**Communication example — Example 1:**

Send: CAN `0x141 0x94 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Reply: CAN `0x241 0x94 0x00 0x00 0x00 0x00 0x00 0x10 0x27`.

Description: `Data[6]`–`Data[7]` form the 16-bit value `0x2710` = `10000` decimal, unit 0.01° → the motor is currently at `100°` relative to the zero position.

### 2.14. Read Motor Status 1 and Error Flag Command (0x9A)

**Instruction description:** Reads the current motor temperature, voltage, and error status flags.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9A` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:**
1. `temperature` — motor temperature (`int8_t`, 1°C/LSB).
2. Brake control command state: `1` = brake-release command, `0` = brake-lock command.
3. `voltage` — bus voltage (`uint16_t`, 0.1 V/LSB).
4. `errorState` — error flags (`uint16_t`, each bit/mask represents a distinct error condition).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9A` |
| `DATA[1]` | Motor temperature | `DATA[1] = (uint8_t)(temperature)` |
| `DATA[2]` | MOS temperature | `DATA[2] = (uint8_t)(motorMOStemperature)` |
| `DATA[3]` | Brake release command state | `DATA[3] = (uint8_t)(RlyCtrlRslt)` |
| `DATA[4]` | Voltage byte 1 (low) | `DATA[4] = (uint8_t)(voltage)` |
| `DATA[5]` | Voltage byte 2 (high) | `DATA[5] = (uint8_t)(voltage>>8)` |
| `DATA[6]` | Error status byte 1 (low) | `DATA[6] = (uint8_t)(errorState)` |
| `DATA[7]` | Error status byte 2 (high) | `DATA[7] = (uint8_t)(errorState>>8)` |

**Remarks:**

1. `System_errorState` value table:

| `System_errorState` | Status description |
| --- | --- |
| `0x0002` | Motor stall |
| `0x0004` | Low voltage |
| `0x0008` | Over voltage |
| `0x0010` | Over current |
| `0x0040` | Power overrun |
| `0x0080` | Calibration parameter writing error |
| `0x0100` | Speeding (over-speed) |
| `0x0800` | Component over-temperature |
| `0x1000` | Motor over-temperature |
| `0x2000` | Encoder calibration error |
| `0x4000` | Encoder data error |

2. When multiple errors occur simultaneously, the error status masks are OR-combined. For example, `0x0016` = `0x0002 + 0x0004 + 0x0010`, meaning motor stall, low voltage, and over-current are all present simultaneously.

**Communication example — Example 1:**

Send: CAN `0x141 0x9A 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Reply: CAN `0x241 0x9A 0x32 0x00 0x01 0xE5 0x01 0x04 0x00`.

Description: `Data[1] = 0x32` = `50` decimal — motor temperature is `50°C`. `Data[3] = 0x01` — the brake-release command has been executed (`1` = released). `Data[4]`/`Data[5]` form `0x01E5` = `485` decimal, scaled by 0.1 V/LSB → `48.5 V` supply voltage. `Data[6]`/`Data[7]` form `0x0004`, which per the `System_errorState` table indicates a low-voltage error.

### 2.15. Read Motor Status 2 Command (0x9C)

**Instruction description:** Reads the current motor temperature, torque current, speed, and output-shaft angle.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9C` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:**
1. `temperature` — motor temperature (`int8_t`, 1°C/LSB).
2. `iq` — torque current (`int16_t`, 0.01 A/LSB).
3. `speed` — motor output-shaft speed (`int16_t`, 1 dps/LSB).
4. `degree` — motor output-shaft angle (`int16_t`, 1°/LSB, max range ±32767°).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9C` |
| `DATA[1]` | Motor temperature | `DATA[1] = (uint8_t)(temperature)` |
| `DATA[2]` | Torque current byte 1 (low) | `DATA[2] = (uint8_t)(iq)` |
| `DATA[3]` | Torque current byte 2 (high) | `DATA[3] = (uint8_t)(iq>>8)` |
| `DATA[4]` | Speed byte 1 (low) | `DATA[4] = (uint8_t)(speed)` |
| `DATA[5]` | Speed byte 2 (high) | `DATA[5] = (uint8_t)(speed>>8)` |
| `DATA[6]` | Angle byte 1 (low) | `DATA[6] = (uint8_t)(degree)` |
| `DATA[7]` | Angle byte 2 (high) | `DATA[7] = (uint8_t)(degree>>8)` |

**Communication example — Example 1:**

Send: CAN `0x141 0x9C 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Reply: CAN `0x241 0x9C 0x32 0x64 0x00 0xF4 0x01 0x2D 0x00`.

Description: `Data[1] = 0x32` = `50°C`. `Data[2]`/`Data[3]` form `0x0064` = `100` decimal, ×0.01 A/LSB → `1 A` actual current. `Data[4]`/`Data[5]` form `0x01F4` = `500` decimal → output-shaft speed `500 dps` (motor speed is `speed × gear ratio`; e.g. for a gear ratio of 6, motor speed is 6× the output-shaft speed). `Data[6]`/`Data[7]` form `0x002D` = `45` decimal → output shaft has moved `45°` positively relative to zero. (The output-shaft pulse count relates to encoder line count × gear ratio — e.g. 16384 lines × gear ratio 6 = 98304 pulses per 360° of output shaft.)

### 2.16. Read Motor Status 3 Command (0x9D)

**Instruction description:** Reads the current motor temperature and three-phase current data.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9D` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:**
1. `temperature` — motor temperature (`int8_t`, 1°C/LSB).
2. `iA`, `iB`, `iC` — phase A/B/C current (`int16_t`, 0.01 A/LSB).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x9D` |
| `DATA[1]` | Motor temperature | `DATA[1] = (uint8_t)(temperature)` |
| `DATA[2]` | Phase A current byte 1 (low) | `DATA[2] = (uint8_t)(iA)` |
| `DATA[3]` | Phase A current byte 2 (high) | `DATA[3] = (uint8_t)(iA>>8)` |
| `DATA[4]` | Phase B current byte 1 (low) | `DATA[4] = (uint8_t)(iB)` |
| `DATA[5]` | Phase B current byte 2 (high) | `DATA[5] = (uint8_t)(iB>>8)` |
| `DATA[6]` | Phase C current byte 1 (low) | `DATA[6] = (uint8_t)(iC)` |
| `DATA[7]` | Phase C current byte 2 (high) | `DATA[7] = (uint8_t)(iC>>8)` |

**Communication example — Example 1:**

Send: CAN `0x141 0x9D 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Reply: CAN `0x241 0x9D 0x32 0xC2 0x0B 0x10 0xFA 0xC0 0xF9`.

Description: `Data[1] = 0x32` = `50°C`. `Data[2]`/`Data[3]` form `0x0BC2` = `3010` decimal, ×0.01 A/LSB → phase A current `30.1 A`. `Data[4]`/`Data[5]` form `0xFA10` = `-1520` decimal → phase B current `-15.2 A`. `Data[6]`/`Data[7]` form `0xF9C0` = `-1600` decimal → phase C current `-16 A`.

### 2.17. Motor Shutdown Command (0x80)

**Instruction description:** Turns off the motor output and clears the running state; the motor is not in any closed-loop mode afterward.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x80` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** Same content as the sent data.

### 2.18. Motor Stop Command (0x81)

**Instruction description:** Stops the motor while leaving the current closed-loop mode active — only the motor speed is stopped.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x81` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** Same content as the sent data.

### 2.19. Torque Closed-Loop Control Command (0xA1)

**Instruction description:** A control command that runs whenever the motor is fault-free. The host uses it to control the motor's torque/current output. The control value `iqControl` is `int16_t`, unit 0.01 A/LSB.

For safety, this command cannot open the brake directly — first send `0x77` to release the brake, then send `0xA1`.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA1` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Torque current control byte 1 (low) | `DATA[4] = (uint8_t)(iqControl)` |
| `DATA[5]` | Torque current control byte 2 (high) | `DATA[5] = (uint8_t)(iqControl>>8)` |
| `DATA[6]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** Same layout as [2.15 (0x9C)](#215-read-motor-status-2-command-0x9c), with `DATA[0] = 0xA1`.

**Communication example:**

Example 1 — send: CAN `0x141 0xA1 0x00 0x00 0x00 0x64 0x00 0x00 0x00`.

Description: `Data[4]`/`Data[5]` form `0x0064` = `100` decimal, ×0.01 A/LSB → target current `1 A`.

Reply: CAN `0x241 0xA1 0x32 0x64 0x00 0xF4 0x01 0x2D 0x00`.

Description: `Data[1] = 50°C`; `Data[2]`/`Data[3]` = `1 A` actual current; `Data[4]`/`Data[5]` = `500 dps` output-shaft speed; `Data[6]`/`Data[7]` = `45°` output-shaft angle relative to zero (same reduction-ratio relationship as in 2.15).

Example 2 — send: CAN `0x141 0xA1 0x00 0x00 0x00 0x9C 0xFF 0x00 0x00`.

Description: `Data[4]`/`Data[5]` form `0xFF9C` = `-100` decimal, ×0.01 A/LSB → target current `-1 A`.

Reply: CAN `0x241 0xA1 0x32 0x9C 0xFF 0x0C 0xFE 0xD3 0xFF`.

Description: `Data[1] = 50°C`; actual current `-1 A`; output-shaft speed `-500 dps`; output-shaft angle `-45°` relative to zero.

### 2.20. Speed Closed-Loop Control Command (0xA2)

**Instruction description:** A control command that runs whenever the motor is fault-free. Controls the motor output-shaft speed. `speedControl` is `int32_t`, unit 0.01 dps/LSB. `maxTorque` limits the maximum torque of the output shaft — `uint8_t`, range 0–255, unit 1% of rated current per LSB. If the given current is `0` or exceeds the stall current, force-control mode is not activated and the maximum torque current is instead limited by the stall-current value set in the configuration software.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA2` |
| `DATA[1]` | Max torque | `DATA[1] = (uint8_t)(maxTorque)` |
| `DATA[2]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Speed control byte 1 (low) | `DATA[4] = (uint8_t)(speedControl)` |
| `DATA[5]` | Speed control byte 2 | `DATA[5] = (uint8_t)(speedControl>>8)` |
| `DATA[6]` | Speed control byte 3 | `DATA[6] = (uint8_t)(speedControl>>16)` |
| `DATA[7]` | Speed control byte 4 (high) | `DATA[7] = (uint8_t)(speedControl>>24)` |

**Remarks:**
1. The maximum torque current under this command is bounded by the **Max Torque Current** value set in the configuration software.
2. In this mode, maximum acceleration is bounded by the **Max Acceleration** value.
3. When the speed-loop acceleration value is `0`, the speed-loop acceleration is limited only by the maximum current output capability.

**Reply data field:** Same layout as [2.15 (0x9C)](#215-read-motor-status-2-command-0x9c), with `DATA[0] = 0xA2`.

**Communication example:**

Example 1 — send: CAN `0x141 0xA2 0x00 0x00 0x00 0x10 0x27 0x00 0x00`.

Description: `Data[4]`–`Data[7]` form `0x00002710` = `10000` decimal, ×0.01 dps/LSB → target speed `100 dps`.

Reply: CAN `0x241 0xA2 0x32 0x64 0x00 0xF4 0x01 0x2D 0x00` (same field meanings as [2.15](#215-read-motor-status-2-command-0x9c): `50°C`, `1 A`, `500 dps`, `45°`).

Example 2 — send: CAN `0x141 0xA2 0x00 0x00 0x00 0xF0 0xD8 0xFF 0xFF`.

Description: `Data[4]`–`Data[7]` form `0xFFFFD8F0` = `-10000` decimal → target speed `-100 dps`.

Reply: CAN `0x241 0xA2 0x32 0x9C 0xFF 0x0C 0xFE 0xD3 0xFF` (`50°C`, `-1 A`, `-500 dps`, `-45°`).

### 2.21. Absolute Position Closed-Loop Control Command (0xA4)

**Instruction description:** A control command that runs whenever the motor is fault-free. Controls the motor's multi-turn position. `angleControl` is `int32_t`, unit 0.01°/LSB (`36000` = `360°`); rotation direction follows the sign of (target − current) position. `maxSpeed` limits the output-shaft speed — `uint16_t`, unit 1 dps/LSB.

Depending on the configured position-planning acceleration, behavior differs:

1. If the position-loop acceleration is `0`, the position loop enters **direct tracking mode**, tracking the target position via a PI controller. `maxSpeed` caps the speed during this motion; if `maxSpeed = 0`, the output is entirely determined by the PI controller (see "Figure 2-1: Block Diagram of Position Tracking Mode with Speed Limit" in the source PDF — not reproduced here).
2. If the position-loop acceleration is non-zero, the motor runs a speed-planned motion profile with acceleration/deceleration; the maximum operating speed is `maxSpeed` and the acceleration is the configured position-loop acceleration.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA4` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | Speed limit byte 1 (low) | `DATA[2] = (uint8_t)(maxSpeed)` |
| `DATA[3]` | Speed limit byte 2 (high) | `DATA[3] = (uint8_t)(maxSpeed>>8)` |
| `DATA[4]` | Position control byte 1 (low) | `DATA[4] = (uint8_t)(angleControl)` |
| `DATA[5]` | Position control byte 2 | `DATA[5] = (uint8_t)(angleControl>>8)` |
| `DATA[6]` | Position control byte 3 | `DATA[6] = (uint8_t)(angleControl>>16)` |
| `DATA[7]` | Position control byte 4 (high) | `DATA[7] = (uint8_t)(angleControl>>24)` |

**Reply data field:** Same layout as [2.15 (0x9C)](#215-read-motor-status-2-command-0x9c), with `DATA[0] = 0xA4`.

**Communication example:**

Example 1 — send: CAN `0x141 0xA4 0x00 0xF4 0x01 0xA0 0x8C 0x00 0x00`.

Description: `Data[2]`/`Data[3]` form `0x01F4` = `500` decimal → max speed `500 dps` for the position loop. `Data[4]`–`Data[7]` form `0x00008CA0` = `36000` decimal, ×0.01°/LSB → `360°`. The motor moves the output shaft `+360°` relative to zero.

Reply: CAN `0x241 0xA4 0x32 0x64 0x00 0xF4 0x01 0x2D 0x00` (`50°C`, `1 A`, `500 dps`, `+45°`).

Example 2 — send: CAN `0x141 0xA4 0x00 0xF4 0x01 0x60 0x73 0xFF 0xFF`.

Description: `Data[4]`–`Data[7]` form `0xFFFF7360` = `-36000` decimal → `-360°`. The motor moves the output shaft `-360°` relative to zero.

Reply: CAN `0x241 0xA4 0x32 0x9C 0xFF 0x0C 0xFE 0xD3 0xFF` (`50°C`, `-1 A`, `-500 dps`, `-45°`).

### 2.22. Single-Turn Position Control Command (0xA6)

**Instruction description:** Controls the motor's single-turn position. When the multi-turn-save function is off, the default mode is single-turn, and this command applies in that mode.

1. `angleControl` (`uint16_t`, range 0–35999, unit 0.01°/LSB → 0°–359.99°).
2. `spinDirection` (`uint8_t`) — `0x00`: clockwise, `0x01`: counterclockwise.
3. `maxSpeed` (`uint16_t`, unit 1 dps/LSB) limits the rotation speed.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA6` |
| `DATA[1]` | Rotation direction byte | `DATA[1] = spinDirection` |
| `DATA[2]` | Speed limit byte 1 (low) | `DATA[2] = (uint8_t)(maxSpeed)` |
| `DATA[3]` | Speed limit byte 2 (high) | `DATA[3] = (uint8_t)(maxSpeed>>8)` |
| `DATA[4]` | Position control byte 1 (low) | `DATA[4] = (uint8_t)(angleControl)` |
| `DATA[5]` | Position control byte 2 (high) | `DATA[5] = (uint8_t)(angleControl>>8)` |
| `DATA[6]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:**
1. `temperature` (`int8_t`, 1°C/LSB).
2. `iq` — torque current (`int16_t`, 0.01 A/LSB).
3. `speed` — motor output-shaft speed (`int16_t`, 1 dps/LSB).
4. `encoder` — encoder value (`uint16_t`, range determined by the encoder's bit width).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA6` |
| `DATA[1]` | Motor temperature | `DATA[1] = (uint8_t)(temperature)` |
| `DATA[2]` | Torque current byte 1 (low) | `DATA[2] = (uint8_t)(iq)` |
| `DATA[3]` | Torque current byte 2 (high) | `DATA[3] = (uint8_t)(iq>>8)` |
| `DATA[4]` | Speed byte 1 (low) | `DATA[4] = (uint8_t)(speed)` |
| `DATA[5]` | Speed byte 2 (high) | `DATA[5] = (uint8_t)(speed>>8)` |
| `DATA[6]` | Encoder value byte 1 (low) | `DATA[6] = (uint8_t)(encoder)` |
| `DATA[7]` | Encoder value byte 2 (high) | `DATA[7] = (uint8_t)(encoder>>8)` |

**Communication example:**

Example 1 — send: CAN `0x141 0xA6 0x00 0xF4 0x01 0xA0 0x8C 0x00 0x00`.

Description: `Data[1] = 0` → clockwise. `Data[2]`/`Data[3]` form `0x01F4` = `500 dps` max speed. `Data[4]`/`Data[5]` form `0x8CA0` = `36000` decimal, unit 0.01° → the motor moves `360°` clockwise (coincides with `0°` in single-turn position, so the reported position may read `0°`).

Reply: CAN `0x241 0xA6 0x32 0x64 0x00 0xF4 0x01 0xE8 0x03`.

Description: `Data[1] = 50°C`; `Data[2]`/`Data[3]` = `1 A`; `Data[4]`/`Data[5]` = `500 dps`; `Data[6]`/`Data[7]` form `0x03E8` = `1000` decimal → encoder value relative to zero is `1000` pulses.

Example 2 — send: CAN `0x141 0xA6 0x01 0xF4 0x01 0xA0 0x8C 0x00 0x00`.

Description: `Data[1] = 1` → counterclockwise; otherwise identical to Example 1 (`500 dps` max speed, `360°` move).

Reply: CAN `0x241 0xA6 0x32 0x64 0x00 0xF4 0x01 0xE8 0x03` (same as Example 1's reply).

### 2.23. Incremental Position Closed-Loop Control Command (0xA8)

**Instruction description:** A control command that runs whenever the motor is fault-free. Controls the motor's incremental position (multi-turn angle), moving by the given position increment from the current position. `angleControl` is `int32_t`, unit 0.01°/LSB (`36000` = `360°`); rotation direction follows the sign of the increment. `maxSpeed` (`uint16_t`, unit 1 dps/LSB) limits the output-shaft rotation speed.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA8` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | Speed limit byte 1 (low) | `DATA[2] = (uint8_t)(maxSpeed)` |
| `DATA[3]` | Speed limit byte 2 (high) | `DATA[3] = (uint8_t)(maxSpeed>>8)` |
| `DATA[4]` | Position control byte 1 (low) | `DATA[4] = (uint8_t)(angleControl)` |
| `DATA[5]` | Position control byte 2 | `DATA[5] = (uint8_t)(angleControl>>8)` |
| `DATA[6]` | Position control byte 3 | `DATA[6] = (uint8_t)(angleControl>>16)` |
| `DATA[7]` | Position control byte 4 (high) | `DATA[7] = (uint8_t)(angleControl>>24)` |

**Reply data field:** Same layout as [2.15 (0x9C)](#215-read-motor-status-2-command-0x9c), with `DATA[0] = 0xA8`.

**Communication example:**

Example 1 — send: CAN `0x141 0xA8 0x00 0xF4 0x01 0xA0 0x8C 0x00 0x00`.

Description: `Data[2]`/`Data[3]` = `500 dps` max speed. `Data[4]`–`Data[7]` form `0x00008CA0` = `36000` decimal → `+360°` increment from the current position.

Reply: CAN `0x241 0xA8 0x32 0x64 0x00 0xF4 0x01 0x2D 0x00` (`50°C`, `1 A`, `500 dps`, `+45°`).

Example 2 — send: CAN `0x141 0xA8 0x00 0xF4 0x01 0x60 0x73 0xFF 0xFF`.

Description: `Data[4]`–`Data[7]` form `0xFFFF7360` = `-36000` decimal → `-360°` increment from the current position.

Reply: CAN `0x241 0xA8 0x32 0x9C 0xFF 0x0C 0xFE 0xD3 0xFF` (`50°C`, `-1 A`, `-500 dps`, `-45°`).

### 2.24. Force Control Position Closed-Loop Command (0xA9)

**Instruction description:** A control command that runs whenever the motor is fault-free. Controls the motor's multi-turn position with an explicit torque cap. `angleControl` is `int32_t`, unit 0.01°/LSB (`36000` = `360°`); direction follows the sign of (target − current) position. `maxSpeed` (`uint16_t`, 1 dps/LSB) limits the output-shaft speed. `maxTorque` (`uint8_t`, 0–255, unit 1% of rated current per LSB) limits the output-shaft torque; if the given current exceeds the stall current, force-control mode is not activated and torque is instead bounded by the configured stall-current value.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xA9` |
| `DATA[1]` | Max torque | `DATA[1] = (uint8_t)(maxTorque)` |
| `DATA[2]` | Speed limit byte 1 (low) | `DATA[2] = (uint8_t)(maxSpeed)` |
| `DATA[3]` | Speed limit byte 2 (high) | `DATA[3] = (uint8_t)(maxSpeed>>8)` |
| `DATA[4]` | Position control byte 1 (low) | `DATA[4] = (uint8_t)(angleControl)` |
| `DATA[5]` | Position control byte 2 | `DATA[5] = (uint8_t)(angleControl>>8)` |
| `DATA[6]` | Position control byte 3 | `DATA[6] = (uint8_t)(angleControl>>16)` |
| `DATA[7]` | Position control byte 4 (high) | `DATA[7] = (uint8_t)(angleControl>>24)` |

**Reply data field:** Same layout as [2.15 (0x9C)](#215-read-motor-status-2-command-0x9c), with `DATA[0] = 0xA9`.

**Communication example:**

Example 1 — send: CAN `0x141 0xA9 0x3C 0xF4 0x01 0xA0 0x8C 0x00 0x00`.

Description: `Data[1] = 0x3C` = `60` decimal → torque cap `60% × rated current`. `Data[2]`/`Data[3]` = `500 dps` max speed. `Data[4]`–`Data[7]` form `0x00008CA0` = `36000` decimal → `+360°` relative to zero.

Reply: CAN `0x241 0xA9 0x32 0x64 0x00 0xF4 0x01 0x2D 0x00` (`50°C`, `1 A`, `500 dps`, `+45°`).

Example 2 — send: CAN `0x141 0xA9 0x3C 0xF4 0x01 0x60 0x73 0xFF 0xFF`.

Description: same `60%` torque cap and `500 dps` max speed; `Data[4]`–`Data[7]` form `0xFFFF7360` = `-36000` decimal → `-360°` relative to zero.

Reply: CAN `0x241 0xA9 0x32 0x9C 0xFF 0x0C 0xFE 0xD3 0xFF` (`50°C`, `-1 A`, `-500 dps`, `-45°`).

### 2.25. System Operating Mode Acquisition (0x70)

**Instruction description:** Reads the current motor running mode.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x70` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** `runmode` (`uint8_t`) — one of three states: `0x01` current-loop mode, `0x02` speed-loop mode, `0x03` position-loop mode.

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x70` |
| `DATA[1]`–`DATA[6]` | NULL | `0x00` |
| `DATA[7]` | Motor operating mode | `DATA[7] = (uint8_t)(runmode)` |

**Communication example — Example 1:**

Send: CAN `0x141 0x70 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Reply: CAN `0x241 0x70 0x00 0x00 0x00 0x00 0x00 0x00 0x03`.

Description: `Data[7] = 0x03` — the system is currently in position-loop mode.

### 2.26. System Reset Command (0x76)

**Instruction description:** Resets the system program.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x76` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** The motor resets after receiving the command and does **not** send a reply.

**Communication example — Example 1:**

Send: CAN `0x141 0x76 0x00 0x00 0x00 0x00 0x00 0x00 0x00`. After sending, the system resets and the program restarts.

### 2.27. System Brake Release Command (0x77)

**Instruction description:** Releases the system holding brake; the motor becomes movable, unrestricted by the brake.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x77` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** Same content as the sent data.

### 2.28. System Brake Lock Command (0x78)

**Instruction description:** Closes (engages) the system holding brake; the motor is locked and cannot run. The brake remains engaged in this state after a power-off.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x78` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** Same content as the sent data.

### 2.29. System Runtime Read Command (0xB1)

**Instruction description:** Obtains the system running time in ms.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB1` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** `SysRunTime` (`uint32_t`, unit ms).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB1` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Runtime byte 1 (low) | `DATA[4] = (uint8_t)(SysRunTime)` |
| `DATA[5]` | Runtime byte 2 | `DATA[5] = (uint8_t)(SysRunTime>>8)` |
| `DATA[6]` | Runtime byte 3 | `DATA[6] = (uint8_t)(SysRunTime>>16)` |
| `DATA[7]` | Runtime byte 4 (high) | `DATA[7] = (uint8_t)(SysRunTime>>24)` |

**Communication example — Example 1:**

Send: CAN `0x141 0xB1 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Reply: CAN `0x241 0xB1 0x00 0x00 0x00 0x00 0x00 0x00 0x10`.

Description: `Data[4]`–`Data[7]` = `0x10000000` = `268435456` decimal — the system has run for `268435456 ms` (~74 hours) since restart/reset.

### 2.30. System Software Version Date Read Command (0xB2)

**Instruction description:** Gets the update date of the system software version.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB2` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** `VersionDate` (`uint32_t`), formatted as `YYYYMMDD` (e.g. `20211126`).

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB2` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Version date byte 1 (low) | `DATA[4] = (uint8_t)(VersionDate)` |
| `DATA[5]` | Version date byte 2 | `DATA[5] = (uint8_t)(VersionDate>>8)` |
| `DATA[6]` | Version date byte 3 | `DATA[6] = (uint8_t)(VersionDate>>16)` |
| `DATA[7]` | Version date byte 4 (high) | `DATA[7] = (uint8_t)(VersionDate>>24)` |

**Communication example — Example 1:**

Send: CAN `0x141 0xB2 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Reply: CAN `0x241 0xB2 0x00 0x00 0x00 0x2E 0x89 0x34 0x01`.

Description: `Data[4]`–`Data[7]` = `0x0134892E` = `20220206` decimal — software version date is February 6, 2022.

### 2.31. Communication Interruption Protection Time Setting Command (0xB3)

**Instruction description:** Sets the communication-interruption protection time in ms. If communication is interrupted for longer than the configured time, the drive cuts the brake-release output; stable, continuous communication must be re-established before it will run again. Writing `0` disables the protection function.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB3` |
| `DATA[1]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | `CanRecvTime_MS` byte 1 (low) | `DATA[4] = (uint8_t)(CanRecvTime_MS)` |
| `DATA[5]` | `CanRecvTime_MS` byte 2 | `DATA[5] = (uint8_t)(CanRecvTime_MS>>8)` |
| `DATA[6]` | `CanRecvTime_MS` byte 3 | `DATA[6] = (uint8_t)(CanRecvTime_MS>>16)` |
| `DATA[7]` | `CanRecvTime_MS` byte 4 (high) | `DATA[7] = (uint8_t)(CanRecvTime_MS>>24)` |

**Reply data field:** Same content as the sent data.

**Communication example:**

Example 1 — send: CAN `0x141 0xB3 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Note: all-zero value disables the protection function — if communication is interrupted, the motor continues executing the current command.

Reply: CAN `0x241 0xB3 0x00 0x00 0x00 0x00 0x00 0x00 0x00` (echoes the sent frame).

Example 2 — send: CAN `0x141 0xB3 0x00 0x00 0x00 0xE8 0x03 0x00 0x00`.

Description: `Data[4]`–`Data[7]` form `0x000003E8` = `1000` decimal — sets the interruption-protection time to `1000 ms`, stored in ROM (retained across power cycles). If the communication gap exceeds `1000 ms`, protection triggers and the brake-release output is cut; normal operation resumes once the gap returns to within `1000 ms`.

Reply: CAN `0x241 0xB3 0x00 0x00 0x00 0x00 0x00 0x00 0x00` (echoes the sent frame).

### 2.32. Communication Baud Rate Setting Command (0xB4)

**Instruction description:** Sets the CAN and RS485 bus baud rates. The parameters are saved to ROM and persist across power cycles; the drive runs at the new baud rate on the next power-up.

Baud rate codes:

- **RS485:** `0` = 115200 bps, `1` = 500 Kbps, `2` = 1 Mbps, `3` = 1.5 Mbps, `4` = 2.5 Mbps.
- **CAN:** `0` = 500 Kbps, `1` = 1 Mbps.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB4` |
| `DATA[1]`–`DATA[6]` | NULL | `0x00` |
| `DATA[7]` | Baud rate code | `DATA[7] = (uint8_t)baudrate` |

**Reply data field:** Since the baud rate itself changes, the reply frame is unpredictable and can be ignored.

**Communication example:**

Example 1 — send: CAN `0x141 0xB4 0x00 0x00 0x00 0x00 0x00 0x00 0x00`. `Data[7] = 0` — RS485 changes to `115200 bps`, CAN changes to `500 Kbps`.

Example 2 — send: CAN `0x141 0xB4 0x00 0x00 0x00 0x00 0x00 0x00 0x01`. `Data[7] = 1` — RS485 changes to `500 Kbps`, CAN changes to `1 Mbps`.

Example 3 — send: CAN `0x141 0xB4 0x00 0x00 0x00 0x00 0x00 0x00 0x02`. `Data[7] = 2` — RS485 changes to `1 Mbps`; this code is invalid for CAN.

### 2.33. Motor Model Reading Command (0xB5)

**Instruction description:** Reads the motor model as ASCII characters; look up the resulting bytes in an ASCII table to obtain the corresponding symbols.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB5` |
| `DATA[1]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB5` |
| `DATA[1]` | Motor model char 1 | `Type1` (ASCII) |
| `DATA[2]` | Motor model char 2 | `Type2` (ASCII) |
| `DATA[3]` | Motor model char 3 | `Type3` (ASCII) |
| `DATA[4]` | Motor model char 4 | `Type4` (ASCII) |
| `DATA[5]` | Motor model char 5 | `Type5` (ASCII) |
| `DATA[6]` | Motor model char 6 | `Type6` (ASCII) |
| `DATA[7]` | Motor model char 7 | `Type7` (ASCII) |

**Communication example — Example 1:**

Send: CAN `0x141 0xB5 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Reply: CAN `0x241 0xB5 0x58 0x38 0x53 0x32 0x56 0x31 0x30`.

Description: The 7 returned ASCII bytes (`0x58 0x38 0x53 0x32 0x56 0x31 0x30`) decode to `X8 S2 V10`, giving the motor model `RMD-X8 S2 V10`.

### 2.34. Active Reply Function Command (0xB6)

**Instruction description:** Selects one or more commands for the motor to actively (unsolicited) reply on a fixed schedule; multiple selected commands are cyclically alternated. Once an active-reply command is configured, the motor stops replying to that command upon receipt (only the scheduled push applies). CAN-only — not supported on RS485.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0xB6` |
| `DATA[1]` | Command to auto-reply | One of `0x60`, `0x61`, `0x62`, `0x92`, `0x9A`, `0x9C`, `0x9D`, `0x9E` |
| `DATA[2]` | Active-reply enable | `0`: disable active reply for this command; `1`: enable it |
| `DATA[3]` | Reply interval, low byte | Unit 10 ms. Commands alternate in a round-robin when multiple are enabled. |
| `DATA[4]` | Reply interval, high byte | (see `DATA[3]`) |
| `DATA[5]`–`DATA[7]` | NULL | `0x00` |

**Reply data field:** No reply is returned when enabling; instead, the motor actively pushes the selected command's reply content at the configured interval.

**Communication example — Example 1:**

Send: CAN `0x141 0xB6 0x60 0x01 0x01 0x00 0x00 0x00 0x00`.

> **Source note:** the manual's prose says "the time interval is 20ms" while the raw `Data[3]/Data[4] = 0x0001` (unit 10 ms/LSB) computes to `10 ms`, and then restates "reply 0x60 command at intervals of 10ms." This 20 ms/10 ms inconsistency is present in the original text; the byte-level math (`0x0001 × 10 ms = 10 ms`) is taken as authoritative.

Description: Enables active reply for `0x60` at a `10 ms` interval. After this, the motor does not reply to `0x60` when polled, but instead pushes a `0x60`-format reply every `10 ms`.

### 2.35. Function Control Command (0x20)

**Instruction description:** A compound command used to invoke specific auxiliary functions; a single command byte can carry multiple distinct function controls, selected by index. Avoid writing parameters immediately after motor start-up while it is in motion.

**Send data field:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x20` |
| `DATA[1]` | Function index | `DATA[1] = (uint8_t)index` |
| `DATA[2]`–`DATA[3]` | NULL | `0x00` |
| `DATA[4]` | Input parameter byte 1 (low) | `DATA[4] = (uint8_t)(Value)` |
| `DATA[5]` | Input parameter byte 2 | `DATA[5] = (uint8_t)(Value>>8)` |
| `DATA[6]` | Input parameter byte 3 | `DATA[6] = (uint8_t)(Value>>16)` |
| `DATA[7]` | Input parameter byte 4 (high) | `DATA[7] = (uint8_t)(Value>>24)` |

**Reply data field:** Same content as the sent data.

**Function index description:**

| Index value | Command name | Function description |
| --- | --- | --- |
| `0x01` | Clear multi-turn value | Clears the motor's multi-turn value, updates the zero point, and saves. Takes effect after restart. |
| `0x02` | CANID filter enable | `1`: enables the CANID filter (improves CAN send/receive efficiency); `0`: disables it (required when using the multi-motor commands `0x280`/`0x300`). Saved to FLASH; retained across power-off. |
| `0x03` | Error status transmission enable | `1`: enables the function — after an error occurs, the motor actively sends status command `0x9A` on the bus every 100 ms, stopping once the error clears. `0`: disables the function. |
| `0x04` | Save multi-turn value on power-off | `1`: enables the function — the motor saves its current multi-turn value before powering off. `0`: disables the function (system defaults to single-turn mode). Takes effect after restart. |
| `0x05` | Set CANID | The value is the new CANID to apply; saved to ROM, takes effect after reboot. |
| `0x06` | Set the maximum positive angle for position-operation mode | The value is the maximum positive angle for position mode; set and saved to ROM, effective immediately. |
| `0x07` | Set the maximum negative angle for position-operation mode | The value is the maximum negative angle for position mode; set and saved to ROM, effective immediately. |

**Communication example:**

Example 1 — send: CAN `0x141 0x20 0x01 0x00 0x00 0x00 0x00 0x00 0x00`.

Description: `Data[1] = 0x01` — per the index table, this clears the multi-turn value.

Reply: CAN `0x241 0x20 0x01 0x00 0x00 0x00 0x00 0x00 0x00` (echoes the sent frame).

Example 2 — send: CAN `0x141 0x20 0x02 0x00 0x00 0x01 0x00 0x00 0x00`.

Description: `Data[1] = 0x02` — enables the CANID filter. Note: the `0x280` multi-motor command cannot be used while the filter is enabled; disable the filter first before using `0x280` again.

Reply: CAN `0x241 0x20 0x02 0x00 0x00 0x01 0x00 0x00 0x00` (echoes the sent frame).

---

## 3. CAN Multi-Motor Command (0x280 + Command)

### 3.1. Instruction Description

Sending to ID `0x280` addresses all motors on the bus simultaneously with the same command. The content and function of the command are identical to the corresponding single-motor command.

### 3.2. Communication Example

Assume 4 motors on the CAN bus with IDs `1`, `2`, `3`, `4` (send addresses `0x141`–`0x144`).

**Example 1 — broadcast Motor Shutdown (0x80):**

Send: `0x280 0x80 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Description: All 4 motors receive the `0x80` motor-shutdown command (see [2.17](#217-motor-shutdown-command-0x80)) simultaneously and immediately execute it.

Reply: each motor replies independently on its own ID, in an order depending on bus arbitration delay:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |
| `0x242` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |
| `0x243` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |
| `0x244` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

**Example 2 — broadcast Read Multi-Turn Encoder Position Data (0x60):**

Send: `0x280 0x60 0x00 0x00 0x00 0x00 0x00 0x00 0x00`.

Description: All 4 motors receive the `0x60` command (see [2.6](#26-read-multi-turn-encoder-position-data-command-0x60)) and each replies with its own multi-turn encoder position.

Reply:

| ID | Data[0] | Data[1] | Data[2] | Data[3] | Data[4] | Data[5] | Data[6] | Data[7] | Encoder value |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x241` | `0x60` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | `10000` |
| `0x242` | `0x60` | `0x00` | `0x00` | `0x00` | `0x20` | `0x4E` | `0x00` | `0x00` | `20000` |
| `0x243` | `0x60` | `0x00` | `0x00` | `0x00` | `0x30` | `0x75` | `0x00` | `0x00` | `30000` |
| `0x244` | `0x60` | `0x00` | `0x00` | `0x00` | `0x40` | `0x9C` | `0x00` | `0x00` | `40000` |

(`Data[4]`–`Data[7]`, low byte first, form the 32-bit multi-turn encoder value in pulses, relative to each motor's own zero offset.)

---

## 4. CANID Setting Command (0x79)

### 4.1. Instruction Description

Sets or reads the CAN ID. Parameters:

1. `wReadWriteFlag` (`bool`) — `1` = read, `0` = write.
2. `CANID` (`uint16_t`, range 1–32) — device identifier, corresponding to `0x140 + ID`.

### 4.2. Send Data Field Definition

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x79` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | Read/write flag | `DATA[2] = wReadWriteFlag` |
| `DATA[3]`–`DATA[6]` | NULL | `0x00` |
| `DATA[7]` | CANID | `DATA[7] = CANID (1–32)` |

### 4.3. Reply Data Field Definition

Two cases:

1. **Set CANID** (range 1–32): the reply echoes the sent command.
2. **Read CANID:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x79` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | Read/write flag | `DATA[2] = wReadWriteFlag` |
| `DATA[3]`–`DATA[5]` | NULL | `0x00` |
| `DATA[6]` | CANID byte 1 (low) | `DATA[6] = (uint8_t)(CANID)` |
| `DATA[7]` | CANID byte 2 (high) | `DATA[7] = (uint8_t)(CANID>>8)` |

### 4.4. Communication Example

Example 1 — write CANID. Note: this command must be sent on the broadcast/config address `0x300` (rather than `0x140 + ID`) since the target ID is not yet known or is being changed.

Send: `0x300 0x79 0x00 0x00 0x00 0x00 0x00 0x00 0x02`.

Description: `Data[2] = 0` — write CANID. `Data[7] = 0x02` — sets the motor's CANID to `2`, i.e. its send address becomes `0x142` and reply address `0x242`.

Reply: `0x300 0x79 0x00 0x00 0x00 0x00 0x00 0x00 0x02` (same as sent).

Example 2 — read CANID.

Send: `0x300 0x79 0x00 0x01 0x00 0x00 0x00 0x00 0x00`.

Description: `Data[2] = 1` — read CANID.

Reply: `0x300 0x79 0x00 0x01 0x00 0x00 0x00 0x42 0x02`.

Description: `Data[6]`/`Data[7]` form `0x242` — the motor's send ID is `0x142` and reply ID is `0x242`.

---

## 5. Motion Mode Control Command_CAN (0x400 + ID)

### 5.1. Instruction Description

This is the MIT-style joint-control command, addressed to `0x400 + ID` with the reply on `0x500 + ID`. It packs 5 parameters into an 8-byte CAN frame:

- **`p_des`** (desired position): range **−12.5 to 12.5 rad**, encoded as `uint16_t` (16-bit, 0–65535). `0` → `-12.5 rad`, `65535` → `12.5 rad`, linearly interpolated.
- **`v_des`** (desired velocity): range **−45 to 45 rad/s**, encoded as a 12-bit unsigned value (0–4095). `0` → `-45 rad/s`, `4095` → `45 rad/s`.
- **`kp`** (position-error gain): range **0 to 500**, encoded as a 12-bit unsigned value (0–4095). `0` → `0`, `4095` → `500`.
- **`kd`** (velocity-error gain): range **0 to 5**, encoded as a 12-bit unsigned value (0–4095). `0` → `0`, `4095` → `5`.
- **`t_ff`** (feed-forward torque): range **−24 to 24 N·m**, encoded as a 12-bit unsigned value (0–4095). `0` → `-24 N·m`, `4095` → `24 N·m`.

Output current formula:

```
IqRef = [ kp * (p_des - p_fb_actual_position) + kd * (v_des - v_fb_actual_speed) + t_ff ] * KT_torque_coefficient
```

`IqRef` is the resulting output current command for the motor.

### 5.2. Send Data Field Definition (Big-endian byte order)

| Data field | Bit range | Field | Description | Data range |
| --- | --- | --- | --- | --- |
| `DATA[0]` | 7–0 | `p_des[15:8]` | `p_des` upper 8 bits | 16-bit total |
| `DATA[1]` | 7–0 | `p_des[7:0]` | `p_des` lower 8 bits | (see above) |
| `DATA[2]` | 7–0 | `v_des[11:4]` | `v_des` upper 8 bits | 12-bit total |
| `DATA[3]` | 7–4 | `v_des[3:0]` | `v_des` lower 4 bits | (see above) |
| `DATA[3]` | 3–0 | `kp[11:8]` | `kp` upper 4 bits | 12-bit total |
| `DATA[4]` | 7–0 | `kp[7:0]` | `kp` lower 8 bits | (see above) |
| `DATA[5]` | 7–0 | `kd[11:4]` | `kd` upper 8 bits | 12-bit total |
| `DATA[6]` | 7–4 | `kd[3:0]` | `kd` lower 4 bits | (see above) |
| `DATA[6]` | 3–0 | `t_ff[11:8]` | `t_ff` upper 4 bits | 12-bit total |
| `DATA[7]` | 7–0 | `t_ff[7:0]` | `t_ff` lower 8 bits | (see above) |

### 5.3. Reply Data Field Definition (Big-endian byte order)

| Data field | Bit range | Field | Description | Data range |
| --- | --- | --- | --- | --- |
| `DATA[0]` | 7–0 | `CANID[7:0]` | Device CAN address number | 8-bit |
| `DATA[1]` | 7–0 | `p_des[15:8]` | `p_des` upper 8 bits (actual position feedback) | 16-bit total |
| `DATA[2]` | 7–0 | `p_des[7:0]` | `p_des` lower 8 bits | (see above) |
| `DATA[3]` | 7–0 | `v_des[11:4]` | `v_des` upper 8 bits (actual velocity feedback) | 12-bit total |
| `DATA[4]` | 7–4 | `v_des[3:0]` | `v_des` lower 4 bits | (see above) |
| `DATA[4]` | 3–0 | `t_ff[11:8]` | `t_ff` upper 4 bits (actual torque feedback) | 12-bit total |
| `DATA[5]` | 7–0 | `t_ff[7:0]` | `t_ff` lower 8 bits | (see above) |
| `DATA[6]` | — | NULL | — | — |
| `DATA[7]` | — | NULL | — | — |

The reply frame carries only `CANID`, actual position, actual velocity, and actual torque — `kp`/`kd` are not echoed back.

### 5.4. Communication Example

**Example 1 — send command, ID `0x401`:**

| Data field | Byte value | Combined field value | Field range | Decoded result |
| --- | --- | --- | --- | --- |
| `DATA[0]` | `0xE6` | `p_des` = `0xE666` = `58982` | −12.5 to 12.5 rad (span 25 rad) | `p_des = (58982/65535)×25 + (−12.5) = 9.99 rad` |
| `DATA[1]` | `0x66` | (low byte of `p_des`, combined above) | | |
| `DATA[2]` | `0x82` | `v_des` = `0x82E` = `2094` | −45 to 45 rad/s (span 90 rad/s) | `v_des = (2094/4095)×90 + (−45) = 1.021 rad/s` |
| `DATA[3]` | `0xE0` | upper nibble `0xE` = `v_des` low nibble; lower nibble `0x0` = `kp` high nibble | | |
| `DATA[4]` | `0x52` | `kp` = `0x052` = `82` | 0 to 500 | `kp = (82/4095)×500 = 10.012` |
| `DATA[5]` | `0x33` | `kd` = `0x333` = `819` | 0 to 5 | `kd = (819/4095)×5 = 1` |
| `DATA[6]` | `0x3B` | upper nibble `0x3` = `kd` low nibble; lower nibble `0xB` = `t_ff` high nibble | | |
| `DATA[7]` | `0x55` | `t_ff` = `0xB55` = `2901` | −24 to 24 N·m (span 48 N·m) | `t_ff = (2901/4095)×48 + (−24) = 10.004 N·m` |

So the send frame is: `0x401 0xE6 0x66 0x82 0xE0 0x52 0x33 0x3B 0x55`.

**Reply command, ID `0x501`:**

| Data field | Byte value | Combined field value | Field range | Decoded result |
| --- | --- | --- | --- | --- |
| `DATA[0]` | `0x01` | `CANID` = `1` | 0–32 | Device address ID number |
| `DATA[1]` | `0xE6` | `p_des` = `0xE666` = `58982` | −12.5 to 12.5 rad (span 25 rad) | `p_des = (58982/65535)×25 + (−12.5) = 9.99 rad` |
| `DATA[2]` | `0x66` | (low byte of `p_des`, combined above) | | |
| `DATA[3]` | `0x82` | `v_des` = `0x82E` = `2094` | −45 to 45 rad/s (span 90 rad/s) | `v_des = (2094/4095)×90 + (−45) = 1.021 rad/s` |
| `DATA[4]` | `0xEB` | upper nibble `0xE` = `v_des` low nibble; lower nibble `0xB` = `t_ff` high nibble | | |
| `DATA[5]` | `0x55` | `t_ff` = `0xB55` = `2901` | −24 to 24 N·m (span 48 N·m) | `t_ff = (2901/4095)×48 + (−24) = 10.004 N·m` |
| `DATA[6]` | NULL | — | — | — |
| `DATA[7]` | NULL | — | — | — |

So the reply frame is: `0x501 0x01 0xE6 0x66 0x82 0xEB 0x55 0x00 0x00`.

> **Source note:** this reply-example table is on a heavily-wrapped PDF table (printed page "91/101", PDF page 100) where the layout extraction misattributed a couple of nibble values (e.g. showing `DATA[2] = 0x65` and splitting `DATA[4]` across lines as `0xE`/`B`). The reconstruction above follows directly from the field-bit-mapping in [5.3](#53-reply-data-field-definition-big-endian-byte-order) and is internally consistent (it reproduces the same `p_des`/`v_des`/`t_ff` decoded values stated in the source prose). Recommend a human cross-check against PDF page 100 if exact reply byte values matter.

---

## 6. RS485 Multi-Motor Command (0xCD + Command)

### 6.1. Instruction Description

Sending to RS485 address `0xCD` addresses all motors on the bus simultaneously with the same command. The content and function of the command are identical to the corresponding single-motor command — see [Section 2](#2-single-motor-command-description).

### 6.2. Communication Example

Assume 4 motors on the RS485 bus with IDs `01`, `02`, `03`, `04`.

**Example 1 — broadcast Motor Shutdown (0x80):**

Send: `0x3E 0xCD 0x08 0x80 0x00 0x00 0x00 0x00 0x00 0x00 0x00 CRC16L CRC16H`.

Description: All 4 motors receive the `0x80` motor-shutdown command (see [2.17](#217-motor-shutdown-command-0x80)) simultaneously and immediately execute it.

Reply: each motor replies on its own ID, in an order depending on bus delay:

| ID | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x01` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |
| `0x02` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |
| `0x03` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |
| `0x04` | `0x80` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` | `0x00` |

**Example 2 — broadcast Read Multi-Turn Encoder Position Data (0x60):**

Send: `0x3E 0xCD 0x08 0x60 0x00 0x00 0x00 0x00 0x00 0x00 0x00 CRC16L CRC16H`.

Description: All 4 motors receive the `0x60` command (see [2.6](#26-read-multi-turn-encoder-position-data-command-0x60)) and each replies with its own multi-turn encoder position.

Reply:

| ID | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | Encoder value |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `0x01` | `0x60` | `0x00` | `0x00` | `0x00` | `0x10` | `0x27` | `0x00` | `0x00` | `10000` |
| `0x02` | `0x60` | `0x00` | `0x00` | `0x00` | `0x20` | `0x4E` | `0x00` | `0x00` | `20000` |
| `0x03` | `0x60` | `0x00` | `0x00` | `0x00` | `0x30` | `0x75` | `0x00` | `0x00` | `30000` |
| `0x04` | `0x60` | `0x00` | `0x00` | `0x00` | `0x40` | `0x9C` | `0x00` | `0x00` | `40000` |

---

## 7. RS485-ID Setting Command (0x79)

### 7.1. Instruction Description

Sets or reads the RS485 device ID. This command is sent to the RS485 broadcast address `0xCD`, so **all** devices on the bus receive and process it — take care when multiple devices are connected, as multiple IDs could be changed to the same value simultaneously.

Parameters:

1. `wReadWriteFlag` (`bool`) — `1` = read, `0` = write.
2. `RS485-ID` (`uint16_t`, range 1–32) — device identifier.

### 7.2. Send Data Field Definition

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x79` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | Read/write flag | `DATA[2] = wReadWriteFlag` |
| `DATA[3]`–`DATA[6]` | NULL | `0x00` |
| `DATA[7]` | RS485 ID | `DATA[7] = RS485ID (1–32)` |

### 7.3. Reply Data Field Definition

Two cases:

1. **Set RS485 ID** (range 1–32): the reply echoes the sent command.
2. **Read RS485 ID:**

| Data field | Description | Data |
| --- | --- | --- |
| `DATA[0]` | Command byte | `0x79` |
| `DATA[1]` | NULL | `0x00` |
| `DATA[2]` | Read/write flag | `DATA[2] = wReadWriteFlag` |
| `DATA[3]`–`DATA[5]` | NULL | `0x00` |
| `DATA[6]` | RS485 ID byte 1 (low) | `DATA[6] = (uint8_t)(RS485ID)` |
| `DATA[7]` | RS485 ID byte 2 (high) | `DATA[7] = (uint8_t)(RS485ID>>8)` |

### 7.4. Communication Example

Example 1 — write RS485 ID.

Send: `0x3E 0xCD 0x08 0x79 0x00 0x00 0x00 0x00 0x00 0x00 0x02 CRC16L CRC16H`.

Description: `Data[2] = 0` — write RS485 ID. `Data[7] = 0x02` — sets the motor's RS485 ID to `2`.

Reply: `0x3E 0xCD 0x08 0x79 0x00 0x00 0x00 0x00 0x00 0x00 0x02 CRC16L CRC16H` (same as sent).

Example 2 — read RS485 ID.

Send: `0x3E 0xCD 0x08 0x79 0x00 0x01 0x00 0x00 0x00 0x00 0x00 CRC16L CRC16H`.

Description: `Data[2] = 1` — read RS485 ID.

Reply: `0x3E 0xCD 0x08 0x79 0x00 0x01 0x00 0x00 0x00 0x00 0x02 CRC16L CRC16H`.

Description: `Data[7] = 0x2` — the motor's send/reply ID is `0x2`.

---

## 8. Indicator Light Description

### 8.1. Status Description

- **Solid on:** the motor is running normally.
- **Slow flashing:** the motor has a second-level (recoverable) error. Operation resumes automatically and the light returns to solid-on once the recovery condition is met.
- **Fast flashing:** the motor has a first-level (unrecoverable) error. The motor cannot recover on its own — the fault must be investigated and the motor restarted before it can run again.

### 8.2. Failure Description Table

| Fault name | Description | Error level |
| --- | --- | --- |
| Hardware over-current | Motor current exceeds the limit value; may indicate a short circuit, phase loss, loss of control, motor damage, etc. | Level 1 |
| Stall error | Current has reached the stall current and speed remains very low for a sustained period, indicating the motor load is too large. | Level 1 |
| Under-voltage error | Power input is below the configured under-voltage threshold. | Level 2 |
| Over-voltage error | Power input is above the configured over-voltage threshold. | Level 2 |
| Phase-current over-current | Software detects motor current exceeding the limit value; may indicate a short circuit, phase loss, loss of control, motor damage, etc. | Level 1 |
| Power overrun error | Power-supply input current exceeds the limit value; may indicate excessive load or excessive speed. | Level 2 |
| Calibration parameter read error | Parameter write failed, causing parameter loss. | Level 1 |
| Over-speed error | Motor running speed exceeds the limit value; may indicate over-voltage or excessive dragging load. | Level 2 |
| Motor over-temperature error | Motor temperature exceeds the configured threshold; may indicate a short circuit, parameter error, or prolonged overload use. | Level 2 |
| Encoder calibration error | The encoder calibration result deviates too far from the standard value. | Level 2 |

---

## 9. Version Revision Information

- **V3.1** (2022.6.23): Revised the reply-data definition in the operation control command (section 5.0 at the time).
- **V3.2** (2022.7.27): Added the indicator-light description.
- **V3.3** (2022.7.31): Added a function to the Function Control Command (`0x20`) — CAN filter disable control.
- **V3.4** (2022.8.17): Added position-tracking command `0xA3`; added the 4-value position/speed-planning acceleration and deceleration settings in the `0x43` command.
- **V3.5** (2022.9.05): Added position-tracking command `0xA5` with speed limit; added Function Control Command (`0x20`) features for error-status sending and multi-turn-value power-down-save selection; added command `0xB5` to read the motor model.
- **V3.6** (2022.10.13): Added the RS485 broadcast command description (`0xCD`).
- **V3.7** (2022.11.26): Removed command `0xA3`; merged `0xA5` into `0xA4`; added `0xA6` single-turn position command; added command `0x90` to read the single-turn encoder; added command `0x94` to read the motor's single-turn angle.
- **V3.8** (2022.11.26): Changed the RS485 protocol's "2 Mbps" baud rate designation to "2.5 Mbps."
- **V3.9** (2023.3.11): Added RS485 serial-port configuration instructions; added the function index to the `0x42` command (read position/speed acceleration and deceleration values by index); added the `0xB6` active-reply function.
- **V4.0** (2023.10.16): Added the CANID-setting function to the `0x20` command; added the maximum positive angle limit to the `0x20` command; added the maximum negative angle limit to the `0x20` command.
- **V4.1** (2024.2.13): Changed the motor's `circleAngle` field to `uint16_t` type.
- **V4.2** (2024.5.28): Changed the Read PID Parameters command (`0x30`) to use an index to read current/speed/position loop PID parameters; changed the Write PID Parameters to RAM command (`0x31`) to use an index to write current/speed/position loop PID parameters; changed the Write PID Parameters to ROM command (`0x32`) to use an index to write current/speed/position loop PID parameters.
- **V4.3** (2024.5.12): Added the Force Control Position Closed-Loop Control Command (`0xA9`); added force-control mode to the Speed Closed-Loop Control Command (`0xA2`); improved the MIT command; added the encoder-data-abnormal error.
