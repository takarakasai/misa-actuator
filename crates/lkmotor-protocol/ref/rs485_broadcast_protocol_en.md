# Shanghai Lingkong Technology — Motor RS485 Broadcast Communication Protocol

**Version:** V2.35

**Note**: The source PDF's internal metadata title reads "Chapter 5: RS485 Broadcast Protocol V2.35" — this is a short, standalone chapter (3 pages) from a larger manual, not a complete document in its own right. It describes a **broadcast mode** that is distinct from, and additional to, the single-motor RS485 protocol documented in `rs485_protocol_desc_en.md` (sibling reference project, vendor doc V2.36).

---

## Table of Contents

- [Broadcast Mode RS485 Bus Parameters](#broadcast-mode-rs485-bus-parameters)
- [Broadcast Mode Commands](#broadcast-mode-commands)
  - [2.1 Torque / Open-loop Control Command](#21-torque--open-loop-control-command)
  - [2.2 Speed Control Command](#22-speed-control-command)
  - [2.3 Position Control Command](#23-position-control-command)
  - [2.4 Hybrid Command](#24-hybrid-command)
- [Other](#other)

---

## Broadcast Mode RS485 Bus Parameters

- **Bus interface:** RS485
- **Baud rate:** 1 Mbps, 2 Mbps, 4 Mbps
- **Data bits:** 8
- **Parity:** None
- **Stop bit:** 1

---

## Broadcast Mode Commands

The commands described in this protocol are for high-speed, broadcast-style communication: a single command frame can simultaneously control up to 4 motors.

- Broadcast mode must be enabled in the host software, and the RS485 baud rate must be set to 1 Mbps or higher.
- The command length sent by the host is always 11 bytes.
- To prevent bus collisions, each drive must be set to a distinct ID — 1, 2, 3, or 4 (if fewer than 4 motors are present, the unused IDs may simply be omitted from the bus). The ID can be selected via the DIP switch on the driver board, or set from the host software.
- The motor determines frame boundaries by observing bus idle time, so the host must send the 11 bytes of the command frame back-to-back without gaps.
- The host sends the command in broadcast form. Each driver board executes the command on receipt and, after a short delay, sends a reply back to the host in ID order (the lowest ID replies first).
- Every command consists of 3 parts: **Head + CMD + data + checksum**, as follows:

| Field | Length | Description |
| --- | --- | --- |
| Head | 1 byte | `0x02` |
| CMD | 1 byte | Command byte |
| data | 8 bytes | Data accompanying the command |
| checksum | 1 byte | Sum of all bytes from Head through data; high bits discarded (i.e. low byte of the sum) |

The motor currently supports the following commands:

| No. | Command | Command byte |
| --- | --- | --- |
| 1 | Torque / open-loop control command | `0x80` |
| 2 | Speed control command | `0x81` |
| 3 | Position control command | `0x82` |
| 4 | Hybrid command | `0x88` |

### 2.1 Torque / Open-loop Control Command

Simultaneously carries the torque-current control values (MF, MG series) or open-loop voltage control values (MS series) for 4 motors. The control quantity `torqueValue` is 16-bit signed data. For MF/MG motors the data range is -2000 to +2000; for MS motors the range is -850 to +850.

| Field | Description | Note |
| --- | --- | --- |
| head | `0x02` | |
| CMD | `0x80` | |
| `data[0]` | Motor #1 `torqueValue` low byte | |
| `data[1]` | Motor #1 `torqueValue` high byte | |
| `data[2]` | Motor #2 `torqueValue` low byte | |
| `data[3]` | Motor #2 `torqueValue` high byte | |
| `data[4]` | Motor #3 `torqueValue` low byte | |
| `data[5]` | Motor #3 `torqueValue` high byte | |
| `data[6]` | Motor #4 `torqueValue` low byte | |
| `data[7]` | Motor #4 `torqueValue` high byte | |
| checksum | Sum of all bytes above | |

Example: the host sends torque current 100 to motor #1 and torque current -100 to motor #3. The command data is (HEX):

```
02 80 64 00 00 00 9C FF 00 00 81
```

**Torque / open-loop control command — drive reply:** Same as the single-motor torque control command reply.

### 2.2 Speed Control Command

Simultaneously carries the speed control values for 4 motors. The control quantity `speedValue` is 16-bit signed data. Resolution is 1 dps/LSB; because of the data-length limit, the `speedValue` range is -32768 to 32767 dps.

| Field | Description | Note |
| --- | --- | --- |
| head | `0x02` | |
| CMD | `0x81` | |
| `data[0]` | Motor #1 `speedValue` low byte | |
| `data[1]` | Motor #1 `speedValue` high byte | |
| `data[2]` | Motor #2 `speedValue` low byte | |
| `data[3]` | Motor #2 `speedValue` high byte | |
| `data[4]` | Motor #3 `speedValue` low byte | |
| `data[5]` | Motor #3 `speedValue` high byte | |
| `data[6]` | Motor #4 `speedValue` low byte | |
| `data[7]` | Motor #4 `speedValue` high byte | |
| checksum | Sum of all bytes above | |

Example: the host sends speed 360 dps to motor #2 and speed -720 dps to motor #4. The command data is (HEX):

```
02 81 00 00 68 01 00 00 30 FD 19
```

**Speed control command — drive reply:** Same as the single-motor speed control command reply.

### 2.3 Position Control Command

Simultaneously carries the absolute position control values for 4 motors. The control quantity `angleValue` is 16-bit signed data. Resolution is 0.01 degree/LSB; because of the data-length limit, the `angleValue` range is -327.68 to 327.67°.

| Field | Description | Note |
| --- | --- | --- |
| head | `0x02` | |
| CMD | `0x82` | |
| `data[0]` | Motor #1 `angleValue` low byte | |
| `data[1]` | Motor #1 `angleValue` high byte | |
| `data[2]` | Motor #2 `angleValue` low byte | |
| `data[3]` | Motor #2 `angleValue` high byte | |
| `data[4]` | Motor #3 `angleValue` low byte | |
| `data[5]` | Motor #3 `angleValue` high byte | |
| `data[6]` | Motor #4 `angleValue` low byte | |
| `data[7]` | Motor #4 `angleValue` high byte | |
| checksum | Sum of all bytes above | |

Example: the host sends angle 180° to motor #1 and angle -90° to motor #4. The command data is (HEX):

```
02 82 50 46 00 00 00 00 D8 DC CE
```

**Position control command — drive reply:** Same as single-motor position control command 1's reply.

### 2.4 Hybrid Command

Simultaneously carries several different commands, one per motor, for up to 4 motors. The command each motor executes is determined by its `motorCmd` byte.

| Field | Description | Note |
| --- | --- | --- |
| head | `0x02` | |
| CMD | `0x88` | |
| `data[0]` | Motor #1 `motorCmd` byte | |
| `data[1]` | `0x00` | |
| `data[2]` | Motor #2 `motorCmd` byte | |
| `data[3]` | `0x00` | |
| `data[4]` | Motor #3 `motorCmd` byte | |
| `data[5]` | `0x00` | |
| `data[6]` | Motor #4 `motorCmd` byte | |
| `data[7]` | `0x00` | |
| checksum | Sum of all bytes above | |

**Note**: The odd-indexed bytes (`data[1]`, `data[3]`, `data[5]`, `data[7]`) are fixed at `0x00` — the source manual does not describe them as carrying any per-motor sub-parameter, so they appear to be padding to keep each motor's slot 2 bytes wide (mirroring the layout of the other three broadcast commands).

`motorCmd` supports the following commands:

| No. | Command | `motorCmd` byte |
| --- | --- | --- |
| 1 | Read motor status 1 and error flags | `0x9A` |
| 2 | Clear motor error flags | `0x9B` |
| 3 | Read motor status 2 | `0x9C` |
| 4 | Motor off | `0x80` |
| 5 | Motor on | `0x88` |
| 6 | Motor stop | `0x81` |

Example: the host sends "read status 2" to motor #1 and "stop" to motor #4. The command data is (HEX):

```
02 88 9C 00 00 00 00 00 81 00 CF
```

**Note**: Recomputing the checksum per the algorithm given above (`head + CMD + data[0..7]`, low byte of the sum) over `02 88 9C 00 00 00 00 00 81 00` yields `0xA7`, not the `0xCF` shown in the source manual. The other three worked examples in this document (torque, speed, position) all check out exactly against the stated algorithm, so this appears to be a typo in the vendor's original example rather than an error in the algorithm description. Verify against real hardware before relying on this exact byte sequence in a test.

**Hybrid command — drive reply:** Same as the corresponding single-motor command's reply.

---

## Other

- After the host finishes sending a command, it must release control of the bus so the driver board can reply with an ACK.
- Because of bus speed and timing constraints:
  - At 1 Mbps, the maximum command frequency for simultaneously controlling 4 motors is approximately 1 kHz.
  - At 2 Mbps, the maximum command frequency for simultaneously controlling 4 motors is approximately 2 kHz.
  - At 4 Mbps, the maximum command frequency for simultaneously controlling 4 motors is approximately 3.5 kHz.
- Because the bus runs fast, it is recommended that the host use DMA to read the bus automatically, to avoid losing driver-reply data due to excessive polling latency or overly frequent interrupts.
