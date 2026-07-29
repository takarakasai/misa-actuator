# DM-J4310-2EC V1.2 User Manual

**Document:** DM-J4310-2EC V1.2 User Manual V1.2
**Publisher:** Shenzhen Damiao (DAMIAO 达妙科技) Technology Co., Ltd.
**Manual revision date:** 2025-11-21

## Revision History

| Date | Version | Changes |
| --- | --- | --- |
| 2025-11-21 | V1.2 | New Table of Contents Page |

---

## Table of Contents

- [Disclaimer](#disclaimer)
- [Safety Precautions](#safety-precautions)
- [Motor Features](#motor-features)
- [Naming Conventions](#naming-conventions)
- [Specifications](#specifications)
  - [Operating Voltage](#operating-voltage)
  - [Maximum Phase Current](#maximum-phase-current)
  - [Maximum Rotational Speed](#maximum-rotational-speed)
  - [Torque Constant](#torque-constant)
- [Torque–Speed (T–N) Curve](#torquespeed-tn-curve)
- [Packing List](#packing-list)
- [Interface & Pin Description](#interface--pin-description)
- [Motor Dimensions and Mounting](#motor-dimensions-and-mounting)
- [LED Status](#led-status)
- [Operating Modes](#operating-modes)
  - [MIT Mode](#mit-mode)
  - [Position-Velocity Mode](#position-velocity-mode)
  - [Velocity Mode](#velocity-mode)
  - [Force-Position Hybrid Mode](#force-position-hybrid-mode)
  - [Mode Switching (host-PC / general)](#mode-switching-host-pc--general)
- [CAN Communication](#can-communication)
  - [CAN Baud Rate Configuration (overview)](#can-baud-rate-configuration-overview)
  - [Feedback Frames](#feedback-frames)
  - [Control Frame in MIT Mode](#control-frame-in-mit-mode)
  - [Control Frame in Position Velocity Mode](#control-frame-in-position-velocity-mode)
  - [Control Frame in Velocity Mode](#control-frame-in-velocity-mode)
  - [Control Frame in Force-Position Hybrid Mode](#control-frame-in-force-position-hybrid-mode)
  - [Enable](#enable)
  - [Disable](#disable)
  - [Set Zero Position](#set-zero-position)
  - [Clear Faults](#clear-faults)
  - [Read Parameters](#read-parameters)
  - [Write Parameters](#write-parameters)
  - [Save Parameters](#save-parameters)
  - [Mode Switching (register-based)](#mode-switching-register-based)
  - [CAN Baud Rate Configuration (register-based)](#can-baud-rate-configuration-register-based)
  - [Register Map](#register-map)
- [Motor Debugging Procedure](#motor-debugging-procedure)
  - [Device Connection](#device-connection)
  - [Motor-side Encoder Calibration](#motor-side-encoder-calibration)
  - [Parameter Identification](#parameter-identification)
  - [Output Shaft Encoder Calibration (Dual Encoder Motors)](#output-shaft-encoder-calibration-dual-encoder-motors)
  - [Parameter Management](#parameter-management)
  - [Debugging via CAN](#debugging-via-can)
- [Firmware Version Check and Upgrade](#firmware-version-check-and-upgrade)
  - [Version Check](#version-check)
  - [Firmware Upgrade via Serial Port](#firmware-upgrade-via-serial-port)
  - [CAN Firmware Upgrade](#can-firmware-upgrade)

**Note on duplicate section titles:** The source manual's own Table of Contents lists "Mode Switching" and "CAN Baud Rate Configuration" twice each — once as a general/host-PC-oriented subsection under *Operating Modes* / *CAN Communication*, and again as a register-driven subsection later in *CAN Communication*. This transcription preserves that duplication faithfully but disambiguates the heading anchors (`(host-PC / general)` vs. `(register-based)`) purely so links resolve; the body text and heading style otherwise match the source.

---

## Disclaimer

Thank you for choosing the DAMIAO DM-J4310-2EC V1.2 Gear Motor (hereinafter referred to as the "Motor") from DAMIAO Technology.

Before using this product, please read this manual and all safety precautions provided by DAMIAO Technology and follow them strictly. Failure to do so may result in personal injury, product damage, or damage to surrounding property. By using this product, you acknowledge that you have read, understood, and agreed to all terms and conditions in this document and all related documentation. You agree to use this product only for lawful and intended purposes. You assume full responsibility for the use of this product and any consequences arising from it. DAMIAO Technology shall not be liable for any direct or indirect damage, injury, or legal liability arising from the use of this product.

DAMIAO is a trademark of Shenzhen Damiao Technology Co., Ltd. All product names and brands mentioned herein are the property of their respective owners. This manual is protected under applicable copyright laws. All rights are reserved. No part of this document may be reproduced in any form or by any means without prior written permission from Shenzhen DAMIAO Technology Co., Ltd. Shenzhen Damiao Technology Co., Ltd. reserves the right of final interpretation of this document and all related materials. This document is subject to change without prior notice.

---

## Safety Precautions

1. Operate the motor strictly within the specified environmental conditions and maximum winding temperature limits. Failure to do so may result in permanent damage.
2. Prevent foreign objects from entering the rotor. Otherwise, abnormal operation may occur.
3. Inspect all components before use. Do not operate the motor if any components are missing, worn, or damaged.
4. Ensure correct wiring and proper, secure motor installation.
5. Do not touch the rotating or energized parts during operation to avoid injury. High torque operation may generate heat. Avoid contact to prevent burns.
6. Do not disassemble the motor. Unauthorized disassembly may affect control accuracy or cause malfunction.

---

## Motor Features

1. Dual encoders provide single-turn absolute position feedback at the output shaft, ensuring position information is retained even after power loss.
2. Integrated motor and driver design offers compact structure with high integration.
3. Supports PC-based configuration, monitoring, and firmware upgrades.
4. Provides real-time feedback of motor velocity, position, torque, and temperature via CAN bus.
5. Features dual temperature protection mechanisms.
6. Supports trapezoidal acceleration and deceleration profiles in position mode.

---

## Naming Conventions

The manual presents the naming scheme as a labeled diagram of the part-number template `DM-J43 40 X-2EC VX.X (48V)` for the **Joint Motor** / **Separable Motor** families. Transcribed as a field breakdown:

```
DM - J 43 40 X - 2EC VX.X (48V)
 |   |  |  |  |    |    |    |
 |   |  |  |  |    |    |    +-- Drive rated voltage
 |   |  |  |  |    |    +------- Motor version
 |   |  |  |  +----+------------ Encoder
 |   |  |  +--------------------- Output bearing / special version
 |   |  +------------------------ Gear ratio
 |   +--------------------------- Stator diameter (mm)
 +------------------------------- Product line
```

| Field | Values / meaning |
| --- | --- |
| Brand | Damiao Technology |
| Product Line | `J`: Joint Motor Series; `S`: Separable Motor Series |
| Stator Diameter (mm) | 35, 43, 60, 62, 80, 100 |
| Gear Ratio | 06, 07, 09, 10, 19, 40, 48, etc. |
| Output Bearing / Special Versions | `X` represents letters such as `L`, `P`, etc. Default: deep groove ball bearing. `P`: crossed roller bearing. `L`: Lite version (not related to bearing type) |
| Encoder | Default: single encoder. `1EC`: single encoder, CAN communication. `2EC`: dual encoders (single-turn absolute, output shaft), CAN communication. `1EE`: single encoder, EtherCAT communication. `2EE`: dual encoders (single-turn absolute, output shaft), EtherCAT communication |
| Motor Version | `VX.X`: upgrade version (X = 0–9); version omitted = V1.0 |
| Drive rated voltage | Default: 24 V drive; `48V`: 48 V drive |

The motor covered by this manual, DM-J4310-2EC V1.2, therefore decodes as: Joint Motor series, 43 mm stator, 10:1 gear ratio, default bearing, dual encoders with CAN communication, upgrade version 1.2.

---

## Specifications

Please operate the motor within the specifications listed below.

| | | DM-J4310-2EC V1.2 (24V) | DM-J4310-2EC V1.2 (48V) |
| --- | --- | --- | --- |
| **Motor Specifications** | Model | DM-J4310-2EC V1.2（24V） | DM-J4310-2EC V1.2（48V） |
| | Rated Voltage | 24V | 48V |
| | Rated Phase Current / Rated Supply Current | 4.9A/3.1A@24V | 4.8A/1.6A@48V |
| | Peak Phase Current / Peak Supply Current | 20A/16.5A@24V | 20A/12.8A@48V |
| | Rated Torque | 3.5NM | 3.5NM |
| | Peak Torque | 12.5NM | 12.5NM |
| | Rated Speed | 120rpm | 120rpm |
| | Maximum No-Load Speed | 200rpm | 450rpm |
| **Motor characteristics** | Reduction Ratio | 10:1 | 10:1 |
| | Pole Pairs | 14 | 14 |
| | Phase Inductance | ~320uH | ~320uH |
| | Phase Resistance | ~580mΩ | ~580mΩ |
| **Mechanical & Physical** | Outer Diameter | 57mm | 57mm |
| | Height | 46mm | 46mm |
| | Weight | ~325g | ~325g |
| **Encoder** | Resolution (bits) | 16 bits | 16 bits |
| | Number Of Encoders | 2 | 2 |
| | Type | Magnetic Encoder (Single-turn) | Magnetic Encoder (Single-turn) |
| **Communication Interfaces** | Control Interface | CAN@1Mbps | CAN@1Mbps |
| | Configuration Interface | UART@921600bps | UART@921600bps |
| **Control & Protection** | Control Modes | MIT Mode / Position-Velocity Mode / Velocity Mode / Force-Position Hybrid Mode | (same) |
| | Protection | See protection table below | (same) |

**Protection details** (from the "Control & Protection" row, transcribed in full):

| Protection | Description |
| --- | --- |
| Drive over-temperature protection | Shutdown at 120° C; the motor exits Enable Mode when triggered. |
| Motor over-temperature protection | Configurable, recommended ≤ 100° C; the motor exits Enable Mode when triggered. |
| Over-voltage protection | Configurable, recommended ≤ 32V; exits Enable Mode when triggered. |
| Communication Loss Protection | The motor exits Enable Mode if no CAN command is received within the specified timeout period. |
| Over-current protection | Configurable, recommended ≤ 9.8A; exits Enable Mode when triggered. |
| Under-voltage protection | The motor exits Enable Mode if supply voltage falls below the threshold (recommended ≥ 15V). |

### Operating Voltage

The operating voltage range for the 24V motor driver is 24V–28V, with a minimum operating voltage of 20V and a maximum of 28V.

For the 48V motor driver, the operating voltage range is 24V–48V, with a minimum operating voltage of 20V and a maximum of 58V. Hot-plugging is recommended to be avoided when the voltage exceeds 36V.

### Maximum Phase Current

**Correction (printed in source):** The baud rate in the communication settings in the PC software shown in the screenshot has been updated to 921600.

The maximum phase current of the driver can be obtained from the startup serial output (example values shown in the source screenshot: `Imax: 20.522388`, `CAN ID: 0x000`, `MASTER ID: 0x000`, `CAN Baud: 1.00Mbps`, `Rs = 593.7970 mΩ`, `Ls = 204.7071 μH`, `Ψf = 0.0045 Wb`, `V_BUS = 24.0666`).

The maximum phase current percentage can be limited by setting a percentage in the configuration tool. The default value is 0.8 (i.e., 80% of the measurable maximum current). It is recommended that this value not exceed 98%.

### Maximum Rotational Speed

The maximum rotational speed is limited by multiple factors, including the power supply voltage (V<sub>BUS</sub>), rotor flux linkage (ψ<sub>f</sub>), and gear ratio (GR). An upper limit can typically be calculated using the following formula:

```
V_MAX(rad/s) = 0.57735 * V_BUS / (Npp * GR * ψf)   (rad/s)
```

where V<sub>BUS</sub> is the supply voltage, N<sub>pp</sub> is the number of pole pairs, and ψ<sub>f</sub> is the rotor flux linkage.

### Torque Constant

The torque constant of the motor can be considered constant within its rated range. With a gearbox, it can be calculated using the following formula:

```
Kt = 1.5 * Npp * ψf * GR * GREF
```

Where N<sub>pp</sub> is the number of pole pairs, ψ<sub>f</sub> is the rotor flux linkage, GR is the gear ratio, and GREF is the gearbox torque transmission coefficient.

---

## Torque–Speed (T–N) Curve

The manual shows a measured performance chart captured at 24 V, a constant speed of 36 rpm, and an ambient temperature of 25° C, plotting the following quantities against output torque (Tor, N·m, x-axis range roughly 0–4.5 N·m):

- **Vbus (V)** — bus voltage, roughly flat around 23–25 V across the torque sweep.
- **Ibus (A)** — bus current, rising from 0 toward the top of its ~0–5 A scale as torque increases.
- **Speed (rpm)** — plotted on a 116–120 rpm scale; rises quickly at low torque, peaks near 1 N·m, then declines as torque increases toward the high end of the sweep.
- **Pin (W)** — input power, increasing roughly linearly with torque up to ~120 W.
- **Pout (W)** — output power, increasing with torque, with a shallower slope than Pin.
- **Effi (%)** — efficiency, rising quickly to a peak (~65–70%) around 1 N·m, then rolling off as torque increases further.

**Note:** The chart is a multi-axis analog curve in the source PDF; exact numeric sample values could not be extracted precisely by the automated transcription. The qualitative trends above (efficiency peak near 1 N·m, speed roll-off at high torque) are reproduced faithfully from the plotted curve shapes, but consult the original PDF (page 8) if precise curve-fit values are needed.

---

## Packing List

1. Motor (with integrated driver) ×1
2. Power cable (with CAN interface): XT30(2+2)-F connector ×1
3. Debug serial cable: GH1.25 3-pin cable ×1

---

## Interface & Pin Description

| Interface / Pin No. | Interface Diagram (Pinout) | Instruction |
| --- | --- | --- |
| Power Interface-1 (with CAN interface) | 2-pin power connector (`VCC`, `GND`) plus 2-pin CAN connector (`CAN_L`, `CAN_H`), XT30(2+2) style | 1. Connect the power supply via the XT30(2+2)-F cable. The rated voltage is 24 V. 2. Connect external controller via the CAN communication interface to receive CAN control commands and transmit motor status feedback. |
| Power Interface-2 (with CAN interface) | Same as Power Interface-1 | 3. The motor features two power interfaces. Either interface can be used independently, or multiple motors can be daisy-chained for convenient wiring. |
| Debug Serial Interface (Port 3) | 3-pin GH1.25 connector: `GND`, `RX`, `TX` | Connect to a PC via the GH1.25 3-pin cable using a USB-to-CAN adapter (or USB-to-serial module) for parameter configuration and firmware upgrades. |

---

## Motor Dimensions and Mounting

Please install the motor onto the target equipment according to the motor mounting hole dimensions and layout. Key dimensions transcribed from the mechanical drawing:

- Overall height: 46 mm; body length: 45 mm.
- Front-face housing diameter: Ø35 mm.
- Output-flange outer diameter: Ø57 mm.
- Output-flange bolt circle: 6× M3, depth 5 mm (⊥5), on a Ø27 mm circle.
- Output shaft / spigot register: Ø24.30 mm, with a Ø50 mm circle carrying a second set of 6× M3 (⊥5) mounting holes.
- 3× Ø4 <sup>+0.04</sup><sub>+0.02</sub> dowel/locating holes, spaced at 15° from the M3 bolt pattern.
- Rear-face (motor back-cover) pattern: 2× Ø3 <sup>+0.04</sup><sub>+0.02</sub> (⊥5) locating holes, plus 4× M3 (⊥4) mounting holes on a Ø38 mm circle.

**Note:** This is a mechanical drawing in the source PDF (page 9); the dimension callouts above are transcribed from the drawing's annotations as precisely as legible. Verify exact tolerances and hole positions against the original PDF drawing before machining mating parts.

---

## LED Status

| Status | LED | Meaning |
| --- | --- | --- |
| Normal Status | Solid Green | Enable Mode (ERR = 1), normal operation |
| Normal Status | Solid Red | Disable Mode (ERR = 0) (Default state after power-on) |
| Fault Status | Flashing Red | Fault codes and fault conditions (see below) |

Fault codes shown via flashing red LED:

| Code | Condition |
| --- | --- |
| 3 | Output shaft calibration error |
| 4 | Sensor output error |
| 5 | Motor encoder calibration error |
| 8 | Over-voltage |
| 9 | Under-voltage |
| A | Over-current |
| B | MOS over-temperature |
| C | Motor winding over-temperature |
| D | Communication loss |
| E | Overload |

Fault conditions can be identified via the feedback frames or displayed in the Damiao configuration software.

**Note:** The LED fault-code table above includes codes `3`, `4`, and `5` (output-shaft calibration error, sensor output error, motor-encoder calibration error). The CAN feedback-frame `ERR` field, as documented later in [Feedback Frames](#feedback-frames), only enumerates `0`, `1`, `8`, `9`, `A`, `B`, `C`, `D`, `E` — it does not list `3`/`4`/`5`. It is unclear from the source manual whether these three calibration-related fault codes are ever surfaced over the CAN feedback frame's `ERR` nibble, or whether they are LED/UART-only diagnostic codes. Verify empirically before relying on `ERR` to detect encoder-calibration faults.

---

## Operating Modes

### MIT Mode

The MIT mode is compatible with the standard MIT control method, enabling seamless switching while allowing flexible configuration of control limits (P_MAX, V_MAX, T_MAX).

CAN commands are converted into torque values. The torque is then used as the reference for current control, per the following block diagram:

```
p_des --[sum: p_des - theta_m]--> [Kp] --\
                                            \
v_des --[sum: v_des - d(theta_m)]--> [Kd] ---+--> [sum] --> T_ref --> [1/KT_OUT] --> iqref
                                            /
                                  t_ff ----/

                                                              idref = 0 (fixed)
```

Based on the MIT model, various control strategies can be derived. For instance, when `kp=0` and `kd≠0`, a constant rotational speed can be achieved by setting `v_des`. When `kp=0` and `kd=0`, a torque output is applied directly by setting `t_ff` (feedforward torque).

**Note:** When controlling position, `kd` must not be set to zero, as this may cause motor oscillation or even loss of control.

### Position-Velocity Mode

This mode uses three control loops: position, velocity, and current (torque). The position loop sets the target for the velocity loop. The velocity loop then sets the target for the current loop:

```
p_des --[sum: p_des - theta_m]--> [PI] --> v_des --[sum: v_des - d(theta_m)]--> [PI] --> iqref

                                                                                  idref = 0 (fixed)
```

`p_des` represents the target position, while `v_des` limits the maximum absolute velocity during motion.

When tuned using recommended parameters from the configuration tool, this mode provides high accuracy and smooth motion, at the cost of slower response time.

In addition to `v_des`, acceleration and deceleration can also be configured. If additional oscillations occur, increasing acceleration/deceleration may help stabilize the system.

**Note:** The units of `p_des` and `v_des` are rad and rad/s respectively, and both are of type float. The damping factor must be set to a non-zero positive value. Refer to the notes for velocity mode.

### Velocity Mode

This mode keeps the motor running at the target velocity:

```
v_des --[sum: v_des - d(theta_m)]--> [PI] --> iqref

                                       idref = 0 (fixed)
```

**Note:** The unit of `v_des` is rad/s, and its data type is float. To enable automatic parameter calculation via the configuration tool, the damping factor must be set to a non-zero positive value, typically between 2.0 and 10.0. An excessively low damping factor may cause velocity oscillations and significant overshoot, while an excessively high damping factor may increase the rise time. The recommended value is 4.0.

### Force-Position Hybrid Mode

The force-position hybrid control mode is based on position-velocity control, with additional torque adjustment. It allows control of both position and output torque at the same time:

```
p_des --[sum: p_des - theta_m]--> [PI] --> v_des --[sum: v_des - d(theta_m)]--> [PI] --> [saturation] --> iqref

                                                                                            idref = 0 (fixed)
```

A current command saturation stage is added after the velocity loop output command. This limits the current loop reference within the specified range.

### Mode Switching (host-PC / general)

Mode switching can be configured via the host PC over a serial port. Simply select the desired mode and click "Write Parameters". Once applied, the motor will automatically reset, and the selected mode is stored in the motor driver. It will be retained after power cycling.

Alternatively, modification can be changed via the CAN interface by updating the mode register. For details, refer to the [Mode Switching (register-based)](#mode-switching-register-based) section.

When using the CAN method, the motor will not reset, but the following five variables will be cleared to zero:

- Position command value
- Velocity command value
- Torque command value (MIT mode)
- kp (MIT mode)
- kd (MIT mode)

**Note:** If the "Save Parameters" command is not issued, the mode will not be stored. It will be lost after power loss and will load the last saved mode upon power-up.

---

## CAN Communication

After calibration, parameter identification, and parameter configuration are completed, the motor is ready for operation.

Control is performed using CAN standard frames (STD). The default baud rate is 1 Mbps, which can be modified via commands. Refer to the CAN Baud Rate section for details.

**Frame Types:**

CAN communication is divided into two types:

- **Command Frames** — Frames received by the driver, used to send control commands to the motor.
- **Feedback Frames** — Frames transmitted by the driver to the upper-level controller, containing motor status data.

**Feedback Mechanism:**

Feedback operates in a request-response manner. When the driver receives a CAN frame whose ID matches the configured motor CAN ID:

1. The lower 8 bits are used for validation.
2. The upper 3 bits are ignored.

Once a match is detected, the driver will transmit the current motor status to the CAN bus.

The command frame format and frame ID vary depending on the selected control mode. The feedback frame format remains the same across all control modes.

### CAN Baud Rate Configuration (overview)

The CAN baud rate can be configured in two ways:

**1. Configuration via Serial Interface**

The baud rate can be set through the host PC software via the serial interface.

1. Select the desired baud rate.
2. Click "Write Parameters" to apply.

After the setting is successfully written:

1. The motor will automatically reboot.
2. The baud rate will be stored in the driver.
3. The setting will persist after power cycling.

**2. Configuration via CAN Interface**

The baud rate can also be modified via the CAN interface by writing to the corresponding baud rate register. See [CAN Baud Rate Configuration (register-based)](#can-baud-rate-configuration-register-based) for details.

**Note:** When modifying the baud rate via CAN, the operation may fail if multiple devices are present on the bus. Use this method with caution. It is strongly recommended to configure the baud rate before connecting multiple devices.

### Feedback Frames

The feedback frame ID is set via the configuration tool (Master ID), with a default value of 0. It primarily reports the motor position, velocity, and torque with the frame format defined as follows:

| Feedback message | D[0] | D[1] | D[2] | D[3] | D[4] | D[5] | D[6] | D[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| MST_ID | ID\|ERR<<4 | POS[15:8] | POS[7:0] | VEL[11:4] | VEL[3:0]\|T[11:8] | T[7:0] | T_MOS | T_Rotor |

- **ID:** Controller ID, using the lower 8 bits of CAN_ID
- **ERR:** Status code, with the following meanings:

  | Code | Meaning |
  | --- | --- |
  | 0 | Disabled (Default state after power-on) |
  | 1 | Enabled |
  | 8 | Over-voltage |
  | 9 | Under-voltage |
  | A | Over-current |
  | B | MOSFET over-temperature |
  | C | Motor coil over-temperature |
  | D | Communication Lost |
  | E | Overload |

- **POS:** Motor position
- **VEL:** Motor velocity
- **T:** Motor torque
- **T_MOS:** Average MOSFET temperature on the driver (℃)
- **T_Rotor:** Average motor coil temperature (℃)

Position, velocity, and torque are converted from floating-point data to signed fixed-point data using a linear mapping:

1. Position: 16-bit signed fixed-point representation.
2. Velocity: 12-bit signed fixed-point representation.
3. Torque: 12-bit signed fixed-point representation.

### Control Frame in MIT Mode

| Control Message | D[0] | D[1] | D[2] | D[3] | D[4] | D[5] | D[6] | D[7] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| ID | p_des[15:8] | p_des[7:0] | v_des[11:4] | v_des[3:0]\|Kp[11:8] | Kp[7:0] | Kd[11:4] | Kd[3:0]\|t_ff[11:8] | t_ff[7:0] |

- **Frame ID:** Equals the configured CAN ID
- **P_des:** Desired position
- **V_des:** Desired velocity
- **Kp:** Position proportional gain
- **Kd:** Position derivative gain
- **T_ff:** Feedforward torque

All parameters follow the mapping rules described in the previous section. The ranges for p_des, v_des, and t_ff can be configured via the configuration tool. Kp range: [0,500], Kd range: [0,5].

A standard CAN frame contains 8 bytes. In MIT mode, the control command packs Position, Velocity, Kp, Kd, and Torque into these 8 bytes:

- Position: 16 bits (2 bytes)
- Velocity: 12 bits
- Kp: 12 bits
- Kd: 12 bits

### Control Frame in Position Velocity Mode

| Control Message | D[0]–D[3] | D[4]–D[7] |
| --- | --- | --- |
| 0x100+ID | p_des | v_des |

- **Frame ID:** Equals the configured CAN ID plus an offset of 0x100.
- **P_des:** Desired position, float type, little-endian (low byte first, high byte last).
- **V_des:** Desired velocity, float type, little-endian (low byte first, high byte last).

The CAN ID used to send commands here is 0x100 + ID. The velocity command represents the maximum velocity during trapezoidal acceleration, i.e., the constant velocity segment.

### Control Frame in Velocity Mode

| Control Message | D[0]–D[3] |
| --- | --- |
| 0x200+ID | v_des |

- **Frame ID:** Equals the configured CAN ID plus an offset of 0x200.
- **V_des:** Desired velocity, float type, little-endian (low byte first, high byte last).

The CAN ID used to send commands here is 0x200 + ID.

### Control Frame in Force-Position Hybrid Mode

| Control Message | D[0]–D[3] | D[4]–D[5] | D[6]–D[7] |
| --- | --- | --- | --- |
| 0x300+ID | p_des | v_des | i_des |

- **P_des:** Desired position, in radians, float type, little-endian (low byte first, high byte last).
- **V_des:** Velocity limit, in rad/s, scaled by 100, unsigned 16-bit, little-endian (low byte first, high byte last). Valid range: 0–10000. Values exceeding 10000 are capped at 10000, corresponding to an actual velocity limit range of 0~100 rad/s.
- **I_des:** Torque current limit (per-unit), scaled by 10000, unsigned 16-bit type, little-endian (low byte first, high byte last). Valid range: 0-10000. Values exceeding 10000 are capped at 10000, corresponding to an actual current limit amplitude of 0-1.0.
- **Per-unit current:** Actual current divided by the maximum phase current.

### Enable

After power-up self-test, an "Enable" command must be sent to allow motor control. The "Enable" frame is a control frame, with the frame ID as described above (i.e., depends on the currently active control mode's ID scheme). The data payload is identical across all modes as follows:

| D[0] | D[1] | D[2] | D[3] | D[4] | D[5] | D[6] | D[7] |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFC |

### Disable

The default power-on state of the motor is "Disable." In this state, the three-phase terminal voltages are identical, each being a 50% modulation of the power supply voltage. The "Disable" frame is a control frame, with the frame ID as described above. The data payload is defined as follows:

| D[0] | D[1] | D[2] | D[3] | D[4] | D[5] | D[6] | D[7] |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFD |

### Set Zero Position

The "Set Zero Position" frame is a control frame. This command sets the current output shaft position as the zero reference and resets the position command value to 0. The frame ID follows the definition above, and the data payload is defined as follows:

| D[0] | D[1] | D[2] | D[3] | D[4] | D[5] | D[6] | D[7] |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFE |

### Clear Faults

When faults such as overheating occur, sending a "Clear" command can reset the fault state. The "Clear" frame is a control frame. The frame ID follows the definition above, and the data payload is defined as follows:

| D[0] | D[1] | D[2] | D[3] | D[4] | D[5] | D[6] | D[7] |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFB |

### Read Parameters

Request frame:

| Message ID | Attribute | D[0] | D[1] | D[2] | D[3] |
| --- | --- | --- | --- | --- | --- |
| 0x7FF | STD | CANID_L | CANID_H | 0x33 | RID |

RID represents the register address; refer to the [Register Map](#register-map) section for details.

Upon successful read, the value of the register is returned. The frame format is as follows:

| Message ID | Attribute | D[0] | D[1] | D[2] | D[3] | D[4]–D[7] |
| --- | --- | --- | --- | --- | --- | --- |
| MST_ID | STD | CANID_L | CANID_H | 0x33 | RID | Data |

The data is either a floating-point value or an unsigned integer, occupying 32 bits (4 bytes) in total. The least significant byte is D4, and the most significant byte is D7. The same convention applies below.

### Write Parameters

Request frame:

| Message ID | Attribute | D[0] | D[1] | D[2] | D[3] | D[4]–D[7] |
| --- | --- | --- | --- | --- | --- | --- |
| 0x7FF | STD | CANID_L | CANID_H | 0x55 | RID | Data |

RID is defined as above. Upon successful write, the written value is returned. The frame format is identical to the transmitted frame:

| Message ID | Attribute | D[0] | D[1] | D[2] | D[3] | D[4]–D[7] |
| --- | --- | --- | --- | --- | --- | --- |
| MST_ID | STD | CANID_L | CANID_H | 0x55 | RID | Data |

Writing register data takes effect immediately but is not stored; it will be lost upon power-off unless a "Save Parameters" command is issued to store it in internal memory.

### Save Parameters

Request frame:

| Message ID | Attribute | D[0] | D[1] | D[2] | D[3] |
| --- | --- | --- | --- | --- | --- |
| 0x7FF | STD | CANID_L | CANID_H | 0xAA | 0x01 |

Upon successful execution, the return frame format is:

| Message ID | Attribute | D[0] | D[1] | D[2] | D[3] |
| --- | --- | --- | --- | --- | --- |
| MST_ID | STD | CANID_L | CANID_H | 0xAA | 0x01 |

**Notes:**

1. The "Save Parameters" command is only effective in Disabled Mode.
2. All parameters are stored simultaneously during this operation.
3. This operation writes parameters to the on-chip flash memory. The maximum execution time is 30ms; please allow sufficient time.
4. The flash memory supports approximately 10,000 erase/write cycles. Avoid frequent execution of the "Save Parameters" command.

### Mode Switching (register-based)

Multiple control modes are supported and can be switched as follows:

| Code/ID | Mode |
| --- | --- |
| 1 | MIT |
| 2 | Position-Velocity |
| 3 | Velocity |
| 4 | Force-Position Hybrid Control |

The mode can be changed by modifying the mode register (`0x0A`). During switching, the motor resets all command values to zero, including position, velocity, and torque feed-forward, KP, and KD in MIT mode.

When switching to a position control mode, to avoid impact, it is recommended to first read the precise position (value in register `0x50`) and perform the switch when the motor is at zero speed whenever possible.

Mode changes are not saved to flash memory and will be lost after power-off. Upon restart, the motor loads the last mode stored in flash.

### CAN Baud Rate Configuration (register-based)

By writing a specific value to the baud rate register (address `0x23`), the current CAN communication baud rate can be modified. Supported baud rates are listed below:

| Code/ID | Baud Rate |
| --- | --- |
| 0 | 125K |
| 1 | 200K |
| 2 | 250K |
| 3 | 500K |
| 4 | 1M |
| 5 | 2M |
| 6 | 2.5M |
| 7 | 3.2M |
| 8 | 4M |
| 9 | 5M |

**Baud Rate Switching Behavior:** After a successful update, the driver first transmits feedback using the original baud rate and then switches to communication using the new baud rate.

**Baud Rate Handling at Power-Up:** At power-up, the driver determines the communication mode based on the stored baud rate.

- If the stored baud rate exceeds 5Mbps, the baud rate is reset to 1Mbps by default.
- If the baud rate is greater than 1 Mbps (excluding 1 Mbps), the driver operates in CAN FD mode.
- If the baud rate is ≤ 1 Mbps, the driver operates in CAN 2.0B mode.

**CAN FD Compatibility Note:** A motor configured in CAN FD mode can still receive CAN 2.0B data frames. However, feedback frames are transmitted using CAN FD. As a result, if the upper-level controller only supports CAN 2.0B, it will not receive feedback data, and the driver may continuously report communication errors.

**Recovery Note:** For controllers operating in CAN 2.0B mode, even if the CAN ID is incorrectly configured, the baud rate can still be restored by sending a baud rate configuration command.

### Register Map

**This is the authoritative register map for the DM-J4310-2EC V1.2.** Every row is transcribed exactly as printed in the source manual (pages 19–20).

| Address (HEX) | Address (DEC) | Name | Description | Access | Range | Type |
| --- | --- | --- | --- | --- | --- | --- |
| 0x00 | 0 | UV_Value | Undervoltage Threshold | RW | (10.0,fmax] | float |
| 0x01 | 1 | KT_Value | Torque Constant | RW | [0.0,fmax] | float |
| 0x02 | 2 | OT_Value | Over-temperature Threshold | RW | [80.0,200) | float |
| 0x03 | 3 | OC_Value | Overcurrent Threshold | RW | (0.0,1.0) | float |
| 0x04 | 4 | ACC | Acceleration | RW | (0.0,fmax) | float |
| 0x05 | 5 | DEC | Deceleration | RW | [-fmax,0.0) | float |
| 0x06 | 6 | MAX_SPD | Max Velocity | RW | (0.0,fmax] | float |
| 0x07 | 7 | MST_ID | Master ID | RW | [0,0x7FF] | uint32 |
| 0x08 | 8 | ESC_ID | Command CAN ID | RW | [0,0x7FF] | uint32 |
| 0x09 | 9 | TIMEOUT | Timeout Threshold | RW | [0,2^32-1] | uint32 |
| 0x0A | 10 | CTRL_MODE | Control Mode | RW | [0,4] | uint32 |
| 0x0B | 11 | Damp | Viscous Damping | RO | — | float |
| 0x0C | 12 | Inertia | Rotor Inertia | RO | — | float |
| 0x0D | 13 | hw_ver | Reserved | RO | — | uint32 |
| 0x0E | 14 | sw_ver | Firmware Version | RO | — | uint32 |
| 0x0F | 15 | SN | Reserved | RO | — | uint32 |
| 0x10 | 16 | NPP | Number of Pole Pairs | RO | — | uint32 |
| 0x11 | 17 | Rs | Phase Resistance | RO | — | float |
| 0x12 | 18 | Ls | Phase Inductance | RO | — | float |
| 0x13 | 19 | Flux | Flux Linkage | RO | — | float |
| 0x14 | 20 | Gr | Gear Reduction Ratio | RO | — | float |
| 0x15 | 21 | PMAX | Position Mapping Range | RW | (0.0,fmax] | float |
| 0x16 | 22 | VMAX | Velocity Mapping Range | RW | (0.0,fmax] | float |
| 0x17 | 23 | TMAX | Torque Mapping Range | RW | (0.0,fmax] | float |
| 0x18 | 24 | I_BW | Current-loop Bandwidth | RW | [100.0,1.0e4] | float |
| 0x19 | 25 | KP_ASR | Velocity Loop Kp | RW | [0.0,fmax] | float |
| 0x1A | 26 | KI_ASR | Velocity Loop Ki | RW | [0.0,fmax] | float |
| 0x1B | 27 | KP_APR | Position Loop Kp | RW | [0.0,fmax] | float |
| 0x1C | 28 | KI_APR | Position Loop Ki | RW | [0.0,fmax] | float |
| 0x1D | 29 | OV_Value | Overvoltage Threshold | RW | TBD | float |
| 0x1E | 30 | GREF | Gear Torque Efficiency | RW | (0.0,1.0] | float |
| 0x1F | 31 | Deta | Velocity Loop Damping Coefficient | RW | [1.0,30.0] | float |
| 0x20 | 32 | V_BW | Velocity Loop Filter Bandwidth | RW | (0.0,500.0) | float |
| 0x21 | 33 | IQ_c1 | Iq Gain | RW | [100.0, 1.0e4] | float |
| 0x22 | 34 | VL_c1 | Velocity Loop Gain Factor | RW | (0.0,1.0e4] | float |
| 0x23 | 35 | can_br | CAN Baud Rate | RW | [0,9] | uint32 |
| 0x24 | 36 | sub_ver | Sub-version | RO | — | uint32 |
| 0x25 | 37 | Boot_ver | Bootloader Version | RO | — | uint32 |
| 0x37 | 55 | dir | Direction | RO | — | float |
| 0x38 | 56 | m_off | Motor-Side Angle Offset | RO | — | float |
| 0x3B | 59 | Imax | Driver Current Limit | RO | — | float |
| 0x3C | 60 | VBus | Bus Voltage | RO | — | float |
| 0x3D | 61 | Tpcb | PCB Temperature | RO | — | float |
| 0x3E | 62 | Tmtr | Motor Temperature | RO | — | float |
| 0x3F | 63 | I_U_OFF | Phase U Current Offset | RO | — | float |
| 0x40 | 64 | I_V_OFF | Phase V Current Offset | RO | — | float |
| 0x41 | 65 | I_W_OFF | Phase W Current Offset | RO | — | float |
| 0x50 | 80 | p_m | Motor Position | RO | — | float |
| 0x51 | 81 | xout | Output Shaft Position | RO | — | float |

**Notes (as printed):**

1. RW: Read/Write.
2. RO: Read-Only.
3. Motor output shaft position is calculated from the rotor position, in radians (rad).
4. Output shaft position is measured directly by the output shaft encoder, in radians (rad).

**Additional transcription notes:**

- The register map has a gap between address `0x25` (37, Boot_ver) and `0x37` (55, dir) — addresses `0x26`–`0x36` (38–54) are not documented in the source manual and are presumably reserved/unused or internal.
- Row `0x1D` (`OV_Value`, Overvoltage Threshold) has its Range column printed literally as **"TBD"** in the source — this is not a transcription error; the manual itself leaves the valid range undocumented. Cross-reference against the Specifications table's recommended over-voltage threshold (≤ 32V) if a concrete bound is needed.
- Rows `0x0D` (`hw_ver`) and `0x0F` (`SN`) are both named suggestively ("hardware version" and "serial number") but their Description column reads only **"Reserved"** — the same discrepancy pattern seen in other DAMIAO/RobStride-family register maps where the printed Name and Description disagree. Treat these two registers' actual returned values as unverified/undocumented rather than assuming they return a real hardware version or serial number.
- `fmax` appears throughout the Range column as an unresolved symbolic bound (e.g., `(0.0,fmax]`); the manual does not define its numeric value in this section — it likely denotes a firmware/hardware-defined ceiling rather than a fixed literal.

---

## Motor Debugging Procedure

Please use configuration software version V2.0.0.0 or above for debugging. This guide uses version V2.1.4.6 as an example. The procedures shown are based on the 4340P model in the 43 series and apply to other motors in the same family (including the DM-J4310-2EC).

Before use, first locate the Chinese-English switch button in the lower-left corner of the interface.

### Device Connection

Connect the following interfaces:

- UART interface
- CAN interface
- Power supply

Open the configuration software on the PC, select the correct serial port, and open the connection.

Once powered on, the motor will output status information via UART. "Control Mode" indicates the current control mode of the drive. Different modes use different command formats (see [CAN Communication](#can-communication)).

### Motor-side Encoder Calibration

Motor encoder calibration compensates for installation offsets.

During calibration, the motor will rotate forward and backward for one electrical cycle. Ensure the motor can rotate freely, preferably under no-load conditions, to prevent calibration failure. Motors are factory-calibrated. Recalibration is only required after hardware changes or abnormal behavior.

**Procedure:**

**Step 1: Motor-side Encoder Calibration.** Click "Calibration" to start. The motor will rotate and report the number of pole pairs (shown in a "Number of pole pairs to calibrate is 14" dialog in the source example). It will then automatically perform parameter identification, followed by encoder calibration.

During this process:

- The motor will move.
- Secure the motor to prevent movement.

A waveform will be displayed after calibration. **Validation guideline:** The peak of the red curve should not significantly exceed the blue curve. If it does, calibration may fail; repeat the process.

**Result Verification:**

**Step 2:** Calibration data is uploaded automatically, presented as four plots: Original Data, Compensation Data, Ripple Data, and Compensation Benefit.

**Step 3: Calibration Data Check and Storage.** Pay close attention to the value of Compensation Data. Recommended range: ±300. Out-of-range values may indicate:

- Incorrect pole pair identification.
- Excessive mechanical resistance / unstable rotation.
- Improper sensor installation.

**Step 4: Troubleshooting.** Check and resolve the issue based on the possible causes listed above.

### Parameter Identification

This process identifies key motor parameters:

- Phase resistance
- Phase inductance
- Flux linkage

These values are pre-calibrated at the factory and stored internally. The motor is ready for immediate use without requiring recalibration.

**Procedure:** Go to Parameter Settings, click "Parameter Identification". The driver will initiate the identification process. The motor will rotate during identification. Make sure no load is applied, and the motor is secured.

Results are uploaded automatically after completion (example values from the source screenshot: PhaseRes(R) = 687.35 mΩ, PhaseInd(L) = 315.962 μH, FluxLinkage(λ) = 0.00449057 Wb, Fri Coeff = 0.000138623, Inertia = 1.7493e-05 kg·m²).

**Notes:**

1. Viscous coefficient is for reference only.
2. Multiple runs may yield different results.
3. All parameters (except viscous coefficient) must be non-negative. If negative values occur, verify the motor status before proceeding with calibration.

### Output Shaft Encoder Calibration (Dual Encoder Motors)

Required for motors equipped with an output shaft encoder (such as the DM-J4310-2EC, which has two encoders) to enhance encoder accuracy.

**Important Notes Before Calibration:**

- During calibration, the motor will rotate forward one full revolution.
- Ensure the motor can rotate freely and operate under no load; otherwise, excessive calibration errors may occur.
- Motors are pre-calibrated at the factory, and the calibration data is stored in the motor. Under normal conditions, no additional calibration is required.

Output shaft encoder calibration should be performed if:

- The motor's driver board has been replaced.
- The output position has changed.
- Other abnormal conditions occur.

**Step 1: Output Shaft Encoder Calibration.** Click the slider next to the Calibration button until "Output Shaft Encoder" status is displayed, then click "Calibration". The motor begins rotating one full revolution and uploads the raw encoder data waveform.

**Step 2:** Calibration results (a "Calibration Compensation Table (output shaft)" curve) are uploaded automatically.

**Step 3: Calibration Data Verification.** The driver performs internal validation:

- If deviation is excessive, see Error Code 3 (Output shaft calibration error, per the [LED Status](#led-status) table).
- Red LED will flash.

This typically indicates an encoder issue. Contact after-sales support for resolution.

### Parameter Management

**1. Read parameters**

Go to the "Parameter Settings" tab, click "Read Parameters". The drive will upload all stored parameters. Please carefully check the parameters.

**Parameter Categories:**

**(1) Drive Parameters** — Settings for the driver:

- **Pole Pairs** — Auto-detected. Do not modify.
- **Undervoltage Threshold** — Default: 20 V. If the supply voltage falls below this set value, the driver will disable motor control to prevent unstable operation.
- **Overvoltage Threshold** — Defines the upper limit of the supply voltage. If exceeded, the driver will report a fault and exit the enabled state. If overvoltage is detected at power-up, the fault will persist.
- **Acceleration/Deceleration** — Limits the rate of change of motor velocity in non-MIT modes. Unit: Krad/s². Deceleration is represented as a negative value.
- **Reduction Ratio** — Defines the mechanical transmission ratio. Affects output velocity, position scaling, and torque feedback. This parameter is preconfigured; do not modify.
- **Overtemperature Threshold** — Temperature protection threshold for motor windings. Recommended ≤ 100° C. When exceeded, the driver will disable output and report a fault.
- **CAN_ID** — Identifier used for command frames (hexadecimal format). Must be unique on the CAN bus. Recommended value: < 16 to avoid conflict with system-reserved IDs.
- **Master ID** — Feedback CAN ID used for feedback frames transmitted by the driver, hexadecimal.
- **CAN Timeout** — Defines the communication timeout period. If no valid CAN command is received within the specified time, the driver will trigger motor protection. Time base: 50 μs per count. Only effective when the motor is enabled.
- **Velocity Limit** — Used only in velocity mode to prevent overspeed of the motor rotor (before deceleration). Unit: rad/s.
- **Current Limit** — Limits the maximum phase current as a percentage of rated current. Used for torque protection and safe operation.
- **CAN Baud Rate** — Configures the communication data rate (bps) of the CAN bus, supporting 125Kbps–5Mbps.
- **Ki_current (Current Loop Gain)** — Auxiliary parameter for current loop control. Typically pre-tuned and not recommended for manual adjustment.
- **fbw (Velocity Loop Filter Bandwidth)** — Defines the bandwidth of the velocity signal filter. Unit: Hz.
- **Minor Version** — Firmware minor version number.
- **Ki_speed (Velocity Loop Gain)** — Auxiliary gain for velocity loop control. Recommended adjustable range: 200–800.

**Note:** After a successful parameter read, the values in the debugging interface will be updated automatically (e.g., PMAX, ID).

**(2) Motor Parameters** — Automatically identified during calibration and stored in the driver (recalibration is required when replacing the driver board).

**(3) Control Limits** — Define the valid range for command input and feedback scaling:

- **PMAX / VMAX / TMAX** — Define the mapping range for position, velocity, and torque. In MIT mode: used to scale incoming command values. In other modes: used to scale feedback data. Refer to [CAN Communication](#can-communication) for mapping rules.
- **KT_OUT** — Motor torque constant. Set to 0 when motor parameter identification is accurate.
- **Torque Ratio** — Defines the torque transmission efficiency of the gearbox. Value range: (0, 1].
- **Damping factor** — Ratio of current loop bandwidth to velocity loop bandwidth. Currently not used.

**(4) Control settings:**

- **Control Mode** — The driver supports: MIT Mode, Position-Velocity Mode, Velocity Mode, Force-Position Hybrid Mode.
- **Current Bandwidth** — Defines the response speed of the current control loop, default 1000.
- **Velocity Loop Gains (KP / KI)** — Control parameters for velocity regulation.
- **Position Loop Gains (KP / KI)** — Control parameters for position tracking.

**2. Write parameters**

Verify parameters such as drive parameters, control amplitude, and control settings. Modify parameters as required, then click "Write Parameters" to save them to the driver.

After writing: the driver will automatically reboot, and updated parameters take effect immediately.

**Notes:**

1. The motor must be in disabled state before writing parameters.
2. No manual power cycle is required.
3. Writing parameters during operation may cause unexpected behavior.

### Debugging via CAN

Debugging is available only when the CAN interface is connected.

- Only one motor can be debugged at a time.
- Ensure correct CAN wiring before operation.

**Before Starting:**

- Verify wiring connections.
- Confirm the active control mode.
- Ensure CAN IDs are correctly configured.

**1. Control Mode Selection and Verification**

Go to "Parameter Settings", click "Control Mode" and select one of: MIT Mode, Position-Velocity Mode, Velocity Mode, or Force-Position Hybrid Mode (PVT).

Click "Write Parameters" to apply the selected control mode. A confirmation message "Parameter write successful!" will appear.

**Note:** After writing, the driver will reboot automatically, and the selected mode becomes active.

**Control Mode Verification:**

- **Option 1:** Check the serial output at power-on. The mode indicated by the arrow next to the printed `Control Mode:` list (`1:MIT Mode`, `2:position-speed cascade Mode`, `3:speed Mode`, `4:Hybrid control Mode`) is the currently active control mode.
- **Option 2:** Refresh the parameter page and read the displayed "Control Mode" field to verify the control mode.

**2. MIT Mode**

(1) **Mode Selection:** In the debugging page, select MIT mode as described above. Confirm the current control mode and open the corresponding MIT tab.

(2) **CAN ID:** Ensure the CAN ID is correct (obtainable via serial port printouts, parameter settings page, or read/set buttons in the debugging interface).

(3) **Control Options:** The MIT model supports three control methods: velocity, position, and torque.

**① Velocity Control**

- Step 1: Click "Enable" in the Motor Mode section. The driver's green LED will light (motor enabled).
- Step 2: Set the target velocity. Example: Velocity = 5 rad/s, KD = 1 N·s/r, other gains = 0. Enable "Timed" for periodic send, then click "Update" and "Send". Monitor parameter curves in the debugging interface. Adjust control parameters as needed while testing; keep "Timed" enabled and click "Update" to apply changes. Monitor real-time motor and driver temperature, status, and feedback frames (see [Feedback Frames](#feedback-frames)).
- Step 3: Exit procedure — click "Stop", then "Disable"; the Red LED turns ON (motor disabled).

**② Position Control**

- Step 1: Click "Enable" in the Motor Mode section.
- Step 2: Set the target position. Consider the motor's initial position; avoid large deviations to prevent impact. Use "Save Zero" in the command section to set the current position as the zero reference. Example: Position = 3.14 rad, KP = 2 N/r, KD = 1 N·s/r, others = 0. Enable "Timed" for periodic send, then click "Update" and "Send". Ensure the motor is physically fixed.
- Step 3: Exit procedure — click "Stop", then "Disable".

**③ Torque Control**

- Step 1: Click "Enable" in the Motor Mode section.
- Step 2: Set the target torque. **Note:** Even small torque commands can accelerate the motor to max speed if unloaded. Example: Torque = 1 N·m, others = 0. Enable "Timed" for periodic send, then click "Update" and "Send". Ensure the motor is physically fixed.
- Step 3: Exit procedure — click "Stop", then "Disable".

**3. Position Velocity Mode**

(1) **Mode Selection:** In the debugging page, select Position Velocity mode. Confirm the current control mode and open the corresponding "Position" tab.

(2) **CAN ID:** Ensure the CAN ID is correct.

(3) Click "Enable" in the Motor Mode section.

(4) Set the command parameters so that the motor moves to the target position at the specified velocity. Before setting the parameters, check the motor's initial position and use it as a reference. Example: Position = 10 rad, Velocity = 5 rad/s. Enable "Timed" for periodic send, then click "Update" and "Send". Ensure the motor is physically fixed.

(5) Exit procedure: Click "Stop", then "Disable".

**4. Velocity Mode**

(1) In the debugging page, select Velocity mode. Confirm the current control mode and open the corresponding "Speed" tab.

(2) Ensure the CAN ID is correct.

(3) Click "Enable" in the Motor Mode section.

(4) Set the target velocity. Example: Speed = 5 rad/s. Enable "Timed" for periodic send, then click "Update" and "Send". Ensure the motor is physically fixed.

(5) Exit procedure: Click "Stop", then "Disable".

**5. Force-Position Hybrid Control Mode**

(1) In the debugging page, select Force-Position Hybrid Control Mode. Confirm the current control mode and open the corresponding "PVT" tab.

(2) Ensure the CAN ID is correct.

(3) Click "Enable" in the Motor Mode section.

(4) Set the command parameters so that the motor moves to the target position at the specified velocity. Before setting the parameters, check the motor's initial position and use it as a reference. Example: Position = 10 rad, Speed = 5 rad/s, current = 0.2%FS. Enable "Timed" (labeled "Periodic Send" in this mode's panel) for periodic send, then click "Update" and "Send". Ensure the motor is physically fixed.

(5) Exit procedure: Click "Stop", then "Disable".

---

## Firmware Version Check and Upgrade

### Version Check

Connect the motor via serial port, CAN port, and power supply. Select the correct serial port on the PC and open it. Click "Read Version" to display the current firmware version of the driver (example from the source screenshot: `FW REV: 7.0.1.8`, `BL REV: 3.2.0.6`).

Before performing any firmware upgrade, always read and confirm the current version. Unless otherwise specified, select firmware versions with a prefix of "70" for upgrading.

Using an incorrect firmware version may cause unexpected issues.

### Firmware Upgrade via Serial Port

When new features are released or bug fixes are available, the firmware can be upgraded via the serial interface.

Before upgrading, ensure the serial connection is established.

**Upgrade Procedure:**

1. Click "Select" and choose the appropriate firmware file.
2. After loading, the firmware name will be displayed in the interface.
3. Verify the firmware is correct, then click "Upgrade Firmware".
4. Wait for the progress bar to complete.

The upgrade status can also be monitored through the serial interface (the source example shows a per-packet progress log, e.g. `Pack 1 pass:OK` … `Pack 7 pass:OK`, followed by `Update firmware completed!!!`).

### CAN Firmware Upgrade

Firmware upgrade via CAN is supported for Bootloader version 3.2.0.5 and above. The CAN baud rate must match the one configured on the driver board.

Currently, firmware upgrade is supported only via a USB-to-CAN adapter.

**Preparation:** Before starting, connect the CAN interface and ensure communication is functioning properly. Then, in the interface:

- Switch the communication mode from UART to CAN.
- Enter the target motor CAN ID in the left panel.

If the CAN bus is not detected, a prompt will be displayed (source example: "ID 0x02 does not exist on the bus, the number of devices on the bus is 1, IDs: 0001"). If the connection is successful, no message will be displayed.

**Upgrade Procedure:**

1. Click "Select" and choose the desired firmware file.
2. After loading, the firmware name will be displayed in the interface.
3. Verify the firmware is correct, then click "Upgrade".
4. Wait for the progress bar to complete.

The upgrade status can also be monitored through the serial interface. Once the upgrade is complete, a confirmation message will be displayed in the serial interface (source example: a per-packet log `1th OK` … `8th OK`, `complete`, `Enter Bootloader!`).
