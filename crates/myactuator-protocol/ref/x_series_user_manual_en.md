# Suzhou Micro Actuator Technology — Instruction Manual for the Product

**Applicable series:** X series
**Version:** V1.1
**Date:** 2025.05

---

## Table of Contents

- [Preface](#preface)
- [Imprint Notice](#imprint-notice)
- [1. Safety Precautions](#1-safety-precautions)
- [2. Quality Assurance](#2-quality-assurance)
  - [2.1. After-sales Policy](#21-after-sales-policy)
  - [2.2. Disclaimer](#22-disclaimer)
- [3. Basic Parameters of the Module](#3-basic-parameters-of-the-module)
  - [3.1. Module Nameplate and Model](#31-module-nameplate-and-model)
  - [3.2. Module Appearance Size](#32-module-appearance-size)
  - [3.3. Module Parameters](#33-module-parameters)
  - [3.4. Module Stall Characteristics](#34-module-stall-characteristics)
- [4. Mechanical Installation Requirements](#4-mechanical-installation-requirements)
- [5. Electrical Installation Requirements](#5-electrical-installation-requirements)
  - [5.1. About the Input Power Supply](#51-about-the-input-power-supply)
  - [5.2. Interface Description](#52-interface-description)
  - [5.3. Indicator Description](#53-indicator-description)
- [6. Cable Connection between Multi-joint Modules](#6-cable-connection-between-multi-joint-modules)
  - [6.1. Description of the Power Supply Wiring](#61-description-of-the-power-supply-wiring)
  - [6.2. CAN Communication Wiring Instructions](#62-can-communication-wiring-instructions)
  - [6.3. EtherCAT Communication Wiring Instructions](#63-ethercat-communication-wiring-instructions)
- [7. Kinetic Energy Recovery](#7-kinetic-energy-recovery)
  - [7.1. Reasons for Kinetic Energy Recovery](#71-reasons-for-kinetic-energy-recovery)
  - [7.2. Handling Methods](#72-handling-methods)
- [8. Encoder Description](#8-encoder-description)
  - [8.1. Resolution and Position Feedback](#81-resolution-and-position-feedback)
  - [8.2. Instructions for the Use of the Mechanical Zero Calibration Function](#82-instructions-for-the-use-of-the-mechanical-zero-calibration-function)
- [9. Connect and Debug the Setup Software](#9-connect-and-debug-the-setup-software)
- [10. Communication Instruction Description](#10-communication-instruction-description)

---

## Preface

Thanks for choosing MYACTUATOR.

X series joint module is a highly integrated joint module provided by the company under the premise of years of experience accumulation for customer service, which has a large transmission speed ratio, strong bearing capacity, precise control, is easy to carry, and saves a lot of time for developers.

This manual introduces the parameters, usage methods, precautions, and other information of the X series integrated harmonic module. Please read carefully before starting to operate.

## Imprint Notice

The copyright of this manual belongs to Suzhou Micro Actuator Technology Co., Ltd. Without the permission of the company, the content of this manual cannot be copied or plagiarized in any way. The product manual is only used as a reference for users and does not constitute the basis for meeting the user's overall usage requirements. Please be sure to evaluate it in combination with the user's overall system. The content of the manual strives to be detailed and accurate, but oversights are inevitable. If you find any errors, please provide your valuable feedback.

The company reserves the right to modify and improve this manual at any time without prior notice. For the latest version of the manual, please visit the official website (www.myactuator.com) to download it yourself, or contact the company to obtain it.

**Table I — Version update instructions**

| Version | Update instructions |
| --- | --- |
| V1.0 | First edition |
| V1.1 | Update the module models and parameters |

---

## 1. Safety Precautions

This product is a high-precision product. Only professionals with corresponding qualifications can perform tasks such as installation, debugging, and maintenance. Corresponding personnel must understand and comply with IEC60364/IEC60664 and national accident-prevention regulations. Please read the manual carefully before installing, operating, or repairing this product. Wrong operation may damage the module or even cause casualties. Be sure to follow the safety precautions in this manual.

*[The manual defines a set of safety symbols here — see figure in original PDF.]*

In this manual, hazardous situations are recorded as much as possible; see Table 1-1 for details. Relevant personnel are requested to understand and follow the precautions below. There are, in addition, too many uncertain factors that cannot all be foreseen and recorded — in actual application it is necessary to prevent and handle situations according to the real circumstances.

**Table 1-1 — Safety precautions**

### Unboxing

- **Check whether the outer packaging of the device is intact.** Before unpacking, check whether the outer packaging is intact and whether it is damaged, damp, deformed, etc.
- **Do not unbox violently.** Unpack in accordance with the hierarchical order; violent knocking is strictly prohibited.
- **Check whether the module and its accessories are complete.** Refer to the packing list to check whether the module name is correct, whether the accessories are complete, and whether there is any damage on the surface of the equipment and its accessories. If there is any problem, do not install it and contact the company promptly.

### Installation and maintenance phase

- **Assemble the module in place.** When assembling the module, tighten it in place according to the screw torque standards to ensure there is no danger of accidental falling.
- **Do not plug or unplug the power cord while the power is on.** Make sure the power indicator light goes out before wiring or maintaining the device.
- **Do not plug or unplug communication lines while the power is on.** When the GND of the control terminal and the module are not connected, the voltage is inconsistent; the voltage difference between the two at the moment of connection may damage the communication interface.
- **Do not disassemble the module and its associated equipment while the power is on.** It is strictly prohibited to disassemble any device or accessory of the equipment while powered, otherwise there is a risk of electric shock.
- **Keep the module shell grounded and use the shielding layer properly.** If the module shell is not grounded, it may cause charge accumulation in the shell, affecting the normal operation of the motor and even causing harm to the human body. The wiring cable must meet the corresponding wire-diameter and shielding requirements, and the shielding layer of the shielded cable must be reliably grounded.
- **Use a multi-turn encoder power battery that meets the specifications.** To prevent the multi-turn value from being lost due to power outage, the multi-turn encoder is powered by a specific battery with a required specification of 2.7–3.6 V. Do not use other types of batteries — if the module is damaged as a result, the company will not provide technical support for it.
- **It is strictly prohibited to connect the power supply to the output of the device**, to avoid damage to the device or even fire.
- **Perform a risk assessment** before use and take appropriate measures to ensure personal safety and equipment safety.
- **Observe the technical data and specifications.** Refer to the parameters of each model in the manual to set reasonable parameters and prevent damage to the module.
- **Set appropriate protection limits** — position limits, speed limits, current limits, etc. Exceeding the limits may damage the motor or even threaten personal safety.
- **Perform a no-load test run before using the module**, to prevent accidents.
- **Do not disassemble or replace parts by yourself.** Product failure due to abnormal use will void the product's warranty rights.
- **Do not hit or squeeze the module and its components with gravity.** The module is a precision device — do not hit it with a hammer, and place it carefully to prevent it from falling off the table and cracking or otherwise being damaged.
- **The use environment must comply with regulations.** The working environment temperature of the module is 0–50 °C. When the temperature is low, it is recommended to use low-temperature grease to improve the operating resistance of the module. Keep the environment free of dust, corrosive gases, flammable gases, etc.
- **Be careful of high-temperature burns.** During operation, the module surface may be very hot — pay attention to protection. When the surface temperature exceeds 40 °C, avoid long-term contact (risk of low-temperature burns); when it exceeds 85 °C, avoid touching it entirely (risk of burns).

### Storage

- **Storage environment must meet standards.** Follow the manual's requirements for transportation and storage temperature/humidity, and avoid direct sunlight, strong magnetic fields, strong electric fields, strong vibration, and similar conditions.
- **Storage time should not be too long.** Avoid storing the module for more than 3 months. If the storage time is too long, take more stringent protective measures and perform necessary inspection and maintenance.
- **Do not mix and transport equipment that may cause damage.** Pack the module strictly before transporting it; it is strictly prohibited to transport it mixed with equipment that may affect it.
- **Perform regular inspection and maintenance.** Perform daily and periodic inspection and maintenance on the module, and keep maintenance records.

### Others

- **Do not remove the anti-tear warranty label**, otherwise you will lose your warranty rights.
- **Dispose of it as industrial waste** — please dispose of the module and its accessories as industrial waste.

---

## 2. Quality Assurance

### 2.1. After-sales Policy

This product strictly implements the following after-sales services in accordance with the "Law of the People's Republic of China on the Protection of Consumers' Rights and Interests" and the "Law of the People's Republic of China on Product Quality".

1. All users who purchase this product can enjoy a return-and-exchange service if there is a product quality problem within 7 days. When returning or exchanging, provide a valid proof of purchase and return invoice, and ensure that the returned product has intact functions, no damage to appearance, and complete accessories.
2. Users who purchase this product enjoy free warranty service within one year from the day after receipt. In the event of man-made damage or manual disassembly, no warranty service will be provided; if, after testing, it is confirmed that the motor needs to be replaced, the merchant will need to negotiate with the customer on whether to purchase additional repair parts.
3. If there is a quality problem with the product within 7 to 15 days from the day after receipt, the customer can enjoy an exchange service after confirmation, under the same proof-of-purchase and condition requirements as above.
4. The following situations are **not** covered by the warranty:
   - Failure to install and connect other control equipment according to the requirements of the user manual, which may cause the motor to burn out;
   - Exceeding the specifications or standards shown in the user manual during use (e.g. wrong motor parameter settings);
   - Storage method or working environment exceeding the specified range in the user manual (e.g. pollution, salt damage, condensation, etc.);
   - Product damage caused by abnormal working conditions (falling, impact, liquid intrusion, violent impact, etc.);
   - Product damage caused by force majeure (natural disasters, fires, floods, etc.);
   - Users dismantling the product themselves, causing damage to the motor;
   - Exceeding the warranty period provided by the after-sales policy;
   - Inability to provide valid proof of purchase;
   - Failures other than those mentioned above that are not caused by Suzhou Micro Actuator Technology Co., Ltd.'s responsibility.

In the event of a joint-module failure, contact Suzhou Micro Actuator Technology Co., Ltd. as soon as possible to obtain a solution. Users are not allowed to disassemble and reassemble the joint module for any reason, otherwise the warranty service will be terminated.

### 2.2. Disclaimer

Please read this statement carefully before use. Once used, it is deemed to be recognition and acceptance of the entire content of this statement. Install and use this product in strict compliance with the manual, product instructions, and relevant laws, regulations, policies, and guidelines. In the process of using the product, users take responsibility for their own actions and all consequences arising therefrom. Myactuator will not be held legally responsible for any losses caused by improper use, installation, or modification by users. The final right to interpret this disclaimer belongs to Myactuator.

---

## 3. Basic Parameters of the Module

The X series module integrates a frameless torque motor, an absolute encoder, a servo driver, and a planetary reducer. It has a compact structure, strong integration, and is easy to install. X series modules are currently available as X2, X4, X6, X8, X12, and X15, making robot development more convenient and flexible.

### 3.1. Module Nameplate and Model

*[See figure in original PDF — module nameplate photo.]*

Taking **RMD-X2-P28-7-C** as an example, the module product-model parameters are explained in Table 3-1.

**Table 3-1 — Product Model Parameter Explanation**

| Parameter | Explanation |
| --- | --- |
| `RMD` | Myactuator's sub-brand name: Reducer Motor Drive. |
| `X2` | The base number, available in six models: X2, X4, X6, X8, X12, and X15. |
| `P28` | The gear ratio, rounded to the nearest integer. For the specific (exact) gear ratio, refer to the module parameter table (Table 3-3). |
| `7` | Peak torque — indicates that the peak torque for this model is 7 N·m. |
| `C` | Indicates the communication interface type of the module: `C` = CANBUS, `E` = EtherCAT. |

The specific models of the company's X series are shown in Table 3-2.

**Table 3-2 — The specific models of Series X**

| Abbreviation | Full Name (CAN variant) | Full Name (EtherCAT variant) |
| --- | --- | --- |
| X2-P28 | RMD-X2-P28-7-C | RMD-X2-P28-7-E |
| X4-P12.5 | RMD-X4-P12.5-10-C | RMD-X4-P12.5-10-E |
| X4-P36 | RMD-X4-P36-36-C | RMD-X4-P36-36-E |
| X6-P20 | RMD-X6-P20-60-C | RMD-X6-P20-60-E |
| X8-P20 | RMD-X8-P20-120-C | RMD-X8-P20-120-E |
| X8-P33 | RMD-X8-P33-150-C | RMD-X8-P33-150-E |
| X12-P20 | RMD-X12-P20-320-C | RMD-X12-P20-320-E |
| X15-P20 | RMD-X15-P20-450-C | RMD-X15-P20-450-E |

### 3.2. Module Appearance Size

*[See figure in original PDF for each of the following dimension drawings — the figures themselves are not transcribable as text, but are listed here for reference:]*

- Figure 3-1 — X2-P28 appearance dimension drawing
- Figure 3-2 — X4-P12.5 appearance dimension drawing
- Figure 3-3 — X4-P36 appearance dimension drawing
- Figure 3-4 — X6-P20 appearance dimension drawing
- Figure 3-5 — X8-P20 appearance dimension drawing
- Figure 3-6 — X12-P20 appearance dimension drawing
- Figure 3-7 — X15-P20 appearance dimension drawing

### 3.3. Module Parameters

**Table 3-3 — Module Parameter List**

| Parameter | X2-P28 | X4-P12.5 | X4-P36 | X6-P20 | X8-P20 | X8-P33 | X12-P20 | X15-P20 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Ratio | 28.17 | 12.6 | 36 | 19.612 | 19.612 | 33 | 20 | 20.25 |
| Input Voltage (V) | 24 | 24 | 24 | 48 | 48 | 48 | 48 | 72 |
| Peak Speed (RPM) | 178 | 317 | 111 | 176 | 158 | 90 | 125 | 108 |
| No-load Current (A) | 1.0 | 1.0 | 0.9 | 0.9 | 1.6 | 1.5 | 2.7 | 3.5 |
| Rated Speed (RPM) | 142 | 238 | 83 | 153 | 127 | 75 | 100 | 98 |
| Rated Torque (N·m) | 2.5 | 4.0 | 10.5 | 20.0 | 43.0 | 66 | 85 | 145.0 |
| Rated Output Power (W) | 37 | 100 | 100 | 320 | 574 | 520 | 900 | 1480 |
| Rated Current (Arms) | 3.0 | 7.8 | 6.1 | 9.5 | 17.6 | 15 | 30.0 | 25.0 |
| Peak Torque (N·m) | 7 | 10 | 34 | 60 | 120 | 150 | 320 | 450 |
| Peak Current (Arms) | 8.1 | 19.5 | 21.5 | 29.1 | 43.8 | 34 | 100.0 | 69.2 |
| Efficiency (%) | 63 | 69.5 | 63.1 | 72.7 | 79.0 | 77 | 75 | 82.4 |
| EMF Constant (Vdc/Krpm) | 4.3 | 6.0 | 6.0 | 16.0 | 19.2 | 19.2 | 17.9 | 29.9 |
| Torque Constant (N·m/A) | 0.8 | 0.8 | 1.9 | 2.1 | 2.4 | 4.4 | 3.3 | 5.8 |
| Wire Resistance (Ω) | 0.61 | 0.32 | 0.35 | 0.41 | 0.18 | 0.16 | 0.12 | 0.08 |
| Wire Inductance (mH) | 0.13 | 0.14 | 0.17 | 0.51 | 0.31 | 0.33 | 0.05 | 0.14 |
| Polar (pole pairs) | 13 | 13 | 13 | 10 | 10 | 10 | 20 | 20 |
| Wiring Method | Y | Y | Y | Y | Y | Y | Y | Y |
| Backlash (Arcmin) | 12 | 10 | 10 | 10 | 10 | 12 | 10 | 10 |
| Output shaft bearing type | Deep groove ball bearing | Deep groove ball bearing | Deep groove ball bearing | Deep groove ball bearing | Cross roller bearing | Cross roller bearing | Cross roller bearing | Cross roller bearing |
| Radial Load (KN) | 1 | 1.2 | 1.5 | 2 | 4.5 | 6 | 5 | 6 |
| Axial Load — Tension (KN) | 0.25 | 1.2 | 1.3 | 1.8 | 4 | 6 | 4.5 | 5.4 |
| Axial Load — Compression (KN) | 0.25 | 1.2 | 1.3 | 0.8 | 1 | 6 | 4.5 | 5.4 |
| Repeatability (Degree) | 0.17 | 0.25 | 0.3 | 0.66 | 1.5 | 2.4 | 12.9 | 31.6 |
| Encoder Type | Dual Encoder, ABS-17BIT/ABS-18BIT | Dual Encoder, ABS-17BIT/ABS-18BIT | Dual Encoder, ABS-17BIT/ABS-18BIT | Dual Encoder, ABS-17BIT/ABS-18BIT | Dual Encoder, ABS-17BIT/ABS-17B [as printed in source] | Dual Encoder, ABS-17BIT/ABS-17B [as printed in source] | Dual Encoder, ABS-17BIT/ABS-17B [as printed in source] | Dual Encoder, ABS-17BIT/ABS-17B [as printed in source] |
| Control accuracy (Degree) | < 0.01 | < 0.01 | < 0.01 | < 0.01 | < 0.01 | < 0.01 | < 0.01 | < 0.01 |
| Communication Method | EtherCAT/CAN | EtherCAT/CAN | EtherCAT/CAN | EtherCAT/CAN | EtherCAT/CAN | EtherCAT/CAN | EtherCAT/CAN | EtherCAT/CAN |
| Weight (Kg) | 0.26 | 0.26 | 0.36 | 0.82 | 1.4 | 1.48 | 2.37 | 3.5 |
| Insulation level | F | F | F | F | F | F | F | F |

> **Note on transcription:** This table's columns are misaligned in the plain-text PDF dump (`pdftotext -layout`), so every value above was re-verified against the PDF's word bounding boxes (`pdftotext -bbox-layout`, pages 14–15) rather than trusted from the naive text layout. In particular, for the **X4-P36** column: Ratio = 36, Input Voltage = 24 V, Peak Speed = 111 RPM, Rated Torque = 10.5 N·m, Peak Torque = 34 N·m, Rated Current = 6.1 Arms, **Peak Current = 21.5 Arms**, Torque Constant = 1.9 N·m/A. The Peak Current value is **21.5 Arms, not 19.5 Arms** — 19.5 Arms is the X4-P12.5 column's Peak Current; the naive `-layout` text dump visually suggests a one-column shift on that row which bbox coordinates disprove.
>
> The "Encoder Type" row for the X8-P20/X8-P33/X12-P20/X15-P20 group reads `ABS-17BIT/ABS-17B` in the source PDF (missing a trailing "IT" compared to the smaller-module group's `ABS-17BIT/ABS-18BIT`); this looks like a typo in the original document and is transcribed as printed.

### 3.4. Module Stall Characteristics

When the module transitions from a stationary state to start-up operation, the significant initial static-friction force requires a higher torque to initiate movement. This can cause the speed to drop to 0 rpm, an increase in current, and a rapid rise in temperature. Below are the temperature-increase scenarios for the module when it exceeds the rated load.

**Table 3-4 — Stall Data for X2-P28**

| Torque (N·m) | Temperature Rise (°C) | Stall Time (s) | Phase Current (Arms) |
| --- | --- | --- | --- |
| 3.75 | 20 | 15 | 4.3 |
| 5 | 48 | 10 | 5.7 |
| 6.25 | 31 | 8 | 7.4 |
| 7.5 | 59 | 5 | 8.6 |

**Table 3-5 — Stall Data for X4-P36**

| Torque (N·m) | Temperature Rise (°C) | Stall Time (s) | Phase Current (Arms) |
| --- | --- | --- | --- |
| 17.25 | 30 | 15 | 9.2 |
| 23 | 58 | 10 | 12.7 |
| 28.75 | 41 | 5 | 16.3 |
| 34.5 | 50 | 3 | 21.2 |

**Table 3-6 — Stall Data for X6-P20**

| Torque (N·m) | Temperature Rise (°C) | Stall Time (s) | Phase Current (Arms) |
| --- | --- | --- | --- |
| 30 | 17 | 15 | 12.7 |
| 40 | 29 | 10 | 17.7 |
| 50 | 37 | 8 | 22.6 |
| 60 | 24 | 5 | 28.3 |

**Table 3-7 — Stall Data for X8-P20**

| Torque (N·m) | Temperature Rise (°C) | Stall Time (s) | Phase Current (Arms) |
| --- | --- | --- | --- |
| 64.5 | 7 | 15 | 23.3 |
| 86 | 10 | 10 | 31.1 |
| 107.5 | 26 | 8 | 38.9 |
| 129 | 30 | 5 | 43.8 |

**Table 3-8 — Stall Data for X12-P20**

| Torque (N·m) | Temperature Rise (°C) | Stall Time (s) | Phase Current (Arms) |
| --- | --- | --- | --- |
| 150 | 13 | 10 | 37.5 |
| 200 | 5 | 8 | 49.5 |
| 250 | 7 | 7 | 61.5 |
| 300 | 43 | 3 | 75.3 |

**Table 3-9 — Stall Data for X15-P20**

| Torque (N·m) | Temperature Rise (°C) | Stall Time (s) | Phase Current (Arms) |
| --- | --- | --- | --- |
| 217.5 | 15 | 15 | 31.1 |
| 290 | 15 | 10 | 41 |
| 362.5 | 20 | 8 | 51.6 |
| 435 | 25 | 5 | 67.2 |

---

## 4. Mechanical Installation Requirements

Carry out structural design and assembly based on the drawings of each model module provided by the company. Refer to this chapter for details of the screw types and techniques required for assembly. During assembly, all fixing screws must be threaded with thread glue; the position and amount of thread glue applied must be consistent for every screw. Use the diagonal (cross) method to tighten the screws. The specific tightening steps are as follows:

1. Tighten the screw to the end but do not fully tighten it;
2. Slightly tighten the screws in diagonal steps;
3. Tighten the screws using a torque wrench in diagonal steps.

For screw tightening torque, refer to Table 4-1.

**Table 4-1 — Screw Tightening Torque Table**

| Screw specification (mm) | Tightening torque (kgf·cm) | Screw specification (mm) | Tightening torque (kgf·cm) |
| --- | --- | --- | --- |
| M3 × 0.5 | 17 | M14 × 2.0 | 1840 |
| M4 × 0.7 | 40 | M16 × 2.0 | 2870 |
| M5 × 0.8 | 81 | M18 × 2.5 | 3950 |
| M6 × 1.0 | 138 | M20 × 2.5 | 5600 |
| M8 × 1.25 | 334 | M22 × 2.5 | 7620 |
| M10 × 1.25 | 663 | M24 × 3.0 | 9680 |

Foreign matter such as metal shavings, dust particles, and various types of sealant may adhere to the installation surface, preventing reliable mating of the installation surfaces and causing jitter and noise in the module. Therefore, clean the installation surface carefully before installation.

---

## 5. Electrical Installation Requirements

### 5.1. About the Input Power Supply

The power supply uses 48 VDC or 72 VDC power, and the input-voltage specification varies among the different module models — refer to Section 3.3 (Module Parameters) for details. Modules with an input voltage of 48 VDC have a maximum withstand voltage of 55 VDC at the power interface, while those with an input voltage of 72 VDC have a maximum withstand voltage of 90 VDC; exceeding the maximum withstand voltage can easily lead to driver failure.

When a switch is used to control power to the joint, there may be an overvoltage transient at the moment of power-up. This power-supply mode needs an electrolytic capacitor connected in parallel after the switch and before the joint's power input (reference specification: 1000 µF / 100 V — the specific specification should be selected based on the actual filtering effect required), as shown in Figure 5-1, to suppress overshoot of the input voltage at the moment of power-up.

*[See figure in original PDF — Figure 5-1: Protection circuits in the case of switching power supplies.]*

There is no need to consider the effect of back-EMF when using battery power, because the module's back-EMF directly charges the battery. To make the system safer and more reliable, the over-voltage/under-voltage protection thresholds of the module can be modified according to the actual test situation. The modification path is: **debugging host computer → advanced parameters → protection parameters → over-voltage/under-voltage protection voltage**, as shown in Figure 5-2.

*[See figure in original PDF — Figure 5-2: Schematic diagram of over-voltage/under-voltage protection voltage modification.]*

### 5.2. Interface Description

The interfaces are described below. Each module variant exposes a power connector (VCC/GND), a CAN connector (CAN_L/CAN_H), and EtherCAT input/output connectors (`EtherCAT_IN`/`EtherCAT_OUT`); the EtherCAT connectors additionally break out to four signal pins, `T+`/`T-`/`R+`/`R-`, whose function is identical across all module variants:

- `T+` — Master sends control commands to the module
- `T-` — Module sends status feedback to the master
- `R+` — Master reflects the status data of the module
- `R-` — Module reflects the control commands of the master

**Table 5-1 — Interface description for X2-P28/X4-P36**

| Port | Port description |
| --- | --- |
| VCC | The positive pole of the power supply |
| GND | The negative pole of the power supply |
| CAN_L | CAN_L network signal interface |
| CAN_H | CAN_H network signal interface |
| EtherCAT_IN | EtherCAT input port |
| EtherCAT_OUT | EtherCAT output port |
| T+ | Master sends control commands to the module |
| T- | Module sends status feedback to the master |
| R+ | Master reflects the status data of the module |
| R- | Module reflects the control commands of the master |

**Table 5-2 — Interface description for X6-P20**

| Port | Port description |
| --- | --- |
| VCC | The positive pole of the power supply |
| GND | The negative pole of the power supply |
| CAN_L | CAN_L network signal interface |
| CAN_H | CAN_H network signal interface |
| EtherCAT_IN | EtherCAT input port |
| EtherCAT_OUT | EtherCAT output port |
| R+ | Master reflects the status data of the module |
| R- | Module reflects the control commands of the master |
| T+ | Master sends control commands to the module |
| T- | Module sends status feedback to the master |

**Table 5-3 — Interface description for X8-P20**

| Port | Port description |
| --- | --- |
| VCC | The positive pole of the power supply |
| GND | The negative pole of the power supply |
| CAN_L | CAN_L network signal interface |
| CAN_H | CAN_H network signal interface |
| EtherCAT_IN | EtherCAT input port |
| EtherCAT_OUT | EtherCAT output port |
| R+ | Master reflects the status data of the module |
| R- | Module reflects the control commands of the master |
| T+ | Master sends control commands to the module |
| T- | Module sends status feedback to the master |

**Table 5-4 — Interface description for X12-P20/X15-P20**

| Port | Port description |
| --- | --- |
| VCC | The positive pole of the power supply |
| GND | The negative pole of the power supply |
| CAN_L | CAN_L network signal interface |
| CAN_H | CAN_H network signal interface |
| EtherCAT_IN | EtherCAT input port |
| EtherCAT_OUT | EtherCAT output port |
| R+ | Master reflects the status data of the module |
| R- | Module reflects the control commands of the master |
| T+ | Master sends control commands to the module |
| T- | Module sends status feedback to the master |

> **Note on transcription:** Tables 5-1 through 5-4 list the same ports; only the row order of `T+`/`T-` vs. `R+`/`R-` differs between them in the source PDF (verified via `pdftotext -bbox-layout`, pages 20–21) — the pin-to-function mapping itself is identical across all module variants.

### 5.3. Indicator Description

**Table 5-5 — Explanation of the status of the indicator**

| Expression | Situation |
| --- | --- |
| The green light is always on | The motor is operating normally |
| The green light flashes rapidly | There is a level-1 error in the motor |
| The green light flashes slowly | There is a level-2 error in the motor |

If there is an error in the operation of the motor, read the specific error information from the host computer or via a CAN command, and refer to the "Setup Software Instruction Manual" and "Servo Motor Control Protocol" for the specific error reason.

---

## 6. Cable Connection between Multi-joint Modules

### 6.1. Description of the Power Supply Wiring

There are two power-wiring modes for this series of joint modules: single-axis direct connection and chain-topology connection, as shown in Figure 6-1. When applied to the collaborative work of multiple modules, the two wiring methods perform differently: single-axis direct connection has small wiring resistance and small line-loss voltage drop; the chain topology has somewhat larger wiring resistance and a somewhat larger line-loss voltage drop. It is therefore recommended to use a single-axis direct connection for high-power modules and a chain-type topology connection for low-power modules.

**Note:** Do not connect to other electrical devices in series, as it may cause unpredictable voltage drops or voltage boosts that may cause module failure.

*[See figure in original PDF — Figure 6-1: Schematic diagram of multi-module power supply — (a) single-axis direct connection, (b) chain topology connection.]*

### 6.2. CAN Communication Wiring Instructions

The CAN communication line is made of twisted-pair cable, individually shielded; it is important to ensure that the ID of each module is unique before establishing CAN communication. In addition, the CAN communication controller and module use the ground-connection method for power supply, as shown in Figure 6-2. To eliminate signal reflections in the communication cables, a 120 Ω termination resistor is required in parallel at the CAN communication interface of both the controller and the end module.

*[See figure in original PDF — Figure 6-2: Schematic diagram of CAN communication wiring.]*

### 6.3. EtherCAT Communication Wiring Instructions

The EtherCAT communication cable uses twisted-pair cable and is individually shielded, as shown in Figure 6-3. If this communication method is used, it is recommended to keep the CAN communication line connected during setup and to set the communication ID for subsequent debugging and troubleshooting.

*[See figure in original PDF — Figure 6-3: Schematic diagram of EtherCAT communication wiring.]*

---

## 7. Kinetic Energy Recovery

### 7.1. Reasons for Kinetic Energy Recovery

During normal operation of the module, the power supply outputs electrical energy to it. When the module decelerates, the circuit loop engages in kinetic-energy recovery. Figure 7-1 shows a simplified circuit diagram of the module during normal operation and deceleration. The amount of kinetic energy recovered is related to torque and rotational speed, and is directly proportional to the product of torque and rotational speed — the faster the speed and the greater the load, the more kinetic energy is recovered. If the power-supply voltage rises above the maximum allowable bus voltage set by the drive, the module will report an error for excessively high bus voltage.

*[See figure in original PDF — Figure 7-1: Diagram of module operating states — (a) normal operating state, (b) deceleration operating state.]*

### 7.2. Handling Methods

**1. Adding a bleeder resistor**

By paralleling a resistor, when the module is decelerating the recovered kinetic energy is consumed through the resistor, preventing the power-supply voltage from becoming too high due to kinetic-energy recovery. As shown in Figure 7-2, the disconnection/connection of the resistor can be operated through a logic control circuit.

*[See figure in original PDF — Figure 7-2: Diagram of module operating state with bleeder resistor connected — (a) resistor disconnected when voltage is normal, (b) resistor connected when voltage is too high.]*

**2. Adding a super capacitor**

During normal operation, the switching power supply powers both the super capacitor and the module. When the module decelerates, the super capacitor rapidly absorbs part of the kinetic energy, preventing the power-supply voltage from becoming too high, as shown in Figure 7-3.

*[See figure in original PDF — Figure 7-3: Diagram of module operating state with super capacitor added — (a) charging the capacitor when voltage is normal, (b) capacitor kinetic-energy recovery when voltage is too high.]*

**3. Adding a storage battery**

During normal operation, both the switching power supply and the storage battery power the module simultaneously. When the module decelerates, the storage battery recovers the kinetic energy, as shown in Figure 7-4.

*[See figure in original PDF — Figure 7-4: Diagram of module operating state with storage battery added — (a) storage battery powers the system when voltage is normal, (b) storage battery recovers kinetic energy when voltage is too high.]*

---

## 8. Encoder Description

### 8.1. Resolution and Position Feedback

The module is controlled by dual absolute encoders and achieves full closed-loop control. Encoder resolution varies slightly between module models — for specific values, refer to Section 3.3 (Module Parameters). When the encoder resolution is 17 bits, the number of position counts output by the motor shaft for one complete rotation is 2¹⁷ (131072), with a single-turn position range of 0 to 131071. The conversion formula between angle and single-turn position is:

```
single_turn_position = angle / 360 * 131072
```

For example, a lap angle of 30° corresponds to a lap position of `30 / 360 * 131072`.

When the encoder position crosses a boundary, the multi-turn count is updated: when the position moves in the negative direction past 0, the position wraps from 0 to 131071 and the turn count decreases by 1; when the position moves in the positive direction past 131071, it wraps from 131071 to 0 and the turn count increases by 1. The encoder's current (multi-turn) position is therefore calculated as:

```
position = turn_count * 131072 + single_turn_position
```

The current position can be obtained through the CAN bus or the setup software; the motor-operation interface status bar of the setup software displays the current motor angle in real time. Detailed setup-software functions and operating procedures can be found in the company's "Setup Software Instruction Manual", and CAN communication control instructions are detailed in the "Servo Motor Control Protocol".

### 8.2. Instructions for the Use of the Mechanical Zero Calibration Function

Users can use the mechanical zero-point calibration function to flexibly set the mechanical zero-point value according to the intended use of the module. There are two ways to set it:

1. Connect and debug via the setup software, and set the zero point in the basic-parameter interface — see the company's "Setup Software Instruction Manual" for details.
2. CAN command setting — refer to the company's "Servo Motor Control Protocol" for details.

---

## 9. Connect and Debug the Setup Software

Download the latest debugging software from the company's official website. The installation and debugging method is detailed in the "Setup Software Instruction Manual".

---

## 10. Communication Instruction Description

The X series module adopts the company's customized communication instructions; the communication control instructions are detailed in the "Servo Motor Control Protocol".

> **Note on transcription:** The source PDF literally reads "The RH series module adopts our company's customized communication instructions..." in this section — "RH series" appears to be a leftover from another product manual template and is inconsistent with the rest of this document, which otherwise refers throughout to the "X series" exclusively. It has been corrected to "X series" here; flagged for cross-checking against the official document if precision on this point matters.
