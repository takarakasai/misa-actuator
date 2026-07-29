# RMD-X V4 Series — Product Manual

Manufacturer: Suzhou Micro Actuator Technology Co., Ltd. (MYACTUATOR)

RMD-X V4 Series Planetary Actuator — Special Series For Robot.

## Product Features

- EtherCAT & CAN BUS communication
- Crossed roller bearings (on higher-torque models)
- Dual encoder (single encoder on the RS485 variant)
- High torque density
- High precision
- Hollow design

## Naming Convention

Example part number breakdown: `RMD - X2 - P28 - 7 - E`

| # | Field | Meaning |
|---|-------|---------|
| 1 | `RMD` | Brand name: **R**educer, **M**otor, **D**rive |
| 2 | `X2` | Series name: Integrated Planetary Actuator. The number (`2`, `4`, `6`, `8`, `12`, `15`, ...) is the motor model number, e.g. X2, X4, X6, X8, X12, X15 |
| 3 | `P28` | Planetary gear ratio, e.g. P12, P28, P32, etc. |
| 4 | `7` | Peak torque in N·m |
| 5 | `E` | Communication interface: EtherCAT & CAN BUS |

### Second example

Model: `RMD-X8-P20-120-E`

| Segment | Meaning |
|---------|---------|
| `RMD` | Brand name |
| `X8` | Motor model name |
| `P20` | Planetary gear ratio, 20:1 |
| `120` | Peak torque (N·m) |
| `E` | EtherCAT & CAN BUS |

Note: the communication suffix is not always `E`. Observed variants in this series include `E` (EtherCAT & CAN BUS), `C` (CAN BUS only), and `R` (RS485).

## Product Parameters by Model

Each model below lists: actuator full name, brake/communication configuration, full parameter table, and stall-torque-vs-time data. All models in this brochure ship as "N (without Brake)".

### X2-7

- **Actuator Full Name:** RMD-X2-P28-7-E
- **Communication:** EtherCAT & CAN BUS
- **Encoder:** Dual Encoder ABS-17BIT (Input) / 18BIT (Output)

| Parameter | Unit | Value |
|---|---|---|
| Gear Ratio | – | 28.17 |
| Input Voltage | V | 24 |
| No Load Speed | RPM | 178 |
| No-Load Input Current | A | 1 |
| Rated Speed | RPM | 142 |
| Rated Torque | N·m | 2.5 |
| Rated Power | W | 37 |
| Rated Current | A | 3 |
| Peak Torque | N·m | 7 |
| Peak Current | A | 8.1 |
| Efficiency | % | 63 |
| Motor Back-EMF Constant | Vdc/Krpm | 4.3 |
| Module Torque Constant | N·m/A | 0.8 |
| Motor Phase Resistance | Ω | 0.61 |
| Motor Phase Inductance | mH | 0.13 |
| Pole Pair | – | 13 |
| 3 Phase Connection | – | Y |
| Back Drive Torque | N·m | 0.4 |
| Backlash | Arcmin | ≤15 |
| Output Bearing Type | – | Deep Groove Ball Bearings |
| Axial Load – Tensile | KN | 0.25 |
| Axial Load – Compressive | KN | 0.25 |
| Radial Load | KN | 1 |
| Inertia | Kg·cm² | 0.17 |
| Encoder Type & Interface | – | Dual Encoder ABS-17BIT(Input) / 18BIT(Output) |
| Control Accuracy | Degree | <0.01 |
| Communication | – | EtherCAT & CAN BUS |
| Weight | Kg | 0.26 |
| Insulation Grade | – | F |

**Stall Torque Data (X2-7)**

| Torque (N·m) | Temperature Rise (°C) | Stall Time (s) | Phase Current (Arms) |
|---|---|---|---|
| 3.75 | 20 | 15 | 4.3 |
| 5 | 48 | 10 | 5.7 |
| 6.25 | 31 | 8 | 7.4 |
| 7.5 | 59 | 5 | 8.6 |

**Mechanical / installation drawing (approximate, reconstructed from a garbled multi-column CAD callout — verify against the original PDF before use):**
- Two 2×⌀3 +0.02/0 locating-pin patterns, depth 4
- Two 4×M3 bolt patterns on P.C.D. 25 and P.C.D. 38 (depths 5 mm and 4 mm)
- Spigot/shaft diameters: ⌀31 −0.03/0 and ⌀44 −0.1/0
- Overall output flange diameter: ⌀63.50 ±0.5

### X4-10

- **Actuator Full Name:** RMD-X4-P12-10-E
- **Communication:** EtherCAT & CAN BUS
- **Encoder:** Dual Encoder ABS-17BIT (Input) / 18BIT (Output)

| Parameter | Unit | Value |
|---|---|---|
| Gear Ratio | – | 12.5 |
| Input Voltage | V | 24 |
| No Load Speed | RPM | 317 |
| No-Load Input Current | A | 1 |
| Rated Speed | RPM | 238 |
| Rated Torque | N·m | 4 |
| Rated Output Power | W | 100 |
| Rated Phase Current | A(rms) | 7.8 |
| Peak Torque | N·m | 10 |
| Peak Phase Current | A(rms) | 19.5 |
| Efficiency | % | 69.5 |
| Motor Back-EMF Constant | Vdc/Krpm | 6 |
| Module Torque Constant | N·m/A | 0.8 |
| Motor Phase Resistance | Ω | 0.32 |
| Motor Phase Inductance | mH | 0.14 |
| Pole Pair | – | 13 |
| 3 Phase Connection | – | Y |
| Back Drive Torque | N·m | 0.8 |
| Backlash | Arcmin | ≤15 |
| Output Bearing Type | – | Deep Groove Ball Bearings |
| Axial Load – Tensile | KN | 1.2 |
| Axial Load – Compressive | KN | 1.2 |
| Radial Load | KN | 1.2 |
| Inertia | Kg·cm² | 0.25 |
| Encoder Type & Interface | – | Dual Encoder ABS-17BIT (Input) / 18BIT(Output) |
| Control Accuracy | Degree | <0.01 |
| Communication | – | EtherCAT & CAN BUS |
| Weight | Kg | 0.33 |
| Insulation Grade | – | F |

**Stall Torque Data (X4-10)**

| Torque (N·m) | Temperature Rise (°C) | Stall Time (s) | Phase Current (Arms) |
|---|---|---|---|
| 5.2 | 41 | 10 | 7.8 |
| 6 | 16 | 8 | 9.1 |
| 7.2 | 26 | 5 | 11 |
| 8 | 30 | 3 | 12.1 |

**Mechanical / installation drawing:** an installation drawing is present in the source, but the extracted dimension fragments (`2.5`, `4`) were too incomplete to reliably reconstruct — refer to the original PDF for exact figures.

### X4-36

- **Actuator Full Name:** RMD-X4-P36-36-E
- **Communication:** EtherCAT & CAN BUS
- **Encoder:** Dual Encoder ABS-17BIT (Input) / 18BIT (Output)

| Parameter | Unit | Value |
|---|---|---|
| Gear Ratio | – | 36 |
| Input Voltage | V | 24 |
| No Load Speed | RPM | 111 |
| No-Load Input Current | A | 0.9 |
| Rated Speed | RPM | 83 |
| Rated Torque | N·m | 10.5 |
| Rated Output Power | W | 100 |
| Rated Phase Current | A(rms) | 6.1 |
| Peak Torque | N·m | 34 |
| Peak Phase Current | A(rms) | 21.5 |
| Efficiency | % | 63.1 |
| Motor Back-EMF Constant | Vdc/Krpm | 6 |
| Module Torque Constant | N·m/A | 1.9 |
| Motor Phase Resistance | Ω | 0.39 |
| Motor Phase Inductance | mH | 0.07 |
| Pole Pair | – | 11 |
| 3 Phase Connection | – | Y |
| Back Drive Torque | N·m | 1.14 |
| Backlash | Arcmin | ≤15 |
| Output Bearing Type | – | Crossed Roller Bearings |
| Axial Load – Tensile | KN | 1.3 |
| Axial Load – Compressive | KN | 1.3 |
| Radial Load | KN | 1.5 |
| Inertia | Kg·cm² | 0.3 |
| Encoder Type & Interface | – | Dual Encoder ABS-17BIT (Input) / 18BIT (Output) |
| Control Accuracy | Degree | <0.01 |
| Communication | – | EtherCAT & CAN BUS |
| Weight | Kg | 0.36 |
| Insulation Grade | – | F |

**Stall Torque Data (X4-36)**

| Torque (N·m) | Temperature Rise (°C) | Stall Time (s) | Phase Current (Arms) |
|---|---|---|---|
| 17.25 | 30 | 15 | 9.2 |
| 23 | 58 | 10 | 12.7 |
| 28.75 | 41 | 5 | 16.3 |
| 34.5 | 50 | 3 | 21.2 |

No installation-drawing dimension figures were recoverable for this model from the OCR text.

### X6-8

- **Actuator Full Name:** RMD-X6-P8-8-C
- **Communication:** CAN BUS
- **Encoder:** Dual Encoder ABS-17BIT (Input) / 18BIT (Output)

| Parameter | Unit | Value |
|---|---|---|
| Gear Ratio | – | 8 |
| Input Voltage | V | 48 |
| No Load Speed | RPM | 387 |
| No-Load Input Current | A | 1.1 |
| Rated Speed | RPM | 310 |
| Rated Torque | N·m | 4.5 |
| Rated Output Power | W | 135 |
| Rated Phase Current | A(rms) | 3.6 |
| Peak Torque | N·m | 8 |
| Peak Phase Current | A(rms) | 7.2 |
| Efficiency | % | 78 |
| Motor Back-EMF Constant | Vdc/Krpm | 19.4 |
| Module Torque Constant | N·m/A | 1.3 |
| Motor Phase Resistance | Ω | 1.1 |
| Motor Phase Inductance | mH | 0.57 |
| Pole Pair | – | 14 |
| 3 Phase Connection | – | Y |
| Back Drive Torque | N·m | 0.8 |
| Backlash | Arcmin | 10 |
| Output Bearing Type | – | Deep Groove Ball Bearings |
| Axial Load – Tensile | KN | 0.85 |
| Axial Load – Compressive | KN | 0.775 |
| Radial Load | KN | 1.04 |
| Inertia | Kg·cm² | 0.61 |
| Encoder Type & Interface | – | Dual Encoder ABS-17BIT(Input) / 18BIT(Output) |
| Control Accuracy | Degree | <0.01 |
| Communication | – | CAN BUS |
| Weight | Kg | 0.49 |
| Insulation Grade | – | F |

**Stall Torque Data (X6-8)**

| Torque (N·m) | Temperature Rise (°C) | Stall Time (s) | Phase Current (Arms) |
|---|---|---|---|
| 6.75 | 30 | 10 | 7 |
| 9 | 49 | 8 | 9.5 |
| 11.25 | 31 | 5 | 11.3 |
| 13.5 | 19 | 3 | 12.7 |

No installation-drawing dimension figures were recoverable for this model from the OCR text.

### X6-60

- **Actuator Full Name:** RMD-X6-P20-60-E
- **Communication:** EtherCAT & CAN BUS
- **Encoder:** Dual Encoder ABS-17BIT (Input) / 17BIT (Output)

| Parameter | Unit | Value |
|---|---|---|
| Gear Ratio | – | 19.612 |
| Input Voltage | V | 48 |
| No Load Speed | RPM | 176 |
| No-Load Input Current | A | 0.9 |
| Rated Speed | RPM | 153 |
| Rated Torque | N·m | 20 |
| Rated Output Power | W | 320 |
| Rated Phase Current | A(rms) | 9.5 |
| Peak Torque | N·m | 60 |
| Peak Phase Current | A(rms) | 29.1 |
| Efficiency | % | 72.7 |
| Motor Back-EMF Constant | Vdc/Krpm | 16 |
| Module Torque Constant | N·m/A | 2.1 |
| Motor Phase Resistance | Ω | 0.41 |
| Motor Phase Inductance | mH | 0.51 |
| Pole Pair | – | 10 |
| 3 Phase Connection | – | Y |
| Back Drive Torque | N·m | 1.6 |
| Backlash | Arcmin | ≤15 |
| Output Bearing Type | – | Crossed Roller Bearings |
| Axial Load – Tensile | KN | 1.8 |
| Axial Load – Compressive | KN | 0.8 |
| Radial Load | KN | 2 |
| Inertia | Kg·cm² | 0.66 |
| Encoder Type & Interface | – | Dual Encoder ABS-17BIT(Input) / 17BIT(Output) |
| Control Accuracy | Degree | <0.01 |
| Communication | – | EtherCAT & CAN BUS |
| Weight | Kg | 0.82 |
| Insulation Grade | – | F |

**Stall Torque Data (X6-60)**

| Torque (N·m) | Temperature Rise (°C) | Stall Time (s) | Phase Current (Arms) |
|---|---|---|---|
| 30 | 17 | 15 | 12.7 |
| 40 | 29 | 10 | 17.7 |
| 50 | 37 | 8 | 22.6 |
| 60 | 24 | 5 | 28.3 |

**Mechanical / installation drawing (approximate, reconstructed — verify against the original PDF before use):**
- Hollow through-bore: ⌀8.6
- Overall body diameter: ⌀80 ±0.1
- Mounting-side bolt pattern: 10×⌀3.4, depth 3, on P.C.D. 44
- Output-side bolt pattern: 12×M4, depth 5, on P.C.D. 72.5
- Spigot diameter: ⌀53 −0.03/0; shaft diameter: ⌀5 −0.02/0
- Overall output flange diameter: ⌀67.5 ±0.5

### X8-32

- **Actuator Full Name:** RMD-X8-P9-32-R
- **Communication:** RS485
- **Encoder:** Single Encoder ABS-18BIT

| Parameter | Unit | Value |
|---|---|---|
| Gear Ratio | – | 9 |
| Input Voltage | V | 24 |
| No Load Speed | RPM | 277 |
| No-Load Input Current | A | 0.9 |
| Rated Speed | RPM | 244 |
| Rated Torque | N·m | 8 |
| Rated Output Power | W | 204 |
| Rated Phase Current | A(rms) | 6.2 |
| Peak Torque | N·m | 32 |
| Peak Phase Current | A(rms) | 30 |
| Efficiency | % | 82 |
| Motor Back-EMF Constant | Vdc/Krpm | 10.9 |
| Module Torque Constant | N·m/A | 1.3 |
| Phase Resistance | Ω | 0.13 |
| Phase Inductance | mH | 0.08 |
| Pole Pair | – | 21 |
| 3 Phase Connection | – | △ (Delta) |
| Back Drive Torque | N·m | 0.8 |
| Backlash | Arcmin | ≤10 |
| Output Bearing Type | – | Deep Groove Ball Bearings |
| Axial Load – Tensile | KN | 0.6 |
| Axial Load – Compressive | KN | 0.6 |
| Radial Load | KN | 2 |
| Inertia | Kg·cm² | 1.43 |
| Encoder Type & Interface | – | Single Encoder ABS-18BIT |
| Control Accuracy | Degree | <0.01 |
| Communication | – | RS485 |
| Weight | Kg | 0.55 |
| Insulation Grade | – | F |

**Stall Torque Data (X8-32)**

| Torque (N·m) | Temperature Rise (°C) | Stall Time (s) | Phase Current (Arms) |
|---|---|---|---|
| 15.6 | 6 | 10 | 18.4 |
| 18 | 11 | 8 | 21.2 |
| 24 | 36 | 5 | 29 |
| 30 | 54 | 3 | 38.2 |

Note: this is the only model in the brochure using a Delta (△) 3-phase connection, single encoder, and RS485 communication — all other models use a Y connection, dual encoder, and EtherCAT & CAN BUS (or plain CAN BUS).

**Mechanical / installation drawing (approximate, reconstructed — verify against the original PDF before use):**
- Two 8×M3-6H bolt patterns, depth 6, at 45° spacing (input and output faces)
- Bolt circle diameters observed: P.C.D. 34 and P.C.D. 88
- Overall body diameters: ⌀96 ±0.1 and ⌀47 ±0.1
- Spigot diameter: ⌀59 −0.03/0
- Dowel pattern: 3×⌀9 +0.04/0, depth 3
- Additional output bolt pattern: 6×M4-6H, depth 8, on P.C.D. 38

### X8-120

- **Actuator Full Name:** RMD-X8-P20-120-E
- **Communication:** EtherCAT & CAN BUS
- **Encoder:** Dual Encoder ABS-17BIT (Input) / 17BIT (Output)

| Parameter | Unit | Value |
|---|---|---|
| Gear Ratio | – | 19.612 |
| Input Voltage | V | 48 |
| No Load Speed | RPM | 158 |
| No-Load Input Current | A | 1.6 |
| Rated Speed | RPM | 127 |
| Rated Torque | N·m | 43 |
| Rated Output Power | W | 574 |
| Rated Phase Current | A(rms) | 17.6 |
| Peak Torque | N·m | 120 |
| Peak Phase Current | A(rms) | 43.8 |
| Efficiency | % | 79 |
| Motor Back-EMF Constant | Vdc/Krpm | 19.2 |
| Module Torque Constant | N·m/A | 2.4 |
| Motor Phase Resistance | Ω | 0.18 |
| Motor Phase Inductance | mH | 0.31 |
| Pole Pair | – | 10 |
| 3 Phase Connection | – | Y |
| Back Drive Torque | N·m | 1.5 |
| Backlash | Arcmin | ≤15 |
| Output Bearing Type | – | Crossed Roller Bearings |
| Axial Load – Tensile | KN | 4 |
| Axial Load – Compressive | KN | 1 |
| Radial Load | KN | 4.5 |
| Inertia | Kg·cm² | 1.5 |
| Encoder Type & Interface | – | Dual Encoder ABS-17BIT (Input) / 17BIT (Output) |
| Control Accuracy | Degree | <0.01 |
| Communication | – | EtherCAT & CAN BUS |
| Weight | Kg | 1.40 |
| Insulation Grade | – | F |

**Stall Torque Data (X8-120)**

| Torque (N·m) | Temperature Rise (°C) | Stall Time (s) | Phase Current (Arms) |
|---|---|---|---|
| 64.5 | 7 | 15 | 23.3 |
| 86 | 10 | 10 | 31.1 |
| 107.5 | 26 | 8 | 38.9 |
| 129 | 30 | 5 | 43.8 |

**Mechanical / installation drawing (approximate, reconstructed — verify against the original PDF before use):**
- Overall body diameter: ⌀96 −0.1/0
- Mounting-side bolt pattern: 10×⌀3.4, depth 5, on P.C.D. 88
- Through/via hole: ⌀12.60
- Spigot diameter: ⌀64 −0.03/0
- Output-side bolt pattern: 12×M4, depth 8, on P.C.D. 55
- Dowel pattern: 2×⌀4 +0.02/0, depth 4
- Overall output flange diameter: ⌀76 ±0.5

### X12-320

- **Actuator Full Name:** RMD-X12-P20-320-E
- **Communication:** EtherCAT & CAN BUS
- **Encoder:** Dual Encoder ABS-17BIT (Input) / 17BIT (Output)

| Parameter | Unit | Value |
|---|---|---|
| Gear Ratio | – | 20 |
| Input Voltage | V | 48 |
| No Load Speed | RPM | 125 |
| No-Load Input Current | A | 2.7 |
| Rated Speed | RPM | 100 |
| Rated Torque | N·m | 85 |
| Rated Output Power | W | 900 |
| Rated Phase Current | A(rms) | 30 |
| Peak Torque | N·m | 320 |
| Peak Phase Current | A(rms) | 100 |
| Efficiency | % | 75 |
| Motor Back-EMF Constant | Vdc/Krpm | 17.9 |
| Module Torque Constant | N·m/A | 3.3 |
| Motor Phase Resistance | Ω | 0.12 |
| Motor Phase Inductance | mH | 0.05 |
| Pole Pair | – | 20 |
| 3 Phase Connection | – | Y |
| Back Drive Torque | N·m | 3.8 |
| Backlash | Arcmin | ≤15 |
| Output Bearing Type | – | Crossed Roller Bearings |
| Axial Load – Tensile | KN | 4.5 |
| Axial Load – Compressive | KN | 4.5 |
| Radial Load | KN | 5 |
| Inertia | Kg·cm² | 12.9 |
| Encoder Type & Interface | – | Dual Encoder ABS-17BIT (Input) / 17BIT (Output) |
| Control Accuracy | Degree | <0.01 |
| Communication | – | EtherCAT & CAN BUS |
| Weight | Kg | 2.37 |
| Insulation Grade | – | F |

**Stall Torque Data (X12-320)**

| Torque (N·m) | Temperature Rise (°C) | Stall Time (s) | Phase Current (Arms) |
|---|---|---|---|
| 150 | 13 | 10 | 37.5 |
| 200 | 5 | 8 | 49.5 |
| 250 | 7 | 7 | 61.5 |
| 300 | 43 | 3 | 75.3 |

**Mechanical / installation drawing (approximate, reconstructed — verify against the original PDF before use):**
- Overall body diameter: ⌀124 ±0.1
- Mounting-side bolt pattern: 12×M5-6H, depth 9, on P.C.D. 116.5
- Dowel pattern: 3×⌀4 +0.012/0, depth 5
- Spigot diameters: ⌀109 −0.1/0 and ⌀85 −0.03/0
- Through/via hole: ⌀12.6
- Output-side bolt pattern: 16×M4-6H, depth 8, on P.C.D. 72
- Overall output flange diameter: ⌀85 ±0.5

### X15-450

- **Actuator Full Name:** RMD-X15-P20-450-E
- **Communication:** EtherCAT & CAN BUS
- **Encoder:** Dual Encoder ABS-17BIT (Input) / 17BIT (Output)

| Parameter | Unit | Value |
|---|---|---|
| Gear Ratio | – | 20.25 |
| Input Voltage | V | 72 |
| No Load Speed | RPM | 108 |
| No-Load Input Current | A | 3.5 |
| Rated Speed | RPM | 98 |
| Rated Torque | N·m | 145 |
| Rated Output Power | W | 1480 |
| Rated Phase Current | A(rms) | 25 |
| Peak Torque | N·m | 450 |
| Peak Phase Current | A(rms) | 69.2 |
| Efficiency | % | 82.4 |
| Motor Back-EMF Constant | Vdc/Krpm | 29.9 |
| Module Torque Constant | N·m/A | 5.8 |
| Motor Phase Resistance | Ω | 0.08 |
| Motor Phase Inductance | mH | 0.14 |
| Pole Pair | – | 20 |
| 3 Phase Connection | – | Y |
| Back Drive Torque | N·m | 4 |
| Backlash | Arcmin | ≤15 |
| Output Bearing Type | – | Crossed Roller Bearings |
| Axial Load – Tensile | KN | 5.4 |
| Axial Load – Compressive | KN | 5.4 |
| Radial Load | KN | 6 |
| Inertia | Kg·cm² | 31.6 |
| Encoder Type & Interface | – | Dual Encoder ABS-17BIT (Input) / 17BIT (Output) |
| Control Accuracy | Degree | <0.01 |
| Communication | – | EtherCAT & CAN BUS |
| Weight | Kg | 3.50 |
| Insulation Grade | – | F |

**Stall Torque Data (X15-450)**

| Torque (N·m) | Temperature Rise (°C) | Stall Time (s) | Phase Current (Arms) |
|---|---|---|---|
| 217.5 | 15 | 15 | 31.1 |
| 290 | 15 | 10 | 41 |
| 362.5 | 20 | 8 | 51.6 |
| 435 | 25 | 5 | 67.2 |

No installation-drawing dimension figures were recoverable for this model beyond a reference to a through/via hole ("过孔") in the source; refer to the original PDF for exact figures.

## Model Summary

| Model | Gear Ratio | Input Voltage | Peak Torque (N·m) | Rated Torque (N·m) | Communication | Encoder |
|---|---|---|---|---|---|---|
| X2-7 | 28.17 | 24 V | 7 | 2.5 | EtherCAT & CAN BUS | Dual, 17/18-bit |
| X4-10 | 12.5 | 24 V | 10 | 4 | EtherCAT & CAN BUS | Dual, 17/18-bit |
| X4-36 | 36 | 24 V | 34 | 10.5 | EtherCAT & CAN BUS | Dual, 17/18-bit |
| X6-8 | 8 | 48 V | 8 | 4.5 | CAN BUS | Dual, 17/18-bit |
| X6-60 | 19.612 | 48 V | 60 | 20 | EtherCAT & CAN BUS | Dual, 17/17-bit |
| X8-32 | 9 | 24 V | 32 | 8 | RS485 | Single, 18-bit |
| X8-120 | 19.612 | 48 V | 120 | 43 | EtherCAT & CAN BUS | Dual, 17/17-bit |
| X12-320 | 20 | 48 V | 320 | 85 | EtherCAT & CAN BUS | Dual, 17/17-bit |
| X15-450 | 20.25 | 72 V | 450 | 145 | EtherCAT & CAN BUS | Dual, 17/17-bit |

## Content Notes

This conversion is derived from `pdftotext -layout` output of a 10-page marketing/spec brochure with a jumbled multi-column layout. The following was intentionally dropped as non-technical fluff:
- Repeated page footers ("Suzhou Micro Actuator Technology Co., Ltd." with page numbers) and website URLs (`www.myactuator.cn`, `www.myactuator.com`) repeated on every page
- The marketing disclaimer "The pictures of this series of products are subject to the actual products."
- Stray single-letter/page-artifact tokens (e.g. a lone "P" on the first line) from OCR noise

All technical content was kept: the naming-convention breakdown, per-model actuator full names, full parameter tables, stall-torque-vs-time tables, and communication/encoder/brake configuration for every model (X2-7, X4-10, X4-36, X6-8, X6-60, X8-32, X8-120, X12-320, X15-450). Installation-drawing dimensions were reconstructed on a best-effort basis where the OCR text preserved enough structure to associate numbers with bolt patterns/diameters; these are explicitly flagged as approximate and should be cross-checked against the original PDF before use in mechanical design. Two small untranslated Chinese CAD annotations were paraphrased in English where encountered (hollow bore hole, shaded-area depth callout, via-hole).
