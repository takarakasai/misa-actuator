# RobStride EduLite 05 (EL05) — Instruction Manual

**Source:** EL05 User Manual (English), RobStride / Beijing Lingfoot (Lingzu) Times Technology Co., LTD.

---

## Table of Contents

- [Precautions](#precautions)
- [Legal Statement](#legal-statement)
- [After-sales Policy](#after-sales-policy)
- [Motor Specification](#motor-specification)
  - [Outline and Mounting Dimensions](#outline-and-mounting-dimensions)
  - [Standard Service Condition](#standard-service-condition)
  - [Electrical Characteristic](#electrical-characteristic)
  - [Mechanical Characteristic](#mechanical-characteristic)
- [Driver Product Information](#driver-product-information)
  - [Driver Product Specifications](#driver-product-specifications)
  - [Driver Interface Definition](#driver-interface-definition)
  - [Main Devices and Specifications](#main-devices-and-specifications)
- [Upper Computer Instructions](#upper-computer-instructions)
  - [Hardware Disposition](#hardware-disposition)
  - [Upper Computer Interface and Description](#upper-computer-interface-and-description)
  - [Motor Settings](#motor-settings)
  - [Parameter Settings](#parameter-settings)
  - [Oscilloscope](#oscilloscope)
- [Upper-Computer Parameter Table](#upper-computer-parameter-table)
- [Communication Box Instruction Example (AT/USB-CAN Wrapper)](#communication-box-instruction-example-atusb-can-wrapper)
- [CAN Communication Failure Protection](#can-communication-failure-protection)
- [Motor Fault Instructions](#motor-fault-instructions)
- [Control Demo](#control-demo)
- [Driver Protocol and Instructions](#driver-protocol-and-instructions)
  - [Description of the Communication Protocol Type](#description-of-the-communication-protocol-type)
- [Read and Write a Single Parameter List](#read-and-write-a-single-parameter-list)
- [Motor Function Description](#motor-function-description)
- [Notes for Implementation](#notes-for-implementation)
- [Control Mode Instructions](#control-mode-instructions)
  - [Program Sample](#program-sample)
  - [Operation Control Mode](#operation-control-mode)
  - [Current Mode](#current-mode-1)
  - [Velocity Mode](#velocity-mode-1)
  - [Location Mode (CSP)](#location-mode-csp)
  - [Location Mode (PP)](#location-mode-pp)
  - [Stop Running](#stop-running)
- [Explanation of CANopen Communication Protocol Types](#explanation-of-canopen-communication-protocol-types)
- [MIT Communication Protocol Description](#mit-communication-protocol-description)

---

## Precautions

1. Please use according to the working parameters specified in this article, otherwise it may cause serious damage to the product!
2. Do not switch the control mode when the joint is running. If you need to switch, send the command to stop the operation before switching.
3. Check whether the parts are in good condition before use. If the parts are missing or damaged, contact technical support in time.
4. Do not disassemble the motor at will, so as to avoid unrecoverable failure.
5. Ensure that there is no short circuit when the motor is connected, and the interface is correctly connected as required.

## Legal Statement

Before using this product, please read this manual carefully and operate the product according to the contents of this manual. If the user violates the contents of this manual to use this product, resulting in any property damage or personal injury accident, the company does not assume any responsibility. Because this product is composed of many parts, do not allow children to touch this product to avoid accidents. In order to prolong the service life of the product, do not use this product in high temperature and high pressure environment. This manual has been printed to the extent possible to include a description of the functions and instructions for use. However, due to the continuous improvement of product functions, design changes, etc., there may still be discrepancies with the products purchased by users.

The color and appearance of this manual may differ from the actual product. Please refer to the actual product. This manual is published by Beijing Lingfoot Times Technology Co., LTD. (hereinafter referred to as Lingfoot), and Lingfoot may at any time make necessary improvements and changes to the inaccurate and up-to-date information in this manual, or make improvements to procedures and/or equipment. Such changes will be uploaded to the company's official website in electronic format. Details can be found in the download center (www.robstride.com). All images are for reference only. Please refer to actual objects.

## After-sales Policy

The after-sales service of this product is implemented in strict accordance with the Law of the People's Republic of China on the Protection of Consumer Rights and Interests and the Product Quality Law of the People's Republic of China. The service content is as follows:

**1. Warranty period and contents**

a. Users who place orders on the online channel to purchase this product can enjoy the return service without reason within seven days from the day after signing. When returning goods, the user must present a valid proof of purchase and return the invoice. The user must ensure that the returned goods maintain the original quality and function, the appearance is intact, the trademarks and various logos of the goods themselves and accessories are complete, and if there are gifts, they should be returned together. If the goods are artificially damaged, artificially disassembled, missing packaging boxes, or missing parts and accessories, they will not be returned. The logistics cost incurred during the return shall be borne by the user (see "After-sales Service Fee Standard"). If the user does not settle the logistics cost, it will be deducted from the refund amount according to the actual amount incurred. Refund the amount paid to the user within seven days from the date of receipt of the returned item. Refund method is the same as payment method. The specific arrival date may be affected by factors such as banks and payment institutions.

b. The warranty period of this product is 1 year.

c. Within 7 days after the user signs for the next day, if a non-human-damage performance failure occurs, through the Lingzu after-sales service center test and confirmation, the user can process a return; the user must present a valid purchase voucher and return the invoice. Any freebies should be returned.

d. From 7 days to 15 days after the user signs for the next day, if a non-human-damage performance failure occurs, through the Lingfoot after-sales service center test and confirmation, the user can have the whole set of goods replaced. After the replacement, the warranty period of the goods themselves is recalculated.

e. From 15 days to 365 days after the user signs for the next day, after inspection and confirmation of the Lingfoot after-sales service center that it is a quality fault of the product itself, free maintenance services can be provided. The replacement faulty product is owned by Lingzu Company. If the product is not faulty, it will be returned as is. This product has been strictly tested after the factory; if there is a quality fault other than in the product itself, we reserve the right to refuse the user's return demand.

**2. Non-warranty regulations.** The following circumstances are not covered by the warranty:

3. Exceeding the warranty period specified in the warranty terms.
4. Failure to follow the instructions, resulting in product damage caused by wrong use.
5. Damage caused by improper operation, maintenance, installation, modification, testing, or other improper use.
6. Non-quality failure caused by conventional mechanical loss or wear.
7. Damage caused by abnormal working conditions, including but not limited to falling, impact, liquid immersion, violent impact, etc.
8. Damage caused by natural disasters (such as floods, fires, lightning strikes, earthquakes, etc.) or force majeure.
9. Damage caused by exceeding peak torque.
10. Damage caused by exceeding peak torque. *(sic — repeated verbatim in the source)*
11. Failure or damage caused by other non-product design, technology, manufacturing, quality, or other problems.

In the case of the above situations, the user must pay the cost.

---

## Motor Specification

### Outline and Mounting Dimensions

The EL05 is a round, flanged articulated (joint) motor module. Key mechanical dimensions from the drawing:

- Front flange bolt circle: 8× M3 (depth 6) equally spaced, and 6× M4 (depth 3) equally spaced, at 120° spacing
- Output flange outer diameters: Ø41.5 / Ø24 / Ø17.7 ±0.03
- Rear housing: Ø38.5 bolt circle with 4× M3 (depth 8) at 37°/37° spacing, 3× Ø4 through-holes
- Body diameter: Ø46, body length: 44 mm
- Connectors: CN1, CN2 on the side of the housing

**Note:** When fixing, the screw depth should not exceed the depth of the casing thread.

### Standard Service Condition

1. Rated voltage: 48 VDC
2. Operating voltage range: 15 V – 60 VDC
3. Rated load (CW): 1.8 N·m
4. Operation direction: CW/CCW from the direction of the exit shaft
5. Use posture: the direction of the exit axis is horizontal or vertical
6. Standard operating temperature: 25 ± 5 °C
7. Operating temperature range: -20 ~ 50 °C
8. Standard operating humidity: 65%
9. Humidity range: 5 ~ 85%, no condensation
10. Storage temperature range: -30 ~ 70 °C
11. Insulation Class: Class B

### Electrical Characteristic

1. No load speed: 430 rpm ± 10%
2. No-load current: 0.14 Arms ± 10%
3. Rated load: 1.8 N·m
4. Rated load speed: 100 rpm ± 10%
5. Rated load phase current (peak): 2.6 Apk ± 10%
6. Peak load: 6 N·m
7. Maximum load phase current (peak): 11 Apk ± 10%
8. Insulation resistance/stator winding: DC 500 VAC, 100 MΩ
9. High voltage/stator and housing: 600 VAC, 1 s, 2 mA
10. Motor back potential: 7.4 Vrms/kRPM ± 10%
11. Torque constant: 0.94 N·m/Arms
12. **T-N curve (48 V):** torque falls roughly linearly from about 5.6 N·m near-zero speed down to 0 N·m at ~430 rpm (the no-load speed), with a knee around 220–250 rpm / 4 N·m.
13. **Maximum overload curve.** Test conditions: ambient temperature 25 °C; winding limit temperature 135 °C (this is the constraint/protection temperature — the actual insulation limit is 180 °C); test speed: 100 rpm.

| Load (N·m) | Operating time (s) |
|---|---|
| 6 | 5 |
| 5 | 7 |
| 4 | 14 |
| 3 | 44 |
| 2 | 300 |
| 1.8 | rated (continuous) |

### Mechanical Characteristic

1. Weight: 242 g ± 3 g
2. Number of poles: 20
3. Phase number: 3 phases
4. Drive mode: FOC
5. Deceleration ratio: 9:1

---

## Driver Product Information

### Driver Product Specifications

| Project | Data |
|---|---|
| The rated working voltage | 48 VDC |
| The maximum allowable voltage | 60 VDC |
| Rated working phase current | 2.6 Apk |
| Maximum allowable phase current | 11.0 Apk |
| Standby power | ≤ 18 mA |
| CAN bus bit rate | 1 Mbps |
| Dimensions | Φ41 mm |
| Working environment temperature | -20 °C ~ 50 °C |
| The maximum allowable temperature of the control board | 105 °C |
| Encoder resolution | 14 bit (absolute, single-turn) |

### Driver Interface Definition

**Recommended driver interface brand and model:**

| Board-end model | Brand/manufacturer | Line-end model | Brand/manufacturer |
|---|---|---|---|
| XT30PB(2+2)-M.G.B | AMASS (Ams) | XT30(2+2)-F.G.B | AMASS (Ams) |

### Main Devices and Specifications

| No. | Item | Specifications | Quantity |
|---|---|---|---|
| 1 | MCU chip | GD32F303RET6 | 1 PCS |
| 2 | Driver chip | EG2124 | 1 PCS |
| 3 | Magnetic encoder chip | VCE2755Q | 2 PCS |
| 4 | Thermistor | LTS00-104J395T19E010 / NCP18XH103F03RB | 2 PCS |
| 5 | Power MOS | SFS10R20GCF | 6 PCS |

---

## Upper Computer Instructions

Please go to the www.robstride.com website download center for the upper-computer software ("motorstudio").

### Hardware Disposition

The articulated motor uses the CAN communication mode and has two communication cables. It is connected to the debugger through the CAN-to-USB tool. The debugger needs to be installed with the ch340 driver in advance and works in AT mode by default.

It should be noted that this setup is based on a specific CAN-to-USB tool developed for the debugger, so the recommended serial port tool must be used to debug the debugger. If you want to port this to another debugger platform, refer to the development chapter of the instructions.

The CAN-to-USB tool is recommended to be the official USB-CAN module from Lingzu Times. The frame header of the corresponding serial port protocol is `41 54` ("AT"), and the frame tail is `0D 0A` (`\r\n`).

When using the CAN-to-USB module, pay attention to the DIP switch settings on the module:
- DIP switch 1 ON: the module enters Boot mode and cannot establish a connection with the host computer.
- DIP switch 2 ON: a 120 Ω terminating resistor is connected to the module port, allowing normal communication with the host computer.

### Upper Computer Interface and Description

The "motorstudio" desktop application interface is organized into:

**A. Motor Connection Module**
- Refreshing the Serial Port
- Opening the Serial Port
- Testing the Device

**B. Motor Configuration Module**
- Starting the Upgrade
- Opening a File
- Modifying the Motor CAN ID
- Setting the Motor's Mechanical Zero Position

**C. Motor Upgrade Module**
- Magnetic Encoder Calibration
- Motor Active Reporting Switch
- Setting the Motor Active Reporting Time
- Modifying the Motor CAN ID
- Setting the Motor's Mechanical Zero Position

**D. Motor Main Interface**
- Parameter Settings
- Motor Oscilloscope

**E. Run and Debug Area**
- Parameter Debugging Buttons
- Motor Mode Configuration and Parameter Modification
- Sine Signal Testing

### Motor Settings

**Motor connection settings:** Connect the CAN-to-USB tool (install the ch340 driver, which works in AT mode by default), click **Refresh Serial Port**, open the serial port, and click **Detect Device** to detect the corresponding motor. The green text below shows the motor type.

**Motor configuration module:**

1. Recalibrate the motor magnetic encoder. Reinstalling the motor board and motor, or reconnecting the motor's three-phase wiring, requires recalibrating the magnetic encoder.
2. Enable active motor reporting. Click **Start Reporting** to enable active motor reporting (communication type 2 frames). You can set the interval below, with a minimum of 10 ms.
3. Set ID: Set the motor's CAN ID.
4. Set Zero Position: Set the current position to 0.

**Motor upgrade module:**

1. Click to open the file and select the firmware to upgrade. The `rs-0x` in the firmware name identifies the selected motor type.
2. Click **Start Upgrade**, and the motor will enter the upgrade preparation stage.
3. When the green text "Device has entered upgrade mode" pops up, click to start the upgrade.
4. When the green text "Upgrade Successfully" pops up, the upgrade is complete.

If the green progress bar gets stuck halfway through the upgrade, you can click to stop the upgrade, or re-power on and re-enter the upgrade process. The internal program of the motor is not lost if the upgrade fails. Please check whether the communication environment is good before upgrading again.

### Parameter Settings

After successfully connecting to the motor:

1. Click **Refresh Parameter Table**. "Updated Parameter Table Successfully" appears at the top, indicating that the motor parameters have been read successfully. (Note: the parameter table can only be refreshed while the motor is in standby mode; if the motor is running, the parameter table refresh cannot be performed.) The interface displays the motor's parameters. Parameters shown in blue are stored internally in the motor and can be modified in the Current Value field next to the parameter.
2. Click **Read Parameters** to upload the motor parameters to the debugger. Parameters in light blue are observed parameters, which are collected and can be observed in real time.
3. Click **Write Parameters** to download the debugger parameters to the motor.
4. Click **Restore Factory** to restore the motor's default parameters for the latest firmware.
5. Click **Export** to export the current motor parameters from the parameter table.
6. Click **Open Multi-Device Connection** to connect the host computer to multiple motors. Because the parameter interfaces for different motor types vary, multi-device connection is only used for upgrades; after upgrading and debugging the motor, close multi-device connection and search for the motor again.

**Note:** Please do not change the torque limit, protection temperature, and overtemperature time of the motor. The company will not bear any legal responsibility for damage to human body or irreversible damage to joints caused by illegal operation of this product.

### Oscilloscope

The interface supports viewing and observing the graph generated by real-time data, including motor Id/Iq current, temperature, real-time speed at the output end, rotor (encoder) position, output end position, etc.

Click the oscilloscope module in the analysis module, select the appropriate parameters for each channel (parameter meaning can be found in the parameter table), set the output frequency, click **start** to observe the data graph, and stop the plot to stop the observation graph.

---

## Upper-Computer Parameter Table

This is the full parameter table exposed by the "motorstudio" host software (read via the AT/USB-CAN wrapper protocol described below, and internally via communication types 17/18/22 against these same function codes). It is a different, larger address space from the [CAN type-17/18 direct parameter index table](#read-and-write-a-single-parameter-list) used for raw CAN-frame access.

### Device Identity / Version

| Function code | Name | Type | Attribute | Current value (for reference) | Notes |
|---|---|---|---|---|---|
| 0X0000 | Name | String | Read/Write | *(device name string)* | |
| 0X0001 | BarCode | String | Read/Write | *(barcode string)* | |
| 0X1000 | BootCodeVersion | String | Read only | 0.1.5 | |
| 0X1001 | BootBuildDate | String | Read only | Mar 16 2022 | |
| 0X1002 | BootBuildTime | String | Read only | 20:22:09 | |
| 0X1003 | AppCodeVersion | String | Read only | 0.0.0.1 | Motor program version number |
| 0X1004 | AppGitVersion | String | Read only | 7b844b0fM | |
| 0X1005 | AppBuildDate | String | Read only | Apr 14 2022 | |
| 0X1006 | AppBuildTime | String | Read only | 20:30:22 | |
| 0X1007 | AppCodeName | String | Read only | Lingzu_motor | |

### Read/Write Configuration Parameters (0x2000 range)

| Function code | Name | Type | Attribute | Max | Min | Current value (ref.) | Notes |
|---|---|---|---|---|---|---|---|
| 0X2000 | echoPara1 | uint16 | disposition | 74 | 5 | 5 | |
| 0X2001 | echoPara2 | uint16 | disposition | 74 | 5 | 5 | |
| 0X2002 | echoPara3 | uint16 | disposition | 74 | 5 | 5 | |
| 0X2003 | echoPara4 | uint16 | disposition | 74 | 5 | 5 | |
| 0X2004 | echoFreHz | uint32 | Read/Write | 10000 | 1 | 500 | |
| 0X2005 | MechOffset | float | Settings | 7 | -7 | 4.619583 | Motor magnetic encoder angle offset |
| 0X2006 | status4 | float | Read/Write | 50 | -50 | 4.52 | Reserved parameter |
| 0X2007 | limit_torque | float | Read/Write | 17 | 0 | 17 | Torque limitation |
| 0X2008 | I_FW_MAX | float | Read/Write | 33 | 0 | 0 | Weak magnetic current value, default 0 |
| 0X2009 | motor_baud | uint8 | Settings | 20 | 0 | 1 | Baud rate flag bit |
| 0X200a | CAN_ID | uint8 | Settings | 127 | 0 | 1 | ID of this object |
| 0X200b | CAN_MASTER | uint8 | Settings | 127 | 0 | 0 | CAN host ID |
| 0X200c | CAN_TIMEOUT | uint32 | Read/Write | 100000 | 0 | 0 | CAN timeout threshold; default value is 0 |
| 0X200d | status2 | int16 | Read/Write | 1500 | 0 | 800 | Reserved parameter |
| 0X200e | status3 | uint32 | Read/Write | 1000000 | 1000 | 20000 | Reserved parameter |
| 0X200f | status1 | float | Read/Write | 64 | 1 | 7.75 | Reserved parameter |
| 0X2010 | Status6 | uint8 | Read/Write | 1 | 0 | 1 | Reserved parameter |
| 0X2011 | cur_filt_gain | float | Read/Write | 1 | 0 | 0.9 | Current filtering parameter |
| 0X2012 | cur_kp | float | Read/Write | 200 | 0 | 0.025 | Current kp |
| 0X2013 | cur_ki | float | Read/Write | 200 | 0 | 0.0258 | Current ki |
| 0X2014 | spd_kp | float | Read/Write | 200 | 0 | 2 | Velocity kp |
| 0X2015 | spd_ki | float | Read/Write | 200 | 0 | 0.021 | Speed ki |
| 0X2016 | loc_kp | float | Read/Write | 200 | 0 | 30 | Position kp |
| 0X2017 | spd_filt_gain | float | Read/Write | 1 | 0 | 0.1 | Velocity filter parameter |
| 0X2018 | limit_spd | float | Read/Write | 200 | 0 | 2 | Location mode speed limit |
| 0X2019 | limit_cur | float | Read/Write | 23 | 0 | 23 | Position/Velocity mode current limit |
| 0X201a | loc_ref_filt_gain | float | Read/Write | 100 | 0 | 0 | Reserved parameter |
| 0X201b | add_elec_offset | float | Read/Write | 100 | 0 | 0 | Reserved parameter |
| 0X201c | position_offset | float | Read/Write | 27 | 0 | 0 | High-speed segment offset |
| 0X201d | chasu_angle_offset | float | Read/Write | 27 | 0 | 0 | Low-end offset |
| 0X201e | spd_step_value | float | Read/Write | 150 | 0 | | Velocity-mode acceleration |
| 0X201f | vel_max | float | Read/Write | 20 | 0 | | PP mode speed |
| 0X2020 | acc_set | float | Read/Write | 1000 | 0 | | PP mode acceleration |
| 0X2021 | zero_sta | float | Read/Write | 100 | 0 | 0 | Zero marker |
| 0x2022 | protocol_1 | uint8 | Read/Write | | | 0 | Protocol flag |
| 0x2023 | add_offset | float | Read/Write | 7 | -7 | 0 | Bias compensation |

### Read-Only Telemetry / Status Parameters (0x3000 range)

| Function code | Name | Type | Current value (ref.) | Notes |
|---|---|---|---|---|
| 0X3000 | timeUse0 | uint16 | 5 | |
| 0X3001 | timeUse1 | uint16 | 0 | |
| 0X3002 | timeUse2 | uint16 | 10 | |
| 0X3003 | timeUse3 | uint16 | 0 | |
| 0X3004 | encoderRaw | int16 | 11396 | Magnetic encoder sampling value |
| 0X3005 | mcuTemp | int16 | 337 | MCU internal temperature, ×10 |
| 0X3006 | motorTemp | int16 | 333 | Motor NTC temperature, ×10 |
| 0X3007 | vBus(mv) | uint16 | 24195 | Bus voltage |
| 0X3008 | adc1Offset | int32 | 2084 | ADC sampling channel 1 zero-current bias |
| 0X3009 | adc2Offset | int32 | 2084 | ADC sampling channel 2 zero-current bias |
| 0X300a | adc1Raw | uint16 | 1232 | ADC sampling value 1 |
| 0X300b | adc2Raw | uint16 | 1212 | ADC sampling value 2 |
| 0X300c | VBUS | float | 36 | Bus voltage, V |
| 0X300d | cmdId | float | 0 | Id ring instruction, A |
| 0X300e | cmdIq | float | 0 | Iq ring command, A |
| 0X300f | cmdlocref | float | 0 | Position loop command, rad |
| 0X3010 | cmdspdref | float | 0 | Speed loop command, rad/s |
| 0X3011 | cmdTorque | float | 0 | Torque instruction, N·m |
| 0X3012 | cmdPos | float | 0 | MIT protocol angle instruction |
| 0X3013 | cmdVel | float | 0 | MIT protocol speed instruction |
| 0X3014 | rotation | int16 | 1 | Number of turns |
| 0X3015 | modPos | float | 4.363409 | Motor uncounted coil mechanical angle, rad |
| 0X3016 | mechPos | float | 0.777679 | Load end loop mechanical angle, rad |
| 0X3017 | mechVel | float | 0.036618 | Load speed, rad/s |
| 0X3018 | elecPos | float | 4.714761 | Electrical angle |
| 0X3019 | ia | float | 0 | U-wire current, A |
| 0X301a | ib | float | 0 | V-wire current, A |
| 0X301b | ic | float | 0 | W-wire current, A |
| 0X301c | timeout | uint32 | 31600 | Timeout counter value |
| 0X301d | phaseOrder | uint8 | 0 | Directional marking |
| 0X301e | iqf | float | 0 | iq filter value, A |
| 0X301f | boardTemp | int16 | 359 | Plate temperature, ×10 |
| 0X3020 | iq | float | 0 | iq original value, A |
| 0X3021 | id | float | 0 | id original value, A |
| 0X3022 | faultSta | uint32 | 0 | Fault status value — see [Motor Fault Instructions](#motor-fault-instructions) |
| 0X3023 | warnSta | uint32 | 0 | Warning status value |
| 0X3024 | drv_fault | uint16 | 0 | Driver chip fault value 1 — see [Table 11](#motor-fault-instructions) |
| 0X3025 | drv_temp | int16 | 48 | Driver chip fault value 2 — see [Table 12](#motor-fault-instructions) |
| 0X3026 | Uq | float | 0 | Q-axis voltage |
| 0X3027 | Ud | float | 0 | D-axis voltage |
| 0X3028 | dtc_u | float | 0 | Duty cycle of the U-phase output |
| 0X3029 | dtc_v | float | 0 | Duty cycle of the V-phase output |
| 0X302a | dtc_w | float | 0 | Duty cycle of the W-phase output |
| 0X302b | v_bus | float | 24.195 | Vbus in the closed loop |
| 0X302c | torque_fdb | float | 0 | Torque feedback value, N·m |
| 0X302d | rated_i | float | 8 | Rated current of motor |
| 0X302e | limit_i | float | 27 | Motor limits the maximum current |
| 0X302f | spd_ref | float | 0 | Motor speed expectation |
| 0X3030 | spd_reff | float | 0 | Motor speed expectation 2 |
| 0X3031 | zero_fault | float | 0 | Motor position determination parameter |
| 0X3032 | chasu_coder_raw | float | 0 | Motor position determination parameter |
| 0X3033 | chasu_angle | float | 0 | Motor position determination parameter |
| 0X3034 | as_angle | float | 0 | Motor position determination parameter |
| 0X3035 | cs_angle | float | 0 | Motor position determination parameter |
| 0X3036 | chasu_age_out | float | 0 | Motor position determination parameter |
| 0X3037 | motormechpositio | float | 0 | Motor position determination parameter |
| 0X3038 | fault1 | uint32 | 0 | Log failure |
| 0X3039 | fault2 | uint32 | 0 | Log failure |
| 0X303a | fault3 | uint32 | 0 | Log failure |
| 0X303b | fault4 | uint32 | 0 | Log failure |
| 0X303c | fault5 | uint32 | 0 | Log failure |
| 0X303d | fault6 | uint32 | 0 | Log failure |
| 0X303e | fault7 | uint32 | 0 | Log failure |
| 0X303f | fault8 | uint32 | 0 | Log failure |
| 0X3040 | ElecOffset | float | 0 | Electrical angle offset |
| 0X3041 | mcOverTemp | int16 | 0 | Overtemperature threshold |
| 0X3042 | Kt_Nm/Amp | float | 0 | Moment (torque) coefficient |
| 0X3043 | Tqcali_Type | uint8 | 0 | Motor type |
| 0X3044 | low_position | float | 0 | Motor position determination parameter |
| 0X3045 | theta_mech_1 | float | 0 | Type-2 low-speed angle |

---

## Communication Box Instruction Example (AT/USB-CAN Wrapper)

The debugger's "COM send data" box wraps a raw extended CAN frame inside an AT-command envelope. Example:

```
41 54 90 07 e8 0c 08 05 70 00 00 01 00 00 00 0d 0a
```

| `41 54` | `90 07 e8 0c` | `08` | `05 70 00 00 01 00 00 00` | `0d 0a` |
|---|---|---|---|---|
| frame header ("AT") | extended-frame CAN ID (raw, pre-shift) | number of data bytes (DLC) | data frame (8 bytes) | frame tail (`\r\n`) |

*(The source manual's table labels this row as "frame header / Number of data bits / extended frame / data frame / frame tail" — the middle two labels appear swapped relative to their natural meaning; the mapping above reflects the actual byte semantics used in the worked example below.)*

The translation of the extended-frame CAN ID field into the real CAN ID requires the following transformation:

`90 07 e8 0c` converts to binary as `1001 0000 0000 0111 1110 1000 0000 1100`. Remove the low 3 bits (the trailing `100`), giving `1 0010 0000 0000 1111 1101 0000 0001`. Converting this to hexadecimal gives `12 00 FD 01`. According to the communication protocol, this decodes as:

| `12` (hex) | `00` | `FD` | `01` |
|---|---|---|---|
| Communication type 18 (0x12, decimal) | *(no meaning)* | host ID | motor CAN ID |

---

## CAN Communication Failure Protection

When the value of `CAN_TIMEOUT` is 0, this function is disabled.

When the `CAN_TIMEOUT` value is non-zero, if the motor does not receive a CAN command within that period of time, the motor enters reset mode. A value of 20000 corresponds to 1 s.

---

## Motor Fault Instructions

Function code `0x3022` (`faultSta`) indicates the fault code, where:

- **bit14:** motor blocking/overload algorithm protection
- **bit7:** encoder uncalibrated — motor uncalibrated encoder
- **bit3:** overvoltage fault — the motor voltage exceeds the protection voltage of 60 V
- **bit2:** undervoltage fault — the motor voltage is lower than the protection voltage of 12 V
- **bit1:** driver chip failure — motor driver chip failure reported
- **bit0:** motor overtemperature fault — motor thermistor temperature exceeds 145 °C

Function code `0x3024` (`drv_fault`) is driver chip fault code 1:

**Table 11. Fault Status Register 1 Field Descriptions**

| Bit | Field | Type | Default | Description |
|---|---|---|---|---|
| 10 | FAULT | R | 0b | Logic OR of FAULT status registers. Mirrors nFAULT pin. |
| 9 | VDS_OCP | R | 0b | Indicates VDS monitor overcurrent fault condition |
| 8 | GDF | R | 0b | Indicates gate drive fault condition |
| 7 | UVLO | R | 0b | Indicates undervoltage lockout fault condition |
| 6 | OTSD | R | 0b | Indicates overtemperature shutdown |
| 5 | VDS_HA | R | 0b | Indicates VDS overcurrent fault on the A high-side MOSFET |
| 4 | VDS_LA | R | 0b | Indicates VDS overcurrent fault on the A low-side MOSFET |
| 3 | VDS_HB | R | 0b | Indicates VDS overcurrent fault on the B high-side MOSFET |
| 2 | VDS_LB | R | 0b | Indicates VDS overcurrent fault on the B low-side MOSFET |
| 1 | VDS_HC | R | 0b | Indicates VDS overcurrent fault on the C high-side MOSFET |
| 0 | VDS_LC | R | 0b | Indicates VDS overcurrent fault on the C low-side MOSFET |

Function code `0x3025` (`drv_temp`) is driver chip fault code 2:

**Table 12. Fault Status Register 2 Field Descriptions**

| Bit | Field | Type | Default | Description |
|---|---|---|---|---|
| 10 | SA_OC | R | 0b | Indicates overcurrent on phase A sense amplifier (DRV8353xS) |
| 9 | SB_OC | R | 0b | Indicates overcurrent on phase B sense amplifier (DRV8353xS) |
| 8 | SC_OC | R | 0b | Indicates overcurrent on phase C sense amplifier (DRV8353xS) |
| 7 | OTW | R | 0b | Indicates overtemperature warning |
| 6 | GDUV | R | 0b | Indicates VCP charge pump and/or VGLS undervoltage fault condition |
| 5 | VGS_HA | R | 0b | Indicates gate drive fault on the A high-side MOSFET |
| 4 | VGS_LA | R | 0b | Indicates gate drive fault on the A low-side MOSFET |
| 3 | VGS_HB | R | 0b | Indicates gate drive fault on the B high-side MOSFET |
| 2 | VGS_LB | R | 0b | Indicates gate drive fault on the B low-side MOSFET |
| 1 | VGS_HC | R | 0b | Indicates gate drive fault on the C high-side MOSFET |
| 0 | VGS_LC | R | 0b | Indicates gate drive fault on the C low-side MOSFET |

---

## Control Demo

The "motorstudio" host software's Run/Debug area lets you exercise each control mode directly from the desktop UI (independent of writing your own CAN driver).

**Jog Run:** Click **JOG+ / JOG-** to run the motor forward and reverse at a speed of 1 rad/s.

**Control Mode Switching:** Select the desired control mode in the command box to the right of the run mode selector. Do not switch control modes while the joint is running — stop it first.

**Operation and control mode (MIT-style, 5-parameter):**
1. Switch the control mode to operation mode.
2. The motor starts running and enters `motor_mode`.
3. Set five parameter values and click **Start** or **Send** continuously. The motor returns feedback frames and runs according to the target command.
4. Click **Stop** to stop the motor and terminate the continuous sending of commands.

**Current mode:**
1. Switch the control mode to current mode.
2. The motor starts running and enters `motor_mode`.
3. Set the current command value for Iq Command (A). Click the **>>** button on the right. The motor follows the current command.
4. Click **Stop** to stop the motor.

**Motor Current Sine Test:** Switch to current mode, enter `motor_mode`, set amplitude and frequency, click OK, then click Start — the target command is then planned sinusoidally. Click Stop to stop.

**Speed Mode:**
1. Switch the control mode to speed mode.
2. The motor starts running and enters `motor_mode`.
3. First set the current limit (maximum phase current) and speed step value (motor acceleration); if not set, the motor uses default values. Finally set the speed command (target speed) — the motor follows it.
4. Click **Stop** to stop the motor.

**Motor Speed Sine Test:** Same pattern as above — set amplitude/frequency, click OK, then Start.

**Position Mode (PP):**
1. Switch the control mode to interpolation position mode (PP).
2. Start the motor, entering `motor_mode`.
3. First set speed and acceleration (or use defaults). Finally set the position command (target position) — the motor follows it.
4. Set the speed to 0 to stop the motor at the current position; re-issue speed and position to continue.
5. Click **Stop** to stop the motor.

**Motor Position Sine Test (PP):** Same sine-test pattern as above.

**Position Mode (CSP):**
1. Switch the control mode to position mode (CSP).
2. The motor starts running and enters `motor_mode`.
3. Set the speed first (or use the default). Finally set the position command (target position) — the motor follows it.
4. Click **Stop** to stop the motor.

**Motor Position Sine Test (CSP):** Same sine-test pattern as above.

---

## Driver Protocol and Instructions

The motor communication uses the CAN 2.0 communication interface, the baud rate is 1 Mbps, and the extended frame format is adopted as follows:

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| **Size** | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| **Description** | Communication type | data area 2 | Destination address | data area 1 |

The control modes supported by the motor include:

- **Operation control mode:** set 5 parameters of motor operation control;
- **Current mode:** the specified Iq current of the given motor;
- **Velocity mode:** the specified running speed of the given motor;
- **Position mode:** given the specified position of the motor, the motor will run to the specified position.

### Description of the Communication Protocol Type

#### Communication type 0: Get device ID

Gets the device's ID and 64-bit MCU unique identifier.

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x0 | bit15~8: identifies host CAN_ID | target motor CAN_ID | 0 |

**Reply frame:**

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x0 | target motor CAN_ID | 0xFE | 64-bit MCU unique identifier |

#### Communication Type 1: operation control mode motor control instruction

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x1 | Byte2: Torque (0~65535) corresponds to (-6 N·m ~ 6 N·m)¹ | target motor CAN_ID | Byte0~1: target angle [0~65535] corresponds to (-4π~4π); Byte2~3: target angular velocity [0~65535] corresponds to (-50 rad/s~50 rad/s); Byte4~5: Kp [0~65535] corresponds to (0.0~500.0); Byte6~7: Kd [0~65535] corresponds to (0.0~5.0). After conversion, the high byte is first and the low byte is second. |

¹ *The frame table states a torque range of -6 N·m ~ 6 N·m here (and in the equivalent MIT-protocol fields later in this document), while the firmware sample code's `T_MIN`/`T_MAX` macros define ±5.5 N·m. This is a genuine discrepancy in the source manual — verify the actual torque scaling against firmware/device behavior before relying on it.*

Response frame: Response motor feedback frame (see communication type 2).

#### Communication Type 2: motor feedback data

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x2 | Bit8~Bit15: CAN ID of the current motor. Bit21~16: fault information (0 = none, 1 = has). Bit22~23: mode status. | host CAN_ID | Byte0~1: current angle [0~65535] corresponding to (-4π~4π); Byte2~3: current angular velocity [0~65535] corresponds to (-50 rad/s~50 rad/s); Byte4~5: current torque [0~65535] corresponds to (-6 N·m~6 N·m); Byte6~7: current temperature: Temp(°C)×10. If the value is higher than 10, the high byte is first and the low byte is last. |

Fault/mode bit assignments (within bit21~16 / bit22~23 of the ID field):

- bit21: uncalibrated
- bit20: uncalibrated / gridlock overload fault
- bit19: magnetic coding fault
- bit18: overtemperature
- bit17: overcurrent
- bit16: undervoltage fault
- bit22~23 (mode status): `0` = Reset mode [reset]; `1` = Cali mode [calibration]; `2` = Motor mode [Run]

#### Communication Type 3: Motor enabled to run

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x3 | bit15~8: identifies the main CAN_ID | target motor CAN_ID | *(unused)* |

Response frame: Response motor feedback frame (see communication type 2).

#### Communication Type 4: Motor stops running

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x4 | bit15~8: identifies the main CAN_ID | target motor CAN_ID | When the motor is running normally, 0 must be cleared in the data field. Byte[0]=1: the fault is cleared. |

Response frame: Response motor feedback frame (see communication type 2).

#### Communication type 6: Set motor mechanical zero

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x6 | bit15~8: identifies the main CAN_ID | target motor CAN_ID | Byte[0]=1 |

Response frame: Response motor feedback frame (see communication type 2).

#### Communication type 7: Set motor CAN_ID

Changes the current motor CAN_ID, effective immediately.

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x7 | bit15~8: used to identify main CAN_ID; bit16~23: preset CAN_ID | Target motor CAN_ID | *(unused)* |

Answer frame: Answer motor broadcast frame (see communication type 0).

#### Communication type 17: Single parameter read

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x11 | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | Byte0~1: index (see the [read/write parameter list](#read-and-write-a-single-parameter-list)); Byte2~3: 00; Byte4~7: reserved (low byte first, high byte second) |

**Reply frame:**

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x11 | bit15~8: indicates the master CAN_ID; bit23~16: 00 indicates the master CAN_ID was successfully read, 01 indicates the master CAN_ID | *(see above)* | Byte0~1: index; Byte2~3: 00; Byte4~7: parameter data (low byte first, high byte second) |

#### Communication type 18: Single parameter write (lost on power failure)

With type 22, parameters starting with function code 0x20 in the upper-computer parameter table can be persisted.

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x12 | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | Byte0~1: index (see the [read/write parameter list](#read-and-write-a-single-parameter-list)); Byte2~3: 00; Byte4~7: parameter data (low byte first, high byte last) |

Response frame: Response motor feedback frame (see communication type 2).

#### Communication type 21: Fault feedback frame

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x15 | bit15~8: motor CAN_ID | identifies the main CAN_ID | Byte0~3: fault value (non-0: faulty; 0: normal); bit14: gridlock overload fault; bit7: encoder not calibrated; bit3: overvoltage fault; bit2: undervoltage fault; bit1: driver chip fault; bit0: motor overtemperature fault (default threshold 135 °C). Byte4~7: warning value; bit0: motor overtemperature warning (default 135 °C). |

#### Communication type 22: Motor data save frame

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x16 | bit15~8: identifies the main CAN_ID | target motor CAN_ID | `01 02 03 04 05 06 07 08` |

Response frame: Response motor feedback frame (see communication type 2).

#### Communication type 23: Motor baud rate modification frame (re-power-on effect)

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x17 | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | `01 02 03 04 05 06 F_CMD` — `F_CMD` is the motor baud rate: `01`=1 M, `02`=500 K, `03`=250 K, `04`=125 K |

Response frame: Response motor feedback frame (see communication type 0).

#### Communication type 24: The motor actively reports frames

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x18 | bit15~8: identifies the main CAN_ID | target motor CAN_ID | `01 02 03 04 05 06 F_CMD` — `F_CMD` is the motor reporting switch: `00` = disable active reporting (default); `01` = enable active reporting (default reporting interval 10 ms) |

**Response frame** (motor feedback, same layout as communication type 2):

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x18 | Bit8~Bit15: CAN ID of the current motor; bit21~16: fault info; bit22~23: mode status (same bit layout as communication type 2) | target motor CAN_ID | Byte0~1: current angle [0~65535] corresponds to (-4π~4π); Byte2~3: current angular velocity [0~65535] corresponds to (-50 rad/s~50 rad/s); Byte4~5: current torque [0~65535] corresponds to (-6 N·m~6 N·m); Byte6~7: current temperature ×10 |

#### Communication type 25: Motor protocol modification frame (re-power-on effect)

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x19 | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | `01 02 03 04 05 06 F_CMD` — `F_CMD` is the motor protocol type: `0` = private protocol (default), `1` = CANopen protocol, `2` = MIT protocol |

Response frame: Response motor feedback frame (see communication type 0).

#### Communication type 26: Version number read frame

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x4 ² | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | Byte[0]=0x00, Byte[1]=0xC4 |

² *The source table literally shows `0x4` in the Bit28~bit24 column for the "communication type 26" request frame; the expected value for type 26 is `0x1A`. This looks like a copy/paste artifact in the original vendor document (the surrounding cell text is boilerplate reused from other command sections). Treat with suspicion and verify against real device traffic.*

**Response frame:**

| Data field | | 29-bit ID | | 8-byte data field |
|---|---|---|---|---|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | *(see note ³)* | *(see note ³)* | target motor CAN_ID | Byte0=0x00; Byte1=0xC4; Byte2=0x56; Byte3~6: motor version number, ordered from high to low |

³ *The source table's ID-field description cells for this response frame are identical to the communication-type-2 (motor feedback) cells (CAN ID / fault bits / mode status), which does not semantically match a version-read response. This appears to be another copy-paste artifact in the original document — the meaningful content is the Byte0~6 version payload; the ID-field text should likely be disregarded or re-verified against real device behavior.*

---

## Read and Write a Single Parameter List

This is the direct parameter index table used by communication types 17 and 18 (distinct from the larger [upper-computer parameter table](#upper-computer-parameter-table) above).

| Index | Name | Description | Type | Bytes | Range | R/W permission |
|---|---|---|---|---|---|---|
| 0X7005 | run_mode | 0: operation mode; 1: position mode (PP); 2: Velocity mode; 3: Operation mode (current mode); 5: Position mode (CSP) | uint8 | 1 | | W/R |
| 0X7006 | iq_ref | Current mode Iq command | float | 4 | -11 to 11 A | W/R |
| 0X700A | spd_ref | Velocity mode rotational speed command | float | 4 | -50 to 50 rad/s | W/R |
| 0X700B | limit_torque | Torque limit | float | 4 | 0 to 6 N·m | W/R |
| 0X7010 | cur_kp | Current-loop Kp | float | 4 | default value is 0.17 | W/R |
| 0X7011 | cur_ki | Current-loop Ki | float | 4 | default value is 0.012 | W/R |
| 0X7014 | cur_filt_gain | Current filter gain | float | 4 | 0 to 1.0, default 0.1 | W/R |
| 0X7016 | loc_ref | Position mode angle instruction | float | 4 | rad | W/R |
| 0X7017 | limit_spd | Position mode (CSP) speed limit | float | 4 | 0 to 50 rad/s | W/R |
| 0X7018 | limit_cur | Velocity/position mode current limit | float | 4 | 0 to 11 A | W/R |
| 0x7019 | mechPos | Mechanical angle of the loading coil | float | 4 | rad | R |
| 0x701A | iqf | Iq filter | float | 4 | -11 to 11 A | R |
| 0x701B | mechVel | Speed of the load | float | 4 | -50 to 50 rad/s | R |
| 0x701C | VBUS | Bus voltage | float | 4 | V | R |
| 0x701E | loc_kp | Position-loop Kp | float | 4 | default value is 40 | W/R |
| 0x701F | spd_kp | Speed-loop Kp | float | 4 | default value is 6 | W/R |
| 0x7020 | spd_ki | Speed-loop Ki | float | 4 | default value is 0.02 | W/R |
| 0x7021 | spd_filt_gain | Speed filter value | float | 4 | default value is 0.1 | W |
| 0x7022 | acc_rad | Velocity-mode acceleration | float | 4 | default value is 20 rad/s² | W |
| 0x7024 | vel_max | Location mode (PP) speed | float | 4 | default value is 10 rad/s | W |
| 0x7025 | acc_set | Location mode (PP) acceleration | float | 4 | default value is 10 rad/s² | W |
| 0x7026 | EPScan_time | Report time; 1 = 10 ms, +1 increment = +5 ms | uint16 | 2 | default value is 1 | W |
| 0x7028 | canTimeout | CAN timeout threshold; 20000 = 1 s | uint32 | 4 | default value is 0 | W |
| 0x7029 | zero_sta | Zero flag bit: 0 means power-on range 0–2π, 1 means power-on range -π–π | uint8 | 1 | default is 0 | W |

### Read Example

Take reading `loc_kp` as an example.

**Read instruction:**

| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
|---|---|---|---|---|
| | 0x11 | 0x00FD | 0x7F | `1E 70 00 00 00 00 00 00` |
| Description | Type 17 | Host id 0xFD | Target motor CAN_ID 7F | Byte0~1: index, corresponding to `loc_kp` |

**Feedback instruction:**

| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
|---|---|---|---|---|
| | 0x11 | 0x007F | 0xFD | `1E 70 00 00 00 00 F0 41` |
| Description | Type 17 | bit15~8: target motor CAN_ID 7F | Host id 0xFD | Byte0~1: index, corresponding to `loc_kp`; Byte4~7: `loc_kp` value 30, high-byte-first, (32-bit single-precision) hexadecimal IEEE-754 standard floating-point number |

---

## Motor Function Description

*(If the following features are unavailable, please upgrade to the latest version via the official Git repository.)*

**1. Active Reporting**
- Disabled by default. Enable via **Type 24**.
- Report type: **Type 2** (default interval: 10 ms). Adjust interval by modifying `EPScan_time` via **Type 18**.

**2. Zero-Point Flag (`zero_sta`)**
- Modify via:
  - Host computer (上位机)
  - Type 18 (requires saving via **Type 22** for it to persist)
- Default flag: `0` → power-on position range: 0–2π.
- If set to `1`: power-on position range: -π–π.

**3. Type 2 Update**
- Updated to periodic looping within -4π–4π (enables cycle counting).
- **Note:** position interface parameters must be adjusted accordingly:
  - `P_MIN`: 12.57f (as printed in source; see also the firmware macros below, `P_MIN = -12.57f`)
  - `P_MAX`: 12.57f

**4. Protocol Switching** *(requires CAN adapter)*
- Methods:
  - Modify `protocol_1` via host computer.
  - Send **Type 25** command.
- Reboot required after switching.
- Post-switch CAN commands:
  - CANopen: send extended frame (protocol switch frame).
  - MIT Protocol: send standard frame (Command 8).

**5. Post-Power-Off Anti-Backdrive Protection**
- Default: motor imposes damping if rotated rapidly while powered off (prevents voltage surge).
- Disable: set `damper = 1`.

**6. Zero Calibration Rules**
- Supported modes: CSP and Motion Control.
- PP Mode: zero calibration is blocked.
- Old vs. new versions:
  - Old: zero calibration causes large deviation → motor immediately moves to target.
  - New (CSP/Motion Control): target updates to 0 instantly → motor remains stationary.

**7. Position Offset (`add_offset`)**
- Example: if offset = 1, the current zero shifts to (current position + 1 rad).
- Use case: bypass mechanical limits (e.g., set zero at 1 rad → power-on treats 1 rad as the new zero).

**8. CANopen ID**
- Old version: fixed to 1.
- New version: matches the private-protocol CAN ID.

---

## Notes for Implementation

- Always save settings (e.g., **Type 22** for `zero_sta`).
- Verify CAN adapter compatibility for protocol switching.
- For zero offsets, ensure mechanical safety limits are respected.

---

## Control Mode Instructions

### Program Sample

Examples of various mode control motors are provided below (using the GD32F303 as an example). The following are library, function, and macro definitions for the various instances:

```c
#define P_MIN -12.57f

#define P_MAX 12.57f

#define V_MIN -50.0f

#define V_MAX 50.0f

#define KP_MIN 0.0f

#define KP_MAX 500.0f

#define KD_MIN 0.0f

#define KD_MAX 5.0f

#define T_MIN -5.5f

#define T_MAX 5.5f

struct exCanIdInfo{
      uint32_t id:8;
      uint32_t data:16;
      uint32_t mode:5;
      uint32_t res:3;
};

can_receive_message_struct rxMsg;

can_trasnmit_message_struct txMsg={
      .tx_sfid = 0,
      .tx_efid = 0xff,
      .tx_ft = CAN_FT_DATA,
      .tx_ff = CAN_FF_EXTENDED,
      .tx_dlen = 8,
};

#define txCanIdEx (*((struct exCanIdInfo*)&(txMsg.tx_efid)))

#define rxCanIdEx (*((struct exCanIdInfo*)&(rxMsg.rx_efid))) // parses the extended frame id into a custom struct

int float_to_uint(float x, float x_min, float x_max, int bits){
      float span = x_max - x_min;
      float offset = x_min;
      if(x > x_max) x=x_max;
      else if(x < x_min) x= x_min;
      return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

#define can_txd() can_message_transmit(CAN0, &txMsg)

#define can_rxd() can_message_receive(CAN0, CAN_FIFO1, &rxMsg)
```

The following lists the common types of communication sent:

**Motor Enabled Run frame (communication type 3):**

```c
void motor_enable(uint8_t id, uint16_t master_id)
{
      txCanIdEx.mode = 3;
      txCanIdEx.id = id;
      txCanIdEx.res = 0;
      txCanIdEx.data = master_id;
      txMsg.tx_dlen = 8;
      txCanIdEx.data = 0;
      can_txd();
}
```

**Operation control mode Motor control instruction (communication type 1):**

```c
void motor_controlmode(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd)
{
     txCanIdEx.mode = 1;
     txCanIdEx.id = id;
     txCanIdEx.res = 0;
     txCanIdEx.data = float_to_uint(torque,T_MIN,T_MAX,16);
     txMsg.tx_dlen = 8;
     txMsg.tx_data[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8;
     txMsg.tx_data[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16);
     txMsg.tx_data[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;
     txMsg.tx_data[3]=float_to_uint(speed,V_MIN,V_MAX,16);
     txMsg.tx_data[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;
     txMsg.tx_data[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);
     txMsg.tx_data[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;
     txMsg.tx_data[7]=float_to_uint(kd,KD_MIN,KD_MAX,16);
     can_txd();
}
```

*(The source manual's function signature for `motor_controlmode` is truncated mid-parameter-list by a page break — the body clearly uses a `kp` and `kd` parameter in addition to `torque`, `MechPosition`, and `speed`, so the signature above has been completed accordingly; verify against your own firmware if porting this verbatim.)*

**Motor stop frame (communication type 4):**

```c
void motor_reset(uint8_t id, uint16_t master_id)
{
     txCanIdEx.mode = 4;
     txCanIdEx.id = id;
     txCanIdEx.res = 0;
     txCanIdEx.data = master_id;
     txMsg.tx_dlen = 8;
     for(uint8_t i=0;i<8;i++)
     {
            txMsg.tx_data[i]=0;
     }
     can_txd();
}
```

**Motor mode parameter write command (communication type 18, running mode switch):**

```c
uint8_t runmode;
uint16_t index;

void motor_modechange(uint8_t id, uint16_t master_id)
{
     txCanIdEx.mode = 0x12;
     txCanIdEx.id = id;
     txCanIdEx.res = 0;
     txCanIdEx.data = master_id;
     txMsg.tx_dlen = 8;
     for(uint8_t i=0;i<8;i++)
     {
             txMsg.tx_data[i]=0;
     }
     memcpy(&txMsg.tx_data[0],&index,2);
     memcpy(&txMsg.tx_data[4],&runmode, 1);
     can_txd();
}
```

**Motor mode parameter write command (communication type 18, control parameter write):**

```c
uint16_t index;
float ref;

void motor_write(uint8_t id, uint16_t master_id)
{
     txCanIdEx.mode = 0x12;
     txCanIdEx.id = id;
     txCanIdEx.res = 0;
     txCanIdEx.data = master_id;
     txMsg.tx_dlen = 8;
     for(uint8_t i=0;i<8;i++)
     {
             txMsg.tx_data[i]=0;
     }
     memcpy(&txMsg.tx_data[0],&index,2);
     memcpy(&txMsg.tx_data[4],&ref,4);
     can_txd();
}
```

### Operation Control Mode

The motor is in operation control mode by default after power-on.

Send motor Enable Run frame (communication type 3) → Send operation mode motor control command (communication type 1) → Receive motor feedback frame (communication type 2).

**Operation control mode description:** the control logic of the operation control mode is:

```
t_ref = Kd * (v_set - v_actual) + Kp * (p_set - p_actual) + t_ff
```

`t_ref` is converted to the expected `iq` current through an internal formula and output through the current loop.

Simple control demonstrations:

- Set `t_ff` to 0, `v_set` to 1, `Kd` to 1, `p_set` to 0, `Kp` to 0. If there is no external load on the motor, it runs at a speed of 1 rad/s. If there is an external load, `kd` needs to be increased to resist the external load.
- Set `t_ff` to 0, `v_set` to 0, `Kd` to 1, `p_set` to 0, `Kp` to 0: the motor is in damping mode. When the motor is externally rotated, a damping torque is applied, increasing with `kd`. Note that the motor generates electricity under this condition and requires the power supply to sink it, to prevent overvoltage.
- Set `t_ff` to 0, `v_set` to 0, `Kd` to 1, `p_set` to 5, `Kp` to 1. If there is no external load, the motor runs to the target position of 5. Increasing `kp` increases the force required to maintain the target position; `kd` provides damping — without `kd`, the motor will sway around the target position.

### Current Mode

Send motor mode parameter write command (communication type 18), set the `runmode` parameter to 3 → Send motor Enable run frame (communication type 3) → Send motor mode parameter write command (communication type 18), set the `iq_ref` parameter to the desired current instruction.

### Velocity Mode

Send motor mode parameter write command (communication type 18), set the `runmode` parameter to 2 → Send motor Enable run frame (communication type 3) → Send motor mode parameter write command (communication type 18), set `limit_cur` parameter as the desired maximum current instruction → Send motor mode parameter write command (communication type 18), set `acc_rad` parameter as the desired acceleration instruction → Send motor mode parameter write command (communication type 18), set `spd_ref` parameter as the desired speed instruction.

### Location Mode (CSP)

Send motor mode parameter write command (communication type 18), set the `runmode` parameter to 5 → Send motor Enable run frame (communication type 3) → Send motor mode parameter write command (communication type 18), set `limit_spd` parameter as the desired maximum speed instruction → Send motor mode parameter write command (communication type 18), set `loc_ref` parameter as the desired position instruction.

### Location Mode (PP)

Send motor mode parameter write command (communication type 18), set the `runmode` parameter to 1 → Send motor Enable run frame (communication type 3) → Send motor mode parameter write command (communication type 18), set the `vel_max` parameter as the desired maximum speed instruction → Send motor mode parameter write command (communication type 18), set the `acc_set` parameter to the desired acceleration instruction → Send motor mode parameter write command (communication type 18), set the `loc_ref` parameter to the desired position instruction.

**Note:** This mode does not support changing speed and acceleration during operation. To make an emergency stop, you can change `vel_max` to 0 during the process; the motor will stop according to the current speed/acceleration plan.

### Stop Running

Send motor stop frame (communication type 4).

---

## Explanation of CANopen Communication Protocol Types

### State Machine Description

The CANopen state machine (DS402-style) has three top-level groups of states: **POWER DISABLED** (`START` → `NOT READY TO SWITCH ON` → `SWITCH ON DISABLED` → `READY TO SWITCH ON` → `SWITCHED ON`), **FAULT** (`FAULT REACTION ACTIVE` → back to `SWITCH ON DISABLED` via transition 130), and **POWER ENABLED** (`OPERATION ENABLE` ⇄ `QUICK STOP ACTIVE`). Numbered transitions:

- `SWITCH ON DISABLED` → `READY TO SWITCH ON`: transition 6
- `READY TO SWITCH ON` → `SWITCHED ON`: transition 7
- `SWITCHED ON` → `OPERATION ENABLE`: transition 15
- `OPERATION ENABLE` → `SWITCH ON DISABLED`: transition 1
- `OPERATION ENABLE` ⇄ `QUICK STOP ACTIVE`: transitions 11 (enter) / 15 (return)
- `FAULT REACTION ACTIVE` → `SWITCH ON DISABLED`: transition 130

**Motor Enable:** When initially powered on, the motor defaults to the `SWITCH_ON_DISABLED` state. To transition to `OPERATION_ENABLE`, modify the Controlword (6040H) to 6, 7, or 15 (step-by-step transition), or directly set it to 15 for immediate enablement.

**Stopping the Motor:** If the motor is in `OPERATION_ENABLE` state and needs to stop normally, modify the Controlword (6040H) to 1. The motor returns to the disabled state (`SWITCH_ON_DISABLED`).

**Emergency Stop (use with caution — risk of voltage surge):** during operation, an emergency stop can be triggered by setting the Controlword (6040H) to 11.

**Fault Clearance:** if the motor enters a FAULT state due to protection mechanisms, modifying the Controlword (6040H) can clear standard errors.

**Important Note:** mode changes for this motor must be performed in the disabled state (`SWITCH_ON_DISABLED`). Ensure the desired mode is configured before enabling `OPERATION_ENABLE` to avoid unexpected behavior.

### Status Feedback Parameters

| Index | Name | Attribute | Type | Unit |
|---|---|---|---|---|
| 603F | Error_code | Read-only | UINTEGER16 | / |
| 6041 | Statusword | Read-only | UINTEGER16 | / |
| 6061 | Modes_of_operation_display | Read-only | INTEGER8 | / |
| 6062 | Position_demand_value | Read-only | INTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 6064 | Position_actual_value | Read-only | INTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 606B | Velocity_demand_value | Read-only | INTEGER32 | 0.1 rpm |
| 606C | Velocity_actual_value | Read-only | INTEGER32 | 0.1 rpm |
| 6077 | Torque_actual_value | Read-only | INTEGER16 | 0.1% load ratio (1000 = 2 N·m) |
| 6078 | Current_actual_value | Read-only | INTEGER16 | mA |
| 6079 | DC_link_circuit_voltage | Read-only | INTEGER32 | mV |

### Homing Mode (Zero Position Setting)

| Index | Name | Attribute | Type | Unit |
|---|---|---|---|---|
| 6040 | Controlword | Read-write | UINTEGER16 | / |
| 6060 | Modes of operation | Read-write | INTEGER8 | / |

**Homing method:**
- Set **Modes of operation** to 6 while the motor is in the disabled state (`SWITCH_ON_DISABLED`). The motor then defines the current position as the zero point.
- To hold the zero position, modify the **Controlword** to 15, and the motor maintains its position at the home location.

### Position Mode (PP - Profile Position)

| Index | Name | Attribute | Type | Unit |
|---|---|---|---|---|
| 6040 | Controlword | Read-write | UINTEGER16 | / |
| 6060 | Modes of operation | Read-write | INTEGER8 | / |
| 6067 | Position_window | Read-write | UINTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 6068 | Position_window_time | Read-write | UINTEGER16 | ms |
| 6071 | Target_torque | Read-write | INTEGER16 | 0.1% load ratio (1000 = 2 N·m) |
| 607A | Target_position | Read-write | INTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 6081 | Profile_velocity | Read-write | UINTEGER32 | 0.1 rpm |
| 6083 | Profile_acceleration | Read-write | UINTEGER32 | 0.1 rpm/s |

**Steps to configure Position Mode (PP):**

1. While the motor is in the disabled state (`SWITCH_ON_DISABLED`), set **Modes of operation** to 1.
   - Mandatory parameters: `Target_torque` (absolute max torque in position mode), `Profile_velocity` (absolute speed in position mode), `Profile_acceleration` (absolute acceleration in position mode)
   - Optional parameters: `Position_window` (if not set, window check is disabled), `Position_window_time` (if not set, window check is disabled)
2. Set **Controlword** (6040) to 15 to enable operation.
3. Set **Target_position** (absolute position) to move the motor to the desired position.

### Position Mode (CSP - Cyclic Synchronous Position)

| Index | Name | Attribute | Type | Unit |
|---|---|---|---|---|
| 6040 | Controlword | Read-write | UINTEGER16 | / |
| 6060 | Modes of operation | Read-write | INTEGER8 | / |
| 6067 | Position_window | Read-write | UINTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 6068 | Position_window_time | Read-write | UINTEGER16 | ms |
| 6071 | Target_torque | Read-write | INTEGER16 | 0.1% load ratio (1000 = 2 N·m) |
| 607A | Target_position | Read-write | INTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 6081 | Profile_velocity | Read-write | UINTEGER32 | 0.1 rpm |

**Steps to configure Position Mode (CSP):**

1. While the motor is in the disabled state (`SWITCH_ON_DISABLED`), set **Modes of operation** to 5.
   - Mandatory parameters: `Target_torque` (absolute max torque in position mode), `Profile_velocity` (absolute speed in position mode)
   - Optional parameters: `Position_window` (0 = disabled), `Position_window_time` (0 = disabled)
2. Set **Controlword** (6040) to 15 to enable operation.
3. Set **Target_position** (absolute position) to move the motor to the desired position.

### Velocity Mode

| Index | Name | Attribute | Type | Unit |
|---|---|---|---|---|
| 6040 | Controlword | Read-write | UINTEGER16 | / |
| 6060 | Modes of operation | Read-write | INTEGER8 | / |
| 6071 | Target_torque | Read-write | INTEGER16 | 0.1% load ratio (1000 = 2 N·m) |
| 60FF | Target_velocity | Read-write | INTEGER32 | 0.1 rpm |

**Steps to configure Velocity Mode:**

1. While the motor is in the disabled state (`SWITCH_ON_DISABLED`), set **Modes of operation** to 3.
   - Mandatory parameter: `Target_torque` (absolute max torque in velocity mode)
2. Set **Controlword** (6040) to 15 to enable operation.
3. Set **Target_velocity** to reach the desired speed.

### Torque Mode

| Index | Name | Attribute | Type | Unit |
|---|---|---|---|---|
| 6040 | Controlword | Read-write | UINTEGER16 | / |
| 6060 | Modes of operation | Read-write | INTEGER8 | / |
| 6071 | Target_torque | Read-write | INTEGER16 | 0.1% load ratio (1000 = 2 N·m) |

**Steps to configure Torque Mode:**

1. While the motor is in the disabled state (`SWITCH_ON_DISABLED`), set **Modes of operation** to 4.
2. Set **Controlword** (6040) to 15 to enable operation.
3. Set **Target_torque** to output the desired torque.

### Protocol Switching (Extended Frame): Switch Motor Protocol (Takes Effect After Power Cycle)

| Data Field | 29-bit ID | 8-Byte Data Area |
|---|---|---|
| Size | Bit 28~0 | Byte 0~6 |
| Description | 0xFFF | `01 02 03 04 05 06 F_CMD` |

- **F_CMD** (Byte 6) defines the motor protocol:
  - `0`: Private protocol (default)
  - `1`: CANopen protocol
  - `2`: MIT protocol

**Response Frame:**

| Data Field | 11-bit ID | 8-Byte Data Area |
|---|---|---|
| Size | Bit 10~0 | Byte 0~7 |
| Description | Motor ID | 64-bit MCU unique identifier |

---

## MIT Communication Protocol Description

The motor communication adopts the CAN 2.0 interface with a default baud rate of 1 Mbps. The baud rate can be modified by switching to the private protocol. The standard frame format is as follows:

| Data Field | | 11-bit ID | | 8-byte Data Area |
|---|---|---|---|---|
| Size | Bit 10~8 | Bit 7~0 | | Byte 0~7 |
| Description | Mode type | ID | | |

**Supported Control Modes:**
- **MIT Mode:** provides five motion control parameters to the motor.
- **Velocity Mode:** specifies the target speed for the motor.
- **Position Mode:** specifies the target position and speed, allowing the motor to run to the designated position at the configured speed.

### Response Command 1: Data Feedback (Motor Status)

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | Byte 0~7 |
| Description | Host ID | Byte 0: Motor CAN ID. Byte 1~2: current angle [0~65535], corresponds to (-12.57 rad ~ 12.57 rad). Byte 3 (high 8 bits) + Byte 4[7-4] (low 4 bits): current speed [0~4096], corresponds to (-50 rad/s ~ 50 rad/s). Byte 4[3-0] (high 4 bits) + Byte 5 (low 8 bits): current torque [0~4096], corresponds to (-6 N·m ~ 6 N·m). Byte 6~7: winding temperature (in degrees). |

### Response Command 2: MCU Identification

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | Byte 0~7 |
| Description | Motor ID | 64-bit MCU unique identifier |

### Command 1: Enable Motor Operation

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | Byte 0~7 |
| Description | Target motor CAN ID | `FF FF FF FF FF FF FF FC` |

Response: Response Command 1.

### Command 2: Stop Motor Operation

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | Byte 0~7 |
| Description | Target motor CAN ID | `FF FF FF FF FF FF FF FD` |

Response: Response Command 1.

### Command 3: MIT Dynamic Parameters

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | Byte 0~1: target angle [0~65535], (-12.57 rad ~ 12.57 rad). Byte 2 (high 8 bits) + Byte 3[7-4] (low 4 bits): target speed [0~4096], (-50 rad/s ~ 50 rad/s). Byte 3[3-0] (high 4 bits) + Byte 4 (low 8 bits): Kp [0~4096], (0~500). Byte 5 (high 8 bits) + Byte 6[7-4] (low 4 bits): Kd [0~4096], (0~5). Byte 6[3-0] (high 4 bits) + Byte 7 (low 8 bits): target torque [0~4096], (-6 N·m ~ 6 N·m). |

Response: Response Command 1.

### Command 4: Set Zero Position (Non-Position Mode)

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | Byte 0~7 |
| Description | Target motor CAN ID | `FF FF FF FF FF FF FF FE` |

Response: Response Command 1.

### Command 5: Clear Errors & Read Fault Status

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | `FF FF FF FF FF FF F_CMD FB`. **F_CMD:** `0xFF` → clear current fault; any other value → returns fault value in **Byte 1** of the response. |

Response (Fault Clear): Response Command 1.

**Fault Status Response:**

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | Byte 0: Motor CAN ID. Byte 1~4: fault value (non-zero: fault present; 0: normal). Bit 14: stall/I²t overload fault. Bit 7: encoder not calibrated. Bit 3: overvoltage fault. Bit 2: undervoltage fault. Bit 1: driver IC fault. Bit 0: motor overtemperature fault (default threshold 145 °C). |

### Command 6: Set Operation Mode

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | `FF FF FF FF FF FF F_CMD FC`. **F_CMD:** mode type — `0`: MIT mode (default); `1`: Position mode; `2`: Velocity mode. |

Response: Response Command 1.

### Command 7: Modify Motor CAN ID

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | `FF FF FF FF FF FF F_CMD FA`. **F_CMD:** target motor CAN ID. |

Response: Response Command 2.

### Command 8: Change Communication Protocol (Takes Effect After Power Cycle)

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | `FF FF FF FF FF FF F_CMD FD`. **F_CMD:** protocol type — `0`: Private protocol (default); `1`: CANopen; `2`: MIT protocol. |

Response: Response Command 2.

### Command 9: Modify Host CAN ID

| Data Field | 11-bit ID | 8-byte Data Area |
|---|---|---|
| Size | Bit 10~0 | `FF FF FF FF FF FF F_CMD 01`. **F_CMD:** host CAN ID. |

Response: Response Command 2.

### Command 10: Position Mode Control Command

| Data Field | 11-bit ID | Bit 7~0 | 8-byte Data Area |
|---|---|---|---|
| Size | Bit 10~8 | | Byte 0~3: target position (rad, 32-bit float). Byte 4~7: target speed (rad/s, 32-bit float). |
| Description | 1 | Target motor CAN ID | |

Response: Response Command 1.

### Command 11: Velocity Mode Control Command

| Data Field | 11-bit ID | Bit 7~0 | 8-byte Data Area |
|---|---|---|---|
| Size | Bit 10~8 | | Byte 0~3: target speed (rad/s, 32-bit float). Byte 4~7: current limit in speed/position mode (A, 32-bit float). |
| Description | 2 | Target motor CAN ID | |

Response: Response Command 1.

### Motion Control Mode

Block-diagram summary of the control law: a position error (expected angle − current angle) is scaled by `Kp`; a velocity error (expected speed − current speed) is scaled by `Kd`; the two are summed with the expected torque to form the final torque `t_ref`, which passes through a mapping function `f(t_ref)` to produce the expected `iq` current reference. The expected `id` reference is fixed at 0.

The motor defaults to Motion Control Mode upon power-up.

1. Send the Motor Enable Command (Command 1).
2. Send the Motion Control Command (Command 3) to activate dynamic parameter control.
3. Send the Motor Stop Command (Command 2) to halt operation when needed.

### Velocity Mode

Block-diagram summary: expected speed vs. current speed error is fed through a PI controller (`Kp` of the speed loop, `Ki` of the speed loop via an integrator), producing an `iq_ref` after torque-protection clamping; `id_ref` is fixed at 0. Both feed the FOC current loop.

1. Configure the motor's operation mode by sending Set Operation Mode Command (Command 6) with **Mode = 2 (Velocity Mode)**.
2. Send the Motor Enable Command (Command 1) to activate the motor.
3. Send the Velocity Mode Control Command (Command 11) to set the maximum current (absolute value) and target speed.
4. To stop, send the Motor Stop Command (Command 2).

### Position Mode (CSP - Cyclic Synchronous Position)

Block-diagram summary: expected position vs. current position error is scaled by the position-loop `Kp`, producing a speed command that is clamped by the speed limit; this is compared against current speed and fed through the speed-loop PI (`Kp`/`Ki` of the speed loop) and torque protection to produce `iq_ref` (with `id_ref` fixed at 0) into the FOC current loop.

1. Configure the motor's operation mode by sending Set Operation Mode Command (Command 6) with **Mode = 1 (Position Mode)**.
2. Send the Motor Enable Command (Command 1) to activate the motor.
3. Send the Position Mode Control Command (Command 10) to set the maximum speed (absolute value) and target position.
4. To stop, send the Motor Stop Command (Command 2).
