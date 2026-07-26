# RS04 Instruction Manual

**Document:** RS04-EN
**Publisher:** Beijing Lingfoot Times Technology Co., LTD. ("Lingfoot" / RobStride)

---

## Table of Contents

- [Precautions](#precautions)
- [Legal Statement](#legal-statement)
- [After-sales Policy](#after-sales-policy)
- [Motor Specification](#motor-specification)
  - [Standard Service Condition](#standard-service-condition)
  - [Electrical Characteristics](#electrical-characteristics)
  - [Mechanical Characteristic](#mechanical-characteristic)
- [Driver Product Information](#driver-product-information)
  - [Driver Product Specifications](#driver-product-specifications)
  - [Driver Interface Definition](#driver-interface-definition)
  - [Driver Harness Definition](#driver-harness-definition)
  - [Driver Interface Recommended Brand and Model](#driver-interface-recommended-brand-and-model)
  - [Driver Function Pin and Device Description](#driver-function-pin-and-device-description)
  - [Main Devices and Specifications](#main-devices-and-specifications)
- [Upper Computer Instructions](#upper-computer-instructions)
  - [Hardware Disposition](#hardware-disposition)
  - [Upper Computer Interface and Description](#upper-computer-interface-and-description)
  - [Motor Settings](#motor-settings)
  - [Parameter Settings](#parameter-settings)
  - [Oscilloscope](#oscilloscope)
  - [Communication Box Instruction Example](#communication-box-instruction-example)
  - [CAN Communication Failure Protection](#can-communication-failure-protection)
  - [Motor Fault Instructions](#motor-fault-instructions)
- [Control Demo](#control-demo)
- [Driver Protocol and Instructions (Private / RobStride Protocol)](#driver-protocol-and-instructions-private--robstride-protocol)
  - [Description of the Communication Protocol Type](#description-of-the-communication-protocol-type)
  - [Read and Write a Single Parameter List](#read-and-write-a-single-parameter-list)
  - [Motor Function Description](#motor-function-description)
  - [Control Mode Instructions / Program Sample](#control-mode-instructions--program-sample)
- [Explanation of CANopen Communication Protocol Types](#explanation-of-canopen-communication-protocol-types)
- [MIT Communication Protocol Description](#mit-communication-protocol-description)

---

## Precautions

1. Please use according to the working parameters specified in this article, otherwise it may cause serious damage to the product!
2. Do not switch the control mode when the joint is running. If you need to switch, send the command to stop the operation before switching.
3. Check whether the parts are in good condition before use. If the parts are missing or damaged, contact technical support in time.
4. Do not disassemble the motor at will, so as to avoid unrecoverable failure.
5. Ensure that there is no short circuit when the motor is connected, and the interface is correctly connected as required.

---

## Legal Statement

Before using this product, please read this manual carefully and operate the product according to the contents of this manual. If the user violates the contents of this manual to use this product, resulting in any property damage or personal injury accident, the company does not assume any responsibility. Because this product is composed of many parts, do not allow children to touch this product to avoid accidents. In order to prolong the service life of the product, do not use this product in a high temperature and high pressure environment. This manual has been printed to the extent possible to include a description of the functions and instructions for use. However, due to the continuous improvement of product functions, design changes, etc., there may still be discrepancies with the products purchased by users.

The color and appearance of this manual may differ from the actual product. Please refer to the actual product. This manual is published by Beijing Lingfoot Times Technology Co., LTD. (hereinafter referred to as Lingfoot), and Lingfoot may at any time make necessary improvements and changes to the inaccurate and up-to-date information in this manual, or make improvements to procedures and/or equipment. Such changes will be uploaded to the company's official website in electronic format. Details can be found in the download center (www.robstride.com). All images are for reference only. Please refer to actual objects.

---

## After-sales Policy

The after-sales service of this product is implemented in strict accordance with the Law of the People's Republic of China on the Protection of Consumer Rights and Interests and the Product Quality Law of the People's Republic of China. The service content is as follows:

1. **Warranty period and contents**

   a. Users who place orders on the online channel to purchase this product can enjoy the return service without reason within seven days from the day after signing. When returning goods, the user must present a valid proof of purchase and return the invoice. The user must ensure that the returned goods maintain the original quality and function, the appearance is intact, the trademarks and various logos of the goods themselves and accessories are complete, and if there are gifts, they should be returned together. If the goods are artificially damaged, artificially disassembled, missing packaging boxes, missing parts and accessories, they will not be returned. The logistics cost incurred during the return shall be borne by the user (see "After-sales Service Fee Standard"). If the user does not settle the logistics cost, it will be deducted from the refund amount according to the actual amount incurred. Refund the amount paid to the user within seven days from the date of receipt of the returned item. Refund method is the same as payment method. The specific arrival date may be affected by factors such as banks and payment institutions.

   b. The warranty period of this product is 1 year.

   c. Within 7 days after the user signs for the next day, if a non-human-damage performance failure occurs, confirmed through testing by the Lingfoot after-sales service center, the user may return the goods; the user must present a valid purchase voucher and return the invoice. Any freebies should be returned.

   d. From 7 days to 15 days after the user signs for the next day, if a non-human-damage performance failure occurs, confirmed through testing by the Lingfoot after-sales service center, the user may have the whole set of goods replaced. After replacement, the warranty period of the goods is recalculated.

   e. From 15 days to 365 days after the user signs for the next day, after inspection and confirmation by the Lingfoot after-sales service center that it is a quality fault of the product itself, free maintenance service will be provided. The replaced faulty product becomes the property of Lingfoot. A product with no fault found will be returned as-is. This product is strictly tested after manufacture; if a fault is found that is not a quality fault of the product itself, the company reserves the right to refuse the user's return request.

2. **Non-warranty regulations.** The following circumstances are not covered by the warranty:

   3. Exceeding the warranty period specified in the warranty terms.
   4. Failure to follow the instructions, resulting in product damage caused by wrong use.
   5. Damage caused by improper operation, maintenance, installation, modification, testing, or other improper use.
   6. Non-quality failure caused by conventional mechanical loss or wear.
   7. Damage caused by abnormal working conditions, including but not limited to falling, impact, liquid immersion, violent impact, etc.
   8. Damage caused by natural disasters (such as floods, fires, lightning strikes, earthquakes, etc.) or force majeure.
   9. Damage caused by exceeding peak torque.
   10. Damage caused by exceeding peak torque. *(duplicated in the source manual)*
   11. Failure or damage caused by other problems not related to the product's design, technology, manufacturing, or quality.
   12. Use of this product for commercial purposes.

   In the case of the above situations, the user must pay the associated cost.

---

## Motor Specification

### Standard Service Condition

1. Rated voltage: 48 VDC
2. Operating voltage range: 24 V–60 VDC
3. Rated load (CW): 40 N·m
4. Operation direction: CW/CCW from the direction of the exit shaft
5. Use posture: the direction of the exit axis is horizontal or vertical
6. Standard operating temperature: 25±5℃
7. Operating temperature range: -20 ~ 50℃
8. Standard operating humidity: 65%
9. Humidity range: 5 ~ 85%, no condensation
10. Storage temperature range: -30 ~ 70℃
11. Insulation Class: Class B

### Electrical Characteristics

1. No load speed: 200 rpm ± 10%
2. No load current: 2 Arms
3. Rated load: 40 N·m
4. Rated load speed: 167 rpm ± 10%
5. Rated load phase current (peak): 27 Apk ± 10%
6. Peak load: 120 N·m
7. Maximum load phase current (peak): 90 Apk ± 10%
8. Insulation resistance/stator winding: DC 500 VAC, 100 MΩ
9. High voltage/stator and housing: 600 VAC, 1 s, 2 mA
10. Motor back potential: 16.9 Vrms/krpm ± 10%
11. Torque constant (valid value): 2.1 N·m/Arms
12. T-N curve: torque stays flat at ~120 N·m from 0 to ~100 rpm, then rolls off roughly linearly to 0 N·m at 200 rpm.
13. Maximum overload curve

   Test conditions: Ambient temperature 25℃; winding limit temperature 130℃ (this is the constraint temperature — the actual limit is 180 degrees); speed 24 rpm.

   **Test data** (maximum allowed continuous operating time at a given load):

   | Load (N·m) | Operating time (s) |
   | --- | --- |
   | 120 | 10 |
   | 110 | 12 |
   | 100 | 14 |
   | 90 | 20 |
   | 80 | 40 |
   | 70 | 59 |
   | 60 | 116 |
   | 50 | 300 |
   | 40 | rated (continuous) |

### Mechanical Characteristic

1. Weight: 1420 g ± 20 g
2. Number of poles: 42
3. Phase number: 3 phases
4. Drive mode: FOC
5. Deceleration ratio: 9:1

---

## Driver Product Information

### Driver Product Specifications

| Project | Data |
| --- | --- |
| The rated working voltage | 48 VDC |
| The maximum allowable voltage | 60 VDC |
| Rated working phase current | 27 Apk |
| Maximum allowable phase current | 90 Apk |
| Standby power | ≤40 mA |
| CAN bus bit rate | 1 Mbps |
| Dimensions | Φ84 mm |
| Working environment temperature | -20℃ to 50℃ |
| Maximum allowable temperature of the control board | 105℃ |
| Encoder resolution | 14 bit (absolute, single-turn) |

### Driver Interface Definition

The driver connector exposes 4 pins, from one side to the other: **CAN-H**, a data/USB-style connector body, **CAN-L**, **GND**, and, on a separate 2-pin power connector, **VBAT+**.

### Driver Harness Definition

| Harness color | Definition |
| --- | --- |
| blue | CAN_H |
| brown | CAN_L |
| black | GND |
| red | VBAT+ |

### Driver Interface Recommended Brand and Model

| Board end model | Brand manufacturer | Line end model | Brand manufacturer |
| --- | --- | --- | --- |
| XT30APW-M | AMASS (Ams) | XT30UW-F | AMASS (AMS) |
| GH1.25-2PWT | any | GH1.25-T | any |

### Driver Function Pin and Device Description

**1. Power supply and CAN communication**

| Pin | Description |
| --- | --- |
| 1 | The positive electrode of the power supply (+) |
| 2 | Negative electrode of the power supply (-) |
| 3 | CAN low side of the communication, CAN_L |
| 4 | CAN high side of the communication, CAN_H |

**2. Download port**

| Pin | Description |
| --- | --- |
| 1 | SWDIO (data) |
| 2 | SWCLK (clock) |
| 3 | 3V3 (positive 3.3 V) |
| 4 | GND |

**3. Indicator light** — status LED (see the "Motor connection settings" section for LED behavior during enable/disable).

### Main Devices and Specifications

| No. | Item | Specifications | Quantity |
| --- | --- | --- | --- |
| 1 | MCU chip | GD32F303RET6 | 1 PCS |
| 2 | Driver chip | DRV8353SRTAT | 1 PCS |
| 3 | Magnetic encoder chip | AS5047P | 2 PCS |
| 4 | Thermistor | LTS00-104J395T19E010 / NCP18XH103F03RB | 2 PCS |
| 5 | Power MOS | ISC030N12NM6 | 12 PCS |

---

## Upper Computer Instructions

Please go to the www.robstride.com website download center to obtain the upper computer (host) software, named **motorstudio**.

### Hardware Disposition

The articulated motor uses the CAN communication mode and has two communication cables. It is connected to the debugger through the CAN-to-USB tool. The debugger needs to be installed with the ch340 driver in advance and works in AT mode by default.

It should be noted that this debugger is based on RobStride's specific CAN-to-USB tool, so the recommended serial port tool must be used to debug the debugger. If you want to port this to another debugger platform, refer to the development instructions (see the driver protocol chapter).

The CAN-to-USB tool recommended is the official USB-CAN module of Lingzu (RobStride) Times. The frame header of the corresponding serial port protocol is `41 54`, and the frame tail is `0D 0A`.

When using the CAN-to-USB module, pay attention to the DIP switch settings on the module:

- When DIP switch 1 is in the ON position, the module enters Boot mode and cannot establish a connection with the host computer.
- When DIP switch 2 is in the ON position, a 120Ω terminal resistor is connected to the module port, allowing normal communication with the host computer.

### Upper Computer Interface and Description

The **motorstudio** software mainly includes:

**A. Motor Connection Module**
- Refreshing the Serial Port
- Opening the Serial Port
- Testing the Device (Detect Device)

**B. Motor Configuration Module**
- Encoder Calibration
- Motor Active Reporting Switch (Start Auto Report)
- Setting the Motor Active Reporting Time (Set Report Time)
- Modifying the Motor CAN ID (Set ID)
- Setting the Motor's Mechanical Zero Position (Set Zero Position)

**C. Motor Upgrade Module (OAT)**
- Opening a File
- Starting the Upgrade
- Erase

**D. Motor Main Interface**
- Parameter Settings
- Motor Oscilloscope

**E. Run and Debug Area**
- Parameter Debugging Buttons (Enable / Stop / Jog+ / Jog-)
- Motor Mode Configuration and Parameter Modification (MIT / PP / Velocity / Current tabs)
- Sine Signal Testing

### Motor Settings

#### Motor connection settings

Connect the CAN-to-USB tool (install the ch340 driver, which works in AT mode by default), click **Refresh Serial Port**, open the serial port, and click **Detect Device** to detect the corresponding motor. The green text below the connection panel shows the detected motor type (e.g. `motor type RS00`), and the connection panel also shows the assigned CAN address, e.g. `CAN: 127  id: 6b02...`.

#### Motor configuration module

1. **Recalibrate the motor magnetic encoder** (Encoder Calibration). Reinstalling the motor board and motor, or reconnecting the motor's three-phase wiring, requires recalibrating the magnetic encoder.
2. **Enable active motor reporting.** Click **Start Auto Report** to enable active motor reporting in communication type 2. You can set the interval below (**Set Report Time**), with a minimum of 10 ms.
3. **Set ID**: Set the motor's CAN ID.
4. **Set Zero Position**: Set the current position to 0.

#### Motor upgrade module (OAT)

1. Click **Open File** and select the firmware to upgrade. The `rs-0x` in the firmware file name identifies the target motor type.
2. Click **Start Update** (Start the Upgrade), and the motor will enter the upgrade preparation stage (progress bar shows "Erasing flash, please wait...").
3. When the green text "The equipment has entered the upgrade mode..." appears, click to start the upgrade.
4. When the green text "Upgrade Successfully" pops up and the progress bar reaches 100%, the upgrade is complete.

If the green progress bar gets stuck halfway through the upgrade, you can click to stop the upgrade, or re-power on and re-enter the upgrade process. The internal program of the motor will not be lost after a failed upgrade. Please check whether the communication environment is good before upgrading again.

### Parameter Settings

After successfully connecting to the motor:

1. Click **Refresh Parameter Table**. "Update parameter table successfully!" will appear, indicating that the motor parameters have been successfully read. (Note: The parameter table must be read while the motor is in standby mode. If the motor is running, the parameter table refresh cannot be performed.) The interface displays the motor's parameters. Parameters highlighted in **green** are string/identity fields; parameters in **blue** are stored internally in the motor and can be modified in the Current Value field.
2. Click **Read Parameters** to upload the motor parameters to the debugger. Parameters in light blue/cyan are observed (telemetry) parameters, collected and observable in real time.
3. Click **Write Parameters** to download the debugger's parameter values to the motor.
4. Click **Factory Reset** to restore the motor's default parameters for the currently installed firmware.
5. Click **Export ParaTable** to export the current motor parameters.
6. Click **Open Multi-device** to connect the host computer to multiple motors. Because the parameter interfaces differ across motor types, multi-device connection is intended for upgrades only. After upgrading and debugging a motor, close the multi-device connection and search for the motor again.

> **Note:** Please do not change the torque limit, protection temperature, and overtemperature time of the motor. The company will not bear any legal responsibility for any damage to the human body or irreversible damage to joints caused by illegal operation of this product.

#### Full parameter table

This is the complete parameter table as shown in the motorstudio "Parameter Setting" tab (identity/string fields, tunable control-loop fields, and read-only telemetry fields):

| Function code | Name | Type | Attribute | Max | Min | Current value (reference) | Note |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0X0000 | Name | String | Read/Write | | | (device name string) | |
| 0X0001 | BarCode | String | Read/Write | | | (barcode string) | |
| 0X1000 | BootCodeVersion | String | Read only | | | 0.1.5 | |
| 0X1001 | BootBuildDate | String | Read only | | | Mar 16 2022 | |
| 0X1002 | BootBuildTime | String | Read only | | | 20:22:09 | |
| 0X1003 | AppCodeVersion | String | Read only | | | 0.0.0.1 | Motor program version number |
| 0X1004 | AppGitVersion | String | Read only | | | 7b844b0fM | |
| 0X1005 | AppBuildDate | String | Read only | | | Apr 14 2022 | |
| 0X1006 | AppBuildTime | String | Read only | | | 20:30:22 | |
| 0X1007 | AppCodeName | String | Read only | | | Lingzu_motor | |
| 0X2000 | echoPara1 | uint16 | disposition | 74 | 5 | 5 | |
| 0X2001 | echoPara2 | uint16 | disposition | 74 | 5 | 5 | |
| 0X2002 | echoPara3 | uint16 | disposition | 74 | 5 | 5 | |
| 0X2003 | echoPara4 | uint16 | disposition | 74 | 5 | 5 | |
| 0X2004 | echoFreHz | uint32 | Read/Write | 10000 | 1 | 500 | |
| 0X2005 | MechOffset | float | Settings | 7 | -7 | 4.619583 | Motor magnetic encoder angle offset |
| 0X2006 | MechPos_init | float | Read/Write | 50 | -50 | 4.52 | Reserved parameter |
| 0X2007 | limit_torque | float | Read/Write | 17 | 0 | 17 | Torque limitation |
| 0X2008 | I_FW_MAX | float | Read/Write | 33 | 0 | 0 | Weak magnetic current value, default 0 |
| 0X2009 | motor_baud | uint8 | Settings | 20 | 0 | 1 | Motor index, marking the joint position of the motor* |
| 0X200A | CAN_ID | uint8 | Settings | 127 | 0 | 1 | ID of this device |
| 0X200B | CAN_MASTER | uint8 | Settings | 127 | 0 | 0 | CAN host ID |
| 0X200C | CAN_TIMEOUT | uint32 | Read/Write | 100000 | 0 | 0 | CAN timeout threshold; default value is 0 |
| 0X200D | status2 | int16 | Read/Write | 1500 | 0 | 800 | Reserved parameter |
| 0X200E | status3 | uint32 | Read/Write | 1000000 | 1000 | 20000 | Reserved parameter |
| 0X200F | status1 | float | Read/Write | 64 | 1 | 7.75 | Reserved parameter |
| 0X2010 | Status6 | uint8 | Read/Write | 1 | 0 | 1 | Reserved parameter |
| 0X2011 | cur_filt_gain | float | Read/Write | 1 | 0 | 0.9 | Current filtering parameter |
| 0X2012 | cur_kp | float | Read/Write | 200 | 0 | 0.025 | Current Kp |
| 0X2013 | cur_ki | float | Read/Write | 200 | 0 | 0.0258 | Current Ki |
| 0X2014 | spd_kp | float | Read/Write | 200 | 0 | 2 | Velocity Kp |
| 0X2015 | spd_ki | float | Read/Write | 200 | 0 | 0.021 | Speed Ki |
| 0X2016 | loc_kp | float | Read/Write | 200 | 0 | 30 | Position Kp |
| 0X2017 | spd_filt_gain | float | Read/Write | 1 | 0 | 0.1 | Velocity filter parameter |
| 0X2018 | limit_spd | float | Read/Write | 200 | 0 | 2 | Location mode speed limit |
| 0X2019 | limit_cur | float | Read/Write | 23 | 0 | 23 | Position/velocity mode current limit |
| 0X201A | loc_ref_filt_gain | float | Read/Write | 100 | 0 | 0 | Reserved parameter |
| 0X201B | limit_loc | float | Read/Write | 100 | 0 | 0 | Reserved parameter |
| 0X201C | position_offset | float | Read/Write | 27 | 0 | 0 | High speed segment offset |
| 0X201D | chasu_angle_offset | float | Read/Write | 27 | 0 | 0 | The low end offset |
| 0X201E | spd_step_value | float | Read/Write | 150 | 0 | | Velocity-mode acceleration |
| 0X201F | vel_max | float | Read/Write | 20 | 0 | | PP mode speed |
| 0X2020 | acc_set | float | Read/Write | 1000 | 0 | | PP mode acceleration |
| 0X2021 | zero_sta | float | Read/Write | 100 | 0 | 0 | Zero marker |
| 0X3000 | timeUse0 | uint16 | Read only | | | 5 | |
| 0X3001 | timeUse1 | uint16 | Read only | | | 0 | |
| 0X3002 | timeUse2 | uint16 | Read only | | | 10 | |
| 0X3003 | timeUse3 | uint16 | Read only | | | 0 | |
| 0X3004 | encoderRaw | int16 | Read only | | | 11396 | Magnetic encoder sampling value |
| 0X3005 | mcuTemp | int16 | Read only | | | 337 | MCU internal temperature, ×10 |
| 0X3006 | motorTemp | int16 | Read only | | | 333 | Motor NTC temperature, ×10 |
| 0X3007 | vBus(mv) | uint16 | Read only | | | 24195 | Bus voltage |
| 0X3008 | adc1Offset | int32 | Read only | | | 2084 | ADC sampling channel 1 zero current bias |
| 0X3009 | adc2Offset | int32 | Read only | | | 2084 | ADC sampling channel 2 zero current bias |
| 0X300A | adc1Raw | uint16 | Read only | | | 1232 | ADC sampling value 1 |
| 0X300B | adc2Raw | uint16 | Read only | | | 1212 | ADC sampling value 2 |
| 0X300C | VBUS | float | Read only | | | 36 | Bus voltage, V |
| 0X300D | cmdId | float | Read only | | | 0 | Id ring instruction, A |
| 0X300E | cmdIq | float | Read only | | | 0 | Iq ring command, A |
| 0X300F | cmdlocref | float | Read only | | | 0 | Position loop command, rad |
| 0X3010 | cmdspdref | float | Read only | | | 0 | Speed loop command, rad/s |
| 0X3011 | cmdTorque | float | Read only | | | 0 | Torque instruction, N·m |
| 0X3012 | cmdPos | float | Read only | | | 0 | MIT protocol angle instruction |
| 0X3013 | cmdVel | float | Read only | | | 0 | MIT protocol speed instruction |
| 0X3014 | rotation | int16 | Read only | | | 1 | Number of turns |
| 0X3015 | modPos | float | Read only | | | 4.363409 | Motor uncounted coil mechanical angle, rad |
| 0X3016 | mechPos | float | Read only | | | 0.777679 | Load end loop mechanical angle, rad |
| 0X3017 | mechVel | float | Read only | | | 0.036618 | Load speed, rad/s |
| 0X3018 | elecPos | float | Read only | | | 4.714761 | Electrical angle |
| 0X3019 | ia | float | Read only | | | 0 | U-wire current, A |
| 0X301A | ib | float | Read only | | | 0 | V-wire current, A |
| 0X301B | ic | float | Read only | | | 0 | W-wire current, A |
| 0X301C | timeout | uint32 | Read only | | | 31600 | Timeout counter value |
| 0X301D | phaseOrder | uint8 | Read only | | | 0 | Directional marking |
| 0X301E | iqf | float | Read only | | | 0 | Iq filter value, A |
| 0X301F | boardTemp | int16 | Read only | | | 359 | Plate temperature, ×10 |
| 0X3020 | iq | float | Read only | | | 0 | Iq original value, A |
| 0X3021 | id | float | Read only | | | 0 | Id original value, A |
| 0X3022 | faultSta | uint32 | Read only | | | 0 | Fault status value |
| 0X3023 | warnSta | uint32 | Read only | | | 0 | Warning status value |
| 0X3024 | drv_fault | uint16 | Read only | | | 0 | Driver chip fault value 1 |
| 0X3025 | drv_temp | int16 | Read only | | | 48 | Driver chip fault value 2 |
| 0X3026 | Uq | float | Read only | | | 0 | Q-axis voltage |
| 0X3027 | Ud | float | Read only | | | 0 | D-axis voltage |
| 0X3028 | dtc_u | float | Read only | | | 0 | Duty cycle of the U-phase output |
| 0X3029 | dtc_v | float | Read only | | | 0 | Duty cycle of the V-phase output |
| 0X302A | dtc_w | float | Read only | | | 0 | Duty cycle of the W-phase output |
| 0X302B | v_bus | float | Read only | | | 24.195 | Vbus in the closed loop |
| 0X302C | torque_fdb | float | Read only | | | 0 | Torque feedback value, N·m |
| 0X302D | rated_i | float | Read only | | | 8 | Rated current of motor |
| 0X302E | limit_i | float | Read only | | | 27 | Motor's maximum current limit |
| 0X302F | spd_ref | float | Read only | | | 0 | Motor speed expectation |
| 0X3030 | spd_reff | float | Read only | | | 0 | Motor speed expectation 2 |
| 0X3031 | zero_fault | float | Read only | | | 0 | Motor position determination parameter |
| 0X3032 | chasu_coder_raw | float | Read only | | | 0 | Motor position determination parameter |
| 0X3033 | chasu_angle | float | Read only | | | 0 | Motor position determination parameter |
| 0X3034 | as_angle | float | Read only | | | 0 | Motor position determination parameter |
| 0X3035 | vel_max | float | Read only | | | 0 | Motor position determination parameter |
| 0X3036 | judge | float | Read only | | | 0 | Motor position determination parameter |
| 0X3037 | fault1 | uint32 | Read only | | | 0 | Log failure |
| 0X3038 | fault2 | uint32 | Read only | | | 0 | Log failure |
| 0X3039 | fault3 | uint32 | Read only | | | 0 | Log failure |
| 0X303A | fault4 | uint32 | Read only | | | 0 | Log failure |
| 0X303B | fault5 | uint32 | Read only | | | 0 | Log failure |
| 0X303C | fault6 | uint32 | Read only | | | 0 | Log failure |
| 0X303D | fault7 | uint32 | Read only | | | 0 | Log failure |
| 0X303E | fault8 | uint32 | Read only | | | 0 | Log failure |
| 0X303F | ElecOffset | float | Read only | | | 0 | Electrical angle offset |
| 0X3040 | mcOverTemp | int16 | Read only | | | 0 | Overtemperature threshold |
| 0X3041 | Kt_Nm/Amp | float | Read only | | | 0 | Torque (moment) coefficient |
| 0X3042 | Tqcali_Type | uint8 | Read only | | | 0 | Motor type |
| 0X3043 | low_position | float | Read only | | | 0 | Motor position determination parameter |
| 0X3044 | theta_mech_1 | float | Read only | | | 0 | Type 2 low-speed angle |
| 0X3045 | instep | float | Read only | | | 0 | Motor protection decision parameter |

\* This description for `motor_baud` (0X2009) as "motor index / joint position" looks inconsistent with the parameter's name (which suggests a baud-rate setting); transcribed as printed in the source — verify against firmware/register map before relying on it.

### Oscilloscope

The oscilloscope interface supports viewing and observing real-time data graphs, including motor Id/Iq current, temperature, real-time speed at the output end, rotor (encoder) position, output-end position, etc.

Click the oscilloscope tab, select the desired parameters in the 8 channel-selection drop-downs (parameter meaning can be looked up in the parameter table above), set the output frequency (Hz), then click **start** to plot the data graph; click again to stop the observation.

The command that is sent by the oscilloscope module appears in the communication command box below the plot (see the worked example in the next section).

### Communication Box Instruction Example

Example raw bytes sent over the serial-to-CAN adapter:

```
41 54 90 07 e8 0c 08 05 70 00 00 01 00 00 00 0d 0a
```

Breakdown (note: the source table's "extended frame" / "number of data bits" column labels were transposed relative to their values; the table below restores the logical order):

| Bytes | `41 54` | `90 07 e8 0c` | `08` | `05 70 00 00 01 00 00 00` | `0d 0a` |
| --- | --- | --- | --- | --- | --- |
| Meaning | frame header | extended frame (29-bit CAN ID, raw) | number of data bytes (DLC) | data field (8 bytes) | frame tail |

Translating the extended-frame CAN ID into the real 29-bit CAN ID requires the following transformation:

`90 07 e8 0c` converts to binary as `1001 0000 0000 0111 1110 1000 0000 1100`. Remove the rightmost 3 bits (the adapter pads the 29-bit ID up to 32 bits), leaving `1 0010 0000 0000 1111 1101 0000 0001`. Converting that to hexadecimal gives `12 00 FD 01`. According to the communication protocol (see [Driver protocol and instructions](#driver-protocol-and-instructions-private--robstride-protocol)), this decodes as:

| Field | `12` (hex) | `00` | `FD` | `01` |
| --- | --- | --- | --- | --- |
| Meaning | Communication type 18 (`0x12` = 18 decimal) | no meaning | host id | motor CAN ID |

### CAN Communication Failure Protection

When the value of `CAN_TIMEOUT` is 0, this function is disabled.

When the `CAN_TIMEOUT` value is non-zero, if the motor does not receive a CAN command within that period, the motor enters reset mode. The unit is such that 20000 = 1 s.

### Motor Fault Instructions

Function code `0x3022` (`faultSta`) indicates the fault code, where:

- bit14: motor blocking / overload algorithm protection
- bit7: Encoder uncalibrated
- bit3: Overvoltage fault — the motor voltage exceeds the protection voltage of 60 V
- bit2: Undervoltage fault — the motor voltage is lower than the protection voltage of 12 V
- bit1: Driver chip failure
- bit0: Motor overtemperature fault — motor thermistor temperature exceeds 145 degrees

Function code `0x3024` (`drv_fault`) is driver-chip fault code 1:

**Table 11. Fault Status Register 1 Field Descriptions**

| Bit | Field | Type | Default | Description |
| --- | --- | --- | --- | --- |
| 10 | FAULT | R | 0b | Logic OR of fault status registers. Mirrors nFAULT pin. |
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

Function code `0x3025` (`drv_temp`) is driver-chip fault code 2:

**Table 12. Fault Status Register 2 Field Descriptions**

| Bit | Field | Type | Default | Description |
| --- | --- | --- | --- | --- |
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

This section walks through using the motorstudio "Run and Debug Area" to exercise each control mode manually.

### Jog Run

Click **JOG+** / **JOG-** to run the motor forward and reverse at a speed of 1 rad/s.

### Control Mode Switching

Select the desired control mode (MIT / PP / Velocity / Current / ...) in the tab strip to the right of "Run Mode".

### Operation and control mode

1. Switch the control mode to operation (MIT) mode. The panel exposes **Torque (Nm)**, **Angle (rad)**, **Velocity (rad/s)**, **KP**, and **KD** fields plus **Continuous Send** / **Single Send** buttons.
2. Click **Enable**. The motor starts running and enters `motor_mode`.
3. Set the five parameter values and click **Single Send** or **Continuous Send**. The motor will return feedback frames and run according to the target command.
4. Click **Stop** to stop the motor and terminate the continuous sending of commands.

**Operation control mode description:** the control logic of the operation and control mode is

```
t_ref = Kd * (v_set - v_actual) + Kp * (p_set - p_actual) + t_ff
```

`t_ref` is converted to the expected `iq` current through an internal formula and output through the current loop.

Simple control demonstrations:
- Set `t_ff` = 0, `v_set` = 1, `Kd` = 1, `p_set` = 0, `Kp` = 0. If there is no external load on the motor, it will run at a speed of 1 rad/s. If there is an external load, `Kd` needs to be increased to resist it.
- Set `t_ff` = 0, `v_set` = 0, `Kd` = 1, `p_set` = 0, `Kp` = 0 — the motor is in damping mode. When the motor is externally rotated, damping is applied, increasing with `Kd`. Note: the motor generates electricity under this condition and requires the power supply to be able to sink current, to prevent overvoltage.
- Set `t_ff` = 0, `v_set` = 0, `Kd` = 1, `p_set` = 5, `Kp` = 1. If there is no external load, the motor will run to the target position of 5. Increasing `Kp` increases the force required to maintain the target position; `Kd` provides damping. Without `Kd`, the motor will oscillate around the target position.

### Current mode

1. Switch the control mode to current mode.
2. Click **Enable**. The motor starts running and enters `motor_mode`.
3. Set the current command value for **Iq Command (A)**. Click the `>>` button on the right. The motor will follow the current command.
4. Click **Stop** to stop the motor.

### Motor Current Sine Test

1. Switch the control mode to current mode.
2. Click **Enable**. The motor starts running and enters `motor_mode`.
3. Set the amplitude and frequency, click **Apply**, then click **Start**. The corresponding mode target command is then sinusoidally planned.
4. Click **Stop** to stop the motor.

### Speed Mode

1. Switch the control mode to speed mode.
2. Click **Enable**. The motor starts running and enters `motor_mode`.
3. First set the current limit (maximum phase current) and speed step value (acceleration). If not set, the motor operates at the default values. Finally set the speed command (target speed). The motor will follow the command.
4. Click **Stop** to stop the motor.

### Motor Speed Sine Test

1. Switch the control mode to speed mode.
2. Click **Enable**. The motor starts running and enters `motor_mode`.
3. Set the amplitude and frequency, click **Apply**, then click **Start**.
4. Click **Stop** to stop the motor.

### Position Mode (PP)

1. Switch the control mode to interpolated position mode (PP).
2. Click **Enable**. The motor starts running and enters `motor_mode`.
3. First set the speed and acceleration. If not set, the motor operates at the default values. Finally set the position command (target position). The motor will follow the command.
4. Set the speed to 0 to stop the motor at the current position. To continue operation, re-issue the speed and position.
5. Click **Stop** to stop the motor.

### Motor Position Sine Test

1. Switch the control mode to interpolated position mode (PP).
2. Click **Enable**. The motor starts running and enters `motor_mode`.
3. Set the amplitude and frequency, click **Apply**, then click **Start**.
4. Click **Stop** to stop the motor.

### Position Mode (CSP)

1. Switch the control mode to (cyclic synchronous) position mode.
2. Click **Enable**. The motor starts running and enters `motor_mode`.
3. Set the speed first. If no speed setting is made, the motor runs at the default value. Finally set the position command (target position). The motor will follow the command.
4. Click **Stop** to stop the motor.

### Motor Position Sinusoidal Test (CSP)

1. Switch the control mode to position mode.
2. Click **Enable**. The motor starts running and enters `motor_mode`.
3. Set the amplitude and frequency, click **Apply**, then click **Start**. The target command is planned sinusoidally.
4. Click **Stop** to stop the motor.

---

## Driver Protocol and Instructions (Private / RobStride Protocol)

The motor communication uses the CAN 2.0 interface, baud rate 1 Mbps, extended frame format, laid out as follows:

| Data field | 29-bit ID: bit28~bit24 | 29-bit ID: bit23~8 | 29-bit ID: bit7~0 | 8-byte data field: Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | Communication type | data area 2 | Destination address | data area 1 |

The control modes supported by the motor include:

- **Operation control mode:** set 5 parameters of motor operation control.
- **Current mode:** the specified Iq current of the given motor.
- **Velocity mode:** the specified running speed of the given motor.
- **Position mode:** given the specified position, the motor will run to the specified position.

### Description of the Communication Protocol Type

#### Communication type 0: Get device ID

Gets the device's ID and 64-bit MCU unique identifier.

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Size / Description | `0x0` | bit15~8: identifies host CAN_ID | target motor CAN_ID | `0` |

**Reply frame:**

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x0` | target motor CAN_ID | `0xFE` | 64-bit MCU unique identifier |

#### Communication Type 1: operation control mode motor control instruction

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x1` | Byte2 (of bit23~8 area): Torque `[0~65535]` corresponds to `(-120 N·m ~ 120 N·m)` | target motor CAN_ID | Byte0~1: target angle `[0~65535]` corresponds to `(-4π~4π)`.<br>Byte2~3: target angular velocity `[0~65535]` corresponds to `(-15 rad/s~15 rad/s)`.<br>Byte4~5: Kp `[0~65535]` corresponds to `(0.0~5000.0)`.<br>Byte6~7: Kd `[0~65535]` corresponds to `(0.0~100.0)`.<br>After conversion, the high byte goes first and the low byte follows. |

**Response frame:** motor feedback frame (see Communication Type 2).

#### Communication Type 2: motor feedback data

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x2` | Bit8~Bit15: CAN ID of the current motor.<br>bit21~16: fault information (`0`=none, `1`=has fault): bit21 uncalibrated, bit20 gridlock/overload fault, bit19 magnetic coding fault, bit18 overtemperature, bit17 overcurrent, bit16 undervoltage fault.<br>bit22~23: Mode status — `0`: Reset [reset]; `1`: Cali mode [calibration]; `2`: Motor mode [Run] | host CAN_ID | Byte0~1: current angle `[0~65535]` corresponds to `(-4π~4π)`.<br>Byte2~3: current angular velocity `[0~65535]` corresponds to `(-15 rad/s~15 rad/s)`.<br>Byte4~5: current torque `[0~65535]` corresponds to `(-120 N·m~120 N·m)`.<br>Byte6~7: current temperature: Temp(Celsius) × 10. High byte first, low byte second. |

#### Communication Type 3: Motor enabled to run

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x3` | bit15~8: identifies the main CAN_ID | target motor CAN_ID | — |

**Response frame:** motor feedback frame (see Communication Type 2).

#### Communication Type 4: Motor stops running

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x4` | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | When the motor is running normally, 0 must be present in the data field. `Byte[0]=1`: the fault is cleared. |

**Response frame:** motor feedback frame (see Communication Type 2).

#### Communication type 6: Set motor mechanical zero

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x6` | bit15~8: identifies the main CAN_ID | target motor CAN_ID | `Byte[0]=1` |

**Response frame:** motor feedback frame (see Communication Type 2).

#### Communication type 7: Set motor CAN_ID

Changes the current motor CAN_ID, effective immediately.

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x7` | bit15~8: used to identify main CAN_ID; bit16~23: preset CAN_ID | target motor CAN_ID | — |

**Answer frame:** motor broadcast frame (see Communication type 0).

#### Communication type 17: Single parameter read

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x11` | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | Byte0~1: index — see the [read/write parameter list](#read-and-write-a-single-parameter-list) below.<br>Byte2~3: `00`.<br>Byte4~7: unused in the request (00). |

**Reply frame:**

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x11` | bit15~8: indicates the master CAN_ID; bit23~16: `00` | Byte0~1: index that the reply corresponds to | Byte2~3: `00`.<br>Byte4~7: parameter data, low byte first / high byte last. |

#### Communication type 18: Single parameter write (lost on power failure)

With type 22, the parameter starting at function code `0x20xx` in the upper-computer parameter table can be persisted.

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x12` | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | Byte0~1: index — see the [read/write parameter list](#read-and-write-a-single-parameter-list) below.<br>Byte2~3: `00`.<br>Byte4~7: parameter data, low byte first / high byte last. |

**Response frame:** motor feedback frame (see Communication type 2).

#### Communication type 21: Fault feedback frame

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x15` | bit15~8: motor CAN_ID | identifies the main CAN_ID | Byte0~3: fault value (non-0: faulty; 0: normal). bit14: gridlock/overload fault. bit7: encoder not calibrated. bit3: overvoltage fault. bit2: undervoltage fault. bit1: driver chip fault. bit0: motor overtemperature fault (default threshold 145℃).<br>Byte4~7: warning value. bit0: motor overtemperature warning (default 135℃). |

#### Communication type 22: Motor data save frame

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x16` | bit15~8: identifies the main CAN_ID | target motor CAN_ID | `01 02 03 04 05 06 07 08`* |

\* The source manual shows all 8 data bytes as the literal sequence `01 02 03 04 05 06 07 08` with no per-byte meaning documented — likely placeholder/example filler for "value not checked." Verify against firmware before relying on any specific byte here.

**Response frame:** motor feedback frame (see Communication type 2).

#### Communication type 23: Motor baud rate modification frame (re-power-on effect)

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x17` | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | `01 02 03 04 05 06 F_CMD` — bytes 0-5 unused/placeholder; `F_CMD` (byte 6) is the motor baud rate: `01`=1 M, `02`=500 K, `03`=250 K, `04`=125 K. |

**Response frame:** motor feedback frame (see Communication type 0).

#### Communication type 24: The motor actively reports frames

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x18` | bit15~8: identifies the main CAN_ID | target motor CAN_ID | `01 02 03 04 05 06 F_CMD` — bytes 0-5 unused/placeholder; `F_CMD` (byte 6) is the motor active-reporting switch: `00`=disable active reporting (default), `01`=enable active reporting (default reporting interval 10 ms). |

**Response frame:**

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x18` | (same fault/mode bit layout as Communication Type 2) | target motor CAN_ID | (same angle/velocity/torque/temperature layout as Communication Type 2) |

#### Communication type 25: Motor protocol modification frame (re-power-on effect)

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x19` | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | `01 02 03 04 05 06 F_CMD` — bytes 0-5 unused/placeholder; `F_CMD` (byte 6) is the motor protocol type: `0`=private protocol (default), `1`=CANopen protocol, `2`=MIT protocol. |

**Response frame:** motor feedback frame (see Communication type 0).

#### Communication type 26: Version number read frame

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x4`* | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | `Byte[0]=0x00`, `Byte[1]=0xC4` |

\* The source shows `0x4` here, which does not match the "Communication type 26" heading (expected `0x1A`, i.e. 26 decimal). This looks like a transcription/OCR error in the original manual — flagged for verification against firmware/CAN traces.

**Response frame:**

| Data field | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| Description | `0x2` | (same fault/mode bit layout as Communication Type 2) | target motor CAN_ID | `Byte0=0x00`, `Byte1=0xC4`, `Byte2=0x56`, `Byte3~6`: motor version number, ordered high to low |

### Read and Write a Single Parameter List

This is the index space used by Communication types 17 (single parameter read) and 18 (single parameter write):

| Index | Name | Description | Type | Bytes | Range / default | R/W |
| --- | --- | --- | --- | --- | --- | --- |
| 0X7005 | run_mode | `0`: operation mode; `1`: position mode (PP); `2`: velocity mode; `3`: operation mode (current mode); `5`: position mode (CSP) | uint8 | 1 | | W/R |
| 0X7006 | iq_ref | Current-mode Iq command | float | 4 | -90 to 90 A | W/R |
| 0X700A | spd_ref | Velocity-mode rotational speed command | float | 4 | -20 to 20 rad/s | W/R |
| 0X700B | limit_torque | Torque limit | float | 4 | 0 to 120 N·m | W/R |
| 0X7010 | cur_kp | Current Kp | float | 4 | default 0.17 | W/R |
| 0X7011 | cur_ki | Current Ki | float | 4 | default 0.012 | W/R |
| 0X7014 | cur_filt_gain | Current filter gain | float | 4 | 0 to 1.0, default 0.1 | W/R |
| 0X7016 | loc_ref | Position-mode angle instruction | float | 4 | rad | W/R |
| 0X7017 | limit_spd | Position mode (CSP) speed limit | float | 4 | 0 to 20 rad/s | W/R |
| 0X7018 | limit_cur | Velocity/position mode current limit | float | 4 | 0 to 90 A | W/R |
| 0x7019 | mechPos | Mechanical angle of the loading coil | float | 4 | rad | R |
| 0x701A | iqf | Iq filter | float | 4 | -90 to 90 A | R |
| 0x701B | mechVel | Speed of the load | float | 4 | -15 to 15 rad/s | R |
| 0x701C | VBUS | Bus voltage | float | 4 | V | R |
| 0x701E | loc_kp | Position-loop Kp | float | 4 | default 60 | W/R |
| 0x701F | spd_kp | Speed-loop Kp | float | 4 | default 6 | W/R |
| 0x7020 | spd_ki | Speed-loop Ki | float | 4 | default 0.02 | W/R |
| 0x7021 | spd_filt_gain | Speed filter value | float | 4 | default 0.1 | W/R |
| 0x7022 | acc_rad | Velocity-mode acceleration | float | 4 | default 15 rad/s² | W/R |
| 0x7024 | vel_max | Location mode (PP) speed | float | 4 | default 10 rad/s | W/R |
| 0x7025 | acc_set | Location mode (PP) acceleration | float | 4 | default 10 rad/s² | W/R |
| 0x7026 | EPScan_time | Report time. `1` = 10 ms; each additional `+1` increments by 5 ms | uint16 | 2 | default 1 | W |
| 0x7028 | canTimeout | CAN timeout threshold; 20000 = 1 s | uint32 | 4 | default 0 | W |
| 0x7029 | zero_sta | Zero flag bit: `0` means power-on range 0–2π; `1` means power-on range -π–π | uint8 | 1 | default 0 | W |

#### Read example

Take reading `loc_kp` (index `0x701E`) as an example.

**Read instruction:**

| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| | `0x11` (Type 17) | Host id `0xFD` | Target motor CAN_ID `0x7F` | `1E 70 00 00 00 00 00 00` — Byte0~1: index, corresponding to `loc_kp` (`0x701E`, low byte first) |

**Feedback instruction:**

| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| --- | --- | --- | --- | --- |
| | `0x11` (Type 17) | bit15~8: target motor CAN_ID `0x7F` | Host id `0xFD` | `1E 70 00 00 00 00 F0 41` — Byte0~1: index, corresponding to `loc_kp`.<br>Byte4~7: `loc_kp` value = 30, IEEE-754 single-precision float (`0x41F00000` = 30.0). |

### Motor Function Description

*(If the following features are unavailable, please upgrade to the latest version via the official Git repository.)*

**1. Active Reporting**
- Disabled by default. Enable via Type 24.
- Report type: Type 2 (default interval: 10 ms). Adjust the interval by modifying `EPScan_time` via Type 18.

**2. Zero-Point Flag (`zero_sta`)**
- Modify via:
  - Host computer
  - Type 18 (requires saving via Type 22 to persist)
- Default flag: `0` → Power-on position range: 0–2π.
- If set to `1`: Power-on position range: -π–π.

**3. Type 2 Update**
- Updated to periodic looping within -4π–4π (enables cycle counting).
- Note: Position interface parameters must be adjusted:
  - `P_MIN`: 12.57f
  - `P_MAX`: 12.57f

  (As printed in the source; `P_MIN` is presumably meant to be `-12.57f` by symmetry with the `P_MIN`/`P_MAX` definitions in the [program sample](#control-mode-instructions--program-sample) below — flagged for verification.)

**4. Protocol Switching (Requires CAN adapter)**
- Methods:
  - Modify `protocol_1` via host computer.
  - Send Type 25 command.
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
- Use case: bypass mechanical limits (e.g. set zero at 1 rad → power-on treats 1 rad as the new zero).

**8. CANopen ID**
- Old version: fixed to 1.
- New version: matches the private-protocol CAN ID.

**Notes for implementation:**
- Always save settings (e.g. Type 22 for `zero_sta`).
- Verify CAN adapter compatibility for protocol switching.
- For zero offsets, ensure mechanical safety limits are respected.

### Control Mode Instructions / Program Sample

Examples of controlling the motor in various modes are given below (using the gd32f303 MCU as an example). The following are the library, function, and macro definitions used by the examples:

```c
#define P_MIN -12.57f
#define P_MAX 12.57f

#define V_MIN -15.0f
#define V_MAX 15.0f

#define KP_MIN 0.0f
#define KP_MAX 5000.0f

#define KD_MIN 0.0f
#define KD_MAX 100.0f

#define T_MIN -120.0f
#define T_MAX 120.0f

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
#define rxCanIdEx (*((struct exCanIdInfo*)&(rxMsg.rx_efid))) // parses the extended frame id into the custom struct

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

**Motor Enabled Run frame (communication type 3)**

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

**Operation control mode Motor control instruction (communication type 1)**

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

**Motor stop frame (communication type 4)**

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

**Motor mode parameter write command (communication type 18, running-mode switch)**

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

**Motor mode parameter write command (communication type 18, control parameter write)**

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

#### Operation control mode

The motor is in operation control mode by default after power-on.

Send motor Enable Run frame (communication type 3) → Send operation mode motor control command (communication type 1) → Receive motor feedback frame (communication type 2).

The operation-mode control loop implements `t_ref = Kd*(v_set-v_actual) + Kp*(p_set-p_actual) + t_ff`, which is passed through torque protection (saturation) and converted to `iq_ref`/`id_ref` and fed into the FOC current loop (`id_ref` fixed at 0). See [Operation and control mode](#operation-and-control-mode) above for worked examples of the Kp/Kd/target combinations.

#### Current mode

Send motor mode parameter write command (communication type 18), set the `run_mode` parameter to 3 → Send motor Enable run frame (communication type 3) → Send motor mode parameter write command (communication type 18), set the `iq_ref` parameter to the desired current instruction.

#### Velocity mode

Control loop: expected speed and current speed feed a PI (`Kp`/`Ki` of the speed loop) whose output passes through torque protection to produce `iq_ref`, which (with `id_ref` = 0) feeds the FOC current loop.

Send motor mode parameter write command (communication type 18), set the `run_mode` parameter to 2 → Send motor Enable run frame (communication type 3) → Send motor mode parameter write command (communication type 18), set the `limit_cur` parameter as the maximum current instruction → Send motor mode parameter write command (communication type 18), set the `acc_rad` parameter as the acceleration instruction → Send motor mode parameter write command (communication type 18), set the `spd_ref` parameter as the speed instruction.

#### Location Mode (CSP)

Control loop: expected position and current position feed a P (`Kp` of the position loop), whose output is speed-limited, then combined with current speed through the speed-loop PI (`Kp`/`Ki`), through torque protection, into `iq_ref`/`id_ref` and the FOC current loop.

Send motor mode parameter write command (communication type 18), set the `run_mode` parameter to 5 → Send motor Enable run frame (communication type 3) → Send motor mode parameter write command (communication type 18), set the `limit_spd` parameter as the maximum speed instruction → Send motor mode parameter write command (communication type 18), set the `loc_ref` parameter as the position instruction.

#### Location Mode (PP)

Control loop: a trapezoidal (T-curve) planning algorithm turns Set Angle / Set Speed / Set Acceleration into a planned position and speed trajectory, which then drives the same position-Kp → speed-limit → speed-loop-PI → torque-protection → FOC chain as CSP mode.

Send motor mode parameter write command (communication type 18), set the `run_mode` parameter to 1 → Send motor Enable run frame (communication type 3) → Send motor mode parameter write command (communication type 18), set the `vel_max` parameter as the maximum speed instruction → Send motor mode parameter write command (communication type 18), set the `acc_set` parameter as the acceleration instruction → Send motor mode parameter write command (communication type 18), set the `loc_ref` parameter as the position instruction.

> **Note:** This mode does not support changing the speed and acceleration during operation. If you want to make an emergency stop, you can change `vel_max` to 0 during the process; the motor will stop following the current speed/acceleration plan.

#### Stop running

Send the motor stop frame (communication type 4).

---

## Explanation of CANopen Communication Protocol Types

### State Machine Description

State transitions (DS402-style state machine), with the `Controlword` (0x6040) values that trigger each transition marked on the arrows:

```
POWER DISABLED:
  START -> NOT_READY_TO_SWITCH_ON -> SWITCH_ON_DISABLED
POWER ENABLED (from SWITCH_ON_DISABLED):
  SWITCH_ON_DISABLED --[6]--> READY_TO_SWITCH_ON --[7]--> SWITCHED_ON --[15]--> OPERATION_ENABLE
  OPERATION_ENABLE --[1]--> SWITCH_ON_DISABLED
  OPERATION_ENABLE --[11]--> QUICK_STOP_ACTIVE --[15]--> OPERATION_ENABLE
FAULT:
  FAULT_REACTION_ACTIVE --[130]--> SWITCH_ON_DISABLED
```

**Motor Enable:** When initially powered on, the motor defaults to the `SWITCH_ON_DISABLED` state. To transition to `OPERATION_ENABLE`, modify the Controlword (6040H) to 6, 7, or 15 (step-by-step transition), or directly set it to 15 for immediate enablement.

**Stopping the Motor:** If the motor is in `OPERATION_ENABLE` state and needs to stop normally, modify the Controlword (6040H) to 1. The motor will return to the disabled state (`SWITCH_ON_DISABLED`).

**Emergency Stop (use with caution — risk of voltage surge):** During operation, an emergency stop can be triggered by setting the Controlword (6040H) to 11.

**Fault Clearance:** If the motor enters a `FAULT` state due to protection mechanisms, modifying the Controlword (6040H) can clear standard errors.

**Important Note:** Mode changes for this motor must be performed in the disabled state (`SWITCH_ON_DISABLED`). Ensure the desired mode is configured before enabling `OPERATION_ENABLE` to avoid unexpected behavior.

### Status Feedback Parameters

| Index | Name | Attribute | Type | Unit |
| --- | --- | --- | --- | --- |
| 603F | Error_code | Read-only | UINTEGER16 | / |
| 6041 | Statusword | Read-only | UINTEGER16 | / |
| 6061 | Modes_of_operation_display | Read-only | INTEGER8 | / |
| 6062 | Position_demand_value | Read-only | INTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 6064 | Position_actual_value | Read-only | INTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 606B | Velocity_demand_value | Read-only | INTEGER32 | 0.1 rpm |
| 606C | Velocity_actual_value | Read-only | INTEGER32 | 0.1 rpm |
| 6077 | Torque_actual_value | Read-only | INTEGER16 | 0.1% load ratio (1000 = 40 N·m) |
| 6078 | Current_actual_value | Read-only | INTEGER16 | mA |
| 6079 | DC_link_circuit_voltage | Read-only | INTEGER32 | mV |

### Homing Mode (Zero Position Setting)

| Index | Name | Attribute | Type | Unit |
| --- | --- | --- | --- | --- |
| 6040 | Controlword | Read-write | UINTEGER16 | / |
| 6060 | Modes of operation | Read-write | INTEGER8 | / |

**Homing method:**
- Set **Modes of operation** to 6 while the motor is in the disabled state (`SWITCH_ON_DISABLED`). The motor will then define the current position as the zero point.
- To hold the zero position, modify the Controlword to 15, and the motor will maintain its position at the home location.

### Position Mode (PP - Profile Position)

| Index | Name | Attribute | Type | Unit |
| --- | --- | --- | --- | --- |
| 6040 | Controlword | Read-write | UINTEGER16 | / |
| 6060 | Modes of operation | Read-write | INTEGER8 | / |
| 6067 | Position_window | Read-write | UINTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 6068 | Position_window_time | Read-write | UINTEGER16 | ms |
| 6071 | Target_torque | Read-write | INTEGER16 | 0.1% load ratio (1000 = 40 N·m) |
| 607A | Target_position | Read-write | INTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 6081 | Profile_velocity | Read-write | UINTEGER32 | 0.1 rpm |
| 6083 | Profile_acceleration | Read-write | UINTEGER32 | 0.1 rpm/s |

**Steps to configure Position Mode (PP):**

1. While the motor is in the disabled state (`SWITCH_ON_DISABLED`), set **Modes of operation** to 1.
   - Mandatory parameters: `Target_torque` (absolute max torque in position mode), `Profile_velocity` (absolute speed in position mode), `Profile_acceleration` (absolute acceleration in position mode).
   - Optional parameters: `Position_window` (if not set, window check is disabled), `Position_window_time` (if not set, window check is disabled).
2. Set **Controlword** (6040) to 15 to enable operation.
3. Set **Target_position** (absolute position) to move the motor to the desired position.

### Position Mode (CSP - Cyclic Synchronous Position)

| Index | Name | Attribute | Type | Unit |
| --- | --- | --- | --- | --- |
| 6040 | Controlword | Read-write | UINTEGER16 | / |
| 6060 | Modes of operation | Read-write | INTEGER8 | / |
| 6067 | Position_window | Read-write | UINTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 6068 | Position_window_time | Read-write | UINTEGER16 | ms |
| 6071 | Target_torque | Read-write | INTEGER16 | 0.1% load ratio (1000 = 40 N·m) |
| 607A | Target_position | Read-write | INTEGER32 | Pulses (1 rev = 16,384 pulses) |
| 6081 | Profile_velocity | Read-write | UINTEGER32 | 0.1 rpm |

**Steps to configure Position Mode (CSP):**

1. While the motor is in the disabled state (`SWITCH_ON_DISABLED`), set **Modes of operation** to 5.
   - Mandatory parameters: `Target_torque` (absolute max torque in position mode), `Profile_velocity` (absolute speed in position mode).
   - Optional parameters: `Position_window` (0 = disabled), `Position_window_time` (0 = disabled).
2. Set **Controlword** (6040) to 15 to enable operation.
3. Set **Target_position** (absolute position) to move the motor to the desired position.

### Velocity Mode

| Index | Name | Attribute | Type | Unit |
| --- | --- | --- | --- | --- |
| 6040 | Controlword | Read-write | UINTEGER16 | / |
| 6060 | Modes of operation | Read-write | INTEGER8 | / |
| 6071 | Target_torque | Read-write | INTEGER16 | 0.1% load ratio (1000 = 40 N·m) |
| 60FF | Target_velocity | Read-write | INTEGER32 | 0.1 rpm |

**Steps to configure Velocity Mode:**

1. While the motor is in the disabled state (`SWITCH_ON_DISABLED`), set **Modes of operation** to 3.
   - Mandatory parameter: `Target_torque` (absolute max torque in velocity mode).
2. Set **Controlword** (6040) to 15 to enable operation.
3. Set **Target_velocity** to reach the desired speed.

### Torque Mode

| Index | Name | Attribute | Type | Unit |
| --- | --- | --- | --- | --- |
| 6040 | Controlword | Read-write | UINTEGER16 | / |
| 6060 | Modes of operation | Read-write | INTEGER8 | / |
| 6071 | Target_torque | Read-write | INTEGER16 | 0.1% load ratio (1000 = 40 N·m) |

**Steps to configure Torque Mode:**

1. While the motor is in the disabled state (`SWITCH_ON_DISABLED`), set **Modes of operation** to 4.
2. Set **Controlword** (6040) to 15 to enable operation.
3. Set **Target_torque** to output the desired torque.

### Protocol Switching (Extended Frame): Switch Motor Protocol (Takes Effect After Power Cycle)

| Data Field | 29-bit ID | 8-Byte Data Area |
| --- | --- | --- |
| Size | Bit 28~0 | Byte 0~6 |
| Description | `0xFFF` | `01 02 03 04 05 06 F_CMD` |

- `F_CMD` (Byte 6) defines the motor protocol:
  - `0`: Private protocol (default)
  - `1`: CANopen protocol
  - `2`: MIT protocol

**Response Frame:**

| Data Field | 11-bit ID | 8-Byte Data Area |
| --- | --- | --- |
| Size | Bit 10~0 | Byte 0~7 |
| Description | Motor ID | 64-bit MCU unique identifier |

---

## MIT Communication Protocol Description

The motor communication adopts the CAN 2.0 interface with a default baud rate of 1 Mbps. The baud rate can be modified by switching to the private protocol. The standard frame format is as follows:

| Data Field | 11-bit ID: Bit 10~8 | 11-bit ID: Bit 7~0 | 8-byte Data Area: Byte 0~7 |
| --- | --- | --- | --- |
| Description | Mode type | ID | — |

**Supported Control Modes:**
- **MIT Mode:** Provides five motion control parameters to the motor.
- **Velocity Mode:** Specifies the target speed for the motor.
- **Position Mode:** Specifies the target position and speed, allowing the motor to run to the designated position at the configured speed.

### Response Command 1: Data Feedback (Motor Status)

| Data Field | 11-bit ID | 8-byte Data Area |
| --- | --- | --- |
| Size | Bit 10~0 | Byte 0~7 |
| Description | Host ID | **Byte 0:** Motor CAN ID.<br>**Byte 1~2:** current angle `[0~65535]`, corresponds to (-15 rad ~ 15 rad).<br>**Byte 3 (high 8 bits), Byte 4[7-4] (low 4 bits):** current speed `[0~4096]`, corresponds to (-33 rad/s ~ 33 rad/s).<br>**Byte 4[3-0] (high 4 bits), Byte 5 (low 8 bits):** current torque `[0~4096]`, corresponds to (-120 N·m ~ 120 N·m).<br>**Byte 6~7:** winding temperature (in degrees). |

### Response Command 2: MCU Identification

| Data Field | 11-bit ID | 8-byte Data Area |
| --- | --- | --- |
| Size | Bit 10~0 | Byte 0~7 |
| Description | Motor ID | 64-bit MCU unique identifier |

### Command 1: Enable Motor Operation

| Data Field | 11-bit ID | 8-byte Data Area |
| --- | --- | --- |
| Size | Bit 10~0 | Byte 0~7 |
| Description | Target motor CAN ID | `FF FF FF FF FF FF FF FC` |

**Response:** Response Command 1.

### Command 2: Stop Motor Operation

| Data Field | 11-bit ID | 8-byte Data Area |
| --- | --- | --- |
| Size | Bit 10~0 | Byte 0~7 |
| Description | Target motor CAN ID | `FF FF FF FF FF FF FF FD` |

**Response:** Response Command 1.

### Command 3: MIT Dynamic Parameters

| Data Field | 11-bit ID | 8-byte Data Area |
| --- | --- | --- |
| Size | Bit 10~0 | **Byte 0~1:** target angle `[0~65535]`, (-15 rad ~ 15 rad).<br>**Byte 2 (high 8 bits), Byte 3[7-4] (low 4 bits):** target speed `[0~4096]`, (-33 rad/s ~ 33 rad/s).<br>**Byte 3[3-0] (high 4 bits), Byte 4 (low 8 bits):** Kp `[0~4096]`, (0~500).<br>**Byte 5 (high 8 bits), Byte 6[7-4] (low 4 bits):** Kd `[0~4096]`, (0~5).<br>**Byte 6[3-0] (high 4 bits), Byte 7 (low 8 bits):** target torque `[0~4096]`, (-120 N·m ~ 120 N·m). |

**Response:** Response Command 1.

### Command 4: Set Zero Position (Non-Position Mode)

| Data Field | 11-bit ID | 8-byte Data Area |
| --- | --- | --- |
| Size | Bit 10~0 | Byte 0~7 |
| Description | Target motor CAN ID | `FF FF FF FF FF FF FF FE` |

**Response:** Response Command 1.

### Command 5: Clear Errors & Read Fault Status

| Data Field | 11-bit ID | 8-byte Data Area |
| --- | --- | --- |
| Size | Bit 10~0 | `FF FF FF FF FF FF F_CMD FB`.<br>**F_CMD:** `0xFF` → clear current fault; any other value → returns the fault value in **Byte 1** of the response. |

**Response (Fault Clear):** Response Command 1.

**Fault Status Response:**

| Data Field | 11-bit ID | 8-byte Data Area |
| --- | --- | --- |
| Size | Bit 10~0 | **Byte 0:** Motor CAN ID.<br>**Byte 1~4:** fault value (non-zero: fault present; 0: normal). Bit 14: stall / I²t overload fault.<br>Bit 7: encoder not calibrated.<br>Bit 3: overvoltage fault.<br>Bit 2: undervoltage fault.<br>Bit 1: driver IC fault.<br>Bit 0: motor overtemperature fault (default threshold 145℃). |

### Command 6: Set Operation Mode

| Data Field | 11-bit ID | 8-byte Data Area |
| --- | --- | --- |
| Size | Bit 10~0 | `FF FF FF FF FF FF F_CMD FC`.<br>**F_CMD:** mode type — `0`: MIT mode (default); `1`: Position mode; `2`: Velocity mode. |

**Response:** Response Command 1.

### Command 7: Modify Motor CAN ID

| Data Field | 11-bit ID | 8-byte Data Area |
| --- | --- | --- |
| Size | Bit 10~0 | `FF FF FF FF FF FF F_CMD FA`.<br>**F_CMD:** target motor CAN ID. |

**Response:** Response Command 2.

### Command 8: Change Communication Protocol (Takes Effect After Power Cycle)

| Data Field | 11-bit ID | 8-byte Data Area |
| --- | --- | --- |
| Size | Bit 10~0 | `FF FF FF FF FF FF F_CMD FD`.<br>**F_CMD:** protocol type — `0`: private protocol (default); `1`: CANopen; `2`: MIT protocol. |

**Response:** Response Command 2.

### Command 9: Modify Host CAN ID

| Data Field | 11-bit ID | 8-byte Data Area |
| --- | --- | --- |
| Size | Bit 10~0 | `FF FF FF FF FF FF F_CMD 01`.<br>**F_CMD:** host CAN ID. |

**Response:** Response Command 2.

### Command 10: Position Mode Control Command

| Data Field | 11-bit ID: Bit 10~8 | 11-bit ID: Bit 7~0 | 8-byte Data Area |
| --- | --- | --- | --- |
| Size / Description | `1` | Target motor CAN ID | **Byte 0~3:** target position (rad, 32-bit float).<br>**Byte 4~7:** target speed (rad/s, 32-bit float). |

**Response:** Response Command 1.

### Command 11: Velocity Mode Control Command

| Data Field | 11-bit ID: Bit 10~8 | 11-bit ID: Bit 7~0 | 8-byte Data Area |
| --- | --- | --- | --- |
| Size / Description | `2` | Target motor CAN ID | **Byte 0~3:** target speed (rad/s, 32-bit float).<br>**Byte 4~7:** current limit in speed/position mode (A, 32-bit float). |

**Response:** Response Command 1.

### Motion Control Mode

Control loop: expected angle vs. current angle through `Kp`, plus expected speed vs. current speed through `Kd`, are summed with expected torque to produce the final torque `t_ref`; `t_ref` is passed through `f(t_ref)` to produce the expected `iq` (with expected `id` fixed at 0) for the FOC current loop.

The motor defaults to Motion Control Mode upon power-up.

1. Send the Motor Enable Command (Command 1).
2. Send the Motion Control Command (Command 3) to activate dynamic parameter control.
3. Send the Motor Stop Command (Command 2) to halt operation when needed.

### Velocity Mode

Control loop: expected speed vs. current speed through the speed loop's `Kp`/`Ki`, through torque protection, into `iq_ref`/`id_ref` and the FOC current loop (same structure as the private-protocol velocity mode).

1. Configure the motor's operation mode by sending the Set Operation Mode Command (Command 6) with Mode = 2 (Velocity Mode).
2. Send the Motor Enable Command (Command 1) to activate the motor.
3. Send the Velocity Mode Control Command (Command 11) to set the maximum current (absolute value) and target speed.
4. To stop, send the Motor Stop Command (Command 2).

### Position Mode (CSP - Cyclic Synchronous Position)

Control loop: expected position vs. current position through the position loop's `Kp`, speed-limited, combined with current speed through the speed loop's `Kp`/`Ki`, through torque protection, into `iq_ref`/`id_ref` and the FOC current loop (same structure as the private-protocol CSP mode).

1. Configure the motor's operation mode by sending the Set Operation Mode Command (Command 6) with Mode = 1 (Position Mode).
2. Send the Motor Enable Command (Command 1) to activate the motor.
3. Send the Position Mode Control Command (Command 10) to set the maximum speed (absolute value) and target position.
4. To stop, send the Motor Stop Command (Command 2).
