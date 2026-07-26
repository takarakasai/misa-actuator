# misa-actuator

Common actuator-control interface for multiple servo-motor families on
multiple bus transports, plus a debug TUI.

## Workspace layout

```
crates/
├── misa-actuator/          # common Actuator trait, types, unified Error
├── misa-actuator-tui/      # ratatui-based debug TUI (driver-agnostic)
├── misa-sysid/             # chirp system-identification + FRF tooling
├── lkmotor-protocol/       # LK Motor V3 frame codec (no_std)
├── lkmotor-driver/         # LK Motor driver (RS485), impl Actuator
├── lkmotor-cli/            # CLI test app for LK Motor
├── robstride-protocol/     # Robstride frame codec (no_std)
├── robstride-driver/       # Robstride driver (SocketCAN), impl Actuator
├── robstride-cli/          # CLI test app for Robstride
├── damiao-protocol/        # DAMIAO frame codec (no_std)
├── damiao-driver/          # DAMIAO driver (CAN / CAN-FD), impl Actuator
├── damiao-cli/             # CLI test app for DAMIAO
├── myactuator-protocol/    # MyActuator RMD CAN-V3 frame codec (no_std)
├── myactuator-driver/      # MyActuator RMD driver (SocketCAN), impl Actuator
└── myactuator-cli/         # CLI test app for MyActuator RMD
```

## Supported motors

| family | models | transport | notes |
|--------|--------|-----------|-------|
| Robstride | RS-00…RS-06 (`--model rs04`, `robstride04`, `Edulite05`, …) | SocketCAN | per-model MIT scales |
| LK Motor (上海凌控) V3 | MG4005 etc. | RS485 | current-based torque (`--kt`), soft zero |
| DAMIAO | DM-J4310, DM-J3507 | SocketCAN classic / CAN-FD | register-based run modes |
| MyActuator RMD (CAN V3) | RMD-X / RMD-L V3 firmware | SocketCAN | output-shaft wire units, native motion (MIT) mode |

See [`crates/robstride-protocol/doc/communication-protocols.md`](crates/robstride-protocol/doc/communication-protocols.md)
for a full rundown of RobStride's wire protocols (classic private protocol vs. the undocumented
bulk/single parameter-table reads used to fetch firmware version etc.).

## Architecture

`misa_actuator::Actuator` is the SI-unit motor-control trait that
applications speak. Each motor family has its own internal **bus** trait
(e.g. `LkBus`, `RobstrideBus`, `DamiaoBus`, `MyActuatorBus`) so that the
same motor protocol can run on RS485 / SocketCAN / EtherCAT / etc. Driver
structs are generic over the bus, and the `Actuator` impl is
bus-independent.

```
        application / TUI  ─►  dyn Actuator
                                   ▲
        ┌──────────────────┬───────┴───────┬───────────────────┐
        │                  │               │                   │
   LkMotor<B>       RobstrideMotor<B>  DamiaoMotor<B>   MyActuatorMotor<B>
        │                  │               │                   │
    LkBus trait      RobstrideBus     DamiaoBus trait   MyActuatorBus trait
        │                  │               │                   │
  (RS485 / CAN / ...) (SocketCAN)   (CAN / CAN-FD)       (SocketCAN)
```

## Quick start (TUI)

```text
# Robstride RS-04 on can0
misa-actuator-tui --driver robstride --interface can0 --motor-id 1 --model rs04

# MyActuator RMD (CAN V3) on can0, Kt 0.83 N·m/A (0 → current-units mode)
misa-actuator-tui --driver myactuator --interface can0 --motor-id 1 --kt 0.83

# DAMIAO DM-J4310 on CAN-FD
misa-actuator-tui --driver damiao --interface can0 --bus can-fd --motor-id 1 --model DM4310

# LK Motor MG4005 on RS485
misa-actuator-tui --driver lkmotor --interface /dev/ttyUSB0 --motor-id 1 --baud 1000000 --gear-ratio 10.0
```
