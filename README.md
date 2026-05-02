# misa-actuator

Common actuator-control interface for multiple servo-motor families on
multiple bus transports, plus a debug TUI.

## Workspace layout

```
crates/
├── misa-actuator/          # common Actuator trait, types, unified Error
├── misa-actuator-tui/      # ratatui-based debug TUI (driver-agnostic)
├── lkmotor-protocol/       # LK Motor V3 frame codec (no_std)
├── lkmotor-driver/         # LK Motor driver, impl Actuator
├── robstride-protocol/     # Robstride frame codec (no_std)
├── robstride-driver/       # Robstride driver, impl Actuator
└── robstride-cli/          # CLI test app for Robstride
```

## Architecture

`misa_actuator::Actuator` is the SI-unit motor-control trait that
applications speak. Each motor family has its own internal **bus** trait
(e.g. `LkBus`, `RobstrideBus`) so that the same motor protocol can run on
RS485 / SocketCAN / EtherCAT / etc. Driver structs are generic over the
bus, and the `Actuator` impl is bus-independent.

```
        application / TUI  ─►  dyn Actuator
                                   ▲
                ┌──────────────────┼──────────────────┐
                │                  │                  │
           LkMotor<B>        RobstrideMotor<B>   MyActuatorMotor<B>
                │                  │                  │
            LkBus trait       RobstrideBus trait   MyActBus trait
                │                  │                  │
       (RS485 / CAN / ...)  (SocketCAN / USB-CAN)  (EtherCAT / CAN / ...)
```
