# misa-actuator

Common actuator-control interface for multiple servo-motor families on
multiple bus transports, plus a debug TUI and a desktop GUI. Runs on Linux and
Windows — see [`doc/windows.md`](doc/windows.md) for the Windows setup and for
building the GUI.

**Picking this up after a break, or for the first time?** Start with
[`doc/handover.md`](doc/handover.md). It carries the things the code cannot
tell you: what the motors actually reported on a bench, why several decisions
went the way they did, and the traps that have already cost time once.

## Workspace layout

```
crates/
├── misa-actuator/          # common Actuator trait, types, unified Error
├── misa-actuator-core/     # session: I/O worker thread, setpoints, watchdog
├── misa-actuator-gui/      # Tauri desktop GUI (front end in ../ui)
├── misa-actuator-sim/      # simulated Actuator, calibrated from bench data
├── misa-actuator-tui/      # ratatui-based debug TUI (driver-agnostic)
├── misa-can/               # CAN transport: SocketCAN / PCAN-Basic / SLCAN
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
| Robstride | RS-00…RS-06 (`--model rs04`, `robstride04`, `Edulite05`, …) | CAN | per-model MIT scales |
| LK Motor (上海凌控) V3 | MG4005 etc. | RS485 | current-based torque (`--kt`), soft zero |
| DAMIAO | DM-J4310, DM-J3507 | CAN classic / CAN-FD | register-based run modes |
| MyActuator RMD (CAN V3) | RMD-X / RMD-L V3 firmware | CAN | output-shaft wire units, native motion (MIT) mode |

"CAN" above means any transport `misa-can` supports, selected by the
`--interface` string:

| backend | platform | CAN-FD | interface |
|---------|----------|--------|-----------|
| SocketCAN | Linux | yes | `can0` |
| PCAN-Basic (PEAK) | Windows | yes | `pcan:usb1`, `pcan:usb2@1M,5M` |
| SLCAN (CANable, USBtin, …) | Linux + Windows | no | `slcan:COM5`, `slcan:/dev/ttyACM0` |
| USB-CAN Analyzer (CH340) | Linux + Windows | no | `usbcan:COM1` |

`robstride-cli interfaces` (also `damiao-cli`, `myactuator-cli`) lists what is
attached without opening anything; the GUI offers the same list as a dropdown.
Entries marked `?` are serial ports whose protocol is a guess — see below.

**Which protocol a cheap dongle speaks is not visible from the outside.** SLCAN
is the Lawicel ASCII protocol; "USB-CAN" on the box does not imply it. The
common *USB-CAN Analyzer* units exchange 20-byte binary packets instead — those
have their own backend (`usbcan:`), which is **never inferred from a bare
`COM*`**, so name it explicitly. RobStride's own module uses a third protocol
(`41 54` … `0D 0A`) and has no backend here. Quick way to tell an slcan adapter:
send `V` + CR to the port and look for an ASCII reply.

`usbcan:` sets the bitrate itself (5 kbit/s … 1 Mbit/s) and can monitor silently,
but reports **no receive overruns and no hardware timestamps** — prefer PEAK for
diagnostic work, where those two are the whole point.

RS485 needs no backend selection — pass the port name (`/dev/ttyUSB0`,
`COM5`); `lkmotor-cli ports` lists what is attached.

See [`crates/robstride-protocol/doc/communication-protocols.md`](crates/robstride-protocol/doc/communication-protocols.md)
for a full rundown of RobStride's wire protocols (classic private protocol vs. the undocumented
bulk/single parameter-table reads used to fetch firmware version etc.).

## Architecture

The summary below is the layer view. For the full picture — a process/thread
diagram, the channel design between the UI and the motor worker, and every loop
period in one table — see [`doc/architecture.md`](doc/architecture.md).

`misa_actuator::Actuator` is the SI-unit motor-control trait that
applications speak. Each motor family has its own internal **bus** trait
(e.g. `LkBus`, `RobstrideBus`, `DamiaoBus`, `MyActuatorBus`) so that the
same motor protocol can run on RS485 / CAN / EtherCAT / etc. Driver
structs are generic over the bus, and the `Actuator` impl is
bus-independent.

The three CAN families share one transport layer, `misa-can`, so a new
adapter is implemented once rather than per family.

```
        application / TUI  ─►  dyn Actuator
                                   ▲
        ┌──────────────────┬───────┴───────┬───────────────────┐
        │                  │               │                   │
   LkMotor<B>       RobstrideMotor<B>  DamiaoMotor<B>   MyActuatorMotor<B>
        │                  │               │                   │
    LkBus trait      RobstrideBus     DamiaoBus trait   MyActuatorBus trait
        │                  └───────────────┼───────────────────┘
        │                                  ▼
        │                        misa_can::CanBus trait
        │                                  │
        │                ┌─────────────────┼─────────────────┐
   (RS485 serial)   SocketCAN          PCAN-Basic          SLCAN
                     (Linux)            (Windows)      (serial, any OS)
```

## Quick start (TUI)

```text
# Robstride RS-04 on can0
misa-actuator-tui --driver robstride --interface can0 --motor-id 1 --model rs04

# ...the same motor on Windows, through a PEAK adapter
misa-actuator-tui --driver robstride --interface pcan:usb1 --motor-id 1 --model rs04

# MyActuator RMD (CAN V3) on can0, Kt 0.83 N·m/A (0 → current-units mode)
misa-actuator-tui --driver myactuator --interface can0 --motor-id 1 --kt 0.83

# DAMIAO DM-J4310 on CAN-FD
misa-actuator-tui --driver damiao --interface can0 --bus can-fd --motor-id 1 --model DM4310

# LK Motor MG4005 on RS485 (--interface COM5 on Windows)
misa-actuator-tui --driver lkmotor --interface /dev/ttyUSB0 --motor-id 1 --baud 1000000 --gear-ratio 10.0
```

## GUI

A Tauri desktop app: Rust backend (the same drivers everything else uses) with
a TypeScript front end.

```powershell
cd ui
npm ci
npm run app            # starts Vite + the app, with hot reload
```

That is the command to use day to day. `npm run app` runs `tauri dev`, which
starts the Vite dev server and then `cargo run` — both halves, in the right
order.

**`cargo` cannot build a runnable GUI on its own — not even in release.** The
front end is embedded by the *Tauri CLI*, not by `cargo`, so a binary from
`cargo build`/`cargo run` contains no front end at all and falls back to the dev
URL (`http://localhost:5173`). Without Vite running, the window opens on
`ERR_CONNECTION_REFUSED`; with Vite running you get the dev server rather than
the build you thought you were testing.

Verified 2026-08-05 by comparing the two binaries: the Tauri-built one contains
the `assets/index-*.js` name, the `cargo build --release` one does not.

So there are exactly two ways to run it, both through the CLI:

```powershell
cd ui
npm run app            # development: Vite + the app, hot reload
npm run app:build      # distributable: exe + installer → target/release/bundle/nsis/
```

For a release binary without waiting for the installer, skip the bundling step:

```powershell
cd crates\misa-actuator-gui
..\..\ui\node_modules\.bin\tauri build --no-bundle
```

(The CLI lives in `ui/node_modules`, which is why `npm run app:build` does its
work from `ui/` — a bare `npx tauri` from the crate directory will not find it.)

Every other crate needs nothing but a Rust toolchain — the workspace's
`default-members` excludes the GUI, so a plain `cargo build` / `cargo test`
still works without Node installed.

The GUI opens a motor exactly the way the CLIs do, including `--driver sim`, so
it is fully usable with no hardware attached.

**Safety.** Three independent mechanisms, because a UI is one more thing that
can stop working while a motor is energised:

- **STOP** (button, or `Esc` from anywhere) sets a flag the worker checks
  before every bus transaction. It is not a queued command, so it cannot end
  up behind a backlog.
- A **watchdog** disables the motor if the window stops polling while
  streaming. That covers a hung renderer, where by definition nobody can press
  stop. Minimising the window while streaming will trip it — deliberately.
- **Closing the window** stops the motor and waits for the worker to confirm
  before the process exits.

## Developing without hardware

`misa-actuator-sim` is a simulated `Actuator` — motor, load, friction and
thermal model — that every tool in the workspace accepts:

```powershell
misa-actuator-tui --driver sim --model rs04 --motor-id 1
misa-actuator-identify --interface sim --vendors sim --from 1 --to 12
```

The `dm4310` and `rs04` presets are calibrated from
[`doc/bench-measurements-2026-07-30.md`](doc/bench-measurements-2026-07-30.md)
— torque constants, inertia, static and kinetic friction, heating rate. That
makes it more than a stub: a characterization run against the simulator has a
*known right answer*, which no real motor can offer. `misa-actuator-sim`'s
integration tests assert that `misa-sysid` recovers the planted values.

## Platform notes

Linux is the reference platform. Windows is fully supported with two
differences worth knowing up front:

- **No kernel CAN stack.** Pick a PEAK adapter (`pcan:usb1`, CAN-FD capable)
  or an SLCAN dongle (`slcan:COM5`, classic CAN only, lower throughput).
- **Coarse default timer.** Windows rounds sleeps up to ~15.6 ms, so the
  binaries request a 1 ms tick and busy-wait the last fraction of a
  millisecond in periodic loops. Rates up to ~1 kHz hold; the highest-rate
  identification runs are still better done on Linux.

Full setup, interface-string reference and troubleshooting:
[`doc/windows.md`](doc/windows.md).
