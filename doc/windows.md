# Running misa-actuator on Windows

Everything in this workspace builds and runs on Windows: the four CLIs, the
TUI, the monitor and the identify tool. The only thing that genuinely differs
from Linux is **how CAN frames reach the wire** — Windows has no kernel CAN
stack, so there is no `can0` and no `ip link`. A vendor adapter API stands in
for it.

| | Linux | Windows |
|---|---|---|
| CAN transport | SocketCAN (`can0`) | PEAK PCAN-Basic (`pcan:usb1`) or SLCAN (`slcan:COM5`) |
| Bring-up | `ip link set can0 ... up` | none — the bitrate is part of `--interface` |
| RS485 (LK Motor) | `/dev/ttyUSB0` | `COM5` |
| Loop timing | accurate by default | needs the 1 ms timer the CLIs already request |

## 1. Toolchain setup

Not installed on this machine yet — none of the commands below have been run
for you.

**Rust (MSVC toolchain, recommended)**

1. Install the Microsoft C++ build tools, which provide the linker Rust needs:

   ```powershell
   winget install --id Microsoft.VisualStudio.2022.BuildTools `
     --override "--quiet --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
   ```

   Or download "Build Tools for Visual Studio" manually and tick the
   **Desktop development with C++** workload.

2. Install rustup:

   ```powershell
   winget install --id Rustlang.Rustup
   ```

3. Open a **new** terminal (so `%USERPROFILE%\.cargo\bin` is on `PATH`) and
   confirm:

   ```powershell
   rustc --version
   cargo --version
   rustup show          # host should read x86_64-pc-windows-msvc
   ```

**Build**

```powershell
cargo build --release
cargo test                       # unit tests need no hardware
```

The binaries land in `target\release\`:
`robstride-cli.exe`, `myactuator-cli.exe`, `damiao-cli.exe`, `lkmotor-cli.exe`,
`misa-actuator-tui.exe`, `misa-actuator-monitor.exe`, `misa-actuator-identify.exe`.

> The GNU toolchain (`x86_64-pc-windows-gnu`) also works if you prefer it —
> nothing here depends on MSVC specifically.

## 2. Picking a CAN adapter

### PEAK PCAN-Basic — recommended

Install the **PEAK-System device driver** for your adapter (PCAN-USB,
PCAN-USB FD, PCAN-USB Pro FD, ...). It ships `PCANBasic.dll` into `System32`,
which is all the binaries need — the SDK is *not* required to build, because
the DLL is resolved at run time.

```powershell
robstride-cli.exe -i pcan:usb1 scan
myactuator-cli.exe -i pcan:usb1 -m 1 status
damiao-cli.exe -i pcan:usb1@1M,5M --fd -m 1 status
```

Channel names: `pcan:usb1` … `pcan:usb16`, `pcan:pci1` …, `pcan:lan1` …
PEAK's own spelling (`PCAN_USBBUS1`) and a bare index (`pcan:1`) both work.

Classic CAN accepts the fixed PCAN bitrate table (1M, 800K, 500K, 250K, 125K,
100K, 95K, 83K, 50K, 47K, 33K, 20K, 10K, 5K). CAN-FD timings are derived for
the adapter's 80 MHz clock at an 80 % sample point.

### SLCAN (CANable, candleLight, USBtin, ...) — cheap fallback

The adapter appears as a virtual COM port and needs no vendor DLL, only its
USB-serial driver.

```powershell
lkmotor-cli.exe ports                   # lists every COM port with USB ids
robstride-cli.exe -i slcan:COM5 scan
robstride-cli.exe -i slcan:COM5@500K -m 1 status
```

Two limits worth knowing before you commit to it:

- **Classic CAN only.** The Lawicel protocol has no FD framing, so
  `damiao-cli --fd` is rejected rather than silently downgraded.
- **Throughput.** Every 8-byte frame costs 22 ASCII bytes each way over USB
  serial. Fine for `scan`, `status` and the quasi-static characterizations;
  marginal for a high-rate `chirp`. Use PEAK when the loop rate matters.

If the adapter's USB link runs faster than the 115200 baud default:

```powershell
robstride-cli.exe -i "slcan:COM5@1M?serial-baud=2000000" scan
```

> Quote the interface in PowerShell when it contains `?` or `&`.

COM10 and above are handled for you — both the CAN and RS485 paths rewrite
them to the `\\.\COM12` form the OS requires.

## 3. Interface string reference

One `--interface` / `-i` grammar covers all transports:

```
[<backend>:]<device>[@<bitrate>[,<data-bitrate>]][?<option>=<value>]
```

| spec | meaning |
|------|---------|
| `can0` | SocketCAN (Linux only) |
| `pcan:usb1` | first PEAK USB channel, 1 Mbit/s |
| `pcan:usb1@500K` | …at 500 kbit/s |
| `pcan:usb2@1M,5M` | second channel, CAN-FD 1 Mbit/s arbitration / 5 Mbit/s data |
| `slcan:COM5` | SLCAN adapter on COM5, 1 Mbit/s |
| `slcan:/dev/ttyACM0` | the same adapter on Linux |
| `slcan:COM5@1M?serial-baud=2000000` | faster host link |

Bitrates accept `1M`, `500K` or a plain `500000`. The default is 1 Mbit/s,
which is what every motor family here ships with.

Omitting the backend works when the name is unambiguous: `COM5` → SLCAN,
`PCAN_USBBUS1` → PCAN, and on Linux anything else → SocketCAN.

## 4. RS485 (LK Motor)

No transport change at all — only the port name.

```powershell
lkmotor-cli.exe ports
lkmotor-cli.exe -i COM5 --baud 1000000 -m 1 scan
lkmotor-cli.exe -i COM5 --baud 1000000 -m 1 --gear-ratio 10 status
```

If `ports` lists nothing, the USB-RS485 adapter's chipset driver (CH340,
FTDI or CP210x) is missing — check Device Manager → **Ports (COM & LPT)**.

## 5. Loop timing

Windows runs a ~15.6 ms scheduler tick by default, so an unmodified
`sleep(1ms)` sleeps ~15 ms and a 1 kHz control loop silently degrades to
~64 Hz. Every binary here therefore holds a
`misa_actuator::realtime::TimerResolutionGuard` for its whole run, which
requests a 1 ms tick, and the periodic loops sleep through
`realtime::sleep_until` / `sleep_precise`, which busy-wait the last ~1.5 ms so
the wake-up lands on time.

What that buys, and what it does not:

- Loop rates up to roughly 1 kHz hold their period, with jitter in the tens of
  microseconds rather than 15 ms.
- The busy-wait tail costs CPU. A 1 kHz loop keeps one core meaningfully
  occupied; that is the trade for timing accuracy.
- Windows is not a real-time OS. A scheduling hiccup can still stretch one
  iteration. `ChirpLog::achieved_rate_hz` reports what the run actually
  managed, and the logs carry real timestamps, so the identification stays
  valid even when the rate slips.

For the highest-rate identification work, Linux with SocketCAN remains the
better bench.

## 6. Hardware check

`scripts\hw_check.ps1` is the PowerShell port of `scripts/hw_check.sh` — the
same read-only scan/status sweep across RobStride and MyActuator, with an
opt-in motion phase.

Run it through the `.cmd` wrapper:

```powershell
.\scripts\hw_check.cmd -Interface pcan:usb1
.\scripts\hw_check.cmd -RsInterface pcan:usb1 -MyaInterface pcan:usb2
.\scripts\hw_check.cmd -Interface pcan:usb1 -Motion       # shafts must be free
```

The wrapper exists because PowerShell's execution policy defaults to
`Restricted`, which refuses to load *any* `.ps1` — including this repo's. The
wrapper passes `-ExecutionPolicy Bypass` for that one invocation and changes
nothing machine-wide. The same policy is why Node tooling has to be invoked as
`npm.cmd` rather than `npm`. To be rid of both:

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

There is no `--setup` equivalent: on Windows the bitrate travels in the
interface string, so there is nothing to bring up first.

## 7. Troubleshooting

**`PCANBasic.dll not found`** — the PEAK device driver is not installed, or a
32-bit binary is looking for the 64-bit DLL. Install the driver and use a
64-bit build.

**`CAN_Initialize failed … PCAN_ERROR_NETINUSE`** — PCAN-View or another
process holds the channel. Close it.

**Scan finds nothing, no error** — almost always bitrate or termination.
Confirm the motor's rate (all four families default to 1 Mbit/s) and that the
bus has 120 Ω at both ends. `pcan:usb1@500K` is a one-flag way to test the
other likely rate.

**`bus-off`** — wiring or a bitrate mismatch; every node on the wire has to
agree. Unplug and replug the adapter to clear the controller.

**SLCAN: `adapter did not answer command "S8"`** — the port opened but nothing
speaks Lawicel there. Either it is the wrong COM port, or the adapter is
running candleLight/gs_usb firmware instead of slcan firmware.

**Everything is slow / the chirp rate is far below target** — check the
warning about the timer resolution in the log output (`RUST_LOG=warn`, the
default). If it appears, `winmm.dll` could not be loaded, which is unusual and
worth investigating before trusting any timing-sensitive result.
