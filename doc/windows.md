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

### Finding out what is attached

```powershell
robstride-cli.exe interfaces        # same in damiao-cli and myactuator-cli
```

Opens nothing and puts no frame on any wire. PEAK channels come back named
(`pcan:usb1  PCAN-USB Pro FD`); serial ports are marked `?` and listed with every
protocol that could be behind them, because nothing about a port name says which
one is. The GUI shows the same list as a dropdown next to `INTERFACE`, with `↻` to
look again after plugging something in — and keeps a text field, since a device
the driver does not report still needs opening.

If a PEAK adapter is attached and missing from the list, the log says why: the
enumeration compares the driver's channel count against how many it could name and
warns when those disagree.

### SLCAN (CANable, candleLight, USBtin, ...) — cheap fallback

Verified 2026-08-06 against an RS-03 with a **WeAct Studio USB2CAN**
(`USB 0483:5740`, an STM32 CDC device): `scan` and `identify` both work, and return
byte-for-byte what the PEAK adapter and the USB-CAN Analyzer return.

Its answer to the `V` probe is a product banner rather than the four hex digits the
protocol describes:

```
WeAct Studio V1.0.0.5_f655d0fb\r
```

Which is fine — `bring_up` treats anything that is not a bare CR or a BEL as
unsolicited and keeps waiting for the verdict. Worth knowing before concluding an
adapter is not slcan because its version string looks wrong.

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

### USB-CAN Analyzer (CH340) — a different protocol on similar-looking hardware

The dongle labelled *USB-CAN Analyzer* is **not** an SLCAN adapter. It answers
nothing at all to the Lawicel commands — verified 2026-08-05 with `V`+CR at
1228800 / 2000000 / 115200 / 38400 / 9600 baud, silent at every one — and speaks
its own binary protocol instead. It has its own backend:

```powershell
lkmotor-cli.exe ports                   # find the CH340's COM number
robstride-cli.exe -i usbcan:COM1 scan
```

**Name it explicitly.** A bare `COM1` still means `slcan:`, because nothing in a
port name distinguishes the two protocols and changing that inference would
break working command lines.

Bitrate and mode are set by the backend, from the adapter's settings command:
**5 kbit/s … 1 Mbit/s** in twelve steps, and a silent mode, so `--listen-only`
style monitoring works here as well as on PEAK.

What it cannot do:

- **No overrun count, no hardware timestamps.** The protocol carries neither, so
  `rx_overruns()` stays 0 and there are no adapter timestamps. Those two are what
  settled the DAMIAO CAN-FD investigation (`handover.md` §4) — for diagnostic
  work this adapter is a step backwards.
- **Classic CAN only.** No FD framing exists in the protocol.

Verified against an RS-04 on 2026-08-05: `scan` and `identify` return the same
answers as the PEAK adapter does, extended ids and all.

**If nothing comes back, test the adapter first:**

```powershell
robstride-cli.exe interfaces --selftest usbcan:COM1
```

That puts it in loopback, where a frame comes straight back and the bus is not
involved. A pass means the adapter, the serial link, the settings command and the
packet format are all fine, so the fault is on the CAN side — wiring, termination,
bitrate, or no motor at the ids being probed. A fail means stop looking at the
wiring and look at `?serial-baud=` / `?stop-bits=`.

Worth having because **this protocol acknowledges nothing, not even its settings
command**: a loose wire, the wrong serial framing, a dead unit and an empty id
range all produce the same silence. Two evenings went into that — one on a loose
connection that also made the *PEAK* adapter lose the motor twice, one on a motor
sitting at id 96 while the scan looked at 1..32.

**Serial framing: 2 Mbaud, 8 data bits, 2 stop bits** — the reference's values, and
the ones that work. The vendor's note says 1,228,800 and one stop bit, which does
not:

```powershell
robstride-cli.exe -i "usbcan:COM1?serial-baud=1228800&stop-bits=1" scan
```

> Quote the interface in PowerShell when it contains `?` or `&`.

One correctness note, because it matters for RobStride: a data frame is
`DLC + 5` bytes for an 11-bit id and `DLC + 7` for a 29-bit one. The reference
implementation uses `DLC + 5` for both, which declares an extended frame complete
two bytes early and desyncs the stream. Our decoder counts per frame type and
checks the trailing `0x55`, so a length mistake becomes a skipped packet instead
of a mangled reading.

**The adapter cannot transmit for ~100 ms after its settings command.** It
re-initialises its CAN controller and drops whatever arrives meanwhile, so the
backend waits before returning from open. Without that wait the first one or two
frames vanish — probing four ids put only the last two on the wire, and probing a
single id put nothing there. The reference implementation listens after
configuring, so it never notices.

The protocol itself is written out in `crates/misa-can/src/backend/usbcan.rs`.
**The source material is not in the repository** — the licensing of some of it is
unclear — so that module's documentation is the record, not a pointer to one.

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

### RS485 port pairing on a multi-port adapter

`lkmotor-cli ports` lists the COM numbers the OS reports, but on a multi-port
RS485 adapter it cannot say which of them face each other — and a CH348 comes
back with a bare `Unknown` detail, because the `serialport` crate does not
classify it as a USB port on Windows and so there is not even a VID:PID to
group the ports by. `scripts\win\rs485_pair_probe.cmd` answers that by
transmitting a tag from one port at a time and reporting who hears it:

```powershell
.\scripts\win\rs485_pair_probe.cmd                              # sweep COM12-COM19
.\scripts\win\rs485_pair_probe.cmd -Ports "COM12,COM13"         # just this pair
```

The 16-byte tag proves the wiring, not that the link survives 1 Mbit/s — the
rate the LK motors run at. `rs485_pair_bulk.cmd` pushes 8 KB per direction and
reports the first mismatching offset, with the theoretical wire time alongside
so a link that arrives intact but slowly is still visible:

```powershell
.\scripts\win\rs485_pair_bulk.cmd
.\scripts\win\rs485_pair_bulk.cmd -Pairs "COM13,COM12;COM15,COM14"   # reverse
```

Both take a list as **one quoted string** through the `.cmd` wrapper —
semicolons between pairs for the bulk script. `powershell.exe -File` does not
re-parse its command line, so separate tokens would bind to the next parameter
instead of building an array. Called as `.ps1` directly, an ordinary PowerShell
array works too.

Ports have to be free: close the GUI and any `lkmotor-cli` run first. Power the
motors down as well — the payload carries no vendor header or checksum and so
cannot decode into a valid command, but nothing stops a motor midway through
its own reply from being confused by unsolicited traffic.

Bench result 2026-08-17 (CH348 as COM12-COM19): the pairs are COM12/COM13,
COM14/COM15 and **COM16/COM19**, with COM17 and COM18 wired to nothing. The
third pair breaks the adjacent numbering, so guessing COM16/COM17 and
COM18/COM19 tests a wire that does not exist and reads as a dead link. All six
directions passed 8 KB at 1 Mbit/s with zero byte errors.

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

**WSL says every file is modified while Windows says the tree is clean** — two
independent causes, and the first one hides the second. Both bite when this
directory is worked on from Windows and from WSL at the same path under
`/mnt/c`: one `.git`, two git installations, no shared global config.

*Line endings.* Git for Windows ships `core.autocrlf=true` in its system
config; WSL's git leaves it unset. The same bytes then get two verdicts. This
is fixed for good by `.gitattributes`, which pins `eol=lf` and so does not
depend on either installation's setting — if you see this symptom with content
differences (`git diff --stat` reporting a huge symmetric insertion and
deletion count), check that `.gitattributes` is present and that the files were
checked out after it landed.

*File modes.* drvfs reports every file under `/mnt/c` as `0755`, so with
`core.filemode=true` WSL's git reads a mode change on all of them. The tell is
a diff of `old mode 100644 / new mode 100755` with **zero** insertions and
deletions — files listed, no lines changed. `core.filemode` lives in
`.git/config`, which both installations share, so one command settles it:

```powershell
git config core.filemode false
```

Correct here regardless: nothing in this repo is recorded executable
(`git ls-files -s` shows no `100755`), and Windows cannot represent the bit
anyway. It is per-clone configuration, so a Linux-native clone is unaffected
and needs nothing.

Beware reading the two symptoms as one. `git diff --ignore-cr-at-eol --stat`
strips the line-ending noise, but files with a mode change stay listed with a
zero count — that residue is the second cause, not a rounding artefact of the
first.

## 8. Building and shipping the GUI

The desktop app is a Tauri shell around a React front end. Every other crate
in the workspace builds with nothing but a Rust toolchain; this one also needs
Node, because the front end is bundled into the binary at compile time.

```
cd ui
npm ci
npm run app:build
```

That produces two things:

| artifact | size | what it is |
|---|---|---|
| `target/release/misa-actuator-gui.exe` | ~12 MB | the app, runnable as-is |
| `target/release/bundle/nsis/misa-actuator_0.1.0_x64-setup.exe` | ~2.6 MB | NSIS installer |

The `.exe` is self-contained apart from the WebView2 runtime — the front end
is embedded, not loaded from disk.

**The embedding is done by the Tauri CLI, not by `cargo`, and `--release` does
not change that.** A binary from `cargo build -p misa-actuator-gui --release`
contains no front end at all; it falls back to `devUrl` and opens on
`ERR_CONNECTION_REFUSED` exactly like a debug build. Confirmed 2026-08-05 by
scanning both binaries: the Tauri-built one carries the `assets/index-*.js`
name, the cargo-built one does not. Use `npm run app` for development and
`npm run app:build` for anything you hand to somebody; for a release binary
without the installer, `tauri build --no-bundle` from
`crates/misa-actuator-gui` using the CLI in `ui/node_modules/.bin`.

### The bundler downloads its own tools

`tauri build` fetches NSIS and `nsis_tauri_utils.dll` from GitHub the first
time it bundles. On a machine with no outbound network the bundle step fails
even though the `.exe` builds fine. The downloads are cached under the Tauri
CLI's data directory afterwards.

### WebView2

The app renders in WebView2, which is preinstalled on Windows 11 and on
current Windows 10 builds, but not guaranteed on older ones. `tauri.conf.json`
does not set `bundle.windows.webviewInstallMode`, so the installer uses the
default: it downloads a bootstrapper at install time, which needs the target
machine to be online. Set it to `embedBootstrapper` (or `offlineInstaller`) if
you are shipping into an environment that will not be.

### Machines without a CAN adapter

The app starts fine with no PEAK driver installed — `PCANBasic.dll` is loaded
lazily, on the first connect, not at startup. Pressing **Connect** on a
`pcan:` interface then reports:

```
PCANBasic.dll not found. Install the PEAK-System device driver
(PCAN-Basic ships with it) and reconnect the adapter.
```

That surfaces as a banner above the tabs as well as in the Console log. The
simulator driver needs no adapter at all and is the right way to try the app
before any hardware arrives.

## 9. Do not build inside a synced OneDrive folder

Observed on 2026-08-02, in `…\OneDrive - Sony\work\…`: after a successful
`npm run app:build`, every later front-end build failed with

```
EPERM, Permission denied: …\ui\dist\index.html
```

`ui/dist/index.html` was held open by another process — not by any of ours
(no `node`, `vite`, `cargo` or `misa-actuator-gui` was running), and the file
could be neither deleted nor renamed. The sync client holds handles on files
it is uploading, and `ui/dist` and `target/` churn constantly.

`tsc --noEmit` still works, since it writes nothing, so type errors are still
catchable while this is happening.

The fix is to keep the working tree out of the synced tree, or to exclude
`target/`, `ui/dist/` and `ui/node_modules/` from sync. Beyond the locks,
syncing a Rust `target/` directory is a large and pointless amount of upload.
