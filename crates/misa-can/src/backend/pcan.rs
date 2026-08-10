//! PEAK-System PCAN-Basic backend (Windows).
//!
//! `PCANBasic.dll` is resolved with `LoadLibrary` at run time rather than
//! linked at build time. That matters for distribution: the CLIs compile and
//! run on any Windows box, the PEAK SDK is not needed to build, and a machine
//! without the PEAK driver gets a clear "install the driver" message instead
//! of failing to start at all. The DLL ships with the PEAK device driver and
//! lands in `System32`, so a plain `LoadLibrary("PCANBasic.dll")` finds it.
//!
//! Receives block properly instead of spinning: PCAN-Basic can signal a Win32
//! event on every incoming frame (`PCAN_RECEIVE_EVENT`), so [`CanBus::recv`]
//! is a `CAN_Read` / `WaitForSingleObject` pair bounded by the timeout.

#![allow(non_camel_case_types, non_snake_case)]

use std::ffi::{c_char, c_void, CString};
use std::ptr;
use std::sync::OnceLock;
use std::time::{Duration, Instant};

use crate::error::{Error, Result};
use crate::frame::Frame;
use crate::spec::InterfaceSpec;
use crate::{CanBus, OpenOptions};

// ---------------------------------------------------------------------------
// PCAN-Basic constants (PCANBasic.h)
// ---------------------------------------------------------------------------

const PCAN_ERROR_OK: u32 = 0x0000_0000;
const PCAN_ERROR_QRCVEMPTY: u32 = 0x0000_0020;
const PCAN_ERROR_BUSLIGHT: u32 = 0x0000_0004;
const PCAN_ERROR_BUSHEAVY: u32 = 0x0000_0008;
const PCAN_ERROR_BUSOFF: u32 = 0x0000_0010;
const PCAN_ERROR_BUSPASSIVE: u32 = 0x0004_0000;
const PCAN_ERROR_ANYBUSERR: u32 =
    PCAN_ERROR_BUSLIGHT | PCAN_ERROR_BUSHEAVY | PCAN_ERROR_BUSOFF | PCAN_ERROR_BUSPASSIVE;

/// The CAN controller's own buffer was read too late: frames were lost inside
/// the adapter, before the driver ever saw them.
const PCAN_ERROR_OVERRUN: u32 = 0x0000_0002;
/// The *host* receive queue overran: frames reached the driver and were
/// dropped before this process got round to reading them.
const PCAN_ERROR_QOVERRUN: u32 = 0x0000_0040;
/// Either kind of loss.
///
/// Worth its own name because these bits are the one failure mode the bus
/// error counters cannot see. A host-side queue overrun happens entirely
/// above the wire: every frame arrived intact, the controller is happy, and
/// `PCAN_ERROR_ANYBUSERR` stays clear while data goes missing. "No bus
/// errors" was used as evidence against exactly this in the DAMIAO CAN-FD
/// investigation (`doc/handover.md` §4), and it never was.
const PCAN_ERROR_ANYOVERRUN: u32 = PCAN_ERROR_OVERRUN | PCAN_ERROR_QOVERRUN;

const PCAN_MESSAGE_STANDARD: u8 = 0x00;
const PCAN_MESSAGE_RTR: u8 = 0x01;
const PCAN_MESSAGE_EXTENDED: u8 = 0x02;
const PCAN_MESSAGE_FD: u8 = 0x04;
const PCAN_MESSAGE_BRS: u8 = 0x08;
const PCAN_MESSAGE_ERRFRAME: u8 = 0x40;
const PCAN_MESSAGE_STATUS: u8 = 0x80;

const PCAN_RECEIVE_EVENT: u8 = 0x03;

/// `TPCANBaudrate` (BTR0/BTR1) codes for classic CAN. PCAN-Basic only accepts
/// this fixed set for `CAN_Initialize`.
const CLASSIC_BITRATES: &[(u32, u16)] = &[
    (1_000_000, 0x0014),
    (800_000, 0x0016),
    (500_000, 0x001C),
    (250_000, 0x011C),
    (125_000, 0x031C),
    (100_000, 0x432F),
    (95_000, 0xC34E),
    (83_000, 0x852B),
    (50_000, 0x472F),
    (47_000, 0x1414),
    (33_000, 0x8B2F),
    (20_000, 0x532F),
    (10_000, 0x672F),
    (5_000, 0x7F7F),
];

// ---------------------------------------------------------------------------
// PCAN-Basic structs
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Clone, Copy)]
struct TPCANMsg {
    id: u32,
    msgtype: u8,
    len: u8,
    data: [u8; 8],
}

#[repr(C)]
#[derive(Clone, Copy)]
struct TPCANMsgFD {
    id: u32,
    msgtype: u8,
    /// Data *length code* (0..=15), not a byte count — see [`dlc_to_len`].
    dlc: u8,
    data: [u8; 64],
}

/// What one driver read produced. `Skipped` is deliberately distinct from
/// `Empty`: both yield no frame, but only one of them means it is safe to
/// block waiting for the next arrival.
enum ReadOutcome {
    Frame(Frame),
    /// Nothing in the receive queue.
    Empty,
    /// A frame was consumed but is not one we surface (error / status / RTR).
    Skipped,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct TPCANTimestamp {
    millis: u32,
    millis_overflow: u16,
    micros: u16,
}

// ---------------------------------------------------------------------------
// Win32 / dynamic loading
// ---------------------------------------------------------------------------

type HModule = *mut c_void;
type Handle = *mut c_void;

const WAIT_OBJECT_0: u32 = 0;
const WAIT_TIMEOUT: u32 = 0x0000_0102;

#[link(name = "kernel32")]
extern "system" {
    fn LoadLibraryA(name: *const c_char) -> HModule;
    fn GetProcAddress(module: HModule, name: *const c_char) -> *mut c_void;
    fn CreateEventA(
        attributes: *mut c_void,
        manual_reset: i32,
        initial_state: i32,
        name: *const c_char,
    ) -> Handle;
    fn WaitForSingleObject(handle: Handle, millis: u32) -> u32;
    fn CloseHandle(handle: Handle) -> i32;
}

type FnInitialize = unsafe extern "system" fn(u16, u16, u8, u32, u16) -> u32;
type FnInitializeFD = unsafe extern "system" fn(u16, *const c_char) -> u32;
type FnUninitialize = unsafe extern "system" fn(u16) -> u32;
type FnRead = unsafe extern "system" fn(u16, *mut TPCANMsg, *mut TPCANTimestamp) -> u32;
type FnReadFD = unsafe extern "system" fn(u16, *mut TPCANMsgFD, *mut u64) -> u32;
type FnWrite = unsafe extern "system" fn(u16, *mut TPCANMsg) -> u32;
type FnWriteFD = unsafe extern "system" fn(u16, *mut TPCANMsgFD) -> u32;
type FnSetValue = unsafe extern "system" fn(u16, u8, *mut c_void, u32) -> u32;
type FnGetErrorText = unsafe extern "system" fn(u32, u16, *mut c_char) -> u32;

/// Resolved entry points of `PCANBasic.dll`.
struct PcanApi {
    initialize: FnInitialize,
    initialize_fd: Option<FnInitializeFD>,
    uninitialize: FnUninitialize,
    read: FnRead,
    read_fd: Option<FnReadFD>,
    write: FnWrite,
    write_fd: Option<FnWriteFD>,
    set_value: FnSetValue,
    get_error_text: Option<FnGetErrorText>,
}

// The DLL is loaded once and never freed, and PCAN-Basic is documented as
// thread-safe, so sharing the resolved pointers across threads is sound.
unsafe impl Send for PcanApi {}
unsafe impl Sync for PcanApi {}

static API: OnceLock<std::result::Result<PcanApi, String>> = OnceLock::new();

/// Load `PCANBasic.dll` (once per process) and resolve its entry points.
fn api() -> Result<&'static PcanApi> {
    let loaded = API.get_or_init(|| unsafe {
        let name = CString::new("PCANBasic.dll").unwrap();
        let module = LoadLibraryA(name.as_ptr());
        if module.is_null() {
            return Err(
                "PCANBasic.dll not found. Install the PEAK-System device driver \
                 (PCAN-Basic ships with it) and reconnect the adapter."
                    .to_string(),
            );
        }

        // Safety: every name below is a documented PCAN-Basic export and the
        // signatures are transcribed from PCANBasic.h, so the transmute from
        // the resolved address to the matching fn pointer is sound.
        macro_rules! sym {
            ($name:literal) => {{
                let c = CString::new($name).unwrap();
                GetProcAddress(module, c.as_ptr())
            }};
        }
        macro_rules! required {
            ($name:literal, $ty:ty) => {{
                let p = sym!($name);
                if p.is_null() {
                    return Err(format!(
                        "PCANBasic.dll is missing {}; the installed driver is too old",
                        $name
                    ));
                }
                std::mem::transmute::<*mut c_void, $ty>(p)
            }};
        }
        macro_rules! optional {
            ($name:literal, $ty:ty) => {{
                let p = sym!($name);
                if p.is_null() {
                    None
                } else {
                    Some(std::mem::transmute::<*mut c_void, $ty>(p))
                }
            }};
        }

        Ok(PcanApi {
            initialize: required!("CAN_Initialize", FnInitialize),
            initialize_fd: optional!("CAN_InitializeFD", FnInitializeFD),
            uninitialize: required!("CAN_Uninitialize", FnUninitialize),
            read: required!("CAN_Read", FnRead),
            read_fd: optional!("CAN_ReadFD", FnReadFD),
            write: required!("CAN_Write", FnWrite),
            write_fd: optional!("CAN_WriteFD", FnWriteFD),
            set_value: required!("CAN_SetValue", FnSetValue),
            get_error_text: optional!("CAN_GetErrorText", FnGetErrorText),
        })
    });

    loaded.as_ref().map_err(|e| Error::Config(e.clone()))
}

/// Ask the driver to describe a status code, falling back to the raw value.
/// What a `CAN_Read` status means, once the overrun bits are set aside.
///
/// Split out from the reporting so the bit handling can be tested without a
/// driver, an adapter or a bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReadStatus {
    /// A message was read and nothing is wrong.
    Ok,
    /// Nothing waiting in the queue.
    Empty,
    /// The controller has taken itself off the bus. Not recoverable here.
    BusOff,
    /// Bus trouble that the controller is still riding out.
    BusError,
    /// Something else the driver calls an error.
    Fatal,
}

/// Where an overrun lost frames, or `None` if the status reports no loss.
///
/// Reported separately from [`ReadStatus`] because an overrun can accompany
/// any of those: it does not describe how the call went, it says how much
/// never made it to the caller.
fn overrun_kind(status: u32) -> Option<&'static str> {
    // Check the host queue first: when both are set, the nearer one is the
    // one that has already swallowed the frames, so it is the more useful
    // thing to name.
    if status & PCAN_ERROR_QOVERRUN != 0 {
        Some("host receive queue")
    } else if status & PCAN_ERROR_OVERRUN != 0 {
        Some("CAN controller buffer")
    } else {
        None
    }
}

/// Classify a `CAN_Read` status.
///
/// The overrun bits are masked out first, deliberately. An overrun means data
/// was lost, not that the read failed — a status carrying nothing else is a
/// successful read that happens to come with bad news, and turning it into an
/// error would end the session over frames that are already gone.
fn classify_read_status(status: u32) -> ReadStatus {
    let status = status & !PCAN_ERROR_ANYOVERRUN;
    if status & PCAN_ERROR_QRCVEMPTY != 0 {
        return ReadStatus::Empty;
    }
    if status == PCAN_ERROR_OK {
        return ReadStatus::Ok;
    }
    if status & PCAN_ERROR_BUSOFF != 0 {
        return ReadStatus::BusOff;
    }
    if status & PCAN_ERROR_ANYBUSERR != 0 {
        return ReadStatus::BusError;
    }
    ReadStatus::Fatal
}

fn status_text(api: &PcanApi, status: u32) -> String {
    if let Some(f) = api.get_error_text {
        let mut buf = [0u8; 256];
        // 0x09 = LANG_ENGLISH.
        let rc = unsafe { f(status, 0x09, buf.as_mut_ptr() as *mut c_char) };
        if rc == PCAN_ERROR_OK {
            let end = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
            return String::from_utf8_lossy(&buf[..end]).trim().to_string();
        }
    }
    format!("PCAN status 0x{status:08X}")
}

// ---------------------------------------------------------------------------
// Channel names and bit timing
// ---------------------------------------------------------------------------

/// Resolve `usb1`, `PCAN_USBBUS3`, `pci2`, `lan1`, `3`, `0x51` to a
/// `TPCANHandle`.
fn resolve_channel(target: &str, raw_spec: &str) -> Result<(u16, String)> {
    let t = target.trim().to_ascii_lowercase();
    let t = t.strip_prefix("pcan_").unwrap_or(&t);

    if let Some(hex) = t.strip_prefix("0x") {
        let handle = u16::from_str_radix(hex, 16)
            .map_err(|_| Error::bad_spec(raw_spec, format!("bad PCAN handle {target:?}")))?;
        return Ok((handle, format!("PCAN handle 0x{handle:02X}")));
    }

    // `usbbus1` and `usb1` are the same thing.
    let (family, index) = if let Some(rest) = t.strip_prefix("usbbus") {
        ("usb", rest)
    } else if let Some(rest) = t.strip_prefix("usb") {
        ("usb", rest)
    } else if let Some(rest) = t.strip_prefix("pcibus") {
        ("pci", rest)
    } else if let Some(rest) = t.strip_prefix("pci") {
        ("pci", rest)
    } else if let Some(rest) = t.strip_prefix("lanbus") {
        ("lan", rest)
    } else if let Some(rest) = t.strip_prefix("lan") {
        ("lan", rest)
    } else if !t.is_empty() && t.chars().all(|c| c.is_ascii_digit()) {
        // A bare number is the common case: `pcan:1` = first USB adapter.
        ("usb", t)
    } else {
        return Err(Error::bad_spec(
            raw_spec,
            format!("unknown PCAN channel {target:?} (try usb1, pci1 or lan1)"),
        ));
    };

    let n: u16 = index
        .parse()
        .map_err(|_| Error::bad_spec(raw_spec, format!("bad PCAN channel number in {target:?}")))?;
    if !(1..=16).contains(&n) {
        return Err(Error::bad_spec(
            raw_spec,
            format!("PCAN channel index {n} is outside 1..=16"),
        ));
    }

    // Channels 1..=8 are contiguous from the family base; 9..=16 live in a
    // second block PEAK added later (0x409 / 0x509 / ...).
    let base = match family {
        "usb" => 0x50u16,
        "pci" => 0x40,
        _ => 0x800,
    };
    let handle = match family {
        "lan" => 0x800 + n,
        _ if n <= 8 => base + n,
        _ => (base << 4) + n,
    };
    Ok((handle, format!("PCAN_{}BUS{n}", family.to_uppercase())))
}

/// Bit timing for one phase of a CAN-FD link, derived for an 80 MHz clock at
/// an ~80 % sample point. The values this produces for the usual rates match
/// the reference strings in PEAK's documentation exactly (1 Mbit/s →
/// tseg1 63 / tseg2 16, 5 Mbit/s → tseg1 11 / tseg2 4).
struct FdTiming {
    brp: u32,
    tseg1: u32,
    tseg2: u32,
    sjw: u32,
}

const FD_CLOCK_MHZ: u32 = 80;

fn fd_timing(bitrate: u32, max_tseg1: u32, max_tseg2: u32, max_sjw: u32) -> Option<FdTiming> {
    let clock = FD_CLOCK_MHZ * 1_000_000;
    for brp in 1..=32u32 {
        if clock % (bitrate * brp) != 0 {
            continue;
        }
        let total = clock / (bitrate * brp); // time quanta per bit
        if total < 4 {
            continue;
        }
        // 80 % sample point, i.e. everything before the sample lands in
        // sync-seg (1 tq) + tseg1.
        let before = (total * 4) / 5;
        let tseg1 = before.saturating_sub(1);
        let tseg2 = total - 1 - tseg1;
        if tseg1 == 0 || tseg2 == 0 || tseg1 > max_tseg1 || tseg2 > max_tseg2 {
            continue;
        }
        return Some(FdTiming {
            brp,
            tseg1,
            tseg2,
            sjw: tseg2.min(max_sjw),
        });
    }
    None
}

/// Build the `TPCANBitrateFD` string `CAN_InitializeFD` wants.
fn fd_bitrate_string(nominal: u32, data: u32, raw_spec: &str) -> Result<CString> {
    let nom = fd_timing(nominal, 256, 128, 128).ok_or_else(|| {
        Error::Config(format!(
            "cannot derive CAN-FD arbitration timing for {nominal} bit/s on an \
             {FD_CLOCK_MHZ} MHz clock; pass a standard rate such as @1M or @500K \
             (spec {raw_spec:?})"
        ))
    })?;
    let dat = fd_timing(data, 32, 16, 16).ok_or_else(|| {
        Error::Config(format!(
            "cannot derive CAN-FD data timing for {data} bit/s on an \
             {FD_CLOCK_MHZ} MHz clock; try 2M, 4M, 5M or 8M (spec {raw_spec:?})"
        ))
    })?;

    let s = format!(
        "f_clock_mhz={FD_CLOCK_MHZ}, nom_brp={}, nom_tseg1={}, nom_tseg2={}, nom_sjw={}, \
         data_brp={}, data_tseg1={}, data_tseg2={}, data_sjw={}",
        nom.brp, nom.tseg1, nom.tseg2, nom.sjw, dat.brp, dat.tseg1, dat.tseg2, dat.sjw
    );
    CString::new(s).map_err(|_| Error::Config("bad CAN-FD bitrate string".into()))
}

fn classic_btr(bitrate: u32, raw_spec: &str) -> Result<u16> {
    CLASSIC_BITRATES
        .iter()
        .find(|(rate, _)| *rate == bitrate)
        .map(|(_, code)| *code)
        .ok_or_else(|| {
            let known: Vec<String> = CLASSIC_BITRATES
                .iter()
                .map(|(r, _)| format!("{}", r / 1000))
                .collect();
            Error::bad_spec(
                raw_spec,
                format!(
                    "PCAN classic CAN supports only these bitrates (kbit/s): {}",
                    known.join(", ")
                ),
            )
        })
}

/// CAN-FD data-length-code → byte count.
fn dlc_to_len(dlc: u8) -> usize {
    match dlc {
        0..=8 => dlc as usize,
        9 => 12,
        10 => 16,
        11 => 20,
        12 => 24,
        13 => 32,
        14 => 48,
        _ => 64,
    }
}

/// Byte count → the smallest CAN-FD data-length code that holds it.
fn len_to_dlc(len: usize) -> u8 {
    match len {
        0..=8 => len as u8,
        9..=12 => 9,
        13..=16 => 10,
        17..=20 => 11,
        21..=24 => 12,
        25..=32 => 13,
        33..=48 => 14,
        _ => 15,
    }
}

// ---------------------------------------------------------------------------
// The backend
// ---------------------------------------------------------------------------

/// PCAN-Basic transport bound to one channel.
pub struct PcanBackend {
    api: &'static PcanApi,
    channel: u16,
    channel_name: String,
    bitrate: u32,
    data_bitrate: u32,
    fd: bool,
    timeout: Duration,
    /// Auto-reset event PCAN signals on every received frame, or null if the
    /// driver refused to install one (then [`Self::recv`] polls instead).
    rx_event: Handle,
    /// How many reads reported frames lost before this process could take
    /// them — see [`PCAN_ERROR_ANYOVERRUN`].
    ///
    /// Counted rather than merely logged so a measurement run can state a
    /// number instead of an impression. The log line says which side lost
    /// them; this is just how often.
    rx_overruns: u64,
}

// The channel handle is a plain integer and the event is owned exclusively by
// this struct; PCAN-Basic itself is thread-safe.
unsafe impl Send for PcanBackend {}

impl PcanBackend {
    /// Initialize a PCAN channel from a parsed interface spec.
    pub fn open(spec: &InterfaceSpec, opts: &OpenOptions) -> Result<Self> {
        let api = api()?;
        let (channel, channel_name) = resolve_channel(&spec.target, &spec.raw)?;

        let status = if opts.fd {
            let init_fd = api.initialize_fd.ok_or_else(|| {
                Error::Config(
                    "the installed PCANBasic.dll has no CAN-FD support; \
                     update the PEAK driver"
                        .into(),
                )
            })?;
            let bitrate = fd_bitrate_string(spec.bitrate, spec.data_bitrate, &spec.raw)?;
            unsafe { init_fd(channel, bitrate.as_ptr()) }
        } else {
            let btr = classic_btr(spec.bitrate, &spec.raw)?;
            unsafe { (api.initialize)(channel, btr, 0, 0, 0) }
        };

        if status != PCAN_ERROR_OK {
            return Err(Error::Driver {
                api: if opts.fd {
                    "CAN_InitializeFD"
                } else {
                    "CAN_Initialize"
                },
                message: format!(
                    "{} on {channel_name}: {}",
                    status_text(api, status),
                    "check that the adapter is plugged in and not already in use"
                ),
                code: status,
            });
        }

        // Install the RX event so recv() can block instead of spin. Losing
        // this is not fatal — we fall back to polling — but it costs CPU, so
        // say something.
        let mut rx_event =
            unsafe { CreateEventA(ptr::null_mut(), 0 /* auto-reset */, 0, ptr::null()) };
        if !rx_event.is_null() {
            let st = unsafe {
                (api.set_value)(
                    channel,
                    PCAN_RECEIVE_EVENT,
                    &mut rx_event as *mut Handle as *mut c_void,
                    std::mem::size_of::<Handle>() as u32,
                )
            };
            if st != PCAN_ERROR_OK {
                log::warn!(
                    "PCAN: could not install a receive event ({}); falling back to polling",
                    status_text(api, st)
                );
                unsafe { CloseHandle(rx_event) };
                rx_event = ptr::null_mut();
            }
        }

        Ok(Self {
            api,
            channel,
            channel_name,
            bitrate: spec.bitrate,
            data_bitrate: spec.data_bitrate,
            fd: opts.fd,
            timeout: opts.timeout,
            rx_event,
            rx_overruns: 0,
        })
    }

    /// Drain until a usable frame turns up. `Ok(None)` = the queue is empty.
    ///
    /// The loop matters. A single read that hits an error/status/RTR frame has
    /// consumed a queue entry but has nothing to return, and reporting that as
    /// "empty" sends the caller into `WaitForSingleObject` — where it blocks
    /// on an auto-reset event that will not fire again for a frame already
    /// sitting in the queue. The wanted reply is then only found by the *next*
    /// arrival, or not at all. That showed up as roughly one register read in
    /// nine timing out over CAN-FD, which is exactly the sort of intermittent
    /// failure that gets blamed on the bus.
    fn try_read(&mut self) -> Result<Option<Frame>> {
        loop {
            match self.read_one()? {
                ReadOutcome::Frame(f) => return Ok(Some(f)),
                ReadOutcome::Empty => return Ok(None),
                ReadOutcome::Skipped => continue,
            }
        }
    }

    /// A single `CAN_Read`/`CAN_ReadFD` call.
    fn read_one(&mut self) -> Result<ReadOutcome> {
        if self.fd {
            let read_fd = self
                .api
                .read_fd
                .ok_or(Error::Unsupported("CAN_ReadFD missing from PCANBasic.dll"))?;
            let mut msg = TPCANMsgFD {
                id: 0,
                msgtype: 0,
                dlc: 0,
                data: [0; 64],
            };
            let mut ts: u64 = 0;
            let status = unsafe { read_fd(self.channel, &mut msg, &mut ts) };
            self.check_read_status(status)?;
            if status & PCAN_ERROR_QRCVEMPTY != 0 {
                return Ok(ReadOutcome::Empty);
            }
            if msg.msgtype & (PCAN_MESSAGE_ERRFRAME | PCAN_MESSAGE_STATUS | PCAN_MESSAGE_RTR) != 0 {
                return Ok(ReadOutcome::Skipped);
            }
            let len = dlc_to_len(msg.dlc);
            Ok(ReadOutcome::Frame(Frame::from_raw(
                msg.id,
                msg.msgtype & PCAN_MESSAGE_EXTENDED != 0,
                &msg.data[..len.min(64)],
                msg.msgtype & PCAN_MESSAGE_FD != 0,
                msg.msgtype & PCAN_MESSAGE_BRS != 0,
            )?))
        } else {
            let mut msg = TPCANMsg {
                id: 0,
                msgtype: 0,
                len: 0,
                data: [0; 8],
            };
            let mut ts = TPCANTimestamp::default();
            let status = unsafe { (self.api.read)(self.channel, &mut msg, &mut ts) };
            self.check_read_status(status)?;
            if status & PCAN_ERROR_QRCVEMPTY != 0 {
                return Ok(ReadOutcome::Empty);
            }
            if msg.msgtype & (PCAN_MESSAGE_ERRFRAME | PCAN_MESSAGE_STATUS | PCAN_MESSAGE_RTR) != 0 {
                return Ok(ReadOutcome::Skipped);
            }
            let len = (msg.len as usize).min(8);
            Ok(ReadOutcome::Frame(Frame::from_raw(
                msg.id,
                msg.msgtype & PCAN_MESSAGE_EXTENDED != 0,
                &msg.data[..len],
                false,
                false,
            )?))
        }
    }

    /// A read status carries bus health and frame loss alongside the queue
    /// state: report both, only fail on something genuinely unrecoverable.
    fn check_read_status(&mut self, status: u32) -> Result<()> {
        // Loss first, and independently of how the read itself went. This is
        // the case "no bus errors were reported" cannot rule out, so it is
        // said out loud every time rather than inferred later from silence.
        if let Some(where_) = overrun_kind(status) {
            self.rx_overruns += 1;
            log::warn!(
                "PCAN {}: {} overran — frames were dropped before this process \
                 read them ({} so far). A reply that never arrives after this \
                 was lost here, not on the wire.",
                self.channel_name,
                where_,
                self.rx_overruns
            );
        }

        match classify_read_status(status) {
            ReadStatus::Ok | ReadStatus::Empty => Ok(()),
            ReadStatus::BusOff => Err(Error::Driver {
                api: "CAN_Read",
                message: format!(
                    "{} is bus-off — check termination, wiring and that every \
                     node agrees on {} bit/s",
                    self.channel_name, self.bitrate
                ),
                code: status,
            }),
            ReadStatus::BusError => {
                log::warn!(
                    "PCAN {}: bus error ({})",
                    self.channel_name,
                    status_text(self.api, status)
                );
                Ok(())
            }
            ReadStatus::Fatal => Err(Error::Driver {
                api: "CAN_Read",
                message: format!("{} on {}", status_text(self.api, status), self.channel_name),
                code: status,
            }),
        }
    }
}

impl CanBus for PcanBackend {
    fn send(&mut self, frame: &Frame) -> Result<()> {
        let status = if self.fd {
            let write_fd = self
                .api
                .write_fd
                .ok_or(Error::Unsupported("CAN_WriteFD missing from PCANBasic.dll"))?;
            let (buf, len) = frame.fd_padded();
            let mut msgtype = if frame.is_extended() {
                PCAN_MESSAGE_EXTENDED
            } else {
                PCAN_MESSAGE_STANDARD
            };
            // On an FD-initialized channel a classic frame is still legal, so
            // only tag FD/BRS when the caller asked for it.
            if frame.is_fd() {
                msgtype |= PCAN_MESSAGE_FD;
                if frame.is_brs() {
                    msgtype |= PCAN_MESSAGE_BRS;
                }
            }
            let mut msg = TPCANMsgFD {
                id: frame.raw_id(),
                msgtype,
                dlc: len_to_dlc(len),
                data: buf,
            };
            unsafe { write_fd(self.channel, &mut msg) }
        } else {
            if frame.is_fd() {
                return Err(Error::Unsupported(
                    "CAN-FD frame on a channel opened in classic mode",
                ));
            }
            let mut data = [0u8; 8];
            let len = frame.len().min(8);
            data[..len].copy_from_slice(&frame.data()[..len]);
            let mut msg = TPCANMsg {
                id: frame.raw_id(),
                msgtype: if frame.is_extended() {
                    PCAN_MESSAGE_EXTENDED
                } else {
                    PCAN_MESSAGE_STANDARD
                },
                len: len as u8,
                data,
            };
            unsafe { (self.api.write)(self.channel, &mut msg) }
        };

        if status != PCAN_ERROR_OK {
            return Err(Error::Driver {
                api: "CAN_Write",
                message: format!(
                    "{} on {}",
                    status_text(self.api, status),
                    self.channel_name
                ),
                code: status,
            });
        }
        Ok(())
    }

    fn recv(&mut self) -> Result<Frame> {
        let deadline = Instant::now() + self.timeout;
        loop {
            if let Some(frame) = self.try_read()? {
                return Ok(frame);
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(Error::Timeout);
            }
            if self.rx_event.is_null() {
                // No event available: poll, but yield so a busy CLI loop does
                // not pin a core.
                std::thread::sleep(Duration::from_micros(200).min(remaining));
                continue;
            }
            // Round up so a sub-millisecond remainder still waits rather than
            // degenerating into a spin.
            let millis = remaining.as_millis().min(u32::MAX as u128) as u32;
            match unsafe { WaitForSingleObject(self.rx_event, millis.max(1)) } {
                WAIT_OBJECT_0 => continue,
                WAIT_TIMEOUT => return Err(Error::Timeout),
                other => {
                    return Err(Error::Driver {
                        api: "WaitForSingleObject",
                        message: format!("waiting on the PCAN receive event failed ({other:#X})"),
                        code: other,
                    })
                }
            }
        }
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        // PCAN has no socket-level timeout; recv() enforces it itself.
        self.timeout = timeout;
        Ok(())
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }

    fn supports_fd(&self) -> bool {
        self.fd
    }

    fn rx_overruns(&self) -> u64 {
        self.rx_overruns
    }

    fn backend_name(&self) -> &'static str {
        "pcan"
    }

    fn description(&self) -> String {
        if self.fd {
            format!(
                "{} (CAN-FD, {} bit/s arbitration, {} bit/s data)",
                self.channel_name, self.bitrate, self.data_bitrate
            )
        } else {
            format!("{} (CAN, {} bit/s)", self.channel_name, self.bitrate)
        }
    }
}

impl Drop for PcanBackend {
    fn drop(&mut self) {
        unsafe {
            (self.api.uninitialize)(self.channel);
            if !self.rx_event.is_null() {
                CloseHandle(self.rx_event);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An overrun says data was lost, not that the call failed. Classifying it
    /// as an error would end a session over frames that are already gone —
    /// and, worse, would report a driver failure where the honest answer is
    /// "some replies will be missing".
    #[test]
    fn an_overrun_alone_is_still_a_successful_read() {
        assert_eq!(classify_read_status(PCAN_ERROR_QOVERRUN), ReadStatus::Ok);
        assert_eq!(classify_read_status(PCAN_ERROR_OVERRUN), ReadStatus::Ok);
        assert_eq!(
            classify_read_status(PCAN_ERROR_QOVERRUN | PCAN_ERROR_QRCVEMPTY),
            ReadStatus::Empty,
        );
    }

    /// An overrun must never mask a condition that does need acting on.
    #[test]
    fn an_overrun_does_not_hide_a_bus_fault() {
        assert_eq!(
            classify_read_status(PCAN_ERROR_QOVERRUN | PCAN_ERROR_BUSOFF),
            ReadStatus::BusOff,
        );
        assert_eq!(
            classify_read_status(PCAN_ERROR_OVERRUN | PCAN_ERROR_BUSHEAVY),
            ReadStatus::BusError,
        );
    }

    /// Which side dropped the frames decides whether the wire is even a
    /// suspect, so the two must not be conflated.
    #[test]
    fn overrun_names_the_side_that_lost_the_frames() {
        assert_eq!(
            overrun_kind(PCAN_ERROR_QOVERRUN),
            Some("host receive queue")
        );
        assert_eq!(
            overrun_kind(PCAN_ERROR_OVERRUN),
            Some("CAN controller buffer")
        );
        // Both set: name the host queue, the nearer of the two.
        assert_eq!(
            overrun_kind(PCAN_ERROR_OVERRUN | PCAN_ERROR_QOVERRUN),
            Some("host receive queue"),
        );
    }

    /// The whole point of the exercise: a clean bus and an empty-looking read
    /// must not report loss, or the counter means nothing.
    #[test]
    fn ordinary_statuses_report_no_loss() {
        for status in [
            PCAN_ERROR_OK,
            PCAN_ERROR_QRCVEMPTY,
            PCAN_ERROR_BUSLIGHT,
            PCAN_ERROR_BUSHEAVY,
            PCAN_ERROR_BUSOFF,
            PCAN_ERROR_BUSPASSIVE,
        ] {
            assert_eq!(overrun_kind(status), None, "status {status:#X}");
        }
    }

    /// Bit 0x40 is `PCAN_ERROR_QOVERRUN` and 0x02 is `PCAN_ERROR_OVERRUN` in
    /// PCANBasic.h. Nothing at run time will complain if these drift — the
    /// counter would simply stay at zero and the investigation would conclude
    /// the wrong thing.
    #[test]
    fn overrun_bits_match_pcan_basic() {
        assert_eq!(PCAN_ERROR_OVERRUN, 0x0000_0002);
        assert_eq!(PCAN_ERROR_QOVERRUN, 0x0000_0040);
        // Distinct from the queue-empty bit next to it.
        assert_ne!(PCAN_ERROR_QOVERRUN, PCAN_ERROR_QRCVEMPTY);
        assert_eq!(PCAN_ERROR_ANYOVERRUN & PCAN_ERROR_ANYBUSERR, 0);
    }

    #[test]
    fn unknown_status_is_still_an_error() {
        // 0x100 = PCAN_ERROR_REGTEST.
        assert_eq!(classify_read_status(0x0000_0100), ReadStatus::Fatal);
    }

    #[test]
    fn channel_names_resolve() {
        assert_eq!(resolve_channel("usb1", "").unwrap().0, 0x51);
        assert_eq!(resolve_channel("USBBUS8", "").unwrap().0, 0x58);
        assert_eq!(resolve_channel("usb9", "").unwrap().0, 0x509);
        assert_eq!(resolve_channel("usb16", "").unwrap().0, 0x510);
        assert_eq!(resolve_channel("pci1", "").unwrap().0, 0x41);
        assert_eq!(resolve_channel("pci9", "").unwrap().0, 0x409);
        assert_eq!(resolve_channel("lan1", "").unwrap().0, 0x801);
        // A bare number means the first USB family channel.
        assert_eq!(resolve_channel("2", "").unwrap().0, 0x52);
        assert_eq!(resolve_channel("0x51", "").unwrap().0, 0x51);

        assert!(resolve_channel("usb0", "").is_err());
        assert!(resolve_channel("usb17", "").is_err());
        assert!(resolve_channel("kvaser1", "").is_err());
    }

    #[test]
    fn fd_timing_matches_peaks_reference_values() {
        // PEAK documents 1 Mbit/s arbitration on an 80 MHz clock as
        // nom_brp=1, nom_tseg1=63, nom_tseg2=16.
        let t = fd_timing(1_000_000, 256, 128, 128).unwrap();
        assert_eq!((t.brp, t.tseg1, t.tseg2), (1, 63, 16));
        // ...and 5 Mbit/s data as data_brp=1, data_tseg1=11, data_tseg2=4.
        let t = fd_timing(5_000_000, 32, 16, 16).unwrap();
        assert_eq!((t.brp, t.tseg1, t.tseg2), (1, 11, 4));
        // 2 Mbit/s data.
        let t = fd_timing(2_000_000, 32, 16, 16).unwrap();
        assert_eq!((t.brp, t.tseg1, t.tseg2), (1, 31, 8));
        // 500 kbit/s arbitration.
        let t = fd_timing(500_000, 256, 128, 128).unwrap();
        assert_eq!((t.brp, t.tseg1, t.tseg2), (1, 127, 32));
    }

    #[test]
    fn classic_bitrates_are_limited_to_the_btr_table() {
        assert_eq!(classic_btr(1_000_000, "").unwrap(), 0x0014);
        assert_eq!(classic_btr(500_000, "").unwrap(), 0x001C);
        assert!(classic_btr(750_000, "").is_err());
    }

    #[test]
    fn dlc_round_trips() {
        for len in [0usize, 1, 8, 12, 16, 20, 24, 32, 48, 64] {
            assert_eq!(dlc_to_len(len_to_dlc(len)), len, "len={len}");
        }
        // Odd lengths pad up to the next legal step.
        assert_eq!(dlc_to_len(len_to_dlc(9)), 12);
        assert_eq!(dlc_to_len(len_to_dlc(33)), 48);
    }

    /// These structs are handed straight to a C DLL, so a wrong offset is not
    /// a compile error — it is garbage IDs and payloads at runtime, or a
    /// buffer the DLL writes past.
    ///
    /// `PCANBasic.h` declares them with natural alignment:
    ///
    /// ```c
    /// typedef struct tagTPCANMsgFD {
    ///     DWORD             ID;        // offset 0
    ///     TPCANMessageType  MSGTYPE;   // BYTE, offset 4
    ///     BYTE              DLC;       // offset 5
    ///     BYTE              DATA[64];  // offset 6
    /// } TPCANMsgFD;                    // size 72 (padded to 4-byte align)
    /// ```
    ///
    /// The classic struct below follows the same shape and is already proven
    /// against real hardware, which is the evidence that this convention is
    /// the right one.
    #[test]
    fn the_pcan_structs_match_the_c_header_layout() {
        use core::mem::{align_of, size_of};

        assert_eq!(size_of::<TPCANMsg>(), 16);
        assert_eq!(align_of::<TPCANMsg>(), 4);
        assert_eq!(size_of::<TPCANMsgFD>(), 72);
        assert_eq!(align_of::<TPCANMsgFD>(), 4);

        let msg = TPCANMsgFD {
            id: 0,
            msgtype: 0,
            dlc: 0,
            data: [0; 64],
        };
        let base = &msg as *const _ as usize;
        assert_eq!(&msg.id as *const _ as usize - base, 0);
        assert_eq!(&msg.msgtype as *const _ as usize - base, 4);
        assert_eq!(&msg.dlc as *const _ as usize - base, 5);
        assert_eq!(&msg.data as *const _ as usize - base, 6);
    }

    /// DAMIAO sends 8-byte payloads whether the link is classic or FD, so the
    /// FD path must not pad them into a longer DLC step — a 12-byte frame
    /// where the motor expects 8 would be rejected or misparsed.
    #[test]
    fn an_eight_byte_fd_payload_stays_eight_bytes() {
        assert_eq!(len_to_dlc(8), 8);
        assert_eq!(dlc_to_len(8), 8);

        let id = embedded_can::StandardId::new(0x01).unwrap();
        let frame = crate::Frame::new_fd(id, &[1, 2, 3, 4, 5, 6, 7, 8], true).unwrap();
        let (_, padded) = frame.fd_padded();
        assert_eq!(padded, 8, "an 8-byte DAMIAO frame must not grow");
        assert!(frame.is_brs(), "the data phase must switch bitrate");
    }
}
