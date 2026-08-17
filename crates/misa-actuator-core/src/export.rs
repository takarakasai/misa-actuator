//! Writing a finished run to disk.
//!
//! **Saved automatically, not on request.** A quasi-static run costs tens of
//! seconds of a motor's time and cannot be reconstructed from the plot — the
//! plot keeps two derived series, and the samples behind them are dropped as
//! soon as the result is mapped for display. A run whose data was lost because
//! nobody pressed Export is a run that has to be done again, and once several
//! motors are measured in one batch there is nobody there to press it.
//!
//! CSV rather than xlsx: the sample writer already existed in `misa_sysid`, and
//! xlsx would mean a new dependency for a format Excel opens either way. The
//! metadata that a spreadsheet would put on a second sheet goes in `#` comment
//! lines above the header instead, which Excel imports as text rows and every
//! plotting tool skips.

use std::fs;
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

/// Environment variable that overrides where runs are written.
///
/// An environment variable rather than a setting in the app, because the app has
/// nowhere to keep a setting: there is no preferences file, and a directory typed
/// into a field that forgets it every launch is worse than a fixed default. This
/// works for the CLI too, which is where a scripted campaign would set it.
pub const DATA_DIR_ENV: &str = "MISA_ACTUATOR_DATA_DIR";

/// Where runs are written.
///
/// [`DATA_DIR_ENV`] wins if it names anything; measurements usually want to live
/// with the project they belong to rather than in one pile under a home
/// directory.
///
/// Otherwise under the user's home — not beside the executable, which is often
/// read-only once installed, and not under a temp directory, which is the one
/// place measurements are expected to disappear from. The temp directory is the
/// last resort only when the environment names no home at all, because losing
/// the run outright would be worse than putting it somewhere awkward.
pub fn data_dir() -> PathBuf {
    // Order matters, most explicit first. The environment beats the remembered
    // choice so a scripted campaign can redirect one run without disturbing what
    // the operator picked in the app.
    //
    // Whitespace-only is treated as unset: an exported-but-empty variable is a
    // shell accident, and writing to the process's working directory because of
    // one would scatter runs wherever the app happened to be launched from.
    if let Some(dir) = std::env::var_os(DATA_DIR_ENV) {
        let dir = PathBuf::from(dir);
        if !dir.as_os_str().is_empty() && dir.to_string_lossy().trim() != "" {
            return dir;
        }
    }
    if let Some(dir) = remembered_dir() {
        return dir;
    }
    default_dir()
}

/// Whether [`data_dir`] is answering with something that was chosen, rather than
/// the default. A UI should be able to say which.
pub fn data_dir_is_chosen() -> bool {
    std::env::var_os(DATA_DIR_ENV)
        .map(|v| !v.to_string_lossy().trim().is_empty())
        .unwrap_or(false)
        || remembered_dir().is_some()
}

/// The fallback: under the user's home.
///
/// Not beside the executable, which is often read-only once installed, and not
/// under a temp directory, which is the one place measurements are expected to
/// disappear from. Temp is the last resort only when the environment names no
/// home at all, because losing the run outright would be worse than putting it
/// somewhere awkward.
fn default_dir() -> PathBuf {
    let home = std::env::var_os("USERPROFILE")
        .or_else(|| std::env::var_os("HOME"))
        .map(PathBuf::from);
    match home {
        Some(h) => h.join("misa-actuator-data"),
        None => std::env::temp_dir().join("misa-actuator-data"),
    }
}

/// Where the remembered choice is kept.
///
/// Beside the default output directory rather than in a config directory of its
/// own: one place to look, and a settings file that travels with the data it
/// describes is easier to reason about than one that does not.
fn settings_path() -> PathBuf {
    default_dir().join("output-dir.txt")
}

/// The directory the operator last chose, if it is still usable.
///
/// A remembered path that has since been deleted or unmounted returns `None`
/// rather than being honoured: a run written into a recreated stub of a path that
/// used to be a network share is a run in a place nobody will look. Falling back
/// to the default is visible in the UI, which says whether the location was
/// chosen.
fn remembered_dir() -> Option<PathBuf> {
    let text = fs::read_to_string(settings_path()).ok()?;
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return None;
    }
    let dir = PathBuf::from(trimmed);
    dir.is_dir().then_some(dir)
}

/// Remember `dir` as where runs go, or forget the choice with `None`.
///
/// Verified before it is stored — a path that cannot be created is not a setting,
/// it is a run that will fail later with nothing on screen explaining why.
pub fn set_data_dir(dir: Option<PathBuf>) -> io::Result<()> {
    let path = settings_path();
    match dir {
        Some(dir) => {
            fs::create_dir_all(&dir)?;
            fs::create_dir_all(default_dir())?;
            fs::write(path, dir.display().to_string())
        }
        None => match fs::remove_file(&path) {
            Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(()),
            other => other,
        },
    }
}

/// `YYYYmmdd-HHMMSS` in UTC, from seconds since the epoch.
///
/// UTC, and the file name says so, because the alternative is a local time this
/// crate cannot obtain without a dependency — and a timestamp whose zone is
/// unstated is worse than one in a zone nobody lives in. Runs sort correctly
/// either way, which is what the name is for.
pub fn utc_stamp(secs: u64) -> String {
    let (days, rem) = ((secs / 86_400) as i64, secs % 86_400);
    let (h, m, s) = (rem / 3600, (rem % 3600) / 60, rem % 60);
    let (y, mo, d) = civil_from_days(days);
    format!("{y:04}{mo:02}{d:02}-{h:02}{m:02}{s:02}")
}

/// Days since 1970-01-01 to a calendar date.
///
/// Howard Hinnant's `civil_from_days`, which is exact over the range any file
/// name will ever see and needs no table. Written out rather than pulled in: the
/// workspace has no direct date dependency and one timestamp does not justify
/// the first.
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64; // [0, 146096]
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365; // [0, 399]
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // [0, 365]
    let mp = (5 * doy + 2) / 153; // [0, 11]
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32; // [1, 31]
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32; // [1, 12]
    (if m <= 2 { y + 1 } else { y }, m, d)
}

/// Seconds since the epoch, or 0 if the clock is before it.
pub fn now_secs() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

/// Make `text` safe to put in a file name.
///
/// Run names contain spaces ("breakaway map"), and a caller could pass anything;
/// a name that lands outside the intended directory because it contained a slash
/// would be a path traversal written by our own code.
fn slug(text: &str) -> String {
    let s: String = text
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() {
                c.to_ascii_lowercase()
            } else {
                '-'
            }
        })
        .collect();
    // Collapse runs of dashes and trim them, so "breakaway map" is
    // "breakaway-map" and not "breakaway--map".
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        if c == '-' && out.ends_with('-') {
            continue;
        }
        out.push(c);
    }
    let trimmed = out.trim_matches('-');
    if trimmed.is_empty() {
        "run".to_string()
    } else {
        trimmed.to_string()
    }
}

/// `YYYYmmdd_HHMM` in UTC — the run-group folder's stamp.
///
/// Minutes, not seconds: the folder holds every run of one execution, and a
/// batch's runs are seconds apart, so a second-resolution name would make one
/// folder per run and defeat the grouping. See [`begin_run_group`] for why the
/// stamp is taken once rather than per run.
pub fn utc_stamp_minute(secs: u64) -> String {
    let (days, rem) = ((secs / 86_400) as i64, secs % 86_400);
    let (h, m) = (rem / 3600, (rem % 3600) / 60);
    let (y, mo, d) = civil_from_days(days);
    format!("{y:04}{mo:02}{d:02}_{h:02}{m:02}")
}

/// The folder one execution's files go in: `id5_20260812_0802Z`.
///
/// `Z` for the same reason [`utc_stamp`] carries it — a timestamp whose zone is
/// unstated is worse than one in a zone nobody lives in, and this crate cannot
/// obtain local time without a dependency.
pub fn run_dir_name(motor_id: u8, secs: u64) -> String {
    format!("id{motor_id}_{}Z", utc_stamp_minute(secs))
}

/// The stamp shared by every run of the execution in progress.
///
/// Process-wide because the alternative is threading a group id through
/// `JobSpec` → `Command` → the worker, and the worker is the only thing that
/// writes files: three types would gain a field that only one function reads.
/// [`data_dir`] is already process-wide state consulted at write time, so a
/// reader of this module has one pattern to learn rather than two.
static RUN_GROUP: std::sync::Mutex<Option<u64>> = std::sync::Mutex::new(None);

/// Fix one folder stamp for every run that follows, until [`end_run_group`].
///
/// Taken once at the start rather than per run, because a batch straddles a
/// minute boundary routinely — the RS-04 batch of 2026-08-12 ran 08:02:49
/// through 08:03:09, which a per-run stamp would have split across two folders
/// with three runs in one and two in the other.
///
/// The motor id is *not* part of the stamp: a batch walks several motors and
/// they should share a timestamp, which [`run_dir_name`] then separates by id.
pub fn begin_run_group(secs: u64) {
    if let Ok(mut g) = RUN_GROUP.lock() {
        *g = Some(secs);
    }
}

/// Release the fixed stamp, so later single runs each get their own folder.
pub fn end_run_group() {
    if let Ok(mut g) = RUN_GROUP.lock() {
        *g = None;
    }
}

/// Where this motor's run should be written: the group's folder, or a fresh one.
///
/// A single run outside a batch has no group, so it gets a folder of its own
/// named for the minute it happened in. Two single runs started inside the same
/// minute land together, which is the same answer grouping would have given.
pub fn run_dir(motor_id: u8) -> PathBuf {
    let secs = RUN_GROUP
        .lock()
        .ok()
        .and_then(|g| *g)
        .unwrap_or_else(now_secs);
    data_dir().join(run_dir_name(motor_id, secs))
}

/// The file name a run is saved under, without a directory.
pub fn run_file_name(stamp: &str, run: &str, motor_id: u8) -> String {
    format!("{stamp}Z-{}-id{motor_id}.csv", slug(run))
}

/// The PNG beside a run's CSV, named so the pair sorts together.
///
/// Derived from the CSV's name rather than rebuilt from the stamp, so the two
/// cannot drift apart: the plot is only meaningful as the picture of *that*
/// file's numbers, and a mismatched pair would be worse than no picture.
pub fn png_file_name(csv_file_name: &str) -> String {
    match csv_file_name.strip_suffix(".csv") {
        Some(base) => format!("{base}.png"),
        None => format!("{csv_file_name}.png"),
    }
}

/// Write a plot rendered by the front end next to its run.
///
/// The bytes come from the webview's canvas because that is where the plot
/// exists — re-plotting in Rust would mean a second implementation of the axes,
/// the two-leg merge and the y-range padding, which would then disagree with
/// what the operator saw. `file_name` is slugged by the caller via
/// [`png_file_name`]; it is joined, never trusted, so the traversal test that
/// guards [`write_run_csv`] covers this too.
pub fn write_run_png(dir: &Path, file_name: &str, bytes: &[u8]) -> io::Result<PathBuf> {
    if file_name.contains('/') || file_name.contains('\\') || file_name.contains("..") {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("refusing to write a plot to {file_name:?}"),
        ));
    }
    fs::create_dir_all(dir)?;
    let path = dir.join(file_name);
    fs::write(&path, bytes)?;
    Ok(path)
}

/// Write one run: `#` metadata lines, then the samples.
///
/// Returns the path written. Creates `dir` if it is missing — the first run of a
/// fresh install would otherwise fail on a directory nobody was asked to make.
pub fn write_run_csv(
    dir: &Path,
    file_name: &str,
    run: &str,
    motor_id: u8,
    description: &str,
    summary: &[(String, String)],
    points: &[misa_sysid::Point],
) -> io::Result<PathBuf> {
    fs::create_dir_all(dir)?;
    let path = dir.join(file_name);
    let mut w = BufWriter::new(fs::File::create(&path)?);

    // Metadata first, so a file found on its own still says what it is. A CSV
    // with only numbers in it is indistinguishable from any other run six
    // months later.
    writeln!(w, "# run,{run}")?;
    writeln!(w, "# motor_id,{motor_id}")?;
    if !description.is_empty() {
        writeln!(w, "# connection,{description}")?;
    }
    writeln!(w, "# samples,{}", points.len())?;
    for (k, v) in summary {
        // The findings, keyed as they appear on screen, so a number quoted from
        // a plot can be traced to the file it came from.
        writeln!(w, "# {},{}", slug(k), v)?;
    }
    misa_sysid::write_points_csv(points, &mut w, true)?;
    w.flush()?;
    Ok(path)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Serialises the tests that touch process-wide state.
    ///
    /// Two of them do: one sets the environment variable, the other writes the
    /// settings file, and `data_dir()` consults both. Run in parallel they see
    /// each other's setup and fail on assertions about the fallback — observed
    /// 2026-08-08, passing under `--test-threads=1` and failing without it,
    /// which is the signature of exactly this.
    ///
    /// `pub(crate)` because `batch`'s output-layout test redirects the same
    /// variable, and a second lock would serialise each module against itself
    /// while still letting the two collide.
    pub(crate) static ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    #[test]
    fn the_stamp_is_a_real_utc_date() {
        // 2026-08-08T04:15:30Z. Cross-checked by hand: 1970-01-01 to 2026-01-01
        // is 20454 days (56 x 365 + 14 leap days), and 20673 - 20454 = 219 days
        // into a non-leap year, which is 8 August.
        assert_eq!(utc_stamp(1_786_162_530), "20260808-041530");
        // Epoch itself, and a leap day, which is where a hand-rolled calendar
        // goes wrong if it is going to.
        assert_eq!(utc_stamp(0), "19700101-000000");
        assert_eq!(utc_stamp(1_709_164_800), "20240229-000000");
        // A century boundary that is not a leap year.
        assert_eq!(utc_stamp(4_107_542_400), "21000301-000000");
    }

    /// Stamps must sort in time order as plain text, since that is how a file
    /// listing orders them.
    #[test]
    fn stamps_sort_chronologically_as_text() {
        let mut v = vec![utc_stamp(2_000_000_000), utc_stamp(0), utc_stamp(1_786_162_530)];
        v.sort();
        assert_eq!(v, vec![utc_stamp(0), utc_stamp(1_786_162_530), utc_stamp(2_000_000_000)]);
    }

    #[test]
    fn a_run_name_cannot_escape_its_directory() {
        // Spaces become dashes, and nothing that could redirect the write
        // survives.
        assert_eq!(slug("breakaway map"), "breakaway-map");
        assert_eq!(slug("../../etc/passwd"), "etc-passwd");
        assert_eq!(slug("..\\..\\windows"), "windows");
        assert_eq!(slug(""), "run");
        assert_eq!(slug("///"), "run");

        let name = run_file_name("20260807-041530", "../oops", 96);
        assert!(!name.contains('/') && !name.contains('\\'), "{name}");
        assert_eq!(name, "20260807-041530Z-oops-id96.csv");
    }

    /// The folder carries the motor and the minute, and says which zone.
    #[test]
    fn a_run_folder_names_the_motor_and_the_minute() {
        // 2026-08-12 08:02:50 UTC — the RS-04 batch this grouping was built for.
        let secs = 1_786_521_770;
        assert_eq!(utc_stamp_minute(secs), "20260812_0802");
        assert_eq!(run_dir_name(5, secs), "id5_20260812_0802Z");
    }

    /// The whole point of the group: runs seconds apart either side of a minute
    /// boundary land in one folder, which a per-run stamp would have split.
    #[test]
    fn a_group_holds_a_batch_that_crosses_a_minute() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        // 08:02:49 and 08:03:09 — the real span of the 2026-08-12 batch.
        let start = 1_786_521_769;
        begin_run_group(start);
        let first = run_dir(5);
        let last = run_dir(5);
        end_run_group();

        assert_eq!(first, last, "one execution must be one folder");
        assert!(
            first.ends_with("id5_20260812_0802Z"),
            "{}",
            first.display()
        );

        // Two motors in the same batch share the stamp and differ by id.
        begin_run_group(start);
        let a = run_dir(5);
        let b = run_dir(6);
        end_run_group();
        assert_ne!(a, b);
        assert!(b.ends_with("id6_20260812_0802Z"), "{}", b.display());
    }

    /// A plot is named for the run it pictures, so the pair cannot drift apart.
    #[test]
    fn a_plot_is_named_after_its_run() {
        assert_eq!(
            png_file_name("20260812-080250Z-load-map-id5.csv"),
            "20260812-080250Z-load-map-id5.png"
        );
        // Not a CSV: still gets a suffix rather than losing one.
        assert_eq!(png_file_name("odd-name"), "odd-name.png");
    }

    #[test]
    fn a_plot_cannot_escape_its_directory() {
        let dir = std::env::temp_dir().join(format!("misa-png-test-{}", now_secs()));
        for bad in ["../oops.png", r"..\oops.png", "sub/oops.png"] {
            let e = write_run_png(&dir, bad, b"x").expect_err("must refuse");
            assert_eq!(e.kind(), io::ErrorKind::InvalidInput, "{bad}");
        }

        // And a good name lands, byte for byte.
        let path = write_run_png(&dir, "plot.png", b"\x89PNG\r\n\x1a\n").unwrap();
        assert_eq!(fs::read(&path).unwrap(), b"\x89PNG\r\n\x1a\n");
        let _ = fs::remove_dir_all(&dir);
    }

    /// Serialised, because it mutates process-wide environment state and the
    /// test runner is threaded.
    #[test]
    fn the_data_directory_can_be_pointed_somewhere_else() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|p| p.into_inner());
        // Any remembered directory would win the fallback assertions below, so
        // this test owns that state too for its duration.
        let restore_file = fs::read_to_string(settings_path()).ok();
        set_data_dir(None).ok();
        let restore = std::env::var_os(DATA_DIR_ENV);
        let set = |v: Option<&str>| match v {
            Some(v) => std::env::set_var(DATA_DIR_ENV, v),
            None => std::env::remove_var(DATA_DIR_ENV),
        };

        set(Some(r"D:\campaign-2026-08"));
        assert_eq!(data_dir(), PathBuf::from(r"D:\campaign-2026-08"));

        // An exported-but-empty variable is a shell accident, not a request to
        // write into the working directory.
        set(Some(""));
        assert!(data_dir().ends_with("misa-actuator-data"), "{:?}", data_dir());
        set(Some("   "));
        assert!(data_dir().ends_with("misa-actuator-data"), "{:?}", data_dir());

        set(None);
        assert!(data_dir().ends_with("misa-actuator-data"), "{:?}", data_dir());

        set(restore.as_deref().and_then(|s| s.to_str()));
        if let Some(text) = restore_file {
            fs::write(settings_path(), text).ok();
        }
    }

    /// A remembered directory has to survive a restart, or picking one is
    /// theatre — and a remembered one that has gone away must not be honoured.
    #[test]
    fn a_chosen_directory_is_remembered_and_a_vanished_one_is_not() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|p| p.into_inner());
        // The settings file lives beside the default output directory, so this
        // touches real paths. Restored at the end.
        let restore = fs::read_to_string(settings_path()).ok();
        let chosen = std::env::temp_dir().join(format!("misa-chosen-{}", now_secs()));

        set_data_dir(Some(chosen.clone())).expect("remember");
        assert_eq!(remembered_dir(), Some(chosen.clone()));
        assert!(data_dir_is_chosen());

        // Gone since it was chosen — a network share that is no longer mounted,
        // say. Honouring it would write runs into a recreated stub nobody looks
        // in, so it falls back and the UI can say the location is not chosen.
        fs::remove_dir_all(&chosen).expect("remove");
        assert_eq!(remembered_dir(), None);
        assert_eq!(data_dir(), default_dir());

        set_data_dir(None).expect("forget");
        assert_eq!(remembered_dir(), None);
        // Forgetting twice is not an error.
        set_data_dir(None).expect("forget again");

        if let Some(text) = restore {
            fs::write(settings_path(), text).ok();
        }
    }

    #[test]
    fn a_written_run_carries_its_metadata_and_its_samples() {
        let dir = std::env::temp_dir().join(format!("misa-export-test-{}", now_secs()));
        let points = vec![misa_sysid::Point {
            t_s: 0.5,
            cmd: 0.2,
            position_rad: 1.0,
            velocity_rad_per_s: 0.05,
            torque_nm: 0.3,
            current_a: 0.19,
            temperature_c: 31.0,
        }];
        let path = write_run_csv(
            &dir,
            "run.csv",
            "velocity sweep",
            96,
            "robstride RS-03 on slcan:COM11",
            &[("kinetic friction".to_string(), "0.581 N·m".to_string())],
            &points,
        )
        .expect("write");

        let text = fs::read_to_string(&path).expect("read back");
        assert!(text.contains("# run,velocity sweep"), "{text}");
        assert!(text.contains("# motor_id,96"), "{text}");
        assert!(text.contains("# kinetic-friction,0.581 N·m"), "{text}");
        assert!(text.contains(misa_sysid::POINTS_CSV_HEADER), "{text}");
        // The sample row, which is the point of the file.
        assert!(text.contains("0.500000,0.200000,1.000000"), "{text}");

        fs::remove_dir_all(&dir).ok();
    }
}
