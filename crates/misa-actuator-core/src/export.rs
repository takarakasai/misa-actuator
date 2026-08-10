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
    // Whitespace-only is treated as unset: an exported-but-empty variable is a
    // shell accident, and writing to the process's working directory because of
    // one would scatter runs wherever the app happened to be launched from.
    if let Some(dir) = std::env::var_os(DATA_DIR_ENV) {
        let dir = PathBuf::from(dir);
        if !dir.as_os_str().is_empty() && dir.to_string_lossy().trim() != "" {
            return dir;
        }
    }
    let home = std::env::var_os("USERPROFILE")
        .or_else(|| std::env::var_os("HOME"))
        .map(PathBuf::from);
    match home {
        Some(h) => h.join("misa-actuator-data"),
        None => std::env::temp_dir().join("misa-actuator-data"),
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

/// The file name a run is saved under, without a directory.
pub fn run_file_name(stamp: &str, run: &str, motor_id: u8) -> String {
    format!("{stamp}Z-{}-id{motor_id}.csv", slug(run))
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
mod tests {
    use super::*;

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

    /// Serialised, because it mutates process-wide environment state and the
    /// test runner is threaded.
    #[test]
    fn the_data_directory_can_be_pointed_somewhere_else() {
        // SAFETY-ish: single test touching this variable, and it is restored
        // before returning. Rust 2024 marks `set_var` unsafe for exactly the
        // data race this comment is promising not to cause.
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
