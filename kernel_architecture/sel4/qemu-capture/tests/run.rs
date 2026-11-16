// SPDX-License-Identifier: BSD-3-Clause
//! run() against commands that replay a recorded transcript and then behave
//! as QEMU might: quit on Ctrl-A x, exit early, hang, or ignore SIGTERM.

use std::io::{self, Write};
use std::os::unix::process::ExitStatusExt;
use std::path::PathBuf;
use std::process::Command;
use std::time::{Duration, Instant};

use qemu_capture::{Checks, Failure, Limits, Outcome, Summary, Verdict, run};

fn transcript(name: &str) -> PathBuf {
    [env!("CARGO_MANIFEST_DIR"), "tests", "transcripts", name]
        .iter()
        .collect()
}

fn iommu_gate() -> Checks {
    Checks {
        pass_marker: b"IOMMU_GATE: PASS".to_vec(),
        fail_marker: Some(b"IOMMU_GATE: FAIL".to_vec()),
        summary: Some(Summary::parse("IOMMU gate: {passed}/{total} checks passed").unwrap()),
        fail_line: Some(b"FAIL: ".to_vec()),
    }
}

/// `sh -c SCRIPT TRANSCRIPT`: the script finds the transcript in `$0`.
fn replay(script: &str, name: &str) -> Command {
    let mut command = Command::new("sh");
    command.arg("-c").arg(script).arg(transcript(name));
    command
}

const LIMITS: Limits = Limits {
    timeout: Duration::from_secs(20),
    grace: Duration::from_secs(1),
};

fn capture(command: Command, limits: Limits) -> (Outcome, Vec<u8>) {
    let mut log = Vec::new();
    let outcome = run(command, &iommu_gate(), &mut log, limits).unwrap();
    (outcome, log)
}

fn read(name: &str) -> Vec<u8> {
    std::fs::read(transcript(name)).unwrap()
}

#[test]
fn pass_then_ctrl_a_x_quits() {
    // Prints the two bytes it is sent after the verdict, then exits.
    let script = r#"cat "$0"; echo on-stderr >&2; od -An -tx1 -N2; echo quit"#;
    let (outcome, log) = capture(replay(script, "iommu-gate-pass.log"), LIMITS);
    assert!(matches!(outcome.verdict, Verdict::Pass(Some(_))));
    assert!(outcome.status.success());
    // Both output streams, and what follows the marker, are logged in order.
    let mut expected = read("iommu-gate-pass.log");
    expected.extend_from_slice(b"on-stderr\n 01 78\nquit\n");
    assert_eq!(log, expected);
}

#[test]
fn failure_is_logged_whole() {
    let script = r#"cat "$0"; od -An -tx1 -N2 >/dev/null"#;
    let (outcome, log) = capture(replay(script, "iommu-gate-narrow-dma.log"), LIMITS);
    assert_eq!(
        outcome.verdict,
        Verdict::Fail(Failure::FailMarker("IOMMU_GATE: FAIL".into()))
    );
    assert_eq!(log, read("iommu-gate-narrow-dma.log"));
}

#[test]
fn early_exit_fails() {
    let script = r#"cat "$0"; exit 3"#;
    let (outcome, log) = capture(replay(script, "iommu-gate-no-image.log"), LIMITS);
    assert_eq!(outcome.verdict, Verdict::Fail(Failure::Ended));
    assert_eq!(outcome.status.code(), Some(3));
    assert_eq!(log, read("iommu-gate-no-image.log"));
}

#[test]
fn timeout_fails_and_sigterm_stops_what_ignores_ctrl_a_x() {
    let script = r#"cat "$0"; exec sleep 60"#;
    let limits = Limits {
        timeout: Duration::from_secs(1),
        ..LIMITS
    };
    let start = Instant::now();
    let (outcome, log) = capture(replay(script, "iommu-gate-timeout.log"), limits);
    assert_eq!(outcome.verdict, Verdict::Fail(Failure::TimedOut));
    assert_eq!(outcome.status.signal(), Some(15));
    assert!(
        start.elapsed() < Duration::from_secs(10),
        "{:?}",
        start.elapsed()
    );
    assert_eq!(log, read("iommu-gate-timeout.log"));
}

#[test]
fn sigkill_stops_what_ignores_sigterm_and_the_verdict_stands() {
    let script = r#"trap "" TERM; cat "$0"; exec sleep 60"#;
    let start = Instant::now();
    let (outcome, _) = capture(replay(script, "iommu-gate-pass.log"), LIMITS);
    assert!(matches!(outcome.verdict, Verdict::Pass(Some(_))));
    assert_eq!(outcome.status.signal(), Some(9));
    assert!(
        start.elapsed() < Duration::from_secs(15),
        "{:?}",
        start.elapsed()
    );
}

/// A log that refuses every write.
struct Full;

impl Write for Full {
    fn write(&mut self, _: &[u8]) -> io::Result<usize> {
        Err(io::Error::from(io::ErrorKind::StorageFull))
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[test]
fn log_error_is_reported_after_qemu_is_stopped() {
    let script = r#"cat "$0"; od -An -tx1 -N2 >/dev/null; echo quit"#;
    let command = replay(script, "iommu-gate-pass.log");
    let error = run(command, &iommu_gate(), Full, LIMITS).unwrap_err();
    assert_eq!(error.kind(), io::ErrorKind::StorageFull);
}

#[test]
fn missing_command_is_an_error() {
    let command = Command::new("/nonexistent/qemu-system-riscv64");
    let error = run(command, &iommu_gate(), io::sink(), LIMITS).unwrap_err();
    assert_eq!(error.kind(), io::ErrorKind::NotFound);
}
