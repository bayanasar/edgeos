// SPDX-License-Identifier: BSD-3-Clause
//! The exit status and messages the seL4 scripts rely on: 0 pass, 1 fail,
//! 2 when the run could not be done.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

const GATE: [&str; 8] = [
    "--pass",
    "IOMMU_GATE: PASS",
    "--fail",
    "IOMMU_GATE: FAIL",
    "--summary",
    "IOMMU gate: {passed}/{total} checks passed",
    "--fail-line",
    "FAIL: ",
];

/// Replays a transcript, then exits once it is sent two bytes.
const REPLAY: &str = r#"cat "$0"; od -An -tx1 -N2 >/dev/null"#;

fn transcript(name: &str) -> PathBuf {
    [env!("CARGO_MANIFEST_DIR"), "tests", "transcripts", name]
        .iter()
        .collect()
}

fn scratch(test: &str) -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(test);
    std::fs::create_dir_all(&dir).unwrap();
    dir.join("test.log")
}

fn capture(log: &Path, timeout: &str, extra: &[&str], command: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_qemu-capture"))
        .arg("--log")
        .arg(log)
        .args(["--timeout", timeout])
        .args(extra)
        .arg("--")
        .args(command)
        .output()
        .unwrap()
}

fn replay(log: &Path, timeout: &str, name: &str) -> Output {
    let file = transcript(name);
    let command = ["sh", "-c", REPLAY, file.to_str().unwrap()];
    capture(log, timeout, &GATE, &command)
}

fn text(bytes: &[u8]) -> &str {
    std::str::from_utf8(bytes).unwrap()
}

#[test]
fn pass_exits_0_with_the_summary() {
    let log = scratch("pass");
    let output = replay(&log, "20", "iommu-gate-pass.log");
    assert_eq!(output.status.code(), Some(0));
    assert_eq!(
        text(&output.stdout),
        "IOMMU gate: 31/31 checks passed\nPASS: summary and success marker observed\n"
    );
    assert_eq!(
        std::fs::read(&log).unwrap(),
        std::fs::read(transcript("iommu-gate-pass.log")).unwrap()
    );
}

#[test]
fn failure_exits_1_and_names_the_log() {
    let log = scratch("fail");
    let output = replay(&log, "20", "iommu-gate-verdict-forced.log");
    assert_eq!(output.status.code(), Some(1));
    let expected = format!(
        "FAIL: summary \"IOMMU gate: 23/31 checks passed\" counts checks that did not pass; \
         see {}\n",
        log.display()
    );
    assert_eq!(text(&output.stderr), expected);
}

#[test]
fn timeout_exits_1() {
    let log = scratch("timeout");
    let file = transcript("iommu-gate-timeout.log");
    let script = r#"cat "$0"; sleep 5; od -An -tx1 -N2 >/dev/null"#;
    let command = ["sh", "-c", script, file.to_str().unwrap()];
    let output = capture(&log, "1", &GATE, &command);
    assert_eq!(output.status.code(), Some(1));
    assert!(
        text(&output.stderr).starts_with("FAIL: neither marker appeared before the timeout (1 s)")
    );
}

#[test]
fn early_exit_exits_1_with_the_status() {
    let log = scratch("ended");
    let file = transcript("iommu-gate-no-image.log");
    let command = ["sh", "-c", r#"cat "$0"; exit 1"#, file.to_str().unwrap()];
    let output = capture(&log, "20", &GATE, &command);
    assert_eq!(output.status.code(), Some(1));
    assert!(
        text(&output.stderr)
            .starts_with("FAIL: output ended before either marker (exit status: 1)")
    );
}

#[test]
fn usage_errors_exit_2() {
    let log = scratch("usage");
    for timeout in ["0", "x"] {
        let output = capture(&log, timeout, &GATE, &["true"]);
        assert_eq!(output.status.code(), Some(2), "timeout {timeout}");
    }
    let output = capture(&log, "5", &["--summary", "{passed}"], &["true"]);
    assert_eq!(output.status.code(), Some(2), "no --pass");
}

#[test]
fn a_run_that_cannot_be_done_exits_2() {
    let log = scratch("cannot");
    let output = capture(&log, "5", &GATE, &["/nonexistent/qemu-system-riscv64"]);
    assert_eq!(output.status.code(), Some(2), "missing command");
    let output = capture(Path::new("/nonexistent/test.log"), "5", &GATE, &["true"]);
    assert_eq!(output.status.code(), Some(2), "log cannot be created");
}
