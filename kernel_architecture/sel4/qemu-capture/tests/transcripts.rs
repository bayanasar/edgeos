// SPDX-License-Identifier: BSD-3-Clause
//! Recorded QEMU output fed through the matcher, with the checks the seL4
//! scripts pass on the command line.
//!
//! Every transcript in `transcripts/` is the complete output of a real run.
//! The `-pass` ones were recorded through a terminal, as the earlier capture
//! scripts did, so their lines end in `\r\r\n`; the rest were recorded
//! through a pipe, as qemu-capture does, and end in `\r\n`.
//!
//! - `sel4test-pass`: the pinned seL4test build.
//! - `sel4test-one-test-fails`: the same with `test_trivial` changed to
//!   assert `1 >= 2`.
//! - `iommu-gate-pass`: the IOMMU gate.
//! - `iommu-gate-narrow-dma`: the gate with both `edu` devices left at their
//!   default DMA mask, so they cannot reach every address the checks aim at.
//! - `iommu-gate-verdict-forced`: the same QEMU, with the gate changed to
//!   print `IOMMU_GATE: PASS` whatever its count.
//! - `iommu-gate-failures-counted`: the same QEMU, with the gate changed to
//!   count a failed check as passed, while still printing its `FAIL:` line.
//! - `iommu-gate-timeout`: the gate stopped by a 2 s timeout.
//! - `iommu-gate-no-image`: QEMU started with a kernel image that does not
//!   exist.
//! - `zero-copy-pass`: the zero-copy handoff.
//! - `zero-copy-no-checks`: the handoff changed to skip every step, so it
//!   reports 0/0 and its own marker says PASS.
//! - `zero-copy-qemu-killed`: QEMU sent SIGTERM 1.5 s into the handoff.

use qemu_capture::{Checks, Failure, Summary, Verdict, Watch};

macro_rules! transcript {
    ($name:literal) => {
        include_bytes!(concat!("transcripts/", $name, ".log")).as_slice()
    };
}

/// The checks in `sel4test.sh`.
fn sel4test() -> Checks {
    Checks {
        pass_marker: b"All is well in the universe".to_vec(),
        fail_marker: Some(b"Test suite failed.".to_vec()),
        summary: Some(
            Summary::parse("Test suite passed. {passed} tests passed. {_} tests disabled.")
                .unwrap(),
        ),
        fail_line: None,
    }
}

/// The checks in `iommu_gate.sh`.
fn iommu_gate() -> Checks {
    Checks {
        pass_marker: b"IOMMU_GATE: PASS".to_vec(),
        fail_marker: Some(b"IOMMU_GATE: FAIL".to_vec()),
        summary: Some(Summary::parse("IOMMU gate: {passed}/{total} checks passed").unwrap()),
        fail_line: Some(b"FAIL: ".to_vec()),
    }
}

/// The checks in `zero_copy.sh`.
fn zero_copy() -> Checks {
    Checks {
        pass_marker: b"ZERO_COPY: PASS".to_vec(),
        fail_marker: Some(b"ZERO_COPY: FAIL".to_vec()),
        summary: Some(Summary::parse("Zero copy: {passed}/{total} checks passed").unwrap()),
        fail_line: Some(b"FAIL: ".to_vec()),
    }
}

/// Feeds the transcript in pieces of several sizes, checks that they all
/// agree, and returns the verdict, or `None` if no marker appeared.
fn verdict(checks: &Checks, transcript: &[u8]) -> Option<Verdict> {
    let whole = Watch::new(checks).feed(transcript);
    for size in [1, 2, 7, 64, 4096] {
        let mut watch = Watch::new(checks);
        let pieced = transcript.chunks(size).find_map(|piece| watch.feed(piece));
        assert_eq!(pieced, whole, "in pieces of {size} bytes");
    }
    whole
}

fn summary_of(verdict: Option<Verdict>) -> String {
    match verdict {
        Some(Verdict::Pass(Some(counts))) => counts.text,
        other => panic!("expected a pass with a summary, got {other:?}"),
    }
}

fn failure(verdict: Option<Verdict>) -> Failure {
    match verdict {
        Some(Verdict::Fail(failure)) => failure,
        other => panic!("expected a failure, got {other:?}"),
    }
}

#[test]
fn sel4test_passes() {
    let text = summary_of(verdict(&sel4test(), transcript!("sel4test-pass")));
    assert_eq!(
        text,
        "Test suite passed. 116 tests passed. 51 tests disabled."
    );
}

#[test]
fn iommu_gate_passes() {
    let text = summary_of(verdict(&iommu_gate(), transcript!("iommu-gate-pass")));
    assert_eq!(text, "IOMMU gate: 31/31 checks passed");
}

#[test]
fn zero_copy_passes() {
    let text = summary_of(verdict(&zero_copy(), transcript!("zero-copy-pass")));
    assert_eq!(text, "Zero copy: 37/37 checks passed");
}

#[test]
fn sel4test_failure_marker_fails() {
    let failure = failure(verdict(&sel4test(), transcript!("sel4test-one-test-fails")));
    assert_eq!(failure, Failure::FailMarker("Test suite failed.".into()));
}

#[test]
fn iommu_gate_failure_marker_fails() {
    let failure = failure(verdict(&iommu_gate(), transcript!("iommu-gate-narrow-dma")));
    assert_eq!(failure, Failure::FailMarker("IOMMU_GATE: FAIL".into()));
}

#[test]
fn wrong_check_count_fails_despite_the_success_marker() {
    let transcript = transcript!("iommu-gate-verdict-forced");
    let expected = Failure::ChecksFailed("IOMMU gate: 23/31 checks passed".into());
    assert_eq!(failure(verdict(&iommu_gate(), transcript)), expected);
    // The count alone decides, without the FAIL: lines.
    let checks = Checks {
        fail_line: None,
        ..iommu_gate()
    };
    assert_eq!(failure(verdict(&checks, transcript)), expected);
}

#[test]
fn fail_line_fails_despite_a_full_count() {
    let failure = failure(verdict(
        &iommu_gate(),
        transcript!("iommu-gate-failures-counted"),
    ));
    assert_eq!(
        failure,
        Failure::FailLine("FAIL: write by physical address outside grant refused".into())
    );
}

#[test]
fn zero_checks_fail_despite_the_success_marker() {
    let failure = failure(verdict(&zero_copy(), transcript!("zero-copy-no-checks")));
    assert_eq!(
        failure,
        Failure::NothingPassed("Zero copy: 0/0 checks passed".into())
    );
}

#[test]
fn missing_summary_fails() {
    // The zero-copy marker with the gate's summary: right marker, no summary.
    let checks = Checks {
        summary: iommu_gate().summary,
        ..zero_copy()
    };
    let failure = failure(verdict(&checks, transcript!("zero-copy-pass")));
    assert_eq!(
        failure,
        Failure::NoSummary("IOMMU gate: {passed}/{total} checks passed".into())
    );
}

#[test]
fn no_marker_without_a_verdict() {
    // What run() reports as a timeout or as early exit, depending on whether
    // the output ended.
    for (checks, transcript) in [
        (iommu_gate(), transcript!("iommu-gate-timeout")),
        (iommu_gate(), transcript!("iommu-gate-no-image")),
        (zero_copy(), transcript!("zero-copy-qemu-killed")),
    ] {
        assert_eq!(verdict(&checks, transcript), None);
    }
}

#[test]
fn each_workload_ignores_the_others_markers() {
    for (checks, name, transcript) in [
        (sel4test(), "gate", transcript!("iommu-gate-pass")),
        (iommu_gate(), "zero copy", transcript!("zero-copy-pass")),
        (zero_copy(), "sel4test", transcript!("sel4test-pass")),
    ] {
        assert_eq!(verdict(&checks, transcript), None, "{name}");
    }
}
