// SPDX-License-Identifier: BSD-3-Clause
//! Run a QEMU test under a command, keep everything it prints, and decide
//! its verdict from that output.
//!
//! The output is watched for a success marker and a failure marker. The
//! first marker to appear ends the watch. The failure marker, the end of the
//! output and a timeout each fail the run. The success marker passes it only
//! if the output before the marker also holds a [`Summary`] that counts every
//! check as passed, when one is required, and no line that starts with the
//! failure prefix, when one is given.
//!
//! [`Watch`] decides the verdict from the output alone, so recorded output
//! can be fed to it in tests. [`run()`] runs the command, saves its output,
//! applies the timeout and then stops QEMU.

mod run;
mod summary;

use std::fmt;

pub use run::{CTRL_A_X, Limits, Outcome, run};
pub use summary::{Counts, Summary, TemplateError};

/// What the output must show for the run to pass.
#[derive(Clone, Debug)]
pub struct Checks {
    /// Text that ends the watch as a pass, if the checks below hold.
    pub pass_marker: Vec<u8>,
    /// Text that ends the watch as a failure.
    pub fail_marker: Option<Vec<u8>>,
    /// A summary that must appear before the success marker, with a
    /// nonzero `{passed}` count equal to `{total}` when it has one.
    pub summary: Option<Summary>,
    /// No line before the success marker may start with this text.
    pub fail_line: Option<Vec<u8>>,
}

/// The result of a run.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Every check held. Holds the summary found, if one was required.
    Pass(Option<Counts>),
    /// The run failed, for this reason.
    Fail(Failure),
}

/// Why a run failed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Failure {
    /// The failure marker (given) appeared before the success marker.
    FailMarker(String),
    /// No summary matching the template (given) preceded the success marker.
    NoSummary(String),
    /// The summary (given) counted no passing checks.
    NothingPassed(String),
    /// The summary (given) counted fewer passing checks than it ran.
    ChecksFailed(String),
    /// This line, starting with the failure prefix, preceded the success
    /// marker.
    FailLine(String),
    /// The output ended before either marker.
    Ended,
    /// Neither marker appeared before the timeout.
    TimedOut,
}

impl fmt::Display for Failure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::FailMarker(marker) => write!(f, "failure marker \"{marker}\" observed"),
            Self::NoSummary(template) => {
                write!(f, "no summary \"{template}\" before the success marker")
            }
            Self::NothingPassed(text) => write!(f, "summary \"{text}\" counts no passes"),
            Self::ChecksFailed(text) => {
                write!(f, "summary \"{text}\" counts checks that did not pass")
            }
            Self::FailLine(line) => write!(f, "\"{line}\" before the success marker"),
            Self::Ended => write!(f, "output ended before either marker"),
            Self::TimedOut => write!(f, "neither marker appeared before the timeout"),
        }
    }
}

impl Checks {
    /// The verdict when the success marker follows `before`.
    pub fn judge(&self, before: &[u8]) -> Verdict {
        let counts = match &self.summary {
            None => None,
            Some(summary) => {
                let Some(counts) = summary.find(before) else {
                    return Verdict::Fail(Failure::NoSummary(summary.to_string()));
                };
                if counts.passed == 0 {
                    return Verdict::Fail(Failure::NothingPassed(counts.text));
                }
                if counts.total.is_some_and(|total| total != counts.passed) {
                    return Verdict::Fail(Failure::ChecksFailed(counts.text));
                }
                Some(counts)
            }
        };
        if let Some(prefix) = &self.fail_line {
            let mut lines = before.split(|&b| b == b'\n');
            if let Some(line) = lines.find(|line| line.starts_with(prefix)) {
                let line = line.strip_suffix(b"\r").unwrap_or(line);
                return Verdict::Fail(Failure::FailLine(text(line)));
            }
        }
        Verdict::Pass(counts)
    }
}

/// Watches output as it arrives, until a marker decides the verdict.
#[derive(Debug)]
pub struct Watch<'a> {
    checks: &'a Checks,
    output: Vec<u8>,
    searched: usize,
}

impl<'a> Watch<'a> {
    pub fn new(checks: &'a Checks) -> Self {
        Self {
            checks,
            output: Vec::new(),
            searched: 0,
        }
    }

    /// Adds the next piece of output. Returns the verdict once a marker has
    /// appeared; the output after that marker is not examined, so stop
    /// feeding then. Where pieces are cut does not change the verdict.
    pub fn feed(&mut self, piece: &[u8]) -> Option<Verdict> {
        self.output.extend_from_slice(piece);
        let fail_marker = self.checks.fail_marker.as_deref().unwrap_or_default();
        let longest = self.checks.pass_marker.len().max(fail_marker.len());
        // A marker may straddle two pieces: search again from where one
        // could have started in the output already seen.
        let from = self.searched.saturating_sub(longest.saturating_sub(1));
        self.searched = self.output.len();
        let window = &self.output[from..];
        let pass = find(window, &self.checks.pass_marker);
        let fail = (!fail_marker.is_empty())
            .then(|| find(window, fail_marker))
            .flatten();
        // The marker that starts first wins; the success marker on a tie.
        match (pass, fail) {
            (None, None) => None,
            (Some(pass), fail) if fail.is_none_or(|fail| pass <= fail) => {
                Some(self.checks.judge(&self.output[..from + pass]))
            }
            _ => Some(Verdict::Fail(Failure::FailMarker(text(fail_marker)))),
        }
    }
}

fn find(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    if needle.is_empty() {
        return Some(0);
    }
    haystack
        .windows(needle.len())
        .position(|window| window == needle)
}

fn text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes).into_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn checks() -> Checks {
        Checks {
            pass_marker: b"GATE: PASS".to_vec(),
            fail_marker: Some(b"GATE: FAIL".to_vec()),
            summary: Some(Summary::parse("gate: {passed}/{total} checks passed").unwrap()),
            fail_line: Some(b"FAIL: ".to_vec()),
        }
    }

    #[test]
    fn marker_split_across_pieces_is_found() {
        let checks = checks();
        let mut watch = Watch::new(&checks);
        assert_eq!(watch.feed(b"gate: 2/2 checks passed\nGATE: P"), None);
        assert!(matches!(watch.feed(b"ASS\n"), Some(Verdict::Pass(_))));
    }

    #[test]
    fn earlier_marker_wins_within_one_piece() {
        let checks = checks();
        let mut watch = Watch::new(&checks);
        let verdict = watch.feed(b"GATE: FAIL\ngate: 2/2 checks passed\nGATE: PASS\n");
        assert_eq!(
            verdict,
            Some(Verdict::Fail(Failure::FailMarker("GATE: FAIL".into())))
        );
    }

    #[test]
    fn summary_after_the_marker_does_not_count() {
        let checks = checks();
        let mut watch = Watch::new(&checks);
        let verdict = watch.feed(b"GATE: PASS\ngate: 2/2 checks passed\n");
        assert!(matches!(
            verdict,
            Some(Verdict::Fail(Failure::NoSummary(_)))
        ));
    }
}
