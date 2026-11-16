// SPDX-License-Identifier: BSD-3-Clause
//! Command-line front end of qemu-capture; see README.md.

use std::ffi::OsString;
use std::fs::File;
use std::io::BufWriter;
use std::os::unix::ffi::OsStringExt;
use std::path::PathBuf;
use std::process::{Command, ExitCode};
use std::time::Duration;

use qemu_capture::{Checks, Failure, Limits, Summary, Verdict, run};

const USAGE: &str = "\
Usage: qemu-capture --log PATH --timeout SECONDS --pass TEXT [--fail TEXT]
                    [--summary TEMPLATE] [--fail-line PREFIX] -- COMMAND [ARG...]

Runs COMMAND, saves everything it prints to PATH, and decides a verdict from
that output. The first of the two markers to appear ends the wait. The run
passes only on --pass TEXT, and only if the output before it holds the
--summary, with a nonzero {passed} equal to {total} when the template has it,
and no line that starts with the --fail-line PREFIX. Then Ctrl-A x is written
to COMMAND's input to quit QEMU.

TEMPLATE is literal text with fields that match a number: {passed} (required),
{total} (optional) and {_} (ignored).

Exit status: 0 pass, 1 fail, 2 usage error or the run could not be done.";

/// After the verdict, how long QEMU has to quit before it is signalled.
const GRACE: Duration = Duration::from_secs(10);

struct Args {
    log: PathBuf,
    timeout: Duration,
    checks: Checks,
    command: Vec<OsString>,
}

fn main() -> ExitCode {
    let args = match parse(std::env::args_os().skip(1).collect()) {
        Ok(Some(args)) => args,
        Ok(None) => {
            println!("{USAGE}");
            return ExitCode::SUCCESS;
        }
        Err(message) => {
            eprintln!("qemu-capture: {message}\n\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    let log = match File::create(&args.log) {
        Ok(file) => BufWriter::new(file),
        Err(error) => {
            eprintln!(
                "qemu-capture: cannot create {}: {error}",
                args.log.display()
            );
            return ExitCode::from(2);
        }
    };
    let mut command = Command::new(&args.command[0]);
    command.args(&args.command[1..]);
    let limits = Limits {
        timeout: args.timeout,
        grace: GRACE,
    };
    let outcome = match run(command, &args.checks, log, limits) {
        Ok(outcome) => outcome,
        Err(error) => {
            eprintln!(
                "qemu-capture: {}: {error}; see {}",
                args.command[0].to_string_lossy(),
                args.log.display()
            );
            return ExitCode::from(2);
        }
    };
    match outcome.verdict {
        Verdict::Pass(counts) => {
            match counts {
                Some(counts) => {
                    println!("{}", counts.text);
                    println!("PASS: summary and success marker observed");
                }
                None => println!("PASS: success marker observed"),
            }
            ExitCode::SUCCESS
        }
        Verdict::Fail(failure) => {
            let reason = match failure {
                Failure::Ended => format!("{failure} ({})", outcome.status),
                Failure::TimedOut => format!("{failure} ({} s)", args.timeout.as_secs()),
                failure => failure.to_string(),
            };
            eprintln!("FAIL: {reason}; see {}", args.log.display());
            ExitCode::FAILURE
        }
    }
}

/// Parses the arguments; `None` asks for the usage text.
fn parse(args: Vec<OsString>) -> Result<Option<Args>, String> {
    let mut log = None;
    let mut timeout = None;
    let mut pass = None;
    let mut fail = None;
    let mut summary = None;
    let mut fail_line = None;
    let mut args = args.into_iter();
    loop {
        let Some(arg) = args.next() else {
            return Err("no command: put it after --".into());
        };
        let name = arg.to_string_lossy().into_owned();
        if name == "--" {
            break;
        }
        if name == "-h" || name == "--help" {
            return Ok(None);
        }
        let slot = match name.as_str() {
            "--log" => &mut log,
            "--timeout" => &mut timeout,
            "--pass" => &mut pass,
            "--fail" => &mut fail,
            "--summary" => &mut summary,
            "--fail-line" => &mut fail_line,
            _ => return Err(format!("unknown option {name}")),
        };
        if slot.is_some() {
            return Err(format!("{name} given twice"));
        }
        let value = args.next().ok_or_else(|| format!("{name} needs a value"))?;
        if value.is_empty() {
            return Err(format!("{name} needs a nonempty value"));
        }
        *slot = Some(value);
    }
    let command: Vec<OsString> = args.collect();
    if command.is_empty() {
        return Err("no command after --".into());
    }

    let log = log.ok_or("--log is required")?;
    let timeout = timeout.ok_or("--timeout is required")?;
    let seconds = timeout
        .to_str()
        .filter(|s| s.bytes().all(|b| b.is_ascii_digit()))
        .and_then(|s| s.parse::<u64>().ok())
        .filter(|&s| s > 0)
        .ok_or("--timeout must be a positive whole number of seconds")?;
    let pass = pass.ok_or("--pass is required")?;
    let summary = match summary {
        None => None,
        Some(template) => {
            let template = template.to_str().ok_or("--summary must be UTF-8")?;
            Some(Summary::parse(template).map_err(|error| format!("--summary: {error}"))?)
        }
    };
    Ok(Some(Args {
        log: log.into(),
        timeout: Duration::from_secs(seconds),
        checks: Checks {
            pass_marker: pass.into_vec(),
            fail_marker: fail.map(OsString::into_vec),
            summary,
            fail_line: fail_line.map(OsString::into_vec),
        },
        command,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse_strs(args: &[&str]) -> Result<Option<Args>, String> {
        parse(args.iter().map(OsString::from).collect())
    }

    const GOOD: [&str; 12] = [
        "--log",
        "out.log",
        "--timeout",
        "120",
        "--pass",
        "OK",
        "--summary",
        "{passed}/{total}",
        "--",
        "qemu",
        "-nographic",
        "--pass",
    ];

    #[test]
    fn takes_options_then_the_command() {
        let args = parse_strs(&GOOD).unwrap().unwrap();
        assert_eq!(args.log, PathBuf::from("out.log"));
        assert_eq!(args.timeout, Duration::from_secs(120));
        assert_eq!(args.checks.pass_marker, b"OK");
        assert_eq!(args.checks.fail_marker, None);
        assert_eq!(args.command, ["qemu", "-nographic", "--pass"]);
    }

    #[test]
    fn refuses_what_the_old_scripts_refused() {
        for bad in ["0", "-1", "1.5", "", "ten", "+5"] {
            let mut args = GOOD;
            args[3] = bad;
            assert!(parse_strs(&args).is_err(), "timeout {bad:?}");
        }
    }

    #[test]
    fn refuses_missing_or_repeated_options() {
        assert!(parse_strs(&GOOD[2..]).is_err(), "no --log");
        assert!(parse_strs(&GOOD[..9]).is_err(), "no command");
        let mut twice = GOOD.to_vec();
        twice.splice(0..0, ["--pass", "X"]);
        assert!(parse_strs(&twice).is_err(), "--pass twice");
        assert!(parse_strs(&["--bogus", "1", "--", "x"]).is_err());
    }
}
