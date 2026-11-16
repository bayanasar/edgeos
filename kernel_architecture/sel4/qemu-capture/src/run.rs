// SPDX-License-Identifier: BSD-3-Clause
//! Running the command: one pipe for its output, the timeout, and stopping
//! QEMU afterwards.

use std::io::{self, PipeReader, Read, Write};
use std::process::{Child, ChildStdin, Command, ExitStatus, Stdio};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, TryRecvError};
use std::thread;
use std::time::{Duration, Instant};

use crate::{Checks, Failure, Verdict, Watch};

/// QEMU's escape to quit when its console is on standard input: Ctrl-A, x.
pub const CTRL_A_X: [u8; 2] = [0x01, b'x'];

/// How long to wait for SIGTERM to work before SIGKILL.
const TERM_WAIT: Duration = Duration::from_secs(5);
/// How long to wait for the last output once the command has exited.
const LAST_OUTPUT_WAIT: Duration = Duration::from_secs(1);
/// How often to look whether the command has exited.
const POLL: Duration = Duration::from_millis(50);

/// Time limits for [`run()`].
#[derive(Clone, Copy, Debug)]
pub struct Limits {
    /// From the start of the command until a marker must have appeared.
    pub timeout: Duration,
    /// After the verdict, how long the command has to exit after Ctrl-A x
    /// before it is sent SIGTERM, and SIGKILL after a further 5 seconds.
    pub grace: Duration,
}

/// What [`run()`] found.
#[derive(Debug)]
pub struct Outcome {
    pub verdict: Verdict,
    /// How the command ended.
    pub status: ExitStatus,
}

/// Runs `command`, writes everything it prints to `log`, and returns the
/// verdict of `checks` on that output.
///
/// The command's standard output and standard error share one pipe, so the
/// log keeps the order in which the two were written, byte for byte. Its
/// standard input is a pipe too. Once the verdict is decided, Ctrl-A x is
/// written to it, which quits QEMU when its monitor is multiplexed on
/// standard input (as with `-nographic`). The output that follows is logged
/// as well. A command that is still running after `limits.grace` is sent
/// SIGTERM, then SIGKILL. The verdict does not depend on how it ends.
///
/// An error means the command could not be started or the log could not be
/// written; the command has been stopped in either case.
pub fn run(
    mut command: Command,
    checks: &Checks,
    log: impl Write,
    limits: Limits,
) -> io::Result<Outcome> {
    let (reader, writer) = io::pipe()?;
    command
        .stdin(Stdio::piped())
        .stdout(writer.try_clone()?)
        .stderr(writer);
    let mut child = command.spawn()?;
    // The command still holds the pipe's write ends. Drop them, so that the
    // pipe ends when the child and whatever it started have closed theirs.
    drop(command);
    let stdin = child.stdin.take();
    let output = forward(reader);
    let mut log = Log::new(log);

    let verdict = watch(&output, checks, &mut log, Instant::now() + limits.timeout);
    let status = stop(&mut child, stdin, &output, &mut log, limits.grace)?;
    log.finish()?;
    Ok(Outcome { verdict, status })
}

/// Reads the pipe on a thread of its own, so that the timeout can be applied
/// to a channel. The channel disconnects at the end of the output.
fn forward(mut reader: PipeReader) -> Receiver<Vec<u8>> {
    let (sender, receiver) = mpsc::channel();
    thread::spawn(move || {
        let mut buffer = vec![0; 64 * 1024];
        loop {
            match reader.read(&mut buffer) {
                Ok(0) => break,
                Ok(n) => {
                    if sender.send(buffer[..n].to_vec()).is_err() {
                        break;
                    }
                }
                Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
                Err(_) => break,
            }
        }
    });
    receiver
}

fn watch<W: Write>(
    output: &Receiver<Vec<u8>>,
    checks: &Checks,
    log: &mut Log<W>,
    deadline: Instant,
) -> Verdict {
    let mut watch = Watch::new(checks);
    loop {
        let now = Instant::now();
        if now >= deadline {
            return Verdict::Fail(Failure::TimedOut);
        }
        match output.recv_timeout(deadline - now) {
            Ok(piece) => {
                log.write(&piece);
                if let Some(verdict) = watch.feed(&piece) {
                    return verdict;
                }
            }
            Err(RecvTimeoutError::Timeout) => return Verdict::Fail(Failure::TimedOut),
            Err(RecvTimeoutError::Disconnected) => return Verdict::Fail(Failure::Ended),
        }
    }
}

/// Quits QEMU with Ctrl-A x, and signals the command if that does not end it.
fn stop<W: Write>(
    child: &mut Child,
    stdin: Option<ChildStdin>,
    output: &Receiver<Vec<u8>>,
    log: &mut Log<W>,
    grace: Duration,
) -> io::Result<ExitStatus> {
    if let Some(mut stdin) = stdin.as_ref()
        && child.try_wait()?.is_none()
    {
        // A command that has closed its input is past quitting this way.
        let _ = stdin.write_all(&CTRL_A_X).and_then(|()| stdin.flush());
    }
    let mut status = wait_logging(child, output, log, Instant::now() + grace)?;
    if status.is_none() {
        terminate(child);
        status = wait_logging(child, output, log, Instant::now() + TERM_WAIT)?;
    }
    let status = match status {
        Some(status) => status,
        None => {
            // Fails only if the child has been reaped, which it has not.
            let _ = child.kill();
            child.wait()?
        }
    };
    drop(stdin);
    // Output still in the pipe.
    let deadline = Instant::now() + LAST_OUTPUT_WAIT;
    loop {
        let now = Instant::now();
        match output.recv_timeout(deadline.saturating_duration_since(now)) {
            Ok(piece) => log.write(&piece),
            Err(_) => break,
        }
    }
    Ok(status)
}

/// Logs output until the child exits or `deadline` passes.
fn wait_logging<W: Write>(
    child: &mut Child,
    output: &Receiver<Vec<u8>>,
    log: &mut Log<W>,
    deadline: Instant,
) -> io::Result<Option<ExitStatus>> {
    loop {
        if let Some(status) = child.try_wait()? {
            return Ok(Some(status));
        }
        let now = Instant::now();
        if now >= deadline {
            return Ok(None);
        }
        let wait = POLL.min(deadline - now);
        match output.try_recv() {
            Ok(piece) => log.write(&piece),
            Err(TryRecvError::Empty) => {
                if let Ok(piece) = output.recv_timeout(wait) {
                    log.write(&piece);
                }
            }
            Err(TryRecvError::Disconnected) => thread::sleep(wait),
        }
    }
}

/// Sends SIGTERM. `docker run` passes it on to the container, where SIGKILL
/// would stop only the client and leave the container running.
fn terminate(child: &Child) {
    const SIGTERM: i32 = 15;
    unsafe extern "C" {
        fn kill(pid: i32, signal: i32) -> i32;
    }
    if let Ok(pid) = i32::try_from(child.id()) {
        // SAFETY: kill(2) has no memory effects. The child has not been
        // reaped, so its process ID still names it.
        unsafe {
            kill(pid, SIGTERM);
        }
    }
}

/// Writes the log, keeping the first error so that a failing log does not
/// stop the run before QEMU has been stopped.
struct Log<W: Write> {
    out: W,
    error: Option<io::Error>,
}

impl<W: Write> Log<W> {
    fn new(out: W) -> Self {
        Self { out, error: None }
    }

    fn write(&mut self, piece: &[u8]) {
        if self.error.is_none()
            && let Err(error) = self.out.write_all(piece)
        {
            self.error = Some(error);
        }
    }

    fn finish(mut self) -> io::Result<()> {
        match self.error.take() {
            Some(error) => Err(error),
            None => self.out.flush(),
        }
    }
}
