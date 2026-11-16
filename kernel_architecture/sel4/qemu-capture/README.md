# qemu-capture

Runs a QEMU test, saves everything it prints, and decides from that output
whether the test passed. `sel4test.sh`, `iommu_gate.sh` and `zero_copy.sh`
run their QEMU tests through it, with `cargo run --release`, so the test step
needs Cargo (Rust 1.88 or later) on the host.

```sh
qemu-capture --log PATH --timeout SECONDS --pass TEXT [--fail TEXT] \
    [--summary TEMPLATE] [--fail-line PREFIX] -- COMMAND [ARG...]
```

COMMAND runs on the host; the scripts give it
`docker run --init -i IMAGE qemu-system-riscv64 -nographic ...`. Its standard
output and standard error share one pipe, and its standard input is another.
QEMU's `-nographic` console needs no terminal.

## Verdict

The first marker to appear in the output ends the wait.

- `--fail TEXT`, the end of the output (the command exited) and the timeout
  each fail the test.
- `--pass TEXT` passes it only if the output before the marker
  - contains a match for `--summary TEMPLATE`, when given, whose `{passed}`
    count is nonzero and equal to its `{total}` count, when the template has
    one; the first match counts;
  - has no line that starts with `--fail-line PREFIX`, when given.

Markers and prefixes are literal text. TEMPLATE is literal text with fields
that each match a run of ASCII digits: `{passed}` (required), `{total}`
(optional) and `{_}` (matched and ignored). The scripts use:

| Script | `--pass` | `--fail` | `--summary` | `--fail-line` |
|---|---|---|---|---|
| `sel4test.sh` | `All is well in the universe` | `Test suite failed.` | `Test suite passed. {passed} tests passed. {_} tests disabled.` | |
| `iommu_gate.sh` | `IOMMU_GATE: PASS` | `IOMMU_GATE: FAIL` | `IOMMU gate: {passed}/{total} checks passed` | `FAIL: ` |
| `zero_copy.sh` | `ZERO_COPY: PASS` | `ZERO_COPY: FAIL` | `Zero copy: {passed}/{total} checks passed` | `FAIL: ` |

The timeout counts from the start of COMMAND, so for the scripts it includes
starting the container.

## Stopping QEMU

Once the verdict is decided, qemu-capture writes Ctrl-A x (bytes `0x01 0x78`)
to the command's input, which quits QEMU, and goes on logging until the
command exits. A command still running 10 seconds later is sent SIGTERM, and
SIGKILL 5 seconds after that. `docker run` passes SIGTERM on to the
container, where `--init` passes it on to QEMU (or to `./simulate`, which
runs QEMU). SIGKILL would stop only the Docker client and leave the container
running. The verdict does not depend on how the command ends.

## Output and exit status

| Exit status | Meaning | Printed |
|---|---|---|
| 0 | pass | the summary found, then `PASS: ...`, on standard output |
| 1 | fail | `FAIL: <reason>; see <log>` on standard error |
| 2 | the test could not be run: bad arguments, a log that cannot be written, or a command that cannot be started | `qemu-capture: <error>` on standard error |

The scripts exit with this status.

## Log

The log holds every byte the command wrote to standard output and standard
error, in the order written, from its start until it exits, including what
follows the marker (QEMU's `QEMU: Terminated`). No terminal is involved, so
line ends are as the guest wrote them: `\r\n` from seL4, where output through
a terminal reads `\r\r\n`. Bytes that are not UTF-8 are kept as they are.

## Tests

```sh
cargo test
cargo fmt --check
cargo clippy --all-targets -- -D warnings
```

The tests feed transcripts recorded from real runs through the matcher with
the checks each script uses: passes, failure markers, a wrong count, a count
of zero, `FAIL:` lines behind a full count, timeouts and early exits.
`tests/transcripts.rs` says how each was recorded. `tests/run.rs` and
`tests/cli.rs` replay them from a shell command to test the timeout, Ctrl-A
x, the signals, the log and the exit status.
