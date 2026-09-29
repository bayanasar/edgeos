"""Run generated seL4test simulate, preserve output, and return a test verdict."""
import re
import sys
from pathlib import Path

import pexpect


def run(log_path: Path, timeout: int) -> int:
    with log_path.open("w") as log:
        child = pexpect.spawn("./simulate", encoding="utf-8", timeout=timeout)
        child.logfile_read = log
        try:
            outcome = child.expect([
                "All is well in the universe",
                r"Test suite failed\.",
                pexpect.EOF,
                pexpect.TIMEOUT,
            ])
            summary = re.search(
                r"Test suite passed\. (\d+) tests passed\. (\d+) tests disabled\.",
                child.before,
            )
            passed = outcome == 0 and summary is not None and int(summary[1]) > 0
            if passed:
                print(summary[0])
                print("PASS: upstream suite summary and success marker observed")
            else:
                reason = ["missing or empty suite summary", "suite failure", "early EOF", "timeout"][outcome]
                print(f"FAIL: {reason}; see {log_path}", file=sys.stderr)
            return 0 if passed else 1
        finally:
            try:
                if child.isalive():
                    child.sendcontrol("a")
                    child.send("x")
                    child.expect(pexpect.EOF, timeout=10)
            finally:
                child.close(force=True)


if __name__ == "__main__":
    if len(sys.argv) != 3 or not sys.argv[2].isdigit() or int(sys.argv[2]) <= 0:
        sys.exit("Usage: qemu_capture.py LOG_PATH TIMEOUT_SECONDS (>0)")
    sys.exit(run(Path(sys.argv[1]), int(sys.argv[2])))
