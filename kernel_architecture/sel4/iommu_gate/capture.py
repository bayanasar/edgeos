"""Run the IOMMU gate image in QEMU, keep the full log, and return its verdict."""
import re
import sys
from pathlib import Path

import pexpect

# Slot numbers are fixed: they are the requester IDs the root task expects.
QEMU = [
    "qemu-system-riscv64", "-machine", "virt", "-cpu", "rv64", "-nographic",
    "-m", "size=3072", "-bios", "none",
    "-device", "riscv-iommu-pci,addr=01.0",
    "-device", "edu,addr=02.0,dma_mask=0xffffffffff",
    "-device", "edu,addr=03.0,dma_mask=0xffffffffff",
    "-kernel", "images/iommu_gate-image-riscv-qemu-riscv-virt",
]


def run(log_path: Path, timeout: int) -> int:
    with log_path.open("w") as log:
        child = pexpect.spawn(QEMU[0], QEMU[1:], encoding="utf-8", timeout=timeout)
        child.logfile_read = log
        try:
            outcome = child.expect([
                "IOMMU_GATE: PASS",
                "IOMMU_GATE: FAIL",
                pexpect.EOF,
                pexpect.TIMEOUT,
            ])
            summary = re.search(r"IOMMU gate: (\d+)/(\d+) checks passed", child.before)
            failed = re.search(r"^FAIL: ", child.before, re.MULTILINE)
            passed = (outcome == 0 and summary is not None and int(summary[2]) > 0
                      and summary[1] == summary[2] and failed is None)
            if passed:
                print(summary[0])
                print("PASS: every gate check passed and the success marker was observed")
            else:
                reason = ["inconsistent summary", "gate failure", "early EOF", "timeout"][outcome]
                print(f"FAIL: {reason}; see {log_path}", file=sys.stderr)
            return 0 if passed else 1
        finally:
            # The verdict is already decided; a slow QEMU exit must not change it.
            try:
                if child.isalive():
                    child.sendcontrol("a")
                    child.send("x")
                    child.expect(pexpect.EOF, timeout=10)
            except (pexpect.TIMEOUT, pexpect.EOF, OSError):
                pass
            finally:
                child.close(force=True)


if __name__ == "__main__":
    if len(sys.argv) != 3 or not sys.argv[2].isdigit() or int(sys.argv[2]) <= 0:
        sys.exit("Usage: capture.py LOG_PATH TIMEOUT_SECONDS (>0)")
    sys.exit(run(Path(sys.argv[1]), int(sys.argv[2])))
