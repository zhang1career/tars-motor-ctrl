#!/usr/bin/env python3
"""Poll FOC mode byte; on mode 2 dump diag, clear tick counters, exit."""
from __future__ import annotations

import re
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ocd_rpc import rpc  # noqa: E402

MODE = 0x200003D0
DIAG = 0x2000099C
TICK_CYC_MAX = DIAG + 0x10
TICK_OVERRUN = DIAG + 0x14
TICK_TOTAL = DIAG + 0x18
TICK_CYC_SUM = DIAG + 0x24

DEADLINE = time.monotonic() + 8.0


def ocd(*cmds: str, timeout: float = 30.0) -> str:
    for _ in range(5):
        try:
            return rpc(list(cmds), timeout=timeout)
        except Exception as e:
            print(f"mdw retry: {e}", file=sys.stderr)
            time.sleep(0.05)
    raise SystemExit("ocd_rpc failed")


def mode_lo() -> int | None:
    out = ocd(f"mdw 0x{MODE:08x} 1")
    m = re.search(r":\s*([0-9a-fA-F]+)", out)
    if not m:
        return None
    return int(m.group(1), 16) & 0xFF


def main() -> int:
    while time.monotonic() < DEADLINE:
        lo = mode_lo()
        if lo == 2:
            dump = ocd(f"mdw 0x{DIAG:08x} 11")
            print("=== MODE2_DIAG_BEFORE_CLEAR ===")
            print(dump, end="" if dump.endswith("\n") else "\n")
            ocd(
                f"mww 0x{TICK_CYC_MAX:08x} 0",
                f"mww 0x{TICK_OVERRUN:08x} 0",
                f"mww 0x{TICK_TOTAL:08x} 0",
                f"mww 0x{TICK_CYC_SUM:08x} 0",
            )
            print("=== MODE2_WATCH_DONE ===")
            return 0
        time.sleep(0.010)
    print("NO_MODE2")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
