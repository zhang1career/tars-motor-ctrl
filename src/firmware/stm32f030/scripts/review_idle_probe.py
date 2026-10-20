#!/usr/bin/env python3
"""Helper for review_idle_probe.sh — read g_motor_adc_raw halfwords via OpenOCD."""
from __future__ import annotations

import os
import sys
from pathlib import Path

import sym_cache
import trace_dump

ROOT = Path(os.environ["BENCH_ROOT"])
ELF = Path(os.environ["ELF"])
CFG = ROOT / "openocd.cfg"


def adc_line() -> str:
    addr = sym_cache.get(ELF, "g_motor_adc_raw")
    out = trace_dump.ocd(CFG, [f"mdh 0x{addr:08x} 5"])
    for line in out.splitlines():
        if ":" in line:
            return line.split(":", 1)[1].strip()
    return "?"


if __name__ == "__main__":
    if sys.argv[-1] == "adc":
        print(adc_line())
    else:
        sys.exit(2)
