"""Build-time constants for the motor-ctrl image next to motor-ctrl.elf.

Scripts must not hardcode MOTOR_TRACE_DEPTH or the 20 kHz control tick: both
come from CMake / the linked firmware. Read CMakeCache.txt beside the ELF.
"""
from __future__ import annotations

import re
from pathlib import Path

FW_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_ELF = FW_ROOT / "build" / "Release" / "motor-ctrl.elf"

# TIM1 center-aligned: 48 MHz / (ARR+1) / 2, ARR=1199 → 20 kHz control ticks.
MOTOR_CTRL_TICK_HZ = 20_000
MOTOR_CTRL_TICK_S = 1.0 / MOTOR_CTRL_TICK_HZ


def _cmake_cache_value(cache_path: Path, key: str) -> str | None:
    if not cache_path.is_file():
        return None
    needle = f"{key}="
    for line in cache_path.read_text().splitlines():
        if line.startswith(needle):
            return line.split("=", 1)[1].strip()
    return None


def trace_depth(elf: Path | None = None) -> int:
    """Samples per channel in g_motor_trace_buf (MOTOR_TRACE_DEPTH from CMake)."""
    elf = elf or DEFAULT_ELF
    cache = elf.parent / "CMakeCache.txt"
    raw = _cmake_cache_value(cache, "MOTOR_TRACE_DEPTH:STRING")
    if raw and raw.isdigit():
        return int(raw)
    # motor_trace.h default when CMake leaves MOTOR_TRACE_DEPTH empty
    return 256


def trace_window_s(
    decim: int,
    *,
    depth: int | None = None,
    elf: Path | None = None,
) -> float:
    """Wall time covered by one oneshot trace window at the given decimation."""
    d = depth if depth is not None else trace_depth(elf)
    return d * MOTOR_CTRL_TICK_S * max(decim, 1)
