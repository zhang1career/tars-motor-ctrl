#!/usr/bin/env python3
"""Write one benchgate Session after a measurement this repo already saved.

The csv stays where the measurement script put it. This copies it into a
Session and appends one line to models/index.jsonl at the repo root, so a
later turn can look up why the run happened and what the numbers were.
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
METER_DESIGN = "/Users/mini/Projects/hw-lab/half-bridge/pcb"


def bus_current_a() -> float | None:
    r = subprocess.run(
        ["benchgate", "lab", "read", "--design", METER_DESIGN, "--count", "1"],
        capture_output=True, text=True, check=False,
    )
    if r.returncode != 0:
        sys.stderr.write(r.stderr or r.stdout)
        return None
    try:
        data = json.loads(r.stdout)
        return float(data["readings"][0]["normalized_value"])
    except (KeyError, IndexError, TypeError, json.JSONDecodeError, ValueError):
        sys.stderr.write(r.stdout)
        return None


def record(purpose: str, *, files=(), values=(), notes: str = "",
           instrument: str = "电机台架", tags: str = "foc") -> bool:
    """Return True when the ledger line was written. A failure is printed, not raised."""
    kept = [str(p) for p in files if Path(p).is_file()]
    vals = [v for v in values if v]
    if not kept and not vals:
        sys.stderr.write("台账\t没有文件也没有读数，跳过\n")
        return False
    cmd = [
        "benchgate", "lab", "import",
        "--design", str(REPO),
        "--purpose", purpose.strip()[:500],
        "--instrument", instrument,
        "--tags", tags,
    ]
    if notes.strip():
        cmd += ["--notes", notes.strip()]
    for path in kept:
        cmd += ["--file", path]
    for value in vals:
        cmd += ["--value", value]
    r = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if r.returncode != 0:
        sys.stderr.write(r.stderr or r.stdout or "台账没记下\n")
        return False
    sid = ""
    try:
        sid = json.loads(r.stdout).get("session_id", "")
    except json.JSONDecodeError:
        pass
    print("台账\t已记下" + (f" {sid}" if sid else ""))
    return True
