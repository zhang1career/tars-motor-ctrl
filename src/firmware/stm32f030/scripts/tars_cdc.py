#!/usr/bin/env python3
"""Send one TARS shell line over USB CDC and print until the next prompt."""
from __future__ import annotations

import glob
import os
import sys
import time

try:
    import serial
except ImportError as exc:
    sys.stderr.write("need pyserial: pip install pyserial\n")
    raise SystemExit(2) from exc

PROMPT = "tars> "


def candidates() -> list[str]:
    seen = []
    for pat in ("/dev/cu.usbmodem*", "/dev/tty.usbmodem*"):
        for p in sorted(glob.glob(pat)):
            if p not in seen:
                seen.append(p)
    return seen


def looks_like_tars(port: str) -> bool:
    try:
        ser = serial.Serial(port, 115200, timeout=0.05)
    except Exception:
        return False
    try:
        time.sleep(0.08)
        ser.reset_input_buffer()
        ser.write(b"\r")
        deadline = time.time() + 0.6
        buf = ""
        while time.time() < deadline:
            chunk = ser.read(128).decode("utf-8", errors="replace")
            if chunk:
                buf += chunk
                if PROMPT in buf:
                    return True
            else:
                time.sleep(0.02)
        return False
    finally:
        ser.close()


def find_port() -> str:
    env = os.environ.get("TARS_CDC", "").strip()
    if env:
        return env
    cands = candidates()
    if not cands:
        sys.stderr.write("no /dev/cu.usbmodem*; set TARS_CDC\n")
        raise SystemExit(2)
    if len(cands) == 1:
        return cands[0]
    for p in cands:
        if looks_like_tars(p):
            return p
    sys.stderr.write("no TARS CDC among: %s (set TARS_CDC)\n" % " ".join(cands))
    raise SystemExit(2)


def open_session():
    port = find_port()
    sys.stderr.write("tars_cdc %s\n" % port)
    ser = serial.Serial(port, 115200, timeout=0.05)
    time.sleep(0.15)
    ser.reset_input_buffer()
    return ser


def session_cmd(ser, line: str, wait_s: float | None = None) -> str:
    if wait_s is None:
        wait_s = 16.0 if "mot start" in line else 6.0
    ser.reset_input_buffer()
    ser.write((line + "\r").encode("ascii"))
    deadline = time.time() + wait_s
    buf = ""
    while time.time() < deadline:
        chunk = ser.read(256).decode("utf-8", errors="replace")
        if chunk:
            buf += chunk
            if PROMPT in buf and buf.rstrip().endswith(PROMPT.rstrip()):
                if line.split()[0] in buf or buf.count(PROMPT) >= 2:
                    break
        else:
            time.sleep(0.02)
    return buf


def main() -> int:
    if len(sys.argv) < 2:
        sys.stderr.write("usage: tars_cdc.py <shell line>\n")
        return 2
    line = " ".join(sys.argv[1:])
    ser = open_session()
    try:
        buf = session_cmd(ser, line)
        sys.stdout.write(buf)
        sys.stdout.flush()
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
