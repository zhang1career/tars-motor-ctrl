#!/usr/bin/env python3
"""Capture the Rigol DS1104Z over VISA and save CSV + PNG.

Written for the stage C question: does the ADC sample inside the low-side
conduction window? That cannot be answered from the target, because sampling is
invisible in the analog signal -- it needs the firmware's strobe on TP1 lined up
against the INA240 outputs.

Default channel map matches the bench wiring:
  CH1 TP1  strobe, pulses when the ADC sequence completes
  CH2 TP11 ISENSE_U
  CH3 TP12 ISENSE_V
  CH4 TP13 ISENSE_W
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_OUT = REPO_ROOT / "models" / "captured"
LABELS = ["TP1 strobe", "TP11 ISENSE_U", "TP12 ISENSE_V", "TP13 ISENSE_W"]


def connect():
    import pyvisa

    rm = pyvisa.ResourceManager()
    hits = [r for r in rm.list_resources() if "DS1Z" in r]
    if not hits:
        sys.exit(f"no DS1000Z found among {rm.list_resources()}")
    scope = rm.open_resource(hits[0])
    scope.timeout = 20000
    return scope


def setup(scope, timebase: float, scale_v: float, offset_v: float,
          trig_level: float) -> None:
    scope.write(":STOP")
    for ch in (1, 2, 3, 4):
        scope.write(f":CHAN{ch}:DISP ON")
        scope.write(f":CHAN{ch}:COUP DC")
        scope.write(f":CHAN{ch}:PROB 1")
        scope.write(f":CHAN{ch}:BWL OFF")
    # Strobe is a 0..3.3 V logic pulse; the INA240 outputs live around 1.65 V.
    scope.write(":CHAN1:SCAL 1")
    scope.write(":CHAN1:OFFS -1.5")
    for ch in (2, 3, 4):
        scope.write(f":CHAN{ch}:SCAL {scale_v}")
        scope.write(f":CHAN{ch}:OFFS {offset_v}")

    scope.write(f":TIM:MAIN:SCAL {timebase}")
    scope.write(":TIM:MAIN:OFFS 0")
    scope.write(":TRIG:MODE EDGE")
    scope.write(":TRIG:EDGE:SOUR CHAN1")
    scope.write(":TRIG:EDGE:SLOP POS")
    scope.write(f":TRIG:EDGE:LEV {trig_level}")
    scope.write(":TRIG:SWE SING")
    scope.write(":ACQ:TYPE NORM")
    scope.write(":ACQ:MDEP AUTO")
    time.sleep(0.3)


def single(scope) -> None:
    scope.write(":SING")
    deadline = time.time() + 10
    while time.time() < deadline:
        if scope.query(":TRIG:STAT?").strip() == "STOP":
            return
        time.sleep(0.1)
    print("warning: no trigger within 10 s; is the strobe running?",
          file=sys.stderr)
    scope.write(":STOP")


def read_channel(scope, ch: int) -> tuple[list[float], dict]:
    scope.write(f":WAV:SOUR CHAN{ch}")
    scope.write(":WAV:MODE RAW")
    scope.write(":WAV:FORM BYTE")
    pre = scope.query(":WAV:PRE?").strip().split(",")
    points = int(pre[2])
    xinc, xorig, xref = float(pre[4]), float(pre[5]), float(pre[6])
    yinc, yorig, yref = float(pre[7]), float(pre[8]), float(pre[9])

    raw: list[int] = []
    chunk = 250000
    start = 1
    while start <= points:
        stop = min(start + chunk - 1, points)
        scope.write(f":WAV:STAR {start}")
        scope.write(f":WAV:STOP {stop}")
        raw += list(scope.query_binary_values(":WAV:DATA?", datatype="B",
                                             container=list))
        start = stop + 1

    volts = [(b - yorig - yref) * yinc for b in raw]
    return volts, {"xinc": xinc, "xorig": xorig, "xref": xref,
                   "points": len(volts)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--timebase", type=float, default=10e-6, help="s/div")
    ap.add_argument("--scale", type=float, default=0.5, help="V/div, CH2..CH4")
    ap.add_argument("--offset", type=float, default=-1.5, help="V, CH2..CH4")
    ap.add_argument("--trig-level", type=float, default=1.65)
    ap.add_argument("--tag", default="scope")
    ap.add_argument("-o", "--outdir", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--no-setup", action="store_true",
                    help="capture with the scope's current settings")
    args = ap.parse_args()

    scope = connect()
    print("idn       ", scope.query("*IDN?").strip())
    if not args.no_setup:
        setup(scope, args.timebase, args.scale, args.offset, args.trig_level)
    single(scope)
    print("sample rate", scope.query(":ACQ:SRAT?").strip(), "Sa/s")

    chans = {}
    meta = None
    for ch in (1, 2, 3, 4):
        volts, meta = read_channel(scope, ch)
        chans[ch] = volts
        print(f"ch{ch}       {len(volts)} points, "
              f"{min(volts):+.3f} .. {max(volts):+.3f} V")
    scope.write(":RUN")

    n = min(len(v) for v in chans.values())
    t = [(i * meta["xinc"] + meta["xorig"]) for i in range(n)]

    args.outdir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = args.outdir / f"{args.tag}-{stamp}.csv"
    with csv_path.open("w", encoding="utf-8") as f:
        f.write("t_s," + ",".join(LABELS) + "\n")
        for i in range(n):
            f.write(f"{t[i]:.9e}," +
                    ",".join(f"{chans[c][i]:.4f}" for c in (1, 2, 3, 4)) + "\n")
    print("csv       ", csv_path)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tus = [x * 1e6 for x in t]
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 8))
    for k, ch in enumerate((1, 2, 3, 4)):
        axes[k].plot(tus, chans[ch][:n], lw=0.8)
        axes[k].set_ylabel(LABELS[k])
        axes[k].grid(alpha=0.3)
    axes[-1].set_xlabel("time (us)")
    axes[0].set_title(f"motor-ctrl scope: {args.tag}, {stamp}, "
                      f"{args.timebase * 1e6:g} us/div")
    fig.tight_layout()
    png_path = csv_path.with_suffix(".png")
    fig.savefig(png_path, dpi=110)
    print("plot      ", png_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
