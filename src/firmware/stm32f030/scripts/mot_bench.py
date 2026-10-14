#!/usr/bin/env python3
"""Repeatable I2C benches: load step (disturb) and thermal soak.

  TARS_CDC=… ./mot_bench.py disturb          # motor already at w=80
  TARS_CDC=… ./mot_bench.py thermal          # already holding; 20 min
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone

import tars_cdc

MOT_RE = re.compile(
    r"mot 0x(?P<addr>[0-9A-Fa-f]+)"
    r" st=(?P<st>\d+) mode=(?P<mode>\d+) dir=(?P<dir>\d+)"
    r" w=(?P<w>-?\d+)/(?P<w_ref>-?\d+) /s"
    r" iq=(?P<iq>-?\d+)/(?P<iq_ref>-?\d+) mA"
    r" id=(?P<id>-?\d+) mA"
    r" th=(?P<th>\d+) acc=(?P<acc>-?\d+)"
    r" wrap=(?P<wrap>-?\d+)/s"
    r" iqmax=(?P<iqmax>-?\d+) mA"
    r" vbus=(?P<vbus>\d+) mV"
    r" hall=(?P<hall>\d+)"
    r" flt=0x(?P<flt>[0-9A-Fa-f]+)"
    r" alert=(?P<alert>\d+)"
)

HEALTH_RE = re.compile(
    r"0x(?P<addr>[0-9A-Fa-f]+): st=0x(?P<st>[0-9A-Fa-f]+)"
    r" flt=0x(?P<flt>[0-9A-Fa-f]+) up=(?P<up>\d+)s"
    r" temp=(?P<temp_i>-?\d+)\.(?P<temp_f>\d+)C"
    r" vdda=(?P<vdda>\d+)mV"
)


def captured_dir() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.abspath(os.path.join(here, "..", "..", "..", ".."))
    out = os.path.join(root, "models", "captured")
    os.makedirs(out, exist_ok=True)
    return out


def stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")


def parse_mot(text: str) -> dict | None:
    m = MOT_RE.search(text)
    if not m:
        return None
    d = m.groupdict()
    out = {
        "st": int(d["st"]),
        "mode": int(d["mode"]),
        "dir": int(d["dir"]),
        "w": int(d["w"]),
        "w_ref": int(d["w_ref"]),
        "iq": int(d["iq"]),
        "iq_ref": int(d["iq_ref"]),
        "id": int(d["id"]),
        "th": int(d["th"]),
        "acc": int(d["acc"]),
        "wrap": int(d["wrap"]),
        "iqmax": int(d["iqmax"]),
        "vbus": int(d["vbus"]),
        "hall": int(d["hall"]),
        "flt": int(d["flt"], 16),
        "alert": int(d["alert"]),
    }
    return out


def parse_health(text: str) -> dict | None:
    m = HEALTH_RE.search(text)
    if not m:
        return None
    d = m.groupdict()
    temp = float(d["temp_i"]) + float(d["temp_f"]) / (10.0 ** len(d["temp_f"]))
    return {
        "h_st": int(d["st"], 16),
        "h_flt": int(d["flt"], 16),
        "up": int(d["up"]),
        "temp_c": temp,
        "vdda": int(d["vdda"]),
    }


def mean(xs: list[float]) -> float | None:
    if not xs:
        return None
    return sum(xs) / float(len(xs))


def cmd_mot(ser, line: str) -> str:
    wait = 12.0 if "mot start" in line else 2.0
    return tars_cdc.session_cmd(ser, line, wait_s=wait)


def schedule_twist_cues(t0: float, twist_s: float, from_tick: str, to_tick: str) -> None:
    """Wall-clock cues; do not wait for the I²C poll."""

    def run() -> None:
        for sec in (2, 1):
            sl = (t0 + twist_s - float(sec)) - time.time()
            if sl > 0:
                time.sleep(sl)
            play_cue("tick")
            print("还有 %d s" % sec, flush=True)
        sl = (t0 + twist_s) - time.time()
        if sl > 0:
            time.sleep(sl)
        play_cue("go")
        print("\n>>> 现在拧 %s→%s <<<\n" % (from_tick, to_tick), flush=True)

    threading.Thread(target=run, daemon=True).start()


def play_cue(kind: str) -> None:
    """Mac system sound; BEL if afplay is missing."""
    sounds = {
        "tick": "/System/Library/Sounds/Tink.aiff",
        "go": "/System/Library/Sounds/Glass.aiff",
    }
    path = sounds.get(kind, sounds["go"])
    try:
        subprocess.Popen(
            ["afplay", path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError:
        sys.stdout.write("\a")
        sys.stdout.flush()


def disturb(args: argparse.Namespace) -> int:
    twist_s = float(args.twist_s)
    dur_s = float(args.dur_s)
    period = float(args.period)
    recover_abs = float(args.recover) if args.recover else None
    band = float(args.band)
    out_path = args.out or os.path.join(
        captured_dir(), "disturb-%s.csv" % stamp()
    )

    ser = tars_cdc.open_session()
    rows: list[dict] = []
    try:
        if args.start:
            print("start speed dir=%s iq=%s w=%s" % (args.dir, args.iq, args.w))
            print(cmd_mot(ser, "nodebus mot start speed %s %s %s" % (
                args.dir, args.iq, args.w
            )))
            print("wait 8 s for hall6 → FOC")
            t_wait = time.time()
            while time.time() - t_wait < 8.0:
                print(cmd_mot(ser, "nodebus mot stat").strip())
                time.sleep(0.4)
        print("t=0 开始记。还有 2 s、1 s 各一声，到点 Glass 提示拧 %s→%s" % (
            args.from_tick, args.to_tick
        ), flush=True)
        t0 = time.time()
        next_t = t0
        schedule_twist_cues(t0, twist_s, args.from_tick, args.to_tick)
        while True:
            now = time.time()
            t = now - t0
            if t >= dur_s:
                break
            raw = cmd_mot(ser, "nodebus mot stat")
            mot = parse_mot(raw)
            rec = {"t_s": round(t, 3)}
            if mot:
                rec.update(mot)
                print("t=%5.2f wrap=%s iq=%s vbus=%s flt=0x%04X" % (
                    t, mot["wrap"], mot["iq"], mot["vbus"], mot["flt"]
                ))
            else:
                rec["parse"] = "fail"
                print("t=%5.2f PARSE FAIL: %s" % (t, raw.replace("\r", " ").strip()[:120]))
            rows.append(rec)
            next_t += period
            sl = next_t - time.time()
            if sl > 0:
                time.sleep(sl)
        if args.stop:
            print(cmd_mot(ser, "nodebus mot stop"))
    finally:
        ser.close()

    fieldnames = [
        "t_s", "wrap", "w", "w_ref", "iq", "iq_ref", "id",
        "vbus", "acc", "st", "flt", "alert", "dir",
    ]
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    print("csv %s  n=%u" % (out_path, len(rows)))

    pre_w = [r["wrap"] for r in rows if "wrap" in r and r["t_s"] < twist_s]
    pre_i = [r["iq"] for r in rows if "iq" in r and r["t_s"] < twist_s]
    post = [r for r in rows if "wrap" in r and r["t_s"] >= twist_s]
    if not pre_w or not post:
        print("分析不够：拧前或拧后没有有效 wrap")
        return 1
    wrap0 = mean(pre_w)
    iq0 = mean(pre_i)
    recover_wrap = recover_abs if recover_abs is not None else (wrap0 - band)
    nadir = min(post, key=lambda r: r["wrap"])
    recov = None
    after_nadir = [r for r in post if r["t_s"] >= nadir["t_s"]]
    for r in after_nadir:
        if r["wrap"] >= recover_wrap:
            recov = r
            break
    last = [r for r in rows if "wrap" in r and r["t_s"] >= (dur_s - 2.0)]
    wrap1 = mean([r["wrap"] for r in last])
    iq1 = mean([r["iq"] for r in last])
    vbus = [r["vbus"] for r in rows if "vbus" in r]
    flts = [r["flt"] for r in rows if "flt" in r]
    or_flt = 0
    for f in flts:
        or_flt |= f
    print("--- 扰动 ---")
    print("拧前 wrap=%.1f  iq=%.0f mA" % (wrap0, iq0 or 0))
    print("最低 wrap=%d @ t=%.2f s  （相对拧前 %+.1f）" % (
        nadir["wrap"], nadir["t_s"], nadir["wrap"] - wrap0
    ))
    if recov:
        print("拉回 wrap>=%.1f @ t=%.2f s  （从最低点 %.2f s）" % (
            recover_wrap, recov["t_s"], recov["t_s"] - nadir["t_s"]
        ))
    else:
        print("窗内未拉回 wrap>=%.1f" % recover_wrap)
    print("收尾 wrap=%s  iq=%s mA  Δiq=%s mA" % (
        ("%.1f" % wrap1) if wrap1 is not None else "?",
        ("%.0f" % iq1) if iq1 is not None else "?",
        ("%.0f" % (iq1 - iq0)) if (iq0 is not None and iq1 is not None) else "?",
    ))
    if vbus:
        print("vbus %u…%u mV" % (min(vbus), max(vbus)))
    print("flt OR=0x%04X" % or_flt)
    ok = (
        recov is not None
        and wrap1 is not None
        and abs(wrap1 - wrap0) <= band
        and or_flt == 0
        and (not vbus or min(vbus) >= 10500)
    )
    print("过线: %s  （拉回拧前−%.0f、收尾拧前±%.0f、无故障、母线≥10.5 V）" % (
        "是" if ok else "否", band, band
    ))
    return 0 if ok else 1


def thermal(args: argparse.Namespace) -> int:
    minutes = float(args.minutes)
    period = float(args.period)
    if period >= 2.8:
        print("period 必须 < 3 s（I²C 超时停机），现在 %.1f" % period, file=sys.stderr)
        return 2
    out_path = args.out or os.path.join(
        captured_dir(), "thermal-%s.csv" % stamp()
    )
    dur_s = minutes * 60.0
    ser = tars_cdc.open_session()
    rows: list[dict] = []
    try:
        print("温升 %g min，每 %.1f s 打点。不要开 nanoDAP 串口。" % (minutes, period), flush=True)
        t0 = time.time()
        next_t = t0
        while True:
            now = time.time()
            t = now - t0
            if t >= dur_s:
                break
            rec = {"t_s": round(t, 3)}
            mot = parse_mot(cmd_mot(ser, "nodebus mot stat"))
            hp = None
            if (int(t) % 15) == 0:
                hp = parse_health(cmd_mot(ser, "nodebus health 0x50"))
            if mot:
                rec.update(mot)
            if hp:
                rec.update(hp)
            rows.append(rec)
            print("t=%6.1f wrap=%s iq=%s temp=%s vbus=%s flt=%s" % (
                t,
                rec.get("wrap", "?"),
                rec.get("iq", "?"),
                ("%.1f" % rec["temp_c"]) if "temp_c" in rec else "?",
                rec.get("vbus", "?"),
                ("0x%04X" % rec["flt"]) if "flt" in rec else "?",
            ), flush=True)
            next_t += period
            sl = next_t - time.time()
            if sl > 0:
                time.sleep(sl)
        if args.stop:
            print(cmd_mot(ser, "nodebus mot stop"))
    finally:
        ser.close()

    fieldnames = [
        "t_s", "wrap", "w", "iq", "iq_ref", "vbus", "flt", "st",
        "temp_c", "vdda", "up",
    ]
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    print("csv %s  n=%u" % (out_path, len(rows)))
    wraps = [r["wrap"] for r in rows if "wrap" in r]
    iqs = [r["iq"] for r in rows if "iq" in r]
    temps = [r["temp_c"] for r in rows if "temp_c" in r]
    flts = [r["flt"] for r in rows if "flt" in r]
    or_flt = 0
    for f in flts:
        or_flt |= f
    print("--- 温升 ---")
    if wraps:
        print("wrap %d…%d  均值 %.1f" % (min(wraps), max(wraps), mean(wraps)))
    if iqs:
        print("iq %d…%d mA" % (min(iqs), max(iqs)))
    if temps:
        print("temp %.1f → %.1f °C  （Δ %+.1f）" % (
            temps[0], temps[-1], temps[-1] - temps[0]
        ))
    print("flt OR=0x%04X" % or_flt)
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description="motor-ctrl I2C benches")
    sub = p.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("disturb", help="15 s log; twist brake at t=3 s")
    d.add_argument("--start", action="store_true", help="start speed loop first")
    d.add_argument("--stop", action="store_true")
    d.add_argument("--dir", default="0")
    d.add_argument("--iq", default="1200")
    d.add_argument("--w", default="80")
    d.add_argument("--from-tick", default="3")
    d.add_argument("--to-tick", default="4")
    d.add_argument("--twist-s", default="6.0")
    d.add_argument("--dur-s", default="20.0")
    d.add_argument("--period", default="0.10")
    d.add_argument("--recover", default="", help="absolute recover line; default = 拧前 wrap − band")
    d.add_argument("--band", default="2.0", help="recover/hold band around 拧前 wrap")
    d.add_argument("--out", default="")
    d.set_defaults(func=disturb)

    t = sub.add_parser("thermal", help="hold and log temp/wrap/iq")
    t.add_argument("--minutes", default="20")
    t.add_argument("--period", default="1.5")
    t.add_argument("--stop", action="store_true")
    t.add_argument("--out", default="")
    t.set_defaults(func=thermal)

    args = p.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
