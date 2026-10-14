#!/usr/bin/env python3
"""One stationary or slow hold, one or more traces, then stop.

The parent runs this directly. A subagent is for a firmware change, not for
this measurement.

  TARS_CDC=/dev/cu.usbmodem345B345933331 \\
    ./foc_hold.py --mode torque --dir 0 --iq 0 \\
      --ab g_motor_pwm_dt_on=1,0,1 \\
      --source foc --decim 8 \\
      --csv ../../../models/captured/foc-iq-step-20261010/example.csv

--ab takes one window per value inside the same hold, so a switch is
compared at one rotor position without restarting. Files get -1, -2, ...
suffixes. The byte is put back to the value it had before the first window.

Does not rebuild, flash, reset, or resume. Reuses the OpenOCD already
listening on port 6666. Current scale in the report is 16.17 mA per LSB.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path

import lab_record
import sym_cache
import tars_cdc
import trace_dump
from mot_bench import parse_mot

FW = Path(__file__).resolve().parent.parent
ELF = FW / "build" / "Release" / "motor-ctrl.elf"
CFG = FW / "openocd.cfg"
COMMON = FW / "scripts" / "bench_common.sh"
TARS_PORT = "/dev/cu.usbmodem345B345933331"
DESIGN = "/Users/mini/Projects/hw-lab/half-bridge/pcb"
MA_PER_LSB = 16.17
STATE = {"torque": 2, "speed": 3}
STAT_WAIT = 1.5
START_WAIT = 12.0
SETTLE_S = 0.15


class HoldFail(SystemExit):
    def __init__(self, message: str, code: int):
        super().__init__(message)
        self.code = code


def ensure_ocd() -> None:
    r = subprocess.run(
        ["bash", "-c", 'source "$1" && ocd_start', "foc_hold", str(COMMON)],
        check=False,
    )
    if r.returncode != 0:
        raise HoldFail("调试器没有连上", 5)


def ocd(cmds: list[str]) -> str:
    return trace_dump.ocd(CFG, cmds)


def read_byte(name: str) -> int:
    addr = sym_cache.get(ELF, name)
    text = ocd([f"mdb 0x{addr:08x} 1"])
    for line in text.splitlines():
        if ":" not in line:
            continue
        parts = line.split(":", 1)[1].split()
        if parts:
            return int(parts[0], 16)
    raise HoldFail(f"{name} 读不出来", 5)


def mwb(name: str, value: int) -> None:
    addr = sym_cache.get(ELF, name)
    ocd([f"mwb 0x{addr:08x} {value & 0xFF}"])
    got = read_byte(name)
    if got != (value & 0xFF):
        raise HoldFail(f"{name} 写成 {value} 后读回 {got}", 5)


def say(ser, line: str, wait: float) -> str:
    return tars_cdc.session_cmd(ser, line, wait_s=wait)


def read_stat(ser) -> dict:
    """Second read wins. The first reply after a write can be the previous one."""
    first = parse_mot(say(ser, "nodebus mot stat", STAT_WAIT))
    second = parse_mot(say(ser, "nodebus mot stat", STAT_WAIT))
    if second is not None:
        return second
    if first is not None:
        return first
    raise HoldFail("状态回复读不懂", 5)


def lab_current() -> float | None:
    r = subprocess.run(
        ["benchgate", "lab", "read", "--design", DESIGN, "--count", "1"],
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


def summarize(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise HoldFail(f"{path} 是空的", 5)
    if "iq_lsb" not in rows[0]:
        out = {"n": len(rows)}
        for col in ("iu_lsb", "iv_lsb", "iw_lsb"):
            if col in rows[0]:
                vals = [int(float(r[col])) for r in rows]
                out[col] = (statistics.mean(vals), statistics.pstdev(vals))
        return out
    iq = [int(float(r["iq_lsb"])) for r in rows]
    out = {
        "n": len(rows),
        "iq_mean": statistics.mean(iq),
        "iq_stdev": statistics.pstdev(iq),
        "iq_min": min(iq),
        "iq_max": max(iq),
    }
    if "id_lsb" in rows[0]:
        id_ = [int(float(r["id_lsb"])) for r in rows]
        out["id_mean"] = statistics.mean(id_)
        out["id_stdev"] = statistics.pstdev(id_)
    if "hall_raw" in rows[0]:
        out["halls"] = sorted({int(float(r["hall_raw"])) for r in rows})
    if "vq_mv" in rows[0]:
        vq = [int(float(r["vq_mv"])) for r in rows]
        out["vq_mean"] = statistics.mean(vq)
        out["vq_max"] = max(vq)
        out["vq_min"] = min(vq)
        out["vq_zero"] = sum(1 for v in vq if v == 0)
    if "vd_mv" in rows[0]:
        vd = [int(float(r["vd_mv"])) for r in rows]
        out["vd_mean"] = statistics.mean(vd)
        out["vd_absmax"] = max(abs(v) for v in vd)
    if "sat" in rows[0]:
        out["sat_n"] = sum(1 for r in rows if int(float(r["sat"])) != 0)
    return out


def parse_assign(text: str) -> tuple[str, int]:
    if "=" not in text:
        raise HoldFail(f"要写成 名字=整数，收到 {text}", 2)
    name, raw = text.split("=", 1)
    return name.strip(), int(raw, 0)


def parse_ab(text: str) -> tuple[str, list[int]]:
    if "=" not in text:
        raise HoldFail(f"要写成 名字=值,值,值，收到 {text}", 2)
    name, raw = text.split("=", 1)
    values = [int(v, 0) for v in raw.split(",") if v.strip()]
    if len(values) < 2:
        raise HoldFail("--ab 至少要两个值", 2)
    return name.strip(), values


def wait_mode(ser, want: int, timeout: float, bump: bool) -> dict:
    deadline = time.time() + timeout
    saw_hall6 = None
    bumped = False
    last = None
    while time.time() < deadline:
        st = read_stat(ser)
        last = st
        if st["st"] == 4 or st["flt"] != 0:
            raise HoldFail(
                f"故障 状态 {st['st']} flt=0x{st['flt']:04x} 霍尔 {st['hall']}"
                f"{'（占空比已改到 22）' if bumped else ''}", 4)
        if st["st"] == want:
            return st
        if st["st"] == 1:
            if saw_hall6 is None:
                saw_hall6 = time.time()
            elif bump and not bumped and time.time() - saw_hall6 >= 3.0:
                mwb("s_run_duty_pct", 22)
                mwb("s_kick_duty_pct", 22)
                bumped = True
    raise HoldFail(
        f"没有进入状态 {want}，最后状态 {None if last is None else last['st']} "
        f"霍尔 {None if last is None else last['hall']}",
        3)


def wait_still(ser, hall: int | None, iq_ma: int, nudge_iq: int | None) -> dict:
    """Same hall on two reads at least 0.2 s apart. With --hall, it must be that one."""
    deadline = time.time() + 8.0
    nudge_deadline = None
    restored = False
    seen = None
    seen_t = 0.0
    last = None
    while time.time() < deadline:
        st = read_stat(ser)
        last = st["hall"]
        if st["st"] == 4 or st["flt"] != 0:
            raise HoldFail(f"故障 flt=0x{st['flt']:04x}", 4)
        if st["hall"] != seen:
            seen, seen_t = st["hall"], time.time()
            continue
        if time.time() - seen_t < 0.2:
            continue
        if hall is None or st["hall"] == hall:
            if nudge_deadline is not None and not restored:
                say(ser, f"nodebus mot iq {iq_ma}", STAT_WAIT)
                restored = True
                seen = None
                continue
            return st
        if nudge_iq is None:
            raise HoldFail(f"停在霍尔 {st['hall']}，不是 {hall}", 3)
        if nudge_deadline is None:
            say(ser, f"nodebus mot iq {nudge_iq}", STAT_WAIT)
            nudge_deadline = time.time() + 3.0
            seen = None
            continue
        if time.time() >= nudge_deadline and not restored:
            say(ser, f"nodebus mot iq {iq_ma}", STAT_WAIT)
            raise HoldFail(f"挪了 3 秒仍不是霍尔 {hall}，现在是 {st['hall']}", 3)
    raise HoldFail(f"8 秒内没有停稳，最后看到霍尔 {last}", 3)


KP_MV = 53.313
KP_KI_MV = 57.276


def mww(name: str, value: int) -> None:
    addr = sym_cache.get(ELF, name)
    ocd([f"mww 0x{addr:08x} {value & 0xFFFFFFFF}"])


def s32(word: int) -> int:
    word &= 0xFFFFFFFF
    return word - 0x100000000 if word >= 0x80000000 else word


def step_once(name: str, new: int, orig: int, source: str, decim: int,
              pre_ms: int, hold_ms: int) -> dict:
    """Arm, write the step, hold, restore. One OpenOCD call, no I2C in between."""
    hdr = sym_cache.get(ELF, "g_motor_trace")
    sym = sym_cache.get(ELF, name)
    src_id = next(k for k, (n, _) in trace_dump.SOURCES.items() if n == source)
    vq_max = sym_cache.get(ELF, "g_motor_foc_vq_max_uv")
    vd_max = sym_cache.get(ELF, "g_motor_foc_vd_max_uv")
    script = "\n".join([
        f"mwb 0x{hdr + 20:08x} 0",
        f"mwh 0x{hdr + 8:08x} 0",
        f"mwh 0x{hdr + 10:08x} {decim}",
        f"mwh 0x{hdr + 12:08x} 0",
        f"mww 0x{hdr + 16:08x} 0",
        f"mwb 0x{hdr + 21:08x} 0",
        f"mwb 0x{hdr + 22:08x} {src_id}",
        f"mwb 0x{hdr + 20:08x} 1",
        f"sleep {pre_ms}",
        f"mww 0x{sym:08x} {new & 0xFFFFFFFF}",
        f"set p1 [mrw 0x{hdr + 16:08x}]",
        f"sleep {hold_ms}",
        f"set rb [mrw 0x{sym:08x}]",
        f"mww 0x{sym:08x} {orig & 0xFFFFFFFF}",
        f"set w8 [mrw 0x{hdr + 8:08x}]",
        f"set w20 [mrw 0x{hdr + 20:08x}]",
        f"set rb2 [mrw 0x{sym:08x}]",
        f"set vqmax [mrw 0x{vq_max:08x}]",
        f"set vdmax [mrw 0x{vd_max:08x}]",
        'set _out "p1 $p1 rb $rb w8 $w8 w20 $w20 rb2 $rb2 vqmax $vqmax vdmax $vdmax"',
    ])
    text = ocd([script])
    got = {}
    for key, raw in re.findall(r"(p1|rb|w8|w20|rb2|vqmax|vdmax) (0x[0-9a-fA-F]+|\d+)", text):
        got[key] = s32(int(raw, 0))
    missing = [k for k in ("p1", "rb", "w8", "w20", "rb2", "vqmax", "vdmax") if k not in got]
    if missing:
        raise HoldFail(f"阶跃读数缺 {missing}：{text[-400:]}", 5)
    write_idx = got["w8"] & 0xFFFF
    mode = got["w20"] & 0xFF
    wrapped = (got["w20"] >> 8) & 0xFF
    return {
        "p1": got["p1"],
        "rb": got["rb"],
        "rb2": got["rb2"],
        "write_idx": write_idx,
        "mode": mode,
        "wrapped": wrapped,
        "full": mode == 0 and wrapped == 1 and write_idx == 160,
        "vq_max_uv": got["vqmax"],
        "vd_max_uv": got["vdmax"],
    }


def find_k0(path: Path, orig: int) -> dict:
    """Rebuild the q-axis command from vq and iq. Only valid while vq is not clamped."""
    with path.open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows or "vq_mv" not in rows[0] or "iq_lsb" not in rows[0]:
        return {}
    vq = [int(float(r["vq_mv"])) for r in rows]
    iq = [int(float(r["iq_lsb"])) for r in rows]
    e = float(orig - iq[0])
    ref = [iq[0] + e]
    for k in range(1, len(iq)):
        e = (vq[k] - vq[k - 1] + KP_MV * e) / KP_KI_MV
        ref.append(iq[k] + e)
    k0 = None
    for k in range(1, len(ref)):
        if ref[k] - ref[k - 1] > 3.0:
            k0 = k
            break
    out = {"k0": k0, "ref": ref}
    if k0 is not None and k0 >= 2:
        out["ref_before"] = statistics.mean(ref[:k0])
        out["ref_after"] = statistics.mean(ref[k0:])
    return out


def dump_trace(source: str, decim: int, path: Path, arm: bool = True) -> None:
    cmd = [
        sys.executable, str(FW / "scripts" / "trace_dump.py"),
        "--elf", str(ELF),
        "--source", source,
        "--decim", str(decim),
        "--timeout", "2",
        "--no-plot",
        "--csv", str(path),
    ]
    if arm:
        cmd[4:4] = ["--arm", "oneshot"]
    r = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if r.returncode != 0:
        sys.stderr.write(r.stdout + r.stderr)
        raise HoldFail("轨迹没有记下来", 5)


def window_path(base: Path, index: int, count: int) -> Path:
    if count == 1:
        return base
    return base.with_name(f"{base.stem}-{index}{base.suffix}")


def main() -> int:
    ap = argparse.ArgumentParser(description="停稳后采一窗或几窗轨迹，然后停电机")
    ap.add_argument("--mode", choices=sorted(STATE), required=True)
    ap.add_argument("--dir", type=int, choices=(0, 1), required=True)
    ap.add_argument("--iq", type=int, required=True, help="q 轴指令，毫安")
    ap.add_argument("--w", type=int, default=0, help="速度指令，电周期每秒")
    ap.add_argument("--source", default="foc",
                    choices=["hall6", "foc", "adc", "angle", "foc_ang", "foc_v"])
    ap.add_argument("--decim", type=int, default=8)
    ap.add_argument("--csv", type=Path, required=True)
    ap.add_argument("--hall", type=int, help="停稳后必须是这个霍尔，否则不采")
    ap.add_argument("--any-motion", action="store_true",
                    help="不等转子停稳，进入模式就采（慢转时用）")
    ap.add_argument("--mwb", action="append", default=[], metavar="NAME=N",
                    help="采之前写一个字节，可重复")
    ap.add_argument("--restore-mwb", action="append", default=[], metavar="NAME=N",
                    help="采完写回，失败也会写回")
    ap.add_argument("--ab", metavar="NAME=V1,V2,...",
                    help="同一次停稳里，每个值采一窗；采完写回原值")
    ap.add_argument("--step", metavar="NAME=N",
                    help="每窗武装后立刻把这个 32 位字写成 N，保持后再写回原值")
    ap.add_argument("--repeat", type=int, default=1, help="阶跃重复几窗")
    ap.add_argument("--pre-ms", type=int, default=0, help="武装后、写入前等待的毫秒")
    ap.add_argument("--hold-ms", type=int, default=10, help="写入后保持的毫秒，要长过窗口")
    ap.add_argument("--nudge-iq", type=int,
                    help="霍尔不对时，用这个毫安最多挪 3 秒，然后立刻回到 --iq")
    ap.add_argument("--purpose", help="这次在验证什么。记进台账，方便以后查")
    ap.add_argument("--timeout", type=float, default=8.0, help="等进入模式的秒数")
    ap.add_argument("--no-bump", action="store_true",
                    help="六步超过 3 秒仍不交接时，不要把占空比改成 22")
    args = ap.parse_args()

    os.environ["TARS_CDC"] = os.environ.get("TARS_CDC", TARS_PORT)
    if not ELF.is_file():
        raise HoldFail(f"找不到 {ELF}，要先编译", 2)

    if args.step and args.ab:
        raise HoldFail("--step 和 --ab 不要同时用", 2)
    step_name, step_new = parse_assign(args.step) if args.step else (None, None)
    window_s = 160 * 50e-6 * max(args.decim, 1)
    if step_name is not None and args.hold_ms < window_s * 1000:
        raise HoldFail(f"--hold-ms {args.hold_ms} 短于窗口 {window_s * 1000:.1f} 毫秒", 2)
    pokes = [parse_assign(s) for s in args.mwb]
    restores = [parse_assign(s) for s in args.restore_mwb]
    ab_name, ab_values = parse_ab(args.ab) if args.ab else (None, [None])
    base = args.csv.resolve()

    t0 = time.time()
    ser = None
    code = 0
    windows: list[tuple[int | None, Path, dict]] = []
    amps = None
    stopped = False
    ab_orig = None
    step_orig = None
    try:
        ensure_ocd()
        ser = tars_cdc.open_session()
        say(ser, "nodebus wr 0x50 0x91 0x02", STAT_WAIT)
        say(ser, f"nodebus mot start {args.mode} {args.dir} {args.iq} {args.w}",
            START_WAIT)
        wait_mode(ser, STATE[args.mode], args.timeout, bump=not args.no_bump)
        if not args.any_motion:
            wait_still(ser, args.hall, args.iq, args.nudge_iq)
        for name, value in pokes:
            mwb(name, value)
        if step_name is not None:
            step_orig = s32(int(ocd([f"mrw 0x{sym_cache.get(ELF, step_name):08x}"]).strip(), 0))
            prev = read_stat(ser)
            prev_t = time.time()
            nwin = args.repeat
            for i in range(1, nwin + 1):
                if not args.any_motion:
                    wait_still(ser, args.hall, args.iq, args.nudge_iq)
                info = step_once(step_name, step_new, step_orig, args.source,
                                 args.decim, args.pre_ms, args.hold_ms)
                path = window_path(base, i, nwin)
                dump_trace(args.source, args.decim, path, arm=False)
                st = read_stat(ser)
                now = time.time()
                summary = summarize(path)
                summary.update(info)
                summary["stat"] = st
                summary["elec_per_s"] = (st["acc"] - prev["acc"]) / 65536.0 / (now - prev_t)
                k0 = find_k0(path, step_orig)
                summary["k0"] = k0.get("k0")
                summary["ref_before"] = k0.get("ref_before")
                summary["ref_after"] = k0.get("ref_after")
                windows.append((step_new, path, summary))
                prev, prev_t = st, now
                if st["st"] != STATE[args.mode] or st["flt"] != 0:
                    raise HoldFail(
                        f"采集中离开模式 状态 {st['st']} flt=0x{st['flt']:04x}", 4)
        else:
            if ab_name is not None:
                ab_orig = read_byte(ab_name)
            for i, value in enumerate(ab_values, start=1):
                if ab_name is not None:
                    mwb(ab_name, value)
                    time.sleep(SETTLE_S)
                path = window_path(base, i, len(ab_values))
                dump_trace(args.source, args.decim, path)
                st = read_stat(ser)
                summary = summarize(path)
                summary["stat"] = st
                windows.append((value, path, summary))
                if st["st"] != STATE[args.mode] or st["flt"] != 0:
                    raise HoldFail(
                        f"采集中离开模式 状态 {st['st']} flt=0x{st['flt']:04x}", 4)
        if args.hall is not None:
            for _, _, s in windows:
                if s.get("halls") not in (None, [args.hall]):
                    code = 3
    except HoldFail as exc:
        sys.stderr.write(str(exc) + "\n")
        code = exc.code
    finally:
        if step_name is not None and step_orig is not None:
            try:
                mww(step_name, step_orig)
            except SystemExit as exc:
                sys.stderr.write(str(exc) + "\n")
                code = code or 5
        if ab_name is not None and ab_orig is not None:
            try:
                mwb(ab_name, ab_orig)
            except SystemExit as exc:
                sys.stderr.write(str(exc) + "\n")
                code = code or 5
        for name, value in restores:
            try:
                mwb(name, value)
            except SystemExit as exc:
                sys.stderr.write(str(exc) + "\n")
                code = code or 5
        if ser is not None:
            try:
                say(ser, "nodebus mot stop", STAT_WAIT)
                stopped = read_stat(ser)["st"] == 0
            except SystemExit as exc:
                sys.stderr.write(str(exc) + "\n")
            if not stopped:
                code = code or 4
            ser.close()
        amps = lab_current()
        if amps is not None and amps > 0.50:
            sys.stderr.write(f"24 伏电流 {amps:.3f} 安，超过 0.50 安\n")
            code = code or 4
    print_report(ab_name, ab_orig, windows, amps, stopped, time.time() - t0, code,
                 step_name, step_orig)
    record_hold(args, windows, amps, stopped, code)
    return code


def record_hold(args, windows, amps, stopped, code) -> None:
    bits = [f"{args.mode} 方向{args.dir} 指令 {args.iq} 毫安"]
    if args.step:
        bits.append(f"阶跃 {args.step}")
    if args.ab:
        bits.append(f"对照 {args.ab}")
    if getattr(args, "w", None):
        bits.append(f"目标 {args.w} 电周期每秒")
    bits.append("采到了" if code == 0 else f"没采成，代码 {code}")
    auto = "，".join(bits)
    purpose = args.purpose.strip() if args.purpose else auto

    lines = [auto]
    halls = []
    iq_ma = []
    for i, (value, path, s) in enumerate(windows, start=1):
        st = s.get("stat") or {}
        if st.get("hall") is not None:
            halls.append(st["hall"])
        item = f"窗{i}"
        if value is not None:
            item += f" 值{value}"
        if "iq_mean" in s:
            ma = s["iq_mean"] * MA_PER_LSB
            iq_ma.append(ma)
            item += f" iq {ma:.1f} 毫安"
        if st.get("hall") is not None:
            item += f" 霍尔{st['hall']}"
        if st.get("flt"):
            item += f" 故障0x{st['flt']:04x}"
        if "elec_per_s" in s:
            item += f" {s['elec_per_s']:+.2f} 电周期每秒"
        item += f" {path.name}"
        lines.append(item)
    if amps is None:
        lines.append("24伏电流没读到")
    else:
        lines.append(f"24伏电流 {amps:.3f} 安")
    lines.append("电机已停" if stopped else "电机没确认停下")

    values = [f"code={code}", f"stopped={1 if stopped else 0}"]
    if amps is not None:
        values.append(f"i24={amps:.4f}A")
    if halls:
        values.append(f"hall={halls[-1]}")
    if iq_ma:
        values.append(f"iq={sum(iq_ma) / len(iq_ma):.1f}mA")
    lab_record.record(
        purpose, files=[path for _, path, _ in windows], values=values,
        notes="\n".join(lines), instrument="foc_hold.py", tags="foc",
    )


def print_report(ab_name, ab_orig, windows, amps, stopped, seconds, code,
                 step_name=None, step_orig=None) -> None:
    print("结论\t" + ("采到了" if code == 0 else f"没采成（代码 {code}）"))
    if step_name is not None and step_orig is not None:
        print(f"阶跃\t{step_name} 原值 {step_orig}，已写回")
    if ab_name is not None and ab_orig is not None:
        print(f"开关\t{ab_name}（原值 {ab_orig}，已写回）")
    elif ab_name is not None:
        print(f"开关\t{ab_name}（没动过）")
    head = ["窗", "值", "霍尔", "iq刻度", "iq毫安", "iq标准差", "iq最小", "iq最大",
            "id刻度", "点数", "csv"]
    print("\t".join(head))
    for i, (value, path, s) in enumerate(windows, start=1):
        if "iq_mean" not in s:
            phases = " ".join(f"{c[:2]} {s[c][0]:.1f}±{s[c][1]:.1f}"
                              for c in ("iu_lsb", "iv_lsb", "iw_lsb") if c in s)
            print(f"{i}\t{'-' if value is None else value}\t{phases}\t{s['n']}\t{path}")
        else:
            cells = [
                str(i),
                "-" if value is None else str(value),
                ",".join(str(h) for h in s.get("halls", [])) or "-",
                f"{s['iq_mean']:.3f}",
                f"{s['iq_mean'] * MA_PER_LSB:.1f}",
                f"{s['iq_stdev']:.2f}",
                str(s["iq_min"]),
                str(s["iq_max"]),
                f"{s['id_mean']:.3f}" if "id_mean" in s else "-",
                str(s["n"]),
                str(path),
            ]
            print("\t".join(cells))
        if "vq_mean" in s:
            print(f"\tvq 平均 {s['vq_mean']:.1f} 毫伏，最小 {s['vq_min']}，"
                  f"最大 {s['vq_max']}，为 0 的点 {s['vq_zero']}")
        if "vd_mean" in s:
            print(f"\tvd 平均 {s['vd_mean']:.1f} 毫伏，绝对值最大 {s['vd_absmax']}")
        if "sat_n" in s:
            print(f"\tsat 点数 {s['sat_n']}")
        if "p1" in s:
            print(f"\tp1 {s['p1']} 写回前 {s['rb']} 写回后 {s['rb2']} "
                  f"满 {int(s['full'])} k0 {s.get('k0')} "
                  f"ref前 {s.get('ref_before')} ref后 {s.get('ref_after')}")
            if s.get("vq_max_uv") is not None:
                print(f"\tvq上限 {s['vq_max_uv']} 微伏 vd上限 {s['vd_max_uv']} 微伏")
        if "elec_per_s" in s:
            print(f"\t这段转速 {s['elec_per_s']:+.2f} 电周期每秒")
        st = s.get("stat")
        if st:
            print(f"\t采后状态 st={st['st']} w={st['w']} flt=0x{st['flt']:04x} "
                  f"hall={st['hall']} acc={st['acc']}")
    print("24伏安\t" + ("没读到" if amps is None else f"{amps:.3f}"))
    print("电机\t" + ("已停" if stopped else "没确认停下，去看一眼"))
    print(f"用时秒\t{seconds:.1f}")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except HoldFail as exc:
        sys.stderr.write(str(exc) + "\n")
        raise SystemExit(exc.code)
