#!/usr/bin/env python3
"""Stationary inverter voltage sweep.

Writes g_vsweep_req, waits until the firmware finishes, then dumps the trace.
Does not start the current loop. The motor must already be stopped and 24 V on.
Each point is one phase pair: one leg PWM, one leg held low, the third off.
Dead-time compensation is off. A polarity stops at 0.50 A.

tag bits 0..9 CCR, 10..11 pair (0 U-V, 1 V-W, 2 W-U), bit 12 swapped, bit 13 hit 0.50 A.
Currents are shunt LSB minus the bridge-off zero. 1 LSB = 16.17 mA. 1 CCR = 24 V / 1199.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lab_record
import ocd_rpc
import sym_cache

FW = Path(__file__).resolve().parents[1]
ELF = FW / "build" / "Release" / "motor-ctrl.elf"
ARR = 1199
VBUS_MV = 24000
MA_PER_LSB = 16.17
PAIRS = ("U-V", "V-W", "W-U")


def mdb(addr: int) -> int:
    """One byte. mrw on an odd address does not return that byte."""
    text = ocd_rpc.rpc([f"mdb 0x{addr:08x} 1"]).strip().split()[-1]
    return int(text, 16)


def record_sweep(purpose, csv_path, st, hall_begin, hall_end, peak_ma, amps, code) -> None:
    auto = "静止扫电压，死区补偿关"
    if st == 2:
        auto += "，做完"
    elif st == 3:
        auto += "，电流到限中止"
    else:
        auto += "，没做完"
    lines = [auto]
    if hall_begin is not None:
        lines.append(f"霍尔开始 {hall_begin} 结束 {hall_end}")
    if peak_ma is not None:
        lines.append(f"相电流峰值 {peak_ma:.0f} 毫安")
    if amps is None:
        lines.append("24伏电流没读到")
    else:
        lines.append(f"24伏电流 {amps:.3f} 安")
    values = [f"code={code}"]
    if st is not None:
        values.append(f"state={st}")
    if hall_end is not None:
        values.append(f"hall={hall_end}")
    if hall_begin is not None:
        values.append(f"hall_begin={hall_begin}")
    if peak_ma is not None:
        values.append(f"ipeak={peak_ma:.0f}mA")
    if amps is not None:
        values.append(f"i24={amps:.4f}A")
    lab_record.record(
        purpose or auto, files=[csv_path] if csv_path is not None else [],
        values=values, notes="\n".join(lines), instrument="vsweep.py", tags="vsweep",
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="静止扫电压，做完记进台账")
    ap.add_argument("csv", nargs="?", type=Path, default=Path("/tmp/vsweep.csv"))
    ap.add_argument("--purpose", help="这次在验证什么。记进台账，方便以后查")
    args = ap.parse_args()
    if not ELF.is_file():
        print(f"找不到 {ELF}", file=sys.stderr)
        return 2
    req = sym_cache.get(ELF, "g_vsweep_req")
    state = sym_cache.get(ELF, "g_vsweep_state")
    ocd_rpc.rpc([f"mwb 0x{req:08x} 1"])
    deadline = time.time() + 8.0
    st = 0
    while time.time() < deadline:
        st = mdb(state)
        if st in (2, 3):
            break
        time.sleep(0.2)
    else:
        print("扫电压没有在 8 秒内结束", file=sys.stderr)
        record_sweep(args.purpose, None, st, None, None, None, lab_record.bus_current_a(), 4)
        return 4
    import subprocess
    r = subprocess.run(
        [sys.executable, str(FW / "scripts" / "trace_dump.py"),
         "--elf", str(ELF), "--no-plot", "--csv", str(args.csv)],
        check=False)
    if r.returncode != 0:
        record_sweep(args.purpose, args.csv, st, None, None, None,
                     lab_record.bus_current_a(), r.returncode)
        return r.returncode
    hall_begin = mdb(sym_cache.get(ELF, "g_vsweep_hall_begin"))
    hall_end = mdb(sym_cache.get(ELF, "g_vsweep_hall_end"))
    print(f"状态 {st}  霍尔开始 {hall_begin}  结束 {hall_end}")
    peak_ma = 0.0
    with args.csv.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            tag = int(float(row["tag"])) & 0xFFFF
            ccr = tag & 0x3FF
            pair = PAIRS[(tag >> 10) & 3]
            swap = "反" if tag & (1 << 12) else "正"
            stop = "停" if tag & (1 << 13) else ""
            iu = int(float(row["iu_lsb"]))
            iv = int(float(row["iv_lsb"]))
            iw = int(float(row["iw_lsb"]))
            peak_ma = max(peak_ma, abs(iu) * MA_PER_LSB, abs(iv) * MA_PER_LSB,
                          abs(iw) * MA_PER_LSB)
            mv = ccr * VBUS_MV / ARR
            print(f"{pair} {swap}  {ccr:3d} 计数 {mv:6.0f} 毫伏  "
                  f"U {iu * MA_PER_LSB:7.0f}  V {iv * MA_PER_LSB:7.0f}  "
                  f"W {iw * MA_PER_LSB:7.0f} 毫安 {stop}")
    code = 0 if st == 2 else 3
    record_sweep(args.purpose, args.csv, st, hall_begin, hall_end, peak_ma,
                 lab_record.bus_current_a(), code)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
