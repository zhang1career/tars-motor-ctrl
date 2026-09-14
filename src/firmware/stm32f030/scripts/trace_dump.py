#!/usr/bin/env python3
"""Dump the on-chip trace buffer over SWD and plot it.

Never halts the target: Cortex-M memory reads go through the DAP, so a capture
of a running control loop is not perturbed by reading it out.

Addresses come from the ELF, never hardcoded -- see docs/measurement-validity.md
section 1.3.1 for what hardcoding them cost last time.

  ./trace_dump.py                    # read whatever is in the buffer
  ./trace_dump.py --arm oneshot      # re-arm one-shot, wait for full, read
  ./trace_dump.py --no-plot -o /tmp  # CSV only
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import struct
import subprocess
import sys
import time
from pathlib import Path

FW_ROOT = Path(__file__).resolve().parent.parent
REPO_ROOT = FW_ROOT.parent.parent.parent
DEFAULT_ELF = FW_ROOT / "build" / "Release" / "motor-ctrl.elf"
DEFAULT_OUT = REPO_ROOT / "models" / "captured"

MAGIC = 0x54524331
MODE_NAMES = {0: "idle", 1: "oneshot", 2: "wrap"}
# Channel meaning per producer, so a dump is self-describing.
SOURCES = {
    0: ("none", ["ch0", "ch1", "ch2", "ch3"]),
    1: ("hall6", ["hall_raw", "step", "ticks_since_edge", "kick"]),
    2: ("foc", ["id_lsb", "iq_lsb", "theta_q15", "hall_raw"]),
    3: ("adc", ["iu_lsb", "iv_lsb", "iw_lsb", "vbus_raw"]),
    4: ("angle", ["hall_raw", "theta_q15", "ticks_in_sector", "edge_jump"]),
    5: ("foc_ang", ["foc_disc_q15", "foc_ip_q15", "dth_q15", "hall_raw"]),
    6: ("foc_v", ["vd_mv", "vq_mv", "sat", "iq_lsb"]),
}

DEG_PER_Q15 = 360.0 / 32768.0   # theta is stored as theta_q16 >> 1
DEG_PER_Q16 = 360.0 / 65536.0

MA_PER_LSB = 3.3124 / 4096 / (50 * 0.010) * 1000  # INA240A2 gain 50, 10 mOhm
VBUS_V_PER_LSB = 3.3124 / 4095 * 4.9              # R30 39k / R31 10k


def find_nm() -> str:
    hits = sorted(glob.glob(
        "/Applications/ArmGNUToolchain/*/arm-none-eabi/bin/arm-none-eabi-nm"))
    return hits[-1] if hits else "arm-none-eabi-nm"


def sym_addr(elf: Path, name: str) -> int:
    out = subprocess.run([find_nm(), str(elf)], capture_output=True, text=True,
                         check=True).stdout
    hits = re.findall(rf"^([0-9a-f]{{8}}) [bBdD] {re.escape(name)}$", out, re.M)
    if not hits:
        sys.exit(f"symbol '{name}' not found in {elf}")
    if len(hits) > 1:
        sys.exit(f"symbol '{name}' is ambiguous in {elf}: {hits}")
    return int(hits[0], 16)


def ocd(cfg: Path, cmds: list[str]) -> str:
    argv = ["openocd", "-f", str(cfg), "-c", "init"]
    for c in cmds:
        argv += ["-c", c]
    argv += ["-c", "exit"]
    r = subprocess.run(argv, capture_output=True, text=True)
    return r.stdout + r.stderr


def read_words(cfg: Path, addr: int, n: int) -> list[int]:
    text = ocd(cfg, [f"mdw 0x{addr:08x} {n}"])
    words: list[int] = []
    for line in text.splitlines():
        m = re.match(r"^0x[0-9a-f]+:\s+(.*)$", line)
        if m:
            words += [int(x, 16) for x in m.group(1).split()]
    if len(words) < n:
        sys.exit(f"read {len(words)} of {n} words at 0x{addr:08x}; "
                 f"is the probe attached?\n{text[-800:]}")
    return words[:n]


def parse_header(words: list[int]) -> dict:
    raw = b"".join(struct.pack("<I", w) for w in words)
    magic, depth, channels, write_idx, decim, decim_count = struct.unpack_from(
        "<IHHHHH", raw, 0)
    pushes, mode, wrapped, source, _ = struct.unpack_from("<IBBBB", raw, 16)
    if magic != MAGIC:
        sys.exit(f"trace magic is 0x{magic:08x}, expected 0x{MAGIC:08x}: "
                 f"firmware built without -DMOTOR_TRACE=ON, or never armed")
    return dict(depth=depth, channels=channels, write_idx=write_idx,
                decim=decim, decim_count=decim_count, pushes=pushes,
                mode=mode, wrapped=bool(wrapped), source=source)


def freeze(cfg: Path, hdr_addr: int) -> None:
    """Stop recording before reading the buffer out.

    The buffer must be static while it is read. 256 samples span 12.8 ms at
    20 kHz, but reading 512 words over SWD takes about a second, so a wrap-mode
    buffer is overwritten ~80 times mid-read. The result looks like a waveform
    and is really dozens of unrelated fragments spliced together -- it showed up
    as hall edges whose ticks_since_edge was not zero, which is impossible.

    Freezing only stops recording; the motor keeps running.
    """
    ocd(cfg, [f"mwb 0x{hdr_addr + 20:08x} 0"])


def arm(cfg: Path, hdr_addr: int, mode: int, decim: int,
        source: int | None) -> None:
    # Order matters: clear the indices while idle, publish the mode last, so the
    # ISR cannot start filling against a half-written header.
    cmds = [
        f"mwb 0x{hdr_addr + 20:08x} 0",                    # mode = idle
        f"mwh 0x{hdr_addr + 8:08x} 0",                     # write_idx
        f"mwh 0x{hdr_addr + 10:08x} {decim}",              # decim
        f"mwh 0x{hdr_addr + 12:08x} 0",                    # decim_count
        f"mww 0x{hdr_addr + 16:08x} 0",                    # pushes
        f"mwb 0x{hdr_addr + 21:08x} 0",                    # wrapped
    ]
    if source is not None:
        # The producer to capture is a host-side choice: each ISR producer only
        # pushes when the header names it.
        cmds.append(f"mwb 0x{hdr_addr + 22:08x} {source}")
    cmds.append(f"mwb 0x{hdr_addr + 20:08x} {mode}")       # mode last
    ocd(cfg, cmds)


def samples_in_time_order(words: list[int], hdr: dict) -> list[list[int]]:
    depth, ch = hdr["depth"], hdr["channels"]
    flat: list[int] = []
    for w in words:
        for half in (w & 0xFFFF, (w >> 16) & 0xFFFF):
            flat.append(half - 0x10000 if half >= 0x8000 else half)
    rows = [flat[i * ch:(i + 1) * ch] for i in range(depth)]

    # Ordering from write_idx/wrapped rather than mode, because freezing before
    # the read sets mode back to idle.
    if not hdr["wrapped"]:
        return rows[:hdr["write_idx"]]          # partially filled
    if hdr["write_idx"] >= depth:
        return rows                            # one-shot, filled exactly once
    start = hdr["write_idx"] % depth           # wrap mode: oldest is the next slot
    return rows[start:] + rows[:start]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--elf", type=Path, default=DEFAULT_ELF)
    ap.add_argument("--cfg", type=Path, default=FW_ROOT / "openocd.cfg")
    ap.add_argument("--arm", choices=["oneshot", "wrap"])
    ap.add_argument("--decim", type=int, default=1)
    ap.add_argument("--source", choices=sorted(
        n for n, _ in SOURCES.values() if n != "none"),
        help="which ISR producer to capture (default: leave as firmware set it)")
    ap.add_argument("--timeout", type=float, default=10.0,
                    help="seconds to wait for a one-shot buffer to fill")
    ap.add_argument("-o", "--outdir", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--json", type=Path,
                    help="also write per-channel statistics for other scripts")
    ap.add_argument("--csv", type=Path,
                    help="write the CSV here instead of a timestamped name")
    args = ap.parse_args()

    hdr_addr = sym_addr(args.elf, "g_motor_trace")
    buf_addr = sym_addr(args.elf, "g_motor_trace_buf")
    if buf_addr % 4:
        sys.exit(f"trace buffer at 0x{buf_addr:08x} is not word aligned, and "
                 f"openocd's mdw needs it to be. Rebuild: the buffer carries "
                 f"__attribute__((aligned(4))) for exactly this reason.")

    src_id = None
    if args.source:
        src_id = next(k for k, (n, _) in SOURCES.items() if n == args.source)

    if args.arm:
        arm(args.cfg, hdr_addr, 1 if args.arm == "oneshot" else 2, args.decim,
            src_id)
        if args.arm == "oneshot":
            deadline = time.time() + args.timeout
            while time.time() < deadline:
                hdr = parse_header(read_words(args.cfg, hdr_addr, 6))
                if hdr["mode"] == 0 and hdr["wrapped"]:
                    break
                time.sleep(0.2)
            else:
                print("warning: one-shot did not fill within the timeout; "
                      "is the control loop running?", file=sys.stderr)

    hdr = parse_header(read_words(args.cfg, hdr_addr, 6))
    if hdr["mode"] != 0:
        freeze(args.cfg, hdr_addr)
        hdr = parse_header(read_words(args.cfg, hdr_addr, 6))
        print("froze recording so the buffer is static during the read")

    src_name, labels = SOURCES.get(hdr["source"], SOURCES[0])
    labels = labels[:hdr["channels"]]

    print(f"source     {src_name} (id {hdr['source']})")
    print(f"mode       {MODE_NAMES.get(hdr['mode'], hdr['mode'])}"
          f"  wrapped={hdr['wrapped']}  write_idx={hdr['write_idx']}")
    print(f"geometry   {hdr['depth']} samples x {hdr['channels']} channels, "
          f"decimation {hdr['decim']}")
    print(f"pushes     {hdr['pushes']}")

    nwords = hdr["depth"] * hdr["channels"] // 2
    rows = samples_in_time_order(read_words(args.cfg, buf_addr, nwords), hdr)
    if not rows:
        sys.exit("buffer is empty: arm it first (--arm wrap) and let the loop run")

    dt_us = 1e6 / 20000 * max(hdr["decim"], 1)
    print(f"captured   {len(rows)} samples, {dt_us:.0f} us apart "
          f"= {len(rows) * dt_us / 1000:.1f} ms")

    args.outdir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = args.csv if args.csv else args.outdir / f"trace-{src_name}-{stamp}.csv"
    if args.csv:
        csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", encoding="utf-8") as f:
        f.write("t_us," + ",".join(labels) + "\n")
        for i, row in enumerate(rows):
            f.write(f"{i * dt_us:.1f}," + ",".join(str(v) for v in row) + "\n")
    print(f"csv        {csv_path}")

    if hdr["source"] == 1:
        codes = [r[0] for r in rows]
        ticks = [r[2] for r in rows]
        distinct = sorted(set(codes))
        edge_idx = [i for i in range(1, len(rows)) if codes[i] != codes[i - 1]]
        bad = [c for c in distinct if c in (0, 7)]
        print(f"hall codes {distinct}  edges {len(edge_idx)}"
              + ("  <-- INVALID CODE PRESENT" if bad else ""))

        # Coherence check. The ISR zeroes ticks_since_edge on the tick it first
        # sees a new hall code, and ticks_since_edge counts real ticks rather
        # than stored samples, so with decimation N the first stored sample after
        # an edge carries a value below N. Anything larger means the samples are
        # not consecutive -- the buffer was still being written while it was read.
        limit = max(hdr["decim"], 1)
        torn = [i for i in edge_idx if ticks[i] >= limit]
        if torn:
            print(f"TORN CAPTURE: {len(torn)} of {len(edge_idx)} edges carry "
                  f"ticks_since_edge >= {limit}, so these samples are not "
                  f"consecutive. Time axis and any speed from it are invalid.")
        else:
            print(f"coherent   every hall edge carries ticks_since_edge < {limit}")

        if len(edge_idx) >= 2 and not torn:
            # Exact sector length in control ticks, independent of decimation.
            # ticks_since_edge counts real ticks, so for an edge whose first
            # stored sample reads k, the previous stored sample sits (decim - k)
            # ticks before the edge and holds sector - (decim - k).
            sectors = []
            for i, j in zip(edge_idx, edge_idx[1:]):
                sectors.append((codes[i], codes[j],
                                ticks[j - 1] + (limit - ticks[j])))
            widths = [s for _, _, s in sectors]
            mean = sum(widths) / len(widths)
            tick_us = 1e6 / 20000
            print(f"mean sector {mean:.1f} ticks = {mean * tick_us:.0f} us")
            print(f"speed      {1e6 / (mean * tick_us * 6):.2f} electrical rev/s")
            print("sector     from->to   ticks       us    deg (of 360/6=60)")
            for frm, to, s in sectors:
                print(f"             {frm}->{to}    {s:5d}  {s * tick_us:7.0f}  "
                      f"{s / mean * 60:8.2f}")
            spread = max(widths) - min(widths)
            print(f"uniformity spread {spread} ticks = {spread / mean * 60:.1f} deg")
            if len(widths) > 6:
                rep = [widths[k + 6] - widths[k] for k in range(len(widths) - 6)]
                print(f"repeatability same sector one revolution later differs by "
                      f"{rep} ticks -- small values mean the non-uniformity is "
                      f"systematic, not noise")

    if hdr["source"] == 3:
        import statistics

        cols = [[r[k] for r in rows] for k in range(4)]
        print()
        print("phase   mean          stdev        min     max      (shunt sign: "
              "positive = current leaving the phase through the low side)")
        for k, name in enumerate("UVW"):
            c = cols[k]
            mean = statistics.mean(c)
            sd = statistics.pstdev(c)
            print(f"  {name}   {mean * MA_PER_LSB:+8.1f} mA   "
                  f"{sd * MA_PER_LSB:6.1f} mA   {min(c) * MA_PER_LSB:+7.0f} "
                  f"{max(c) * MA_PER_LSB:+7.0f}")
        resid = [a + b + c for a, b, c in zip(cols[0], cols[1], cols[2])]
        rms = (sum(v * v for v in resid) / len(resid)) ** 0.5
        # Full scale is +-2.9 A, so 5% of full scale is 145 mA (roadmap stage C).
        print(f"  sum   {statistics.mean(resid) * MA_PER_LSB:+8.1f} mA   "
              f"rms {rms * MA_PER_LSB:6.1f} mA   "
              f"{'ok' if rms * MA_PER_LSB < 145 else 'FAIL: over 5% of full scale'}")

        vb = cols[3]
        vmean = statistics.mean(vb)
        print(f"  vbus  {vmean * VBUS_V_PER_LSB:8.2f} V    raw {vmean:.0f}")
        # A distinctive value on the last channel is also the check that the DMA
        # ring and the conversion sequence are still in step: bus voltage sits
        # near 3020 counts while the current channels sit near 2048, so a
        # rotated mapping is obvious rather than silent.
        if not 2800 <= vmean <= 3300:
            print("  WARNING: vbus channel is out of range. Either the bus is "
                  "not at 12 V, or the ADC sequence and DMA ring have slipped "
                  "and the channels are rotated.")

    if hdr["source"] == 4:
        import statistics

        hall = [r[0] for r in rows]
        theta = [r[1] for r in rows]
        jump = [r[3] for r in rows]
        edge_idx = [i for i in range(1, len(rows)) if hall[i] != hall[i - 1]]

        print()
        # theta must advance monotonically apart from the wrap; a step of the
        # wrong sign means the extrapolation direction is wrong.
        steps = []
        for a, b in zip(theta, theta[1:]):
            d = b - a
            if d < -16384:
                d += 32768
            elif d > 16384:
                d -= 32768
            steps.append(d)
        fwd = sum(1 for s in steps if s > 0)
        print(f"theta      advances on {fwd}/{len(steps)} ticks; mean step "
              f"{statistics.mean(steps) * DEG_PER_Q15:+.3f} deg")
        rate = statistics.mean(steps) * DEG_PER_Q15 / (dt_us * 1e-6) / 360
        print(f"speed      {rate:+.2f} electrical rev/s from the theta slope")

        if edge_idx:
            # edge_jump is |extrapolated - anchor| latched at each edge: the
            # direct measure of how well the previous sector predicted this one.
            at_edges = [jump[i] * DEG_PER_Q16 for i in edge_idx]
            print(f"edge jump  {len(at_edges)} edges, mean "
                  f"{statistics.mean(at_edges):.2f} deg, max "
                  f"{max(at_edges):.2f} deg   "
                  f"{'ok (roadmap wants < 10)' if max(at_edges) < 10 else 'FAIL: over 10 deg'}")
            secs = [(b - a) for a, b in zip(edge_idx, edge_idx[1:])]
            if secs:
                print(f"sectors    {secs} samples")

    if hdr["source"] == 2:
        import math
        import statistics

        id_lsb = [r[0] for r in rows]
        iq_lsb = [r[1] for r in rows]
        hall = [r[3] for r in rows]
        id_ma = [v * MA_PER_LSB for v in id_lsb]
        iq_ma = [v * MA_PER_LSB for v in iq_lsb]
        mid = statistics.mean(id_ma)
        miq = statistics.mean(iq_ma)
        print()
        print(f"id         {mid:+8.1f} mA   stdev {statistics.pstdev(id_ma):6.1f} mA"
              f"   [{min(id_ma):+.0f} .. {max(id_ma):+.0f}]")
        print(f"iq         {miq:+8.1f} mA   stdev {statistics.pstdev(iq_ma):6.1f} mA"
              f"   [{min(iq_ma):+.0f} .. {max(iq_ma):+.0f}]")
        print(f"|id|/|iq|  {abs(mid) / max(abs(miq), 1e-3):.2f}"
              f"   (6-step current is not pure q; DC id is the alignment cue)")

        # φ from mean(id,iq) is circular once the id PI is on: the loop
        # forces id≈0 in whatever frame Park is using. Only print it for
        # OBSERVE (id_on=0).
        id_on = 1
        try:
            id_on = read_words(args.cfg, sym_addr(args.elf, "g_motor_foc_id_on"), 1)[0] & 0xFF
        except (subprocess.CalledProcessError, SystemExit, IndexError):
            pass
        signed = None
        off_q16 = None
        best_id = mid
        best_iq = miq
        if id_on != 0:
            print("chosen φ   skipped — id PI is on, φ≈0 is not an alignment measurement")
        else:
            a = math.degrees(math.atan2(-mid, miq)) % 360.0
            twins = (a, (a + 180.0) % 360.0)
            scored = []
            for deg in twins:
                phi = math.radians(deg)
                c, s = math.cos(phi), math.sin(phi)
                mids = statistics.mean([id_ * c + iq_ * s for id_, iq_ in zip(id_ma, iq_ma)])
                miqs = statistics.mean([-id_ * s + iq_ * c for id_, iq_ in zip(id_ma, iq_ma)])
                scored.append((deg, mids, miqs))
            print("φ twins    (same |iq|, opposite sign; pick the one that keeps iq sign)")
            for deg, mids, miqs in scored:
                q16 = int(round((deg if deg <= 180 else deg - 360) * 65536 / 360))
                tag = "keep-iq-sign" if (miqs * miq) > 0 else "flips-torque"
                print(f"            {deg:6.1f} deg  Q16={q16:+6d}  "
                      f"id {mids:+7.1f}  iq {miqs:+7.1f}  {tag}")
            keep = next((t for t in scored if t[2] * miq > 0), scored[0])
            best_phi = keep[0]
            best_id = keep[1]
            best_iq = keep[2]
            signed = best_phi if best_phi <= 180 else best_phi - 360
            off_q16 = int(round(signed * 65536 / 360))
            print(f"chosen φ   {signed:+.1f} deg  OFFSET_Q16={off_q16}  "
                  f"then mean id {best_id:+.1f} mA  iq {best_iq:+.1f} mA")

        print("per hall   code   n     mean id     mean iq")
        for code in sorted(set(hall)):
            ids = [id_ma[i] for i, h in enumerate(hall) if h == code]
            iqs = [iq_ma[i] for i, h in enumerate(hall) if h == code]
            if ids:
                print(f"            {code:3d}  {len(ids):3d}  "
                      f"{statistics.mean(ids):+8.1f} mA  "
                      f"{statistics.mean(iqs):+8.1f} mA")

        theta = [r[2] for r in rows]
        net = 0
        for a, b in zip(theta, theta[1:]):
            d = b - a
            if d > 16384:
                d -= 32768
            elif d < -16384:
                d += 32768
            net += d
        net_deg = net * DEG_PER_Q15
        codes = sorted(set(hall))
        edges = sum(1 for i in range(1, len(hall)) if hall[i] != hall[i - 1])
        rotating = len(codes) >= 5 and abs(net_deg) > 180.0
        print(f"motion     halls {codes}  edges {edges}  "
              f"net theta {net_deg:+.1f} deg  "
              f"{'ROTATING' if rotating else 'DITHER (not rotation)'}")
        extra_rot = {"hall_codes": codes, "hall_edges": edges,
                     "net_theta_deg": net_deg, "rotating": rotating}

    if hdr["source"] == 5:
        import statistics

        foc = [r[0] for r in rows]
        interp = [r[1] for r in rows]
        dth = [r[2] for r in rows]
        hall = [r[3] for r in rows]
        dth_deg = [v * DEG_PER_Q15 for v in dth]
        mean_d = statistics.mean(dth_deg)
        sd_d = statistics.pstdev(dth_deg) if len(dth_deg) > 1 else 0.0
        print()
        print(f"dth        FocThetaInterp-FocTheta  mean {mean_d:+.1f} deg  "
              f"stdev {sd_d:.1f} deg  [{min(dth_deg):+.1f} .. {max(dth_deg):+.1f}]")
        print("per hall   code   n    mean dth")
        per = {}
        for code in sorted(set(hall)):
            ds = [dth_deg[i] for i, h in enumerate(hall) if h == code]
            if ds:
                per[int(code)] = statistics.mean(ds)
                print(f"            {int(code):3d}  {len(ds):3d}  "
                      f"{per[int(code)]:+8.1f} deg")
        net = 0
        for a, b in zip(interp, interp[1:]):
            d = b - a
            if d > 16384:
                d -= 32768
            elif d < -16384:
                d += 32768
            net += d
        net_deg = net * DEG_PER_Q15
        codes = sorted(set(int(h) for h in hall))
        edges = sum(1 for i in range(1, len(hall)) if hall[i] != hall[i - 1])
        rotating = len(codes) >= 5 and abs(net_deg) > 180.0
        print(f"motion     halls {codes}  edges {edges}  "
              f"net foc_ip {net_deg:+.1f} deg  "
              f"{'ROTATING' if rotating else 'DITHER (not rotation)'}")
        extra_rot = {
            "hall_codes": codes, "hall_edges": edges,
            "net_theta_deg": net_deg, "rotating": rotating,
            "dth_mean_deg": mean_d, "dth_stdev_deg": sd_d,
            "dth_per_hall_deg": per,
        }

    if hdr["source"] == 6:
        import statistics

        vd = [r[0] / 1000.0 for r in rows]
        vq = [r[1] / 1000.0 for r in rows]
        sat = [r[2] for r in rows]
        iq = [r[3] * MA_PER_LSB for r in rows]
        sat_frac = sum(1 for s in sat if s != 0) / len(sat)
        vq_hi = max(abs(v) for v in vq)
        try:
            ceil_v = int(os.environ["VQMAX_UV"]) / 1e6
        except (KeyError, ValueError):
            ceil_v = 2.4
        limited = vq_hi >= 0.95 * ceil_v
        print()
        print(f"vd         {statistics.mean(vd):+.3f} V   "
              f"[{min(vd):+.3f} .. {max(vd):+.3f}]")
        print(f"vq         {statistics.mean(vq):+.3f} V   "
              f"[{min(vq):+.3f} .. {max(vq):+.3f}]")
        print(f"sat        {sat_frac*100:.1f}% of ticks")
        print(f"iq         {statistics.mean(iq):+.1f} mA   "
              f"stdev {statistics.pstdev(iq):.1f} mA")
        if limited:
            print("volt loop  VOLTAGE-LIMITED — empty-load mean iq is not a "
                  "current-loop score")
        else:
            print("volt loop  both axes inside the ceiling this window")
        extra_v = {
            "vd_mean_v": statistics.mean(vd),
            "vq_mean_v": statistics.mean(vq),
            "sat_frac": sat_frac,
            "iq_mean_ma": statistics.mean(iq),
            "voltage_limited": limited,
        }

    extra = {}
    if hdr["source"] == 2:
        extra = {
            "id_mean_ma": statistics.mean(id_ma),
            "iq_mean_ma": statistics.mean(iq_ma),
            "id_stdev_ma": statistics.pstdev(id_ma),
            "iq_stdev_ma": statistics.pstdev(iq_ma),
            "best_offset_deg": signed,
            "best_offset_q16": off_q16,
            "best_id_mean_ma": best_id,
            "best_iq_mean_ma": best_iq,
        }
        extra.update(extra_rot)
    elif hdr["source"] == 5:
        extra = extra_rot
    elif hdr["source"] == 6:
        extra = extra_v

    if args.json:
        import json
        import statistics

        cols = [[r[k] for r in rows] for k in range(hdr["channels"])]
        payload = {
            "source": src_name,
            "samples": len(rows),
            "dt_us": dt_us,
            "labels": labels,
            "mean_lsb": [statistics.mean(c) for c in cols],
            "stdev_lsb": [statistics.pstdev(c) for c in cols],
            "min_lsb": [min(c) for c in cols],
            "max_lsb": [max(c) for c in cols],
            "ma_per_lsb": MA_PER_LSB,
            "vbus_v_per_lsb": VBUS_V_PER_LSB,
        }
        payload.update(extra)
        args.json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(f"json       {args.json}")

    if not args.no_plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        t = [i * dt_us / 1000 for i in range(len(rows))]
        fig, axes = plt.subplots(len(labels), 1, sharex=True,
                                 figsize=(10, 2.0 * len(labels)))
        for k, (ax, label) in enumerate(zip(axes, labels)):
            ax.plot(t, [r[k] for r in rows], drawstyle="steps-post", lw=1.0)
            ax.set_ylabel(label)
            ax.grid(alpha=0.3)
        axes[-1].set_xlabel("time (ms)")
        axes[0].set_title(f"motor-ctrl trace: {src_name}, {stamp}")
        fig.tight_layout()
        png_path = csv_path.with_suffix(".png")
        fig.savefig(png_path, dpi=110)
        print(f"plot       {png_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
