#!/usr/bin/env python3
"""One-shot tick bench: Fx_QuotU check + two 32-tick MotorTick timings."""
from __future__ import annotations

import re
import struct
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ocd_rpc import rpc  # noqa: E402

SCRIPTS = Path(__file__).resolve().parent
ELF_DIR = SCRIPTS.parent / "build" / "Release"
AS = "/Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin/arm-none-eabi-as"
OBJCOPY = (
    "/Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin/arm-none-eabi-objcopy"
)
NM = "/Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin/arm-none-eabi-nm"

# Filled after nm
SYM: dict[str, int] = {}


def ocd_one(cmd: str, timeout: float = 30.0) -> str:
    return rpc([cmd], timeout=timeout)


def mdw(addr: int, n: int = 1) -> list[int]:
    out = ocd_one(f"mdw 0x{addr:08x} {n}")
    vals: list[int] = []
    for line in out.splitlines():
        if ":" not in line:
            continue
        rest = line.split(":", 1)[1]
        for tok in rest.split():
            if re.fullmatch(r"[0-9a-fA-F]+", tok):
                vals.append(int(tok, 16))
                if len(vals) >= n:
                    return vals
    return vals


def mww(addr: int, val: int) -> None:
    ocd_one(f"mww 0x{addr:08x} 0x{val:08x}")


def mwb(addr: int, val: int) -> None:
    ocd_one(f"mwb 0x{addr:08x} 0x{val & 0xFF:08x}")


def mwh(addr: int, val: int) -> None:
    ocd_one(f"mwh 0x{addr:08x} 0x{val & 0xFFFF:08x}")


def load_nm() -> None:
    out = subprocess.check_output([NM, "-n", str(ELF_DIR / "motor-ctrl.elf")], text=True)
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3:
            SYM[parts[2]] = int(parts[0], 16)


def sym(name: str) -> int:
    if name not in SYM:
        raise SystemExit(f"missing symbol {name}")
    return SYM[name]


def bdtr_val() -> int:
    return mdw(0x40012C44)[0]


def safety_halt() -> tuple[int, int]:
    ocd_one("halt")
    mww(0x40015808, 0x1000)
    before = bdtr_val()
    if before & (1 << 15):
        ocd_one("reset halt")
        ocd_one("rbp all")
        ocd_one("resume")
        ocd_one("halt")
        mww(0x40015808, 0x1000)
        before = bdtr_val()
        if before & (1 << 15):
            raise SystemExit(f"BDTR MOE still set: 0x{before:08x}")
    return before, before


def safety_end() -> None:
    ocd_one("reset halt")
    ocd_one("rbp all")
    ocd_one("resume")


def dma1_ch1_disable() -> None:
    ccr = mdw(0x40020008)[0]
    mww(0x40020008, ccr & ~1)


def trace_source_zero() -> None:
    # g_motor_trace.source at byte offset 20
    addr = sym("g_motor_trace") + 20
    mwb(addr, 0)


def tick_pad_zero() -> None:
    mwh(sym("g_tick_pad"), 0)


def write_tick_stub() -> Path:
    src = SCRIPTS / "_bench_tick_gen.s"
    tick = sym("MotorTick_OnAdcComplete") | 1
    res = sym("g_motor_trace_buf") + 0x100
    src.write_text(
        f"""\
.syntax unified
.cpu cortex-m0
.thumb
.text
.global stub
stub:
    cpsid i
    ldr r4, =0x{tick:08x}
    ldr r5, =0xE000E018
    ldr r6, =0x{res:08x}
    movs r7, #32
1:
    ldr r0, [r5]
    blx r4
    ldr r1, [r5]
    subs r2, r0, r1
    bcs 2f
    ldr r3, =0xBB80
    adds r2, r3
2:
    str r2, [r6]
    adds r6, #4
    subs r7, #1
    bne 1b
    ldr r0, =0x40012C20
    ldrh r1, [r0]
    ldr r2, =0x555
    bics r1, r2
    strh r1, [r0]
    bkpt #0
"""
    )
    return src


def write_quot_stub() -> Path:
    src = SCRIPTS / "_bench_quot_gen.s"
    fn = sym("Fx_QuotU") | 1
    res = sym("g_motor_trace_buf") + 0x100
    src.write_text(
        f"""\
.syntax unified
.cpu cortex-m0
.thumb
.text
.global stub
stub:
    cpsid i
    ldr r4, =0x{fn:08x}
    ldr r6, =0x{res:08x}
    ldr r0, =2796288
    ldr r1, =100
    blx r4
    str r0, [r6]
    adds r6, #4
    ldr r0, =3149312
    ldr r1, =100
    blx r4
    str r0, [r6]
    adds r6, #4
    bkpt #0
"""
    )
    return src


def assemble_stub(src: Path) -> Path:
    obj = SCRIPTS / "_bench_stub.o"
    binf = SCRIPTS / "_bench_stub.bin"
    subprocess.check_call([AS, "-mcpu=cortex-m0", "-o", str(obj), str(src)])
    subprocess.check_call(
        [OBJCOPY, "-O", "binary", "-j", ".text", str(obj), str(binf)]
    )
    data = binf.read_bytes()
    if len(data) >= 0x100:
        raise SystemExit(f"stub too large: {len(data)}")
    return binf


def load_stub(binf: Path) -> int:
    base = sym("g_motor_trace_buf")
    ocd_one(f"load_image {binf} 0x{base:08x}")
    return base | 1


def run_stub(entry: int) -> None:
    ocd_one("halt")
    mww(0x40015808, 0x1000)
    ocd_one(f"reg pc 0x{entry:08x}")
    ocd_one("resume")
    try:
        ocd_one("wait_halt 8000")
    except Exception:
        ocd_one("halt")


def read_cycles(n: int = 32) -> list[int]:
    base = sym("g_motor_trace_buf") + 0x100
    words = mdw(base, n)
    return words


def read_quot_results() -> tuple[int, int]:
    base = sym("g_motor_trace_buf") + 0x100
    a, b = mdw(base, 2)
    return a, b


def setup_common() -> None:
    dma1_ch1_disable()
    trace_source_zero()
    tick_pad_zero()


def setup_current_leg() -> None:
    fx = sym("g_motor_foc_fx")
    mwb(fx + 32, 2)
    mwb(sym("g_motor_foc_vq_neg"), 1)
    mwb(sym("g_motor_foc_id_on"), 1)
    mwb(sym("g_motor_foc_interp"), 1)
    mwb(sym("g_motor_foc_leg3"), 1)
    mwb(sym("g_motor_foc_overmod"), 0)
    mwb(sym("g_motor_foc_spd_on"), 0)
    mwb(sym("g_motor_pwm_dt_on"), 1)
    mwb(sym("g_motor_pwm_prot") + 14, 0)
    mww(sym("g_motor_foc_vq_max_uv"), 2400000)
    mww(sym("g_motor_foc_vd_max_uv"), 2400000)
    mww(sym("g_motor_foc_park_slew_q16"), 600)
    mww(sym("g_motor_foc_park_off_q16"), 1274)
    mww(sym("g_motor_foc_iq_ref"), 0)
    mww(sym("g_motor_foc_id_ref"), 0)
    sfx = sym("s_fx")
    mww(sfx + 0, 0)
    mww(sfx + 4, 1200000)
    mwb(sfx + 28, 1)
    mwb(sfx + 29, 1)
    mwb(sfx + 30, 1)
    adc = sym("g_motor_adc_raw")
    mwh(adc + 0, 2048)
    mwh(adc + 2, 2048)
    mwh(adc + 4, 2048)
    check = mdw(adc, 1)[0]
    if check != 0x08000800:
        raise SystemExit(
            f"g_motor_adc_raw readback expected 0x08000800 got 0x{check:08x}"
        )
    sft = sym("s_ft")
    mwh(sft + 0, 0)
    mwh(sft + 2, 100)
    mwb(sft + 10, 1)
    mwb(sft + 12, 1)
    for i in range(6):
        mwh(sft + 20 + 2 * i, 100)
    mwb(sym("s_ftheta") + 2, 0)


def hall_table() -> list[int]:
    table = sym("s_hall_to_sector")
    out = ocd_one(f"mdb 0x{table:08x} 8")
    vals: list[int] = []
    for tok in out.replace(":", " ").split():
        if re.fullmatch(r"[0-9a-fA-F]{2}", tok):
            vals.append(int(tok, 16))
    if len(vals) < 8:
        raise SystemExit(f"s_hall_to_sector read failed: {out!r}")
    return vals


def hall_to_sector(hall: int) -> int:
    return hall_table()[hall & 7]


def setup_angle_edge() -> int:
    fx = sym("g_motor_foc_fx")
    mwb(fx + 32, 0)
    mwb(sym("s_hall6_enable"), 0)
    mwb(sym("s_ol_enable"), 0)
    idr = mdw(0x48000408)[0]
    hall = (idr >> 3) & 7
    s = hall_to_sector(hall)
    if s == 0xFF:
        raise SystemExit(f"invalid hall code {hall}")
    prev = 5 if s == 0 else s - 1
    ang = sym("g_motor_angle")
    mwb(ang + 10, prev)  # sector
    mwh(ang + 8, 100)  # ticks_in_sector
    mwh(sym("s_sector_ticks") + 2 * s, 100)
    mwh(sym("s_sector_ticks") + 2 * prev, 100)
    mwb(sym("s_dir_hint"), 0)
    return s


def stats(cyc: list[int]) -> tuple[int, int, int]:
    return cyc[0], min(cyc), max(cyc)


def main() -> int:
    load_nm()
    try:
        rpc(["echo bench connect"], timeout=5.0)
    except Exception as e:
        print(f"OpenOCD connect failed: {e}", file=sys.stderr)
        return 2

    bdtr_before, _ = safety_halt()
    setup_common()

    quot_bin = assemble_stub(write_quot_stub())
    entry = load_stub(quot_bin)
    run_stub(entry)
    q1, q2 = read_quot_results()
    exp1, exp2 = 27962, 31493
    tol = 0.004
    quot_note = ""
    for got, exp in ((q1, exp1), (q2, exp2)):
        if abs(got - exp) / exp > tol:
            quot_note = f" (Fx_QuotU 超出 ±0.4%: {q1} vs {exp1}, {q2} vs {exp2})"
            break

    setup_current_leg()
    tick_bin = assemble_stub(write_tick_stub())
    entry = load_stub(tick_bin)
    run_stub(entry)
    cur = read_cycles(32)
    c_first, c_min, c_max = stats(cur)

    setup_angle_edge()
    entry = load_stub(tick_bin)
    run_stub(entry)
    ang = read_cycles(32)
    a_first, a_min, a_max = stats(ang)

    bdtr_after = bdtr_val()
    safety_end()

    flash = 30488
    print("=== BENCH REPORT ===")
    print(f"Fx_QuotU(2796288,100) = {q1}")
    print(f"Fx_QuotU(3149312,100) = {q2}{quot_note}")
    print(f"电流环换边 32拍: min={c_min} max={c_max} 第一拍={c_first}")
    print(f"角度换边 32拍: min={a_min} max={a_max} 第一拍={a_first}")
    print(f"FLASH 字节数: {flash}")
    print(f"BDTR 前=0x{bdtr_before:08x} 后=0x{bdtr_after:08x}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
