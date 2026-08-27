#!/usr/bin/env bash
# Run stage-B FOC timing benchmark on target (motor off).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/bench_common.sh"

BUILD="${BUILD:-$BENCH_ROOT/build/Release}"
GDB="${GDB:-arm-none-eabi-gdb}"
LOG="/tmp/motor_foc_bench.gdb"

require_dap

cmake -S "$BENCH_ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
  -DMOTOR_AUTO_START=OFF
cmake --build "$BUILD"

flash_elf

openocd -f "$CFG" >/tmp/motor_foc_bench_ocd.log 2>&1 &
OCD_PID=$!
cleanup() { kill "$OCD_PID" 2>/dev/null || true; }
trap cleanup EXIT
sleep 0.5

"$GDB" -batch \
  -ex "set remotetimeout 10" \
  -ex "file $BUILD/motor-ctrl.elf" \
  -ex "target extended-remote :3333" \
  -ex "monitor halt" \
  -ex "monitor rbp all" \
  -ex "call (void)MotorFocBench_Run()" \
  -ex "x/16wx &g_foc_bench_result" \
  >"$LOG" 2>&1

python3 - "$LOG" <<'PY'
import re
import sys

log = open(sys.argv[1], encoding="utf-8", errors="replace").read()
words = [int(x, 16) for x in re.findall(r"0x[0-9a-f]+", log)]
# last 16 words from the memory dump line(s)
if len(words) < 16:
    print("benchmark dump missing; GDB log tail:")
    print("\n".join(log.strip().splitlines()[-20:]))
    sys.exit(1)
vals = words[-16:]
tag, overhead, nop_med, nop_exp = vals[0], vals[1], vals[2], vals[3]
medians = vals[4:8]
maxes = vals[8:12]
mins = vals[12:16]

if tag != 0xA2B00001:
    print(f"bench incomplete: tag=0x{tag:08X}")
    sys.exit(1)

HZ = 48_000_000
labels = ["idle@360rpm", "run@360rpm", "run@1500rpm", "disabled"]

print("FOC timing bench (stage B)")
print(f"  overhead     : {overhead} cycles")
print(f"  NOP×100 cal  : {nop_med} cycles (expect ~{nop_exp})")
print()
for i, label in enumerate(labels):
    med, mx, mn = medians[i], maxes[i], mins[i]
    us = med / HZ * 1e6
    print(f"  case {i} {label:14s}: median={med:5d}  min={mn:5d}  max={mx:5d}  ({us:.1f} µs)")

worst = max(medians)
print()
if worst < 1800:
    verdict = "fits 20 kHz — proceed to stage C at 20 kHz"
elif worst < 9600:
    verdict = "fits 5 kHz — use 5 kHz control loop, proceed to stage C"
elif worst < 20000:
    verdict = "fits 2.4–5 kHz only — drop loop rate or consider fixed-point"
else:
    verdict = "does not fit even at 2.4 kHz — fixed-point required"

print(f"  worst median : {worst} cycles ({worst / HZ * 1e6:.1f} µs)")
print(f"  verdict      : {verdict}")
PY

echo "motor left halted after bench; re-flash safe firmware or run motor_off if needed."
