#!/usr/bin/env bash
# hall6 closed-loop regression, both directions, against docs/roadmap.md 1.2.
#
# hall6 is the only state on this hardware known to work, so it is the fallback
# every FOC experiment retreats to. Re-establish it whenever the board or the
# wiring changes.
#
# Reads the snapshot without halting: halting would freeze the control ISR while
# TIM1 keeps switching, leaving one commutation step energised as the rotor
# coasts past it.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/bench_common.sh"

DUTY="${DUTY:-20}"
WINDOW_S="${WINDOW_S:-6}"
SETTLE_S="${SETTLE_S:-4}"
DIRS="${DIRS:-0 1}"

require_dap
echo "== baseline =="
"$ROOT/scripts/board_check.sh" | rg 'bus |bus current|nFAULT'

# Two snapshots separated by openocd's own sleep, in one session. Timing the
# pair from bash instead put a benchgate read and an openocd startup inside the
# interval, which inflated the measured loop rate by ~28%.
# Fields: hall_raw|step|duty|enabled, direction|fault|kick|phase, loop_count,
# hall_changes.
snap_pair() {
  ocd -c 'init' -c "mdw $1 4" -c "sleep $((WINDOW_S * 1000))" -c "mdw $1 4" -c 'exit' \
    | rg '^0x[0-9a-f]+:' \
    | sed 's/^[^:]*: *//' \
    | tr ' ' '\n' \
    | rg '^[0-9a-f]{8}$' \
    | tr '\n' ' '
}

for dir in $DIRS; do
  echo
  echo "== hall6 duty=${DUTY}% direction=${dir} =="
  rm -rf "$BUILD"
  cmake -S "$BENCH_ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
    -DMOTOR_AUTO_START=ON -DMOTOR_START_HALL6=ON \
    -DMOTOR_HALL6_RUN_DUTY="$DUTY" -DMOTOR_HALL6_KICK_DUTY="$DUTY" \
    -DMOTOR_HALL6_PHASE="${PHASE:-3}" -DMOTOR_HALL6_CCW="$dir" >/dev/null
  cmake --build "$BUILD" >/dev/null
  flash_elf

  SNAP_ADDR=$(sym_addr s_hall6_snap)
  sleep "$SETTLE_S"

  I1=$(read_i)
  SNAPS=$(snap_pair "$SNAP_ADDR")
  I2=$(read_i)

  RUN_LOG=$(mktemp)
  ocd -c 'init' -c "source $ROOT/scripts/board_probe.tcl" -c 'exit' >"$RUN_LOG"

  motor_off

  SNAPS="$SNAPS" I1="$I1" I2="$I2" WINDOW_S="$WINDOW_S" DUTY="$DUTY" DIR="$dir" \
    python3 - "$RUN_LOG" <<'PY'
import os
import statistics
import sys

words = [int(x, 16) for x in os.environ["SNAPS"].split()]
if len(words) != 8:
    print(f"expected 8 snapshot words, got {len(words)}", file=sys.stderr)
    sys.exit(1)
w1, w2 = words[:4], words[4:]
window = float(os.environ["WINDOW_S"])
duty_cmd = int(os.environ["DUTY"])

def fields(w):
    return {
        "hall_raw": w[0] & 0xFF, "step": (w[0] >> 8) & 0xFF,
        "duty_pct": (w[0] >> 16) & 0xFF, "enabled": (w[0] >> 24) & 0xFF,
        "direction": w[1] & 0xFF, "fault": (w[1] >> 8) & 0xFF,
        "kick": (w[1] >> 16) & 0xFF, "phase": (w[1] >> 24) & 0xFF,
        "loop_count": w[2], "hall_changes": w[3],
    }

a, b = fields(w1), fields(w2)
loop_hz = (b["loop_count"] - a["loop_count"]) / window
elec_hz = (b["hall_changes"] - a["hall_changes"]) / 6 / window
i1, i2 = float(os.environ["I1"]), float(os.environ["I2"])

adc: dict[int, list[int]] = {}
gpioa = cal = tim1 = None
for line in open(sys.argv[1], encoding="utf-8", errors="replace"):
    f = line.split()
    if not f:
        continue
    if f[0] == "ADC":
        adc.setdefault(int(f[1]), []).append(int(f[2]))
    elif f[0] == "GPIOA":
        gpioa = [int(x) for x in f[1:3]]
    elif f[0] == "TIM1":
        tim1 = [int(x) for x in f[1:7]]
    elif f[0] == "CAL":
        cal = [int(x) for x in f[1:4]]

arr, rcr = tim1[4], tim1[5]
isr_hz = 48e6 / (2 * (arr + 1)) * 2 / (rcr + 1)

vdda = 3.3 * cal[1] / statistics.median(adc[17])
vbus = vdda * statistics.median(adc[4]) / 4095 * 4.9

print(f"  readback     duty={b['duty_pct']}% phase={b['phase']} "
      f"direction={b['direction']} enabled={b['enabled']}")
print(f"  fault/kick   {b['fault']} / {b['kick']}")
print(f"  control loop {loop_hz:.0f} Hz counted, {isr_hz:.0f} Hz from "
      f"ARR={arr} RCR={rcr}")
print(f"  speed        {elec_hz:.1f} electrical rev/s "
      f"({elec_hz / 4 * 60:.0f} rpm at 4 pole pairs)")
print(f"  bus current  {i1:.3f} -> {i2:.3f} A")
print(f"  bus voltage  {vbus:.2f} V   (loaded; sag means the supply hit CC)")
print(f"  nFAULT/nOTEMP {(gpioa[1] >> 6) & 1} / {(gpioa[1] >> 12) & 1}")

checks = [
    ("duty applied", b["duty_pct"] == duty_cmd,
     f"chip says {b['duty_pct']}%, asked {duty_cmd}% "
     f"(cmake cache, or the <=25 clamp)"),
    ("timer gives 20 kHz", abs(isr_hz - 20000) < 100,
     f"ARR={arr} RCR={rcr} -> {isr_hz:.0f} Hz; RCR=0 would give 40 kHz"),
    ("ISR keeps up", abs(loop_hz - isr_hz) / isr_hz < 0.02,
     f"counted {loop_hz:.0f} Hz against {isr_hz:.0f} Hz from the timer: "
     f"ticks are being missed, or the window timing is off"),
    ("hall valid", b["fault"] == 0, "fault=1: hall code read 0 or 7"),
    ("kick finished", b["kick"] == 0, "still kicking: hall sync never happened"),
    # A stalled rotor and a dead output look the same in the speed number but
    # not in the current: holding one commutation step draws roughly
    # D_eff * 12/1.3 on the bus, about 0.25 A at 20% duty, while no drive draws
    # the 4 mA idle. The open-loop kick fails to catch the rotor perhaps one
    # start in five depending on where it happens to be parked, and then it
    # times out after 40 steps and leaves exactly that static hold.
    ("spinning", elec_hz > 1.0,
     f"{elec_hz:.1f} elec rev/s with {i2:.3f} A on the bus -- "
     + ("rotor is stalled under a static commutation hold; the open-loop kick "
        "timed out without ever syncing, which is intermittent on this motor. "
        "Re-run before suspecting a code change"
        if i2 > 0.15 else
        "no drive current either, so the outputs are not switching at all")),
    ("bus not in CC", vbus >= 11.5,
     f"{vbus:.2f} V loaded vs ~11.98 V idle: supply is current limiting, so "
     f"every comparison at this operating point is void (roadmap 1.6)"),
    ("nFAULT high", ((gpioa[1] >> 6) & 1) == 1,
     "overcurrent comparator tripped while running"),
]
print()
for name, ok, why in checks:
    print(f"  [{'ok  ' if ok else 'FAIL'}] {name}" + ("" if ok else f" -- {why}"))
PY
done

echo
echo "== reflashing safe firmware =="
rm -rf "$BUILD"
cmake -S "$BENCH_ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
  -DMOTOR_AUTO_START=OFF >/dev/null
cmake --build "$BUILD" >/dev/null
flash_elf
echo "idle bus current: $(read_i) A"
