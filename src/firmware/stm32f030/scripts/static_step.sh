#!/usr/bin/env bash
# Hold one fixed commutation step (U off, V pwm, W low) and read the three
# INA240 channels alongside the bus current.
#
# Purpose is to prove out the analog chain before anything spins:
#   - which shunt channel belongs to which half bridge (the OFF phase must stay
#     at the 2048 zero code while the other two move),
#   - the sign each INA240 gives for motor current,
#   - that nFAULT stays high while the bridges chop, which has to hold before
#     TIM1 BKIN can be enabled.
#
# The ADC samples are NOT synchronised to the PWM here, so readings scatter
# across the switching period on purpose: the min/median/max spread is the
# signature we want. Calibrated current needs TIM1_TRGO sampling (stage C).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/bench_common.sh"

DUTY="${DUTY:-7}"          # MotorApp_DiagGPhase clamps this to 7
HOLD_S="${HOLD_S:-2}"
PASSES="${PASSES:-16}"

require_dap

echo "== baseline (outputs off) =="
"$ROOT/scripts/board_check.sh"

echo
echo "== static step, duty ${DUTY}% =="
rm -rf "$BUILD"
cmake -S "$BENCH_ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
  -DMOTOR_AUTO_START=ON -DMOTOR_START_DIAG=ON -DMOTOR_DUTY_PCT="$DUTY" >/dev/null
cmake --build "$BUILD" >/dev/null
flash_elf
cpu_alive

sleep "$HOLD_S"
STEP_LOG=$(mktemp)
ocd -c 'init' -c "set probe_passes $PASSES" \
    -c "source $ROOT/scripts/board_probe.tcl" -c 'exit' >"$STEP_LOG"
STEP_I=$(read_i)

motor_off

BUS_I="$STEP_I" DUTY="$DUTY" python3 - "$STEP_LOG" <<'PY'
import math
import os
import statistics
import sys

adc: dict[int, list[int]] = {}
gpioa = tim1 = cal = None
for line in open(sys.argv[1], encoding="utf-8", errors="replace"):
    f = line.split()
    if not f:
        continue
    if f[0] == "ADC":
        adc.setdefault(int(f[1]), []).append(int(f[2]))
    elif f[0] == "GPIOA":
        gpioa = [int(x) for x in f[1:3]]
    elif f[0] == "TIM1":
        tim1 = [int(x) for x in f[1:5]]
    elif f[0] == "CAL":
        cal = [int(x) for x in f[1:4]]

if not (adc and gpioa and tim1 and cal):
    print("probe output incomplete", file=sys.stderr)
    sys.exit(1)

vdda = 3.3 * cal[1] / statistics.median(adc[17])
lsb_ma = vdda / 4096 / (50 * 0.010) * 1000
bus_i = float(os.environ["BUS_I"])
duty = int(os.environ["DUTY"])

# Effective high-side conduction: centre-aligned period is 2*ARR counts and the
# 1.5 us dead time comes off the pulse, so it is not the duty number.
arr, dtg, period = 1199, 72, 2400
d_eff = (2 * (arr * duty // 100) - dtg) / period
i_wind_pred = d_eff * 12.0 / 1.3
i_bus_pred = d_eff * d_eff * 12.0 / 1.3

print(f"TIM1 MOE={(tim1[2] >> 15) & 1}  BIF={(tim1[3] >> 7) & 1}  "
      f"CCER=0x{tim1[1]:04x}")
print(f"nFAULT={(gpioa[1] >> 6) & 1}  nOTEMP={(gpioa[1] >> 12) & 1}   "
      f"(1 = no fault; nFAULT must stay 1 while chopping)")
print()
print(f"duty {duty}%  ->  effective conduction {d_eff*100:.1f}%  "
      f"(dead time {dtg/48:.2f} us removed)")
print(f"predicted winding {i_wind_pred:.2f} A, bus {i_bus_pred*1000:.0f} mA "
      f"(12 V / 1.3 ohm phase-to-phase)")
print(f"measured  bus {bus_i:.3f} A")
print()
print("phase  role      min    med    max   |  median current   peak-to-peak")
for ch, name, role in ((1, "U", "off"), (2, "V", "pwm"), (3, "W", "low")):
    v = adc[ch]
    lo, hi, med = min(v), max(v), statistics.median(v)
    print(f"  {name}    {role:4s}   {lo:5d}  {med:6.0f}  {hi:5d}   |  "
          f"{(med-2048)*lsb_ma:+8.0f} mA      {(hi-lo)*lsb_ma:6.0f} mA")
print()
print(f"VDDA {vdda:.4f} V, {lsb_ma:.3f} mA/LSB, zero code 2048, "
      f"{len(adc[1])} samples/channel")
PY

echo
echo "== reflashing safe firmware =="
rm -rf "$BUILD"
cmake -S "$BENCH_ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
  -DMOTOR_AUTO_START=OFF >/dev/null
cmake --build "$BUILD" >/dev/null
flash_elf
"$ROOT/scripts/board_check.sh"
