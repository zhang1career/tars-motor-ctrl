#!/usr/bin/env bash
# Pre-flight check for the motor-ctrl mainboard v0.1.
#
# Replaces the old "idle bus current should be ~6 mA" heuristic, which predates
# this board: 12 V now also feeds the LM339 comparators, the 12 V LED, the
# charge-pump pull-up and two dividers, and the MCU is on the probe's 3.3 V.
# Measured idle is ~4 mA (2026-08-27).
#
# Reads without halting, so it also works with the motor running: pass
# --running to relax the gates that only hold at standstill.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/bench_common.sh"

RUNNING=0
[[ "${1:-}" == "--running" ]] && RUNNING=1

require_dap
cpu_alive

PROBE_LOG=$(mktemp)
trap 'rm -f "$PROBE_LOG"' EXIT
ocd -c 'init' -c "source $ROOT/scripts/board_probe.tcl" -c 'exit' >"$PROBE_LOG"
BUS_I=$(read_i)

RUNNING="$RUNNING" BUS_I="$BUS_I" python3 - "$PROBE_LOG" <<'PY'
import os
import statistics
import sys

running = os.environ["RUNNING"] == "1"
try:
    bus_i = float(os.environ["BUS_I"])
except ValueError:
    bus_i = float("nan")

tim1 = gpioa = gpiob = None
adc: dict[int, list[int]] = {}
cal = None

for line in open(sys.argv[1], encoding="utf-8", errors="replace"):
    f = line.split()
    if not f:
        continue
    if f[0] == "TIM1":
        tim1 = [int(x) for x in f[1:5]]
    elif f[0] == "GPIOA":
        gpioa = [int(x) for x in f[1:3]]
    elif f[0] == "GPIOB":
        gpiob = int(f[1])
    elif f[0] == "ADC":
        adc.setdefault(int(f[1]), []).append(int(f[2]))
    elif f[0] == "CAL":
        cal = [int(x) for x in f[1:4]]
    elif f[0] == "TIMEOUT":
        print(f"probe timeout: {line.strip()}", file=sys.stderr)

if not (tim1 and gpioa and gpiob is not None and cal and len(adc) >= 8):
    print("probe output incomplete; is the target powered and the probe attached?",
          file=sys.stderr)
    sys.exit(1)

ts_cal1, vref_cal, ts_cal2 = cal
med = {ch: statistics.median(v) for ch, v in adc.items()}
spread = {ch: max(v) - min(v) for ch, v in adc.items()}

vdda = 3.3 * vref_cal / med[17]
lsb_ma = vdda / 4096 / (50 * 0.010) * 1000     # INA240A2 gain 50, 10 mOhm shunt
volts = lambda ch: vdda * med[ch] / 4095

cr1, ccer, bdtr, sr = tim1
moder_a, idr_a = gpioa
nfault = (idr_a >> 6) & 1
notemp = (idr_a >> 12) & 1
cp_mode = (moder_a >> 22) & 3
cp_level = (idr_a >> 11) & 1
hall = [(gpiob >> b) & 1 for b in (3, 4, 5)]

vbus = volts(4) * 4.9
vboost = volts(5) * 7.8
r_ntc = 10000 * (vdda / volts(0) - 1)
ts = med[16] * vdda / 3.3
die_c = 30 + (ts - ts_cal1) * 80 / (ts_cal2 - ts_cal1)

import math
ntc_c = 1 / (1 / 298.15 + math.log(r_ntc / 10000) / 3435) - 273.15

print(f"VDDA            {vdda:.4f} V        (VREFINT {med[17]:.0f}, cal {vref_cal})")
print(f"current LSB     {lsb_ma:.3f} mA/LSB")
print(f"bus  (PA4)      {vbus:.2f} V")
print(f"V_BOOST (PA5)   {vboost:.2f} V")
print(f"bus current     {bus_i:.3f} A         (UT61E, in series with the supply)")
print(f"board NTC       {ntc_c:.1f} C          ({r_ntc:.0f} ohm)")
print(f"die temp        {die_c:.1f} C")
print(f"TIM1            CR1=0x{cr1:04x} CCER=0x{ccer:04x} BDTR=0x{bdtr:04x} "
      f"MOE={(bdtr >> 15) & 1} BKE={(bdtr >> 12) & 1} BIF={(sr >> 7) & 1}")
comparators_live = vbus >= 10.5
print(f"nFAULT / nOTEMP {nfault} / {notemp}            "
      f"(1 = no fault{'' if comparators_live else ', BUT SEE BELOW'})")
print(f"CP_DRIVE (PA11) mode={cp_mode} level={cp_level}   (want mode=1 output, level=0)")
print(f"HALL A/B/C      {hall[0]} {hall[1]} {hall[2]}")
for ch, name in ((1, "U"), (2, "V"), (3, "W")):
    off = med[ch] - 2048
    print(f"ISENSE_{name}        {med[ch]:6.0f} counts   offset {off:+.0f} LSB "
          f"= {off * lsb_ma:+.1f} mA   spread {spread[ch]} LSB")

fails = []
if not 3.15 <= vdda <= 3.45:
    fails.append(f"VDDA {vdda:.3f} V outside 3.15..3.45")
if not 10.5 <= vbus <= 13.0:
    # The LM339 comparators run off +12 V while their open-drain outputs pull up
    # to the probe-fed +3V3. With 12 V absent the comparators are dead and both
    # fault lines float high, so they read "no fault" while protecting nothing.
    fails.append(f"bus {vbus:.2f} V outside 10.5..13.0 -- with 12 V absent the "
                 f"LM339 window comparators are unpowered, so nFAULT/nOTEMP "
                 f"reading high proves nothing")
if vboost > 14.0:
    fails.append(f"V_BOOST {vboost:.2f} V - charge pump is running, and the "
                 f"half-bridge boards have no HB-HS clamp (spec 9.5.4)")
if not 10.0 <= vboost <= 14.0:
    fails.append(f"V_BOOST {vboost:.2f} V outside 10.0..14.0")
if nfault == 0:
    fails.append("nFAULT low: overcurrent comparator tripped, or JP2/threshold "
                 "wrong. Enabling BKIN in this state latches MOE off")
if notemp == 0:
    fails.append("nOTEMP low: a remote NTC reads over its trip point")
if cp_mode != 1 or cp_level != 0:
    fails.append(f"CP_DRIVE mode={cp_mode} level={cp_level}: PA11 must be a "
                 f"push-pull output driven low (R25 biases the pump on if it floats)")
if hall in ([0, 0, 0], [1, 1, 1]):
    fails.append(f"HALL code {hall} is invalid (0 or 7): sensor or wiring fault")

if not running:
    if (bdtr >> 15) & 1:
        fails.append("MOE set while idle: outputs are live")
    if ccer:
        fails.append(f"CCER 0x{ccer:04x} nonzero while idle")
    for ch, name in ((1, "U"), (2, "V"), (3, "W")):
        off = med[ch] - 2048
        if abs(off) > 30:
            fails.append(f"ISENSE_{name} zero offset {off:+.0f} LSB "
                         f"({off * lsb_ma:+.0f} mA) at standstill")

print()
if fails:
    print("FAIL")
    for f in fails:
        print(f"  - {f}")
    sys.exit(1)
print("PASS" + (" (running mode: standstill gates skipped)" if running else ""))
PY
