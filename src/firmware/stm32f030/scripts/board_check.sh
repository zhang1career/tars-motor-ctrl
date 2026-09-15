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

_board_ok=0
PROBE_LOG=""
_board_check_exit() {
  rm -f "$PROBE_LOG"
  if [[ "$_board_ok" != 1 ]]; then
    host_alarm
  fi
}
trap _board_check_exit EXIT

require_dap
cpu_alive

PROBE_LOG=$(mktemp)
if [[ "$RUNNING" == 1 ]]; then
  ocd -c 'init' -c 'set probe_skip_adc 1' \
      -c "source $ROOT/scripts/board_probe.tcl" -c 'exit' >"$PROBE_LOG"
  ADC_FW=$(read_words "$(sym_addr g_motor_adc_raw)" 2 | tr '\n' ' ')
else
  ocd -c 'init' -c "source $ROOT/scripts/board_probe.tcl" -c 'exit' >"$PROBE_LOG"
  ADC_FW=""
fi
BUS_I=$(read_i)

RUNNING="$RUNNING" BUS_I="$BUS_I" ADC_FW="$ADC_FW" python3 - "$PROBE_LOG" <<'PY'
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
skip_adc = False

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
    elif f[0] == "SKIPADC":
        skip_adc = True
    elif f[0] == "TIMEOUT":
        print(f"probe timeout: {line.strip()}", file=sys.stderr)

if not (tim1 and gpioa and gpiob is not None):
    print("probe output incomplete; is the target powered and the probe attached?",
          file=sys.stderr)
    sys.exit(1)
if not skip_adc and not (cal and len(adc) >= 8):
    print("probe ADC incomplete; is the target powered and the probe attached?",
          file=sys.stderr)
    sys.exit(1)

cr1, ccer, bdtr, sr = tim1
moder_a, idr_a = gpioa
nfault = (idr_a >> 6) & 1
notemp = (idr_a >> 12) & 1
cp_mode = (moder_a >> 22) & 3
cp_level = (idr_a >> 11) & 1
hall = [(gpiob >> b) & 1 for b in (3, 4, 5)]

if skip_adc:
    words = [int(x, 16) for x in os.environ.get("ADC_FW", "").split() if x]
    if len(words) < 2:
        print("g_motor_adc_raw unreadable; firmware ADC DMA is the running-mode source",
              file=sys.stderr)
        sys.exit(1)
    iu, iv = words[0] & 0xFFFF, (words[0] >> 16) & 0xFFFF
    iw, vbus_raw = words[1] & 0xFFFF, (words[1] >> 16) & 0xFFFF
    vdda = 3.31
    lsb_ma = vdda / 4096 / (50 * 0.010) * 1000
    vbus = vbus_raw / 4095 * vdda * 4.9
    vboost = float("nan")
    med = {1: iu, 2: iv, 3: iw, 4: vbus_raw}
    spread = {1: 0, 2: 0, 3: 0}
    print(f"VDDA            {vdda:.2f} V        (nominal; ADC1 left to firmware)")
    print(f"current LSB     {lsb_ma:.3f} mA/LSB")
    print(f"bus  (PA4)      {vbus:.2f} V        (g_motor_adc_raw)")
    print(f"V_BOOST (PA5)   n/a             (not in the control sequence)")
    print(f"bus current     {bus_i:.3f} A         (UT61E, in series with the supply)")
    print("board NTC       n/a")
    print("die temp        n/a")
else:
    ts_cal1, vref_cal, ts_cal2 = cal
    med = {ch: statistics.median(v) for ch, v in adc.items()}
    spread = {ch: max(v) - min(v) for ch, v in adc.items()}
    vdda = 3.3 * vref_cal / med[17]
    lsb_ma = vdda / 4096 / (50 * 0.010) * 1000
    volts = lambda ch: vdda * med[ch] / 4095
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
if not 10.5 <= vbus <= 16.5:
    # LM339s ride the motor bus. Below ~10.5 they are dead and both
    # fault lines float high, so "no fault" proves nothing. 16.5 is
    # under the UCC27211 17 V recommended VDD (abs max 20 V).
    fails.append(f"bus {vbus:.2f} V outside 10.5..16.5 -- with the bus absent "
                 f"the LM339 window comparators are unpowered, so "
                 f"nFAULT/nOTEMP reading high proves nothing")
if not skip_adc:
    if vboost > vbus + 1.5:
        fails.append(f"V_BOOST {vboost:.2f} V is {vboost - vbus:+.2f} above "
                     f"the bus — charge pump is running, and the half-bridge "
                     f"boards have no HB-HS clamp (spec 9.5.4)")
    if vboost > 17.0:
        fails.append(f"V_BOOST {vboost:.2f} V above UCC27211 17 V recommended")
    if not 10.0 <= vboost <= 17.0:
        fails.append(f"V_BOOST {vboost:.2f} V outside 10.0..17.0")
bkin_af = (moder_a >> 12) & 3
bke = (bdtr >> 12) & 1
bif = (sr >> 7) & 1
if bkin_af != 2:
    fails.append(f"PA6 mode={bkin_af} (want AF=2 TIM1_BKIN); a GPIO/floating "
                 f"BKIN with BKP=low is what locked MOE last time")
if bke == 0:
    fails.append("BKE=0: TIM1 BKIN is disarmed. Hardware nFAULT is ignored. "
                 "Reflash with MOTOR_PWM_BKIN=ON; do not run PWM without it")
if bif != 0:
    fails.append("BIF set: a break latched MOE off")
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
_board_ok=1
