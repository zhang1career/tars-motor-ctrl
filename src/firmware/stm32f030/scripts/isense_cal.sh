#!/usr/bin/env bash
# Stage C calibration: check the ADC current scale against the bus current.
#
# Holds one commutation step statically (open loop, step_ms = 0) at a series of
# duties and compares two independent measurements of the same winding current:
#
#   ADC  : the two conducting shunt channels, sampled at the PWM peak
#   bus  : I_winding = (I_bus - I_idle) / D_eff, from the UT61E in series
#
# Sweeping rather than taking one point is deliberate: a single agreeing number
# proves little (docs/measurement-validity.md 3.1 and 3.2), whereas a scale
# factor that holds across a 3x range of current does.
#
# D_eff = (2*CCR - DTG)/period, not the duty percentage: the 1.5 us dead time
# comes off the high-side pulse. At low duty that correction dominates, which is
# why the low-duty points are expected to be the noisy ones.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/bench_common.sh"

DUTIES="${DUTIES:-7 12 17 22}"
SETTLE_S="${SETTLE_S:-3}"
OUT="${OUT:-/tmp/isense_cal.txt}"

require_dap
trap 'motor_off' EXIT
: >"$OUT"

# Start from a known-off state rather than trusting the previous run's cleanup.
# The baseline check below refuses to proceed with live outputs, and it has
# already caught a static step left energised by a hand-run command.
motor_off

echo "== idle baseline =="
"$ROOT/scripts/board_check.sh" >/dev/null
IDLE_I=$(read_i)
echo "idle bus current ${IDLE_I} A"

for duty in $DUTIES; do
  echo
  echo "== static step, duty ${duty}% =="
  rm -rf "$BUILD"
  cmake -S "$BENCH_ROOT" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DMOTOR_AUTO_START=ON -DMOTOR_DUTY_PCT="$duty" -DMOTOR_STEP_MS=0 \
    -DMOTOR_PHASE_OFFSET=3 >/dev/null
  cmake --build "$BUILD" >/dev/null
  flash_elf
  cpu_alive
  sleep "$SETTLE_S"

  JSON=$(mktemp)
  python3 "$ROOT/scripts/trace_dump.py" --arm oneshot --source adc \
    --no-plot --json "$JSON" >/dev/null
  BUS_I=$(read_i)
  motor_off

  echo "$duty $BUS_I $JSON" >>"$OUT"
  python3 - "$JSON" "$duty" "$BUS_I" <<'PY'
import json
import sys

d = json.load(open(sys.argv[1]))
duty, bus_i = int(sys.argv[2]), float(sys.argv[3])
ma = d["ma_per_lsb"]
means = [m * ma for m in d["mean_lsb"][:3]]
sds = [s * ma for s in d["stdev_lsb"][:3]]
for name, m, s in zip("UVW", means, sds):
    print(f"  {name} {m:+8.1f} mA  (sd {s:4.1f})")
print(f"  vbus {d['mean_lsb'][3] * d['vbus_v_per_lsb']:.2f} V, "
      f"bus current {bus_i:.3f} A")
PY
done

echo
echo "== scale check =="
python3 - "$OUT" "$IDLE_I" <<'PY'
import json
import sys

idle = float(sys.argv[2])
print(f"idle {idle * 1000:.0f} mA subtracted from every bus reading\n")
print("duty  D_eff   ADC winding   bus-derived winding   ratio   sum residual")
rows = []
for line in open(sys.argv[1]):
    duty_s, bus_s, path = line.split()
    duty, bus_i = int(duty_s), float(bus_s)
    d = json.load(open(path))
    ma = d["ma_per_lsb"]
    means = [m * ma for m in d["mean_lsb"][:3]]

    # The conducting pair is whichever two channels are far from zero; the third
    # phase is open and must read zero.
    active = sorted(means, key=abs, reverse=True)[:2]
    i_adc = sum(abs(a) for a in active) / 2
    resid = sum(means)

    arr, dtg, period = 1199, 72, 2400
    d_eff = (2 * (arr * duty // 100) - dtg) / period
    i_bus = (bus_i - idle) / d_eff * 1000 if d_eff > 0 else float("nan")
    rows.append((duty, d_eff, i_adc, i_bus, i_adc / i_bus, resid))
    print(f" {duty:3d}  {d_eff * 100:5.2f}%  {i_adc:8.0f} mA   "
          f"{i_bus:12.0f} mA   {i_adc / i_bus:6.3f}  {resid:+8.1f} mA")

print()
hi = [r for r in rows if r[1] > 0.08]
if hi:
    ratios = [r[4] for r in hi]
    spread = max(ratios) - min(ratios)
    mean = sum(ratios) / len(ratios)
    print(f"over the points with D_eff > 8% the ratio is {mean:.3f} "
          f"+-{spread / 2:.3f}")
    print("Roadmap stage C wants agreement within 10%; the bus-derived figure "
          "carries the dead-time model and the gate-drive current, so treat the "
          "ADC value as the accurate one and this as a sanity bound.")
PY

echo
echo "== reflashing safe firmware =="
rm -rf "$BUILD"
cmake -S "$BENCH_ROOT" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DMOTOR_AUTO_START=OFF >/dev/null
cmake --build "$BUILD" >/dev/null
flash_elf
"$ROOT/scripts/board_check.sh" | tail -2
