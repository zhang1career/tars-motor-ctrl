#!/usr/bin/env bash
# Hall6-only: measure dwell ticks in each raw hall sector.
# No FOC handover. Decimation-1 hall6 trace is supplementary (12.8 ms).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/bench_common.sh"

SETTLE_S="${SETTLE_S:-4}"
DIRS="${DIRS:-1}"
RETRIES="${RETRIES:-3}"
OUT="${OUT:-$BENCH_ROOT/../../../models/captured}"

require_dap
report_and_clear_stale_bif
echo "== baseline =="
"$ROOT/scripts/board_check.sh"

reflash_safe() {
  motor_off
  rm -rf "$BUILD"
  cmake -S "$BENCH_ROOT" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DMOTOR_AUTO_START=OFF -DMOTOR_START_BENCH=OFF \
    -DMOTOR_START_FOC_OBSERVE=OFF -DMOTOR_START_HALL6=OFF >/dev/null
  cmake --build "$BUILD" >/dev/null
  flash_elf
}

trap 'motor_off; host_alarm; reflash_safe' EXIT

hall6_gpio_walk() {
  ocd -c 'init' -c '
    for {set i 0} {$i < 16} {incr i} {
      set v [mrw 0x48000410]
      echo [format "HALL %d" [expr {($v >> 3) & 7}]]
      sleep 2
    }
  ' -c 'exit' | rg '^HALL ' | awk '{print $2}'
}

spinning() {
  local snap a b elec i halls
  snap=$(sym_addr s_hall6_snap)
  a=$(read_words "$snap" 4 | awk 'NR==4')
  sleep 1
  b=$(read_words "$snap" 4 | awk 'NR==4')
  elec=$(python3 -c "print((int('$b',16)-int('$a',16))/6)")
  i=$(read_i)
  halls=$(hall6_gpio_walk)
  python3 -c "
elec=float('$elec'); i=float('$i')
hs=[int(x) for x in '''$halls'''.split()]
codes=sorted(set(hs))
print(f'  hall6 edges {elec:.1f} elec/s  bus {i:.3f} A  gpio {hs} unique {codes}')
raise SystemExit(0 if elec > 1.0 and i > 0.02 and len(codes) >= 5 else 1)
"
}

for dir in $DIRS; do
  echo
  echo "== hall6 sectors direction=${dir} =="
  ok=0
  for try in $(seq 1 "$RETRIES"); do
    rm -rf "$BUILD"
    cmake -S "$BENCH_ROOT" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DMOTOR_AUTO_START=ON -DMOTOR_START_FOC_OBSERVE=ON \
      -DMOTOR_START_HALL6=OFF -DMOTOR_START_BENCH=OFF \
      -DMOTOR_START_DIAG=OFF \
      -DMOTOR_HALL6_RUN_DUTY=20 -DMOTOR_HALL6_KICK_DUTY=20 \
      -DMOTOR_HALL6_PHASE="${PHASE:-3}" -DMOTOR_HALL6_CCW="$dir" >/dev/null
    cmake --build "$BUILD" >/dev/null
    flash_elf
    cpu_alive
    sleep "$SETTLE_S"
    if spinning; then
      ok=1
      break
    fi
    echo "  kick missed (try $try/$RETRIES), retrying"
    motor_off
  done
  if [[ "$ok" != 1 ]]; then
    echo "FAIL: rotor did not start after $RETRIES tries" >&2
    motor_off
    exit 1
  fi

  echo "  hall6 spinning — watch the rotor (no FOC handover)"
  host_beep
  sleep 2

  DW=$(sym_addr g_hall6_sector_dwell)
  DW_WORDS=$(read_words "$DW" 4 | tr '\n' ' ')
  echo "  dwell words $DW_WORDS"
  python3 -c "
ws=[int(x,16) for x in '$DW_WORDS'.split()]
d=[]
for i in range(8):
    d.append((ws[i//2] >> (16*(i%2))) & 0xFFFF)
print('  dwell ticks by raw hall:', {h:d[h] for h in range(1,7)})
vals=[d[h] for h in range(1,7) if d[h]>0]
if len(vals)<6:
    raise SystemExit(f'missing hall dwells: {d}')
mean=sum(vals)/len(vals)
print(f'  mean {mean:.1f} ticks  ratio to mean', {h:round(d[h]/mean,3) for h in range(1,7)})
print(f'  max/min {max(vals)/min(vals):.3f}')
"

  TR=$(sym_addr g_motor_trace)
  ocd -c 'init' \
      -c "mwb $((TR + 20)) 0" \
      -c "mwh $((TR + 8)) 0" \
      -c "mwh $((TR + 10)) 20" \
      -c "mwh $((TR + 12)) 0" \
      -c "mww $((TR + 16)) 0" \
      -c "mwb $((TR + 21)) 0" \
      -c "mwb $((TR + 22)) 5" \
      -c "mwb $((TR + 20)) 2" \
      -c 'exit' >/dev/null
  sleep 0.3
  mkdir -p "$OUT"
  json="$OUT/foc-theta-dir${dir}.json"
  python3 "$ROOT/scripts/trace_dump.py" --no-plot --json "$json" \
    -o "$OUT" --elf "$ELF" --cfg "$CFG" || true
  python3 -c "
import csv, glob, os
Q15=360.0/32768.0
def s16(v):
    v&=0xFFFF
    return v-65536 if v>=32768 else v
csvs=sorted(glob.glob(os.path.expanduser('$OUT/trace-foc_ang-*.csv')))
if not csvs:
    raise SystemExit('no foc_ang csv')
p=csvs[-1]
rows=[]
with open(p) as f:
    for r in csv.DictReader(f):
        rows.append((int(r['foc_disc_q15'])&0xFFFF, int(r['hall_raw'])))
jumps=[]
for (d0,h0),(d1,h1) in zip(rows, rows[1:]):
    if h0==h1 or not (1<=h0<=6 and 1<=h1<=6):
        continue
    j=s16(d1-d0)*Q15
    if j<=-180: j+=360
    elif j>180: j-=360
    jumps.append(j)
print(f'  FocTheta edge jumps deg {[round(j) for j in jumps]}')
if not jumps:
    raise SystemExit('no hall edges in foc_ang')
# One electrical turn is six same-sign 60° steps (wrap 300→0 is +60).
sgn=1 if sum(jumps)>0 else -1
bad=[j for j in jumps if abs(j-sgn*60)>15]
print(f'  jumps {len(jumps)}  expect {sgn*60} deg  outliers {len(bad)}')
if bad:
    raise SystemExit(f'FocTheta still not monotonic: {[round(j) for j in bad]}')
"

  motor_off
done

echo
echo "== reflashing safe firmware =="
trap - EXIT
reflash_safe
"$ROOT/scripts/board_check.sh"
