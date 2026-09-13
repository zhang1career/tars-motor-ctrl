#!/usr/bin/env bash
# Hall6 20%, no FOC handover. Compare every speed meter against wall-clock dt.
# UT372 is optional: a missing tach does not fail the run.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/bench_common.sh"
DESIGN="${DESIGN_DIR:-/Users/mini/Projects/hw-lab/half-bridge/pcb}"
OUT="${OUT:-$BENCH_ROOT/../../../models/captured}"
SETTLE_S="${SETTLE_S:-4}"
RETRIES="${RETRIES:-3}"

reflash_safe() {
  motor_off
  rm -rf "$BUILD"
  cmake -S "$BENCH_ROOT" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DMOTOR_AUTO_START=OFF -DMOTOR_START_BENCH=OFF \
    -DMOTOR_START_FOC_OBSERVE=OFF -DMOTOR_START_HALL6=OFF >/dev/null
  cmake --build "$BUILD" >/dev/null
  flash_elf
}

read_tach() {
  benchgate lab read --design "$DESIGN" --role tach --count 1 2>/dev/null \
    | python3 -c "
import sys,json
try:
    r=json.load(sys.stdin)['readings'][0]
    print(f\"{r.get('value')} {r.get('unit','')} {int(bool((r.get('flags') or {}).get('led')))} {int(bool((r.get('flags') or {}).get('rpm')))}\")
except Exception:
    print('nan none 0 0')
"
}

require_dap
report_and_clear_stale_bif
trap 'motor_off; host_alarm; reflash_safe' EXIT
echo "== baseline =="
"$ROOT/scripts/board_check.sh"

echo "== tach probe (optional) =="
echo "  $(read_tach)   (value unit led rpm)"

ok=0
for try in $(seq 1 "$RETRIES"); do
  rm -rf "$BUILD"
  cmake -S "$BENCH_ROOT" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DMOTOR_AUTO_START=ON -DMOTOR_START_FOC_OBSERVE=ON \
    -DMOTOR_START_HALL6=OFF -DMOTOR_START_BENCH=OFF -DMOTOR_START_DIAG=OFF \
    -DMOTOR_HALL6_RUN_DUTY=20 -DMOTOR_HALL6_KICK_DUTY=20 \
    -DMOTOR_HALL6_PHASE="${PHASE:-3}" -DMOTOR_HALL6_CCW=0 >/dev/null
  cmake --build "$BUILD" >/dev/null
  flash_elf
  cpu_alive
  sleep "$SETTLE_S"
  snap=$(sym_addr s_hall6_snap)
  a=$(read_words "$snap" 4 | awk 'NR==4')
  sleep 1
  b=$(read_words "$snap" 4 | awk 'NR==4')
  i=$(read_i)
  if python3 -c "
elec=(int('$b',16)-int('$a',16))/6
i=float('$i')
print(f'  kick hall6 {elec:.1f} elec/s  bus {i:.3f} A')
raise SystemExit(0 if elec>1.0 and i>0.02 else 1)
"; then
    ok=1
    break
  fi
  echo "  kick missed (try $try/$RETRIES)"
  motor_off
done
if [[ "$ok" != 1 ]]; then
  echo "FAIL: rotor did not start" >&2
  exit 1
fi

ANG=$(sym_addr g_motor_angle)
SNAP=$(sym_addr s_hall6_snap)
FT=""
FT=$(sym_addr s_ft_omega_q8 2>/dev/null) || FT=""
WMEAS=$(sym_addr g_motor_foc_w_meas_eps)
FX=$(sym_addr g_motor_foc_fx)
TR=$(sym_addr g_motor_trace)

echo "== timed window =="
set +e
t0=$(python3 -c "import time; print(time.time())")
h0=$(read_words "$SNAP" 4 | awk 'NR==4')
a0=$(read_words "$ANG" 5 | tr '\n' ' ')
f0=$(read_words "$FX" 6 | awk 'NR==6')
sleep 2
h1=$(read_words "$SNAP" 4 | awk 'NR==4')
a1=$(read_words "$ANG" 5 | tr '\n' ' ')
f1=$(read_words "$FX" 6 | awk 'NR==6')
t1=$(python3 -c "import time; print(time.time())")
i=$(read_i)
tach=$(read_tach)
ft=0
if [[ -n "$FT" ]]; then
  ft=$(read_words "$FT" 1 | head -1)
fi
wm=$(read_words "$WMEAS" 1 | head -1)
echo "  raw h0=$h0 h1=$h1"
echo "  raw a0=$a0"
echo "  raw a1=$a1"
echo "  raw ft=$ft wm=$wm f0=$f0 f1=$f1"
set -e

python3 -c "
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
dt=float('$t1')-float('$t0')
h0=int('$h0',16); h1=int('$h1',16)
w0=[int(x,16) for x in '$a0'.split() if x]
w1=[int(x,16) for x in '$a1'.split() if x]
if len(w0)<5 or len(w1)<5:
    raise SystemExit(f'short angle dump {w0} {w1}')
th0=w0[0]&0xFFFF; th1=w1[0]&0xFFFF
dth=(th1-th0)&0xFFFF
if dth>32768: dth-=65536
e0=(w0[4]>>16)&0xFFFF; e1=(w1[4]>>16)&0xFFFF
de=(e1-e0)&0xFFFF
om=s32(w1[1])
ft=s32(int('$ft',16)) if '$ft' not in ('','0') else None
wm=s32(int('$wm',16))
fx0=int('$f0',16)&0xFFFF; fx1=int('$f1',16)&0xFFFF
dfx=(fx1-fx0)&0xFFFF
if dfx>32768: dfx-=65536
h6=(h1-h0)/6/dt
ang_e=de/6/dt
ang_th=dth/65536/dt
fx_th=dfx/65536/dt
ang_om=om*625/2**19
ft_om=(ft*625/2**19) if ft is not None else float('nan')
print(f'  dt {dt:.3f} s  bus {float(\"$i\"):.3f} A')
print(f'  hall6 edges        {h6:.2f} elec/s')
print(f'  angle.edges        {ang_e:.2f} elec/s')
print(f'  angle.theta        {ang_th:.2f} elec/s')
print(f'  foc_fx.theta       {fx_th:.2f} elec/s')
print(f'  angle.omega_q8     {ang_om:.2f} elec/s  (raw {om})')
print(f'  hall6 FocOmegaQ8   {ft_om:.2f} elec/s  (raw {ft})')
print(f'  w_meas_eps         {wm} elec/s  (speed PI; 0 if spd off)')
tach='$tach'.split()
try:
    tv=float(tach[0]); unit=tach[1]; led=int(tach[2]); rpm=int(tach[3])
except Exception:
    tv=float('nan'); unit='none'; led=0; rpm=0
if unit=='rpm' and tv==tv:
    print(f'  UT372              {tv:.1f} rpm  → {tv/60*4:.2f} elec/s  led={led}')
else:
    print(f'  UT372              unavailable ({tach})')
"

echo "== 256 ms foc_ang wrap =="
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
sleep 0.35
mkdir -p "$OUT"
json="$OUT/speed-reconcile-hall6.json"
python3 "$ROOT/scripts/trace_dump.py" --no-plot --json "$json" \
  -o "$OUT" --elf "$ELF" --cfg "$CFG"
python3 -c "
import json
p=json.load(open('$json'))
net=float(p.get('net_theta_deg') or 0)
dt=float(p.get('samples',256))*float(p.get('dt_us',1000))/1e6
elec=abs(net)/360/dt if dt else float('nan')
print(f'  foc_ang net {net:.1f} deg in {dt*1000:.0f} ms → {elec:.2f} elec/s  rotating={p.get(\"rotating\")}')
"

echo "== stop =="
trap - EXIT
reflash_safe
"$ROOT/scripts/board_check.sh"
echo "DONE"
