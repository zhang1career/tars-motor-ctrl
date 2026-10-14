#!/usr/bin/env bash
# Stage E step 1: hall6 drives, fixed-point FOC only observes id/iq.
#
# Does not close the current loop. Used to check angle alignment before any
# SVPWM from FOC is allowed to reach the bridges.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/bench_common.sh"

DUTY="${DUTY:-20}"
SETTLE_S="${SETTLE_S:-4}"
DIRS="${DIRS:-0 1}"
RETRIES="${RETRIES:-3}"
OUT="${OUT:-$BENCH_ROOT/../../../models/captured}"

require_dap
echo "== baseline =="
"$ROOT/scripts/board_check.sh"

spinning() {
  local snap elec i
  snap=$(sym_addr s_hall6_snap)
  # hall_changes is word 3. Two reads 1 s apart.
  local a b
  a=$(read_words "$snap" 4 | awk 'NR==4')
  sleep 1
  b=$(read_words "$snap" 4 | awk 'NR==4')
  elec=$(python3 -c "print((int('$b',16)-int('$a',16))/6)")
  i=$(read_i)
  python3 -c "
elec=float('$elec'); i=float('$i')
print(f'  speed {elec:.1f} elec rev/s  bus {i:.3f} A')
raise SystemExit(0 if elec > 1.0 and i > 0.02 else 1)
"
}

for dir in $DIRS; do
  echo
  echo "== foc observe duty=${DUTY}% direction=${dir} =="
  ok=0
  for try in $(seq 1 "$RETRIES"); do
    rm -rf "$BUILD"
    cmake -S "$BENCH_ROOT" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DMOTOR_AUTO_START=ON -DMOTOR_START_FOC_OBSERVE=ON \
      -DMOTOR_START_BENCH=OFF -DMOTOR_START_HALL6=OFF -DMOTOR_START_DIAG=OFF \
      -DMOTOR_HALL6_RUN_DUTY="$DUTY" -DMOTOR_HALL6_KICK_DUTY="$DUTY" \
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

  "$ROOT/scripts/board_check.sh" --running | rg 'bus |nFAULT|nOTEMP' || true

  # Drop the kick from the mean, then integrate 2 s at 20 kHz.
  ACC=$(sym_addr g_motor_foc_fx_acc)
  ANG=$(sym_addr g_motor_angle)
  ocd -c 'init' -c "mww $((ACC + 8)) 0" -c 'exit' >/dev/null
  sleep 2
  ACC_WORDS=$(read_words "$ACC" 5 | tr '\n' ' ')
  ANG_WORDS=$(read_words "$ANG" 5 | tr '\n' ' ')
  echo "  acc words $ACC_WORDS"
  echo "  angle words $ANG_WORDS"
  python3 -c "
w=[int(x,16) for x in '$ACC_WORDS'.split()]
a=[int(x,16) for x in '$ANG_WORDS'.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
def s16(u):
    u&=0xFFFF
    return u-(1<<16) if u>=(1<<15) else u
n=w[2]
lsb=1.617
def mean(sum_u):
    return s32(sum_u)/n*lsb if n else 0
id_m,iq_m=mean(w[0]),mean(w[1])
ids,iqs=mean(w[3]),mean(w[4])
print(f'  acc interp  n={n}  id={id_m:+.1f} mA  iq={iq_m:+.1f} mA')
print(f'  acc stair   n={n}  id={ids:+.1f} mA  iq={iqs:+.1f} mA')
# g_motor_angle: theta u16, pad, omega i32, ticks u16, sector u8, hall u8,
# dir i8, valid u8, edge_jump u16, bad_edges u16, edges u16
theta=a[0]&0xFFFF
omega=s32(a[1])
ticks=a[2]&0xFFFF
sector=(a[2]>>16)&0xFF
hall=(a[2]>>24)&0xFF
dire=s16(a[3]&0xFF) if (a[3]&0xFF)<128 else (a[3]&0xFF)-256
valid=(a[3]>>8)&0xFF
edge_jump=(a[3]>>16)&0xFFFF
bad=a[4]&0xFFFF
edges=(a[4]>>16)&0xFFFF
print(f'  angle       valid={valid} dir={dire} sector={sector} hall={hall} '
      f'omega_q8={omega} edge_jump={edge_jump} edges={edges} bad={bad} '
      f'theta={theta} ticks={ticks}')
if n < 20000:
    raise SystemExit('acc n too small — ISR not accumulating')
"

  json="$OUT/foc-observe-dir${dir}.json"
  mkdir -p "$OUT"
  python3 "$ROOT/scripts/trace_dump.py" --no-plot --json "$json" \
    -o "$OUT" --elf "$ELF" --cfg "$CFG"

  motor_off
done

echo
echo "== reflashing safe firmware =="
rm -rf "$BUILD"
cmake -S "$BENCH_ROOT" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DMOTOR_AUTO_START=OFF -DMOTOR_START_BENCH=OFF \
  -DMOTOR_START_FOC_OBSERVE=OFF -DMOTOR_START_HALL6=OFF >/dev/null
cmake --build "$BUILD" >/dev/null
flash_elf
"$ROOT/scripts/board_check.sh"
