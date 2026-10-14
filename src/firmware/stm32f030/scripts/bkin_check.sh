#!/usr/bin/env bash
# Arm TIM1 BKIN and the software current limit, then prove:
#   1. hall6 20% still runs (BKE=1, BIF=0, no false trip)
#   2. writing g_motor_pwm_force_trip kills MOE without real overcurrent
#
# Does not raise duty or close the current loop.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/bench_common.sh"

require_dap

flash_cfg() {
  rm -rf "$BUILD"
  cmake -S "$BENCH_ROOT" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release "$@" >/dev/null
  cmake --build "$BUILD" >/dev/null
  flash_elf
  cpu_alive
}

echo "== flash BKIN-armed idle firmware =="
flash_cfg -DMOTOR_AUTO_START=OFF -DMOTOR_START_BENCH=OFF \
  -DMOTOR_START_FOC_OBSERVE=OFF -DMOTOR_START_HALL6=OFF
"$ROOT/scripts/board_check.sh"

spinning() {
  local snap a b elec i
  snap=$(sym_addr s_hall6_snap)
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

echo
echo "== hall6 20% with BKIN armed =="
ok=0
for try in 1 2 3; do
  flash_cfg -DMOTOR_AUTO_START=ON -DMOTOR_START_HALL6=ON \
    -DMOTOR_START_BENCH=OFF -DMOTOR_START_FOC_OBSERVE=OFF \
    -DMOTOR_HALL6_RUN_DUTY=20 -DMOTOR_HALL6_KICK_DUTY=20 \
    -DMOTOR_HALL6_PHASE=3 -DMOTOR_HALL6_CCW=0
  sleep 4
  if spinning; then
    ok=1
    break
  fi
  echo "  kick missed (try $try/3), retrying"
  motor_off
done
if [[ "$ok" != 1 ]]; then
  echo "FAIL: rotor did not start with BKIN armed" >&2
  motor_off
  exit 1
fi

"$ROOT/scripts/board_check.sh" --running | rg 'TIM1|nFAULT|bus ' || true
TIM1_LINE=$(ocd -c 'init' -c 'mdw 0x40012C20 1' -c 'mdw 0x40012C44 1' \
            -c 'mdw 0x40012C10 1' -c 'exit' \
            | rg '^0x[0-9a-f]+:' | sed 's/^[^:]*: *//' | tr '\n' ' ')
echo "  TIM1 words $TIM1_LINE"
python3 -c "
w=[int(x,16) for x in '$TIM1_LINE'.split()]
ccer,bdtr,sr=w[:3]
moe,bke,bif=(bdtr>>15)&1,(bdtr>>12)&1,(sr>>7)&1
print(f'  TIM1 MOE={moe} BKE={bke} BIF={bif} CCER=0x{ccer:04x}')
if moe!=1 or bke!=1 or bif!=0:
    raise SystemExit('BKIN false-tripped or not armed while spinning')
"

PROT=$(sym_addr g_motor_pwm_prot)
FORCE=$(sym_addr g_motor_pwm_force_trip)
echo
echo "== software trip via SWD =="
ocd -c 'init' -c "mwb $FORCE 1" -c 'exit' >/dev/null
sleep 0.15
TIM1_LINE=$(ocd -c 'init' -c 'mdw 0x40012C44 1' -c 'mdw 0x40012C10 1' -c 'exit' \
            | rg '^0x[0-9a-f]+:' | sed 's/^[^:]*: *//' | tr '\n' ' ')
PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
echo "  BDTR/SR $TIM1_LINE"
echo "  prot words $PROT_WORDS"
python3 -c "
bdtr,sr=[int(x,16) for x in '$TIM1_LINE'.split()][:2]
w=[int(x,16) for x in '$PROT_WORDS'.split()]
sw,bkin=w[0],w[1]
latched=(w[3]>>16)&0xFF
moe,bke,bif=(bdtr>>15)&1,(bdtr>>12)&1,(sr>>7)&1
print(f'  after trip MOE={moe} BKE={bke} BIF={bif} latched={latched} '
      f'sw_trips={sw} bkin_trips={bkin}')
if moe!=0:
    raise SystemExit('software trip did not clear MOE')
if sw<1 or latched!=1:
    raise SystemExit('sw_trips/latched did not record the trip')
"

motor_off
echo
echo "== reflashing safe firmware =="
flash_cfg -DMOTOR_AUTO_START=OFF -DMOTOR_START_BENCH=OFF \
  -DMOTOR_START_FOC_OBSERVE=OFF -DMOTOR_START_HALL6=OFF
"$ROOT/scripts/board_check.sh"
