#!/usr/bin/env bash
# Open-loop duty sweep: flash AUTO_START per duty, hold, read bus current, stop.
# Dead time subtracts from the HS pulse, so duty alone does not describe the
# drive level - see DTG in motor_pwm.c.
set -euo pipefail

source "$(dirname "$0")/bench_common.sh"

DTG="${DTG:-72}"
STEP_MS="${STEP_MS:-20}"
PHASE="${PHASE:-3}"
HS_ONLY="${HS_ONLY:-OFF}"
HOLD_S="${HOLD_S:-6}"
DUTIES="${DUTIES:-2 3 4 5 6 7}"

build_flash() {
  local duty=$1 auto=$2
  cmake -S "$BENCH_ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
    -DMOTOR_AUTO_START="$auto" -DMOTOR_DUTY_PCT="$duty" -DMOTOR_TIM1_DTG="$DTG" \
    -DMOTOR_PULSE_COUNTS=0 -DMOTOR_STEP_MS="$STEP_MS" -DMOTOR_PHASE_OFFSET="$PHASE" \
    -DMOTOR_HS_ONLY="$HS_ONLY" >/dev/null
  cmake --build "$BUILD" >/dev/null
  flash_elf
}

require_dap
echo "duty_sweep: DTG=$DTG step=${STEP_MS}ms hs_only=$HS_ONLY hold=${HOLD_S}s duties=[$DUTIES]"
for duty in $DUTIES; do
  build_flash "$duty" ON
  sleep "$HOLD_S"
  i=$(read_i)
  motor_off
  sleep 0.5
  echo "duty=${duty}% I=${i}A idle_after=$(read_i)A"
done

build_flash 7 OFF
echo "safe firmware flashed (AUTO_START=OFF), idle=$(read_i)A"
