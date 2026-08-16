#!/usr/bin/env bash
# Open-loop sweep by raw CCR counts. 1% duty is 12 counts at ARR=1199, too coarse
# near the dead-time threshold, so sweep counts directly.
set -euo pipefail

source "$(dirname "$0")/bench_common.sh"

DTG="${DTG:-72}"
STEP_MS="${STEP_MS:-200}"
PHASE="${PHASE:-3}"
HS_ONLY="${HS_ONLY:-OFF}"
HOLD_S="${HOLD_S:-6}"
COUNTS="${COUNTS:-38 40 42 44 46}"

build_flash() {
  local counts=$1 auto=$2
  cmake -S "$BENCH_ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
    -DMOTOR_AUTO_START="$auto" -DMOTOR_PULSE_COUNTS="$counts" -DMOTOR_TIM1_DTG="$DTG" \
    -DMOTOR_STEP_MS="$STEP_MS" -DMOTOR_PHASE_OFFSET="$PHASE" \
    -DMOTOR_HS_ONLY="$HS_ONLY" >/dev/null
  cmake --build "$BUILD" >/dev/null
  flash_elf
}

require_dap
echo "ccr_sweep: DTG=$DTG step=${STEP_MS}ms hs_only=$HS_ONLY hold=${HOLD_S}s counts=[$COUNTS]"
for c in $COUNTS; do
  build_flash "$c" ON
  sleep "$HOLD_S"
  i=$(read_i)
  motor_off
  sleep 0.5
  echo "CCR=${c} hs_on=$((2 * c - DTG))counts I=${i}A"
done

build_flash 0 OFF
echo "safe firmware flashed (AUTO_START=OFF), idle=$(read_i)A"
