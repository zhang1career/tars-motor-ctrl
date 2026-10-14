#!/usr/bin/env bash
# Hold each of the 6 commutation steps statically and log bus current.
# A symmetric 3-phase load draws the same current on every step; an outlier
# points at one half-bridge board or phase wire. Keep the drive below the supply
# current limit or every step reads the same clamped value.
set -euo pipefail

source "$(dirname "$0")/bench_common.sh"

DTG="${DTG:-72}"
CCR="${CCR:-44}"
HS_ONLY="${HS_ONLY:-OFF}"
HOLD_S="${HOLD_S:-4}"
# MotorOpenloop_SetStepMs() clamps any non-zero value to 5..500 ms; only 0 truly
# holds a step. This used to be 60000, which became 500 ms and silently turned
# every "static" step into 8 commutations per hold -- see
# docs/measurement-validity.md section 1.4.
STATIC_STEP_MS=0

build_flash() {
  local phase=$1 auto=$2 step_ms=$3
  cmake -S "$BENCH_ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
    -DMOTOR_AUTO_START="$auto" -DMOTOR_PULSE_COUNTS="$CCR" -DMOTOR_TIM1_DTG="$DTG" \
    -DMOTOR_STEP_MS="$step_ms" -DMOTOR_PHASE_OFFSET="$phase" \
    -DMOTOR_HS_ONLY="$HS_ONLY" >/dev/null
  cmake --build "$BUILD" >/dev/null
  flash_elf
}

require_dap
echo "step_table: DTG=$DTG CCR=$CCR hs_only=$HS_ONLY hold=${HOLD_S}s (static, no commutation)"
for phase in 0 1 2 3 4 5; do
  build_flash "$phase" ON "$STATIC_STEP_MS"
  sleep "$HOLD_S"
  i=$(read_i)
  motor_off
  sleep 0.5
  echo "step_phase=${phase} I=${i}A"
done

build_flash 3 OFF 20
echo "safe firmware flashed (AUTO_START=OFF), idle=$(read_i)A"
