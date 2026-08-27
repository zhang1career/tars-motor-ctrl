#!/usr/bin/env bash
# Measure pole pairs: sample hall_changes for WINDOW_S while user counts shaft revs.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/bench_common.sh"

WINDOW_S="${WINDOW_S:-10}"
RUN_DUTY="${RUN_DUTY:-8}"
HALL_CHANGES_ADDR=0x20000120

require_dap

cmake -S "$BENCH_ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
  -DMOTOR_AUTO_START=ON -DMOTOR_START_HALL6=ON \
  -DMOTOR_HALL6_RUN_DUTY="$RUN_DUTY" -DMOTOR_HALL6_KICK_DUTY="$RUN_DUTY" \
  -DMOTOR_HALL6_CCW=0
cmake --build "$BUILD"
flash_elf

read_hall_changes() {
  openocd -f "$CFG" -c 'init' -c "mdw $HALL_CHANGES_ADDR 1" -c 'exit' 2>/dev/null \
    | awk '/^0x/ { print strtonum($2) }'
}

echo "waiting 3 s for motor to stabilize (duty ${RUN_DUTY}%)..."
sleep 3

I=$(read_i)
echo "bus current: ${I} A"

START=$(read_hall_changes)
echo ""
echo ">>> COUNT SHAFT REVOLUTIONS NOW — window ${WINDOW_S} s <<<"
sleep "$WINDOW_S"
END=$(read_hall_changes)

DELTA=$((END - START))
ELEC_REVS=$(python3 -c "print(f'{$DELTA}/6')")

echo ""
echo "hall_changes: ${START} -> ${END}  (delta ${DELTA})"
echo "electrical revolutions: ${ELEC_REVS}  (6 hall edges per elec rev)"
echo ""
echo "pole_pairs = (${DELTA} / 6) / <mechanical revs you counted>"

motor_off
