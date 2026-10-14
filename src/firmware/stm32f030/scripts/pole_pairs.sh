#!/usr/bin/env bash
# Measure pole pairs: sample hall_changes for WINDOW_S while user counts shaft revs.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/bench_common.sh"

WINDOW_S="${WINDOW_S:-10}"
RUN_DUTY="${RUN_DUTY:-8}"

require_dap

cmake -S "$BENCH_ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
  -DMOTOR_AUTO_START=ON -DMOTOR_START_HALL6=ON \
  -DMOTOR_HALL6_RUN_DUTY="$RUN_DUTY" -DMOTOR_HALL6_KICK_DUTY="$RUN_DUTY" \
  -DMOTOR_HALL6_CCW=0
cmake --build "$BUILD"
flash_elf

# hall_changes sits 12 bytes into motor_hall6_snapshot_t (8 uint8 then
# loop_count). Resolve the snapshot from the ELF: this was a hardcoded
# 0x20000120, which no longer pointed at any snapshot field after the layout
# moved.
SNAP_ADDR=$(sym_addr s_hall6_snap)
HALL_CHANGES_ADDR=$(python3 -c "print(hex($SNAP_ADDR + 12))")
echo "hall6 snapshot at $SNAP_ADDR, hall_changes at $HALL_CHANGES_ADDR"

read_hall_changes() {
  read_words "$HALL_CHANGES_ADDR" 1 | python3 -c "import sys; print(int(sys.stdin.read().strip(), 16))"
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
