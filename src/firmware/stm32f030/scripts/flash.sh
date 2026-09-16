#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD_DIR:-$ROOT/build/Release}"
ELF="$BUILD/motor-ctrl.elf"
CFG="${OPENOCD_CFG:-$ROOT/openocd.cfg}"

if [[ ! -f "$ELF" ]]; then
  echo "missing $ELF — run: cmake -S $ROOT -B $BUILD -G Ninja && cmake --build $BUILD" >&2
  exit 1
fi

# PA14 becomes USART1_TX after boot, so SWD only works under SRST.
openocd -f "$CFG" \
  -c "reset_config srst_only srst_nogate connect_assert_srst" \
  -c "program $ELF verify reset exit"

# Drop the program() breakpoint and leave the core running.
openocd -f "$CFG" \
  -c "init" \
  -c "halt" \
  -c "rbp all" \
  -c "resume" \
  -c "exit" || true
