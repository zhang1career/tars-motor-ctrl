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

openocd -f "$CFG" \
  -c "program $ELF verify reset" \
  -c "init" -c "rbp all" -c "resume" -c "exit"
