#!/usr/bin/env bash
# Shared helpers for the motor-ctrl bench sweeps. Source, do not execute.

_bench_self="${BASH_SOURCE[0]:-$0}"
BENCH_ROOT="$(cd "$(dirname -- "$_bench_self")/.." && pwd)"
if [[ ! -f "$BENCH_ROOT/CMakeLists.txt" ]]; then
  echo "bench_common.sh: cannot locate the firmware root from '$_bench_self'" >&2
  return 1 2>/dev/null || exit 1
fi
BUILD="${BUILD:-${BUILD_DIR:-$BENCH_ROOT/build/Release}}"
CFG="${OPENOCD_CFG:-$BENCH_ROOT/openocd.cfg}"
DESIGN="${DESIGN_DIR:-/Users/mini/Projects/hw-lab/half-bridge/pcb}"

TIM1_BDTR=0x40012C44
TIM1_CCER=0x40012C20

ocd() { openocd -f "$CFG" "$@" 2>&1 | rg -v 'gdb to socket|Address already in use' || true; }

# The MCU is powered through the debug probe, so a missing probe looks exactly
# like a dead board: no PWM and no bus current. Fail loudly instead of measuring
# a target that is not running.
require_dap() {
  if ocd -c 'init' -c 'exit' | rg -qi 'unable to find a matching CMSIS-DAP'; then
    echo "no CMSIS-DAP probe found - plug in the nanoDAP and retry" >&2
    exit 1
  fi
}

flash_elf() {
  if ! openocd -f "$CFG" -c "program $BUILD/motor-ctrl.elf verify reset" \
         -c 'init' -c 'rbp all' -c 'resume' -c 'exit' 2>&1 | rg -q 'Verified OK'; then
    echo "flash failed - probe or target lost" >&2
    exit 1
  fi
}

# Clear MOE and CCER only; preserves DTG so dead time survives a stop.
motor_off() {
  ocd -c 'init' -c 'halt' -c 'rbp all' \
      -c "mmw $TIM1_BDTR 0 0x8000" -c "mww $TIM1_CCER 0" \
      -c 'resume' -c 'exit' >/dev/null
}

# UT61E serial telemetry drops frames occasionally; retry before giving up.
read_i() {
  local out
  for _ in 1 2 3 4; do
    out=$(benchgate lab read --design "$DESIGN" --count 1 2>/dev/null) || true
    if [[ -n "$out" ]]; then
      printf '%s' "$out" \
        | python3 -c "import sys,json; print(json.load(sys.stdin)['readings'][0]['value'])" && return 0
    fi
    sleep 1
  done
  echo nan
}
