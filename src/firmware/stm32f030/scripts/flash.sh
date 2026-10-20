#!/usr/bin/env bash
set -euo pipefail

. "$(cd "$(dirname "$0")/../../../../.." && pwd)/prog/lib.sh"

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD_DIR:-$ROOT/build/Release}"
ELF="$BUILD/motor-ctrl.elf"
CFG="${OPENOCD_CFG:-$ROOT/openocd.cfg}"

if [[ ! -f "$BUILD/build.ninja" && ! -f "$BUILD/Makefile" ]]; then
  echo "missing $BUILD — run: cmake -S $ROOT -B $BUILD -G Ninja" >&2
  exit 1
fi
cmake --build "$BUILD" -j 8
if [[ ! -f "$ELF" ]]; then
  echo "missing $ELF after build" >&2
  exit 1
fi

# The bench keeps one OpenOCD on the Tcl port and it owns the probe. A second
# OpenOCD cannot open the nanoDAP, so program through the running one.
# That needs SWD while the core runs, which holds only with UART_LOG=OFF.
#
# Success is the verify_image reply, not a log line: that daemon's stdout may
# not reach OCD_LOG. verify_image runs a CRC routine in RAM at 0x20000000, so
# it is only safe here, before the reset that reloads .data. Never run it
# against firmware that is still running.
. "$ROOT/scripts/bench_common.sh"
if ocd_listening; then
  rpc() { python3 "$OCD_RPC" --port "$OCD_TCL_PORT" --timeout 90 "$1"; }
  rpc "catch {reset halt; flash write_image erase {$ELF}} r; set r"
  echo
  verify=$(rpc "catch {verify_image {$ELF}} r; set r")
  echo "$verify"
  if [[ "$verify" != verified* ]]; then
    echo "verify failed; target left halted with outputs in reset state" >&2
    exit 1
  fi
  rpc "reset halt; rbp all; resume" >/dev/null
  echo "flashed and verified through the running OpenOCD on port $OCD_TCL_PORT"
  exit 0
fi

# PA14 becomes USART1_TX after boot, so SWD only works under SRST.
"$HW_PROG" openocd nanodap -- -f "$CFG" \
  -c "reset_config srst_only srst_nogate connect_assert_srst" \
  -c "program $ELF verify reset exit"

# Drop the program() breakpoint and leave the core running.
"$HW_PROG" openocd nanodap -- -f "$CFG" \
  -c "init" \
  -c "halt" \
  -c "rbp all" \
  -c "resume" \
  -c "exit" || true
