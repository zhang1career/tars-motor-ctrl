#!/usr/bin/env bash
# Reset via nanoDAP, sample PA5 (g_motor_adc_raw[4]) and g_boost while running.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=scripts/bench_common.sh
source "$ROOT/scripts/bench_common.sh"
HW="${HW_PROG:-/Users/mini/Projects/hw-lab/prog/prog}"
CFG="${OPENOCD_CFG:-$ROOT/openocd.cfg}"
WAIT_S="${1:-15}"

GB=$(sym_addr g_boost)
RAW=$(sym_addr g_motor_adc_raw)
PA5_OFF=$((RAW + 8))

parse_mv() {
  python3 - "$1" <<'PY'
import struct, sys
w = int(sys.argv[1], 16)
adc = (w >> 16) & 0xFFFF
mv = adc * 3300 * 19 // 4095
print(mv)
PY
}

echo "boost_reset_verify wait=${WAIT_S}s"
"$HW" openocd nanodap -- -f "$CFG" \
  -c "reset_config srst_only srst_nogate connect_assert_srst" \
  -c "init" \
  -c "reset run" \
  -c "sleep $(($WAIT_S * 1000))" \
  -c "halt" \
  -c "mdw $GB 4" \
  -c "mdh $RAW 5" \
  -c "mdw 0x40012440 1" \
  -c "mdw 0x40012C44 1" \
  -c "mdw 0x40012C40 1" \
  -c "reg pc" \
  -c "exit" 2>&1 | tee /tmp/boost_reset_verify.log

grep -E "^(0x|pc )" /tmp/boost_reset_verify.log || true
PA5=$(grep -F "$RAW" /tmp/boost_reset_verify.log | tail -1 | awk '{print $6}' | tr -d ',:' | sed 's/^0x//' || echo 0)
echo "PA5_raw_halfword=$PA5 mv~$(parse_mv "0x${PA5}0000" 2>/dev/null || echo '?')"
