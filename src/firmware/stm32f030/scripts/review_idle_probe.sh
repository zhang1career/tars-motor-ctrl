#!/usr/bin/env bash
# Read-only SWD: CHSELR, ADC DMA buffer, tick cycles (motor idle, no I2C commands).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=scripts/bench_common.sh
source "$ROOT/scripts/bench_common.sh"
export ELF="${ELF:-$BUILD/motor-ctrl.elf}"
export BENCH_ROOT="$ROOT"

require_dap
ocd_start
cpu_alive || { echo "cpu_alive failed"; exit 1; }

read_u32() { printf '%d' "0x$(read_words "$1" 1 | head -1)"; }

ADC1_CHSELR=0x40012428
diag_addr=$(sym_addr g_reset_diag)

python3 "$OCD_RPC" --port "$OCD_TCL_PORT" "reset run" >/dev/null
sleep 5
python3 "$OCD_RPC" --port "$OCD_TCL_PORT" "halt" >/dev/null

chselr=$(read_u32 "$ADC1_CHSELR")
raw=$(python3 "$ROOT/scripts/review_idle_probe.py" adc)
tick_max=$(read_u32 "$((diag_addr + 16))")
tick_ov=$(read_u32 "$((diag_addr + 20))")
tick_tot=$(read_u32 "$((diag_addr + 24))")
tick_sum=$(read_u32 "$((diag_addr + 36))")
avg=0
if [[ "$tick_tot" -gt 0 ]]; then
  avg=$((tick_sum / tick_tot))
fi

echo "review_idle_probe $(date -Iseconds)"
echo "elf $(basename "$ELF") bytes $(wc -c <"$ELF" | tr -d ' ')"
echo "CHSELR=0x$(printf '%x' "$chselr") (expect 0x3e for IN1-5 scan)"
echo "g_motor_adc_raw: $raw"
echo "tick_cyc_max=$tick_max avg=$avg total=$tick_tot overrun=$tick_ov (steady target<=1200 hard<=2400)"

python3 "$OCD_RPC" --port "$OCD_TCL_PORT" "resume" >/dev/null
