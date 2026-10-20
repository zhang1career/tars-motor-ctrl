#!/usr/bin/env bash
# Idle V_BOOST loop: log g_boost + TIM1->CCR4 (~20 s) without halting.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=scripts/bench_common.sh
source "$ROOT/scripts/bench_common.sh"

TIM1_CCR4=0x40012C40
DUR="${1:-20}"
OUT="${2:-$ROOT/../../../models/captured/boost-idle-watch-$(date +%Y%m%d-%H%M%S).csv}"

parse_u16() {
  local word="$1" which="$2" v
  v=$(printf '%d' "0x$word")
  if [[ "$which" == lo ]]; then
    echo $((v & 0xFFFF))
  else
    echo $(((v >> 16) & 0xFFFF))
  fi
}

require_dap
cpu_alive
GB=$(sym_addr g_boost) || exit 1

echo "t_ms,magic,state,adc_filt,duty_logic,mv,ccr4" >"$OUT"
t0=$(python3 -c 'import time; print(int(time.time()*1000))')
for ((i = 0; i < DUR * 10; i++)); do
  w=$(read_words "$GB" 3) || { sleep 0.1; continue; }
  w1=$(echo "$w" | sed -n '1p')
  w2=$(echo "$w" | sed -n '2p')
  w3=$(echo "$w" | sed -n '3p')
  magic=$(parse_u16 "$w1" lo)
  state=$(parse_u16 "$w1" hi)
  adc_f=$(parse_u16 "$w2" hi)
  duty=$(parse_u16 "$w3" lo)
  mv=$(parse_u16 "$w3" hi)
  ccr=$(read_words "$TIM1_CCR4" 1 | head -1)
  ccr_d=$(printf '%d' "0x$ccr")
  now=$(python3 -c 'import time; print(int(time.time()*1000))')
  echo "$((now - t0)),$magic,$state,$adc_f,$duty,$mv,$ccr_d" >>"$OUT"
  sleep 0.1
done
echo "wrote $OUT"
