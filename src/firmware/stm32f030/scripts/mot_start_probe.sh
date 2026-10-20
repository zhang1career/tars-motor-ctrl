#!/usr/bin/env bash
# Snapshot Boost_Ready, PA6 nFAULT, MOE around one mot start (no halt).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=scripts/bench_common.sh
source "$ROOT/scripts/bench_common.sh"
TIM1_BDTR=0x40012C44
GPIOA_IDR=0x48000010
NFAULT_BIT=64  # PA6

read_u16() { printf '%d' "0x$(read_words "$1" 1 | head -1 | cut -c5-8)"; }
read_u32() { printf '%d' "0x$(read_words "$1" 1 | head -1)"; }

probe() {
  local tag=$1
  local gb sf pa bdtr flags adc mv duty ready
  gb=$(sym_addr g_boost)
  sf=$(python3 "$BENCH_ROOT/scripts/sym_cache.py" "$ELF" s_flags)
  gbw=$(read_words "$gb" 3)
  w2=$(echo "$gbw" | sed -n '2p')
  w3=$(echo "$gbw" | sed -n '3p')
  adc=$(printf '%d' "0x$(echo "$w2" | cut -c5-8)")
  mv=$(printf '%d' "0x$(echo "$w3" | cut -c1-4)")
  duty=$(printf '%d' "0x$(echo "$w3" | cut -c5-8)")
  flags=$(read_u16 "$sf" lo 2>/dev/null || echo 0)
  ready=$((flags & 1))
  pa=$(read_u32 "$GPIOA_IDR")
  nf=$(((pa & NFAULT_BIT) != 0 ? 1 : 0))
  bdtr=$(read_u32 "$TIM1_BDTR")
  moe=$(((bdtr & 0x8000) != 0 ? 1 : 0))
  echo "$tag adc=$adc mv=$mv duty=$duty boost_ready=$ready nFAULT=$nf MOE=$moe BDTR=0x$(printf '%x' "$bdtr")"
}

require_dap
cpu_alive
export TARS_CDC="${TARS_CDC:-/dev/cu.usbmodem345B345933331}"
probe idle_before
python3 - "$TARS_CDC" << 'PY'
import sys, time
sys.path.insert(0, "/Users/mini/Projects/hw-lab/motor-ctrl/src/firmware/stm32f030/scripts")
import tars_cdc
from mot_bench import parse_mot
port = sys.argv[1]
ser = tars_cdc.open_session()
tars_cdc.session_cmd(ser, "nodebus mot stop", 2.0)
tars_cdc.session_cmd(ser, "nodebus wr 0x50 0x91 0x02", 1.5)
ser.write(b"nodebus mot start torque 0 0 0\r")
ser.flush()
time.sleep(0.55)
txt = tars_cdc.session_cmd(ser, "nodebus mot stat", 1.5)
st = parse_mot(txt) or {}
print(f"stat st={st.get('st')} flt=0x{st.get('flt',0):04x} hall={st.get('hall')}")
tars_cdc.session_cmd(ser, "nodebus mot stop", 2.0)
ser.close()
PY
probe after_start
OUT="$ROOT/../../../models/captured/motor-resume-20261020/start-probe.txt"
{
  echo "mot_start_probe $(date -Iseconds)"
  probe idle_before
  probe after_start
} | tee "$OUT" | tail -5
