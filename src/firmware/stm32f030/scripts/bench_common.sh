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

ELF="${ELF:-$BUILD/motor-ctrl.elf}"

# cmake is not on the default PATH on this host; the CMake.app copy is.
if ! command -v cmake >/dev/null 2>&1 && [[ -x /Applications/CMake.app/Contents/bin/cmake ]]; then
  PATH="/Applications/CMake.app/Contents/bin:$PATH"
fi

NM="${NM:-}"
if [[ -z "$NM" ]]; then
  for _nm in /Applications/ArmGNUToolchain/*/arm-none-eabi/bin/arm-none-eabi-nm; do
    [[ -x "$_nm" ]] && NM="$_nm"
  done
  NM="${NM:-arm-none-eabi-nm}"
fi

ocd() { openocd -f "$CFG" "$@" 2>&1 | rg -v 'gdb to socket|Address already in use' || true; }

# Resolve a static's address from the ELF instead of hardcoding it. Layout moves
# whenever the code changes: docs once recorded openloop s_snap at 0x200000fc
# while the linker had since put it at 0x200000b4, and pole_pairs.sh was reading
# an address that belonged to neither.
sym_addr() {
  local name="$1" addr
  addr=$("$NM" "$ELF" | rg "^([0-9a-f]{8}) [bBdD] ${name}\$" -r '$1' | head -1)
  if [[ -z "$addr" ]]; then
    echo "sym_addr: '$name' not found in $ELF" >&2
    return 1
  fi
  printf '0x%s' "$addr"
}

# Read n words at addr without halting. Cortex-M memory access goes through the
# DAP, so this works with the motor running -- halting would freeze commutation
# mid-step while TIM1 keeps switching.
read_words() {
  local addr="$1" n="${2:-1}"
  ocd -c 'init' -c "mdw $addr $n" -c 'exit' \
    | rg '^0x[0-9a-f]+:' \
    | sed 's/^[^:]*: *//' \
    | tr ' ' '\n' \
    | rg '^[0-9a-f]{8}$'
}

# HAL's uwTick advances every 1 ms; two reads that differ prove main() is
# running. Non-invasive, unlike halting to inspect PC.
cpu_alive() {
  local addr t1 t2
  addr=$(sym_addr uwTick) || return 1
  t1=$(read_words "$addr" 1)
  sleep 0.3
  t2=$(read_words "$addr" 1)
  if [[ -z "$t1" || -z "$t2" || "$t1" == "$t2" ]]; then
    echo "cpu_alive: uwTick stuck at 0x$t1 - CPU not running main()" >&2
    return 1
  fi
  return 0
}

# Read TIM1_SR BIF and nFAULT before touching them. A stale BIF with
# nFAULT high is leftover from a previous trip; clear only then.
report_and_clear_stale_bif() {
  local sr idr bif nfault
  sr=$(read_words 0x40012C10 1 | head -1)
  idr=$(read_words 0x48000010 1 | head -1)
  bif=$(( (16#${sr:-0} >> 7) & 1 ))
  nfault=$(( (16#${idr:-0} >> 6) & 1 ))
  echo "  TIM1_SR BIF=${bif}  nFAULT=${nfault}  (1 = no fault)"
  if [[ "$bif" -eq 1 && "$nfault" -eq 1 ]]; then
    echo "  clearing stale BIF (nFAULT is high)"
    ocd -c 'init' -c 'mmw 0x40012C10 0 0x80' -c 'exit' >/dev/null
  elif [[ "$bif" -eq 1 ]]; then
    echo "  BIF set and nFAULT low — not clearing; find the fault" >&2
    return 1
  fi
  return 0
}

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

# Stop the controller, then the outputs. Preserves DTG so dead time survives.
#
# Clearing MOE and CCER alone is not enough: the control ISR keeps running and
# rewrites CCER on the next hall edge, which the coasting rotor supplies. That
# leaves the board armed rather than stopped, and board_check.sh flags it. So
# clear the two enable flags first -- both are volatile, so the ISR sees them.
motor_off() {
  local cmds=() sym addr fx
  for sym in s_hall6_enable s_ol_enable; do
    addr=$(sym_addr "$sym" 2>/dev/null) && cmds+=(-c "mwb $addr 0")
  done
  addr=$(sym_addr g_motor_foc_handover 2>/dev/null) && cmds+=(-c "mwb $addr 2")
  # mode sits at byte +32 of g_motor_foc_fx. Write it here so the next
  # CURRENT tick cannot OR CCER back on after we clear MOE.
  fx=$(sym_addr g_motor_foc_fx 2>/dev/null) && cmds+=(-c "mwb $((fx + 32)) 0")
  if [[ ${#cmds[@]} -gt 0 ]]; then
    ocd -c 'init' "${cmds[@]}" \
        -c "mmw $TIM1_BDTR 0 0x8000" -c "mww $TIM1_CCER 0" -c 'exit' >/dev/null
  else
    ocd -c 'init' \
        -c "mmw $TIM1_BDTR 0 0x8000" -c "mww $TIM1_CCER 0" -c 'exit' >/dev/null
  fi
}

# DAP is host-pull: the MCU cannot push. The ISR writes g_motor_foc_fx.mode
# when it actually takes CURRENT; the host polls that byte and beeps.
host_beep() {
  local snd
  for snd in /System/Library/Sounds/Glass.aiff \
             /System/Library/Sounds/Ping.aiff \
             /System/Library/Sounds/Sosumi.aiff; do
    if [[ -f "$snd" ]]; then
      afplay "$snd" >/dev/null 2>&1 &
      return
    fi
  done
  printf '\a'
}

# Distinct from host_beep (Glass). Three Bassos plus a spoken line so
# a person in the room hears a fault without watching the terminal.
host_alarm() {
  local snd=/System/Library/Sounds/Basso.aiff
  echo "  ALARM — motor fault" >&2
  if [[ -f "$snd" ]]; then
    afplay "$snd" >/dev/null 2>&1
    afplay "$snd" >/dev/null 2>&1
    afplay "$snd" >/dev/null 2>&1
  else
    printf '\a\a\a' >&2
  fi
  say -v Tingting '电机故障' >/dev/null 2>&1 \
    || say 'motor fault' >/dev/null 2>&1 \
    || true
}

# After the host writes g_motor_foc_handover=1, wait until the ISR acks
# CURRENT (mode byte at g_motor_foc_fx+32), then beep.
wait_foc_current_beep() {
  local fx="$1" word mode try
  for try in 1 2 3 4 5 6 7 8 9 10; do
    word=$(read_words "$((fx + 32))" 1 | head -1)
    mode=$((16#${word:-0} & 0xFF))
    if [[ "$mode" -eq 2 ]]; then
      echo "  MCU ack CURRENT — beep"
      host_beep
      return 0
    fi
    sleep 0.05
  done
  echo "  MCU did not ack CURRENT (no beep)" >&2
  return 1
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
