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
TIM1_ARR=0x40012C2C
TIM1_CCR1=0x40012C34

OCD_TCL_PORT="${OCD_TCL_PORT:-6666}"
# BUILD is rm -rf'd every kick; keep the daemon bookkeeping off that tree.
OCD_PID_FILE="${OCD_PID_FILE:-/tmp/motor-ctrl-openocd.pid}"
OCD_LOG="${OCD_LOG:-/tmp/motor-ctrl-openocd.log}"
OCD_RPC="${OCD_RPC:-$BENCH_ROOT/scripts/ocd_rpc.py}"

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

ocd_listening() {
  python3 -c '
import socket, sys
s = socket.socket()
s.settimeout(0.2)
try:
    s.connect(("127.0.0.1", int(sys.argv[1])))
except Exception:
    sys.exit(1)
' "$OCD_TCL_PORT"
}

# One OpenOCD process, Tcl RPC on OCD_TCL_PORT. Spawn-per-mdw was the
# thing that made adapter 400 feel slow and still hung CMSIS-DAP.
ocd_start() {
  local i
  if ocd_listening; then
    return 0
  fi
  mkdir -p "$BUILD"
  # nohup + disown: otherwise the daemon dies with the calling script
  # (board_check, foc_current kick) via SIGHUP.
  nohup openocd -f "$CFG" -c "tcl_port $OCD_TCL_PORT" -c "gdb_port disabled" \
    >"$OCD_LOG" 2>&1 &
  echo $! > "$OCD_PID_FILE"
  disown $! 2>/dev/null || true
  for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25; do
    if ocd_listening; then
      python3 "$OCD_RPC" --port "$OCD_TCL_PORT" "init" >/dev/null 2>&1 || true
      return 0
    fi
    sleep 0.1
  done
  echo "ocd_start: daemon did not listen on $OCD_TCL_PORT" >&2
  if [[ -f "$OCD_LOG" ]]; then
    tail -n 30 "$OCD_LOG" >&2 || true
  fi
  return 1
}

ocd_stop() {
  if [[ -f "$OCD_PID_FILE" ]]; then
    kill "$(cat "$OCD_PID_FILE")" 2>/dev/null || true
    rm -f "$OCD_PID_FILE"
    sleep 0.2
  fi
}

ocd() {
  # Persistent Tcl RPC. Callers still pass -c init / -c exit; those are
  # dropped. Do not `|| true` a failed rpc — truncated mdw is not data.
  local cmds=() out rc
  while [[ $# -gt 0 ]]; do
    if [[ "$1" == -c ]]; then
      shift
      case "$1" in
        init|exit) ;;
        *) cmds+=("$1") ;;
      esac
      shift
    else
      shift
    fi
  done
  ocd_start || return 1
  if [[ ${#cmds[@]} -eq 0 ]]; then
    return 0
  fi
  # echo/source go to OpenOCD stdout (the daemon log), not the Tcl reply.
  # mdw/mww come back on the RPC socket — do not also splice the log or
  # read_words will double-count.
  local need_log=0 log_pos=0
  local c
  for c in "${cmds[@]}"; do
    if [[ "$c" == *echo* || "$c" == *source* || "$c" == *$'\n'* ]]; then
      need_log=1
    fi
  done
  if [[ "$need_log" == 1 && -f "$OCD_LOG" ]]; then
    log_pos=$(wc -c < "$OCD_LOG" | tr -d ' ')
  fi
  if out=$(python3 "$OCD_RPC" --port "$OCD_TCL_PORT" "${cmds[@]}" 2>&1); then
    printf '%s' "$out"
    if [[ "$need_log" == 1 && -f "$OCD_LOG" ]]; then
      tail -c +"$((log_pos + 1))" "$OCD_LOG" | rg -v '^Info :' || true
    fi
    return 0
  fi
  echo "ocd: rpc failed, restarting daemon" >&2
  ocd_stop
  ocd_start || return 1
  if [[ "$need_log" == 1 && -f "$OCD_LOG" ]]; then
    log_pos=$(wc -c < "$OCD_LOG" | tr -d ' ')
  fi
  out=$(python3 "$OCD_RPC" --port "$OCD_TCL_PORT" "${cmds[@]}" 2>&1) || {
    rc=$?
    printf '%s' "$out"
    return "$rc"
  }
  printf '%s' "$out"
  if [[ "$need_log" == 1 && -f "$OCD_LOG" ]]; then
    tail -c +"$((log_pos + 1))" "$OCD_LOG" | rg -v '^Info :' || true
  fi
  return 0
}

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
  local addr="$1" n="${2:-1}" try out count
  for try in 1 2 3; do
    if ! out=$(ocd -c 'init' -c "mdw $addr $n" -c 'exit'); then
      echo "read_words: ocd failed at $addr n=$n try=$try" >&2
      sleep 0.2
      continue
    fi
    out=$(printf '%s\n' "$out" \
      | rg '^0x[0-9a-f]+:' \
      | sed 's/^[^:]*: *//' \
      | tr ' ' '\n' \
      | rg '^[0-9a-f]{8}$' || true)
    count=$(printf '%s\n' "$out" | rg -c '^[0-9a-f]{8}$' || true)
    if [[ "${count:-0}" -eq "$n" ]]; then
      printf '%s\n' "$out"
      return 0
    fi
    echo "read_words: got ${count:-0} of $n at $addr try=$try" >&2
    sleep 0.2
  done
  return 1
}

# HAL's uwTick advances every 1 ms. Two *plausible* reads that increment
# prove main() is running. A garbage DAP word (0x7ba5b08e, or a 10 s
# tick after a 15 min run) is a probe error, not a dead CPU — retry
# before the caller treats this as fatal.
cpu_alive() {
  local addr t1 t2 try v1 v2 dv
  addr=$(sym_addr uwTick) || return 1
  for try in 1 2 3; do
    t1=$(read_words "$addr" 1) || {
      echo "cpu_alive: DAP read failed (try $try)" >&2
      sleep 0.3
      continue
    }
    sleep 0.3
    t2=$(read_words "$addr" 1) || {
      echo "cpu_alive: DAP read failed (try $try)" >&2
      sleep 0.3
      continue
    }
    if [[ ! "$t1" =~ ^[0-9a-f]{8}$ || ! "$t2" =~ ^[0-9a-f]{8}$ ]]; then
      echo "cpu_alive: bad hex '$t1' '$t2' (try $try)" >&2
      continue
    fi
    v1=$((16#$t1))
    v2=$((16#$t2))
    dv=$((v2 - v1))
    # Persistent OpenOCD makes the 0.3 s sleep ~300 ticks. 20..30000
    # still covers a slow DAP; a 24-day garbage word jumps millions.
    if (( dv >= 20 && dv <= 30000 )); then
      return 0
    fi
    echo "cpu_alive: implausible uwTick 0x$t1 -> 0x$t2 dv=$dv (try $try) — DAP garbage, not a dead CPU" >&2
    sleep 0.3
  done
  echo "cpu_alive: no plausible uwTick after 3 tries" >&2
  return 1
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
  if ! ocd_start; then
    if [[ -f "$OCD_LOG" ]] && rg -qi 'unable to find a matching CMSIS-DAP' "$OCD_LOG"; then
      echo "no CMSIS-DAP probe found - plug in the nanoDAP and retry" >&2
    else
      echo "openocd failed to start — see $OCD_LOG" >&2
    fi
    exit 1
  fi
}

flash_elf() {
  local out log_pos=0 log_delta
  ocd_start || {
    echo "flash failed - openocd did not start" >&2
    exit 1
  }
  if [[ -f "$OCD_LOG" ]]; then
    log_pos=$(wc -c < "$OCD_LOG" | tr -d ' ')
  fi
  out=$(python3 "$OCD_RPC" --port "$OCD_TCL_PORT" --timeout 90 \
    "halt" \
    "program {$BUILD/motor-ctrl.elf} verify reset" \
    "rbp all" \
    "resume" 2>&1) || true
  log_delta=""
  if [[ -f "$OCD_LOG" ]]; then
    log_delta=$(tail -c +"$((log_pos + 1))" "$OCD_LOG")
  fi
  # program's "Verified OK" is an OpenOCD echo, same as board_probe.
  if printf '%s\n%s\n' "$out" "$log_delta" | rg -q 'Verified OK'; then
    return 0
  fi
  echo "flash via rpc missed Verified OK; falling back to a one-shot program" >&2
  ocd_stop
  if ! openocd -f "$CFG" -c "program $BUILD/motor-ctrl.elf verify reset" \
         -c 'init' -c 'rbp all' -c 'resume' -c 'exit' 2>&1 | rg -q 'Verified OK'; then
    echo "flash failed - probe or target lost" >&2
    exit 1
  fi
  ocd_start || true
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
