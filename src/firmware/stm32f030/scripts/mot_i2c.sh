#!/usr/bin/env bash
# Control motor-ctrl over TARS I2C (CDC → nodebus mot).
#   TARS_CDC=/dev/cu.usbmodemXXXX ./mot_i2c.sh start
#   ./mot_i2c.sh w 80
#   ./mot_i2c.sh iq 300
#   ./mot_i2c.sh stat
#   ./mot_i2c.sh stop
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
CDC="$ROOT/tars_cdc.py"

if python3 -c "import serial" >/dev/null 2>&1; then
  PY=python3
elif /usr/local/anaconda3/bin/python3 -c "import serial" >/dev/null 2>&1; then
  PY=/usr/local/anaconda3/bin/python3
else
  echo "need pyserial (python3 or /usr/local/anaconda3/bin/python3)" >&2
  exit 2
fi

cdc() { "$PY" "$CDC" "$@"; }

cmd="${1:-stat}"
shift || true

case "$cmd" in
  start)
    kind="${1:-speed}"
    dir="${2:-0}"
    iq="${3:-1200}"
    w="${4:-40}"
    cdc "nodebus mot start $kind $dir $iq $w"
    for _i in 1 2 3 4 5 6 7 8; do
      sleep 0.4
      cdc "nodebus mot stat" || true
    done
    ;;
  stop)
    cdc "nodebus mot stop"
    ;;
  clr|clear)
    cdc "nodebus wr 0x50 0x91 0x02"
    cdc "nodebus alert clear"
    ;;
  w)
    cdc "nodebus mot w ${1:-40}"
    ;;
  iq)
    cdc "nodebus mot iq ${1:-1200}"
    ;;
  dir)
    cdc "nodebus mot dir ${1:-0}"
    ;;
  stat|status)
    cdc "nodebus mot stat"
    ;;
  scan)
    cdc "nodebus scan"
    cdc "nodebus list"
    ;;
  health)
    cdc "nodebus health ${1:-0x50}"
    ;;
  ping)
    cdc "nodebus ping ${1:-0x50}"
    ;;
  caps)
    cdc "nodebus caps ${1:-0x50}"
    ;;
  alert)
    if [[ $# -eq 0 ]]; then
      cdc "nodebus alert"
    else
      cdc "nodebus alert $*"
    fi
    ;;
  disturb)
    exec "$PY" "$ROOT/mot_bench.py" disturb "$@"
    ;;
  thermal)
    exec "$PY" "$ROOT/mot_bench.py" thermal "$@"
    ;;
  *)
    echo "usage: $0 start [speed|torque] [dir] [iq_mA] [w_elec/s]"
    echo "       $0 stop | clr | w <elec/s> | iq <mA> | dir 0|1 | stat | scan"
    echo "       $0 health [addr] | ping [addr] | caps [addr]"
    echo "       $0 alert | alert test | alert clear"
    echo "       $0 disturb [--start] [--dir 0] [--w 80]   # t=3s 拧 3→4，15s 打点"
    echo "       $0 thermal [--minutes 20] [--period 1.5]"
    exit 2
    ;;
esac
