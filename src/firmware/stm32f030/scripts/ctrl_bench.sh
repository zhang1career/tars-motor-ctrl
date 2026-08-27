#!/usr/bin/env bash
# Measure the cycle cost of the control-ISR building blocks on the target.
#
# Runs from a reset, not from a gdb call: docs/roadmap.md 9.5 rules the latter
# out for measurements. Motor stays off throughout.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/bench_common.sh"

require_dap
motor_off

rm -rf "$BUILD"
cmake -S "$BENCH_ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
  -DMOTOR_AUTO_START=OFF -DMOTOR_START_BENCH=ON >/dev/null
cmake --build "$BUILD" >/dev/null
flash_elf
cpu_alive

ADDR=$(sym_addr g_ctrl_bench)
read_words "$ADDR" 12 | tr '\n' ' ' | python3 -c "
import sys

w = [int(x, 16) for x in sys.stdin.read().split()]
tag, overhead, nop, budget = w[0], w[1], w[2], w[3]
med, mx = w[4:8], w[8:12]

if tag != 0xC7B00001:
    sys.exit(f'bench did not complete: tag=0x{tag:08X}')

labels = ['angle update (steady)', 'angle update (edge, has divide)',
          'read 3 shunts + sum', 'trace push']

print(f'counter overhead {overhead} cycles')
print(f'100 straight NOPs reads {nop} cycles; the excess over 100 is the flash '
      f'wait-state penalty')
print(f'tick budget {budget} cycles at 20 kHz on 48 MHz')
print()
print('case                              median    max    % of tick')
for i, label in enumerate(labels):
    print(f'  {label:32s} {med[i]:5d}  {mx[i]:5d}    {med[i]/budget*100:5.1f}%')

# The edge path replaces the steady one on the ticks where a hall edge lands, so
# adding both overstates the cost. At 24 electrical rev/s edges arrive 144 times
# a second, i.e. on 0.7% of ticks.
typical = med[0] + med[2] + med[3]
worst = med[1] + med[2] + med[3]
print()
print(f'  per tick, no edge              {typical:5d}          {typical/budget*100:5.1f}%')
print(f'  per tick, on a hall edge       {worst:5d}          {worst/budget*100:5.1f}%')
print()
print('headroom for the FOC step (worst-case tick):')
for hz, div in ((20000, 1), (10000, 2), (5000, 4)):
    print(f'  {hz//1000:2d} kHz: {div*budget-worst:5d} cycles')
"
