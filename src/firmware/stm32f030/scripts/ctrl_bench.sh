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
read_words "$ADDR" 16 | tr '\n' ' ' | python3 -c "
import sys

w = [int(x, 16) for x in sys.stdin.read().split()]
tag, overhead, nop, budget = w[0], w[1], w[2], w[3]
med, mx = w[4:10], w[10:16]

if tag != 0xC7B00001:
    sys.exit(f'bench did not complete: tag=0x{tag:08X}')

labels = ['angle update (steady)', 'angle update (edge, has divide)',
          'read 3 shunts + sum', 'trace push', 'FOC step (full, closed loop)']

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
typical = med[0] + med[2] + med[3] + med[4]
worst = med[1] + med[2] + med[3] + med[4]
print()
print(f'  per tick, no edge              {typical:5d}          {typical/budget*100:5.1f}%')
print(f'  per tick, on a hall edge       {worst:5d}          {worst/budget*100:5.1f}%')
print()
# The FOC step itself is decimated, so at MOTOR_FOC_DECIM=2 it lands on every
# other tick; the worst tick is one that carries both a hall edge and a FOC step.
print(f'soft float costs {med[5]/100:.0f} cycles per multiply on this part '
      f'(a few of that is loop overhead)')
print(f'the FOC step makes 110 soft-float calls, so {med[4]} cycles is consistent')
print()
print('margin at the ISR rate (20 kHz ticks):')
for name, cost in (('no edge, no FOC step', med[0] + med[2] + med[3]),
                   ('FOC step, no edge', typical),
                   ('FOC step and hall edge', worst)):
    print(f'  {name:24s} {cost:5d} cycles, {cost/budget*100:5.1f}% of 2400, '
          f'margin {budget-cost:+5d}')
"
