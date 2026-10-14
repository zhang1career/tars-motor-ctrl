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
CMP=$(sym_addr g_foc_fx_cmp)
{
  read_words "$ADDR" 18
  echo '---'
  read_words "$CMP" 50
} | python3 -c "
import sys

raw = sys.stdin.read().split('---')
w = [int(x, 16) for x in raw[0].split()]
c = [int(x, 16) for x in raw[1].split()]

def s32(u):
    return u - (1 << 32) if u >= (1 << 31) else u

tag, overhead, nop, budget = w[0], w[1], w[2], w[3]
med, mx = w[4:11], w[11:18]

if tag != 0xC7B00001:
    sys.exit(f'bench did not complete: tag=0x{tag:08X}')

labels = ['angle update (steady)', 'angle update (edge, has divide)',
          'read 3 shunts + sum', 'trace push', 'FOC step (float, closed loop)',
          '100 float muls', 'FOC fx step (closed loop)']

print(f'counter overhead {overhead} cycles')
print(f'100 straight NOPs reads {nop} cycles; the excess over 100 is the flash '
      f'wait-state penalty')
print(f'tick budget {budget} cycles at 20 kHz on 48 MHz')
print()
print('case                              median    max    % of tick')
for i, label in enumerate(labels):
    print(f'  {label:32s} {med[i]:5d}  {mx[i]:5d}    {med[i]/budget*100:5.1f}%')

# ISR totals use the fixed-point step. The edge path replaces the steady one
# on the ticks where a hall edge lands, so adding both overstates the cost.
typical = med[0] + med[2] + med[3] + med[6]
worst = med[1] + med[2] + med[3] + med[6]
print()
print(f'  per tick, no edge              {typical:5d}          {typical/budget*100:5.1f}%')
print(f'  per tick, on a hall edge       {worst:5d}          {worst/budget*100:5.1f}%')
print()
print(f'soft float costs {med[5]/100:.0f} cycles per multiply on this part '
      f'(a few of that is loop overhead)')
print(f'the float FOC step makes 110 soft-float calls, so {med[4]} cycles is consistent')
print(f'fixed-point FOC step median {med[6]} / max {mx[6]}, budget 1837')
if med[6] >= 1837:
    sys.exit(f'FOC fx step median {med[6]} >= 1837')

print()
print('margin at the ISR rate (20 kHz ticks, fixed-point FOC):')
for name, cost in (('no edge, no FOC step', med[0] + med[2] + med[3]),
                   ('FOC fx, no edge', typical),
                   ('FOC fx and hall edge', worst)):
    print(f'  {name:24s} {cost:5d} cycles, {cost/budget*100:5.1f}% of 2400, '
          f'margin {budget-cost:+5d}')

if len(c) < 50:
    sys.exit(f'compare dump short: {len(c)} words')
if c[0] != 0xF0C0C0DE:
    sys.exit(f'compare did not complete: tag=0x{c[0]:08X}')
nfail = c[1]
d_id = [s32(x) for x in c[2:10]]
d_iq = [s32(x) for x in c[10:18]]
id_f = [s32(x) for x in c[18:26]]
iq_f = [s32(x) for x in c[26:34]]
id_x = [s32(x) for x in c[34:42]]
iq_x = [s32(x) for x in c[42:50]]
print()
print('float vs fixed-point id/iq (mA), limit |d| <= 4:')
print('  i   id_f   iq_f   id_x   iq_x    d_id   d_iq')
for i in range(8):
    print(f'  {i} {id_f[i]:6d} {iq_f[i]:6d} {id_x[i]:6d} {iq_x[i]:6d}  {d_id[i]:+6d} {d_iq[i]:+6d}')
print(f'nfail {nfail}')
if nfail != 0:
    sys.exit(f'compare failed {nfail} of 8 cases')
"
