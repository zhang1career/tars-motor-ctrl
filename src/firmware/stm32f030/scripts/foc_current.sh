#!/usr/bin/env bash
# Stage E: hall6 → Hi-Z → DPWMMIN/midrail → id → gold or DT → optional interp.
# Compare only adjacent beeps. Do not stack topology onto the voltage step.
# Do not raise vq and open interp on the same beep. 3.6 V after lock is OK.
# No IMAX raise. Empty-load second-scale mean iq is a voltage-limit point,
# not a current-loop score. Score the loop from HO_STEP / STEP transients.
# REPEAT=N reruns the whole DIRS loop (scatter on 2.4 / 3.6).
# SPD_S / W_REF: after the 2.4 V baseline, enable the speed PI first.
# VQMAX_UV then raises the ceiling under a closed speed loop. Do not
# sit at full iq on a raised ceiling, then turn the PI on.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/bench_common.sh"

FAST="${FAST:-1}"
SETTLE_S="${SETTLE_S:-4}"
if [[ "$FAST" == 1 ]]; then
  HANDOVER_HOLD_S="${HANDOVER_HOLD_S:-2}"
  BASE_S="${BASE_S:-2}"
else
  HANDOVER_HOLD_S="${HANDOVER_HOLD_S:-8}"
fi
HIZ_S="${HIZ_S:-6}"
LEG3_S="${LEG3_S:-6}"
ID_S="${ID_S:-6}"
GOLD_S="${GOLD_S:-6}"
ALT_S="${ALT_S:-6}"
STEP_S="${STEP_S:-0}"
STEP_MS="${STEP_MS:-0}"
HO_STEP="${HO_STEP:-0}"
DT_S="${DT_S:-0}"
INTERP_S="${INTERP_S:-0}"
BASE_INTERP="${BASE_INTERP:-0}"
BASE_VD24="${BASE_VD24:-0}"
VQ36_S="${VQ36_S:-0}"
SPD_S="${SPD_S:-0}"
W_REF="${W_REF:-40}"
# Space-separated |elec/s| after spd_on. First write also enables the
# PI; later writes change only w_ref (no re-seed). Empty keeps W_REF.
W_REFS="${W_REFS:-}"
# After the last w_ref hold: keep spd on and watch. Turn the brake
# dial during this window to score disturbance recovery.
DISTURB_S="${DISTURB_S:-0}"
# Same-run load step without a timed beep window:
#   HOLD_AFTER=1  score, leave CURRENT+spd on, skip reflash
#   SCORE_ONLY=1  no kick/handover; acc+wrap then reflash
HOLD_AFTER="${HOLD_AFTER:-0}"
SCORE_ONLY="${SCORE_ONLY:-0}"
# 1 = skip Vdc/√3 circle, hexagon-clamp iPark, allow VQMAX up to 7.7 V.
# Hexagon sat rewinds current-loop Ki to the delivered vd/vq.
OVERMOD="${OVERMOD:-0}"
SLEW_S="${SLEW_S:-0}"
SLEW_Q16="${SLEW_Q16:-900}"
# 1 = write SLEW_Q16 at spd_on (before the first w_ref hold).
APPLY_SLEW="${APPLY_SLEW:-0}"
# 0/1 = overwrite g_motor_pwm_dt_on at spd_on. Empty keeps handover (1).
# High speed has no low-side window; DT sign follows garbage current.
DT_ON="${DT_ON:-}"
# 1 = skip dump_foc_v (the slow end-of-hold foc_v refill). wrap/acc stay.
SKIP_TRACE="${SKIP_TRACE:-0}"
PARKOFF_S="${PARKOFF_S:-0}"
PARKOFF_Q16="${PARKOFF_Q16:-0}"
VD24_S="${VD24_S:-0}"
WATCH_S="${WATCH_S:-0}"
DIRS="${DIRS:-0 1}"
REPEAT="${REPEAT:-1}"
RETRIES="${RETRIES:-3}"
OUT="${OUT:-$BENCH_ROOT/../../../models/captured}"
IQ_LSB="${IQ_LSB:-186}"   # MOTOR_FOC_IQ_LSB; 186 = 300 mA / 1.617 mA
# Bus abort. Same threshold everywhere (handover, vqmax, foc_rotating).
# Tick 5 / 140 elec/s is 0.44 A of real mechanical power, not a stall.
# 0.40 swallowed that on foc_rotating (|| true) and tripped apply_vqmax.
# Stall is still elec<1 && i>0.15 and w_meas<25. Phase OCP stays 1484 LSB.
BUS_ABORT_A="${BUS_ABORT_A:-0.50}"
# On a w_ref step (not the first enable), arm a foc_ang oneshot at this
# decimation. 200 at 20 kHz is 10 ms/sample, 2.56 s window.
# ±180° Q15 unwrap aliases above 50 elec/s at this dt — print_spd_step
# must add full turns against a running speed estimate.
SPD_STEP_DECIM="${SPD_STEP_DECIM:-200}"
# vq ceiling (µV). ISR clamps 1.2e6..7.7e6. Empty-load:
# 3600000 / 5000000 / 6200000. OVERMOD=1 + 7700000 is six-step.
# With SPD_S=0 this rises after the 2.4 V baseline. With SPD_S>0
# it rises after spd_on so the raised-ceiling window is not a
# free-run.
VQMAX_UV="${VQMAX_UV:-}"
# Space-separated signed id_ref LSB, applied after VQMAX. 62 ≈ 100 mA.
# Empty skips. Handover leaves id_ref=0 so the 2.4 V baseline is unchanged.
ID_REFS="${ID_REFS:-}"
ID_REF_S="${ID_REF_S:-2}"
ID_LSB="${ID_LSB:-}"
# Space-separated park_off Q16 (65536 = 360 deg). Empty keeps the firmware
# default. Applied after VQMAX; each offset then runs ID_REFS (or id=0).
PARK_OFFS="${PARK_OFFS:-}"
PARK_OFF_S="${PARK_OFF_S:-1}"
# Signed id_ref step for Ld. Closed loop with pole-zero cancellation is
# first order, tau = Ld/Kp, so Ld = Kp * tau. Kp = 3.297 V/A.
ID_STEP_LSB="${ID_STEP_LSB:-}"
# Timer counts before the centre-aligned peak at which the ADC sequence
# starts. Default 72 = 1.5 us (fits the 6.2 V low-side window). Sweep
# to tell a sampling-instant error from a shunt/amp error.
ADC_LEAD="${ADC_LEAD:-}"

reflash_safe() {
  motor_off
  rm -rf "$BUILD"
  cmake -S "$BENCH_ROOT" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DMOTOR_AUTO_START=OFF -DMOTOR_START_BENCH=OFF \
    -DMOTOR_START_FOC_OBSERVE=OFF -DMOTOR_START_HALL6=OFF >/dev/null
  cmake --build "$BUILD" >/dev/null
  flash_elf
}

cpu_resume() {
  ocd -c 'init' -c 'rbp all' -c 'resume' -c 'exit' >/dev/null
}

require_dap
report_and_clear_stale_bif
trap 'motor_off; host_alarm; reflash_safe' EXIT
if [[ "${SCORE_ONLY}" == 1 ]]; then
  echo "== score only — motor should already be holding =="
  cpu_resume
  if ! cpu_alive; then
    echo "SCORE_ONLY: CPU not running after resume" >&2
    exit 1
  fi
else
  echo "== baseline =="
  "$ROOT/scripts/board_check.sh"
fi

# Fast GPIO burst. 50 ms samples alias at ~29 elec/s and cannot tell
# dither from rotation. 16 reads at 2 ms span about one electrical turn.
hall6_gpio_walk() {
  ocd -c 'init' -c '
    for {set i 0} {$i < 16} {incr i} {
      set v [mrw 0x48000410]
      echo [format "HALL %d" [expr {($v >> 3) & 7}]]
      sleep 2
    }
  ' -c 'exit' | rg '^HALL ' | awk '{print $2}'
}

spinning() {
  local snap a b elec i halls t0 t1
  snap=$(sym_addr s_hall6_snap)
  t0=$(python3 -c "import time; print(time.time())")
  a=$(read_words "$snap" 4 | awk 'NR==4')
  sleep 1
  b=$(read_words "$snap" 4 | awk 'NR==4')
  t1=$(python3 -c "import time; print(time.time())")
  elec=$(python3 -c "print((int('$b',16)-int('$a',16))/6/(float('$t1')-float('$t0')))")
  i=$(read_i)
  # Locked rotor + current: drop PWM now. The GPIO walk is 32 ms more
  # of the same, and BUS_ABORT_A only runs after handover.
  if python3 -c "
elec=float('$elec'); i=float('$i')
raise SystemExit(2 if elec < 1.0 and i > 0.15 else 0)
"; then
    :
  else
    echo "  hall6 STALL ${elec} elec/s  bus ${i} A — outputs off"
    motor_off
    return 1
  fi
  halls=$(hall6_gpio_walk)
  HALL6_ELEC="$elec"
  python3 -c "
elec=float('$elec'); i=float('$i')
hs=[int(x) for x in '''$halls'''.split()]
codes=sorted(set(hs))
print(f'  hall6 edges {elec:.1f} elec/s  bus {i:.3f} A  gpio {hs} unique {codes}')
raise SystemExit(0 if elec > 1.0 and i > 0.02 and len(codes) >= 5 else 1)
"
}

# After ReleasePwm, hall6 hall_changes freeze. Do not fail on that meter.
# Rotation is decided by the on-chip FOC wrap trace after the watch window.
# Re-arm wrap so a later dump is not the leftover STEP oneshot (12.8 ms,
# 2–3 halls → DITHER). decim 20 at 20 kHz is 256 ms.
arm_wrap() {
  local src="$1" decim="${2:-20}"
  ocd -c 'init' \
      -c "mwb $((TR + 20)) 0" \
      -c "mwh $((TR + 8)) 0" \
      -c "mwh $((TR + 10)) $decim" \
      -c "mwh $((TR + 12)) 0" \
      -c "mww $((TR + 16)) 0" \
      -c "mwb $((TR + 21)) 0" \
      -c "mwb $((TR + 22)) $src" \
      -c "mwb $((TR + 20)) 2" \
      -c 'exit' >/dev/null
  sleep 0.35
}

# One-shot. Mode 1. Host dumps after the buffer fills (depth*decim/20 kHz).
arm_oneshot() {
  local src="$1" decim="${2:-200}"
  ocd -c 'init' \
      -c "mwb $((TR + 20)) 0" \
      -c "mwh $((TR + 8)) 0" \
      -c "mwh $((TR + 10)) $decim" \
      -c "mwh $((TR + 12)) 0" \
      -c "mww $((TR + 16)) 0" \
      -c "mwb $((TR + 21)) 0" \
      -c "mwb $((TR + 22)) $src" \
      -c "mwb $((TR + 20)) 1" \
      -c 'exit' >/dev/null
}

print_spd_step() {
  local csv="$1" w0="$2" w1="$3"
  python3 -c "
import csv, os
p='$csv'
if not p or not os.path.isfile(p):
    print('  spd_step: no CSV')
    raise SystemExit(0)
rows=list(csv.DictReader(open(p)))
if len(rows)<20 or 'foc_ip_q15' not in rows[0]:
    print(f'  spd_step: short n={len(rows)}')
    raise SystemExit(0)
DEG=360/32768.0
w0=float('$w0'); w1=float('$w1')
sg=1.0 if w1>=w0 else -1.0
vals=[int(r['foc_ip_q15']) for r in rows]
ts=[float(r['t_us'])/1e6 for r in rows]
d0=[]; dts=[]
for a,b,t0,t1 in zip(vals,vals[1:],ts,ts[1:]):
    d=(b-a)*DEG
    while d>180.0: d-=360.0
    while d<-180.0: d+=360.0
    d0.append(d); dts.append(t1-t0)
# decim 200 is 10 ms: 80 elec/s already travels 288°. ±180° unwrap
# aliases to ~20 elec/s. Viterbi picks the wrap count whose speed
# starts at w0, ends at w1, and does not jump between samples.
K=list(range(-3,4)); n=len(d0); inf=1e18
def spd(i,k):
    return (d0[i]+k*360.0)/360.0/dts[i]
dp=[[inf]*len(K) for _ in range(n)]
prv=[[-1]*len(K) for _ in range(n)]
for ki,k in enumerate(K):
    dp[0][ki]=abs(abs(spd(0,k))-w0)
for i in range(1,n):
    for ki,k in enumerate(K):
        s=spd(i,k)
        for kj in range(len(K)):
            c=dp[i-1][kj]+abs(s-spd(i-1,K[kj]))
            if c<dp[i][ki]:
                dp[i][ki]=c; prv[i][ki]=kj
last=min(range(len(K)), key=lambda ki: dp[-1][ki]+2*abs(abs(spd(n-1,K[ki]))-w1))
ks=[0]*n; ks[-1]=last
for i in range(n-1,0,-1):
    ks[i-1]=prv[i][ks[i]]
acc=0.0; t=[0.0]; th=[0.0]
for i,ki in enumerate(ks):
    acc+=d0[i]+K[ki]*360.0
    t.append(ts[i+1]*1000.0); th.append(acc)
w=[]; j=0
for i in range(len(t)):
    while t[i]-t[j]>100.0 and i>j:
        j+=1
    dt=(t[i]-t[j])/1000.0
    w.append(abs(th[i]-th[j])/360.0/dt if dt>0.04 else float('nan'))
lo=w0+0.1*(w1-w0); hi=w0+0.9*(w1-w0)
def cross(tgt, s):
    for i,v in enumerate(w):
        if v==v and s*(v-tgt)>=0:
            return i
    return None
i10=cross(lo, sg); i90=cross(hi, sg)
valid=[x for x in w if x==x]
print(f'  spd_step csv={p} n={len(w)}  {int(w0)}→{int(w1)} elec/s')
w0v=w[0] if w[0]==w[0] else float('nan')
print(f'  spd_step window[0]={w0v:.1f}  window[-1]={w[-1]:.1f}  peak={max(valid):.1f}')
if i10 is not None and i90 is not None and t[i90]>=t[i10]:
    print(f'  spd_step 10-90% {t[i90]-t[i10]:.0f} ms  (t10={t[i10]:.0f} t90={t[i90]:.0f})')
    over=max(valid) if sg>0 else min(valid)
    print(f'  spd_step extreme {over:.1f} elec/s  overshoot {sg*(over-w1):+.1f}')
else:
    print('  spd_step no 10-90% cross — window shorter than the rise or no motion')
"
}

foc_rotating() {
  local ang fx a b fa fb i t0 t1
  ang=$(sym_addr g_motor_angle)
  fx=$(sym_addr g_motor_foc_fx)
  t0=$(python3 -c "import time; print(time.time())")
  a=$(read_words "$ang" 5 | awk 'NR==5')
  fa=$(read_words "$fx" 6 | awk 'NR==6')
  sleep 1
  b=$(read_words "$ang" 5 | awk 'NR==5')
  fb=$(read_words "$fx" 6 | awk 'NR==6')
  t1=$(python3 -c "import time; print(time.time())")
  i=$(read_i)
  python3 -c "
a=int('$a',16); b=int('$b',16)
fa=int('$fa',16); fb=int('$fb',16)
dt=float('$t1')-float('$t0')
ea=(a>>16)&0xFFFF; eb=(b>>16)&0xFFFF
d=(eb-ea)&0xFFFF
th0=fa&0xFFFF; th1=fb&0xFFFF
dth=(th1-th0)&0xFFFF
if dth>32768: dth-=65536
i=float('$i')
h6=float('${HALL6_ELEC:-0}')
elec=d/6/dt if dt>0 else float('nan')
dps=dth*360/65536/dt if dt>0 else float('nan')
print(f'  foc edges {elec:.1f} elec/s  foc theta {dps:+.1f} deg/s  dt={dt:.2f}s  bus {i:.3f} A  (hall6 was {h6:.1f})')
print(f'  angle word4 {a} -> {b}  (low=bad_edges high=edges)')
if i > float('$BUS_ABORT_A'):
    raise SystemExit('bus current too high')
if i < 0.008:
    raise SystemExit('bus current at idle — outputs are off')
"
}

# acc / n mean. Compare |vq|,|vd| to the live ceiling, not 2.28 V.
# T = Kt·iq with Kt = 1.5·p·λ = 1.5·4·0.0063 (mc_params).
print_acc() {
  local acc_words fx_words ceil_uv
  acc_words=$(read_words "$ACC" 9 | tr '\n' ' ')
  fx_words=$(read_words "$FX" 9 | tr '\n' ' ')
  if [[ "${VQMAX_APPLIED:-0}" == 1 && -n "${VQMAX_UV}" ]]; then
    ceil_uv=$VQMAX_UV
  else
    ceil_uv=2400000
  fi
  echo "  acc words $acc_words"
  python3 -c "
w=[int(x,16) for x in '''$acc_words'''.split()]
fx=[int(x,16) for x in '''$fx_words'''.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
n=w[2]
lsb=1.617
kt=0.0378
id_m=s32(w[0])/n*lsb if n else 0
iq_m=s32(w[1])/n*lsb if n else 0
i0_m=s32(w[8])/n*lsb if n and len(w)>8 else 0
t_nm=kt*iq_m/1000.0
vd=s32(fx[2])/1e6
vq=s32(fx[3])/1e6
sat=(fx[8]>>8)&0xFF
ceil=int('$ceil_uv')/1e6
print(f'  acc n={n}  id={id_m:+.1f} mA  iq={iq_m:+.1f} mA  i0={i0_m:+.1f} mA  T={t_nm:+.4f} N·m')
print(f'  volt now vd={vd:+.2f} V  vq={vq:.2f} V  sat={sat}  ceil={ceil:.2f} V')
base='''${SPD_IQ_BASE_MA:-}'''
if base:
    d=iq_m-float(base)
    print(f'  delta vs first-hold  Δiq={d:+.1f} mA  ΔT={kt*d/1000:+.4f} N·m')
t4='''${TICK4_IQ_MA:-}'''
if t4:
    d=iq_m-float(t4)
    print(f'  delta vs tick4       Δiq={d:+.1f} mA  ΔT={kt*d/1000:+.4f} N·m')
acc_f='''${ACC_IQ_FILE:-}'''
if acc_f:
    open(acc_f,'w').write(f'{iq_m:.1f}')
if n < 1000 or abs(iq_m) > 2000 or abs(id_m) > 2000:
    print('  acc GARBAGE — n/sum raced the ISR; ignore this mean, use foc_v')
if abs(i0_m) > 10:
    print('  acc GARBAGE — |i0| > 10 mA common-mode; iq/id means are not a score')
spd=int('''${SPD_ON:-0}''')
if abs(vq) >= 0.95*ceil or abs(vd) >= 0.95*ceil:
    print('  volt now VOLTAGE-LIMITED — empty-load mean iq is not a current-loop score')
elif spd:
    print('  volt now SPEED-HELD — vq off the ceiling; iq may be a current-loop score')
"
}

acc_zero() {
  ocd -c 'init' \
      -c "mww $ACC 0" \
      -c "mww $((ACC + 4)) 0" \
      -c "mww $((ACC + 8)) 0" \
      -c 'exit' >/dev/null
}

score_live_hold() {
  local tag="$1" wr="${2:-${W_REF}}"
  FX=$(sym_addr g_motor_foc_fx)
  ACC=$(sym_addr g_motor_foc_fx_acc)
  TR=$(sym_addr g_motor_trace)
  PROT=$(sym_addr g_motor_pwm_prot)
  VQMAX_APPLIED=1
  acc_zero
  sleep "${SPD_S:-8}"
  PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
  FX_WORDS=$(read_words "$FX" 9 | tr '\n' ' ')
  WMEAS=$(read_words "$(sym_addr g_motor_foc_w_meas_eps)" 1)
  python3 -c "
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
fx=[int(x,16) for x in '$FX_WORDS'.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
latched=(pr[3]>>16)&0xFF
w=s32(int('$WMEAS',16))
print(f'  after {tag} latched={latched}  vq={s32(fx[3])/1e6:.2f} V  vd={s32(fx[2])/1e6:.2f} V  w_meas={w} elec/s')
if latched or pr[0] or pr[1]:
    raise SystemExit('protection during '+'$tag')
if w < 25:
    raise SystemExit('speed hold stalled during '+'$tag')
"
  print_acc
  dump_ccr
  wrap_json="$OUT/foc-w${wr}-${tag}.json"
  mkdir -p "$OUT"
  arm_wrap 5 20
  python3 "$ROOT/scripts/trace_dump.py" --no-plot --json "$wrap_json" \
    -o "$OUT" --elf "$ELF" --cfg "$CFG" >/dev/null 2>&1 || true
  python3 -c "
import json
try:
    p=json.load(open('$wrap_json'))
except Exception:
    raise SystemExit(0)
net=float(p.get('net_theta_deg') or 0)
dt=float(p.get('samples',256))*float(p.get('dt_us',1000))/1e6
wrap=abs(net)/360/dt if dt else float('nan')
print(f'  wrap score {wrap:.1f} elec/s  after {tag} w_ref={int(\"$wr\")} rotating={p.get(\"rotating\")}')
"
  dump_foc_v "$OUT/foc-v-w${wr}-${tag}.json"
}

# Raise the vq software ceiling. Call after handover. When SPD_ON=1 the
# PI is already holding, so the settle is not a full-iq free-run.
apply_vqmax() {
  local settle="${1:-2}"
  if [[ -z "${VQMAX_UV}" || "${VQMAX_APPLIED:-0}" == 1 ]]; then
    return 0
  fi
  VQMAX=$(sym_addr g_motor_foc_vq_max_uv)
  if [[ "${SPD_ON:-0}" == 1 ]]; then
    echo "  VQMAX NOW — ${VQMAX_UV} uV (ceiling after spd_on)"
  else
    echo "  VQMAX NOW — ${VQMAX_UV} uV (ceiling only, interp already locked)"
  fi
  ocd -c 'init' -c "mww $VQMAX $VQMAX_UV" -c 'exit' >/dev/null
  host_beep
  sleep "$settle"
  i=$(read_i)
  PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
  FX_WORDS=$(read_words "$FX" 9 | tr '\n' ' ')
  python3 -c "
i=float('$i')
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
fx=[int(x,16) for x in '$FX_WORDS'.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
print(f'  after vqmax={int(\"$VQMAX_UV\")} latched={latched} sw={sw} bkin={bkin}  vq={s32(fx[3])/1e6:.2f} V  vd={s32(fx[2])/1e6:.2f} V  bus {i:.3f} A')
if latched or sw or bkin:
    raise SystemExit('protection after vqmax')
if i>float('$BUS_ABORT_A'):
    raise SystemExit('bus current too high after vqmax')
if i<0.015:
    raise SystemExit('bus idle after vqmax')
"
  VQMAX_APPLIED=1
}

apply_overmod() {
  if [[ "${OVERMOD}" != 1 ]]; then
    return 0
  fi
  local om
  om=$(sym_addr g_motor_foc_overmod)
  echo "  OVERMOD NOW — hexagon clamp, circle off"
  ocd -c 'init' -c "mwb $om 1" -c 'exit' >/dev/null
}

# Park slew before a high w_ref. Default 600 Q16 ≈ 183 elec/s; 900 ≈ 275.
# APPLY_SLEW=1 writes it at spd_on, not after the hold (SLEW_S is too late).
apply_slew() {
  if [[ "${APPLY_SLEW:-0}" != 1 ]]; then
    return 0
  fi
  local sl
  sl=$(sym_addr g_motor_foc_park_slew_q16)
  python3 -c "
q=int('$SLEW_Q16')
print(f'  SLEW NOW — {q} Q16 = {q*20000/65536:.0f} elec/s (Park rate)')
"
  ocd -c 'init' -c "mww $sl $SLEW_Q16" -c 'exit' >/dev/null
}

apply_dt() {
  if [[ -z "${DT_ON}" ]]; then
    return 0
  fi
  local dt
  dt=$(sym_addr g_motor_pwm_dt_on)
  echo "  DT NOW — dead-time compensate ${DT_ON} (handover left it on)"
  ocd -c 'init' -c "mwb $dt $DT_ON" -c 'exit' >/dev/null
}

dump_ccr() {
  # TIM1 ARR,RCR,CCR1..4,BDTR. lo_win is counts from the peak to the
  # end of the highest-duty phase's low-side (minus DTG). ADC lead is
  # 72 by default; if lo_win < lead the sample is not in that window.
  local words
  words=$(read_words "$TIM1_ARR" 7 | tr '\n' ' ') || {
    echo "  TIM1 CCR read failed" >&2
    return 1
  }
  python3 -c "
w=[int(x,16) for x in '''$words'''.split()]
if len(w) < 7:
    raise SystemExit('TIM1 CCR: short read')
arr, rcr, c1, c2, c3, c4, bdtr = w[:7]
ccrs = [c1 & 0xFFFF, c2 & 0xFFFF, c3 & 0xFFFF]
mx, mn = max(ccrs), min(ccrs)
dtg = bdtr & 0xFF
win = arr - mx - dtg
print(f'  TIM1 ARR={arr}  CCR={ccrs[0]}/{ccrs[1]}/{ccrs[2]}  CCR4={c4 & 0xFFFF}  DTG={dtg}  span={mx-mn}  max_ccr={mx}  lo_win={win} counts ({win/48.0:.2f} us)')
if win < 72:
    print('  TIM1 lo_win < ADC_LEAD 72 — highest-duty low-side window is thin or gone')
"
}

dump_foc_v() {
  local json="$1"
  if [[ "${SKIP_TRACE:-0}" == 1 ]]; then
    echo "  skip foc_v dump (SKIP_TRACE=1)"
    return 0
  fi
  ocd -c 'init' \
      -c "mwb $((TR + 20)) 0" \
      -c "mwh $((TR + 8)) 0" \
      -c "mwh $((TR + 10)) 20" \
      -c "mwh $((TR + 12)) 0" \
      -c "mww $((TR + 16)) 0" \
      -c "mwb $((TR + 21)) 0" \
      -c "mwb $((TR + 22)) 6" \
      -c "mwb $((TR + 20)) 2" \
      -c 'exit' >/dev/null
  sleep 0.35
  python3 "$ROOT/scripts/trace_dump.py" --no-plot --json "$json" \
    -o "$OUT" --elf "$ELF" --cfg "$CFG" || true
}

if [[ "${SCORE_ONLY}" == 1 ]]; then
  if [[ ! -f "$ELF" ]]; then
    echo "SCORE_ONLY needs $ELF from the HOLD_AFTER build" >&2
    exit 1
  fi
  i=$(read_i)
  python3 -c "
i=float('$i')
print(f'  live bus {i:.3f} A')
if i < 0.08:
    raise SystemExit('SCORE_ONLY: bus idle — hold is already off')
if i > float('$BUS_ABORT_A'):
    raise SystemExit('SCORE_ONLY: bus current too high')
"
  score_live_hold "${SCORE_TAG:-tick4}" "${W_REFS:-$W_REF}"
  echo
  echo "== reflashing safe firmware =="
  trap - EXIT
  reflash_safe
  "$ROOT/scripts/board_check.sh"
  exit 0
fi

rep=1
while [[ "$rep" -le "$REPEAT" ]]; do
if [[ "$REPEAT" -gt 1 ]]; then
  echo
  echo "== repeat ${rep}/${REPEAT} =="
fi
for dir in $DIRS; do
  echo
  echo "== foc current iq=${IQ_LSB} LSB direction=${dir} =="
  ok=0
  for try in $(seq 1 "$RETRIES"); do
    rm -rf "$BUILD"
    cmake -S "$BENCH_ROOT" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DMOTOR_AUTO_START=ON -DMOTOR_START_FOC_OBSERVE=ON \
      -DMOTOR_START_BENCH=OFF -DMOTOR_START_HALL6=OFF -DMOTOR_START_DIAG=OFF \
      -DMOTOR_HALL6_RUN_DUTY="${H6_DUTY:-20}" -DMOTOR_HALL6_KICK_DUTY="${H6_DUTY:-20}" \
      -DMOTOR_FOC_IQ_LSB="$IQ_LSB" \
      ${ID_LSB:+-DMOTOR_FOC_ID_LSB="$ID_LSB"} \
      ${ADC_LEAD:+-DMOTOR_ADC_LEAD_COUNTS="$ADC_LEAD"} \
      -DMOTOR_HALL6_PHASE="${PHASE:-3}" -DMOTOR_HALL6_CCW="$dir" >/dev/null
    cmake --build "$BUILD" >/dev/null
    flash_elf
    cpu_alive
    sleep "$SETTLE_S"
    if spinning; then
      ok=1
      break
    fi
    echo "  kick missed (try $try/$RETRIES), retrying"
    motor_off
  done
  if [[ "$ok" != 1 ]]; then
    echo "FAIL: rotor did not start after $RETRIES tries" >&2
    motor_off
    exit 1
  fi

  echo "  hall6 spinning — handover in ${HANDOVER_HOLD_S}s"
  host_beep
  if [[ "$FAST" == 1 ]]; then
    sleep "$HANDOVER_HOLD_S"
  else
    t=0
    while [[ "$t" -lt "$HANDOVER_HOLD_S" ]]; do
      sleep 2
      t=$((t + 2))
      halls=$(ocd -c 'init' -c '
        for {set i 0} {$i < 8} {incr i} {
          set v [mrw 0x48000410]
          echo [format "HALL %d" [expr {($v >> 3) & 7}]]
          sleep 50
        }
      ' -c 'exit' | rg '^HALL ' | awk '{print $2}')
      python3 -c "
hs=[int(x) for x in '''$halls'''.split()]
codes=sorted(set(hs))
edges=sum(1 for a,b in zip(hs,hs[1:]) if a!=b)
print(f'  hall6 t={int(\"$t\")}s  gpio halls {hs}  unique {codes}  edges {edges}')
"
    done
  fi
  echo "  HANDOVER NOW — DPWMMIN +id +DT +interp, vq 2.4 V"

  # Voltage-aligned frame: +iq reinforces hall6's present table,
  # including reverse. Do not send −iq for CCW.
  iq_ref=$IQ_LSB
  REF=$(sym_addr g_motor_foc_iq_ref)
  HO=$(sym_addr g_motor_foc_handover)
  FX=$(sym_addr g_motor_foc_fx)
  TR=$(sym_addr g_motor_trace)
  if [[ "$HO_STEP" == 1 ]]; then
    echo "  HO_STEP — handover first, then oneshot (skip OBSERVE fill)"
    ocd -c 'init' \
        -c "mww $REF $iq_ref" \
        -c "mwb $((TR + 20)) 0" \
        -c "mwh $((TR + 8)) 0" \
        -c "mwh $((TR + 10)) 1" \
        -c "mwh $((TR + 12)) 0" \
        -c "mww $((TR + 16)) 0" \
        -c "mwb $((TR + 21)) 0" \
        -c "mwb $((TR + 22)) 2" \
        -c "mwb $HO 1" \
        -c "mwb $((TR + 20)) 1" \
        -c 'exit' >/dev/null
  else
    ocd -c 'init' -c "mww $REF $iq_ref" -c "mwb $HO 1" -c 'exit' >/dev/null
  fi
  wait_foc_current_beep "$FX" || true
  sleep 0.3
  PROT=$(sym_addr g_motor_pwm_prot)
  FX_WORDS=$(read_words "$FX" 9 | tr '\n' ' ')
  PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
  echo "  foc words $FX_WORDS"
  echo "  prot words $PROT_WORDS"
  python3 -c "
fx=[int(x,16) for x in '$FX_WORDS'.split()]
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
mode=fx[8]&0xFF
latched=(pr[3]>>16)&0xFF
sw=pr[0]
print(f'  after handover mode={mode} (want 2) latched={latched} sw_trips={sw}')
if mode!=2:
    raise SystemExit('FOC did not enter CURRENT — angle.valid was probably 0')
if latched or sw:
    raise SystemExit('protection latched at handover (BKIN or software OCP)')
"

  i=$(read_i)
  python3 -c "
i=float('$i')
print(f'  bus after handover {i:.3f} A')
if i>float('$BUS_ABORT_A'):
    raise SystemExit('bus current too high after handover')
if i<0.015:
    raise SystemExit('bus current collapsed after handover')
"

  if [[ "$HO_STEP" == 1 ]]; then
    mkdir -p "$OUT"
    python3 "$ROOT/scripts/trace_dump.py" --no-plot \
      --json "$OUT/foc-ho_step-dir${dir}.json" \
      -o "$OUT" --elf "$ELF" --cfg "$CFG" || true
    latest=$(ls -t "$OUT"/trace-foc-*.csv 2>/dev/null | head -1)
    python3 -c "
import csv
p='$latest'
if not p:
    print('  ho_step: no FOC CSV')
    raise SystemExit(0)
lsb=1.617
t=[]; iq=[]; halls=set()
with open(p) as f:
    r=csv.DictReader(f)
    for row in r:
        t.append(float(row['t_us']))
        iq.append(int(row['iq_lsb'])*lsb)
        halls.add(int(row['hall_raw']))
if len(iq)<20:
    print(f'  ho_step: short n={len(iq)}')
    raise SystemExit(0)
peak=max(iq)
mean_last=sum(iq[-60:])/60.0
hit=next((i for i,v in enumerate(iq) if 255.0<=v<=345.0), None)
print(f'  ho_step csv={p}')
print(f'  ho_step n={len(iq)}  halls={sorted(halls)}  iq0={iq[0]:+.1f} mA  peak={peak:+.1f} mA  last3ms_mean={mean_last:+.1f} mA')
if hit is None:
    print('  ho_step never entered 255..345 mA')
else:
    end=hit
    while end<len(iq) and iq[end]>=255.0:
        end+=1
    hold_ms=(t[end-1]-t[hit])/1000.0
    t_hit=t[hit]/1000.0
    print(f'  ho_step hit 255mA at {t_hit:.2f} ms  hold {hold_ms:.2f} ms')
"
  fi

  gpio_watch() {
    local tag="$1" dur="$2" t=0 halls i pw
    while [[ "$t" -lt "$dur" ]]; do
      sleep 2
      t=$((t + 2))
      halls=$(ocd -c 'init' -c '
        for {set i 0} {$i < 8} {incr i} {
          set v [mrw 0x48000410]
          echo [format "HALL %d" [expr {($v >> 3) & 7}]]
          sleep 50
        }
      ' -c 'exit' | rg '^HALL ' | awk '{print $2}')
      i=$(read_i)
      pw=$(read_words "$PROT" 4 | tr '\n' ' ')
      python3 -c "
hs=[int(x) for x in '''$halls'''.split()]
codes=sorted(set(hs))
edges=sum(1 for a,b in zip(hs,hs[1:]) if a!=b)
pr=[int(x,16) for x in '''$pw'''.split() if x]
latched=(pr[3]>>16)&0xFF if len(pr)>=4 else 0
sw=pr[0] if pr else 0
bkin=pr[1] if len(pr)>1 else 0
print(f'  ${tag} t={int(\"$t\")}s  gpio halls {hs}  unique {codes}  edges {edges}  bus {float(\"$i\"):.3f} A')
if latched or sw or bkin:
    raise SystemExit(f'protection during ${tag} latched={latched} sw={sw} bkin={bkin}')
"
    done
  }

  ACC=$(sym_addr g_motor_foc_fx_acc)
  TR=$(sym_addr g_motor_trace)

  if [[ "$FAST" == 1 ]]; then
    LEG3=$(sym_addr g_motor_foc_leg3)
    IDON=$(sym_addr g_motor_foc_id_on)
    DT=$(sym_addr g_motor_pwm_dt_on)
    TOPO="${TOPO:-dpwm}"
    if [[ "$TOPO" == midrail ]]; then
      leg=2
      tag=midrail
    else
      leg=1
      tag=dpwm
    fi
    echo "  handover is the baseline — DPWMMIN +id +DT +interp, vq 2.4 V"
    ocd -c 'init' \
        -c "mww $((ACC + 8)) 0" \
        -c "mwb $((TR + 20)) 0" \
        -c "mwh $((TR + 8)) 0" \
        -c "mwh $((TR + 10)) 20" \
        -c "mwh $((TR + 12)) 0" \
        -c "mww $((TR + 16)) 0" \
        -c "mwb $((TR + 21)) 0" \
        -c "mwb $((TR + 22)) 5" \
        -c "mwb $((TR + 20)) 2" \
        -c 'exit' >/dev/null
    sleep "$BASE_S"
    i=$(read_i)
    PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
    python3 -c "
i=float('$i')
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
print(f'  after base bus {i:.3f} A  latched={latched} sw={sw} bkin={bkin}')
if latched or sw or bkin:
    raise SystemExit('protection at base')
if i<0.015:
    raise SystemExit('bus idle at base')
"
    VQMAX_APPLIED=0
    SPD_ON=0
    # Speed loop first: do not raise the ceiling onto a full-iq free-run.
    if [[ -n "${VQMAX_UV}" && "${SPD_S}" -eq 0 ]]; then
      apply_vqmax 2
    fi
    # With SPD_S, id_ref is applied after the speed hold (field weakening).
    if [[ "${SPD_S}" -eq 0 && ( -n "${ID_REFS}" || -n "${PARK_OFFS}" ) ]]; then
      IDREF=$(sym_addr g_motor_foc_id_ref)
      POFF=$(sym_addr g_motor_foc_park_off_q16)
      park_list="${PARK_OFFS:-keep}"
      id_list="${ID_REFS:-0}"
      for po in $park_list; do
        if [[ "$po" != keep ]]; then
          po_u=$(python3 -c "print(int('$po') & 0xFFFFFFFF)")
          python3 -c "print(f'  PARK OFFSET NOW — {int(\"$po\")} Q16 = {int(\"$po\")*360/65536:+.1f} deg')"
          ocd -c 'init' \
              -c "mww $POFF $po_u" \
              -c "mww $IDREF 0" \
              -c 'exit' >/dev/null
          host_beep
          sleep "$PARK_OFF_S"
        fi
        for idr in $id_list; do
          idr_u=$(python3 -c "print(int('$idr') & 0xFFFFFFFF)")
          python3 -c "print(f'  ID REF NOW — {int(\"$idr\")} LSB = {int(\"$idr\")*1.617:+.0f} mA')"
          ocd -c 'init' \
              -c "mww $IDREF $idr_u" \
              -c "mww $ACC 0" \
              -c "mww $((ACC + 4)) 0" \
              -c "mww $((ACC + 8)) 0" \
              -c 'exit' >/dev/null
          host_beep
          sleep "$ID_REF_S"
          i=$(read_i)
          PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
          FX_WORDS=$(read_words "$FX" 9 | tr '\n' ' ')
          python3 -c "
i=float('$i')
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
fx=[int(x,16) for x in '$FX_WORDS'.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
po='$po'
print(f'  after park={po} id_ref={int(\"$idr\")} latched={latched} sw={sw} bkin={bkin}  vq={s32(fx[3])/1e6:.2f} V  vd={s32(fx[2])/1e6:.2f} V  bus {i:.3f} A')
if latched or sw or bkin:
    raise SystemExit('protection after id_ref')
if i>float('$BUS_ABORT_A'):
    raise SystemExit('bus current too high after id_ref')
if i<0.015:
    raise SystemExit('bus idle after id_ref')
"
          print_acc
          mkdir -p "$OUT"
          if [[ "$po" == keep ]]; then
            v_json="$OUT/foc-id${idr}-dir${dir}.json"
            ang_json="$OUT/foc-id${idr}-ang-dir${dir}.json"
          else
            v_json="$OUT/foc-po${po}-id${idr}-dir${dir}.json"
            ang_json="$OUT/foc-po${po}-id${idr}-ang-dir${dir}.json"
          fi
          dump_foc_v "$v_json"
          # Speed per id step: +id vs -id is only comparable with a wrap
          # each, and the end-of-run dump only sees the last setting.
          arm_wrap 5 20
          python3 "$ROOT/scripts/trace_dump.py" --no-plot --json "$ang_json" \
            -o "$OUT" --elf "$ELF" --cfg "$CFG" >/dev/null 2>&1 || true
          python3 -c "
import json
try:
    p=json.load(open('$ang_json'))
except Exception:
    raise SystemExit(0)
net=float(p.get('net_theta_deg') or 0)
dt=float(p.get('samples',256))*float(p.get('dt_us',1000))/1e6
elec=abs(net)/360/dt if dt else float('nan')
print(f'  park=$po id={int(\"$idr\")} wrap {elec:.2f} elec/s  rotating={p.get(\"rotating\")}')
"
        done
        if [[ "$po" != keep ]]; then
          ocd -c 'init' -c "mww $IDREF 0" -c 'exit' >/dev/null
        fi
      done
    fi
    if [[ -n "${ID_STEP_LSB}" ]]; then
      IDREF=$(sym_addr g_motor_foc_id_ref)
      idst_u=$(python3 -c "print(int('$ID_STEP_LSB') & 0xFFFFFFFF)")
      echo "  ID STEP — id_ref 0 -> ${ID_STEP_LSB} LSB, oneshot decim 1 src FOC"
      ocd -c 'init' \
          -c "mww $IDREF 0" \
          -c "mwb $((TR + 20)) 0" \
          -c "mwh $((TR + 8)) 0" \
          -c "mwh $((TR + 10)) 1" \
          -c "mwh $((TR + 12)) 0" \
          -c "mww $((TR + 16)) 0" \
          -c "mwb $((TR + 21)) 0" \
          -c "mwb $((TR + 22)) 2" \
          -c "mwb $((TR + 20)) 1" \
          -c "mww $IDREF $idst_u" \
          -c 'exit' >/dev/null
      sleep 0.05
      mkdir -p "$OUT"
      python3 "$ROOT/scripts/trace_dump.py" --no-plot \
        --json "$OUT/foc-idstep-dir${dir}.json" \
        -o "$OUT" --elf "$ELF" --cfg "$CFG" >/dev/null 2>&1 || true
      ocd -c 'init' -c "mww $IDREF 0" -c 'exit' >/dev/null
      latest=$(ls -t "$OUT"/trace-foc-*.csv 2>/dev/null | head -1)
      python3 -c "
import csv
p='$latest'
if not p:
    print('  id_step: no FOC CSV')
    raise SystemExit(0)
rows=list(csv.DictReader(open(p)))
if len(rows)<20:
    print(f'  id_step: short n={len(rows)}')
    raise SystemExit(0)
t=[float(r['t_us']) for r in rows]
d=[int(r['id_lsb']) for r in rows]
tgt=int('$ID_STEP_LSB')
base=sum(d[:4])/4.0
amp=tgt-base
sg=1 if amp>0 else -1
# first sample that has moved 10% of the way
def cross(frac):
    want=base+amp*frac
    for i,v in enumerate(d):
        if sg*(v-want)>=0:
            return i
    return None
print(f'  id_step csv={p}')
print(f'  id_step n={len(d)} base={base:.0f} LSB target={tgt} LSB')
i63=cross(0.632)
i10=cross(0.10); i90=cross(0.90)
KP=3.297
if i63 is None:
    print('  id_step never reached 63% — no tau')
else:
    tau_us=t[i63]-t[cross(0.02) or 0]
    Ld=KP*tau_us*1e-6
    print(f'  id_step tau {tau_us:.0f} us ({tau_us/50:.1f} ticks)  ->  Ld = Kp*tau = {Ld*1e6:.0f} uH')
    print(f'  id_step  (design assumed 437 uH -> tau 133 us; wc_meas = {1/(2*3.14159*tau_us*1e-6):.0f} Hz)')
if i10 is not None and i90 is not None:
    print(f'  id_step 10-90% rise {t[i90]-t[i10]:.0f} us')
over=max(d) if sg<0 else min(d)
print(f'  id_step extreme {over} LSB  final {sum(d[-20:])/20:.0f} LSB')
"
    fi
    if [[ "${SPD_S}" -gt 0 ]]; then
      SPD=$(sym_addr g_motor_foc_spd_on)
      WREF=$(sym_addr g_motor_foc_w_ref_eps)
      spd_first=1
      spd_i=0
      SPD_IQ_BASE_MA=
      for wr in ${W_REFS:-$W_REF}; do
        spd_i=$((spd_i + 1))
        if [[ "$spd_first" == 1 ]]; then
          echo "  SPEED NOW — hold ${wr} elec/s, iq_ref from speed PI"
          ocd -c 'init' -c "mww $WREF $wr" -c "mwb $SPD 1" -c 'exit' >/dev/null
          spd_first=0
          SPD_ON=1
          host_beep
          sleep 0.3
          apply_overmod
          apply_slew
          apply_dt
          apply_vqmax 1
        else
          echo "  SPEED STEP — w_ref ${wr} elec/s, spd stays on (no re-seed)"
          echo "  SPD STEP TRACE — foc_ang oneshot decim ${SPD_STEP_DECIM}"
          arm_oneshot 5 "$SPD_STEP_DECIM"
          ocd -c 'init' -c "mww $WREF $wr" -c 'exit' >/dev/null
          host_beep
        fi
        acc_zero
        mkdir -p "$OUT"
        if [[ "$spd_i" -gt 1 ]]; then
          # gpio_watch steps in 2 s integers. Dump as soon as the oneshot
          # is full — do not wait SPD_S, and do not pick ls -t leftovers.
          fill_s=$(python3 -c "import math; t=256*int('$SPD_STEP_DECIM')/20000+0.3; print(int(math.ceil(t/2.0)*2))")
          gpio_watch "spd${wr}step" "$fill_s"
          step_json="$OUT/foc-w${prev_wr}-to-${wr}-step-rep${rep}-dir${dir}.json"
          step_csv="$OUT/foc-w${prev_wr}-to-${wr}-step-rep${rep}-dir${dir}.csv"
          python3 "$ROOT/scripts/trace_dump.py" --no-plot --json "$step_json" \
            --csv "$step_csv" -o "$OUT" --elf "$ELF" --cfg "$CFG" || true
          print_spd_step "$step_csv" "$prev_wr" "$wr"
          remain=$(python3 -c "print(max(0, int('$SPD_S')-int('$fill_s')))")
          if [[ "$remain" -gt 0 ]]; then
            acc_zero
            gpio_watch "spd${wr}" "$remain"
          fi
        else
          gpio_watch "spd${wr}" "$SPD_S"
        fi
        PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
        FX_WORDS=$(read_words "$FX" 9 | tr '\n' ' ')
        WMEAS=$(read_words "$(sym_addr g_motor_foc_w_meas_eps)" 1)
        IQREF=$(read_words "$(sym_addr g_motor_foc_iq_ref)" 1)
        python3 -c "
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
fx=[int(x,16) for x in '$FX_WORDS'.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
w=s32(int('$WMEAS',16))
iqref=s32(int('$IQREF',16))
kt=0.0378
t_nm=kt*iqref*1.617/1000.0
print(f'  after w_ref={int(\"$wr\")} latched={latched} sw={sw} bkin={bkin}  vq={s32(fx[3])/1e6:.2f} V  vd={s32(fx[2])/1e6:.2f} V  w_meas={w} elec/s  iq_ref={iqref} LSB  T_ref={t_nm:+.4f} N·m')
print('  w_meas is the loop meter — wrap (foc_ang net/dt) is the score')
if latched or sw or bkin:
    raise SystemExit('protection after spd')
if w < 25:
    raise SystemExit('speed hold stalled')
"
        case "$wr" in
          80) TICK4_IQ_MA=400 ;;
          130) TICK4_IQ_MA=457 ;;
          140) TICK4_IQ_MA=452 ;;
          *) TICK4_IQ_MA= ;;
        esac
        export TICK4_IQ_MA
        ACC_IQ_FILE="$OUT/.acc_iq_ma"
        export ACC_IQ_FILE
        print_acc
        dump_ccr
        if [[ "$spd_i" -eq 1 ]]; then
          SPD_IQ_BASE_MA=$(cat "$ACC_IQ_FILE" 2>/dev/null || true)
          export SPD_IQ_BASE_MA
        fi
        wrap_json="$OUT/foc-w${wr}-rep${rep}-s${spd_i}-dir${dir}.json"
        arm_wrap 5 20
        python3 "$ROOT/scripts/trace_dump.py" --no-plot --json "$wrap_json" \
          -o "$OUT" --elf "$ELF" --cfg "$CFG" >/dev/null 2>&1 || true
        python3 -c "
import json
try:
    p=json.load(open('$wrap_json'))
except Exception:
    raise SystemExit(0)
net=float(p.get('net_theta_deg') or 0)
dt=float(p.get('samples',256))*float(p.get('dt_us',1000))/1e6
wrap=abs(net)/360/dt if dt else float('nan')
print(f'  wrap score {wrap:.1f} elec/s  w_ref={int(\"$wr\")} rotating={p.get(\"rotating\")}')
"
        prev_wr=$wr
      done
      if [[ "${DISTURB_S}" -gt 0 ]]; then
        echo "  DISTURB — hold ${prev_wr} elec/s ${DISTURB_S}s. Turn the brake dial to score recovery."
        host_beep
        acc_zero
        gpio_watch "disturb" "$DISTURB_S"
        PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
        FX_WORDS=$(read_words "$FX" 9 | tr '\n' ' ')
        WMEAS=$(read_words "$(sym_addr g_motor_foc_w_meas_eps)" 1)
        python3 -c "
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
fx=[int(x,16) for x in '$FX_WORDS'.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
latched=(pr[3]>>16)&0xFF
w=s32(int('$WMEAS',16))
print(f'  after disturb latched={latched}  vq={s32(fx[3])/1e6:.2f} V  w_meas={w} elec/s')
if latched or pr[0] or pr[1]:
    raise SystemExit('protection after disturb')
if w < 25:
    raise SystemExit('speed hold stalled after disturb')
"
        print_acc
        wrap_json="$OUT/foc-w${prev_wr}-disturb-rep${rep}-dir${dir}.json"
        arm_wrap 5 20
        python3 "$ROOT/scripts/trace_dump.py" --no-plot --json "$wrap_json" \
          -o "$OUT" --elf "$ELF" --cfg "$CFG" >/dev/null 2>&1 || true
        python3 -c "
import json
try:
    p=json.load(open('$wrap_json'))
except Exception:
    raise SystemExit(0)
net=float(p.get('net_theta_deg') or 0)
dt=float(p.get('samples',256))*float(p.get('dt_us',1000))/1e6
wrap=abs(net)/360/dt if dt else float('nan')
print(f'  wrap score {wrap:.1f} elec/s  after disturb w_ref={int(\"$prev_wr\")} rotating={p.get(\"rotating\")}')
"
      fi
      if [[ -n "${ID_REFS}" ]]; then
        IDREF=$(sym_addr g_motor_foc_id_ref)
        id_i=0
        last_wr=$(echo ${W_REFS:-$W_REF} | awk '{print $NF}')
        for idr in $ID_REFS; do
          id_i=$((id_i + 1))
          idr_u=$(python3 -c "print(int('$idr') & 0xFFFFFFFF)")
          python3 -c "print(f'  ID REF NOW — {int(\"$idr\")} LSB = {int(\"$idr\")*1.617:+.0f} mA  (spd stays on, w_ref={int(\"$last_wr\")})')"
          ocd -c 'init' -c "mww $IDREF $idr_u" -c 'exit' >/dev/null
          host_beep
          acc_zero
          gpio_watch "id${idr}" "$ID_REF_S"
          i=$(read_i)
          PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
          FX_WORDS=$(read_words "$FX" 9 | tr '\n' ' ')
          WMEAS=$(read_words "$(sym_addr g_motor_foc_w_meas_eps)" 1)
          IQREF=$(read_words "$(sym_addr g_motor_foc_iq_ref)" 1)
          python3 -c "
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
fx=[int(x,16) for x in '$FX_WORDS'.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
w=s32(int('$WMEAS',16))
iqref=s32(int('$IQREF',16))
i=float('$i')
print(f'  after id_ref={int(\"$idr\")} latched={latched} sw={sw} bkin={bkin}  vq={s32(fx[3])/1e6:.2f} V  vd={s32(fx[2])/1e6:.2f} V  w_meas={w} elec/s  iq_ref={iqref} LSB  bus {i:.3f} A')
if latched or sw or bkin:
    raise SystemExit('protection after id_ref')
if w < 25:
    raise SystemExit('speed hold stalled after id_ref')
if i > 0.55:
    raise SystemExit('bus current too high after id_ref')
"
          print_acc
          wrap_json="$OUT/foc-w${last_wr}-id${idr}-rep${rep}-s${id_i}-dir${dir}.json"
          arm_wrap 5 20
          python3 "$ROOT/scripts/trace_dump.py" --no-plot --json "$wrap_json" \
            -o "$OUT" --elf "$ELF" --cfg "$CFG" >/dev/null 2>&1 || true
          python3 -c "
import json
try:
    p=json.load(open('$wrap_json'))
except Exception:
    raise SystemExit(0)
net=float(p.get('net_theta_deg') or 0)
dt=float(p.get('samples',256))*float(p.get('dt_us',1000))/1e6
wrap=abs(net)/360/dt if dt else float('nan')
print(f'  wrap score {wrap:.1f} elec/s  w_ref={int(\"$last_wr\")} id_ref={int(\"$idr\")} rotating={p.get(\"rotating\")}')
"
        done
        ocd -c 'init' -c "mww $IDREF 0" -c 'exit' >/dev/null
      fi
    fi
    if [[ "${INTERP_S}" -gt 0 ]]; then
      IP=$(sym_addr g_motor_foc_interp)
      echo "  INTERP NOW — only new variable, vq still 2.4 V"
      ocd -c 'init' -c "mwb $IP 1" -c 'exit' >/dev/null
      host_beep
      gpio_watch interp "$INTERP_S"
      PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
      FX_WORDS=$(read_words "$FX" 9 | tr '\n' ' ')
      python3 -c "
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
fx=[int(x,16) for x in '$FX_WORDS'.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
print(f'  after interp latched={latched} sw={sw} bkin={bkin}  vq={s32(fx[3])/1e6:.2f} V  vd={s32(fx[2])/1e6:.2f} V')
if latched or sw or bkin:
    raise SystemExit('protection after interp')
"
    fi
    if [[ "${VD24_S}" -gt 0 ]]; then
      i=$(read_i)
      PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
      if python3 -c "
i=float('$i')
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
raise SystemExit(0 if ((pr[3]>>16)&0xFF)==0 and i>=0.015 else 1)
"; then
        VDMAX=$(sym_addr g_motor_foc_vd_max_uv)
        echo "  VD 2.4 NOW — |vd| ceiling only"
        ocd -c 'init' -c "mww $VDMAX 2400000" -c 'exit' >/dev/null
        host_beep
        gpio_watch vd24 "$VD24_S"
        PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
        FX_WORDS=$(read_words "$FX" 9 | tr '\n' ' ')
        python3 -c "
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
fx=[int(x,16) for x in '$FX_WORDS'.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
print(f'  after vd24 latched={latched} sw={sw} bkin={bkin}  vq={s32(fx[3])/1e6:.2f} V  vd={s32(fx[2])/1e6:.2f} V')
if latched or sw or bkin:
    raise SystemExit('protection after vd24')
"
      else
        echo "  skip vd — base/interp did not hold"
      fi
    fi
    if [[ "${STEP_MS}" -gt 0 ]]; then
      echo "  STEP — iq_ref 62 for ${STEP_MS} ms, then 186 (vq stays 2.4 V)"
      host_beep
      ocd -c 'init' \
          -c "mww $REF 62" \
          -c "sleep $STEP_MS" \
          -c "mwb $((TR + 20)) 0" \
          -c "mwh $((TR + 8)) 0" \
          -c "mwh $((TR + 10)) 1" \
          -c "mwh $((TR + 12)) 0" \
          -c "mww $((TR + 16)) 0" \
          -c "mwb $((TR + 21)) 0" \
          -c "mwb $((TR + 22)) 2" \
          -c "mwb $((TR + 20)) 1" \
          -c "mww $REF 186" \
          -c 'exit' >/dev/null
      sleep 0.05
      mkdir -p "$OUT"
      python3 "$ROOT/scripts/trace_dump.py" --no-plot \
        --json "$OUT/foc-step-dir${dir}.json" \
        -o "$OUT" --elf "$ELF" --cfg "$CFG" || true
      latest=$(ls -t "$OUT"/trace-foc-*.csv 2>/dev/null | head -1)
      python3 -c "
import csv
p='$latest'
if not p:
    print('  step: no FOC CSV')
    raise SystemExit(0)
lsb=1.617
t=[]; iq=[]; halls=set()
with open(p) as f:
    r=csv.DictReader(f)
    for row in r:
        t.append(float(row['t_us']))
        iq.append(int(row['iq_lsb'])*lsb)
        halls.add(int(row['hall_raw']))
if len(iq)<20:
    print(f'  step: short n={len(iq)}')
    raise SystemExit(0)
peak=max(iq)
mean_last=sum(iq[-60:])/60.0
hit=next((i for i,v in enumerate(iq) if 255.0<=v<=345.0), None)
print(f'  step csv={p}')
print(f'  step n={len(iq)}  halls={sorted(halls)}  iq0={iq[0]:+.1f} mA  peak={peak:+.1f} mA  last3ms_mean={mean_last:+.1f} mA')
if hit is None:
    print('  step never entered 255..345 mA')
else:
    end=hit
    while end<len(iq) and iq[end]>=255.0:
        end+=1
    hold_ms=(t[end-1]-t[hit])/1000.0
    t_hit=t[hit]/1000.0
    print(f'  step hit 255mA at {t_hit:.2f} ms  hold {hold_ms:.2f} ms')
    print(f'  step note (6-step/interp ripple; do not retune Kp/Ki)')
"
    fi
    if [[ "${VQ36_S}" -gt 0 ]]; then
      i=$(read_i)
      PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
      if python3 -c "
i=float('$i')
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
raise SystemExit(0 if ((pr[3]>>16)&0xFF)==0 and i>=0.015 else 1)
"; then
        VQMAX=$(sym_addr g_motor_foc_vq_max_uv)
        echo "  VQ 3.6 NOW — ceiling only"
        ocd -c 'init' -c "mww $VQMAX 3600000" -c 'exit' >/dev/null
        host_beep
        gpio_watch vq36 "$VQ36_S"
        PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
        FX_WORDS=$(read_words "$FX" 9 | tr '\n' ' ')
        python3 -c "
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
fx=[int(x,16) for x in '$FX_WORDS'.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
print(f'  after vq36 latched={latched} sw={sw} bkin={bkin}  vq={s32(fx[3])/1e6:.2f} V  vd={s32(fx[2])/1e6:.2f} V')
if latched or sw or bkin:
    raise SystemExit('protection after vq36')
"
      else
        echo "  skip vq36 — previous step did not hold"
      fi
    fi
    if [[ "${SLEW_S}" -gt 0 ]]; then
      i=$(read_i)
      PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
      if python3 -c "
i=float('$i')
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
raise SystemExit(0 if ((pr[3]>>16)&0xFF)==0 and i>=0.015 else 1)
"; then
        SLEW=$(sym_addr g_motor_foc_park_slew_q16)
        python3 -c "
q=int('$SLEW_Q16')
print(f'  SLEW NOW — Park rate limit only, {q} Q16 = {q*360/65536:.2f} deg/tick = {q*20000*1.0/65536:.0f} elec/s')
"
        ocd -c 'init' -c "mww $SLEW $SLEW_Q16" -c 'exit' >/dev/null
        host_beep
        gpio_watch slew "$SLEW_S"
        PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
        FX_WORDS=$(read_words "$FX" 9 | tr '\n' ' ')
        python3 -c "
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
fx=[int(x,16) for x in '$FX_WORDS'.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
print(f'  after slew latched={latched} sw={sw} bkin={bkin}  vq={s32(fx[3])/1e6:.2f} V  vd={s32(fx[2])/1e6:.2f} V')
if latched or sw or bkin:
    raise SystemExit('protection after slew')
"
      else
        echo "  skip slew — previous step did not hold"
      fi
    fi
    if [[ "${PARKOFF_S}" -gt 0 ]]; then
      i=$(read_i)
      PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
      if python3 -c "
i=float('$i')
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
raise SystemExit(0 if ((pr[3]>>16)&0xFF)==0 and i>=0.015 else 1)
"; then
        POFF=$(sym_addr g_motor_foc_park_off_q16)
        python3 -c "
q=int('$PARKOFF_Q16')
print(f'  PARK OFFSET NOW — Park angle only, {q} Q16 = {q*360/65536:+.1f} deg')
"
        # mww takes an unsigned 32-bit word; wrap negatives.
        poff_u=$(python3 -c "print(int('$PARKOFF_Q16') & 0xFFFFFFFF)")
        ocd -c 'init' -c "mww $POFF $poff_u" -c 'exit' >/dev/null
        host_beep
        gpio_watch parkoff "$PARKOFF_S"
        PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
        FX_WORDS=$(read_words "$FX" 9 | tr '\n' ' ')
        python3 -c "
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
fx=[int(x,16) for x in '$FX_WORDS'.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
print(f'  after parkoff latched={latched} sw={sw} bkin={bkin}  vq={s32(fx[3])/1e6:.2f} V  vd={s32(fx[2])/1e6:.2f} V')
if latched or sw or bkin:
    raise SystemExit('protection after parkoff')
"
      else
        echo "  skip parkoff — previous step did not hold"
      fi
    fi
    foc_rotating
    ocd -c 'init' -c "mww $((ACC + 8)) 0" -c 'exit' >/dev/null
    sleep 2
    print_acc
    # Always refill wrap/foc_ang before the rotating verdict. STEP leaves a
    # decim-1 oneshot in the buffer that cannot see a full electrical turn.
    arm_wrap 5 20
    json="$OUT/foc-${tag}-dir${dir}.json"
    mkdir -p "$OUT"
    python3 "$ROOT/scripts/trace_dump.py" --no-plot --json "$json" \
      -o "$OUT" --elf "$ELF" --cfg "$CFG" || true
    python3 -c "
import json
p=json.load(open('$json'))
net=float(p.get('net_theta_deg') or 0)
dt=float(p.get('samples',256))*float(p.get('dt_us',1000))/1e6
wrap=abs(net)/360/dt if dt else float('nan')
print(f\"  verdict rotating={p.get('rotating')} halls={p.get('hall_codes')} net={p.get('net_theta_deg')} deg\")
print(f'  wrap score {wrap:.1f} elec/s  (this is the speed, not w_meas)')
if not p.get('rotating'):
    raise SystemExit('not rotating after FOC')
"
    dump_foc_v "$OUT/foc-v-${tag}-dir${dir}.json"
    if [[ "${HOLD_AFTER}" == 1 ]]; then
      cpu_resume
      echo "  HOLD AFTER — CURRENT+spd still on. Turn the dial, then SCORE_ONLY=1."
      trap - EXIT
      exit 0
    fi
    motor_off
    continue
  fi

  echo "  Hi-Z two-phase ${HIZ_S}s — second beep was handover"
  ocd -c 'init' \
      -c "mww $((ACC + 8)) 0" \
      -c "mwb $((TR + 20)) 0" \
      -c "mwh $((TR + 8)) 0" \
      -c "mwh $((TR + 10)) 20" \
      -c "mwh $((TR + 12)) 0" \
      -c "mww $((TR + 16)) 0" \
      -c "mwb $((TR + 21)) 0" \
      -c "mwb $((TR + 22)) 5" \
      -c "mwb $((TR + 20)) 2" \
      -c 'exit' >/dev/null
  gpio_watch hiz "$HIZ_S"
  i=$(read_i)
  PROT=$(sym_addr g_motor_pwm_prot)
  PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
  python3 -c "
i=float('$i')
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
latched=(pr[3]>>16)&0xFF
print(f'  pre-dpwm bus {i:.3f} A  latched={latched} sw={pr[0]} bkin={pr[1]}')
if latched or pr[0] or pr[1]:
    raise SystemExit('protection before DPWMMIN — not switching topology')
if i<0.015:
    raise SystemExit('bus idle before DPWMMIN — Hi-Z is not driving')
"

  if [[ "${SKIP_DPWM:-0}" == 1 ]]; then
    echo "  skip DPWMMIN"
    foc_rotating
    ACC=$(sym_addr g_motor_foc_fx_acc)
    ocd -c 'init' -c "mww $((ACC + 8)) 0" -c 'exit' >/dev/null
    sleep 2
    print_acc
    json="$OUT/foc-hiz-dir${dir}.json"
    mkdir -p "$OUT"
    python3 "$ROOT/scripts/trace_dump.py" --no-plot --json "$json" \
      -o "$OUT" --elf "$ELF" --cfg "$CFG" || true
    python3 -c "
import json
p=json.load(open('$json'))
print(f\"  verdict rotating={p.get('rotating')} halls={p.get('hall_codes')} net={p.get('net_theta_deg')} deg\")
if not p.get('rotating'):
    raise SystemExit('not rotating after FOC')
"
    motor_off
    continue
  fi

  LEG3=$(sym_addr g_motor_foc_leg3)
  TOPO="${TOPO:-dpwm}"
  if [[ "$TOPO" == midrail ]]; then
    echo "  MIDRAIL NOW — third beep, complementary mid-centred, DT still off"
    ocd -c 'init' -c "mwb $LEG3 2" -c 'exit' >/dev/null
    tag=midrail
  else
    echo "  DPWMMIN NOW — third beep, discrete FocTheta, min phase to rail"
    ocd -c 'init' -c "mwb $LEG3 1" -c 'exit' >/dev/null
    tag=dpwm
  fi
  host_beep
  gpio_watch "$tag" "$LEG3_S"
  i=$(read_i)
  PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
  echo "  prot after $tag $PROT_WORDS"
  python3 -c "
i=float('$i')
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
print(f'  after $tag bus {i:.3f} A  latched={latched} sw={sw} bkin={bkin}')
if latched or sw or bkin:
    raise SystemExit('protection after $tag')
"
  if python3 -c "
i=float('$i')
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
latched=(pr[3]>>16)&0xFF
raise SystemExit(0 if (latched==0 and i>=0.015) else 1)
"; then
    IDON=$(sym_addr g_motor_foc_id_on)
    echo "  ID ON NOW — fourth beep, id PI ±1.2 V, stay $tag"
    ocd -c 'init' -c "mwb $IDON 1" -c 'exit' >/dev/null
    host_beep
    gpio_watch idon "$ID_S"
    PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
    echo "  prot after id $PROT_WORDS"
    python3 -c "
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
print(f'  after id latched={latched} sw={sw} bkin={bkin}')
if latched or sw or bkin:
    raise SystemExit('protection after id')
"
    i=$(read_i)
    if python3 -c "
i=float('$i')
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
latched=(pr[3]>>16)&0xFF
raise SystemExit(0 if (latched==0 and i>=0.015) else 1)
"; then
      if [[ "${GOLD_S}" -gt 0 ]]; then
        DT=$(sym_addr g_motor_pwm_dt_on)
        VQMAX=$(sym_addr g_motor_foc_vq_max_uv)
        VDMAX=$(sym_addr g_motor_foc_vd_max_uv)
        echo "  GOLD NOW — fifth beep, DT + vq 3.6 V + |vd| 2.4 V, stay $tag"
        ocd -c 'init' -c "mwb $DT 1" -c "mww $VQMAX 3600000" -c "mww $VDMAX 2400000" -c 'exit' >/dev/null
        host_beep
        gpio_watch gold "$GOLD_S"
        PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
        echo "  prot after gold $PROT_WORDS"
        python3 -c "
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
print(f'  after gold latched={latched} sw={sw} bkin={bkin}')
if latched or sw or bkin:
    raise SystemExit('protection after gold')
"
        i=$(read_i)
        if python3 -c "
i=float('$i')
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
latched=(pr[3]>>16)&0xFF
raise SystemExit(0 if (latched==0 and i>=0.015) else 1)
"; then
          if [[ "${STEP_MS}" -gt 0 ]]; then
            echo "  STEP — iq_ref 62 LSB for ${STEP_MS} ms in-session, then 186 + FOC oneshot"
            ocd -c 'init' \
                -c "mww $REF 62" \
                -c "sleep $STEP_MS" \
                -c "mwb $((TR + 20)) 0" \
                -c "mwh $((TR + 8)) 0" \
                -c "mwh $((TR + 10)) 1" \
                -c "mwh $((TR + 12)) 0" \
                -c "mww $((TR + 16)) 0" \
                -c "mwb $((TR + 21)) 0" \
                -c "mwb $((TR + 22)) 2" \
                -c "mwb $((TR + 20)) 1" \
                -c "mww $REF 186" \
                -c 'exit' >/dev/null
            sleep 0.05
            step_json="$OUT/foc-step-dir${dir}.json"
            mkdir -p "$OUT"
            python3 "$ROOT/scripts/trace_dump.py" --no-plot --json "$step_json" \
              -o "$OUT" --elf "$ELF" --cfg "$CFG" || true
            latest=$(ls -t "$OUT"/trace-foc-*.csv 2>/dev/null | head -1)
            python3 -c "
import csv
p='$latest'
if not p:
    print('  step: no FOC CSV')
    raise SystemExit(0)
lsb=1.617
t=[]; iq=[]; halls=set()
with open(p) as f:
    r=csv.DictReader(f)
    for row in r:
        t.append(float(row['t_us']))
        iq.append(int(row['iq_lsb'])*lsb)
        halls.add(int(row['hall_raw']))
if len(iq)<20:
    print(f'  step: short capture n={len(iq)} file={p}')
    raise SystemExit(0)
peak=max(iq)
mean_last=sum(iq[-60:])/60.0
hit=next((i for i,v in enumerate(iq) if 255.0<=v<=345.0), None)
hold_ms=0.0
if hit is not None:
    end=hit
    while end<len(iq) and iq[end]>=255.0:
        end+=1
    hold_ms=(t[end-1]-t[hit])/1000.0
    t_hit=t[hit]/1000.0
    print(f'  step csv={p}')
    print(f'  step n={len(iq)}  halls={sorted(halls)}  iq0={iq[0]:+.1f} mA  peak={peak:+.1f} mA  last3ms_mean={mean_last:+.1f} mA')
    print(f'  step hit 255mA at {t_hit:.2f} ms  hold {hold_ms:.2f} ms')
    ok=t_hit<=1.2 and hold_ms>=3.0
    print(f'  step verdict {\"PASS\" if ok else \"FAIL\"} (want hit≤1.2 ms and hold≥3 ms; do not retune Kp/Ki)')
else:
    print(f'  step csv={p}')
    print(f'  step n={len(iq)}  halls={sorted(halls)}  iq0={iq[0]:+.1f} mA  peak={peak:+.1f} mA  last3ms_mean={mean_last:+.1f} mA')
    print('  step never entered 255..345 mA')
    print('  step verdict FAIL (do not retune Kp/Ki)')
"
          elif [[ "${STEP_S}" -gt 0 ]]; then
            echo "  STEP — iq_ref 62 LSB for ${STEP_S}s, then 186 + FOC oneshot"
            ocd -c 'init' -c "mww $REF 62" -c 'exit' >/dev/null
            sleep "$STEP_S"
            i=$(read_i)
            FX_WORDS=$(read_words "$FX" 9 | tr '\n' ' ')
            ocd -c 'init' -c "mww $((ACC + 8)) 0" -c 'exit' >/dev/null
            sleep 0.5
            ACC_WORDS=$(read_words "$ACC" 9 | tr '\n' ' ')
            python3 -c "
i=float('$i')
fx=[int(x,16) for x in '$FX_WORDS'.split()]
w=[int(x,16) for x in '$ACC_WORDS'.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
n=w[2]
lsb=1.617
id_m=s32(w[0])/n*lsb if n else 0
iq_m=s32(w[1])/n*lsb if n else 0
i0_m=s32(w[8])/n*lsb if n and len(w)>8 else 0
print(f'  pre-step bus {i:.3f} A  vq={s32(fx[3])/1e6:.2f} V  vd={s32(fx[2])/1e6:.2f} V  ref={s32(fx[4])} LSB')
print(f'  pre-step acc n={n}  id={id_m:+.1f} mA  iq={iq_m:+.1f} mA  i0={i0_m:+.1f} mA')
"
            ocd -c 'init' \
                -c "mwb $((TR + 20)) 0" \
                -c "mwh $((TR + 8)) 0" \
                -c "mwh $((TR + 10)) 1" \
                -c "mwh $((TR + 12)) 0" \
                -c "mww $((TR + 16)) 0" \
                -c "mwb $((TR + 21)) 0" \
                -c "mwb $((TR + 22)) 2" \
                -c "mwb $((TR + 20)) 1" \
                -c "mww $REF 186" \
                -c 'exit' >/dev/null
            sleep 0.05
            step_json="$OUT/foc-step-dir${dir}.json"
            mkdir -p "$OUT"
            python3 "$ROOT/scripts/trace_dump.py" --no-plot --json "$step_json" \
              -o "$OUT" --elf "$ELF" --cfg "$CFG" || true
            latest=$(ls -t "$OUT"/trace-foc-*.csv 2>/dev/null | head -1)
            python3 -c "
import csv, sys
p='$latest'
if not p:
    print('  step: no FOC CSV')
    raise SystemExit(0)
lsb=1.617
t=[]; iq=[]
with open(p) as f:
    r=csv.DictReader(f)
    for row in r:
        t.append(float(row['t_us']))
        iq.append(int(row['iq_lsb'])*lsb)
if len(iq)<20:
    print(f'  step: short capture n={len(iq)} file={p}')
    raise SystemExit(0)
peak=max(iq)
# 300 mA ±15% = 255..345. Rise from first sample; hold ≥3 ms once in band.
hit=next((i for i,v in enumerate(iq) if 255.0<=v<=345.0), None)
hold_ms=0.0
if hit is not None:
    end=hit
    while end<len(iq) and iq[end]>=255.0:
        end+=1
    hold_ms=(t[end-1]-t[hit])/1000.0
    t_hit=t[hit]/1000.0
    print(f'  step csv={p}')
    print(f'  step n={len(iq)}  iq0={iq[0]:+.1f} mA  peak={peak:+.1f} mA  hit 255mA at {t_hit:.2f} ms  hold {hold_ms:.2f} ms')
    ok=t_hit<=1.2 and hold_ms>=3.0
    print(f'  step verdict {\"PASS\" if ok else \"FAIL\"} (want hit≤1.2 ms and hold≥3 ms; do not retune Kp/Ki)')
else:
    print(f'  step csv={p}')
    print(f'  step n={len(iq)}  iq0={iq[0]:+.1f} mA  peak={peak:+.1f} mA  never entered 255..345 mA')
    print('  step verdict FAIL (do not retune Kp/Ki)')
"
          elif [[ "${ALT_S}" -gt 0 ]]; then
            if [[ "$tag" == midrail ]]; then
              alt=1
              alt_name=DPWMMIN
            else
              alt=2
              alt_name=midrail
            fi
            echo "  ALT NOW — sixth beep, $alt_name, same DT+vq+|vd|"
            ocd -c 'init' -c "mwb $LEG3 $alt" -c 'exit' >/dev/null
            host_beep
            gpio_watch alt "$ALT_S"
            PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
            echo "  prot after alt $PROT_WORDS"
            python3 -c "
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
print(f'  after alt latched={latched} sw={sw} bkin={bkin}')
if latched or sw or bkin:
    raise SystemExit('protection after alt')
"
          fi
        else
          echo "  skip alt — gold did not hold"
        fi
      elif [[ "${DT_S}" -gt 0 ]]; then
        DT=$(sym_addr g_motor_pwm_dt_on)
        echo "  DT NOW — fifth beep, dead-time only, vq stays 2.4 V, stay $tag"
        ocd -c 'init' -c "mwb $DT 1" -c 'exit' >/dev/null
        host_beep
        gpio_watch dt "$DT_S"
        PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
        echo "  prot after dt $PROT_WORDS"
        python3 -c "
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
print(f'  after dt latched={latched} sw={sw} bkin={bkin}')
if latched or sw or bkin:
    raise SystemExit('protection after dt')
"
        i=$(read_i)
        if python3 -c "
i=float('$i')
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
latched=(pr[3]>>16)&0xFF
raise SystemExit(0 if (latched==0 and i>=0.015) else 1)
"; then
          if [[ "${INTERP_S}" -gt 0 ]]; then
            IP=$(sym_addr g_motor_foc_interp)
            echo "  INTERP NOW — sixth beep, FocThetaInterp on Park, vq still 2.4 V"
            ocd -c 'init' -c "mwb $IP 1" -c 'exit' >/dev/null
            host_beep
            gpio_watch interp "$INTERP_S"
            PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
            FX_WORDS=$(read_words "$FX" 9 | tr '\n' ' ')
            echo "  prot after interp $PROT_WORDS"
            python3 -c "
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
fx=[int(x,16) for x in '$FX_WORDS'.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
print(f'  after interp latched={latched} sw={sw} bkin={bkin}  vq={s32(fx[3])/1e6:.2f} V  vd={s32(fx[2])/1e6:.2f} V')
if latched or sw or bkin:
    raise SystemExit('protection after interp')
"
            i=$(read_i)
            if [[ "${VQ36_S}" -gt 0 ]] && python3 -c "
i=float('$i')
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
latched=(pr[3]>>16)&0xFF
raise SystemExit(0 if (latched==0 and i>=0.015) else 1)
"; then
              VQMAX=$(sym_addr g_motor_foc_vq_max_uv)
              echo "  VQ 3.6 NOW — seventh beep, interp already on, vq ceiling only"
              ocd -c 'init' -c "mww $VQMAX 3600000" -c 'exit' >/dev/null
              host_beep
              gpio_watch vq36 "$VQ36_S"
              PROT_WORDS=$(read_words "$PROT" 4 | tr '\n' ' ')
              FX_WORDS=$(read_words "$FX" 9 | tr '\n' ' ')
              echo "  prot after vq36 $PROT_WORDS"
              python3 -c "
pr=[int(x,16) for x in '$PROT_WORDS'.split()]
fx=[int(x,16) for x in '$FX_WORDS'.split()]
def s32(u):
    return u-(1<<32) if u>=(1<<31) else u
latched=(pr[3]>>16)&0xFF
sw=pr[0]; bkin=pr[1]
print(f'  after vq36 latched={latched} sw={sw} bkin={bkin}  vq={s32(fx[3])/1e6:.2f} V  vd={s32(fx[2])/1e6:.2f} V')
if latched or sw or bkin:
    raise SystemExit('protection after vq36')
"
            fi
          fi
        else
          echo "  skip interp — DT did not hold"
        fi
      fi
    else
      echo "  skip fifth — id did not hold"
    fi
  else
    echo "  skip id — $tag did not hold"
  fi
  foc_rotating

  ACC=$(sym_addr g_motor_foc_fx_acc)
  ocd -c 'init' -c "mww $((ACC + 8)) 0" -c 'exit' >/dev/null
  sleep 2
  print_acc

  json="$OUT/foc-${tag}-dir${dir}.json"
  mkdir -p "$OUT"
  python3 "$ROOT/scripts/trace_dump.py" --no-plot --json "$json" \
    -o "$OUT" --elf "$ELF" --cfg "$CFG" || true
  python3 -c "
import json
p=json.load(open('$json'))
print(f\"  verdict rotating={p.get('rotating')} halls={p.get('hall_codes')} net={p.get('net_theta_deg')} deg\")
if not p.get('rotating'):
    raise SystemExit('not rotating after FOC')
"
  dump_foc_v "$OUT/foc-v-${tag}-dir${dir}.json"

  if [[ "${HOLD_AFTER}" == 1 ]]; then
    cpu_resume
    echo "  HOLD AFTER — CURRENT+spd still on. Turn the dial, then SCORE_ONLY=1."
    trap - EXIT
    exit 0
  fi
  motor_off
done
rep=$((rep + 1))
done

echo
echo "== reflashing safe firmware =="
trap - EXIT
reflash_safe
"$ROOT/scripts/board_check.sh"
ocd_stop
