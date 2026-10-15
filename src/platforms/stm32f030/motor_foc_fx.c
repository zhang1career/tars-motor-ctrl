#include "motor_foc_fx.h"
#include "fx_udiv.h"
#include "motor_adc.h"
#include "motor_angle.h"
#include "motor_pwm.h"
#include "motor_tick.h"
#include "hall6.h"

volatile motor_foc_fx_snapshot_t g_motor_foc_fx;
volatile uint8_t g_motor_foc_handover;
volatile int32_t g_motor_foc_iq_ref;
volatile int32_t g_motor_foc_iq_lim;
volatile uint8_t g_motor_foc_leg3;
volatile uint8_t g_motor_foc_interp;
volatile uint8_t g_motor_foc_id_on;
volatile uint8_t g_motor_foc_vq_neg;
volatile int32_t g_motor_foc_id_ref;
volatile int32_t g_motor_foc_vq_max_uv;
volatile int32_t g_motor_foc_vd_max_uv;
volatile int32_t g_motor_foc_park_slew_q16;
volatile int32_t g_motor_foc_park_off_q16;
volatile uint8_t g_motor_foc_spd_on;
volatile uint8_t g_motor_foc_overmod;
/* Observe Parks do not feed the PIs. Leave them off in the 20 kHz step. */
volatile uint8_t g_motor_foc_obs = 0U;
volatile int32_t g_motor_foc_w_ref_eps;
volatile int32_t g_motor_foc_w_meas_eps;

/* 20 kHz control loop, Ts = 50 us. Current-loop bandwidth 1200 Hz
 * (f_s/10). Pole-zero cancellation: Kp = wc*Ld, Ki = wc*Rs,
 * wc = 2*pi*1200 = 7540 rad/s
 *   Kp    = 7540 * 437.3e-6 H      = 3.297 V/A = 3297 uV/mA
 *   Ki*Ts = 7540 * 0.65 * 50e-6 s  = 0.2450 V/A = 245.1 uV/mA
 * then * 16.17 mA/LSB (1 mOhm shunt) to land on LSB.
 * Halving wc to 600 Hz (2026-10-09) left a warmed 100 mA d-step
 * sitting near 60 mA for 3 ms and still peaking near 165 mA, so the
 * 1200 Hz pair stays. */
#define FX_KP_UV_PER_LSB    53313  /* 3297 * 16.17 */
#define FX_KI_TS_UV_PER_LSB 3963   /* 245.1 * 16.17 */
#define FX_VQ_NEG_UV        1200000

/* 3.30 deg/tick = 183 elec/s at 20 kHz. 300 (91.6 elec/s) capped the
 * empty-load 5.0 V run and wound vd to +2.4 V (2026-09-11). 183 clears
 * the 5.0 V / 121 elec/s base speed with margin; first-edge snap is
 * still small vs a 60 deg stair. ISR host clamp stays 60..4000. */
#define FX_PARK_SLEW_Q16    600
/* Residual flux after FocThetaInterp was sector-centered (2026-09-13).
 * Fixed-iq sweep, both dirs, vq 2.4 V: +7 deg nulls |vd| and holds iq
 * flat under ±100 mA id. +23 deg was the old interpolator-lag cancel
 * and now overshoots (vd +0.5..+0.8 V). 0 deg still leaks id into iq.
 * Do not put this in MOTOR_ANGLE_OFFSET_Q16. Never write -30 deg
 * (-5461): that sat +vd and punched the V-path UCC27211. */
#define FX_PARK_OFF_Q16     1274

#define FX_INV_SQRT3_Q15    18919  /* 1/sqrt(3) * 32768 */
#define FX_SQRT3_2_Q15      28378  /* sqrt(3)/2 * 32768 */
#define FX_HALF_Q15         16384
#define FX_INV3_Q16         21846  /* 65536/3 */

/* s_sin_q15[k] = round(sin(2*pi*k/256) * 32767). No interpolation:
 * 1.4 deg quantisation, below the hall-anchor geometry error (roadmap 5.1). */
static const int16_t s_sin_q15[256] = {
         0,    804,   1608,   2410,   3212,   4011,   4808,   5602,
      6393,   7179,   7962,   8739,   9512,  10278,  11039,  11793,
     12539,  13279,  14010,  14732,  15446,  16151,  16846,  17530,
     18204,  18868,  19519,  20159,  20787,  21403,  22005,  22594,
     23170,  23731,  24279,  24811,  25329,  25832,  26319,  26790,
     27245,  27683,  28105,  28510,  28898,  29268,  29621,  29956,
     30273,  30571,  30852,  31113,  31356,  31580,  31785,  31971,
     32137,  32285,  32412,  32521,  32609,  32678,  32728,  32757,
     32767,  32757,  32728,  32678,  32609,  32521,  32412,  32285,
     32137,  31971,  31785,  31580,  31356,  31113,  30852,  30571,
     30273,  29956,  29621,  29268,  28898,  28510,  28105,  27683,
     27245,  26790,  26319,  25832,  25329,  24811,  24279,  23731,
     23170,  22594,  22005,  21403,  20787,  20159,  19519,  18868,
     18204,  17530,  16846,  16151,  15446,  14732,  14010,  13279,
     12539,  11793,  11039,  10278,   9512,   8739,   7962,   7179,
      6393,   5602,   4808,   4011,   3212,   2410,   1608,    804,
         0,   -804,  -1608,  -2410,  -3212,  -4011,  -4808,  -5602,
     -6393,  -7179,  -7962,  -8739,  -9512, -10278, -11039, -11793,
    -12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530,
    -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
    -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790,
    -27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956,
    -30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971,
    -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
    -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285,
    -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
    -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683,
    -27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731,
    -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868,
    -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
    -12539, -11793, -11039, -10278,  -9512,  -8739,  -7962,  -7179,
     -6393,  -5602,  -4808,  -4011,  -3212,  -2410,  -1608,   -804
};

/* One object so the 20 kHz tick keeps a single base pointer. Separate
 * symbols were a literal-pool load on every access. */
static struct {
  int32_t id_int_uv;
  int32_t iq_int_uv;
  int32_t iq_ref_lsb;
  /* Same three words. The current tick stores phase currents here.
   * The speed loop, which is not on that tick, stores its own state.
   * Turning the speed loop on without moving one of them overwrites
   * the currents. */
  union {
    struct {
      int32_t w_int_q8;
      int32_t w_filt_q8;
      int32_t iq_slew;
    } spd;
    struct {
      int32_t ia;
      int32_t ib;
      int32_t ic;
    } ph;
  } u;
  uint16_t park_theta;
  uint16_t last_stair;
  uint8_t park_have;
  uint8_t interp_was;
  uint8_t slew_locked;
  uint8_t spd_was;
  uint8_t iq_slew_div;
  uint8_t w_ki_div;
} s_fx;

#define s_id_int_uv s_fx.id_int_uv
#define s_iq_int_uv s_fx.iq_int_uv
#define s_iq_ref_lsb s_fx.iq_ref_lsb
#define s_park_theta s_fx.park_theta
#define s_last_stair s_fx.last_stair
#define s_park_have s_fx.park_have
#define s_interp_was s_fx.interp_was
#define s_slew_locked s_fx.slew_locked
#define s_w_int_q8 s_fx.u.spd.w_int_q8
#define s_w_filt_q8 s_fx.u.spd.w_filt_q8
#define s_iq_slew s_fx.u.spd.iq_slew
#define s_ph_ia s_fx.u.ph.ia
#define s_ph_ib s_fx.u.ph.ib
#define s_ph_ic s_fx.u.ph.ic
#define s_spd_was s_fx.spd_was
#define s_iq_slew_div s_fx.iq_slew_div
#define s_w_ki_div s_fx.w_ki_div

/* Speed loop. Current-loop Kp/Ki stay at wc=1200 Hz.
 * Kp=3 hunted. Always-on Ki stalled on hall-omega noise (2026-08-29).
 * Band ±8 froze Ki whenever |w_ref−w_meas|>8, so a 5.0 V free-run
 * (~120 elec/s) then hold-80 was P-only: dir0 sat at 61, dir1 at 110
 * (2026-09-13). Band ±40 covers empty-load capture, not a loaded crawl
 * (tick 4 hold-80: w_meas=22, |ew|=58, Ki off, iq stuck at P+seed).
 * Band ±120 keeps Ki on from a crawl up to w_ref. /32 is the hunt
 * damper, not the band. Seed from w_ref, not the voltage-limited iq
 * at enable. Ki every 20 kHz tick is 156 LSB/(elec/s)/s and hunts
 * above 120 (2026-09-13). /32 is ~5 LSB/(elec/s)/s. Slew /8 is not
 * the bottleneck (2500 vs 5 LSB/s). Host Park slew must clear this
 * ceiling (APPLY_SLEW); default 600 Q16 is still 183 elec/s. */
#define FX_W_KP_Q8 384
#define FX_W_KI_Q8 2
#define FX_W_FILT_SHIFT 8
#define FX_W_KI_BAND_EPS 120
#define FX_W_KI_DIV 32
#define FX_W_SLEW_DIV 8
#define FX_W_IQ_FLOOR 12
#define FX_W_REF_DEFAULT_EPS 40
#define FX_W_REF_MAX_EPS 260

volatile motor_foc_fx_acc_t g_motor_foc_fx_acc;

void MotorFocFx_ResetAcc(void)
{
  g_motor_foc_fx_acc.id = 0;
  g_motor_foc_fx_acc.iq = 0;
  g_motor_foc_fx_acc.n = 0U;
  g_motor_foc_fx_acc.id_st = 0;
  g_motor_foc_fx_acc.iq_st = 0;
  g_motor_foc_fx_acc.dth = 0;
  g_motor_foc_fx_acc.id_ip = 0;
  g_motor_foc_fx_acc.iq_ip = 0;
  g_motor_foc_fx_acc.i0 = 0;
}

void MotorFocFx_Init(void)
{
  s_id_int_uv = 0;
  s_iq_int_uv = 0;
  s_iq_ref_lsb = 0;
  g_motor_foc_fx.mode = MOTOR_FOC_FX_OFF;
  g_motor_foc_fx.steps = 0U;
  g_motor_foc_fx.iq_ref_lsb = 0;
  g_motor_foc_fx.sat = 0U;
  g_motor_foc_handover = 0U;
  g_motor_foc_iq_ref = 0;
  g_motor_foc_iq_lim = (int32_t)MOTOR_FOC_IQ_LSB;
  g_motor_foc_leg3 = 0U;
  g_motor_foc_interp = 0U;
  g_motor_foc_vq_neg = 0U;
  g_motor_foc_id_on = 0U;
  g_motor_foc_id_ref = 0;
  g_motor_pwm_dt_on = 0U;
  g_motor_foc_vq_max_uv = 2400000;
  g_motor_foc_vd_max_uv = 1200000;
  g_motor_foc_park_slew_q16 = FX_PARK_SLEW_Q16;
  g_motor_foc_park_off_q16 = FX_PARK_OFF_Q16;
  g_motor_foc_spd_on = 0U;
  g_motor_foc_overmod = 0U;
  g_motor_foc_w_ref_eps = FX_W_REF_DEFAULT_EPS;
  g_motor_foc_w_meas_eps = 0;
  s_w_int_q8 = 0;
  s_w_filt_q8 = 0;
  s_iq_slew = 0;
  s_iq_slew_div = 0U;
  s_w_ki_div = 0U;
  s_spd_was = 0U;
  s_park_have = 0U;
  s_interp_was = 0U;
  s_slew_locked = 0U;
  MotorFocFx_ResetAcc();
}

void MotorFocFx_Handover(void)
{
  uint8_t req = g_motor_foc_handover;

  if (req == 0U)
  {
    return;
  }

  if (req == 2U)
  {
    g_motor_foc_handover = 0U;
    MotorFocFx_SetMode(MOTOR_FOC_FX_OFF);
    return;
  }

  if (g_motor_angle.valid == 0U)
  {
    return;
  }

  g_motor_foc_handover = 0U;

  {
    int32_t iq = g_motor_foc_iq_ref;

    if (iq > (int32_t)MOTOR_FOC_IQ_LSB)
    {
      iq = (int32_t)MOTOR_FOC_IQ_LSB;
    }
    else if (iq < -(int32_t)MOTOR_FOC_IQ_LSB)
    {
      iq = -(int32_t)MOTOR_FOC_IQ_LSB;
    }
    /* Park/iPark uses hall6's voltage axis, so +iq reinforces whatever
     * direction hall6 was already commutating. Host sign is ignored. */
    MotorHall6_ReleasePwm();
    MotorHall6_FocArm(MotorHall6_LastDwell());
    MotorFocFx_SetIqRefLsb((int32_t)MOTOR_FOC_IQ_LSB);
    MotorFocFx_SetMode(MOTOR_FOC_FX_CURRENT);
    s_iq_int_uv = 1200000;
    s_id_int_uv = 0;
    /* Accepted CURRENT baseline: DPWMMIN + id + DT + vq_neg + FocThetaInterp.
     * Seed vq 2.4 V. Raise the ceiling after interp is locked. */
    g_motor_foc_leg3 = 1U;
    g_motor_foc_interp = 1U;
    g_motor_foc_id_on = 1U;
    g_motor_foc_id_ref = 0;
    g_motor_pwm_dt_on = 1U;
    g_motor_foc_vq_neg = 1U;
    g_motor_foc_vq_max_uv = 2400000;
    g_motor_foc_vd_max_uv = 2400000;
    g_motor_foc_park_slew_q16 = FX_PARK_SLEW_Q16;
    g_motor_foc_park_off_q16 = FX_PARK_OFF_Q16;
    g_motor_foc_spd_on = 0U;
    g_motor_foc_overmod = 0U;
    g_motor_foc_w_meas_eps = 0;
    s_w_int_q8 = 0;
    s_w_filt_q8 = 0;
    s_iq_slew = 0;
    s_iq_slew_div = 0U;
    s_w_ki_div = 0U;
    s_spd_was = 0U;
    s_park_have = 0U;
    s_interp_was = 0U;
    s_slew_locked = 0U;
    MotorAngle_SetDirHint(0);
  }
}

void MotorFocFx_SetMode(uint8_t mode)
{
  if (mode != g_motor_foc_fx.mode)
  {
    s_id_int_uv = 0;
    s_iq_int_uv = 0;
    s_w_int_q8 = 0;
    s_w_filt_q8 = 0;
    s_iq_slew = 0;
    s_iq_slew_div = 0U;
    s_w_ki_div = 0U;
    s_spd_was = 0U;
    s_park_have = 0U;
    s_interp_was = 0U;
    s_slew_locked = 0U;
    MotorFocFx_ResetAcc();
  }
  g_motor_foc_fx.mode = mode;
  if (mode != MOTOR_FOC_FX_CURRENT)
  {
    MotorAngle_SetDirHint(0);
  }
  if (mode == MOTOR_FOC_FX_OFF)
  {
    g_motor_foc_w_meas_eps = 0;
    g_motor_foc_spd_on = 0U;
  }
}

void MotorFocFx_SetIqRefLsb(int32_t lsb)
{
  if (lsb > 2048)
  {
    lsb = 2048;
  }
  else if (lsb < -2048)
  {
    lsb = -2048;
  }
  s_iq_ref_lsb = lsb;
  g_motor_foc_fx.iq_ref_lsb = lsb;
}

static inline int32_t fx_clamp(int32_t x, int32_t lo, int32_t hi)
{
  if (x < lo)
  {
    return lo;
  }
  if (x > hi)
  {
    return hi;
  }
  return x;
}

static inline uint16_t fx_slew_theta(uint16_t now, uint16_t dest)
{
  int32_t err = (int16_t)(dest - now);
  int32_t slew = g_motor_foc_park_slew_q16;

  slew = fx_clamp(slew, 60, 4000);
  if (err > slew)
  {
    return (uint16_t)(now + (uint16_t)slew);
  }
  if (err < -slew)
  {
    return (uint16_t)(now - (uint16_t)slew);
  }
  return dest;
}

static inline int32_t fx_vq_max_uv(void)
{
  int32_t m = g_motor_foc_vq_max_uv;

  if (m < 1200000)
  {
    m = 1200000;
  }
  else if (m > 10000000)
  {
    m = 10000000;
  }
  return m;
}

/* Speed PI is not on the signed-off current tick. Compiled out so it
 * cannot rot into the tick. The three words it used are the phase
 * currents on that tick. */
#if 0
/* noinline: handover leaves this off. Inlining it into Step forces the
 * current tick to keep the speed-loop locals in the same register file. */
static int32_t fx_speed_pi(void) __attribute__((noinline, unused));
static int32_t fx_speed_pi(void)
{
  int32_t om = MotorHall6_FocOmegaQ8();
  int32_t w_ref = g_motor_foc_w_ref_eps;
  int32_t w_meas;
  int32_t ew;
  int32_t iq_unlim;
  int32_t iq_hi = g_motor_foc_iq_lim;

  if (iq_hi < FX_W_IQ_FLOOR)
  {
    iq_hi = FX_W_IQ_FLOOR;
  }
  else if (iq_hi > (int32_t)MOTOR_FOC_IQ_LSB)
  {
    iq_hi = (int32_t)MOTOR_FOC_IQ_LSB;
  }

  if (om < 0)
  {
    om = -om;
  }
  /* elec/s = omega_q8 * 20000 / 2^24 = omega_q8 * 625 / 2^19 */
  w_meas = (om * 625) >> 19;

  w_ref = fx_clamp(w_ref, 0, FX_W_REF_MAX_EPS);
  if (s_spd_was == 0U)
  {
    /* Seed from last-tick measured |iq|, not IQ_LSB and not
     * (w_ref*5)/4. The cap couples the start transient to a compile
     * constant; the empty-load seed starves a loaded crawl. */
    {
      int32_t iq_now = g_motor_foc_fx.iq_lsb;

      if (iq_now < 0)
      {
        iq_now = -iq_now;
      }
      s_iq_slew = fx_clamp(iq_now, FX_W_IQ_FLOOR, iq_hi);
    }
    s_w_int_q8 = s_iq_slew << 8;
    s_w_filt_q8 = w_meas << 8;
    s_iq_slew_div = 0U;
    s_w_ki_div = 0U;
  }
  else
  {
    s_w_filt_q8 += ((w_meas << 8) - s_w_filt_q8) >> FX_W_FILT_SHIFT;
  }
  s_spd_was = 1U;
  w_meas = s_w_filt_q8 >> 8;
  g_motor_foc_w_meas_eps = w_meas;

  ew = w_ref - w_meas;
  iq_unlim = (FX_W_KP_Q8 * ew + s_w_int_q8) >> 8;
  if ((ew <= FX_W_KI_BAND_EPS) && (ew >= -FX_W_KI_BAND_EPS) &&
      ((iq_unlim < iq_hi) || (ew < 0)) &&
      ((iq_unlim > FX_W_IQ_FLOOR) || (ew > 0)))
  {
    s_w_ki_div++;
    if (s_w_ki_div >= FX_W_KI_DIV)
    {
      s_w_ki_div = 0U;
      s_w_int_q8 += FX_W_KI_Q8 * ew;
    }
  }
  s_w_int_q8 = fx_clamp(s_w_int_q8, FX_W_IQ_FLOOR << 8, iq_hi << 8);
  iq_unlim = fx_clamp((FX_W_KP_Q8 * ew + s_w_int_q8) >> 8,
                      FX_W_IQ_FLOOR, iq_hi);
  s_iq_slew_div++;
  if (s_iq_slew_div >= FX_W_SLEW_DIV)
  {
    s_iq_slew_div = 0U;
    if (iq_unlim > s_iq_slew)
    {
      s_iq_slew++;
    }
    else if (iq_unlim < s_iq_slew)
    {
      s_iq_slew--;
    }
  }
  return s_iq_slew;
}
#endif

static inline int32_t fx_vd_max_uv(void)
{
  int32_t m = g_motor_foc_vd_max_uv;

  if (m < 1200000)
  {
    m = 1200000;
  }
  else if (m > 2400000)
  {
    m = 2400000;
  }
  return m;
}

static inline int32_t fx_vq_lo_uv(void)
{
  if (g_motor_foc_vq_neg != 0U)
  {
    return -FX_VQ_NEG_UV;
  }
  return 0;
}

#if 0
/* Hexagon sat: recover the vd/vq that actually went out and rewind
 * both current-loop integrators (I := v_sat − Kp·e). Linear mode
 * already clamps Ki to vq_hi/vd_hi, which matches the circle when
 * VQMAX is Vdc/√3. Overmod raises that ceiling above the hexagon, so
 * sat must back-calculate or Ki sits on 7.7 V and hunts. */
static void fx_rewind_hexagon(int32_t va, int32_t vb, int32_t vc,
                              int32_t ct, int32_t sn,
                              int32_t ed, int32_t eq,
                              int32_t *vd_uv, int32_t *vq_uv)
    __attribute__((unused));
static void fx_rewind_hexagon(int32_t va, int32_t vb, int32_t vc,
                              int32_t ct, int32_t sn,
                              int32_t ed, int32_t eq,
                              int32_t *vd_uv, int32_t *vq_uv)
{
  int32_t v_al_d = ((((va << 1) - vb - vc) * FX_INV3_Q16) >> 16);
  int32_t v_be_d = (((vb - vc) * FX_INV_SQRT3_Q15) >> 15);
  int32_t vd_d = (v_al_d * ct + v_be_d * sn) >> 15;
  int32_t vq_d = (-v_al_d * sn + v_be_d * ct) >> 15;
  int32_t vd_hi = fx_vd_max_uv();
  int32_t vq_hi = fx_vq_max_uv();

  *vd_uv = vd_d << 10;
  *vq_uv = vq_d << 10;
  if (g_motor_foc_id_on != 0U)
  {
    s_id_int_uv = fx_clamp(*vd_uv - FX_KP_UV_PER_LSB * ed, -vd_hi, vd_hi);
  }
  s_iq_int_uv = fx_clamp(*vq_uv - FX_KP_UV_PER_LSB * eq, fx_vq_lo_uv(), vq_hi);
}
#endif

#pragma GCC push_options
#pragma GCC optimize ("O2")
/* 65536/span for span = 1..95. Index 0 is unused. One multiply plus at
 * most one correction equals the truncated division, for the products
 * the 8% floor actually forms (num = d * span_min, |d| <= span). */
static const uint32_t s_span_inv_q16[96] = {
    0, 65536, 32768, 21845, 16384, 13107, 10922, 9362, 8192, 7281,
    6553, 5957, 5461, 5041, 4681, 4369, 4096, 3855, 3640, 3449,
    3276, 3120, 2978, 2849, 2730, 2621, 2520, 2427, 2340, 2259,
    2184, 2114, 2048, 1985, 1927, 1872, 1820, 1771, 1724, 1680,
    1638, 1598, 1560, 1524, 1489, 1456, 1424, 1394, 1365, 1337,
    1310, 1285, 1260, 1236, 1213, 1191, 1170, 1149, 1129, 1110,
    1092, 1074, 1057, 1040, 1024, 1008, 992, 978, 963, 949,
    936, 923, 910, 897, 885, 873, 862, 851, 840, 829,
    819, 809, 799, 789, 780, 771, 762, 753, 744, 736,
    728, 720, 712, 704, 697, 689
};

#if 0
/* Same quotient as num/span for span in 1..95. noinline: at -O2 the
 * compiler otherwise folds this back into __divsi3, which is the cost
 * we are removing. The live floor uses fx_span_q. */
static int32_t fx_div_span(int32_t num, int32_t span) __attribute__((noinline, unused));
static int32_t fx_div_span(int32_t num, int32_t span)
{
  int32_t neg = 0;
  uint32_t mag;
  uint32_t q;
  uint32_t s;

  if (span <= 0)
  {
    span = 1;
  }
  else if (span >= 96)
  {
    span = 95;
  }
  if (num < 0)
  {
    neg = 1;
    num = -num;
  }
  s = (uint32_t)span;
  mag = (uint32_t)num;
  q = (mag * s_span_inv_q16[span]) >> 16;
  if (((q * s) + s) <= mag)
  {
    q++;
  }
  if (neg != 0)
  {
    return -(int32_t)q;
  }
  return (int32_t)q;
}
#endif

#if 0
/* noinline: the unsaturated tick must not carry this loop. */
static uint32_t isqrt32(uint32_t x) __attribute__((noinline, unused));
static uint32_t isqrt32(uint32_t x)
{
  uint32_t r = 0U;
  uint32_t b = 1UL << 30;

  while (b > x)
  {
    b >>= 2;
  }
  while (b != 0U)
  {
    if (x >= (r + b))
    {
      x -= r + b;
      r = (r >> 1) + b;
    }
    else
    {
      r >>= 1;
    }
    b >>= 2;
  }
  return r;
}
#endif

#pragma GCC pop_options

#if 0
/* Observe Parks do not feed the PIs. Kept out of Step so the current
 * tick does not allocate their angles and products. */
static void fx_observe_parks(int32_t i_al, int32_t i_be,
                             int32_t *id_st, int32_t *iq_st,
                             int32_t *id_ip, int32_t *iq_ip)
    __attribute__((noinline, unused));
static void fx_observe_parks(int32_t i_al, int32_t i_be,
                             int32_t *id_st, int32_t *iq_st,
                             int32_t *id_ip, int32_t *iq_ip)
{
  uint16_t th_st = MotorAngle_SectorAnchor();
  uint8_t idx_st = (uint8_t)(th_st >> 8);
  int32_t sn_st = (int32_t)s_sin_q15[idx_st];
  int32_t ct_st = (int32_t)s_sin_q15[(uint8_t)(idx_st + 64U)];
  uint8_t idx_ip = (uint8_t)(g_motor_angle.theta >> 8);
  int32_t sn_ip = (int32_t)s_sin_q15[idx_ip];
  int32_t ct_ip = (int32_t)s_sin_q15[(uint8_t)(idx_ip + 64U)];

  *id_st = ( i_al * ct_st + i_be * sn_st) >> 15;
  *iq_st = (-i_al * sn_st + i_be * ct_st) >> 15;
  *id_ip = ( i_al * ct_ip + i_be * sn_ip) >> 15;
  *iq_ip = (-i_al * sn_ip + i_be * ct_ip) >> 15;
}
#endif

#if 0
/* Returned by value. Taking the address of Step's phase voltages would
 * pin them to the stack on the tick that does not overmodulate. */
typedef struct
{
  int32_t va;
  int32_t vb;
  int32_t vc;
  int32_t vmax_ph;
  int32_t vmin_ph;
  int32_t vd_uv;
  int32_t vq_uv;
} fx_over_t;

static fx_over_t fx_overmod_apply(int32_t va, int32_t vb, int32_t vc,
                                  int32_t vmax_ph, int32_t vmin_ph,
                                  int32_t vdc_mv, int32_t ct, int32_t sn,
                                  int32_t ed, int32_t eq,
                                  int32_t vd_uv, int32_t vq_uv)
    __attribute__((noinline, unused));
static fx_over_t fx_overmod_apply(int32_t va, int32_t vb, int32_t vc,
                                  int32_t vmax_ph, int32_t vmin_ph,
                                  int32_t vdc_mv, int32_t ct, int32_t sn,
                                  int32_t ed, int32_t eq,
                                  int32_t vd_uv, int32_t vq_uv)
{
  fx_over_t o;
  int32_t span = vmax_ph - vmin_ph;
  int32_t vdc_u = (vdc_mv * 1000) >> 10;

  o.va = va;
  o.vb = vb;
  o.vc = vc;
  o.vmax_ph = vmax_ph;
  o.vmin_ph = vmin_ph;
  o.vd_uv = vd_uv;
  o.vq_uv = vq_uv;
  if ((span > vdc_u) && (span > 0))
  {
    int32_t mid = (vmax_ph + vmin_ph) >> 1;

    va = mid + Fx_DivS((va - mid) * vdc_u, span, 2U);
    vb = mid + Fx_DivS((vb - mid) * vdc_u, span, 2U);
    vc = mid + Fx_DivS((vc - mid) * vdc_u, span, 2U);
    vmax_ph = va;
    vmin_ph = va;
    if (vb > vmax_ph)
    {
      vmax_ph = vb;
    }
    if (vc > vmax_ph)
    {
      vmax_ph = vc;
    }
    if (vb < vmin_ph)
    {
      vmin_ph = vb;
    }
    if (vc < vmin_ph)
    {
      vmin_ph = vc;
    }
    o.va = va;
    o.vb = vb;
    o.vc = vc;
    o.vmax_ph = vmax_ph;
    o.vmin_ph = vmin_ph;
    g_motor_foc_fx.sat = 1U;
    fx_rewind_hexagon(va, vb, vc, ct, sn, ed, eq, &o.vd_uv, &o.vq_uv);
  }
  return o;
}
#endif

/* The 20 kHz tick is one configuration: handover's current loop.
 * ARR and the 24 V bus are compile-time constants, so the CCR scale
 * is not a division and the first sample does not run long. The vd/vq
 * clamps sit inside Vdc/sqrt(3), so this path does not take the circle.
 *
 * Park/PI and the modulator are separate functions so each has its own
 * registers. One function spilled a 68-byte frame, about a hundred
 * stack accesses, which is what held the tick near 2000 cycles. The
 * modulator reloads the snapshot and recomputes sin and the phase
 * currents for dead-time. That costs a few dozen cycles and no RAM.
 *
 * Observe, speed, and overmodulation are compiled out above. A second
 * copy of the same math does not fit in 32 KB,
 * and it is not the signed-off current law. */
#define FX_TICK_ARR ((int32_t)((MOTOR_PWM_TIM_CLK / (2U * MOTOR_PWM_HZ)) - 1U))
#define FX_TICK_K   ((int32_t)((FX_TICK_ARR * 33554) / (int32_t)MOTOR_VBUS_FIXED_MV))
#define FX_TICK_SPAN_MIN ((int32_t)((FX_TICK_ARR * 8) / 100))
/* DTG 72 counts. Half of that is the dead-time compensation step.
 * 36/3 == 0, so inside the band the step is i * 12. */
#define FX_TICK_HALF_DT 36
#define FX_TICK_DT_BAND 3

#pragma GCC push_options
#pragma GCC optimize ("Os")

/* Positive truncated quotient for the 8% floor. span is 1..94 and
 * num = d * SPAN_MIN with 0 < d < span. A C '/' here lowers to
 * __divsi3 and the tick length follows the magnitude. always_inline:
 * the three floor sites must be the multiply, not a call. */
static inline __attribute__((always_inline)) int32_t fx_span_q(int32_t num, int32_t span)
{
  uint32_t q = ((uint32_t)num * s_span_inv_q16[span]) >> 16;
  uint32_t s = (uint32_t)span;

  if (((q * s) + s) <= (uint32_t)num)
  {
    q++;
  }
  return (int32_t)q;
}

/* Hall calls first, alone. Phase currents must not be live across
 * those calls or the PI spills them for the whole multiply. */
static void fx_tick_angle(void) __attribute__((noinline, optimize("Os")));
static void fx_tick_angle(void)
{
  uint16_t th_stair = MotorHall6_FocTheta();
  uint16_t th_ip = MotorHall6_FocThetaInterp(th_stair);
  uint16_t theta;
  int16_t dth;

  g_motor_foc_fx.theta_interp = th_stair;
  if (g_motor_foc_interp == 0U)
  {
    s_park_theta = th_stair;
    s_last_stair = th_stair;
    s_park_have = 1U;
    s_slew_locked = 0U;
    theta = th_stair;
  }
  else
  {
    if ((s_interp_was == 0U) || (s_park_have == 0U))
    {
      s_park_theta = th_stair;
      s_last_stair = th_stair;
      s_park_have = 1U;
      s_slew_locked = 0U;
    }
    if (s_slew_locked == 0U)
    {
      if (th_stair != s_last_stair)
      {
        s_slew_locked = 1U;
      }
      s_park_theta = th_stair;
      s_last_stair = th_stair;
    }
    else
    {
      s_park_theta = fx_slew_theta(s_park_theta, th_ip);
    }
    theta = s_park_theta;
  }
  s_interp_was = (g_motor_foc_interp != 0U) ? 1U : 0U;
  dth = (int16_t)(th_ip - th_stair);
  theta = (uint16_t)(theta + (uint16_t)fx_clamp(g_motor_foc_park_off_q16,
                                                -16384, 16384));
  g_motor_foc_fx.theta = theta;
  g_motor_foc_fx.dth = dth;
}

static void fx_tick_pi(void) __attribute__((noinline, optimize("Os")));
static void fx_tick_pi(void)
{
  int32_t ia;
  int32_t ib;
  int32_t ic;
  int32_t i0;
  int32_t i_al;
  int32_t i_be;
  int32_t id;
  int32_t iq;
  int32_t sn;
  int32_t ct;
  int32_t ed;
  int32_t eq;
  int32_t vd_uv;
  int32_t vq_uv;
  uint16_t theta;
  uint8_t idx;

  theta = g_motor_foc_fx.theta;
  idx = (uint8_t)(theta >> 8);
  sn = (int32_t)s_sin_q15[idx];
  ct = (int32_t)s_sin_q15[(uint8_t)(idx + 64U)];
  ia = -((int32_t)g_motor_adc_raw[MOTOR_ADC_IU] - 2048);
  ib = -((int32_t)g_motor_adc_raw[MOTOR_ADC_IV] - 2048);
  ic = -((int32_t)g_motor_adc_raw[MOTOR_ADC_IW] - 2048);
  i0 = ((ia + ib + ic) * FX_INV3_Q16) >> 16;
  ia -= i0;
  ib -= i0;
  ic -= i0;
  i_al = ia;
  i_be = ((ia + (ib << 1)) * FX_INV_SQRT3_Q15) >> 15;
  id = ((i_al * ct) + (i_be * sn)) >> 15;
  iq = ((-i_al * sn) + (i_be * ct)) >> 15;
  /* Dead-time reads these. Same storage as the speed-loop state;
   * that loop is not on this tick. */
  s_ph_ia = ia;
  s_ph_ib = ib;
  s_ph_ic = ic;
  g_motor_foc_fx.id_lsb = id;
  g_motor_foc_fx.iq_lsb = iq;

  /* Speed loop is not on this tick. Clearing the seed matches the
   * spd_on == 0 arm of the old step. */
  s_spd_was = 0U;
  {
    int32_t iq_cmd = g_motor_foc_iq_ref;
    int32_t id_cmd = g_motor_foc_id_ref;
    int32_t vq_hi;
    int32_t vq_lo;
    int32_t vd_hi;
    int32_t vd_unlim;
    int32_t vq_unlim;

    if (iq_cmd < 0)
    {
      iq_cmd = 0;
    }
    else if (iq_cmd > (int32_t)MOTOR_FOC_IQ_LSB)
    {
      iq_cmd = (int32_t)MOTOR_FOC_IQ_LSB;
    }
    s_iq_ref_lsb = iq_cmd;
    g_motor_foc_fx.iq_ref_lsb = iq_cmd;
    if (id_cmd > (int32_t)MOTOR_FOC_ID_LSB)
    {
      id_cmd = (int32_t)MOTOR_FOC_ID_LSB;
    }
    else if (id_cmd < -(int32_t)MOTOR_FOC_ID_LSB)
    {
      id_cmd = -(int32_t)MOTOR_FOC_ID_LSB;
    }
    if (g_motor_foc_id_on == 0U)
    {
      id_cmd = 0;
    }
    ed = id_cmd - id;
    eq = iq_cmd - iq;
    vq_hi = fx_vq_max_uv();
    vq_lo = fx_vq_lo_uv();
    if (g_motor_foc_id_on != 0U)
    {
      vd_hi = fx_vd_max_uv();
      vd_unlim = (FX_KP_UV_PER_LSB * ed) + s_id_int_uv;
      if (((vd_unlim < vd_hi) || (ed < 0)) &&
          ((vd_unlim > -vd_hi) || (ed > 0)))
      {
        s_id_int_uv += FX_KI_TS_UV_PER_LSB * ed;
      }
      s_id_int_uv = fx_clamp(s_id_int_uv, -vd_hi, vd_hi);
      vd_uv = fx_clamp((FX_KP_UV_PER_LSB * ed) + s_id_int_uv, -vd_hi, vd_hi);
    }
    else
    {
      s_id_int_uv = 0;
      vd_uv = 0;
    }
    vq_unlim = (FX_KP_UV_PER_LSB * eq) + s_iq_int_uv;
    if (((vq_unlim < vq_hi) || (eq < 0)) &&
        ((vq_lo == 0) || (vq_unlim > vq_lo) || (eq > 0)))
    {
      s_iq_int_uv += FX_KI_TS_UV_PER_LSB * eq;
    }
    s_iq_int_uv = fx_clamp(s_iq_int_uv, vq_lo, vq_hi);
    vq_uv = fx_clamp((FX_KP_UV_PER_LSB * eq) + s_iq_int_uv, vq_lo, vq_hi);
  }

  g_motor_foc_fx.vd_uv = vd_uv;
  g_motor_foc_fx.vq_uv = vq_uv;
  g_motor_foc_fx.sat = 0U;
  if (g_motor_foc_fx_acc.n == 0U)
  {
    g_motor_foc_fx_acc.id = id;
    g_motor_foc_fx_acc.iq = iq;
    g_motor_foc_fx_acc.id_st = 0;
    g_motor_foc_fx_acc.iq_st = 0;
    g_motor_foc_fx_acc.dth = 0;
    g_motor_foc_fx_acc.id_ip = 0;
    g_motor_foc_fx_acc.iq_ip = 0;
    g_motor_foc_fx_acc.i0 = 0;
    g_motor_foc_fx_acc.n = 1U;
  }
  else
  {
    g_motor_foc_fx_acc.id += id;
    g_motor_foc_fx_acc.iq += iq;
    g_motor_foc_fx_acc.n++;
  }
}

static void fx_tick_pwm(void) __attribute__((noinline, optimize("Os")));
static void fx_tick_pwm(void)
{
  int32_t ia;
  int32_t ib;
  int32_t ic;
  int32_t vd_u;
  int32_t vq_u;
  int32_t sn;
  int32_t ct;
  int32_t v_al;
  int32_t v_be;
  int32_t va;
  int32_t vb;
  int32_t vc;
  int32_t vmin;
  int32_t c0;
  int32_t c1;
  int32_t c2;
  int32_t cmin;
  int32_t cmax;
  int32_t span;
  uint16_t theta;
  uint8_t idx;

  /* Fixed 24 V and the vd/vq clamps sit inside Vdc/sqrt(3). sat stays
   * clear. Sin is recomputed from the angle the PI just stored. */
  vd_u = g_motor_foc_fx.vd_uv >> 10;
  vq_u = g_motor_foc_fx.vq_uv >> 10;
  theta = g_motor_foc_fx.theta;
  idx = (uint8_t)(theta >> 8);
  sn = (int32_t)s_sin_q15[idx];
  ct = (int32_t)s_sin_q15[(uint8_t)(idx + 64U)];
  v_al = ((vd_u * ct) - (vq_u * sn)) >> 15;
  v_be = ((vd_u * sn) + (vq_u * ct)) >> 15;
  va = v_al;
  vb = ((-v_al * FX_HALF_Q15) + (v_be * FX_SQRT3_2_Q15)) >> 15;
  vc = ((-v_al * FX_HALF_Q15) - (v_be * FX_SQRT3_2_Q15)) >> 15;
  vmin = va;
  if (vb < vmin)
  {
    vmin = vb;
  }
  if (vc < vmin)
  {
    vmin = vc;
  }
  c0 = fx_clamp(((va - vmin) * FX_TICK_K) >> 15, 0, FX_TICK_ARR);
  c1 = fx_clamp(((vb - vmin) * FX_TICK_K) >> 15, 0, FX_TICK_ARR);
  c2 = fx_clamp(((vc - vmin) * FX_TICK_K) >> 15, 0, FX_TICK_ARR);
  cmax = c0;
  cmin = c0;
  if (c1 > cmax)
  {
    cmax = c1;
  }
  if (c2 > cmax)
  {
    cmax = c2;
  }
  if (c1 < cmin)
  {
    cmin = c1;
  }
  if (c2 < cmin)
  {
    cmin = c2;
  }
  span = cmax - cmin;
  if ((span > 0) && (span < FX_TICK_SPAN_MIN))
  {
    int32_t d;
    int32_t add;

    d = c0 - cmin;
    add = (d == 0) ? 0 : ((d == span) ? FX_TICK_SPAN_MIN
                          : fx_span_q(d * FX_TICK_SPAN_MIN, span));
    c0 = fx_clamp(cmin + add, 0, FX_TICK_ARR);
    d = c1 - cmin;
    add = (d == 0) ? 0 : ((d == span) ? FX_TICK_SPAN_MIN
                          : fx_span_q(d * FX_TICK_SPAN_MIN, span));
    c1 = fx_clamp(cmin + add, 0, FX_TICK_ARR);
    d = c2 - cmin;
    add = (d == 0) ? 0 : ((d == span) ? FX_TICK_SPAN_MIN
                          : fx_span_q(d * FX_TICK_SPAN_MIN, span));
    c2 = fx_clamp(cmin + add, 0, FX_TICK_ARR);
  }

  g_motor_foc_fx.ccr[0] = (uint16_t)c0;
  g_motor_foc_fx.ccr[1] = (uint16_t)c1;
  g_motor_foc_fx.ccr[2] = (uint16_t)c2;
  g_motor_foc_fx.steps++;

  if (g_motor_pwm_prot.latched != 0U)
  {
    return;
  }

  if (g_motor_pwm_dt_on != 0U)
  {
    int32_t du;
    int32_t dv;
    int32_t dw;

    ia = s_ph_ia;
    ib = s_ph_ib;
    ic = s_ph_ic;
    du = (ia > FX_TICK_DT_BAND) ? FX_TICK_HALF_DT :
         ((ia < -FX_TICK_DT_BAND) ? -FX_TICK_HALF_DT :
          (ia * (FX_TICK_HALF_DT / FX_TICK_DT_BAND)));
    dv = (ib > FX_TICK_DT_BAND) ? FX_TICK_HALF_DT :
         ((ib < -FX_TICK_DT_BAND) ? -FX_TICK_HALF_DT :
          (ib * (FX_TICK_HALF_DT / FX_TICK_DT_BAND)));
    dw = (ic > FX_TICK_DT_BAND) ? FX_TICK_HALF_DT :
         ((ic < -FX_TICK_DT_BAND) ? -FX_TICK_HALF_DT :
          (ic * (FX_TICK_HALF_DT / FX_TICK_DT_BAND)));
    c0 = fx_clamp(c0 + du, 0, FX_TICK_ARR);
    c1 = fx_clamp(c1 + dv, 0, FX_TICK_ARR);
    c2 = fx_clamp(c2 + dw, 0, FX_TICK_ARR);
  }
  TIM1->CCR1 = (uint32_t)c0;
  TIM1->CCR2 = (uint32_t)c1;
  TIM1->CCR3 = (uint32_t)c2;
  TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E |
                 TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
}

static void fx_tick_current(void) __attribute__((noinline));
static void fx_tick_current(void)
{
  fx_tick_angle();
  fx_tick_pi();
  fx_tick_pwm();
}

#pragma GCC pop_options

void MotorFocFx_Step(void)
{
  /* CURRENT drives. OBSERVE used to fall into the body below and the
   * main loop missed the window until the watchdog reset the chip. */
  if (g_motor_foc_fx.mode != MOTOR_FOC_FX_CURRENT)
  {
    return;
  }
  fx_tick_current();
}
