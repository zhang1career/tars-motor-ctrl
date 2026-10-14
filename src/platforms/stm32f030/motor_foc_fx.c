#include "motor_foc_fx.h"
#include "motor_adc.h"
#include "motor_angle.h"
#include "motor_pwm.h"
#include "hall6.h"

volatile motor_foc_fx_snapshot_t g_motor_foc_fx;
volatile uint8_t g_motor_foc_handover;
volatile int32_t g_motor_foc_iq_ref;
volatile int32_t g_motor_foc_iq_lim;
volatile uint8_t g_motor_foc_leg3;
volatile uint8_t g_motor_foc_interp;
volatile uint8_t g_motor_foc_id_on;
volatile int32_t g_motor_foc_id_ref;
volatile int32_t g_motor_foc_vq_max_uv;
volatile int32_t g_motor_foc_vd_max_uv;
volatile int32_t g_motor_foc_park_slew_q16;
volatile int32_t g_motor_foc_park_off_q16;
volatile uint8_t g_motor_foc_spd_on;
volatile uint8_t g_motor_foc_overmod;
volatile int32_t g_motor_foc_w_ref_eps;
volatile int32_t g_motor_foc_w_meas_eps;

/* 20 kHz control loop, Ts = 50 us. Current-loop bandwidth 1200 Hz
 * (f_s/10). Pole-zero cancellation: Kp = wc*Ld, Ki = wc*Rs,
 * wc = 2*pi*1200 = 7540 rad/s
 *   Kp    = 7540 * 437.3e-6 H      = 3.297 V/A = 3297 uV/mA
 *   Ki*Ts = 7540 * 0.65 * 50e-6 s  = 0.2450 V/A = 245.1 uV/mA
 * then * 1.617 mA/LSB to land on LSB: */
#define FX_KP_UV_PER_LSB    5331   /* 3297 * 1.617 */
#define FX_KI_TS_UV_PER_LSB 396    /* 245.1 * 1.617 */

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

static int32_t s_id_int_uv;
static int32_t s_iq_int_uv;
static int32_t s_iq_ref_lsb;
static uint16_t s_park_theta;
static uint16_t s_last_stair;
static uint8_t s_park_have;
static uint8_t s_interp_was;
static uint8_t s_slew_locked;
static int32_t s_w_int_q8;
static int32_t s_w_filt_q8;
static int32_t s_iq_slew;
static uint8_t s_spd_was;
static uint8_t s_iq_slew_div;
static uint8_t s_w_ki_div;

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
    MotorFocFx_SetIqRefLsb((int32_t)MOTOR_FOC_IQ_LSB);
    MotorFocFx_SetMode(MOTOR_FOC_FX_CURRENT);
    s_iq_int_uv = 1200000;
    s_id_int_uv = 0;
    /* Accepted CURRENT baseline: DPWMMIN + id + DT + FocThetaInterp.
     * Seed vq 2.4 V. Raise the ceiling after interp is locked. */
    g_motor_foc_leg3 = 1U;
    g_motor_foc_interp = 1U;
    g_motor_foc_id_on = 1U;
    g_motor_foc_id_ref = 0;
    g_motor_pwm_dt_on = 1U;
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

static int32_t fx_clamp(int32_t x, int32_t lo, int32_t hi)
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

static uint16_t fx_slew_theta(uint16_t now, uint16_t dest)
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

static int32_t fx_vq_max_uv(void)
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

static int32_t fx_vd_max_uv(void)
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

/* Hexagon sat: recover the vd/vq that actually went out and rewind
 * both current-loop integrators (I := v_sat − Kp·e). Linear mode
 * already clamps Ki to vq_hi/vd_hi, which matches the circle when
 * VQMAX is Vdc/√3. Overmod raises that ceiling above the hexagon, so
 * sat must back-calculate or Ki sits on 7.7 V and hunts. */
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
  s_iq_int_uv = fx_clamp(*vq_uv - FX_KP_UV_PER_LSB * eq, 0, vq_hi);
}

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

void MotorFocFx_Step(void)
{
  uint8_t mode = g_motor_foc_fx.mode;
  int32_t ia;
  int32_t ib;
  int32_t ic;
  int32_t i0;
  int32_t i_al;
  int32_t i_be;
  int32_t id;
  int32_t iq;
  int32_t id_st;
  int32_t iq_st;
  int32_t id_ip;
  int32_t iq_ip;
  int16_t dth;
  int32_t sn;
  int32_t ct;
  int32_t vdc_mv;
  int32_t vmax_uv;
  int32_t ed;
  int32_t eq;
  int32_t vd_uv;
  int32_t vq_uv;
  int32_t vd_u;
  int32_t vq_u;
  int32_t vmax_u;
  int32_t vmag2;
  int32_t v_al;
  int32_t v_be;
  int32_t va;
  int32_t vb;
  int32_t vc;
  int32_t vcom;
  int32_t vmax_ph;
  int32_t vmin_ph;
  int32_t arr;
  int32_t k;
  int32_t half;
  int32_t ccr[3];
  uint8_t idx;
  uint16_t theta;
  uint8_t j;

  if (mode == MOTOR_FOC_FX_OFF)
  {
    return;
  }

  /* ---- 4.1 three shunts: zero-sequence removal, then Clarke ---- */
  ia = -((int32_t)g_motor_adc_raw[MOTOR_ADC_IU] - 2048);
  ib = -((int32_t)g_motor_adc_raw[MOTOR_ADC_IV] - 2048);
  ic = -((int32_t)g_motor_adc_raw[MOTOR_ADC_IW] - 2048);

  i0 = ((ia + ib + ic) * FX_INV3_Q16) >> 16;
  ia -= i0;
  ib -= i0;
  ic -= i0;

  i_al = ia;
  i_be = ((ia + 2 * ib) * FX_INV_SQRT3_Q15) >> 15;

  /* ---- 4.2 Park ----
   * CURRENT defaults to discrete FocTheta. Following FocThetaInterp
   * on Hi-Z used to trip BKIN when the open phase tracked CCR mid.
   * Host may enable interp after DPWMMIN is already spinning; Park
   * stays on the stair until the next hall edge, then follows. */
  {
    uint16_t th_stair = MotorHall6_FocTheta();
    uint16_t th_ip = MotorHall6_FocThetaInterp(th_stair);
    uint8_t want_slew = ((mode == MOTOR_FOC_FX_CURRENT) &&
                         (g_motor_foc_interp != 0U)) ? 1U : 0U;

    g_motor_foc_fx.theta_interp = th_stair;
    if (mode != MOTOR_FOC_FX_CURRENT)
    {
      theta = g_motor_angle.theta;
    }
    else if (want_slew == 0U)
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
          s_park_theta = th_stair;
        }
        else
        {
          s_park_theta = th_stair;
        }
        s_last_stair = th_stair;
      }
      else
      {
        s_park_theta = fx_slew_theta(s_park_theta, th_ip);
      }
      theta = s_park_theta;
    }
    s_interp_was = want_slew;
    dth = (int16_t)(th_ip - th_stair);
    if (mode == MOTOR_FOC_FX_CURRENT)
    {
      theta = (uint16_t)(theta +
                         (uint16_t)fx_clamp(g_motor_foc_park_off_q16,
                                            -16384, 16384));
    }
  }
  idx = (uint8_t)(theta >> 8);
  sn = (int32_t)s_sin_q15[idx];
  ct = (int32_t)s_sin_q15[(uint8_t)(idx + 64U)];

  id = ( i_al * ct + i_be * sn) >> 15;
  iq = (-i_al * sn + i_be * ct) >> 15;

  /* Staircase Park: same currents, sector-anchor angle. 6-step current
   * is DC in this frame. Do not feed these to the PIs. */
  {
    uint16_t th_st = MotorAngle_SectorAnchor();
    uint8_t idx_st = (uint8_t)(th_st >> 8);
    int32_t sn_st = (int32_t)s_sin_q15[idx_st];
    int32_t ct_st = (int32_t)s_sin_q15[(uint8_t)(idx_st + 64U)];

    id_st = ( i_al * ct_st + i_be * sn_st) >> 15;
    iq_st = (-i_al * sn_st + i_be * ct_st) >> 15;
  }

  /* Interpolator frame, observe only. Not used for Park or PWM. */
  {
    uint8_t idx_ip = (uint8_t)(g_motor_angle.theta >> 8);
    int32_t sn_ip = (int32_t)s_sin_q15[idx_ip];
    int32_t ct_ip = (int32_t)s_sin_q15[(uint8_t)(idx_ip + 64U)];

    id_ip = ( i_al * ct_ip + i_be * sn_ip) >> 15;
    iq_ip = (-i_al * sn_ip + i_be * ct_ip) >> 15;
  }

  /* 3.9639 mV/LSB * 1024 = 4058 */
  vdc_mv = ((int32_t)g_motor_adc_raw[MOTOR_ADC_VBUS] * 4058) >> 10;
  if (vdc_mv < 1000)
  {
    vdc_mv = 1000;
  }
  vmax_uv = vdc_mv * 577; /* 1000/sqrt(3) */

  /* ---- 4.3 current PIs. No cross-coupling / back-EMF this revision. ---- */
  if (mode == MOTOR_FOC_FX_CURRENT)
  {
    int32_t iq_cmd;

    if (g_motor_foc_spd_on != 0U)
    {
      iq_cmd = fx_speed_pi();
      g_motor_foc_iq_ref = iq_cmd;
    }
    else
    {
      s_spd_was = 0U;
      iq_cmd = g_motor_foc_iq_ref;
      if (iq_cmd < 0)
      {
        iq_cmd = 0;
      }
      else if (iq_cmd > (int32_t)MOTOR_FOC_IQ_LSB)
      {
        iq_cmd = (int32_t)MOTOR_FOC_IQ_LSB;
      }
    }
    MotorFocFx_SetIqRefLsb(iq_cmd);
  }
  {
    int32_t id_cmd = g_motor_foc_id_ref;

    if (id_cmd > (int32_t)MOTOR_FOC_ID_LSB)
    {
      id_cmd = (int32_t)MOTOR_FOC_ID_LSB;
    }
    else if (id_cmd < -(int32_t)MOTOR_FOC_ID_LSB)
    {
      id_cmd = -(int32_t)MOTOR_FOC_ID_LSB;
    }
    if ((mode != MOTOR_FOC_FX_CURRENT) || (g_motor_foc_id_on == 0U))
    {
      id_cmd = 0;
    }
    ed = id_cmd - id;
  }
  eq = g_motor_foc_fx.iq_ref_lsb - iq;

  if (mode == MOTOR_FOC_FX_CURRENT)
  {
    int32_t vq_hi = fx_vq_max_uv();
    int32_t vq_unlim;

    /* Conditional integrate: freeze Ki when the output is already at
     * the ceiling and eq would push it further. Integrator floor is 0
     * (handover still seeds 1.2 V). Same rule on d, bipolar. */
    if (g_motor_foc_id_on != 0U)
    {
      int32_t vd_hi = fx_vd_max_uv();
      int32_t vd_unlim = FX_KP_UV_PER_LSB * ed + s_id_int_uv;

      if (((vd_unlim < vd_hi) || (ed < 0)) &&
          ((vd_unlim > -vd_hi) || (ed > 0)))
      {
        s_id_int_uv += FX_KI_TS_UV_PER_LSB * ed;
      }
      s_id_int_uv = fx_clamp(s_id_int_uv, -vd_hi, vd_hi);
      vd_uv = fx_clamp(FX_KP_UV_PER_LSB * ed + s_id_int_uv, -vd_hi, vd_hi);
    }
    else
    {
      s_id_int_uv = 0;
      vd_uv = 0;
    }
    vq_unlim = FX_KP_UV_PER_LSB * eq + s_iq_int_uv;
    if ((vq_unlim < vq_hi) || (eq < 0))
    {
      s_iq_int_uv += FX_KI_TS_UV_PER_LSB * eq;
    }
    s_iq_int_uv = fx_clamp(s_iq_int_uv, 0, vq_hi);
    vq_uv = fx_clamp(FX_KP_UV_PER_LSB * eq + s_iq_int_uv, 0, vq_hi);
  }
  else
  {
    vd_uv = 0;
    vq_uv = FX_KP_UV_PER_LSB * eq + s_iq_int_uv;
  }

  /* ---- 4.4 voltage limit. Linear: inscribed circle Vdc/√3.
   * Overmod: skip the circle; hexagon clamp after iPark (span ≤ Vdc)
   * reaches the vertices (2/3 Vdc, six-step fundamental ~2 Vdc/π). */
  vd_u = vd_uv >> 10;
  vq_u = vq_uv >> 10;
  vmax_u = vmax_uv >> 10;
  vmag2 = vd_u * vd_u + vq_u * vq_u;
  g_motor_foc_fx.sat = 0U;
  if ((g_motor_foc_overmod == 0U) && (vmag2 > (vmax_u * vmax_u)))
  {
    int32_t vmag = (int32_t)isqrt32((uint32_t)vmag2);

    vd_u = (vd_u * vmax_u) / vmag;
    vq_u = (vq_u * vmax_u) / vmag;
    g_motor_foc_fx.sat = 1U;
  }

  /* ---- 4.5 inverse Park + SVPWM ---- */
  v_al = (vd_u * ct - vq_u * sn) >> 15;
  v_be = (vd_u * sn + vq_u * ct) >> 15;

  va = v_al;
  vb = (-v_al * FX_HALF_Q15 + v_be * FX_SQRT3_2_Q15) >> 15;
  vc = (-v_al * FX_HALF_Q15 - v_be * FX_SQRT3_2_Q15) >> 15;

  vmax_ph = va;
  if (vb > vmax_ph)
  {
    vmax_ph = vb;
  }
  if (vc > vmax_ph)
  {
    vmax_ph = vc;
  }
  vmin_ph = va;
  if (vb < vmin_ph)
  {
    vmin_ph = vb;
  }
  if (vc < vmin_ph)
  {
    vmin_ph = vc;
  }
  if (g_motor_foc_overmod != 0U)
  {
    int32_t span = vmax_ph - vmin_ph;
    int32_t vdc_u = (vdc_mv * 1000) >> 10;

    if ((span > vdc_u) && (span > 0))
    {
      int32_t mid = (vmax_ph + vmin_ph) >> 1;

      va = mid + ((va - mid) * vdc_u) / span;
      vb = mid + ((vb - mid) * vdc_u) / span;
      vc = mid + ((vc - mid) * vdc_u) / span;
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
      g_motor_foc_fx.sat = 1U;
      fx_rewind_hexagon(va, vb, vc, ct, sn, ed, eq, &vd_uv, &vq_uv);
    }
  }
  /* ---- 4.6 CCR. 33554 = 1.024 * 32768; v is VU = 1.024 mV. ---- */
  arr = (int32_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
  k = (arr * 33554) / vdc_mv;
  /* Hi-Z: mid-centred. DPWMMIN: vmin → CCR 0, line voltages unchanged. */
  if (g_motor_foc_leg3 == 1U)
  {
    vcom = vmin_ph;
    half = 0;
  }
  else
  {
    vcom = (vmax_ph + vmin_ph) >> 1;
    half = arr >> 1;
  }
  {
    const int32_t vph[3] = {va, vb, vc};

    for (j = 0U; j < 3U; j++)
    {
      int32_t c = half + (((vph[j] - vcom) * k) >> 15);

      ccr[j] = fx_clamp(c, 0, arr);
    }
  }

  /* Same 8% line-voltage floor as ApplyFocVoltages. Mid-centred
   * recenters about mid_c; DPWMMIN keeps the min phase at 0. */
  if (mode == MOTOR_FOC_FX_CURRENT)
  {
    int32_t cmax = ccr[0];
    int32_t cmin = ccr[0];
    int32_t span;
    int32_t span_min = (arr * 8) / 100;

    for (j = 1U; j < 3U; j++)
    {
      if (ccr[j] > cmax)
      {
        cmax = ccr[j];
      }
      if (ccr[j] < cmin)
      {
        cmin = ccr[j];
      }
    }
    span = cmax - cmin;
    if ((span > 0) && (span < span_min))
    {
      if (g_motor_foc_leg3 == 1U)
      {
        for (j = 0U; j < 3U; j++)
        {
          int32_t d = ccr[j] - cmin;

          ccr[j] = fx_clamp(cmin + (d * span_min) / span, 0, arr);
        }
      }
      else
      {
        int32_t mid_c = (cmax + cmin) >> 1;

        for (j = 0U; j < 3U; j++)
        {
          int32_t d = ccr[j] - mid_c;

          ccr[j] = fx_clamp(mid_c + (d * span_min) / span, 0, arr);
        }
      }
    }
  }

  g_motor_foc_fx.theta = theta;
  g_motor_foc_fx.dth = dth;
  g_motor_foc_fx.id_lsb = id;
  g_motor_foc_fx.iq_lsb = iq;
  g_motor_foc_fx.vd_uv = vd_uv;
  g_motor_foc_fx.vq_uv = vq_uv;
  g_motor_foc_fx.ccr[0] = (uint16_t)ccr[0];
  g_motor_foc_fx.ccr[1] = (uint16_t)ccr[1];
  g_motor_foc_fx.ccr[2] = (uint16_t)ccr[2];
  g_motor_foc_fx.steps++;

  /* n==0 is the host's reset: the next sample starts a new mean. */
  if (g_motor_foc_fx_acc.n == 0U)
  {
    g_motor_foc_fx_acc.id = id;
    g_motor_foc_fx_acc.iq = iq;
    g_motor_foc_fx_acc.id_st = id_st;
    g_motor_foc_fx_acc.iq_st = iq_st;
    g_motor_foc_fx_acc.dth = (int32_t)dth;
    g_motor_foc_fx_acc.id_ip = id_ip;
    g_motor_foc_fx_acc.iq_ip = iq_ip;
    g_motor_foc_fx_acc.i0 = i0;
    g_motor_foc_fx_acc.n = 1U;
  }
  else
  {
    g_motor_foc_fx_acc.id += id;
    g_motor_foc_fx_acc.iq += iq;
    g_motor_foc_fx_acc.id_st += id_st;
    g_motor_foc_fx_acc.iq_st += iq_st;
    g_motor_foc_fx_acc.dth += (int32_t)dth;
    g_motor_foc_fx_acc.id_ip += id_ip;
    g_motor_foc_fx_acc.iq_ip += iq_ip;
    g_motor_foc_fx_acc.i0 += i0;
    g_motor_foc_fx_acc.n++;
  }

  if ((mode == MOTOR_FOC_FX_CURRENT) && (MotorPwm_IsLatched() == 0U))
  {
    if (g_motor_foc_leg3 != 0U)
    {
      MotorPwm_SetDutiesCcr(ccr[0], ccr[1], ccr[2], ia, ib, ic);
    }
    else
    {
      MotorPwm_SetDutiesCcrHiZ(ccr[0], ccr[1], ccr[2], ia, ib, ic,
                               MotorHall6_FloatPhase());
    }
  }
}
