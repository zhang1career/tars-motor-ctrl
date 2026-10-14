#include "motor_foc.h"
#include "motor_adc.h"
#include "motor_angle.h"
#include "motor_pwm.h"
#include "motor_tick.h"
#include "board_pins.h"

volatile motor_foc_snapshot_t g_motor_foc;

#define TWO_PI 6.28318531f
#define INV_SQRT3 0.57735027f
#define SQRT3_OVER_2 0.86602540f

#define FOC_TS ((float)MOTOR_FOC_DECIM / (float)MOTOR_CTRL_ISR_HZ)
#define FOC_WC (TWO_PI * MOTOR_FOC_WC_HZ)
#define FOC_KP (FOC_WC * MOTOR_FOC_LD)
#define FOC_KI_TS (FOC_WC * MOTOR_FOC_RS * FOC_TS)

/* 1.617 mA per LSB, from the INA240A2 gain of 50 across a 10 mOhm shunt with
 * VDDA as the ADC reference (measured, roadmap 1.2.1). The shunt reads positive
 * for current leaving the phase, so phase current is the negative of it. */
#define LSB_TO_AMPS (-1.617e-3f)

/* R30 39k / R31 10k divider, VDDA reference. */
#define VBUS_LSB_TO_V (3.3124f / 4095.0f * 4.9f)

/*
 * motor_angle publishes speed as angle counts per tick times 256, signed by
 * direction. One count is 2*pi/65536 electrical radians and a tick is
 * 1/MOTOR_CTRL_ISR_HZ seconds:
 *   rad/s = omega_q8 / 256 * (2*pi/65536) * MOTOR_CTRL_ISR_HZ
 */
#define OMEGA_Q8_TO_RAD_S \
  (TWO_PI * (float)MOTOR_CTRL_ISR_HZ / (256.0f * 65536.0f))

/*
 * Quarter-wave would save flash but not cycles, and 1 KB of the 18 KB free is
 * not worth the indexing. 256 entries indexed by the top 8 bits of the angle
 * quantise to 1.4 degrees, i.e. 0.7 degrees of error -- an order below the
 * anchor spacing error the hall geometry already imposes (roadmap 5.1), so
 * interpolating would be polishing the wrong term.
 */
static const float s_sin_tab[256] = {
    0.000000f, 0.024541f, 0.049068f, 0.073565f, 0.098017f, 0.122411f, 0.146730f,
    0.170962f, 0.195090f, 0.219101f, 0.242980f, 0.266713f, 0.290285f, 0.313682f,
    0.336890f, 0.359895f, 0.382683f, 0.405241f, 0.427555f, 0.449611f, 0.471397f,
    0.492898f, 0.514103f, 0.534998f, 0.555570f, 0.575808f, 0.595699f, 0.615232f,
    0.634393f, 0.653173f, 0.671559f, 0.689541f, 0.707107f, 0.724247f, 0.740951f,
    0.757209f, 0.773010f, 0.788346f, 0.803208f, 0.817585f, 0.831470f, 0.844854f,
    0.857729f, 0.870087f, 0.881921f, 0.893224f, 0.903989f, 0.914210f, 0.923880f,
    0.932993f, 0.941544f, 0.949528f, 0.956940f, 0.963776f, 0.970031f, 0.975702f,
    0.980785f, 0.985278f, 0.989177f, 0.992480f, 0.995185f, 0.997290f, 0.998795f,
    0.999699f, 1.000000f, 0.999699f, 0.998795f, 0.997290f, 0.995185f, 0.992480f,
    0.989177f, 0.985278f, 0.980785f, 0.975702f, 0.970031f, 0.963776f, 0.956940f,
    0.949528f, 0.941544f, 0.932993f, 0.923880f, 0.914210f, 0.903989f, 0.893224f,
    0.881921f, 0.870087f, 0.857729f, 0.844854f, 0.831470f, 0.817585f, 0.803208f,
    0.788346f, 0.773010f, 0.757209f, 0.740951f, 0.724247f, 0.707107f, 0.689541f,
    0.671559f, 0.653173f, 0.634393f, 0.615232f, 0.595699f, 0.575808f, 0.555570f,
    0.534998f, 0.514103f, 0.492898f, 0.471397f, 0.449611f, 0.427555f, 0.405241f,
    0.382683f, 0.359895f, 0.336890f, 0.313682f, 0.290285f, 0.266713f, 0.242980f,
    0.219101f, 0.195090f, 0.170962f, 0.146730f, 0.122411f, 0.098017f, 0.073565f,
    0.049068f, 0.024541f, 0.000000f, -0.024541f, -0.049068f, -0.073565f,
    -0.098017f, -0.122411f, -0.146730f, -0.170962f, -0.195090f, -0.219101f,
    -0.242980f, -0.266713f, -0.290285f, -0.313682f, -0.336890f, -0.359895f,
    -0.382683f, -0.405241f, -0.427555f, -0.449611f, -0.471397f, -0.492898f,
    -0.514103f, -0.534998f, -0.555570f, -0.575808f, -0.595699f, -0.615232f,
    -0.634393f, -0.653173f, -0.671559f, -0.689541f, -0.707107f, -0.724247f,
    -0.740951f, -0.757209f, -0.773010f, -0.788346f, -0.803208f, -0.817585f,
    -0.831470f, -0.844854f, -0.857729f, -0.870087f, -0.881921f, -0.893224f,
    -0.903989f, -0.914210f, -0.923880f, -0.932993f, -0.941544f, -0.949528f,
    -0.956940f, -0.963776f, -0.970031f, -0.975702f, -0.980785f, -0.985278f,
    -0.989177f, -0.992480f, -0.995185f, -0.997290f, -0.998795f, -0.999699f,
    -1.000000f, -0.999699f, -0.998795f, -0.997290f, -0.995185f, -0.992480f,
    -0.989177f, -0.985278f, -0.980785f, -0.975702f, -0.970031f, -0.963776f,
    -0.956940f, -0.949528f, -0.941544f, -0.932993f, -0.923880f, -0.914210f,
    -0.903989f, -0.893224f, -0.881921f, -0.870087f, -0.857729f, -0.844854f,
    -0.831470f, -0.817585f, -0.803208f, -0.788346f, -0.773010f, -0.757209f,
    -0.740951f, -0.724247f, -0.707107f, -0.689541f, -0.671559f, -0.653173f,
    -0.634393f, -0.615232f, -0.595699f, -0.575808f, -0.555570f, -0.534998f,
    -0.514103f, -0.492898f, -0.471397f, -0.449611f, -0.427555f, -0.405241f,
    -0.382683f, -0.359895f, -0.336890f, -0.313682f, -0.290285f, -0.266713f,
    -0.242980f, -0.219101f, -0.195090f, -0.170962f, -0.146730f, -0.122411f,
    -0.098017f, -0.073565f, -0.049068f, -0.024541f
};

static float s_id_int;
static float s_iq_int;
static float s_iq_ref;

void MotorFoc_Init(void)
{
  s_id_int = 0.0f;
  s_iq_int = 0.0f;
  s_iq_ref = 0.0f;
  g_motor_foc.mode = MOTOR_FOC_OFF;
  g_motor_foc.steps = 0U;
  g_motor_foc.iq_ref_ma = 0;
}

void MotorFoc_SetMode(uint8_t mode)
{
  if (mode != g_motor_foc.mode)
  {
    /* Never carry integrator state across a mode change: it was accumulated
     * against a different plant (in observe mode the output goes nowhere, so
     * the integrator would wind up unopposed). */
    s_id_int = 0.0f;
    s_iq_int = 0.0f;
  }
  g_motor_foc.mode = mode;
}

void MotorFoc_SetIqRefMa(int16_t ma)
{
  float a = (float)ma * 1e-3f;

  if (a > MOTOR_FOC_IMAX)
  {
    a = MOTOR_FOC_IMAX;
  }
  else if (a < -MOTOR_FOC_IMAX)
  {
    a = -MOTOR_FOC_IMAX;
  }
  s_iq_ref = a;
  g_motor_foc.iq_ref_ma = (int16_t)(a * 1000.0f);
}

/*
 * Inverse square root without libm. Calling sqrtf costs 388 B of RAM, not
 * cycles: it references __errno, which drags in _impure_ptr and with it newlib's
 * __sf -- the FILE structures for stdin/stdout/stderr, 312 B of a 4 KB part, for
 * a program that has no streams.
 *
 * Classic seed plus two Newton steps, relative error below 1e-4, and it only
 * runs on the ticks where the voltage limit is actually reached.
 */
static float foc_inv_sqrt(float x)
{
  union
  {
    float f;
    uint32_t u;
  } c;
  float y;

  c.f = x;
  c.u = 0x5F3759DFU - (c.u >> 1);
  y = c.f;
  y = y * (1.5f - 0.5f * x * y * y);
  y = y * (1.5f - 0.5f * x * y * y);
  return y;
}

static float foc_clamp(float x, float lo, float hi)
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

void MotorFoc_Step(void)
{
  uint8_t mode = g_motor_foc.mode;
  float ia;
  float ib;
  float ic;
  float i0;
  float i_al;
  float i_be;
  float ct;
  float sn;
  float id;
  float iq;
  float vdc;
  float vmax;
  float we;
  float ed;
  float eq;
  float vd;
  float vq;
  float vmag2;
  float v_al;
  float v_be;
  float va;
  float vb;
  float vc;
  float vcom;
  float inv_vdc;
  float duty[3];
  uint8_t idx;
  uint16_t theta;

  if (mode == MOTOR_FOC_OFF)
  {
    return;
  }

  /* ---- 1. three shunts: zero-sequence removal, then Clarke ---- */
  ia = (float)MotorAdc_ShuntLsb(MOTOR_ADC_IU) * LSB_TO_AMPS;
  ib = (float)MotorAdc_ShuntLsb(MOTOR_ADC_IV) * LSB_TO_AMPS;
  ic = (float)MotorAdc_ShuntLsb(MOTOR_ADC_IW) * LSB_TO_AMPS;

  /* A star-connected motor carries no zero-sequence current, so whatever
   * (ia+ib+ic)/3 comes out as is measurement error. Measured rms 5 mA. */
  i0 = (ia + ib + ic) * (1.0f / 3.0f);
  ia -= i0;
  ib -= i0;

  i_al = ia;
  i_be = (ia + 2.0f * ib) * INV_SQRT3;

  /* ---- 2. angle ---- */
  theta = g_motor_angle.theta;
  idx = (uint8_t)(theta >> 8);
  sn = s_sin_tab[idx];
  ct = s_sin_tab[(uint8_t)(idx + 64U)];

  /* ---- 3. Park ---- */
  id = i_al * ct + i_be * sn;
  iq = -i_al * sn + i_be * ct;

  vdc = (float)g_motor_adc_raw[MOTOR_ADC_VBUS] * VBUS_LSB_TO_V;
  if (vdc < 1.0f)
  {
    vdc = 1.0f;
  }
  vmax = vdc * INV_SQRT3;

  /* ---- 5. current PIs ---- */
  ed = 0.0f - id; /* SPMSM, no field weakening: id_ref = 0 */
  eq = s_iq_ref - iq;

  if (mode == MOTOR_FOC_CURRENT)
  {
    s_id_int = foc_clamp(s_id_int + FOC_KI_TS * ed, -vmax, vmax);
    s_iq_int = foc_clamp(s_iq_int + FOC_KI_TS * eq, -vmax, vmax);
  }

  /* Cross-coupling decoupling uses the measured Lq, so it is safe; the
   * back-EMF term is gated by MOTOR_FOC_LAMBDA, which stays zero until Ke is
   * measured. At 24 electrical rev/s the coupling term is only ~0.1 V anyway. */
  we = (float)g_motor_angle.omega_q8 * OMEGA_Q8_TO_RAD_S;
  vd = FOC_KP * ed + s_id_int - we * MOTOR_FOC_LQ * iq;
  vq = FOC_KP * eq + s_iq_int + we * (MOTOR_FOC_LD * id + MOTOR_FOC_LAMBDA);

  /* Voltage circle limit. Comparing squares avoids a soft-float sqrt on every
   * tick and only pays for one when the limit is actually reached. */
  vmag2 = vd * vd + vq * vq;
  if (vmag2 > (vmax * vmax))
  {
    float sc = vmax * foc_inv_sqrt(vmag2);

    vd *= sc;
    vq *= sc;
    g_motor_foc.sat = 1U;
  }
  else
  {
    g_motor_foc.sat = 0U;
  }

  /* ---- 6. inverse Park ---- */
  v_al = vd * ct - vq * sn;
  v_be = vd * sn + vq * ct;

  /* ---- 8. SVPWM by min/max common-mode injection ---- */
  va = v_al;
  vb = -0.5f * v_al + SQRT3_OVER_2 * v_be;
  vc = -0.5f * v_al - SQRT3_OVER_2 * v_be;

  vcom = va;
  if (vb > vcom)
  {
    vcom = vb;
  }
  if (vc > vcom)
  {
    vcom = vc;
  }
  {
    float vmin = va;

    if (vb < vmin)
    {
      vmin = vb;
    }
    if (vc < vmin)
    {
      vmin = vc;
    }
    vcom = (vcom + vmin) * 0.5f;
  }

  inv_vdc = 1.0f / vdc;
  duty[0] = foc_clamp((va - vcom) * inv_vdc + 0.5f, 0.0f, 1.0f);
  duty[1] = foc_clamp((vb - vcom) * inv_vdc + 0.5f, 0.0f, 1.0f);
  duty[2] = foc_clamp((vc - vcom) * inv_vdc + 0.5f, 0.0f, 1.0f);

  g_motor_foc.theta = theta;
  g_motor_foc.id_ma = (int16_t)(id * 1000.0f);
  g_motor_foc.iq_ma = (int16_t)(iq * 1000.0f);
  g_motor_foc.vd_mv = (int16_t)(vd * 1000.0f);
  g_motor_foc.vq_mv = (int16_t)(vq * 1000.0f);
  g_motor_foc.duty_q12[0] = (uint16_t)(duty[0] * 4096.0f);
  g_motor_foc.duty_q12[1] = (uint16_t)(duty[1] * 4096.0f);
  g_motor_foc.duty_q12[2] = (uint16_t)(duty[2] * 4096.0f);
  g_motor_foc.steps++;

  if (mode == MOTOR_FOC_CURRENT)
  {
    MotorPwm_SetDuties(duty[0], duty[1], duty[2], ia, ib, ic);
  }
}
