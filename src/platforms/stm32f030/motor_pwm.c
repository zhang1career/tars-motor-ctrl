#include "motor_pwm.h"
#include "board_pins.h"
#include "hall6.h"
#include "main.h"
#include "reset_diag.h"
#include <stddef.h>
#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
#include "motor_adc.h"
#endif
#include "boost.h"
#if defined(MOTOR_TRACE) && (MOTOR_TRACE != 0)
#include "motor_trace.h"
#endif

_Static_assert(offsetof(motor_pwm_prot_t, ocp_hold) == 17,
               "ocp_hold moved; host reads this byte");
_Static_assert(offsetof(motor_pwm_prot_t, ocp_zero) == 20,
               "ocp_zero moved; host reads these halfwords");
_Static_assert(offsetof(motor_pwm_prot_t, ocp_zero_ready) == 26,
               "ocp_zero_ready moved; host reads this byte");

TIM_HandleTypeDef htim1;
volatile motor_pwm_prot_t g_motor_pwm_prot;
volatile uint8_t g_motor_pwm_force_trip;
volatile uint8_t g_motor_pwm_dt_on;

/* Cleared by HardwareSafe. ChargeBootstrap refuses to run again until then.
 * s_boot_pending stays set through the low-side window. A start that runs
 * inside the I2C interrupt only arms that window; the thread path waits it
 * out. SysTick cannot preempt that interrupt, so the wait must not spin there. */
static uint8_t s_boot_charged;
static uint8_t s_boot_pending;
static uint32_t s_boot_t0;
/* Bridge-off zero, learned in the main loop. The tick starts only after
 * MOE is already on, so the ISR must not be the one collecting it.
 * Dropped if MOE turns on before 64 calm samples. */
static int32_t s_ocp_zero_sum[3];
static uint8_t s_ocp_zero_n;

#ifndef MOTOR_PWM_BOOT_MS
#define MOTOR_PWM_BOOT_MS 40U
#endif

#define MOTOR_TIM1_ARR  ((MOTOR_PWM_TIM_CLK / (2U * MOTOR_PWM_HZ)) - 1U)

/*
 * DTG is non-linear above 127: 0xx -> DTG*tDTS, 10x -> (64+DTG[5:0])*2*tDTS,
 * 110 -> (32+DTG[4:0])*8*tDTS. TARS uses DTG=216 on a 72 MHz tDTS.
 * At 48 MHz tDTS, DTG=72 -> 1.5 us; DTG=144 -> 3.33 us (TARS-equivalent time).
 * Dead time subtracts from the HS pulse, so it sets the usable duty floor.
 */
#ifndef MOTOR_TIM1_DTG
#define MOTOR_TIM1_DTG  72U
#endif
#define MOTOR_TIM1_DT   MOTOR_TIM1_DTG

static void motor_pwm_disable_oc_preload(TIM_TypeDef *tim)
{
  tim->CCMR1 &= ~(TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
  tim->CCMR2 &= ~TIM_CCMR2_OC3PE;
}

static void motor_pwm_force_update(TIM_TypeDef *tim)
{
  tim->EGR = TIM_EGR_UG;
}

#pragma GCC push_options
#pragma GCC optimize ("O0")

void MotorPwm_SetPhase(motor_phase_mode_t u, motor_phase_mode_t v, motor_phase_mode_t w,
                       uint8_t duty_pct)
{
  TIM_TypeDef *tim;

  if (g_motor_pwm_prot.latched != 0U)
  {
    return;
  }

  tim = htim1.Instance;
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  uint32_t pulse = (arr * (uint32_t)duty_pct) / 100U;

  if (pulse > arr)
  {
    pulse = arr;
  }

  switch (u)
  {
  case MOTOR_PHASE_OFF:
    tim->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
    tim->CCR1 = 0U;
    break;
  case MOTOR_PHASE_PWM:
    tim->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
    tim->CCR1 = pulse;
    break;
  case MOTOR_PHASE_LOW:
    /* CCxE cleared makes OCxN follow OCxREF directly (no complement, no dead
     * time), so OCxREF must stay active for the low side to conduct. */
    tim->CCER &= ~TIM_CCER_CC1E;
    tim->CCER |= TIM_CCER_CC1NE;
    tim->CCR1 = arr + 1U;
    break;
  default:
    tim->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
    tim->CCR1 = 0U;
    break;
  }

  switch (v)
  {
  case MOTOR_PHASE_OFF:
    tim->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE);
    tim->CCR2 = 0U;
    break;
  case MOTOR_PHASE_PWM:
    tim->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);
    tim->CCR2 = pulse;
    break;
  case MOTOR_PHASE_LOW:
    tim->CCER &= ~TIM_CCER_CC2E;
    tim->CCER |= TIM_CCER_CC2NE;
    tim->CCR2 = arr + 1U;
    break;
  default:
    tim->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE);
    tim->CCR2 = 0U;
    break;
  }

  switch (w)
  {
  case MOTOR_PHASE_OFF:
    tim->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE);
    tim->CCR3 = 0U;
    break;
  case MOTOR_PHASE_PWM:
    tim->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);
    tim->CCR3 = pulse;
    break;
  case MOTOR_PHASE_LOW:
    tim->CCER &= ~TIM_CCER_CC3E;
    tim->CCER |= TIM_CCER_CC3NE;
    tim->CCR3 = arr + 1U;
    break;
  default:
    tim->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE);
    tim->CCR3 = 0U;
    break;
  }

  motor_pwm_disable_oc_preload(tim);
  motor_pwm_force_update(tim);
}

void MotorPwm_SetPairCcr(uint8_t pwm_idx, uint8_t low_idx, uint16_t ccr)
{
  TIM_TypeDef *tim;
  volatile uint32_t *ccr_reg[3];
  static const uint32_t en[3] = {
      TIM_CCER_CC1E, TIM_CCER_CC2E, TIM_CCER_CC3E};
  static const uint32_t ne[3] = {
      TIM_CCER_CC1NE, TIM_CCER_CC2NE, TIM_CCER_CC3NE};
  uint32_t arr;
  uint32_t pulse;
  uint8_t k;

  if (g_motor_pwm_prot.latched != 0U)
  {
    return;
  }
  if ((pwm_idx > 2U) || (low_idx > 2U) || (pwm_idx == low_idx))
  {
    return;
  }

  tim = htim1.Instance;
  arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  pulse = ccr;
  if (pulse > arr)
  {
    pulse = arr;
  }

  ccr_reg[0] = &tim->CCR1;
  ccr_reg[1] = &tim->CCR2;
  ccr_reg[2] = &tim->CCR3;

  for (k = 0U; k < 3U; k++)
  {
    if (k == pwm_idx)
    {
      tim->CCER |= (en[k] | ne[k]);
      *ccr_reg[k] = pulse;
    }
    else if (k == low_idx)
    {
      /* Same as MOTOR_PHASE_LOW: OCxN follows OCxREF, so OCxREF stays active. */
      tim->CCER &= ~en[k];
      tim->CCER |= ne[k];
      *ccr_reg[k] = arr + 1U;
    }
    else
    {
      tim->CCER &= ~(en[k] | ne[k]);
      *ccr_reg[k] = 0U;
    }
  }

  motor_pwm_disable_oc_preload(tim);
  motor_pwm_force_update(tim);
}

#pragma GCC pop_options

/* Below this the current sign is noise (the front end reads +-3 mA), and
 * flipping the compensation on noise would inject a square wave at the noise
 * rate. Ramp linearly through the band instead of switching. */
#ifndef MOTOR_PWM_DT_COMP_BAND_A
#define MOTOR_PWM_DT_COMP_BAND_A 0.05f
#endif

static int32_t motor_pwm_dt_comp(float i)
{
  const float half_dt = (float)MOTOR_TIM1_DT * 0.5f;
  float scale;

  if (i > MOTOR_PWM_DT_COMP_BAND_A)
  {
    scale = 1.0f;
  }
  else if (i < -MOTOR_PWM_DT_COMP_BAND_A)
  {
    scale = -1.0f;
  }
  else
  {
    scale = i * (1.0f / MOTOR_PWM_DT_COMP_BAND_A);
  }

  return (int32_t)(scale * half_dt);
}

void MotorPwm_SetDuties(float da, float db, float dc, float ia, float ib, float ic)
{
  TIM_TypeDef *tim;

  if (g_motor_pwm_prot.latched != 0U)
  {
    return;
  }

  tim = htim1.Instance;
  int32_t arr = (int32_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
  const float duties[3] = {da, db, dc};
  const float currents[3] = {ia, ib, ic};
  int32_t ccr[3];
  uint8_t k;

  for (k = 0U; k < 3U; k++)
  {
    int32_t v = (int32_t)(duties[k] * (float)arr) + motor_pwm_dt_comp(currents[k]);

    if (v < 0)
    {
      v = 0;
    }
    else if (v > arr)
    {
      v = arr;
    }
    ccr[k] = v;
  }

  tim->CCR1 = (uint32_t)ccr[0];
  tim->CCR2 = (uint32_t)ccr[1];
  tim->CCR3 = (uint32_t)ccr[2];
  tim->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E |
                TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
}

/* 0.05 A / 16.17 mA per LSB. Below this the current sign is noise, and
 * flipping the compensation on noise would inject a square wave at the
 * noise rate. Ramp linearly through the band instead. */
#ifndef MOTOR_PWM_DT_COMP_BAND_LSB
#define MOTOR_PWM_DT_COMP_BAND_LSB 3
#endif
#ifndef MOTOR_PWM_DT_COMP
#define MOTOR_PWM_DT_COMP 1
#endif

static int32_t motor_pwm_dt_comp_lsb(int32_t i_lsb)
{
  const int32_t half_dt = (int32_t)(MOTOR_TIM1_DT / 2U);
  const int32_t band = (int32_t)MOTOR_PWM_DT_COMP_BAND_LSB;

  if (i_lsb > band)
  {
    return half_dt;
  }
  if (i_lsb < -band)
  {
    return -half_dt;
  }
  /* Exact only when band divides half_dt. Do not hardcode the quotient. */
  if ((half_dt % band) == 0)
  {
    return i_lsb * (half_dt / band);
  }
  return (i_lsb * half_dt) / band;
}

void MotorPwm_SetDutiesCcr(int32_t ccr_u, int32_t ccr_v, int32_t ccr_w,
                           int32_t ia_lsb, int32_t ib_lsb, int32_t ic_lsb)
{
  TIM_TypeDef *tim;

  if (g_motor_pwm_prot.latched != 0U)
  {
    return;
  }

  tim = htim1.Instance;
  int32_t arr = (int32_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
  const int32_t raw[3] = {ccr_u, ccr_v, ccr_w};
  const int32_t currents[3] = {ia_lsb, ib_lsb, ic_lsb};
  int32_t ccr[3];
  uint8_t k;

  /* Default off. The shift is ±MOTOR_TIM1_DT/2 counts on each leg.
   * Host may turn it on after the current loop is already running. */
  for (k = 0U; k < 3U; k++)
  {
    int32_t v = raw[k];

    if (g_motor_pwm_dt_on != 0U)
    {
      v += motor_pwm_dt_comp_lsb(currents[k]);
    }

    if (v < 0)
    {
      v = 0;
    }
    else if (v > arr)
    {
      v = arr;
    }
    ccr[k] = v;
  }

  tim->CCR1 = (uint32_t)ccr[0];
  tim->CCR2 = (uint32_t)ccr[1];
  tim->CCR3 = (uint32_t)ccr[2];
  tim->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E |
                TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
}

void MotorPwm_SetDutiesCcrHiZ(int32_t ccr_u, int32_t ccr_v, int32_t ccr_w,
                              int32_t ia_lsb, int32_t ib_lsb, int32_t ic_lsb,
                              uint8_t hz)
{
  TIM_TypeDef *tim;
  static const uint32_t en[3] = {
      TIM_CCER_CC1E | TIM_CCER_CC1NE,
      TIM_CCER_CC2E | TIM_CCER_CC2NE,
      TIM_CCER_CC3E | TIM_CCER_CC3NE};

  if (g_motor_pwm_prot.latched != 0U)
  {
    return;
  }

  tim = htim1.Instance;
  int32_t arr = (int32_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
  const int32_t raw[3] = {ccr_u, ccr_v, ccr_w};
  const int32_t currents[3] = {ia_lsb, ib_lsb, ic_lsb};
  int32_t ccr[3];
  int32_t half = arr >> 1;
  int32_t best;
  uint32_t ccer;
  uint8_t k;
  uint8_t hz_sel = hz;

  for (k = 0U; k < 3U; k++)
  {
#if MOTOR_PWM_DT_COMP
    int32_t v = raw[k] + motor_pwm_dt_comp_lsb(currents[k]);
#else
    int32_t v = raw[k];
    (void)currents;
#endif

    if (v < 0)
    {
      v = 0;
    }
    else if (v > arr)
    {
      v = arr;
    }
    ccr[k] = v;
  }

  if (hz_sel > 2U)
  {
    hz_sel = 0U;
    best = ccr[0] - half;
    if (best < 0)
    {
      best = -best;
    }
    for (k = 1U; k < 3U; k++)
    {
      int32_t d = ccr[k] - half;

      if (d < 0)
      {
        d = -d;
      }
      if (d < best)
      {
        best = d;
        hz_sel = k;
      }
    }
  }

  tim->CCR1 = (uint32_t)ccr[0];
  tim->CCR2 = (uint32_t)ccr[1];
  tim->CCR3 = (uint32_t)ccr[2];

  ccer = tim->CCER;
  ccer &= ~(en[0] | en[1] | en[2]);
  for (k = 0U; k < 3U; k++)
  {
    if (k != hz_sel)
    {
      ccer |= en[k];
    }
  }
  tim->CCER = ccer;
}

static void motor_pwm_clear_break(TIM_TypeDef *tim)
{
  tim->SR &= ~TIM_SR_BIF;
}

static void motor_pwm_bkin_pin(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF2_TIM1;
  gpio.Pin = BOARD_PWM_BKIN_PIN;
  HAL_GPIO_Init(BOARD_PWM_BKIN_PORT, &gpio);
}

static int32_t s_trip_phase = -1;
static int32_t s_trip_lsb;

static void motor_pwm_note_trip(uint8_t kind)
{
  motor_hall6_snapshot_t snap;
  uint32_t ctx;

  if (g_motor_pwm_prot.ocp_hold != 0U)
  {
    return;
  }
  MotorHall6_GetSnapshot(&snap);
  ctx = (uint32_t)snap.hall_raw;
  if (snap.kick != 0U)
  {
    ctx |= 0x100U;
  }
  g_reset_diag[RESET_DIAG_OCP_KIND] = kind;
  g_reset_diag[RESET_DIAG_OCP_PHASE] = (uint32_t)s_trip_phase;
  g_reset_diag[RESET_DIAG_OCP_LSB] = (uint32_t)s_trip_lsb;
  g_reset_diag[RESET_DIAG_OCP_CTX] = ctx;
  g_reset_diag[RESET_DIAG_OCP_TICK] = HAL_GetTick();
  g_motor_pwm_prot.ocp_kind = kind;
  g_motor_pwm_prot.ocp_hold = 1U;
}

static void motor_pwm_trip(uint8_t sw)
{
  if (g_motor_pwm_prot.latched == 0U)
  {
    if (sw != 0U)
    {
      g_motor_pwm_prot.sw_trips++;
    }
    motor_pwm_note_trip(sw != 0U ? 1U : 2U);
    g_motor_pwm_prot.latched = 1U;
  }
#if defined(MOTOR_TRACE) && (MOTOR_TRACE != 0)
  /* The host sees the fault a serial poll later. Stop the trace here so
   * the last window still contains the samples that crossed the limit. */
  g_motor_trace.mode = MOTOR_TRACE_MODE_IDLE;
#endif
  MotorPwm_HardwareSafe();
}

uint8_t MotorPwm_IsLatched(void)
{
  return g_motor_pwm_prot.latched;
}

void MotorPwm_PollOcpZero(void)
{
#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
  uint8_t k;
  uint8_t calm = 1U;
  int32_t sample[3];
  const uint16_t *raw = (const uint16_t *)g_motor_adc_raw;
  const uint8_t moe = ((TIM1->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U;

  if (g_motor_pwm_prot.ocp_zero_ready != 0U)
  {
    return;
  }
  if (moe != 0U)
  {
    s_ocp_zero_n = 0U;
    s_ocp_zero_sum[0] = 0;
    s_ocp_zero_sum[1] = 0;
    s_ocp_zero_sum[2] = 0;
    return;
  }

  for (k = 0U; k < 3U; k++)
  {
    sample[k] = (int32_t)raw[k] - 2048;
    /* A real current, or a rail stuck at 0 or full scale, is not a zero. */
    if ((sample[k] <= -100) || (sample[k] >= 100))
    {
      calm = 0U;
    }
  }
  if (calm == 0U)
  {
    s_ocp_zero_n = 0U;
    s_ocp_zero_sum[0] = 0;
    s_ocp_zero_sum[1] = 0;
    s_ocp_zero_sum[2] = 0;
    return;
  }

  s_ocp_zero_sum[0] += sample[0];
  s_ocp_zero_sum[1] += sample[1];
  s_ocp_zero_sum[2] += sample[2];
  s_ocp_zero_n++;
  if (s_ocp_zero_n >= 64U)
  {
    g_motor_pwm_prot.ocp_zero[0] = (int16_t)(s_ocp_zero_sum[0] / 64);
    g_motor_pwm_prot.ocp_zero[1] = (int16_t)(s_ocp_zero_sum[1] / 64);
    g_motor_pwm_prot.ocp_zero[2] = (int16_t)(s_ocp_zero_sum[2] / 64);
    g_motor_pwm_prot.ocp_zero_ready = 1U;
  }
#else
  g_motor_pwm_prot.ocp_zero_ready = 1U;
#endif
}

#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
#pragma GCC push_options
#pragma GCC optimize ("Os")
/* Three phases, straight line. The old loop was 200 cycles of reloads. */
static inline int32_t motor_pwm_poll_abs(unsigned k, const uint16_t *raw,
                                        uint8_t zero_ready)
{
  int32_t i = (int32_t)raw[k] - 2048;

  if (zero_ready != 0U)
  {
    i -= (int32_t)g_motor_pwm_prot.ocp_zero[k];
  }
  g_motor_pwm_prot.i_lsb[k] = (int16_t)i;
  return (i < 0) ? -i : i;
}

static void motor_pwm_poll_oc(void) __attribute__((optimize("Os")));
static void motor_pwm_poll_oc(void)
{
  const uint16_t *raw;
  uint8_t zero_ready;
  int32_t a;
  int32_t b;
  int32_t c;

  if (g_motor_pwm_prot.latched != 0U)
  {
    return;
  }
  raw = (const uint16_t *)g_motor_adc_raw;
  zero_ready = g_motor_pwm_prot.ocp_zero_ready;
  a = motor_pwm_poll_abs(0U, raw, zero_ready);
  b = motor_pwm_poll_abs(1U, raw, zero_ready);
  c = motor_pwm_poll_abs(2U, raw, zero_ready);
  if ((a <= (int32_t)MOTOR_PWM_OCP_LSB) && (b <= (int32_t)MOTOR_PWM_OCP_LSB) &&
      (c <= (int32_t)MOTOR_PWM_OCP_LSB))
  {
    g_motor_pwm_prot.ocp_hits = 0U;
    return;
  }
  if (g_motor_pwm_prot.ocp_hits < 255U)
  {
    g_motor_pwm_prot.ocp_hits++;
  }
  if (g_motor_pwm_prot.ocp_hits >= (uint8_t)MOTOR_PWM_OCP_TICKS)
  {
    int32_t best = -1;
    uint8_t phase = 0U;

    if (a > (int32_t)MOTOR_PWM_OCP_LSB)
    {
      best = a;
      phase = 0U;
    }
    if ((b > (int32_t)MOTOR_PWM_OCP_LSB) && (b > best))
    {
      best = b;
      phase = 1U;
    }
    if ((c > (int32_t)MOTOR_PWM_OCP_LSB) && (c > best))
    {
      best = c;
      phase = 2U;
    }
    s_trip_phase = (int32_t)phase;
    s_trip_lsb = g_motor_pwm_prot.i_lsb[phase];
    motor_pwm_trip(1U);
  }
}
#pragma GCC pop_options
#endif

void MotorPwm_PollProtect(void)
{
  TIM_TypeDef *tim = TIM1;

#if MOTOR_PWM_BKIN
  if ((tim->SR & TIM_SR_BIF) != 0U)
  {
    if (g_motor_pwm_prot.bkin_seen == 0U)
    {
      g_motor_pwm_prot.bkin_trips++;
      g_motor_pwm_prot.bkin_seen = 1U;
    }
    s_trip_phase = -1;
    s_trip_lsb = 0;
    motor_pwm_trip(0U);
  }
#else
  (void)tim;
#endif

  if (g_motor_pwm_force_trip != 0U)
  {
    g_motor_pwm_force_trip = 0U;
    s_trip_phase = -1;
    s_trip_lsb = 0;
    motor_pwm_trip(1U);
  }

#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
  motor_pwm_poll_oc();
#endif
}

void MotorPwm_HardwareSafe(void)
{
  TIM_TypeDef *tim = TIM1;

  /* Fault paths (NMI / HardFault / Error_Handler) may run before Init. */
  __HAL_RCC_TIM1_CLK_ENABLE();
  tim->CCR1 = 0U;
  tim->CCR2 = 0U;
  tim->CCR3 = 0U;
  tim->CCER = 0U;
  tim->BDTR &= ~TIM_BDTR_MOE;
  motor_pwm_clear_break(tim);
  s_boot_charged = 0U;
  s_boot_pending = 0U;
  /* CH4 is the boost switch. A motor fault must not drop the gate supply. */
  Boost_KeepAlive();
}

void MotorPwm_PinsIdle(void)
{
  GPIO_InitTypeDef gpio = {0};

  MotorPwm_HardwareSafe();

  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  gpio.Pin = BOARD_PWM_U_H_PIN | BOARD_PWM_U_L_PIN | BOARD_PWM_V_H_PIN |
             BOARD_PWM_W_H_PIN;
  HAL_GPIO_Init(GPIOA, &gpio);
  HAL_GPIO_WritePin(GPIOA, gpio.Pin, GPIO_PIN_RESET);

  gpio.Pin = BOARD_PWM_V_L_PIN | BOARD_PWM_W_L_PIN;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, gpio.Pin, GPIO_PIN_RESET);
}

void MotorPwm_RestoreAfPins(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF2_TIM1;

  gpio.Pin = BOARD_PWM_U_H_PIN | BOARD_PWM_V_H_PIN | BOARD_PWM_W_H_PIN;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = BOARD_PWM_U_L_PIN;
  HAL_GPIO_Init(BOARD_PWM_U_L_PORT, &gpio);

  gpio.Pin = BOARD_PWM_V_L_PIN | BOARD_PWM_W_L_PIN;
  HAL_GPIO_Init(GPIOB, &gpio);

  motor_pwm_bkin_pin();
}

void MotorPwm_Init(void)
{
  TIM_MasterConfigTypeDef master = {0};
  TIM_OC_InitTypeDef oc = {0};
  TIM_BreakDeadTimeConfigTypeDef bdt = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0U;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = MOTOR_TIM1_ARR;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  /* Center-aligned counting raises an update event at both overflow and
   * underflow, so RCR=1 is needed for one control tick per PWM period
   * (MOTOR_CTRL_ISR_HZ). RCR=0 ran the control loop at twice MOTOR_PWM_HZ. */
  htim1.Init.RepetitionCounter = 1U;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  (void)HAL_TIMEx_MasterConfigSynchronization(&htim1, &master);

  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0U;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  /* Pin must be AF2 before BKE: a GPIO/floating PA6 with BKP=low is what
   * locked MOE on TARS (nFAULT had nowhere to go). Here nFAULT is the
   * window comparator through JP2, active low, 4k7 to +3V3. */
  motor_pwm_bkin_pin();

  bdt.OffStateRunMode = TIM_OSSR_ENABLE;
  bdt.OffStateIDLEMode = TIM_OSSI_ENABLE;
  bdt.LockLevel = TIM_LOCKLEVEL_OFF;
  bdt.DeadTime = MOTOR_TIM1_DT;
#if MOTOR_PWM_BKIN
  bdt.BreakState = TIM_BREAK_ENABLE;
#else
  bdt.BreakState = TIM_BREAK_DISABLE;
#endif
  bdt.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  bdt.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bdt) != HAL_OK)
  {
    Error_Handler();
  }

  motor_pwm_disable_oc_preload(htim1.Instance);
  motor_pwm_clear_break(htim1.Instance);
  /* One update event per PWM period (center-aligned), like TARS TIM1 TRGO. */
  htim1.Instance->CR1 |= TIM_CR1_URS;
  MotorPwm_HardwareSafe();
}

void MotorPwm_RunCounter(void)
{
  if ((htim1.Instance->CR1 & TIM_CR1_CEN) == 0U)
  {
    (void)HAL_TIM_Base_Start(&htim1);
  }
}

static int motor_pwm_start_channels(void)
{
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    return 0;
  }

  (void)HAL_TIM_Base_Start(&htim1);
  return 1;
}

int MotorPwm_ArmOutputs(void)
{
  /* V_BOOST (PA5) is in range and nFAULT is high; see mainboard-spec §9.5.5. */
  if (Boost_Ready() == 0U)
  {
    return 0;
  }

  MotorPwm_RestoreAfPins();

  /* Recover HAL channel state after PinsIdle or a partial stop. */
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
  MotorPwm_HardwareSafe();
  /* PWM_Stop halted TIM1. TIM3 kept running, so the ADC edge has slipped. */
  Boost_Resync();

  return motor_pwm_start_channels();
}

void MotorPwm_ClearOcpHold(void)
{
  g_motor_pwm_prot.ocp_hold = 0U;
  g_motor_pwm_prot.ocp_kind = 0U;
  g_motor_pwm_prot.latched = 0U;
  g_motor_pwm_prot.bkin_seen = 0U;
  g_motor_pwm_prot.ocp_hits = 0U;
}

void MotorPwm_MoeEnable(void)
{
  if (g_motor_pwm_prot.ocp_hold != 0U)
  {
    return;
  }
#if MOTOR_PWM_BKIN
  if (HAL_GPIO_ReadPin(BOARD_NFAULT_PORT, BOARD_NFAULT_PIN) == GPIO_PIN_RESET)
  {
    return;
  }
#endif

  g_motor_pwm_prot.latched = 0U;
  g_motor_pwm_prot.bkin_seen = 0U;
  g_motor_pwm_prot.ocp_hits = 0U;
  motor_pwm_clear_break(htim1.Instance);
  __HAL_TIM_MOE_ENABLE(&htim1);
}

int MotorPwm_BootstrapReady(void)
{
  if (s_boot_pending == 0U)
  {
    return 1;
  }
  if ((HAL_GetTick() - s_boot_t0) < MOTOR_PWM_BOOT_MS)
  {
    return 0;
  }
  if (((htim1.Instance->BDTR & TIM_BDTR_MOE) == 0U) ||
      (g_motor_pwm_prot.latched != 0U))
  {
    s_boot_pending = 0U;
    return 0;
  }
  s_boot_pending = 0U;
  s_boot_charged = 1U;
  return 1;
}

void MotorPwm_ChargeBootstrap(void)
{
  if (g_motor_pwm_prot.ocp_hold != 0U)
  {
    return;
  }
  if ((s_boot_charged != 0U) || (s_boot_pending != 0U))
  {
    return;
  }
#if MOTOR_PWM_BKIN
  if (HAL_GPIO_ReadPin(BOARD_NFAULT_PORT, BOARD_NFAULT_PIN) == GPIO_PIN_RESET)
  {
    return;
  }
#endif

  /* VDD is 12 V. The external 12 V boost cannot lift HB once a phase
   * node is at the 24 V bus, so each driver has to charge from VDD
   * while its own low side holds that node at ground. */
  g_motor_pwm_prot.latched = 0U;
  MotorPwm_SetPhase(MOTOR_PHASE_LOW, MOTOR_PHASE_LOW, MOTOR_PHASE_LOW, 0U);
  MotorPwm_MoeEnable();
  if ((htim1.Instance->BDTR & TIM_BDTR_MOE) == 0U)
  {
    return;
  }

  s_boot_t0 = HAL_GetTick();
  s_boot_pending = 1U;
  if (__get_IPSR() != 0U)
  {
    return;
  }
  while (MotorPwm_BootstrapReady() == 0)
  {
    IWDG->KR = 0xAAAAU;
    if (s_boot_pending == 0U)
    {
      return;
    }
  }
}

int MotorPwm_Start(void)
{
  if (MotorPwm_ArmOutputs() == 0)
  {
    return 0;
  }

  MotorPwm_ChargeBootstrap();
  return ((htim1.Instance->BDTR & TIM_BDTR_MOE) != 0U) ? 1 : 0;
}

void MotorPwm_Stop(void)
{
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
  MotorPwm_HardwareSafe();
  Boost_Resync();
}
