#include "motor_vsweep.h"

#include "motor_app.h"
#include "motor_hall.h"
#include "motor_pwm.h"
#include "motor_tick.h"
#include "tnb_motor.h"
#include "main.h"

#if defined(MOTOR_TRACE) && (MOTOR_TRACE != 0) && defined(MOTOR_ADC) && (MOTOR_ADC != 0)
#include "motor_adc.h"
#include "motor_trace.h"

/* 80 counts * 20 mV = 1.6 V commanded, past the ~0.72 V dead-time knee
 * and into the resistive slope. 31 LSB * 16.17 mA = 0.50 A. */
#define VSWEEP_CCR_MAX  80
#define VSWEEP_CCR_STEP 4
#define VSWEEP_I_LIMIT  31
#define VSWEEP_SETTLE   4U
#define VSWEEP_AVG_N    8U
#define VSWEEP_GAP_MS   12U

volatile uint8_t g_vsweep_req;
volatile uint8_t g_vsweep_state;
volatile uint16_t g_vsweep_n;
volatile uint8_t g_vsweep_hall_begin;
volatile uint8_t g_vsweep_hall_end;
volatile int16_t g_vsweep_zero[3];

/* Host hold: one pair stays on until g_vhold_on is cleared. 0 U, 1 V, 2 W.
 * The halfword comes first so it stays aligned; an odd address faults the M0. */
volatile uint16_t g_vhold_ccr;
volatile uint8_t g_vhold_on;
volatile uint8_t g_vhold_pwm;
volatile uint8_t g_vhold_low;
volatile uint8_t g_vhold_tick;

static uint16_t s_vhold_ccr;
static uint8_t s_vhold_on;
static uint8_t s_vhold_pwm;
static uint8_t s_vhold_low;

static void vhold_off(void)
{
  MotorPwm_SetPhase(MOTOR_PHASE_OFF, MOTOR_PHASE_OFF, MOTOR_PHASE_OFF, 0U);
  TnbMotor_RequestStop();
  s_vhold_on = 0U;
}

static void vhold_poll(void)
{
  if ((g_vhold_on == 1U) &&
      ((s_vhold_ccr != g_vhold_ccr) || (s_vhold_pwm != g_vhold_pwm) ||
       (s_vhold_low != g_vhold_low)))
  {
    s_vhold_on = 0U;
  }
  if (g_vhold_on == 1U)
  {
    /* The stop request is handled after this poll and turns the bridge back
     * off. Arm once to raise that request, then arm again after it has run. */
    if (s_vhold_on < 3U)
    {
      if (s_vhold_on == 0U)
      {
        g_motor_pwm_dt_on = 0U;
        TnbMotor_RequestStop();
      }
      if (MotorPwm_ArmOutputs() == 0)
      {
        g_vhold_on = 0U;
        vhold_off();
        return;
      }
      MotorPwm_ChargeBootstrap();
      if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
      {
        g_vhold_on = 0U;
        vhold_off();
        return;
      }
      if (g_vhold_pwm == g_vhold_low)
      {
        /* Same CCR on every leg: line voltage stays near 0, rotor stays put.
         * Clamp to ARR. Dead time is unchanged; a CCR above the dead-time
         * count is what lets the high side conduct. */
        uint32_t pulse = g_vhold_ccr;

        if (pulse > TIM1->ARR)
        {
          pulse = TIM1->ARR;
        }
        MotorPwm_SetPhase(MOTOR_PHASE_PWM, MOTOR_PHASE_PWM, MOTOR_PHASE_PWM, 0U);
        TIM1->CCR1 = pulse;
        TIM1->CCR2 = pulse;
        TIM1->CCR3 = pulse;
        if (g_vhold_tick == 1U)
        {
          MotorTick_Start();
        }
      }
      else
      {
        MotorPwm_SetPairCcr(g_vhold_pwm, g_vhold_low, g_vhold_ccr);
      }
      MotorPwm_MoeEnable();
      if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
      {
        g_vhold_on = 0U;
        vhold_off();
        return;
      }
      s_vhold_on++;
      s_vhold_pwm = g_vhold_pwm;
      s_vhold_low = g_vhold_low;
      s_vhold_ccr = g_vhold_ccr;
    }
    return;
  }
  if (s_vhold_on != 0U)
  {
    vhold_off();
  }
}

static void kick_wdg(void)
{
  IWDG->KR = 0xAAAAU;
}

static void delay_kick(uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();

  while ((HAL_GetTick() - t0) < ms)
  {
    kick_wdg();
  }
}

static int16_t avg_phase(uint8_t idx)
{
  int32_t sum = 0;
  uint8_t n;

  for (n = 0U; n < VSWEEP_AVG_N; n++)
  {
    sum += (int32_t)MotorAdc_ShuntLsb(idx) - (int32_t)g_vsweep_zero[idx];
    delay_kick(1U);
  }
  sum /= (int32_t)VSWEEP_AVG_N;
  if (sum > 32767)
  {
    sum = 32767;
  }
  else if (sum < -32768)
  {
    sum = -32768;
  }
  return (int16_t)sum;
}

static void capture_zero(void)
{
  int32_t sum[3] = {0, 0, 0};
  uint8_t n;
  uint8_t k;

  for (n = 0U; n < VSWEEP_AVG_N; n++)
  {
    for (k = 0U; k < 3U; k++)
    {
      sum[k] += MotorAdc_ShuntLsb(k);
    }
    delay_kick(1U);
  }
  for (k = 0U; k < 3U; k++)
  {
    g_vsweep_zero[k] = (int16_t)(sum[k] / (int32_t)VSWEEP_AVG_N);
  }
}

static void push_point(uint16_t tag, int16_t iu, int16_t iv, int16_t iw)
{
  if (g_vsweep_n >= (uint16_t)MOTOR_TRACE_DEPTH)
  {
    return;
  }
  g_motor_trace_buf[g_vsweep_n][0] = (int16_t)tag;
  g_motor_trace_buf[g_vsweep_n][1] = iu;
  g_motor_trace_buf[g_vsweep_n][2] = iv;
  g_motor_trace_buf[g_vsweep_n][3] = iw;
  g_vsweep_n++;
  g_motor_trace.write_idx = g_vsweep_n;
}

static void finish(uint8_t state)
{
  MotorPwm_SetPhase(MOTOR_PHASE_OFF, MOTOR_PHASE_OFF, MOTOR_PHASE_OFF, 0U);
  TnbMotor_RequestStop();
  g_vsweep_hall_end = MotorHall_ReadRaw();
  g_motor_trace.mode = MOTOR_TRACE_MODE_IDLE;
  g_motor_trace.wrapped = 0U;
  g_motor_trace.source = MOTOR_TRACE_SRC_VSWEEP;
  g_motor_trace.write_idx = g_vsweep_n;
  g_vsweep_state = state;
}

static void run(void)
{
  static const uint8_t pair_a[3] = {0U, 1U, 2U};
  static const uint8_t pair_b[3] = {1U, 2U, 0U};
  uint8_t pair;
  uint8_t swap;
  uint16_t slot;

  g_vsweep_state = MOTOR_VSWEEP_RUN;
  g_vsweep_n = 0U;
  g_motor_pwm_dt_on = 0U;
  TnbMotor_RequestStop();
  g_vsweep_hall_begin = MotorHall_ReadRaw();

  g_motor_trace.mode = MOTOR_TRACE_MODE_IDLE;
  g_motor_trace.source = MOTOR_TRACE_SRC_VSWEEP;
  g_motor_trace.wrapped = 0U;
  g_motor_trace.write_idx = 0U;
  g_motor_trace.decim = 1U;
  for (slot = 0U; slot < (uint16_t)MOTOR_TRACE_DEPTH; slot++)
  {
    g_motor_trace_buf[slot][0] = 0;
    g_motor_trace_buf[slot][1] = 0;
    g_motor_trace_buf[slot][2] = 0;
    g_motor_trace_buf[slot][3] = 0;
  }

  if (MotorPwm_ArmOutputs() == 0)
  {
    finish(MOTOR_VSWEEP_ABORT);
    return;
  }
  MotorTick_Start();
  capture_zero();
  MotorPwm_ChargeBootstrap();
  if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
  {
    finish(MOTOR_VSWEEP_ABORT);
    return;
  }

  for (pair = 0U; pair < 3U; pair++)
  {
    for (swap = 0U; swap < 2U; swap++)
    {
      uint8_t pwm = (swap == 0U) ? pair_a[pair] : pair_b[pair];
      uint8_t low = (swap == 0U) ? pair_b[pair] : pair_a[pair];
      int ccr;

      for (ccr = 0; ccr <= VSWEEP_CCR_MAX; ccr += VSWEEP_CCR_STEP)
      {
        int16_t iu;
        int16_t iv;
        int16_t iw;
        int16_t peak;
        uint16_t tag;

        if (g_motor_pwm_prot.latched != 0U)
        {
          finish(MOTOR_VSWEEP_ABORT);
          return;
        }
        if (g_vsweep_n >= (uint16_t)MOTOR_TRACE_DEPTH)
        {
          finish(MOTOR_VSWEEP_ABORT);
          return;
        }

        MotorPwm_SetPairCcr(pwm, low, (uint16_t)ccr);
        MotorPwm_MoeEnable();
        if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
        {
          finish(MOTOR_VSWEEP_ABORT);
          return;
        }
        delay_kick(VSWEEP_SETTLE);
        iu = avg_phase(0U);
        iv = avg_phase(1U);
        iw = avg_phase(2U);

        peak = iu;
        if (iv > peak)
        {
          peak = iv;
        }
        if (iw > peak)
        {
          peak = iw;
        }
        if (-iu > peak)
        {
          peak = (int16_t)(-iu);
        }
        if (-iv > peak)
        {
          peak = (int16_t)(-iv);
        }
        if (-iw > peak)
        {
          peak = (int16_t)(-iw);
        }

        tag = (uint16_t)(ccr & 0x3FF);
        tag |= (uint16_t)((uint16_t)pair << 10);
        if (swap != 0U)
        {
          tag |= (uint16_t)(1U << 12);
        }
        if (peak >= VSWEEP_I_LIMIT)
        {
          tag |= (uint16_t)(1U << 13);
        }
        push_point(tag, iu, iv, iw);
        if (peak >= VSWEEP_I_LIMIT)
        {
          break;
        }
      }

      MotorPwm_SetPhase(MOTOR_PHASE_OFF, MOTOR_PHASE_OFF, MOTOR_PHASE_OFF, 0U);
      delay_kick(VSWEEP_GAP_MS);
    }
  }

  finish(MOTOR_VSWEEP_DONE);
}

void MotorVsweep_Poll(void)
{
  vhold_poll();
  if (s_vhold_on != 0U)
  {
    return;
  }
  if ((g_vsweep_req == 1U) && (g_vsweep_state != MOTOR_VSWEEP_RUN))
  {
    g_vsweep_req = 0U;
    run();
  }
}

#else

volatile uint8_t g_vsweep_req;
volatile uint8_t g_vsweep_state;
volatile uint16_t g_vsweep_n;
volatile uint8_t g_vsweep_hall_begin;
volatile uint8_t g_vsweep_hall_end;
volatile int16_t g_vsweep_zero[3];

void MotorVsweep_Poll(void)
{
  if (g_vsweep_req == 1U)
  {
    g_vsweep_req = 0U;
    g_vsweep_state = MOTOR_VSWEEP_ABORT;
  }
}

#endif
