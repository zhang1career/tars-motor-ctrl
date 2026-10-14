#include "ctrl_bench.h"
#include "motor_cycles.h"
#include "motor_tick.h"
#include "motor_pwm.h"
#include "stm32f0xx.h"

#if defined(MOTOR_ANGLE) && (MOTOR_ANGLE != 0)
#include "motor_angle.h"
#endif
#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
#include "motor_adc.h"
#endif
#if defined(MOTOR_TRACE) && (MOTOR_TRACE != 0)
#include "motor_trace.h"
#endif
#if defined(MOTOR_FOC) && (MOTOR_FOC != 0)
#include "motor_foc.h"
#include "motor_foc_fx.h"
#endif

volatile MotorCtrlBenchResult g_ctrl_bench;
volatile MotorFocFxCmp g_foc_fx_cmp;

/* Sinks so the optimiser cannot delete the work being timed. */
static volatile int32_t s_sink;
static volatile float s_fsink;
static volatile uint8_t s_edge_hall;

/*
 * Exactly 100 NOP instructions, no loop. A loop with a volatile counter is not
 * a calibration: each iteration costs load/compare/branch/nop/load/add/store,
 * so 100 iterations read ~1600 cycles and look like a broken counter. Straight
 * NOPs give a known 100 instructions, and whatever the reading exceeds 100 by is
 * the flash wait-state penalty at 48 MHz (FLASH_LATENCY_1).
 */
static void bench_nop100(void)
{
  __asm volatile(".rept 100\n\tnop\n\t.endr");
}

static void bench_angle_steady(void)
{
#if defined(MOTOR_ANGLE) && (MOTOR_ANGLE != 0)
  MotorAngle_Update(6U);
#endif
}

static void bench_angle_edge(void)
{
#if defined(MOTOR_ANGLE) && (MOTOR_ANGLE != 0)
  /* Codes 6 and 2 are adjacent sectors, so each call is a legal one-sector
   * transition and takes the edge path including its division. */
  s_edge_hall = (s_edge_hall == 6U) ? 2U : 6U;
  MotorAngle_Update(s_edge_hall);
#endif
}

static void bench_adc_read(void)
{
#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
  int32_t iu = MotorAdc_ShuntLsb(MOTOR_ADC_IU);
  int32_t iv = MotorAdc_ShuntLsb(MOTOR_ADC_IV);
  int32_t iw = MotorAdc_ShuntLsb(MOTOR_ADC_IW);

  s_sink = iu + iv + iw;
#endif
}

static void bench_trace_push(void)
{
#if defined(MOTOR_TRACE) && (MOTOR_TRACE != 0)
  MotorTrace_Push(1, 2, 3, 4);
#endif
}

/*
 * 100 chained single-precision multiplies. The point is to have a measured
 * cost-per-soft-float-operation on this part instead of a guessed one: an
 * assumed 35 cycles produced a 1700-cycle estimate for a FOC step that actually
 * takes 16000. A few cycles per iteration of the result are loop overhead.
 */
static void bench_float_mul100(void)
{
  volatile float a = 1.0000001f;
  float x = 1.0f;
  uint8_t k;

  for (k = 0U; k < 100U; k++)
  {
    x *= a;
  }
  s_fsink = x;
}

static void bench_foc_step(void)
{
#if defined(MOTOR_FOC) && (MOTOR_FOC != 0)
  uint8_t k;

  /* Step() only does the work every MOTOR_FOC_DECIM calls, so call it that many
   * times to time exactly one full pass. */
  for (k = 0U; k < (uint8_t)MOTOR_FOC_DECIM; k++)
  {
    MotorFoc_Step();
  }
#endif
}

static void bench_foc_fx_step(void)
{
#if defined(MOTOR_FOC) && (MOTOR_FOC != 0)
  MotorFocFx_Step();
#endif
}

#if defined(MOTOR_FOC) && (MOTOR_FOC != 0)
static int32_t iabs32(int32_t x)
{
  return (x < 0) ? -x : x;
}

static void foc_fx_compare(void)
{
  static const uint16_t adc_iu[MOTOR_FOC_FX_CMP_N] =
      {2048U, 2814U, 2814U, 2814U, 2814U, 1282U, 2300U, 2048U};
  static const uint16_t adc_iv[MOTOR_FOC_FX_CMP_N] =
      {2048U, 1282U, 1282U, 1282U, 1282U, 2814U, 2300U, 2814U};
  static const uint16_t adc_iw[MOTOR_FOC_FX_CMP_N] =
      {2048U, 2048U, 2048U, 2048U, 2048U, 2048U, 1544U, 1282U};
  static const uint16_t theta[MOTOR_FOC_FX_CMP_N] =
      {0U, 0U, 8192U, 16384U, 40000U, 8192U, 24576U, 49152U};
  uint8_t i;

  g_foc_fx_cmp.tag = 0U;
  g_foc_fx_cmp.nfail = 0U;

  MotorFoc_Init();
  MotorFoc_SetMode(MOTOR_FOC_OBSERVE);
  MotorFocFx_Init();
  MotorFocFx_SetMode(MOTOR_FOC_FX_OBSERVE);

  for (i = 0U; i < MOTOR_FOC_FX_CMP_N; i++)
  {
    int32_t id_x_ma;
    int32_t iq_x_ma;
    int32_t d_id;
    int32_t d_iq;

    g_motor_adc_raw[0] = adc_iu[i];
    g_motor_adc_raw[1] = adc_iv[i];
    g_motor_adc_raw[2] = adc_iw[i];
    g_motor_adc_raw[3] = 3024U;
    g_motor_angle.theta = theta[i];

    MotorFoc_Step();
    MotorFocFx_Step();

    id_x_ma = (g_motor_foc_fx.id_lsb * 1617) / 1000;
    iq_x_ma = (g_motor_foc_fx.iq_lsb * 1617) / 1000;
    d_id = id_x_ma - (int32_t)g_motor_foc.id_ma;
    d_iq = iq_x_ma - (int32_t)g_motor_foc.iq_ma;

    g_foc_fx_cmp.id_f_ma[i] = (int32_t)g_motor_foc.id_ma;
    g_foc_fx_cmp.iq_f_ma[i] = (int32_t)g_motor_foc.iq_ma;
    g_foc_fx_cmp.id_x_ma[i] = id_x_ma;
    g_foc_fx_cmp.iq_x_ma[i] = iq_x_ma;
    g_foc_fx_cmp.d_id_ma[i] = d_id;
    g_foc_fx_cmp.d_iq_ma[i] = d_iq;

    if ((iabs32(d_id) > 4) || (iabs32(d_iq) > 4))
    {
      g_foc_fx_cmp.nfail++;
    }
  }

  MotorFoc_SetMode(MOTOR_FOC_OFF);
  MotorFocFx_SetMode(MOTOR_FOC_FX_OFF);
  g_foc_fx_cmp.tag = MOTOR_FOC_FX_CMP_TAG;
}
#endif

static uint32_t median_u16(uint16_t *v, uint8_t count)
{
  uint8_t i;
  uint8_t j;
  uint16_t tmp;

  for (i = 1U; i < count; i++)
  {
    tmp = v[i];
    j = i;
    while ((j > 0U) && (v[j - 1U] > tmp))
    {
      v[j] = v[j - 1U];
      j--;
    }
    v[j] = tmp;
  }

  if ((count & 1U) != 0U)
  {
    return (uint32_t)v[count / 2U];
  }
  return (uint32_t)(((uint32_t)v[(count / 2U) - 1U] + (uint32_t)v[count / 2U]) / 2U);
}

static void bench_case(uint8_t idx, void (*fn)(void))
{
  uint16_t samples[MOTOR_CTRL_BENCH_RUNS];
  uint8_t run;
  uint32_t worst = 0U;

  for (run = 0U; run < MOTOR_CTRL_BENCH_RUNS; run++)
  {
    samples[run] = MotorCycles_Measure(fn);
    if ((uint32_t)samples[run] > worst)
    {
      worst = (uint32_t)samples[run];
    }
  }

  g_ctrl_bench.median[idx] = median_u16(samples, MOTOR_CTRL_BENCH_RUNS);
  g_ctrl_bench.max[idx] = worst;
}

void MotorCtrlBench_Run(void)
{
  uint16_t nop[8];
  uint8_t i;

  g_ctrl_bench.tag = 0U;
  MotorCycles_Init();

#if defined(MOTOR_ANGLE) && (MOTOR_ANGLE != 0)
  /* Prime the estimator so the steady case really takes the no-edge path and
   * the edge case has a previous sector to compare against. */
  MotorAngle_Reset();
  MotorAngle_Update(6U);
  for (i = 0U; i < 20U; i++)
  {
    MotorAngle_Update(6U);
  }
  MotorAngle_Update(2U);
  for (i = 0U; i < 20U; i++)
  {
    MotorAngle_Update(2U);
  }
  MotorAngle_Update(6U);
  s_edge_hall = 6U;
#endif

  g_ctrl_bench.overhead_cycles = (uint32_t)MotorCycles_Overhead();
  g_ctrl_bench.tick_budget = MOTOR_CYCLES_HZ / MOTOR_CTRL_ISR_HZ;

  for (i = 0U; i < 8U; i++)
  {
    nop[i] = MotorCycles_Measure(bench_nop100);
  }
  g_ctrl_bench.nop100_median = median_u16(nop, 8U);

  bench_case(MOTOR_CTRL_BENCH_ANGLE_STEADY, bench_angle_steady);
  bench_case(MOTOR_CTRL_BENCH_ANGLE_EDGE, bench_angle_edge);
  bench_case(MOTOR_CTRL_BENCH_ADC_READ, bench_adc_read);
  bench_case(MOTOR_CTRL_BENCH_TRACE_PUSH, bench_trace_push);
  bench_case(MOTOR_CTRL_BENCH_FMUL100, bench_float_mul100);

#if defined(MOTOR_FOC) && (MOTOR_FOC != 0)
  /* Closed-current mode is the worst case. Safe with MOE clear: the duty writes
   * land in CCR registers whose outputs are disabled. */
  MotorFoc_Init();
  MotorFoc_SetIqRefMa(500);
  MotorFoc_SetMode(MOTOR_FOC_CURRENT);
  bench_case(MOTOR_CTRL_BENCH_FOC_STEP, bench_foc_step);
  MotorFoc_SetMode(MOTOR_FOC_OFF);

  /* 500 mA / 1.617 mA per LSB = 309 LSB. */
  MotorFocFx_Init();
  MotorFocFx_SetIqRefLsb(309);
  MotorFocFx_SetMode(MOTOR_FOC_FX_CURRENT);
  bench_case(MOTOR_CTRL_BENCH_FOC_FX_STEP, bench_foc_fx_step);
  MotorFocFx_SetMode(MOTOR_FOC_FX_OFF);

  foc_fx_compare();

  /* SetDuties / SetDutiesCcr enabled the output-compare bits on the way
   * through; leave the timer as the pre-flight check expects to find it. */
  MotorPwm_HardwareSafe();
#endif

#if defined(MOTOR_ANGLE) && (MOTOR_ANGLE != 0)
  /* The edge case left the estimator full of synthetic transitions. */
  MotorAngle_Reset();
#endif
#if defined(MOTOR_TRACE) && (MOTOR_TRACE != 0)
  MotorTrace_Stop();
#endif

  g_ctrl_bench.tag = MOTOR_CTRL_BENCH_DONE_TAG;
}
