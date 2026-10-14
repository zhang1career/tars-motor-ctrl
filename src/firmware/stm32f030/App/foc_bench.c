#include "foc_bench.h"
#include "motor_cycles.h"
#include "foc_step_stm32.h"
#include "foc_step_stm32_initialize.h"
#include "stm32f0xx.h"

typedef struct
{
  float ia;
  float ib;
  float ic;
  float vdc;
  float speed_ref_rpm;
  float enable;
} FocBenchCase;

static FocBenchCase s_case;
static float s_da;
static float s_db;
static float s_dc;
static float s_theta;
static float s_speed;
static float s_id;
static float s_iq;

volatile MotorFocBenchResult g_foc_bench_result;

static void foc_bench_step_once(void)
{
  foc_step_stm32(s_case.ia, s_case.ib, s_case.ic, s_case.vdc, s_case.speed_ref_rpm,
                 s_case.enable, &s_da, &s_db, &s_dc, &s_theta, &s_speed, &s_id, &s_iq);
}

/* Exactly 100 NOP instructions. A loop with a volatile counter costs ~16 cycles
 * per iteration, so it reads ~1600 and looks like a broken counter -- this used
 * to be reported against an expectation of 100. */
static void nop100_loop(void)
{
  __asm volatile(".rept 100\n\tnop\n\t.endr");
}

static uint32_t median_u16_samples(uint16_t *v, uint8_t count)
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

static void bench_one_case(uint8_t case_idx, const FocBenchCase *c)
{
  uint16_t samples[MOTOR_FOC_BENCH_RUNS];
  uint8_t run;
  uint32_t max_v = 0U;
  uint32_t min_v = 0xFFFFFFFFU;

  s_case = *c;
  for (run = 0U; run < MOTOR_FOC_BENCH_RUNS; run++)
  {
    samples[run] = MotorCycles_Measure(foc_bench_step_once);
    if ((uint32_t)samples[run] > max_v)
    {
      max_v = (uint32_t)samples[run];
    }
    if ((uint32_t)samples[run] < min_v)
    {
      min_v = (uint32_t)samples[run];
    }
  }

  g_foc_bench_result.foc_median[case_idx] =
      median_u16_samples(samples, MOTOR_FOC_BENCH_RUNS);
  g_foc_bench_result.foc_max[case_idx] = max_v;
  g_foc_bench_result.foc_min[case_idx] = min_v;
}

void MotorFocBench_Run(void)
{
  static const FocBenchCase cases[MOTOR_FOC_BENCH_CASES] = {
      {0.10f, -0.05f, -0.05f, 12.0f, 360.0f, 1.0f},
      {0.50f, -0.25f, -0.25f, 12.0f, 360.0f, 1.0f},
      {0.30f, -0.15f, -0.15f, 12.0f, 1500.0f, 1.0f},
      {0.00f, 0.00f, 0.00f, 12.0f, 0.0f, 0.0f},
  };
  uint16_t nop_samples[8];
  uint8_t i;

  g_foc_bench_result.tag = 0U;
  MotorCycles_Init();
  foc_step_stm32_initialize();

  g_foc_bench_result.overhead_cycles = (uint32_t)MotorCycles_Overhead();
  g_foc_bench_result.nop100_expected = 100U;

  for (i = 0U; i < 8U; i++)
  {
    nop_samples[i] = MotorCycles_Measure(nop100_loop);
  }
  g_foc_bench_result.nop100_median = median_u16_samples(nop_samples, 8U);

  for (i = 0U; i < MOTOR_FOC_BENCH_CASES; i++)
  {
    bench_one_case(i, &cases[i]);
  }

  g_foc_bench_result.tag = MOTOR_FOC_BENCH_DONE_TAG;
}
