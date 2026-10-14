#ifndef FOC_BENCH_H
#define FOC_BENCH_H

#include <stdint.h>

#define MOTOR_FOC_BENCH_CASES 4U
#define MOTOR_FOC_BENCH_RUNS   10U
#define MOTOR_FOC_BENCH_DONE_TAG 0xA2B00001U

typedef struct
{
  uint32_t tag;
  uint32_t overhead_cycles;
  uint32_t nop100_median;
  uint32_t nop100_expected;
  uint32_t foc_median[MOTOR_FOC_BENCH_CASES];
  uint32_t foc_max[MOTOR_FOC_BENCH_CASES];
  uint32_t foc_min[MOTOR_FOC_BENCH_CASES];
} MotorFocBenchResult;

extern volatile MotorFocBenchResult g_foc_bench_result;

void MotorFocBench_Run(void);

#endif /* FOC_BENCH_H */
