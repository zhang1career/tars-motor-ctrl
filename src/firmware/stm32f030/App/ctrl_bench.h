#ifndef CTRL_BENCH_H
#define CTRL_BENCH_H

#include <stdint.h>

/*
 * Cycle cost of the pieces that make up the control ISR, measured on the target
 * with TIM16 (motor_cycles.c). Stage E picks the loop rate from these numbers
 * rather than from an estimate: the stage B estimate for the float FOC was
 * 8000-15000 cycles and the measurement came back 15000-18000.
 *
 * Runs with the motor off and no PWM, and is reached by flashing with
 * -DMOTOR_AUTO_START=ON -DMOTOR_START_BENCH=ON, so it never depends on a gdb
 * call (docs/roadmap.md 9.5).
 *
 * Budget for reference: 20 kHz on a 48 MHz M0 is 2400 cycles per tick.
 */

#define MOTOR_CTRL_BENCH_CASES 6U
#define MOTOR_CTRL_BENCH_RUNS 16U
#define MOTOR_CTRL_BENCH_DONE_TAG 0xC7B00001U

enum
{
  MOTOR_CTRL_BENCH_ANGLE_STEADY = 0U, /* MotorAngle_Update, no hall edge */
  MOTOR_CTRL_BENCH_ANGLE_EDGE = 1U,   /* MotorAngle_Update across an edge */
  MOTOR_CTRL_BENCH_ADC_READ = 2U,     /* read three shunts and form the sum */
  MOTOR_CTRL_BENCH_TRACE_PUSH = 3U,   /* MotorTrace_Push */
  /* One full FOC step in the closed-current mode, i.e. the worst case: Clarke,
   * Park, both PIs with their integrators, inverse Park, SVPWM and the duty
   * write. Includes MOTOR_FOC_DECIM-1 early returns of a few cycles each. */
  MOTOR_CTRL_BENCH_FOC_STEP = 4U,
  /* 100 chained float multiplies: the measured cost of soft float here. */
  MOTOR_CTRL_BENCH_FMUL100 = 5U
};

typedef struct
{
  uint32_t tag;
  uint32_t overhead_cycles;
  uint32_t nop100_median;   /* sanity check on the counter itself */
  uint32_t tick_budget;     /* 48 MHz / MOTOR_CTRL_ISR_HZ */
  uint32_t median[MOTOR_CTRL_BENCH_CASES];
  uint32_t max[MOTOR_CTRL_BENCH_CASES];
} MotorCtrlBenchResult;

extern volatile MotorCtrlBenchResult g_ctrl_bench;

void MotorCtrlBench_Run(void);

#endif /* CTRL_BENCH_H */
