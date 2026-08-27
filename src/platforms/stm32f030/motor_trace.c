#include "motor_trace.h"

/* Statically valid from reset, so the host can arm a capture without the
 * firmware having called Arm() first -- the open-loop diagnostic paths never do,
 * and an all-zero header just looks like "built without MOTOR_TRACE". */
volatile motor_trace_header_t g_motor_trace = {
    .magic = MOTOR_TRACE_MAGIC,
    .depth = (uint16_t)MOTOR_TRACE_DEPTH,
    .channels = (uint16_t)MOTOR_TRACE_CH,
    .decim = 1U,
    .mode = MOTOR_TRACE_MODE_IDLE,
};

/* int16_t only needs 2-byte alignment, and the linker duly placed this at an
 * odd halfword once. The host dumps it with openocd's mdw, which requires word
 * alignment, so force it. */
volatile int16_t g_motor_trace_buf[MOTOR_TRACE_DEPTH][MOTOR_TRACE_CH]
    __attribute__((aligned(4)));

void MotorTrace_Arm(uint8_t mode, uint8_t source, uint16_t decim)
{
  g_motor_trace.mode = MOTOR_TRACE_MODE_IDLE;

  g_motor_trace.magic = MOTOR_TRACE_MAGIC;
  g_motor_trace.depth = (uint16_t)MOTOR_TRACE_DEPTH;
  g_motor_trace.channels = (uint16_t)MOTOR_TRACE_CH;
  g_motor_trace.write_idx = 0U;
  g_motor_trace.decim = decim;
  g_motor_trace.decim_count = 0U;
  g_motor_trace.pushes = 0U;
  g_motor_trace.wrapped = 0U;
  g_motor_trace.source = source;
  g_motor_trace.reserved = 0U;

  /* Last, so the ISR cannot start filling against a half-written header. */
  g_motor_trace.mode = mode;
}

void MotorTrace_Stop(void)
{
  g_motor_trace.mode = MOTOR_TRACE_MODE_IDLE;
}

void MotorTrace_Push(int16_t c0, int16_t c1, int16_t c2, int16_t c3)
{
  uint8_t mode = g_motor_trace.mode;
  uint16_t idx;

  if (mode == MOTOR_TRACE_MODE_IDLE)
  {
    return;
  }

  g_motor_trace.pushes++;

  if (g_motor_trace.decim > 1U)
  {
    uint16_t n = (uint16_t)(g_motor_trace.decim_count + 1U);

    if (n < g_motor_trace.decim)
    {
      g_motor_trace.decim_count = n;
      return;
    }
    g_motor_trace.decim_count = 0U;
  }

  idx = g_motor_trace.write_idx;
  g_motor_trace_buf[idx][0] = c0;
  g_motor_trace_buf[idx][1] = c1;
  g_motor_trace_buf[idx][2] = c2;
  g_motor_trace_buf[idx][3] = c3;

  idx++;
  if (idx >= (uint16_t)MOTOR_TRACE_DEPTH)
  {
    g_motor_trace.wrapped = 1U;
    if (mode == MOTOR_TRACE_MODE_ONESHOT)
    {
      /* Leave write_idx at depth so the host can tell "full" from "wrapped
       * once and still running". */
      g_motor_trace.write_idx = (uint16_t)MOTOR_TRACE_DEPTH;
      g_motor_trace.mode = MOTOR_TRACE_MODE_IDLE;
      return;
    }
    idx = 0U;
  }
  g_motor_trace.write_idx = idx;
}
