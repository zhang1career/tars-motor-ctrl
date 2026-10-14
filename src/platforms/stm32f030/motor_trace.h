#ifndef MOTOR_TRACE_H
#define MOTOR_TRACE_H

#include <stdint.h>

/*
 * On-chip sample buffer for the 20 kHz control loop.
 *
 * The only other way to observe this system is to read a snapshot per openocd
 * invocation, roughly one second apart, which cannot see per-tick behaviour --
 * and that is exactly the timescale a current loop lives on. This buffer trades
 * 2 KB of the 4 KB RAM for the ability to capture 256 consecutive ticks.
 *
 * The host reads it over SWD while the CPU keeps running (Cortex-M memory
 * access goes through the DAP), so a capture never perturbs commutation.
 */

#define MOTOR_TRACE_CH 4U

#ifndef MOTOR_TRACE_DEPTH
#define MOTOR_TRACE_DEPTH 256U
#endif

#define MOTOR_TRACE_MAGIC 0x54524331U /* "TRC1" */

enum
{
  MOTOR_TRACE_MODE_IDLE = 0U,
  MOTOR_TRACE_MODE_ONESHOT = 1U, /* fill once, then stop: catches a transient */
  MOTOR_TRACE_MODE_WRAP = 2U     /* keep overwriting: catches steady state */
};

/* Which producer filled the buffer, so a dump says what its channels mean
 * instead of relying on whoever ran the capture to remember. */
enum
{
  MOTOR_TRACE_SRC_NONE = 0U,
  MOTOR_TRACE_SRC_HALL6 = 1U, /* hall_raw, step, ticks_since_edge, kick */
  MOTOR_TRACE_SRC_FOC = 2U,
  MOTOR_TRACE_SRC_ADC = 3U,   /* iu_lsb, iv_lsb, iw_lsb, vbus_raw */
  MOTOR_TRACE_SRC_ANGLE = 4U, /* hall_raw, theta_q15, ticks_in_sector, edge_jump */
  MOTOR_TRACE_SRC_FOC_ANG = 5U, /* foc_q15, interp_q15, dth_q15, hall_raw */
  MOTOR_TRACE_SRC_FOC_V = 6U    /* vd_mv, vq_mv, sat, iq_lsb */
};

typedef struct
{
  uint32_t magic;
  uint16_t depth;
  uint16_t channels;
  uint16_t write_idx;   /* next slot to be written */
  uint16_t decim;       /* keep one sample every decim pushes; 0 and 1 both mean all */
  uint16_t decim_count;
  uint32_t pushes;      /* every Push() call, kept vs write_idx to check decimation */
  uint8_t  mode;
  uint8_t  wrapped;     /* set once write_idx has passed the end at least once */
  uint8_t  source;
  uint8_t  reserved;
} motor_trace_header_t;

extern volatile motor_trace_header_t g_motor_trace;
extern volatile int16_t g_motor_trace_buf[MOTOR_TRACE_DEPTH][MOTOR_TRACE_CH];

void MotorTrace_Arm(uint8_t mode, uint8_t source, uint16_t decim);
void MotorTrace_Stop(void);

/* Call from the control ISR. Array stores and an index bump only -- no
 * division, no floating point (docs/roadmap.md stage A1). */
void MotorTrace_Push(int16_t c0, int16_t c1, int16_t c2, int16_t c3);

#endif /* MOTOR_TRACE_H */
