#ifndef MOTOR_VSWEEP_H
#define MOTOR_VSWEEP_H

#include <stdint.h>

/*
 * Stationary inverter voltage sweep. The host writes g_vsweep_req = 1.
 * The main loop drives one phase pair at a time (the third leg off), steps
 * the PWM compare from 0 to 80 in steps of 4, and stores each point in the
 * trace buffer. Dead-time compensation stays off, so the CCR is the command
 * under test. One CCR count is Vbus/ARR (about 20 mV at 24 V, ARR = 1199).
 * A polarity stops when any phase current reaches 31 LSB (0.50 A at
 * 16.17 mA/LSB) above the bridge-off zero. The whole sweep is about 2 s.
 *
 * Trace source MOTOR_TRACE_SRC_VSWEEP, four int16 channels:
 *   tag, iu_lsb, iv_lsb, iw_lsb
 * tag bits 0..9   CCR just written
 * tag bits 10..11 pair: 0 = U-V, 1 = V-W, 2 = W-U
 * tag bit 12      0 = first phase of the pair is PWM and the second is low
 *                 1 = swapped
 * tag bit 13      1 = this point reached the 0.50 A stop
 * Currents are shunt LSB minus the bridge-off zero of that phase.
 *
 * g_vsweep_state: 0 idle, 1 running, 2 finished, 3 stopped early.
 * g_vsweep_hall_begin / _end: raw hall, so a later read can see if the
 * rotor moved.
 */

enum
{
  MOTOR_VSWEEP_IDLE = 0,
  MOTOR_VSWEEP_RUN = 1,
  MOTOR_VSWEEP_DONE = 2,
  MOTOR_VSWEEP_ABORT = 3
};

extern volatile uint8_t g_vsweep_req;
extern volatile uint8_t g_vsweep_state;
extern volatile uint16_t g_vsweep_n;
extern volatile uint8_t g_vsweep_hall_begin;
extern volatile uint8_t g_vsweep_hall_end;
extern volatile int16_t g_vsweep_zero[3];

void MotorVsweep_Poll(void);

#endif /* MOTOR_VSWEEP_H */
