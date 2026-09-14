#ifndef MOTOR_HALL6_H
#define MOTOR_HALL6_H

#include <stdint.h>

typedef struct {
  uint8_t  hall_raw;
  uint8_t  step;
  uint8_t  duty_pct;
  uint8_t  enabled;
  uint8_t  direction;    /* 0 spins clockwise on this wiring, 1 counter-clockwise */
  uint8_t  fault;
  uint8_t  kick;
  uint8_t  phase;
  uint32_t loop_count;
  uint32_t hall_changes;
} motor_hall6_snapshot_t;

void MotorHall6_Init(void);

int  MotorHall6_Enable(int enable);
int  MotorHall6_IsEnabled(void);

/* Stop writing the bridges but leave TIM1 and the control tick running so
 * FOC can take over. Enable(0) also stops the tick and clears MOE. */
void MotorHall6_ReleasePwm(void);

void MotorHall6_SetDutyPct(uint8_t pct);
void MotorHall6_SetKickDutyPct(uint8_t pct);
void MotorHall6_SetPhaseOffset(uint8_t offset);
void MotorHall6_SetDirection(int ccw);

void MotorHall6_GetSnapshot(motor_hall6_snapshot_t *out);

/* Ticks spent in each raw hall code before the last edge that left it.
 * Index 1..6 are valid. Host reads after the rotor is spinning. */
extern volatile uint16_t g_hall6_sector_dwell[8];

/* Electrical angle (Q16) at which FOC +vq matches the voltage hall6
 * would apply for the current raw hall, phase offset and direction.
 * Coupled to the hall6 table, not to motor_angle's sector anchors. */
uint16_t MotorHall6_FocTheta(void);

/* FocTheta extrapolated between stairs, centered on the current stair
 * so mean(interp - stair) ~ 0. Same voltage axis as FocTheta, not
 * motor_angle. Pass this tick's discrete FocTheta (one GPIO read,
 * already done). Do not read the halls again in here. Division is on
 * the edge only. */
uint16_t MotorHall6_FocThetaInterp(uint16_t now);

/* Signed Q16 counts per tick, ×256. One electrical turn of
 * Σ(calibrated sector width)/Σ(ticks + 1 + n/2), not the
 * interpolator's dest-stair predictor. +1 is the starting-edge
 * tick the dwell counter drops; n/2 unbiases the integer floor.
 * Widths from 2026-09-14 foc_ang dwell. 0 if no sector time yet. */
int32_t MotorHall6_FocOmegaQ8(void);

/* Phase that hall6 leaves open: 0=U 1=V 2=W, or 0xFF if none. */
uint8_t MotorHall6_FloatPhase(void);

/* Write the hall6 PWM/LOW/OFF pattern from the FOC tick. Used after
 * ReleasePwm so SVPWM does not drive the floating phase. */
void MotorHall6_ApplyFocDuty(uint8_t duty_pct);

/* Inverse-Park phase voltages in ~mV. Duty is the PWM−LOW line voltage
 * scaled so |vq| = 2.4 V still means ~20%, then the hall6 pattern
 * (third phase stays high-Z). */
void MotorHall6_ApplyFocVoltages(int32_t vu, int32_t vv, int32_t vw,
                                 int32_t vdc_mv);

void MotorHall6_ControlLoopISR(void);

#endif /* MOTOR_HALL6_H */
