#ifndef MOTOR_HALL6_H
#define MOTOR_HALL6_H

#include <stdint.h>

typedef struct {
  uint8_t  hall_raw;
  uint8_t  step;
  uint8_t  duty_pct;
  uint8_t  enabled;
  uint8_t  direction;    /* commutation sequence select; 0 spins CCW on this wiring */
  uint8_t  fault;
  uint8_t  kick;
  uint8_t  phase;
  uint32_t loop_count;
  uint32_t hall_changes;
} motor_hall6_snapshot_t;

void MotorHall6_Init(void);

int  MotorHall6_Enable(int enable);
int  MotorHall6_IsEnabled(void);

void MotorHall6_SetDutyPct(uint8_t pct);
void MotorHall6_SetKickDutyPct(uint8_t pct);
void MotorHall6_SetPhaseOffset(uint8_t offset);
void MotorHall6_SetDirection(int ccw);

void MotorHall6_GetSnapshot(motor_hall6_snapshot_t *out);

void MotorHall6_ControlLoopISR(void);

#endif /* MOTOR_HALL6_H */
