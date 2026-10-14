#ifndef MOTOR_OPENLOOP_H
#define MOTOR_OPENLOOP_H

#include <stdint.h>

#define MOTOR_OPENLOOP_MODE_3STEP  3U
#define MOTOR_OPENLOOP_MODE_6STEP  6U

typedef struct {
  uint8_t  enabled;
  uint8_t  mode;
  uint8_t  duty_pct;
  uint8_t  step;
  uint8_t  direction;    /* commutation sequence select; hall6 uses 0 = clockwise */
  uint16_t step_ms;
  uint16_t ramp_start_ms;
  uint16_t ramp_end_ms;
  uint16_t ramp_ms;
  uint32_t step_count;
  uint32_t loop_count;
  uint8_t  hall_raw;
  int8_t   hall_spin;
  uint8_t  hall_invert;
  uint8_t  hall_sync_on;
  uint8_t  hall_locked;
} motor_openloop_snapshot_t;

void MotorOpenloop_Init(void);

int  MotorOpenloop_Enable(int enable);
int  MotorOpenloop_IsEnabled(void);

void MotorOpenloop_SetDutyPct(uint8_t pct);
void MotorOpenloop_SetStepMs(uint16_t ms);
void MotorOpenloop_SetRampMs(uint16_t start_ms, uint16_t end_ms, uint16_t ramp_ms);
void MotorOpenloop_SetDirection(int ccw);
void MotorOpenloop_SetMode(uint8_t mode);
void MotorOpenloop_SetHallSync(int enable);
void MotorOpenloop_SetHallPhase(uint8_t phase);
void MotorOpenloop_SetUvPerm(uint8_t perm);
void MotorOpenloop_SetPulseCounts(uint16_t counts);

void MotorOpenloop_GetSnapshot(motor_openloop_snapshot_t *out);

void MotorOpenloop_ControlLoopISR(void);

#endif /* MOTOR_OPENLOOP_H */
