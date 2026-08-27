#ifndef MOTOR_APP_H
#define MOTOR_APP_H

#include <stdint.h>

void MotorApp_Init(void);
void MotorApp_ApplyDefaults(void);
int  MotorApp_Start(void);
void MotorApp_Stop(void);

int  MotorApp_StartOpenloop6(void);
int  MotorApp_StartOpenloop6Cfg(uint8_t phase, uint8_t uv_perm, uint8_t dir_ccw,
                                uint8_t duty_pct, uint16_t step_ms);

/* SWD bench: force G-phase PWM + W low return at duty_pct (max 7). */
int  MotorApp_DiagGPhase(uint8_t duty_pct);
void MotorApp_DiagStop(void);

/* Stage B: run FOC codegen timing bench (motor off, no PWM). */
void MotorApp_RunFocBench(void);

extern void *const motor_swd_entry[];

#endif /* MOTOR_APP_H */
