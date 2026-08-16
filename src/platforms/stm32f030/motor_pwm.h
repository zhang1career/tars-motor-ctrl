#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <stdint.h>
#include "stm32f0xx_hal.h"

#define MOTOR_PWM_HZ       20000U
#define MOTOR_PWM_TIM_CLK  48000000U

typedef enum {
  MOTOR_PHASE_OFF = 0,
  MOTOR_PHASE_LOW = 1,
  MOTOR_PHASE_PWM = 2
} motor_phase_mode_t;

extern TIM_HandleTypeDef htim1;

void MotorPwm_Init(void);
void MotorPwm_HardwareSafe(void);
void MotorPwm_PinsIdle(void);
void MotorPwm_RestoreAfPins(void);
int  MotorPwm_ArmOutputs(void);
void MotorPwm_MoeEnable(void);
int  MotorPwm_Start(void);
void MotorPwm_Stop(void);
void MotorPwm_SetPhase(motor_phase_mode_t u, motor_phase_mode_t v, motor_phase_mode_t w,
                       uint8_t duty_pct);

#endif /* MOTOR_PWM_H */
