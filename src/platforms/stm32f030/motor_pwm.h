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

/*
 * All three phases in complementary PWM, for FOC. Duties are 0..1.
 *
 * The phase currents are needed for dead-time compensation, not for control:
 * during the dead time both switches are off and the phase is pulled to
 * whichever rail its own current freewheels into, so the error is +DT/2 or
 * -DT/2 depending on current sign. That is 36 counts of 1199, about 3% of the
 * bus, i.e. 0.36 V -- and it does NOT cancel as common mode the way a uniform
 * offset would, because each phase's error follows its own current polarity.
 */
void MotorPwm_SetDuties(float da, float db, float dc, float ia, float ib, float ic);

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
