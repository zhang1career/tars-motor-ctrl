#ifndef MOTOR_TICK_H
#define MOTOR_TICK_H

#include <stdint.h>
#include "stm32f0xx_hal.h"

extern TIM_HandleTypeDef htim3;

#define MOTOR_CTRL_ISR_HZ  20000U

void MotorTick_Init(void);
void MotorTick_Start(void);
void MotorTick_Stop(void);
void MotorTick_OnTim1Update(void);

#endif /* MOTOR_TICK_H */
