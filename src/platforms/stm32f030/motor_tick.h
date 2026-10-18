#ifndef MOTOR_TICK_H
#define MOTOR_TICK_H

#include <stdint.h>
#include "stm32f0xx_hal.h"

#define MOTOR_CTRL_ISR_HZ  20000U

extern volatile uint16_t g_tick_pad;
/* Max cycles inside one current-loop step. Host clears these. They sit in
 * .noinit so a watchdog reset keeps them. 0 if that slice has not run.
 * 0 Clarke and the Park that feeds the PIs
 * 1 the two observe-only Parks
 * 2 PIs, voltage limit, inverse Park, SVPWM, CCR
 * 3 duty write */
extern volatile uint16_t g_tick_part_max[4];

uint32_t MotorTick_Mark(void);
void MotorTick_PartNote(uint32_t a, uint32_t b, uint8_t i);

void MotorTick_Init(void);
void MotorTick_Start(void);
void MotorTick_Stop(void);
void MotorTick_OnTim1Update(void);
void MotorTick_OnAdcComplete(void);

#endif /* MOTOR_TICK_H */
