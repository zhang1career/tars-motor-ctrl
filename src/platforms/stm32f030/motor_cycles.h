#ifndef MOTOR_CYCLES_H
#define MOTOR_CYCLES_H

#include <stdint.h>

/* TIM3 prescaler 0 -> 48 MHz on STM32F030 @ 48 MHz SYSCLK. */
#define MOTOR_CYCLES_HZ 48000000U

void MotorCycles_Init(void);
uint16_t MotorCycles_Read(void);
uint16_t MotorCycles_Delta(uint16_t start, uint16_t end);

/* Median of eight back-to-back read pairs (measurement overhead). */
uint16_t MotorCycles_Overhead(void);

/* Measure fn(); subtract overhead; IRQs disabled for the timed region. */
uint16_t MotorCycles_Measure(void (*fn)(void));

/*
 * Same as MotorCycles_Measure(), but drives PA15 high for the timed region
 * so a scope can cross-check pulse width against cycle count / 48 MHz.
 */
uint16_t MotorCycles_MeasureScope(void (*fn)(void));

#endif /* MOTOR_CYCLES_H */
