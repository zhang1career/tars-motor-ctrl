#ifndef BOOST_H
#define BOOST_H

#include <stdint.h>

/*
 * 24 V -> 32 V boost on PA11 / TIM1_CH4. Q1 is BSS138, L1 is 10 uH,
 * switching at the motor PWM rate (20 kHz, center-aligned ARR 1199).
 *
 * On-time is 2*CCR/48 MHz. One CCR count is 0.10 A of ideal peak current.
 * CCR 8 is 0.80 A, the saturation current already chosen for L1.
 *
 * The motor image steps this from the main loop, once per millisecond, off
 * the PA4 sample already in the PWM ADC scan. The current-loop tick does
 * not call it. Boost_OnPwmTick is the same 1 kHz step for a caller that
 * is already inside the PWM period.
 */

/* (18k + 1k) / 1k. PA4 code = Vout / 19 / 3.3 * 4095. */
#define BOOST_DIV_NUM     19U
#define BOOST_SET_MV      32000U
#define BOOST_SET_CODE    2090U   /* 32.00 V */
#define BOOST_OV_CODE     2351U   /* 36.0 V, under the 40 V diode */
#define BOOST_BAND        2U
#define BOOST_DUTY_MAX    8U

#define BOOST_ST_CLOCK    1U
#define BOOST_ST_RANGE    2U
#define BOOST_ST_SENSE    3U
#define BOOST_ST_RUN      4U

void Boost_Init(void);
void Boost_Poll(void);
/* 1 after PA4 has reached the 32 V window. Stays 1 through a small dip. */
uint8_t Boost_Ready(void);
/* HardwareSafe drops every TIM1 channel. Put CH4 and MOE back. */
void Boost_KeepAlive(void);
/* Start TIM1 and TIM3 together. No-op while both are already running. */
void Boost_Resync(void);
void Boost_Step(uint16_t adc);
void Boost_OnPwmTick(uint16_t adc);
void Boost_Bringup(void);

#endif
