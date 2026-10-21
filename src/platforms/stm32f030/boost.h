#ifndef BOOST_H
#define BOOST_H

#include <stdint.h>

/*
 * 24 V -> 32 V boost on PA11 / TIM1_CH4. Q1 is BSS138, L1 is 10 uH,
 * switching at the motor PWM rate (20 kHz, center-aligned ARR 1199).
 *
 * On-time is 2*CCR/48 MHz. Logical duty 0..BOOST_DUTY_MAX maps linearly to
 * TIM1 CCR4 0..BOOST_CCR_HW_MAX (peak drive unchanged; finer steps).
 *
 * Closed loop on PA5 VSENSE_V_BOOST (R32/R33 18k/1k), sampled in the same
 * PWM-synchronised DMA scan as the phase currents. Boost_Poll low-pass filters
 * PA5 (~16 ms), steps CCR4 only outside a wide deadband and at most once per
 * BOOST_STEP_MIN_MS. The current-loop tick does not call it.
 */

/* (18k + 1k) / 1k. PA5 code = V_BOOST / 19 / 3.3 V * 4095. */
#define BOOST_DIV_NUM     19U
#define BOOST_SET_MV      32000U
#define BOOST_SET_CODE    2090U   /* 32.00 V on V_BOOST */
#define BOOST_OV_CODE     2351U   /* 36.0 V, under the 40 V diode */
#define BOOST_LO_CODE     2040U   /* ~31.2 V: may step duty up */
#define BOOST_HI_CODE     2140U   /* ~32.8 V: may step duty down */
#define BOOST_READY_LO    2000U   /* ~30.6 V: Boost_Ready window low */
/* >2×20 ms @ 50 Hz mains; prime vs 20/10 ms cadences. Not for amplitude. */
#define BOOST_STEP_MIN_MS 43U
#define BOOST_BAND        2U      /* Boost_Step / bringup only */
#define BOOST_CCR_HW_MAX  8U      /* TIM1->CCR4 at full logical duty */
#define BOOST_DUTY_MAX    16U     /* logical notches mapped to CCR 0..8 */

#define BOOST_ST_CLOCK    1U
#define BOOST_ST_RANGE    2U
#define BOOST_ST_SENSE    3U
#define BOOST_ST_RUN      4U

void Boost_Init(void);
void Boost_Poll(void);
/* 1 while filtered PA5 is in BOOST_READY_LO..BOOST_OV_CODE; 0 otherwise. */
uint8_t Boost_Ready(void);
/* HardwareSafe drops every TIM1 channel. Put CH4 and MOE back. */
void Boost_KeepAlive(void);
/* Start TIM1 and TIM3 together. No-op while both are already running. */
void Boost_Resync(void);
void Boost_Step(uint16_t adc);
void Boost_OnPwmTick(uint16_t adc);
void Boost_Bringup(void);

#endif
