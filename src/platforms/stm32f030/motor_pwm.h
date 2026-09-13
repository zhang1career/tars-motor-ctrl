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

/*
 * Same dead-time compensation as SetDuties, but the inputs are already CCR
 * counts and phase currents in ADC LSB.
 *
 * The phase currents are needed for dead-time compensation, not for control:
 * during the dead time both switches are off and the phase is pulled to
 * whichever rail its own current freewheels into, so the error is +DT/2 or
 * -DT/2 depending on current sign. That is 36 counts of 1199, about 3% of the
 * bus, i.e. 0.36 V -- and it does NOT cancel as common mode the way a uniform
 * offset would, because each phase's error follows its own current polarity.
 */
/* All three legs complementary. Dead-time compensation is off unless
 * g_motor_pwm_dt_on is 1 (host, after midrail+id is already spinning). */
void MotorPwm_SetDutiesCcr(int32_t ccr_u, int32_t ccr_v, int32_t ccr_w,
                           int32_t ia_lsb, int32_t ib_lsb, int32_t ic_lsb);

/* Host writes 1 to add ±DT/2 to each CCR from current sign. 0 keeps
 * the three-phase writer raw. Hi-Z already compensates. */
extern volatile uint8_t g_motor_pwm_dt_on;

/* Same CCR math, two legs complementary. hz 0/1/2 floats that phase;
 * 0xFF floats the CCR nearest ARR/2 (unsafe if Park is moving). */
void MotorPwm_SetDutiesCcrHiZ(int32_t ccr_u, int32_t ccr_v, int32_t ccr_w,
                              int32_t ia_lsb, int32_t ib_lsb, int32_t ic_lsb,
                              uint8_t hz);

/* Software overcurrent sits under the ±2.5 A window comparator. 2.0 A
 * (MOTOR_FOC_IMAX) is the current-loop command cap, not a sample limit:
 * hall6 kick at 20% rises at V/L ≈ 27 A/ms and a single tick can read
 * above 2.0 A before the winding settles at ~1.6 A. 2.4 A = 1484 LSB.
 * MOTOR_PWM_OCP_TICKS consecutive hits reject a one-sample spike. */
#ifndef MOTOR_PWM_BKIN
#define MOTOR_PWM_BKIN 1
#endif
#ifndef MOTOR_PWM_OCP_LSB
#define MOTOR_PWM_OCP_LSB 1484
#endif
#ifndef MOTOR_PWM_OCP_TICKS
#define MOTOR_PWM_OCP_TICKS 4
#endif

typedef struct
{
  uint32_t sw_trips;
  uint32_t bkin_trips;
  int16_t  i_lsb[3];
  uint8_t  latched;    /* outputs held off until MotorPwm_MoeEnable */
  uint8_t  bkin_seen;  /* 1 while BIF is still set; counts one trip */
  uint8_t  ocp_hits;
} motor_pwm_prot_t;

extern volatile motor_pwm_prot_t g_motor_pwm_prot;
/* Host writes 1 via SWD to trip without injecting real current. */
extern volatile uint8_t g_motor_pwm_force_trip;

void MotorPwm_Init(void);
void MotorPwm_HardwareSafe(void);
void MotorPwm_PinsIdle(void);
void MotorPwm_RestoreAfPins(void);
int  MotorPwm_ArmOutputs(void);
void MotorPwm_MoeEnable(void);
int  MotorPwm_Start(void);
void MotorPwm_Stop(void);
void MotorPwm_PollProtect(void);
uint8_t MotorPwm_IsLatched(void);
void MotorPwm_SetPhase(motor_phase_mode_t u, motor_phase_mode_t v, motor_phase_mode_t w,
                       uint8_t duty_pct);

#endif /* MOTOR_PWM_H */
