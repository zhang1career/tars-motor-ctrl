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
 * -DT/2 depending on current sign. That is 36 CCR counts (72 timer counts),
 * 0.72 V at 24 V, on a leg that is actually switching. A leg held at CCR 0
 * is on a rail and has no dead-time error. Each phase follows its own
 * current, so the error is not a common-mode offset.
 */
void MotorPwm_SetDuties(float da, float db, float dc, float ia, float ib, float ic);

/*
 * Same dead-time compensation as SetDuties, but the inputs are already CCR
 * counts and phase currents in ADC LSB.
 *
 * The phase currents are needed for dead-time compensation, not for control:
 * during the dead time both switches are off and the phase is pulled to
 * whichever rail its own current freewheels into, so the error is +DT/2 or
 * -DT/2 depending on current sign. That is 36 CCR counts (72 timer counts),
 * 0.72 V at 24 V, on a leg that is actually switching. A leg held at CCR 0
 * is on a rail and has no dead-time error. Each phase follows its own
 * current, so the error is not a common-mode offset.
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
 * above 2.0 A before the winding settles at ~1.6 A. 2.4 A = 148 LSB at
 * 16.17 mA/LSB (1 mOhm). With that shunt the window comparator trips near
 * 25 A, so this limit is the only phase-current protection.
 * The shunt zero is not 2048. The main loop averages 64 calm samples
 * while MOE is off, because the tick starts only after the bridge is on.
 * The compare subtracts that zero, so 148 counts on both sides. 148 is
 * not changed.
 * MOTOR_PWM_OCP_TICKS consecutive hits reject a one-sample spike. */
#ifndef MOTOR_PWM_BKIN
#define MOTOR_PWM_BKIN 1
#endif
#ifndef MOTOR_PWM_OCP_LSB
#define MOTOR_PWM_OCP_LSB 148
#endif
#ifndef MOTOR_PWM_OCP_TICKS
#define MOTOR_PWM_OCP_TICKS 4
#endif

typedef struct
{
  uint32_t sw_trips;
  uint32_t bkin_trips;
  int16_t  i_lsb[3];
  uint8_t  latched;    /* outputs held off until an explicit clear */
  uint8_t  bkin_seen;  /* 1 while BIF is still set; counts one trip */
  uint8_t  ocp_hits;
  uint8_t  ocp_hold;   /* 1 after SWOC or BKIN; MoeEnable refuses */
  uint8_t  ocp_kind;   /* 1 software or forced, 2 BKIN */
  int16_t  ocp_zero[3]; /* raw-2048, bridge off; valid when ready is 1 */
  uint8_t  ocp_zero_ready;
} motor_pwm_prot_t;

extern volatile motor_pwm_prot_t g_motor_pwm_prot;
/* Host writes 1 via SWD to trip without injecting real current. */
extern volatile uint8_t g_motor_pwm_force_trip;

void MotorPwm_Init(void);
/* Count TIM1 without arming CH1–3 or MOE so ADC TRGO (OC4REF) runs at idle. */
void MotorPwm_RunCounter(void);
void MotorPwm_HardwareSafe(void);
void MotorPwm_PinsIdle(void);
void MotorPwm_RestoreAfPins(void);
int  MotorPwm_ArmOutputs(void);
void MotorPwm_MoeEnable(void);
/* Drops ocp_hold. Does not turn the bridge on. A new start calls this
 * before MoeEnable. A status read must not. */
void MotorPwm_ClearOcpHold(void);
/* High sides off, all three low sides on, long enough for each UCC27211
 * to charge its bootstrap from VDD. Call after ArmOutputs, before any
 * high-side pulse. A later HardwareSafe or Stop requires it again. */
void MotorPwm_ChargeBootstrap(void);
/* 0 while the low-side precharge window is still running. */
int  MotorPwm_BootstrapReady(void);
int  MotorPwm_Start(void);
void MotorPwm_Stop(void);
void MotorPwm_PollProtect(void);
/* Bridge-off zero. Call from the main loop; the tick is not running then. */
void MotorPwm_PollOcpZero(void);
uint8_t MotorPwm_IsLatched(void);
void MotorPwm_SetPhase(motor_phase_mode_t u, motor_phase_mode_t v, motor_phase_mode_t w,
                       uint8_t duty_pct);
/* One leg PWM at this CCR (0..ARR), one leg held low, the third off.
 * No dead-time compensation: the CCR is the voltage command under test.
 * Indices 0, 1, 2 are U, V, W. */
void MotorPwm_SetPairCcr(uint8_t pwm_idx, uint8_t low_idx, uint16_t ccr);

#endif /* MOTOR_PWM_H */
