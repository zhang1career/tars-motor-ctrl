#include "motor_app.h"
#if defined(MOTOR_FOC_BENCH) && (MOTOR_FOC_BENCH != 0)
#include "foc_bench.h"
#endif
#include "hall6.h"
#include "openloop.h"
#include "motor_pwm.h"
#include "motor_tick.h"
#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
#include "motor_adc.h"
#endif
#if defined(MOTOR_ANGLE) && (MOTOR_ANGLE != 0)
#include "motor_angle.h"
#endif
#if defined(MOTOR_TRACE) && (MOTOR_TRACE != 0)
#include "motor_trace.h"
#ifndef MOTOR_TRACE_DECIM
#define MOTOR_TRACE_DECIM 1U
#endif
#endif

/*
 * Bench-validated on the UCC27211 + AOD4184 half bridges with the VOUT filter
 * capacitor removed: 20% duty gives ~1.6 A winding current, and once the rotor
 * is turning back-EMF holds the bus at ~70 mA / ~24 electrical rev/s.
 * TARS ran 6%/7% here, but that leaves almost no HS conduction after the 1.5 us
 * dead time is subtracted.
 */
#ifndef MOTOR_HALL6_KICK_DUTY
#define MOTOR_HALL6_KICK_DUTY 20U
#endif
#ifndef MOTOR_HALL6_RUN_DUTY
#define MOTOR_HALL6_RUN_DUTY 20U
#endif
#ifndef MOTOR_HALL6_PHASE
#define MOTOR_HALL6_PHASE 3U
#endif
#ifndef MOTOR_HALL6_CCW
#define MOTOR_HALL6_CCW 0
#endif

void MotorApp_ApplyDefaults(void)
{
  MotorHall6_SetPhaseOffset(MOTOR_HALL6_PHASE);
  MotorHall6_SetKickDutyPct(MOTOR_HALL6_KICK_DUTY);
  MotorHall6_SetDutyPct(MOTOR_HALL6_RUN_DUTY);
  MotorHall6_SetDirection(MOTOR_HALL6_CCW);
}

void MotorApp_Init(void)
{
  MotorHall6_Init();
  MotorOpenloop_Init();
#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
  /* Conversions only happen once TIM1 runs, since TRGO drives them, so starting
   * the DMA here is harmless and means samples exist whenever the timer does. */
  MotorAdc_Init();
  (void)MotorAdc_Start();
#endif
#if defined(MOTOR_ANGLE) && (MOTOR_ANGLE != 0)
  MotorAngle_Reset();
#endif
  MotorApp_ApplyDefaults();
}

int MotorApp_Start(void)
{
#if defined(MOTOR_TRACE) && (MOTOR_TRACE != 0)
  /* Wrap mode: the buffer always holds the most recent 256 ticks, so a capture
   * needs no trigger and the host can read whenever it likes. Stage E will
   * re-arm it one-shot from the host to catch step responses. */
  MotorTrace_Arm(MOTOR_TRACE_MODE_WRAP, MOTOR_TRACE_SRC_HALL6, MOTOR_TRACE_DECIM);
#endif
  return MotorHall6_Enable(1);
}

void MotorApp_Stop(void)
{
  (void)MotorHall6_Enable(0);
  (void)MotorOpenloop_Enable(0);
  MotorPwm_Stop();
}

int MotorApp_StartOpenloop6Cfg(uint8_t phase, uint8_t uv_perm, uint8_t dir_ccw,
                               uint8_t duty_pct, uint16_t step_ms)
{
  (void)MotorHall6_Enable(0);
  MotorOpenloop_SetMode(MOTOR_OPENLOOP_MODE_6STEP);
  MotorOpenloop_SetHallSync(0);
  MotorOpenloop_SetHallPhase(phase);
  MotorOpenloop_SetUvPerm(uv_perm);
  MotorOpenloop_SetDirection(dir_ccw);
  MotorOpenloop_SetDutyPct(duty_pct);
  MotorOpenloop_SetStepMs(step_ms);
  return MotorOpenloop_Enable(1);
}

#ifndef MOTOR_DUTY_PCT
#define MOTOR_DUTY_PCT 7U
#endif
#ifndef MOTOR_PULSE_COUNTS
#define MOTOR_PULSE_COUNTS 0U
#endif
#ifndef MOTOR_STEP_MS
#define MOTOR_STEP_MS 20U
#endif
#ifndef MOTOR_PHASE_OFFSET
#define MOTOR_PHASE_OFFSET 3U
#endif

int MotorApp_StartOpenloop6(void)
{
  MotorOpenloop_SetPulseCounts(MOTOR_PULSE_COUNTS);
  return MotorApp_StartOpenloop6Cfg(MOTOR_PHASE_OFFSET, 0U, 0U, MOTOR_DUTY_PCT,
                                    MOTOR_STEP_MS);
}

/*
 * One fixed commutation step, no Hall dependence and no commutation: the safest
 * way to put current through the bridges. Reachable from cmake
 * (-DMOTOR_AUTO_START=ON -DMOTOR_START_DIAG=ON) so it runs from a reset rather
 * than from a gdb call, which docs/roadmap.md section 9.5 rules out for
 * measurements.
 */
int MotorApp_StartDiagStep(void)
{
  return MotorApp_DiagGPhase(MOTOR_DUTY_PCT);
}

__attribute__((used)) int MotorApp_DiagGPhase(uint8_t duty_pct)
{
  if (duty_pct > 7U)
  {
    duty_pct = 7U;
  }

  (void)MotorHall6_Enable(0);
  (void)MotorOpenloop_Enable(0);

  if (MotorPwm_Start() == 0)
  {
    return 0;
  }

  MotorPwm_SetPhase(MOTOR_PHASE_OFF, MOTOR_PHASE_PWM, MOTOR_PHASE_LOW, duty_pct);
  MotorPwm_MoeEnable();

#if defined(MOTOR_TRACE) && (MOTOR_TRACE != 0)
  MotorTrace_Arm(MOTOR_TRACE_MODE_WRAP, MOTOR_TRACE_SRC_ADC, MOTOR_TRACE_DECIM);
#endif
  /* Neither controller is enabled here, so nothing else would start the control
   * ISR -- and without it there is no per-tick sampling of the very operating
   * point this step exists to calibrate. */
  MotorTick_Start();
  return 1;
}

__attribute__((used)) void MotorApp_DiagStop(void)
{
  MotorTick_Stop();
  MotorApp_Stop();
}

#if defined(MOTOR_FOC_BENCH) && (MOTOR_FOC_BENCH != 0)
__attribute__((used)) void MotorApp_RunFocBench(void)
{
  MotorApp_Stop();
  MotorFocBench_Run();
}
#endif

__attribute__((used)) void *const motor_swd_entry[] = {
  (void *)MotorApp_Start,
  (void *)MotorApp_Stop,
  (void *)MotorApp_StartOpenloop6,
  (void *)MotorApp_StartOpenloop6Cfg,
  (void *)MotorApp_DiagGPhase,
  (void *)MotorApp_DiagStop,
#if defined(MOTOR_FOC_BENCH) && (MOTOR_FOC_BENCH != 0)
  (void *)MotorApp_RunFocBench,
#endif
};
