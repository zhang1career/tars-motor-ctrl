#include "motor_tick.h"
#include "hall6.h"
#include "openloop.h"
#include "motor_pwm.h"
#include "main.h"
#include "stm32f0xx_hal.h"
#if defined(MOTOR_TRACE) && (MOTOR_TRACE != 0)
#include "motor_trace.h"
#endif
#if defined(MOTOR_ANGLE) && (MOTOR_ANGLE != 0)
#include "motor_angle.h"
#include "motor_hall.h"
#endif
#if defined(MOTOR_FOC) && (MOTOR_FOC != 0)
#include "motor_foc.h"
#endif
#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
#include "motor_adc.h"

/* Counter direction at ISR entry, which says whether the update event lands on
 * the peak (DIR=1, counting down) or the valley (DIR=0). RM0091 leaves this
 * ambiguous for centre-aligned mode with an odd RCR, and it sets how old the
 * ADC samples are when the ISR reads them: a few hundred ns if the ISR is at
 * the peak, half a PWM period if it is at the valley. The ADC trigger itself is
 * anchored to the counter through OC4REF, so it is correct either way.
 */
volatile uint8_t g_motor_tick_dir;
#endif

static void motor_tick_dispatch(void)
{
#if defined(MOTOR_ANGLE) && (MOTOR_ANGLE != 0)
  /* Before the controller, so stage E can consume the angle in the same tick.
   * Reads the halls itself rather than borrowing hall6's copy, so FOC does not
   * depend on hall6 being enabled. */
  MotorAngle_Update(MotorHall_ReadRaw());
#endif

  if (MotorHall6_IsEnabled() != 0)
  {
    MotorHall6_ControlLoopISR();
  }
  else if (MotorOpenloop_IsEnabled() != 0)
  {
    MotorOpenloop_ControlLoopISR();
  }

  /* The float FOC deliberately does NOT run here -- 16173 cycles against a
   * 2400-cycle tick would overrun and drop the following ticks. It runs in the
   * background loop; the fixed-point implementation is what belongs in the ISR.
   */

#if defined(MOTOR_TRACE) && (MOTOR_TRACE != 0)
#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
  if (g_motor_trace.source == MOTOR_TRACE_SRC_ADC)
  {
    g_motor_tick_dir =
        ((htim1.Instance->CR1 & TIM_CR1_DIR) != 0U) ? 1U : 0U;
    MotorTrace_Push(MotorAdc_ShuntLsb(MOTOR_ADC_IU),
                    MotorAdc_ShuntLsb(MOTOR_ADC_IV),
                    MotorAdc_ShuntLsb(MOTOR_ADC_IW),
                    (int16_t)g_motor_adc_raw[MOTOR_ADC_VBUS]);
  }
#endif
#if defined(MOTOR_ANGLE) && (MOTOR_ANGLE != 0)
  if (g_motor_trace.source == MOTOR_TRACE_SRC_ANGLE)
  {
    MotorTrace_Push((int16_t)g_motor_angle.hall,
                    (int16_t)(g_motor_angle.theta >> 1),
                    (int16_t)g_motor_angle.ticks_in_sector,
                    (int16_t)g_motor_angle.edge_jump);
  }
#endif
#if defined(MOTOR_FOC) && (MOTOR_FOC != 0)
  if (g_motor_trace.source == MOTOR_TRACE_SRC_FOC)
  {
    MotorTrace_Push(g_motor_foc.id_ma, g_motor_foc.iq_ma,
                    (int16_t)(g_motor_foc.theta >> 1), g_motor_foc.vq_mv);
  }
#endif
#endif
}

void MotorTick_Init(void)
{
  /* No timer of its own: the control loop runs from TIM1 update (TARS-aligned). */
}

void MotorTick_Start(void)
{
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

void MotorTick_Stop(void)
{
  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
}

void MotorTick_OnTim1Update(void)
{
  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE) != RESET)
    {
      __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
      motor_tick_dispatch();
    }
  }
}
