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
#include "motor_foc_fx.h"
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

volatile uint16_t g_tick_pad;
volatile uint16_t g_tick_part_max[4]
    __attribute__((section(".noinit"), aligned(4)));

uint32_t MotorTick_Mark(void)
{
  return SysTick->VAL;
}

void MotorTick_PartNote(uint32_t a, uint32_t b, uint8_t i)
{
  uint32_t load = SysTick->LOAD;
  uint32_t cyc = (a >= b) ? (a - b) : (a + load + 1U - b);

  if ((i < 4U) && (cyc < 20000U) && (cyc > g_tick_part_max[i]))
  {
    g_tick_part_max[i] = (uint16_t)cyc;
  }
}

/* Trace divides and keeps a pile of locals. noinline so a current
 * tick with the trace idle does not pay that prologue. */
#if defined(MOTOR_TRACE) && (MOTOR_TRACE != 0)
static void motor_tick_trace(void) __attribute__((noinline));
static void motor_tick_trace(void)
{
#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
  if (g_motor_trace.source == MOTOR_TRACE_SRC_ADC)
  {
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
    int16_t hall = 0;

#if defined(MOTOR_ANGLE) && (MOTOR_ANGLE != 0)
    hall = (int16_t)g_motor_angle.hall;
#endif
    MotorTrace_Push((int16_t)g_motor_foc_fx.id_lsb,
                    (int16_t)g_motor_foc_fx.iq_lsb,
                    (int16_t)(g_motor_foc_fx.theta >> 1),
                    hall);
  }
  if (g_motor_trace.source == MOTOR_TRACE_SRC_FOC_ANG)
  {
    int16_t hall = 0;

#if defined(MOTOR_ANGLE) && (MOTOR_ANGLE != 0)
    hall = (int16_t)g_motor_angle.hall;
#endif
    MotorTrace_Push((int16_t)(g_motor_foc_fx.theta_interp >> 1),
                    (int16_t)(g_motor_foc_fx.theta >> 1),
                    (int16_t)(g_motor_foc_fx.dth >> 1),
                    hall);
  }
  if (g_motor_trace.source == MOTOR_TRACE_SRC_FOC_V)
  {
    int32_t vd_mv = g_motor_foc_fx.vd_uv / 1000;
    int32_t vq_mv = g_motor_foc_fx.vq_uv / 1000;

    if (vd_mv > 32767)
    {
      vd_mv = 32767;
    }
    else if (vd_mv < -32768)
    {
      vd_mv = -32768;
    }
    if (vq_mv > 32767)
    {
      vq_mv = 32767;
    }
    else if (vq_mv < -32768)
    {
      vq_mv = -32768;
    }
    MotorTrace_Push((int16_t)vd_mv, (int16_t)vq_mv,
                    (int16_t)g_motor_foc_fx.sat,
                    (int16_t)g_motor_foc_fx.iq_lsb);
  }
#endif
}
#endif

/* Six-step and open-loop. Not on the current-loop tick: that tick was
 * pushing eight registers because this lived in the same function. */
static void motor_tick_other(void) __attribute__((noinline));
static void motor_tick_other(void)
{
#if defined(MOTOR_ANGLE) && (MOTOR_ANGLE != 0)
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
#if defined(MOTOR_FOC) && (MOTOR_FOC != 0)
  MotorFocFx_Step();
#endif
}

static void motor_tick_dispatch(void)
{
  /* Before any PWM write: hardware BKIN already dropped MOE, and the
   * software limit must do the same on this tick's samples. */
  MotorPwm_PollProtect();
#if defined(MOTOR_FOC) && (MOTOR_FOC != 0)
  MotorFocFx_Handover();
  if (g_motor_foc_fx.mode == MOTOR_FOC_FX_CURRENT)
  {
    /* Park uses FocTheta. Hall6 was released at handover, so the
     * six-step ISR and the open-loop check are not on this tick. */
    MotorFocFx_Step();
  }
  else
#endif
  {
    motor_tick_other();
  }

  /* Whole-tick cycles are recorded in MotorAdc_DmaIrq. */
#if defined(MOTOR_TRACE) && (MOTOR_TRACE != 0)
  if (g_motor_trace.source != 0U)
  {
    motor_tick_trace();
  }
#endif

  if (g_tick_pad != 0U)
  {
    volatile uint16_t n = g_tick_pad;

    while (n != 0U)
    {
      n--;
      __asm__ volatile("" ::: "memory");
    }
  }
}

void MotorTick_Init(void)
{
  /* No timer of its own. With MOTOR_ADC the tick is DMA TC (samples just
   * finished, near the PWM peak). Without it, TIM1 update (valley). */
#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
  /* Nothing reads this. A live reference keeps the byte, so --gc-sections
   * does not slide the following BSS. Not on the 20 kHz tick. */
  (void)g_motor_tick_dir;
#endif
}

void MotorTick_Start(void)
{
#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
  MotorAdc_EnableTick();
#else
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
#endif
}

void MotorTick_Stop(void)
{
#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
  MotorAdc_DisableTick();
#else
  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
#endif
}

void MotorTick_OnAdcComplete(void)
{
#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
  motor_tick_dispatch();
#endif
}

void MotorTick_OnTim1Update(void)
{
#if !defined(MOTOR_ADC) || (MOTOR_ADC == 0)
  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE) != RESET)
    {
      __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
      motor_tick_dispatch();
    }
  }
#endif
}
