#include "motor_tick.h"
#include "hall6.h"
#include "openloop.h"
#include "motor_pwm.h"
#include "main.h"
#include "stm32f0xx_hal.h"

TIM_HandleTypeDef htim3;

static void motor_tick_dispatch(void)
{
  if (MotorHall6_IsEnabled() != 0)
  {
    MotorHall6_ControlLoopISR();
  }
  else if (MotorOpenloop_IsEnabled() != 0)
  {
    MotorOpenloop_ControlLoopISR();
  }
}

void MotorTick_Init(void)
{
  /* TIM3 unused; control loop runs from TIM1 update (TARS-aligned). */
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    motor_tick_dispatch();
  }
}
