#include "stm32f0xx_it.h"
#include "motor_tick.h"
#include "motor_pwm.h"

void NMI_Handler(void)          { }
void HardFault_Handler(void)    { for (;;) { } }
void SVC_Handler(void)          { }
void PendSV_Handler(void)       { }

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  MotorTick_OnTim1Update();
  HAL_TIM_IRQHandler(&htim1);
}
