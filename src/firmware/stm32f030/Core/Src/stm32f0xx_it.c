#include "stm32f0xx_it.h"
#include "motor_tick.h"
#include "motor_pwm.h"
#include "tnb_i2c.h"
#include "reset_diag.h"
#if defined(MOTOR_ADC) && (MOTOR_ADC != 0)
#include "motor_adc.h"
#endif

_Static_assert(RESET_DIAG_FAULT == 2, "fault word");
_Static_assert(RESET_DIAG_FAULT_PC == 3, "fault_pc word");

/* CSS clear stays in C. The naked entry has already stored the stacked PC. */
__attribute__((used, noinline, noreturn))
static void nmi_rest(void)
{
  /* HSE Clock Security: SYSCLK has already fallen back to HSI. */
  if (__HAL_RCC_GET_IT(RCC_IT_CSS) != RESET)
  {
    __HAL_RCC_CLEAR_IT(RCC_IT_CSS);
  }
#if !defined(BOOST_BRINGUP) || (BOOST_BRINGUP == 0)
  MotorPwm_HardwareSafe();
#endif
  for (;;)
  {
  }
}

__attribute__((naked)) void NMI_Handler(void)
{
  __asm volatile(
      "mrs r0, msp\n"
      "ldr r0, [r0, #24]\n"
      "ldr r1, =g_reset_diag\n"
      "movs r2, #2\n"
      "str r2, [r1, #8]\n"
      "str r0, [r1, #12]\n"
      "b nmi_rest\n"
      ".ltorg\n");
}

__attribute__((naked)) void HardFault_Handler(void)
{
  __asm volatile(
      "mrs r0, msp\n"
      "ldr r0, [r0, #24]\n"
      "ldr r1, =g_reset_diag\n"
      "movs r2, #1\n"
      "str r2, [r1, #8]\n"
      "str r0, [r1, #12]\n"
#if !defined(BOOST_BRINGUP) || (BOOST_BRINGUP == 0)
      "bl MotorPwm_HardwareSafe\n"
#endif
      "1: b 1b\n"
      ".ltorg\n");
}
void SVC_Handler(void)          { }
void PendSV_Handler(void)       { }

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
#if !defined(BOOST_BRINGUP) || (BOOST_BRINGUP == 0)
  MotorTick_OnTim1Update();
  HAL_TIM_IRQHandler(&htim1);
#else
  TIM1->SR = 0U;
#endif
}

#if defined(MOTOR_ADC) && (MOTOR_ADC != 0) && (!defined(BOOST_BRINGUP) || (BOOST_BRINGUP == 0))
void DMA1_Channel1_IRQHandler(void)
{
  MotorAdc_DmaIrq();
}

/* Startup weak default spins here and wedges the core if ADC1_IRQn fires. */
void ADC1_IRQHandler(void)
{
}
#endif

void I2C1_IRQHandler(void)
{
#if !defined(BOOST_BRINGUP) || (BOOST_BRINGUP == 0)
  TnbI2c_IRQHandler();
#endif
}
