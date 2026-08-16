#include "main.h"
#include "motor_pwm.h"
#include "motor_tick.h"
#include "board_pins.h"

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef gpio = {0};

  if (htim->Instance == TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM1;

    gpio.Pin = BOARD_PWM_U_H_PIN | BOARD_PWM_V_H_PIN | BOARD_PWM_W_H_PIN;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = BOARD_PWM_U_L_PIN;
    HAL_GPIO_Init(BOARD_PWM_U_L_PORT, &gpio);

    gpio.Pin = BOARD_PWM_V_L_PIN | BOARD_PWM_W_L_PIN;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* PA6 nFAULT: input pull-up (same idle level as TARS BKIN). */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = BOARD_PWM_BKIN_PIN;
    HAL_GPIO_Init(BOARD_PWM_BKIN_PORT, &gpio);

    HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
  }
}
