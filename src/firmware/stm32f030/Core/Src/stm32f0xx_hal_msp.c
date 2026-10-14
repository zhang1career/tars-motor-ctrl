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
  /* Free-running cycle counter for motor_cycles.c; polled, so no NVIC entry. */
  if (htim->Instance == TIM16)
  {
    __HAL_RCC_TIM16_CLK_ENABLE();
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

    /* Charge pump off. R25 2k2 ties this pin to Q3's base, so leaving it in the
     * reset input state lets the pump bias itself on. Drive it before anything
     * else on the power stage moves. */
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = BOARD_CP_DRIVE_PIN;
    HAL_GPIO_Init(BOARD_CP_DRIVE_PORT, &gpio);
    HAL_GPIO_WritePin(BOARD_CP_DRIVE_PORT, BOARD_CP_DRIVE_PIN, GPIO_PIN_RESET);

    /* nFAULT is TIM1_BKIN (AF2 on PA6). IDR still reads the pin, so
     * board_check can see the comparator. No internal pull-up: a missing
     * board 4k7 must look like a fault, not be masked. nOTEMP stays GPIO. */
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM1;
    gpio.Pin = BOARD_PWM_BKIN_PIN;
    HAL_GPIO_Init(BOARD_PWM_BKIN_PORT, &gpio);

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin = BOARD_NOTEMP_PIN;
    HAL_GPIO_Init(BOARD_NOTEMP_PORT, &gpio);

    HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
  }
}
