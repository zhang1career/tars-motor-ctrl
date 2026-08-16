#include "motor_hall.h"
#include "board_pins.h"
#include "stm32f0xx_hal.h"

void MotorHall_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  gpio.Pin = BOARD_HALL_A_PIN;
  HAL_GPIO_Init(BOARD_HALL_A_PORT, &gpio);

  gpio.Pin = BOARD_HALL_B_PIN;
  HAL_GPIO_Init(BOARD_HALL_B_PORT, &gpio);

  gpio.Pin = BOARD_HALL_C_PIN;
  HAL_GPIO_Init(BOARD_HALL_C_PORT, &gpio);
}

uint8_t MotorHall_ReadRaw(void)
{
  uint8_t ha = (HAL_GPIO_ReadPin(BOARD_HALL_A_PORT, BOARD_HALL_A_PIN) != GPIO_PIN_RESET) ? 1U : 0U;
  uint8_t hb = (HAL_GPIO_ReadPin(BOARD_HALL_B_PORT, BOARD_HALL_B_PIN) != GPIO_PIN_RESET) ? 1U : 0U;
  uint8_t hc = (HAL_GPIO_ReadPin(BOARD_HALL_C_PORT, BOARD_HALL_C_PIN) != GPIO_PIN_RESET) ? 1U : 0U;

  return (uint8_t)(ha | (hb << 1) | (hc << 2));
}
