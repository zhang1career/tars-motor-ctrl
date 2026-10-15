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
  uint32_t idr;

  /* A, B, C are PB3, PB4, PB5, so one IDR read packs the old 3-bit code. */
  _Static_assert(BOARD_HALL_A_PIN == GPIO_PIN_3, "hall A is not PB3");
  _Static_assert(BOARD_HALL_B_PIN == GPIO_PIN_4, "hall B is not PB4");
  _Static_assert(BOARD_HALL_C_PIN == GPIO_PIN_5, "hall C is not PB5");
  _Static_assert(BOARD_HALL_A_PORT == GPIOB, "hall A port is not GPIOB");
  _Static_assert(BOARD_HALL_B_PORT == GPIOB, "hall B port is not GPIOB");
  _Static_assert(BOARD_HALL_C_PORT == GPIOB, "hall C port is not GPIOB");

  idr = GPIOB->IDR;
  return (uint8_t)((idr >> 3) & 7U);
}
