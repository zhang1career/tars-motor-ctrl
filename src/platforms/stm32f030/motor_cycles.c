#include "motor_cycles.h"
#include "motor_tick.h"
#include "board_pins.h"
#include "stm32f0xx_hal.h"

static uint8_t s_scope_gpio_ready;

static void motor_cycles_scope_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  if (s_scope_gpio_ready != 0U)
  {
    return;
  }

  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin = BOARD_TP_CYCLE_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_TP_CYCLE_PORT, &gpio);
  HAL_GPIO_WritePin(BOARD_TP_CYCLE_PORT, BOARD_TP_CYCLE_PIN, GPIO_PIN_RESET);
  s_scope_gpio_ready = 1U;
}

void MotorCycles_Init(void)
{
  if (htim3.Instance == TIM3 && htim3.State == HAL_TIM_STATE_READY)
  {
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    (void)HAL_TIM_Base_Start(&htim3);
    return;
  }

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0U;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFFU;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    return;
  }

  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  (void)HAL_TIM_Base_Start(&htim3);
}

uint16_t MotorCycles_Read(void)
{
  return (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

uint16_t MotorCycles_Delta(uint16_t start, uint16_t end)
{
  return (uint16_t)(end - start);
}

static uint16_t motor_cycles_median8(uint16_t *v)
{
  uint8_t i;
  uint8_t j;
  uint16_t tmp;

  for (i = 1U; i < 8U; i++)
  {
    tmp = v[i];
    j = i;
    while ((j > 0U) && (v[j - 1U] > tmp))
    {
      v[j] = v[j - 1U];
      j--;
    }
    v[j] = tmp;
  }

  return (uint16_t)(((uint32_t)v[3U] + (uint32_t)v[4U]) / 2U);
}

uint16_t MotorCycles_Overhead(void)
{
  uint16_t samples[8];
  uint8_t i;

  for (i = 0U; i < 8U; i++)
  {
    uint16_t start;
    uint16_t end;

    __disable_irq();
    start = MotorCycles_Read();
    end = MotorCycles_Read();
    __enable_irq();
    samples[i] = MotorCycles_Delta(start, end);
  }

  return motor_cycles_median8(samples);
}

static uint16_t motor_cycles_measure_inner(void (*fn)(void), uint8_t scope)
{
  uint16_t start;
  uint16_t end;
  uint16_t overhead;
  uint16_t gross;

  overhead = MotorCycles_Overhead();

  if (scope != 0U)
  {
    motor_cycles_scope_init();
  }

  __disable_irq();
  if (scope != 0U)
  {
    HAL_GPIO_WritePin(BOARD_TP_CYCLE_PORT, BOARD_TP_CYCLE_PIN, GPIO_PIN_SET);
  }
  start = MotorCycles_Read();
  fn();
  end = MotorCycles_Read();
  if (scope != 0U)
  {
    HAL_GPIO_WritePin(BOARD_TP_CYCLE_PORT, BOARD_TP_CYCLE_PIN, GPIO_PIN_RESET);
  }
  __enable_irq();

  gross = MotorCycles_Delta(start, end);
  if (gross <= overhead)
  {
    return 0U;
  }
  return (uint16_t)(gross - overhead);
}

uint16_t MotorCycles_Measure(void (*fn)(void))
{
  return motor_cycles_measure_inner(fn, 0U);
}

uint16_t MotorCycles_MeasureScope(void (*fn)(void))
{
  return motor_cycles_measure_inner(fn, 1U);
}
