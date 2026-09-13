#include "motor_adc.h"
#include "motor_pwm.h"
#include "motor_tick.h"
#include "board_pins.h"
#include "main.h"

volatile uint16_t g_motor_adc_raw[MOTOR_ADC_CH_COUNT];

static ADC_HandleTypeDef s_hadc;
static DMA_HandleTypeDef s_hdma;
static uint8_t s_ready;

static void motor_adc_pins_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
             GPIO_PIN_5;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);
}

/* TIM1 CH4 exists only to time the ADC: PWM2 mode gives one OC4REF rising edge
 * per period at CCR4, and TRGO forwards it. CC4E is left clear so nothing
 * reaches PA11. */
static void motor_adc_trigger_init(void)
{
  TIM_OC_InitTypeDef oc = {0};
  TIM_MasterConfigTypeDef master = {0};
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  uint32_t ccr4;

  ccr4 = (arr > MOTOR_ADC_LEAD_COUNTS) ? (arr - MOTOR_ADC_LEAD_COUNTS) : 0U;

  oc.OCMode = TIM_OCMODE_PWM2;
  oc.Pulse = ccr4;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  master.MasterOutputTrigger = TIM_TRGO_OC4REF;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  (void)HAL_TIMEx_MasterConfigSynchronization(&htim1, &master);
}

void MotorAdc_Init(void)
{
  ADC_ChannelConfTypeDef ch = {0};

  if (s_ready != 0U)
  {
    return;
  }

  motor_adc_pins_init();
  motor_adc_trigger_init();

  __HAL_RCC_DMA1_CLK_ENABLE();
  s_hdma.Instance = DMA1_Channel1;
  s_hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
  s_hdma.Init.PeriphInc = DMA_PINC_DISABLE;
  s_hdma.Init.MemInc = DMA_MINC_ENABLE;
  s_hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  s_hdma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  s_hdma.Init.Mode = DMA_CIRCULAR;
  s_hdma.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&s_hdma) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_ADC1_CLK_ENABLE();
  s_hadc.Instance = ADC1;
  /* PCLK/4 = 12 MHz. PCLK/2 would be 24 MHz and the F030 ADC is specified to
   * 14 MHz. Synchronous to PCLK, so no HSI14 jitter against the timer. */
  s_hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  s_hadc.Init.Resolution = ADC_RESOLUTION_12B;
  s_hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  s_hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  s_hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  s_hadc.Init.LowPowerAutoWait = DISABLE;
  s_hadc.Init.LowPowerAutoPowerOff = DISABLE;
  s_hadc.Init.ContinuousConvMode = DISABLE;
  s_hadc.Init.DiscontinuousConvMode = DISABLE;
  s_hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  s_hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  s_hadc.Init.DMAContinuousRequests = ENABLE;
  /* Overwrite rather than stall: one lost conversion must not wedge the chain
   * that the current loop depends on. A shifted channel mapping is detectable
   * because VBUS reads ~3020 while the current channels sit near 2048. */
  s_hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  s_hadc.Init.SamplingTimeCommon = ADC_SAMPLETIME_7CYCLES_5;

  __HAL_LINKDMA(&s_hadc, DMA_Handle, s_hdma);

  if (HAL_ADC_Init(&s_hadc) != HAL_OK)
  {
    Error_Handler();
  }

  ch.Channel = BOARD_ADC_IU_CH;
  ch.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&s_hadc, &ch) != HAL_OK)
  {
    Error_Handler();
  }
  ch.Channel = BOARD_ADC_IV_CH;
  (void)HAL_ADC_ConfigChannel(&s_hadc, &ch);
  ch.Channel = BOARD_ADC_IW_CH;
  (void)HAL_ADC_ConfigChannel(&s_hadc, &ch);
  ch.Channel = BOARD_ADC_VBUS_CH;
  (void)HAL_ADC_ConfigChannel(&s_hadc, &ch);

  if (HAL_ADCEx_Calibration_Start(&s_hadc) != HAL_OK)
  {
    Error_Handler();
  }

#if defined(MOTOR_ADC_STROBE) && (MOTOR_ADC_STROBE != 0)
  {
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = BOARD_TP_CYCLE_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BOARD_TP_CYCLE_PORT, &gpio);
    HAL_GPIO_WritePin(BOARD_TP_CYCLE_PORT, BOARD_TP_CYCLE_PIN, GPIO_PIN_RESET);
  }
#endif

  /* Same priority as TIM1: the control tick lives here, not on update. */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  s_ready = 1U;
}

void MotorAdc_EnableTick(void)
{
  DMA1->IFCR = DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;
  DMA1_Channel1->CCR &= ~DMA_CCR_HTIE;
  DMA1_Channel1->CCR |= DMA_CCR_TCIE;
}

void MotorAdc_DisableTick(void)
{
  DMA1_Channel1->CCR &= ~DMA_CCR_TCIE;
}

void MotorAdc_DmaIrq(void)
{
  uint32_t isr = DMA1->ISR;

  if ((isr & DMA_ISR_TCIF1) != 0U)
  {
    DMA1->IFCR = DMA_IFCR_CTCIF1;
#if defined(MOTOR_ADC_STROBE) && (MOTOR_ADC_STROBE != 0)
    /* Pulse first: it marks sample end, and the edges must not sit in the
     * next sequence (measurement-validity 3.9). */
    BOARD_TP_CYCLE_PORT->BSRR = BOARD_TP_CYCLE_PIN;
    {
      volatile uint32_t n;
      for (n = 0U; n < 4U; n++)
      {
      }
    }
    BOARD_TP_CYCLE_PORT->BRR = BOARD_TP_CYCLE_PIN;
#endif
    MotorTick_OnAdcComplete();
  }
  if ((isr & DMA_ISR_HTIF1) != 0U)
  {
    DMA1->IFCR = DMA_IFCR_CHTIF1;
  }
  if ((isr & DMA_ISR_TEIF1) != 0U)
  {
    DMA1->IFCR = DMA_IFCR_CTEIF1;
  }
}

int MotorAdc_Start(void)
{
  if (s_ready == 0U)
  {
    MotorAdc_Init();
  }

  if (HAL_ADC_Start_DMA(&s_hadc, (uint32_t *)(void *)g_motor_adc_raw,
                        MOTOR_ADC_CH_COUNT) != HAL_OK)
  {
    return 0;
  }
  /* HAL_DMA_Start_IT also arms HT; one sequence must make one tick. TCIE
   * stays off until MotorTick_Start so init does not run the controller. */
  DMA1_Channel1->CCR &= ~(DMA_CCR_HTIE | DMA_CCR_TCIE);
  return 1;
}

void MotorAdc_Stop(void)
{
  if (s_ready != 0U)
  {
    (void)HAL_ADC_Stop_DMA(&s_hadc);
  }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC1_CLK_ENABLE();
  }
}
