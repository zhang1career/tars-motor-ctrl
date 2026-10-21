#include "boost.h"

#include "board_pins.h"
#include "main.h"
#include "motor_adc.h"
#include "motor_pwm.h"

/*
 * g_boost is read over SWD while this runs. Layout is eight halfwords:
 * magic, state, adc_idle, adc, duty, mv, adc_min, adc_max.
 */
struct boost_mon {
  uint16_t magic;
  uint16_t state;
  uint16_t adc_idle;
  uint16_t adc;
  uint16_t duty;
  uint16_t mv;
  uint16_t adc_min;
  uint16_t adc_max;
};

volatile struct boost_mon g_boost __attribute__((aligned(4)));

static uint16_t s_duty;
static uint16_t s_filt;
static uint8_t s_on;
static uint32_t s_last;
static uint32_t s_step_ms;
static uint8_t s_div = 1U;

static void boost_kick(void)
{
  IWDG->KR = 0xAAAAU;
}

/* PA11. MODER bits 22-23, AFRH bits 12-15. */
#define BOOST_PIN_MODER (3U << 22)
#define BOOST_PIN_AFR   (0xFU << 12)

static void boost_pin_low(void)
{
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA->BSRR = (uint32_t)BOARD_CP_DRIVE_PIN << 16;
  GPIOA->MODER = (GPIOA->MODER & ~BOOST_PIN_MODER) | (1U << 22);
  GPIOA->PUPDR &= ~BOOST_PIN_MODER;
}

static void boost_pin_af(void)
{
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~BOOST_PIN_AFR) | (2U << 12);
  GPIOA->OSPEEDR |= BOOST_PIN_MODER;
  GPIOA->MODER = (GPIOA->MODER & ~BOOST_PIN_MODER) | (2U << 22);
}

/* Center-aligned, same ARR as the motor timer, CH4 only. CH1-3 stay off. */
static void boost_timer(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();
  TIM1->CR1 = 0U;
  TIM1->PSC = 0U;
  TIM1->ARR = 1199U;
  TIM1->RCR = 0U;
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;
  TIM1->CCR4 = 0U;
  TIM1->CCMR2 = (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
  TIM1->CCER = TIM_CCER_CC4E;
  TIM1->BDTR = TIM_BDTR_MOE;
  TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CMS_0;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->CR1 |= TIM_CR1_CEN;
}

static void boost_adc_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_4;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  __HAL_RCC_ADC1_CLK_ENABLE();
  ADC1->CR &= ~ADC_CR_ADEN;
  ADC1->CFGR2 = ADC_CFGR2_CKMODE_1;
  ADC1->CFGR1 = 0U;
  ADC1->SMPR = ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2;
  ADC1->CHSELR = ADC_CHSELR_CHSEL4;
  ADC1->CR |= ADC_CR_ADCAL;
  while ((ADC1->CR & ADC_CR_ADCAL) != 0U)
  {
    boost_kick();
  }
  ADC1->ISR = ADC_ISR_ADRDY;
  ADC1->CR |= ADC_CR_ADEN;
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U)
  {
    boost_kick();
  }
}

static uint16_t boost_adc(void)
{
  ADC1->CR |= ADC_CR_ADSTART;
  while ((ADC1->ISR & ADC_ISR_EOC) == 0U)
  {
  }
  return (uint16_t)ADC1->DR;
}

static uint16_t boost_avg(void)
{
  uint32_t sum = 0U;
  uint32_t i;

  for (i = 0U; i < 32U; i++)
  {
    sum += boost_adc();
  }
  return (uint16_t)(sum >> 5);
}

static uint16_t boost_mv(uint16_t adc)
{
  return (uint16_t)(((uint32_t)adc * 3300U * BOOST_DIV_NUM) / 4095U);
}

static uint16_t boost_ccr_hw(uint16_t duty)
{
  if (duty > BOOST_DUTY_MAX)
  {
    duty = BOOST_DUTY_MAX;
  }
  return (uint16_t)(((uint32_t)duty * (uint32_t)BOOST_CCR_HW_MAX +
                     ((uint32_t)BOOST_DUTY_MAX / 2U)) /
                    (uint32_t)BOOST_DUTY_MAX);
}

static void boost_force(uint16_t duty)
{
  if (duty > BOOST_DUTY_MAX)
  {
    duty = BOOST_DUTY_MAX;
  }
  s_duty = duty;
  TIM1->CCR4 = boost_ccr_hw(duty);
}

void Boost_Step(uint16_t adc)
{
  uint16_t duty = s_duty;

  if (adc > BOOST_OV_CODE)
  {
    duty = 0U;
  }
  else if ((uint16_t)(adc + BOOST_BAND) < BOOST_SET_CODE)
  {
    if (duty < BOOST_DUTY_MAX)
    {
      duty++;
    }
  }
  else if (adc > (uint16_t)(BOOST_SET_CODE + BOOST_BAND))
  {
    if (duty > 0U)
    {
      duty--;
    }
  }

  boost_force(duty);
}

uint8_t Boost_Ready(void)
{
  if (s_filt < 800U)
  {
    return 0U;
  }
  return (s_filt >= BOOST_READY_LO) && (s_filt <= BOOST_OV_CODE);
}

void Boost_KeepAlive(void)
{
  if (s_on == 0U)
  {
    return;
  }
  TIM1->CCER |= TIM_CCER_CC4E;
  if ((TIM1->SR & TIM_SR_BIF) == 0U)
  {
    TIM1->BDTR |= TIM_BDTR_MOE;
  }
}

void Boost_Resync(void)
{
  uint32_t primask;

  if ((TIM3->ARR == 0U) ||
      (((TIM1->CR1 & TIM_CR1_CEN) != 0U) &&
       ((TIM3->CR1 & TIM_CR1_CEN) != 0U)))
  {
    if ((TIM1->CR1 & TIM_CR1_CEN) == 0U)
    {
      TIM1->CR1 |= TIM_CR1_CEN;
    }
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM3->CR1 &= ~TIM_CR1_CEN;
  TIM1->EGR = TIM_EGR_UG;
  TIM3->EGR = TIM_EGR_UG;
  TIM1->SR = 0U;
  TIM3->SR = 0U;
  TIM3->CR1 |= TIM_CR1_CEN;
  TIM1->CR1 |= TIM_CR1_CEN;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static uint16_t boost_read_pa5(void)
{
  uint16_t dma = g_motor_adc_raw[MOTOR_ADC_VBOOST];
  uint16_t poll = MotorAdc_VboostRaw();
  uint16_t adc = dma;

  if (poll >= 800U)
  {
    /* DMA can read PA4/VBUS (~3200) in the VBOOST slot after reset; trust PA5. */
    if ((dma < 800U) || (dma + 400U < poll) || (dma > (uint16_t)(poll + 400U)))
    {
      adc = poll;
    }
  }
  else if (dma < 800U)
  {
    adc = poll;
  }
  return adc;
}

void Boost_Init(void)
{
  if (SystemCoreClock < 40000000U)
  {
    g_boost.state = BOOST_ST_CLOCK;
    return;
  }
  /* CH3 occupies the low half of CCMR2. Touch only CH4. */
  TIM1->CCMR2 = (TIM1->CCMR2 & ~TIM_CCMR2_OC4M_Msk) |
                (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
  TIM1->CCR4 = 0U;
  s_filt = 0U;
  s_duty = 0U;
  s_last = 0U;
  s_step_ms = 0U;
  g_boost.magic = 0xB005U;
  g_boost.state = BOOST_ST_RUN;
  boost_pin_af();
  (void)MotorAdc_BootstrapVboost();
  TIM1->SR &= ~TIM_SR_BIF;
  s_on = 1U;
}

void Boost_Poll(void)
{
  uint32_t now;
  uint16_t adc;
  uint16_t duty;

  if (s_on == 0U)
  {
    return;
  }

  if ((g_motor_pwm_prot.ocp_zero_ready == 0U) && (HAL_GetTick() < 200U))
  {
    return;
  }

  {
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (((TIM1->SR & TIM_SR_BIF) != 0U) &&
        ((GPIOA->IDR & BOARD_NFAULT_PIN) != 0U) &&
        ((TIM1->CCER & (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E |
                        TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE)) == 0U))
    {
      TIM1->SR = ~TIM_SR_BIF;
    }
    if (primask == 0U)
    {
      __enable_irq();
    }
  }

  Boost_KeepAlive();

  if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
  {
    boost_force(0U);
    g_boost.duty = 0U;
    return;
  }

  now = HAL_GetTick();
  if (now == s_last)
  {
    return;
  }
  s_last = now;

  adc = boost_read_pa5();
  if (adc < 800U)
  {
    boost_force(0U);
    s_filt = 0U;
    g_boost.duty = 0U;
    return;
  }

  if (s_filt == 0U)
  {
    s_filt = adc;
  }
  else
  {
    s_filt = (uint16_t)(((uint32_t)s_filt * 15U + (uint32_t)adc) >> 4);
  }
  g_boost.adc = s_filt;
  g_boost.mv = boost_mv(s_filt);
  g_boost.state = BOOST_ST_RUN;

  duty = s_duty;
  if (adc > BOOST_OV_CODE)
  {
    duty = 0U;
  }
  else if ((now - s_step_ms) >= BOOST_STEP_MIN_MS)
  {
    if (s_filt < BOOST_LO_CODE)
    {
      if (duty < BOOST_DUTY_MAX)
      {
        duty++;
        s_step_ms = now;
      }
    }
    else if (s_filt > BOOST_HI_CODE)
    {
      if (duty > 0U)
      {
        duty--;
        s_step_ms = now;
      }
    }
  }

  boost_force(duty);
  g_boost.duty = s_duty;
}

void Boost_OnPwmTick(uint16_t adc)
{
  if (--s_div != 0U)
  {
    return;
  }
  s_div = 20U;
  Boost_Step(adc);
}

static void boost_hold(uint16_t state, uint16_t adc)
{
  boost_force(0U);
  g_boost.state = state;
  g_boost.adc = adc;
  g_boost.duty = 0U;
  g_boost.mv = boost_mv(adc);
  for (;;)
  {
    boost_kick();
  }
}

void Boost_Bringup(void)
{
  uint16_t idle;
  uint16_t after;
  int16_t delta;
  uint32_t waited;

  g_boost.magic = 0xB005U;
  g_boost.adc_min = 4095U;
  g_boost.adc_max = 0U;
  boost_pin_low();

  if (SystemCoreClock < 40000000U)
  {
    boost_hold(BOOST_ST_CLOCK, 0U);
  }

  boost_adc_init();
  boost_timer();
  boost_pin_af();
  boost_force(0U);

  idle = boost_avg();
  g_boost.adc_idle = idle;
  g_boost.adc = idle;
  g_boost.mv = boost_mv(idle);
  if ((idle < 800U) || (idle > 2200U))
  {
    boost_hold(BOOST_ST_RANGE, idle);
  }

  /* A short full-scale pulse. The 19k divider, if it is across the output,
   * already draws more than a small pulse can supply, so the output either
   * rises or sags. The +24V input does neither. */
  boost_force(BOOST_DUTY_MAX);
  for (waited = 0U; waited < 150U; waited++)
  {
    HAL_Delay(1);
    boost_kick();
  }
  boost_force(0U);
  after = boost_avg();
  delta = (int16_t)((int32_t)after - (int32_t)idle);
  g_boost.adc = after;
  g_boost.mv = boost_mv(after);
  if ((delta > -20) && (delta < 20))
  {
    boost_hold(BOOST_ST_SENSE, after);
  }

  boost_force(0U);
  g_boost.state = BOOST_ST_RUN;
  g_boost.adc_min = 4095U;
  g_boost.adc_max = 0U;
  waited = HAL_GetTick();
  for (;;)
  {
    uint16_t adc;
    uint32_t now;
    static uint16_t win_min = 4095U;
    static uint16_t win_max = 0U;
    static uint16_t win_n;
    static uint16_t flat_ms;

    now = HAL_GetTick();
    if (now == waited)
    {
      boost_kick();
      continue;
    }
    waited = now;
    adc = boost_adc();
    Boost_Step(adc);
    /* Max duty that does not move PA4 is the divider sitting on +24V.
     * Stop before an unloaded output can reach the diode rating. */
    if ((s_duty == BOOST_DUTY_MAX) && (adc < (uint16_t)(idle + 15U)))
    {
      if (++flat_ms >= 200U)
      {
        boost_hold(BOOST_ST_SENSE, adc);
      }
    }
    else
    {
      flat_ms = 0U;
    }
    g_boost.adc = adc;
    g_boost.duty = s_duty;
    g_boost.mv = boost_mv(adc);
    if (adc < win_min)
    {
      win_min = adc;
    }
    if (adc > win_max)
    {
      win_max = adc;
    }
    if (++win_n >= 500U)
    {
      g_boost.adc_min = win_min;
      g_boost.adc_max = win_max;
      win_min = 4095U;
      win_max = 0U;
      win_n = 0U;
    }
    boost_kick();
  }
}
