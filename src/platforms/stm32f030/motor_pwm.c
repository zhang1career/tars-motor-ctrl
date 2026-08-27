#include "motor_pwm.h"
#include "board_pins.h"
#include "main.h"

TIM_HandleTypeDef htim1;

#define MOTOR_TIM1_ARR  ((MOTOR_PWM_TIM_CLK / (2U * MOTOR_PWM_HZ)) - 1U)

/*
 * DTG is non-linear above 127: 0xx -> DTG*tDTS, 10x -> (64+DTG[5:0])*2*tDTS,
 * 110 -> (32+DTG[4:0])*8*tDTS. TARS uses DTG=216 on a 72 MHz tDTS.
 * At 48 MHz tDTS, DTG=72 -> 1.5 us; DTG=144 -> 3.33 us (TARS-equivalent time).
 * Dead time subtracts from the HS pulse, so it sets the usable duty floor.
 */
#ifndef MOTOR_TIM1_DTG
#define MOTOR_TIM1_DTG  72U
#endif
#define MOTOR_TIM1_DT   MOTOR_TIM1_DTG

static void motor_pwm_disable_oc_preload(TIM_TypeDef *tim)
{
  tim->CCMR1 &= ~(TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
  tim->CCMR2 &= ~TIM_CCMR2_OC3PE;
}

static void motor_pwm_force_update(TIM_TypeDef *tim)
{
  tim->EGR = TIM_EGR_UG;
}

#pragma GCC push_options
#pragma GCC optimize ("O0")

void MotorPwm_SetPhase(motor_phase_mode_t u, motor_phase_mode_t v, motor_phase_mode_t w,
                       uint8_t duty_pct)
{
  TIM_TypeDef *tim = htim1.Instance;
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  uint32_t pulse = (arr * (uint32_t)duty_pct) / 100U;

  if (pulse > arr)
  {
    pulse = arr;
  }

  switch (u)
  {
  case MOTOR_PHASE_OFF:
    tim->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
    tim->CCR1 = 0U;
    break;
  case MOTOR_PHASE_PWM:
    tim->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
    tim->CCR1 = pulse;
    break;
  case MOTOR_PHASE_LOW:
    /* CCxE cleared makes OCxN follow OCxREF directly (no complement, no dead
     * time), so OCxREF must stay active for the low side to conduct. */
    tim->CCER &= ~TIM_CCER_CC1E;
    tim->CCER |= TIM_CCER_CC1NE;
    tim->CCR1 = arr + 1U;
    break;
  default:
    tim->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
    tim->CCR1 = 0U;
    break;
  }

  switch (v)
  {
  case MOTOR_PHASE_OFF:
    tim->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE);
    tim->CCR2 = 0U;
    break;
  case MOTOR_PHASE_PWM:
    tim->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);
    tim->CCR2 = pulse;
    break;
  case MOTOR_PHASE_LOW:
    tim->CCER &= ~TIM_CCER_CC2E;
    tim->CCER |= TIM_CCER_CC2NE;
    tim->CCR2 = arr + 1U;
    break;
  default:
    tim->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE);
    tim->CCR2 = 0U;
    break;
  }

  switch (w)
  {
  case MOTOR_PHASE_OFF:
    tim->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE);
    tim->CCR3 = 0U;
    break;
  case MOTOR_PHASE_PWM:
    tim->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);
    tim->CCR3 = pulse;
    break;
  case MOTOR_PHASE_LOW:
    tim->CCER &= ~TIM_CCER_CC3E;
    tim->CCER |= TIM_CCER_CC3NE;
    tim->CCR3 = arr + 1U;
    break;
  default:
    tim->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE);
    tim->CCR3 = 0U;
    break;
  }

  motor_pwm_disable_oc_preload(tim);
  motor_pwm_force_update(tim);
}

#pragma GCC pop_options

/* Below this the current sign is noise (the front end reads +-3 mA), and
 * flipping the compensation on noise would inject a square wave at the noise
 * rate. Ramp linearly through the band instead of switching. */
#ifndef MOTOR_PWM_DT_COMP_BAND_A
#define MOTOR_PWM_DT_COMP_BAND_A 0.05f
#endif

static int32_t motor_pwm_dt_comp(float i)
{
  const float half_dt = (float)MOTOR_TIM1_DT * 0.5f;
  float scale;

  if (i > MOTOR_PWM_DT_COMP_BAND_A)
  {
    scale = 1.0f;
  }
  else if (i < -MOTOR_PWM_DT_COMP_BAND_A)
  {
    scale = -1.0f;
  }
  else
  {
    scale = i * (1.0f / MOTOR_PWM_DT_COMP_BAND_A);
  }

  return (int32_t)(scale * half_dt);
}

void MotorPwm_SetDuties(float da, float db, float dc, float ia, float ib, float ic)
{
  TIM_TypeDef *tim = htim1.Instance;
  int32_t arr = (int32_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
  const float duties[3] = {da, db, dc};
  const float currents[3] = {ia, ib, ic};
  int32_t ccr[3];
  uint8_t k;

  for (k = 0U; k < 3U; k++)
  {
    int32_t v = (int32_t)(duties[k] * (float)arr) + motor_pwm_dt_comp(currents[k]);

    if (v < 0)
    {
      v = 0;
    }
    else if (v > arr)
    {
      v = arr;
    }
    ccr[k] = v;
  }

  tim->CCR1 = (uint32_t)ccr[0];
  tim->CCR2 = (uint32_t)ccr[1];
  tim->CCR3 = (uint32_t)ccr[2];
  tim->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E |
                TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
}

static void motor_pwm_clear_break(TIM_TypeDef *tim)
{
  tim->SR &= ~TIM_SR_BIF;
}

void MotorPwm_HardwareSafe(void)
{
  TIM_TypeDef *tim = TIM1;

  tim->CCR1 = 0U;
  tim->CCR2 = 0U;
  tim->CCR3 = 0U;
  tim->CCER = 0U;
  tim->BDTR &= ~TIM_BDTR_MOE;
  motor_pwm_clear_break(tim);
}

void MotorPwm_PinsIdle(void)
{
  GPIO_InitTypeDef gpio = {0};

  MotorPwm_HardwareSafe();

  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  gpio.Pin = BOARD_PWM_U_H_PIN | BOARD_PWM_U_L_PIN | BOARD_PWM_V_H_PIN |
             BOARD_PWM_W_H_PIN;
  HAL_GPIO_Init(GPIOA, &gpio);
  HAL_GPIO_WritePin(GPIOA, gpio.Pin, GPIO_PIN_RESET);

  gpio.Pin = BOARD_PWM_V_L_PIN | BOARD_PWM_W_L_PIN;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, gpio.Pin, GPIO_PIN_RESET);
}

void MotorPwm_RestoreAfPins(void)
{
  GPIO_InitTypeDef gpio = {0};

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
}

void MotorPwm_Init(void)
{
  TIM_MasterConfigTypeDef master = {0};
  TIM_OC_InitTypeDef oc = {0};
  TIM_BreakDeadTimeConfigTypeDef bdt = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0U;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = MOTOR_TIM1_ARR;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  /* Center-aligned counting raises an update event at both overflow and
   * underflow, so RCR=1 is needed for one control tick per PWM period
   * (MOTOR_CTRL_ISR_HZ). RCR=0 ran the control loop at twice MOTOR_PWM_HZ. */
  htim1.Init.RepetitionCounter = 1U;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  (void)HAL_TIMEx_MasterConfigSynchronization(&htim1, &master);

  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0U;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  bdt.OffStateRunMode = TIM_OSSR_ENABLE;
  bdt.OffStateIDLEMode = TIM_OSSI_ENABLE;
  bdt.LockLevel = TIM_LOCKLEVEL_OFF;
  bdt.DeadTime = MOTOR_TIM1_DT;
  /* F030 BKIN AF differs from F429; keep break off until MOE path verified. */
  bdt.BreakState = TIM_BREAK_DISABLE;
  bdt.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  bdt.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bdt) != HAL_OK)
  {
    Error_Handler();
  }

  motor_pwm_disable_oc_preload(htim1.Instance);
  motor_pwm_clear_break(htim1.Instance);
  /* One update event per PWM period (center-aligned), like TARS TIM1 TRGO. */
  htim1.Instance->CR1 |= TIM_CR1_URS;
  MotorPwm_HardwareSafe();
}

static int motor_pwm_start_channels(void)
{
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    return 0;
  }

  (void)HAL_TIM_Base_Start(&htim1);
  return 1;
}

int MotorPwm_ArmOutputs(void)
{
  MotorPwm_RestoreAfPins();

  /* Recover HAL channel state after PinsIdle or a partial stop. */
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
  MotorPwm_HardwareSafe();

  if ((htim1.Instance->CR1 & TIM_CR1_CEN) == 0U)
  {
    (void)HAL_TIM_Base_Start(&htim1);
  }

  return motor_pwm_start_channels();
}

void MotorPwm_MoeEnable(void)
{
  motor_pwm_clear_break(htim1.Instance);
  __HAL_TIM_MOE_ENABLE(&htim1);
}

int MotorPwm_Start(void)
{
  if (MotorPwm_ArmOutputs() == 0)
  {
    return 0;
  }

  MotorPwm_MoeEnable();
  return 1;
}

void MotorPwm_Stop(void)
{
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
  MotorPwm_HardwareSafe();
  (void)HAL_TIM_Base_Start(&htim1);
}
