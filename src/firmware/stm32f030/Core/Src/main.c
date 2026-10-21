#include "main.h"
#include "motor_app.h"
#include "motor_vsweep.h"
#include "tnb_node.h"
#include "uart_log.h"
#if defined(MOTOR_FOC_FLOAT) && (MOTOR_FOC_FLOAT != 0)
#include "motor_foc.h"
#endif
#if defined(MOTOR_START_BENCH) && (MOTOR_START_BENCH != 0)
#include "ctrl_bench.h"
#endif
#include "motor_pwm.h"
#include "motor_hall.h"
#include "motor_tick.h"
#include "reset_diag.h"
#include "boost.h"

static void SystemClock_Config(void);
static void Watchdog_Init(void);
static void Watchdog_Kick(void) __attribute__((unused));
static void ResetDiag_Boot(void);

volatile uint32_t g_reset_diag[RESET_DIAG_WORDS]
    __attribute__((section(".noinit"), aligned(4), used));

int main(void)
{
  /* HW option-byte IWDG can run before Watchdog_Init; survive long init. */
  IWDG->KR = 0xAAAAU;
  ResetDiag_Boot();
  HAL_Init();
  SystemClock_Config();
  SystemCoreClockUpdate();

#if defined(BOOST_BRINGUP) && (BOOST_BRINGUP != 0)
  /* Motor bridge stays off. PA11 is the boost switch only. */
  Watchdog_Init();
  Boost_Bringup();
#else
  MotorHall_Init();
  MotorPwm_Init();
  MotorTick_Init();
  TnbNode_SampleRails();
  MotorApp_Init();
  Boost_Init();
  TnbNode_Init();
  UartLog_Init();
  Watchdog_Init();
  (void)motor_swd_entry[0];

  /* Charge pump must rise without any host/mot command. Warm the poll loop
   * before while(1) so duty starts stepping immediately after init. */
  {
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < 200U)
    {
      MotorPwm_PollOcpZero();
      Boost_Poll();
      Watchdog_Kick();
    }
  }

#if defined(MOTOR_START_BENCH) && (MOTOR_START_BENCH != 0)
  /* Pure CPU measurement with the motor off. Reached by reset rather than by a
   * gdb call, which docs/roadmap.md 9.5 rules out for measurements. */
  MotorCtrlBench_Run();
#endif

#if defined(MOTOR_AUTO_START) && (MOTOR_AUTO_START != 0)
#if defined(MOTOR_START_FOC_OBSERVE) && (MOTOR_START_FOC_OBSERVE != 0)
  (void)MotorApp_StartFocObserve();
#elif defined(MOTOR_START_DIAG) && (MOTOR_START_DIAG != 0)
  (void)MotorApp_StartDiagStep();
#elif defined(MOTOR_START_HALL6) && (MOTOR_START_HALL6 != 0)
  (void)MotorApp_Start();
#else
  (void)MotorApp_StartOpenloop6();
#endif
#endif

  for (;;)
  {
#if defined(MOTOR_FOC_FLOAT) && (MOTOR_FOC_FLOAT != 0)
    MotorFoc_Step();
#endif
    TnbNode_Task();
    Boost_Poll();
    MotorVsweep_Poll();
    g_reset_diag[RESET_DIAG_MAIN_TICK_SEEN] = g_reset_diag[RESET_DIAG_TICK_TOTAL];
    Watchdog_Kick();
  }
#endif
}

static void ResetDiag_Boot(void)
{
  uint32_t i;

  if (g_reset_diag[RESET_DIAG_MAGIC_I] != RESET_DIAG_MAGIC)
  {
    for (i = 0U; i < RESET_DIAG_WORDS; i++)
    {
      g_reset_diag[i] = 0U;
    }
    g_reset_diag[RESET_DIAG_MAGIC_I] = RESET_DIAG_MAGIC;
  }
  else
  {
    g_reset_diag[RESET_DIAG_TICK_GAP] =
        g_reset_diag[RESET_DIAG_TICK_TOTAL] - g_reset_diag[RESET_DIAG_MAIN_TICK_SEEN];
  }
  g_reset_diag[RESET_DIAG_RCC_CSR] = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;
}

/* HSE 8 MHz × 6. The HAL oscillator call carries every other source and
 * costs more flash than this board uses. */
static HAL_StatusTypeDef clock_hse_pll(void)
{
  uint32_t t0;

  if (__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK)
  {
    if ((__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE) &&
        (READ_BIT(RCC->CFGR, RCC_CFGR_PLLMUL) == RCC_PLL_MUL6) &&
        (READ_BIT(RCC->CFGR2, RCC_CFGR2_PREDIV) == RCC_PREDIV_DIV1))
    {
      return HAL_OK;
    }
    return HAL_ERROR;
  }

  __HAL_RCC_HSE_CONFIG(RCC_HSE_ON);
  t0 = HAL_GetTick();
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
  {
    if ((HAL_GetTick() - t0) > HSE_TIMEOUT_VALUE)
    {
      return HAL_TIMEOUT;
    }
  }

  __HAL_RCC_PLL_DISABLE();
  t0 = HAL_GetTick();
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET)
  {
    if ((HAL_GetTick() - t0) > PLL_TIMEOUT_VALUE)
    {
      return HAL_TIMEOUT;
    }
  }

  __HAL_RCC_PLL_CONFIG(RCC_PLLSOURCE_HSE, RCC_PREDIV_DIV1, RCC_PLL_MUL6);
  __HAL_RCC_PLL_ENABLE();
  t0 = HAL_GetTick();
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
  {
    if ((HAL_GetTick() - t0) > PLL_TIMEOUT_VALUE)
    {
      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clk = {0};

  /* PF0/PF1 = OSC_IN/OSC_OUT on LQFP-32; GPIO clock required before HSE start. */
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /* If the crystal misses this window, stay on the default HSI 8 MHz so I2C
   * still answers; do not spin the motor. */
  if (clock_hse_pll() != HAL_OK)
  {
    HAL_Delay(80);
    if (clock_hse_pll() != HAL_OK)
    {
      return;
    }
  }

  clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) == HAL_OK)
  {
    HAL_RCC_EnableCSS();
  }
}

void MotorClock_Retry(void)
{
  RCC_ClkInitTypeDef clk = {0};

  if (SystemCoreClock >= 40000000U)
  {
    return;
  }

  __HAL_RCC_GPIOF_CLK_ENABLE();
  if (clock_hse_pll() != HAL_OK)
  {
    return;
  }
  clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) == HAL_OK)
  {
    HAL_RCC_EnableCSS();
    SystemCoreClockUpdate();
  }
}

static void Watchdog_Init(void)
{
  /* LSI ~40 kHz / 64, reload 1250 → about 2 s. Independent of SYSCLK.
   * The prescaler update only finishes while LSI is running. Start the
   * watchdog first so that clock is forced on, then apply the 2 s reload. */
  RCC->CSR |= RCC_CSR_LSION;
  while ((RCC->CSR & RCC_CSR_LSIRDY) == 0U)
  {
  }
  IWDG->KR = 0xCCCCU;
  IWDG->KR = 0x5555U;
  IWDG->PR = IWDG_PR_PR_2;
  IWDG->RLR = 1250U;
  while (IWDG->SR != 0U)
  {
    IWDG->KR = 0xAAAAU;
  }
}

static void Watchdog_Kick(void)
{
  IWDG->KR = 0xAAAAU;
}

void Error_Handler(void)
{
  g_reset_diag[RESET_DIAG_FAULT] = 3U;
  g_reset_diag[RESET_DIAG_FAULT_PC] = (uint32_t)__builtin_return_address(0);
#if !defined(BOOST_BRINGUP) || (BOOST_BRINGUP == 0)
  MotorPwm_HardwareSafe();
#endif
  __disable_irq();
  for (;;)
  {
  }
}
