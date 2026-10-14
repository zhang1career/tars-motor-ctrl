#include "main.h"
#include "motor_app.h"
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

static void SystemClock_Config(void);
static void Watchdog_Init(void);
static void Watchdog_Kick(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  SystemCoreClockUpdate();

  MotorHall_Init();
  MotorPwm_Init();
  MotorTick_Init();
  TnbNode_SampleRails();
  MotorApp_Init();
  TnbNode_Init();
  UartLog_Init();
  Watchdog_Init();
  (void)motor_swd_entry[0];

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
    Watchdog_Kick();
  }
}

static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  /* PF0/PF1 = OSC_IN/OSC_OUT on LQFP-32; GPIO clock required before HSE start. */
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /* HSE 8 MHz × 6 = 48 MHz. If the crystal misses this window, stay on
   * the default HSI 8 MHz so I2C still answers; do not spin the motor. */
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL = RCC_PLL_MUL6;
  osc.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
  {
    HAL_Delay(80);
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
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
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  if (SystemCoreClock >= 40000000U)
  {
    return;
  }

  __HAL_RCC_GPIOF_CLK_ENABLE();
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL = RCC_PLL_MUL6;
  osc.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
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
  /* LSI ~40 kHz / 64, reload 1250 → about 2 s. Independent of SYSCLK. */
  IWDG->KR = 0x5555U;
  IWDG->PR = IWDG_PR_PR_2;
  IWDG->RLR = 1250U;
  while (IWDG->SR != 0U)
  {
  }
  IWDG->KR = 0xCCCCU;
}

static void Watchdog_Kick(void)
{
  IWDG->KR = 0xAAAAU;
}

void Error_Handler(void)
{
  MotorPwm_HardwareSafe();
  __disable_irq();
  for (;;)
  {
  }
}
