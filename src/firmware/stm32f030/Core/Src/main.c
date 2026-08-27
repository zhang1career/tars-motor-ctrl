#include "main.h"
#include "motor_app.h"
#if defined(MOTOR_START_BENCH) && (MOTOR_START_BENCH != 0)
#include "ctrl_bench.h"
#endif
#if defined(MOTOR_FOC) && (MOTOR_FOC != 0)
#include "motor_foc.h"
#endif
#include "motor_pwm.h"
#include "motor_hall.h"
#include "motor_tick.h"

static void SystemClock_Config(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MotorHall_Init();
  MotorPwm_Init();
  MotorTick_Init();
  MotorApp_Init();
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
#if defined(MOTOR_FOC) && (MOTOR_FOC != 0)
    /* Float FOC observation runs here rather than in the control ISR: it takes
     * 16173 cycles and the tick is 2400 (roadmap 3.3.2). The ISR preempts it, so
     * the control loop is unaffected; the cost is only that samples arrive at
     * whatever rate the loop achieves, which is plenty for watching id.
     * MotorFoc_Step() returns immediately unless a mode is set. */
    MotorFoc_Step();
#else
    __WFI();
#endif
  }
}

static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  /* PF0/PF1 = OSC_IN/OSC_OUT on LQFP-32; GPIO clock required before HSE start. */
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /* HSE 8 MHz × 6 = 48 MHz. */
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL = RCC_PLL_MUL6;
  osc.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
  {
    Error_Handler();
  }

  clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  for (;;)
  {
  }
}
