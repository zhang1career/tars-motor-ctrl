#include "uart_log.h"
#include "main.h"

/*
 * PA14 = USART1_TX only. No RX. App takes the pin after a short SWD window
 * so connect-under-reset can still flash.
 */

#define UART_LOG_SWD_HOLD_MS  300U
#define UART_LOG_PCLK         48000000U
#define UART_LOG_BAUD         115200U

static uint8_t s_ready;

void UartLog_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint32_t t0 = HAL_GetTick();

  while ((HAL_GetTick() - t0) < UART_LOG_SWD_HOLD_MS)
  {
  }

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_14;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF1_USART1;
  HAL_GPIO_Init(GPIOA, &gpio);

  USART1->BRR = (UART_LOG_PCLK + (UART_LOG_BAUD / 2U)) / UART_LOG_BAUD;
  USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
  s_ready = 1U;

  UartLog_Puts("motor-ctrl 日志 115200 8N1");
  UartLog_Nl();
}

static void tx(uint8_t c)
{
  if (s_ready == 0U)
  {
    return;
  }
  while ((USART1->ISR & USART_ISR_TXE) == 0U)
  {
  }
  USART1->TDR = c;
}

void UartLog_Puts(const char *s)
{
  if (s == 0)
  {
    return;
  }
  while (*s != '\0')
  {
    tx((uint8_t)*s);
    s++;
  }
}

void UartLog_Nl(void)
{
  tx((uint8_t)'\r');
  tx((uint8_t)'\n');
}

void UartLog_PutU(uint32_t v)
{
  char buf[10];
  uint8_t n = 0U;

  if (v == 0U)
  {
    tx((uint8_t)'0');
    return;
  }
  while (v > 0U)
  {
    buf[n++] = (char)('0' + (v % 10U));
    v /= 10U;
  }
  while (n > 0U)
  {
    n--;
    tx((uint8_t)buf[n]);
  }
}

void UartLog_PutI(int32_t v)
{
  if (v < 0)
  {
    tx((uint8_t)'-');
    UartLog_PutU((uint32_t)(-v));
  }
  else
  {
    UartLog_PutU((uint32_t)v);
  }
}

void UartLog_PutHex16(uint16_t v)
{
  static const char hex[] = "0123456789ABCDEF";
  uint8_t i;

  tx((uint8_t)'0');
  tx((uint8_t)'x');
  for (i = 0U; i < 4U; i++)
  {
    uint8_t sh = (uint8_t)(12U - (i * 4U));
    tx((uint8_t)hex[(v >> sh) & 0xFU]);
  }
}
