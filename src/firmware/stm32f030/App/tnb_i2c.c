#include "tnb_i2c.h"
#include "tnb_node.h"
#include "tnb_motor.h"
#include "tnb_protocol.h"
#include "tnb_crc.h"
#include "main.h"
#include "board_pins.h"

#define TNB_RX_MAX 32U
#define TNB_TX_MAX 32U

static uint8_t s_rx[TNB_RX_MAX];
static uint8_t s_tx[TNB_TX_MAX];
static volatile uint16_t s_rx_len;
static volatile uint16_t s_tx_len;
static volatile uint16_t s_tx_i;
static volatile uint8_t s_receiving;
static tnb_match_t s_write_kind;

static tnb_match_t match_kind(uint8_t a7)
{
  if (a7 == TNB_ADDR_GENERAL_CALL)
  {
    return TNB_MATCH_GENERAL;
  }
  return TNB_MATCH_PRIMARY;
}

static void commit_write(void)
{
  if ((s_receiving != 0U) && (s_rx_len > 0U))
  {
    uint16_t n = s_rx_len;

    /* Optional PEC: if the last byte is CRC(addr_w || frame), strip it. */
    if (n >= 3U)
    {
      uint8_t addr_w = (uint8_t)(TnbNode_PrimaryAddr() << 1);
      uint8_t pec = TnbCrc_Byte(0U, addr_w);

      pec = TnbCrc_Buf(pec, s_rx, (uint32_t)(n - 1U));
      if (pec == s_rx[n - 1U])
      {
        n = (uint16_t)(n - 1U);
      }
    }
    TnbNode_OnWrite(s_write_kind, s_rx, n);
  }
  s_receiving = 0U;
  s_rx_len = 0U;
}

static void load_tx(tnb_match_t kind)
{
  uint16_t n = TnbNode_OnRead(kind, s_tx, (uint16_t)(TNB_TX_MAX - 1U));
  uint8_t addr;
  uint8_t pec;

  if (n == 0U)
  {
    s_tx[0] = 0xFFU;
  }
  /* NOSTRETCH: first data byte in TXDR before this handler returns.
   * Read PEC is always over one data byte — the slave cannot know a
   * longer master burst until NACK, which is too late for SMBus PEC. */
  I2C1->TXDR = s_tx[0];
  s_tx_i = 1U;
  addr = TnbNode_PrimaryAddr();
  pec = TnbCrc_Byte(0U, (uint8_t)(addr << 1));
  pec = TnbCrc_Byte(pec, TnbNode_RegPtr());
  pec = TnbCrc_Byte(pec, (uint8_t)((addr << 1) | 1U));
  pec = TnbCrc_Byte(pec, s_tx[0]);
  s_tx[1] = pec;
  s_tx_len = 2U;
}

void TnbNode_ApplyI2cAddress(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t addr = TnbNode_PrimaryAddr();

  RCC->CFGR3 |= RCC_CFGR3_I2C1SW;
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();
  RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

  I2C1->CR1 = 0U;
  while ((I2C1->CR1 & I2C_CR1_PE) != 0U)
  {
  }

  gpio.Pin = BOARD_I2C_SCL_PIN | BOARD_I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF1_I2C1;
  HAL_GPIO_Init(BOARD_I2C_SCL_PORT, &gpio);

  I2C1->TIMINGR = 0x10805E89U;
  I2C1->OAR1 = 0U;
  I2C1->OAR1 = ((uint32_t)addr << 1) | I2C_OAR1_OA1EN;
  I2C1->OAR2 = 0U;
  I2C1->CR1 = I2C_CR1_PE | I2C_CR1_ADDRIE | I2C_CR1_RXIE | I2C_CR1_TXIE |
              I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_ERRIE | I2C_CR1_GCEN |
              I2C_CR1_NOSTRETCH | (15U << I2C_CR1_DNF_Pos);

  HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_IRQn);
}

void TnbI2c_IRQHandler(void)
{
  uint32_t isr = I2C1->ISR;

  if ((isr & (I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR)) != 0U)
  {
    I2C1->ICR = I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
    commit_write();
  }

  if ((isr & I2C_ISR_ADDR) != 0U)
  {
    uint8_t a7 = (uint8_t)((isr & I2C_ISR_ADDCODE) >> I2C_ISR_ADDCODE_Pos);
    tnb_match_t kind = match_kind(a7);

    I2C1->ICR = I2C_ICR_ADDRCF;
    TnbNode_NoteI2c();
    if ((isr & I2C_ISR_DIR) == 0U)
    {
      s_write_kind = kind;
      s_rx_len = 0U;
      s_receiving = 1U;
    }
    else
    {
      commit_write();
      load_tx(kind);
      TnbMotor_PollIsr();
      TnbNode_RefreshMot();
    }
  }

  if ((I2C1->ISR & I2C_ISR_RXNE) != 0U)
  {
    uint8_t b = (uint8_t)I2C1->RXDR;
    if (s_rx_len < TNB_RX_MAX)
    {
      s_rx[s_rx_len] = b;
      s_rx_len++;
    }
  }

  if ((I2C1->ISR & I2C_ISR_TXIS) != 0U)
  {
    if (s_tx_i < s_tx_len)
    {
      I2C1->TXDR = s_tx[s_tx_i];
      s_tx_i++;
    }
    else
    {
      I2C1->TXDR = 0xFFU;
    }
  }

  if ((I2C1->ISR & I2C_ISR_NACKF) != 0U)
  {
    I2C1->ICR = I2C_ICR_NACKCF;
  }

  if ((I2C1->ISR & I2C_ISR_STOPF) != 0U)
  {
    I2C1->ICR = I2C_ICR_STOPCF;
    I2C1->ISR |= I2C_ISR_TXE;
    commit_write();
    s_tx_len = 0U;
    s_tx_i = 0U;
  }
}
