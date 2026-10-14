#include "tnb_node.h"
#include "tnb_motor.h"
#include "tnb_protocol.h"
#include "motor_adc.h"
#include "motor_foc_fx.h"
#include "main.h"
#include "board_pins.h"
#include <string.h>

#ifndef TNB_BOARD_ID
#define TNB_BOARD_ID 16U
#endif
#ifndef TNB_FW_VER
#define TNB_FW_VER 0x0101U
#endif

#define STM32_UID_BASE 0x1FFFF7ACUL

typedef struct {
  uint8_t  addr;
  uint8_t  reg_ptr;
  uint8_t  cap_index;
  uint8_t  alert_ctrl;
  uint8_t  status;
  uint16_t fault_flags;
  uint32_t uptime_s;
  uint8_t  heartbeat;
  uint8_t  last_err;
  uint8_t  i2c_ok;
  uint32_t tick_ms;
  uint8_t  resp[8];
  uint8_t  resp_len;
} tnb_node_t;

#define VREFINT_CAL_ADDR  ((uint16_t *)0x1FFFF7BAU)
#define TS_CAL1_ADDR      ((uint16_t *)0x1FFFF7B8U)
#define TS_CAL2_ADDR      ((uint16_t *)0x1FFFF7C2U)
#define VREFINT_CAL_VREF  3300U

static tnb_node_t s_node;
static uint8_t s_view[TNB_REG_SPACE_SIZE];
static int16_t s_temp_c10;
static int16_t s_t_board_c10 = TNB_TEMP_NONE;
static uint16_t s_vdda_mv;
static uint16_t s_vboost_mv;
static uint8_t s_tmcu_ok;
static volatile uint32_t s_i2c_t;
static volatile uint8_t s_i2c_seen;

static const tnb_cap_desc_t s_cap_motor = {
  TNB_CAP_MOTOR, 0U, TNB_ACC_RW, 1U,
  TNB_REG_MOT_MODE, 0U, 2U, TNB_UNIT_NONE, 5U
};
static const tnb_cap_desc_t s_cap_alert = {
  TNB_CAP_ALERT, 0U, TNB_ACC_R, 1U,
  TNB_REG_ALERT_CTRL, 0U, 1U, TNB_UNIT_NONE, 5U
};

static void put16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)((v >> 8) & 0xFFU);
  p[2] = (uint8_t)((v >> 16) & 0xFFU);
  p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static uint16_t get16(const uint8_t *p)
{
  return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static void read_uid(uint8_t *dst)
{
  const volatile uint8_t *u = (const volatile uint8_t *)STM32_UID_BASE;
  uint8_t i;
  for (i = 0U; i < 12U; i++)
  {
    dst[i] = u[i];
  }
}

static void alert_set(uint8_t assert_low)
{
  HAL_GPIO_WritePin(BOARD_TP_CYCLE_PORT, BOARD_TP_CYCLE_PIN,
                    (assert_low != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void alert_apply(void)
{
  uint8_t assert_low = 0U;

  if ((s_node.alert_ctrl & TNB_ALERT_CTRL_EN) != 0U)
  {
    if ((s_node.fault_flags != 0U) ||
        ((s_node.alert_ctrl & TNB_ALERT_CTRL_FORCE) != 0U))
    {
      assert_low = 1U;
    }
  }
  alert_set(assert_low);
  if (assert_low != 0U)
  {
    s_node.status |= TNB_ST_ALERT;
  }
  else
  {
    s_node.status &= (uint8_t)~TNB_ST_ALERT;
  }
}

static void refresh_view(void)
{
  s_view[TNB_REG_PROTO_VER] = TNB_PROTO_VER;
  s_view[TNB_REG_STATUS] = s_node.status;
  put16(&s_view[TNB_REG_VENDOR_ID], TNB_VENDOR_TARS);
  put16(&s_view[TNB_REG_PRODUCT_ID], TNB_PRODUCT_MOTOR_CTRL);
  put16(&s_view[TNB_REG_FW_VER], TNB_FW_VER);
  s_view[TNB_REG_BOARD_ID] = (uint8_t)TNB_BOARD_ID;
  s_view[TNB_REG_ADDR] = s_node.addr;
  s_view[TNB_REG_CAP_COUNT] = 2U;
  s_view[TNB_REG_PROFILE] = TNB_PROFILE_FULL;
  read_uid(&s_view[TNB_REG_UID]);

  put16(&s_view[TNB_REG_FAULT_FLAGS], s_node.fault_flags);
  put32(&s_view[TNB_REG_UPTIME_S], s_node.uptime_s);
  put16(&s_view[TNB_REG_TEMP_C10], (uint16_t)s_temp_c10);
  put16(&s_view[TNB_REG_VDDA_MV], s_vdda_mv);
  s_view[TNB_REG_HEARTBEAT] = s_node.heartbeat;
  s_view[TNB_REG_LAST_ERR] = s_node.last_err;
  s_view[TNB_REG_I2C_OK] = s_node.i2c_ok;
  s_view[TNB_REG_MAX_WRITE_PAYLOAD] = TNB_MAX_WRITE_PAYLOAD_FULL;
  s_view[TNB_REG_MAX_READ_BURST] = TNB_MAX_READ_BURST_FULL;
  s_view[TNB_REG_XFER_FLAGS] = (uint8_t)(TNB_XFER_FLAGS_FULL | TNB_XFER_PEC);
  s_view[TNB_REG_ALERT_CTRL] = s_node.alert_ctrl;

  TnbNode_RefreshMot();

  memcpy(&s_view[TNB_REG_RESP], s_node.resp, s_node.resp_len);
}

static uint16_t cap_desc(uint8_t index, uint8_t *buf, uint16_t cap)
{
  const tnb_cap_desc_t *d;
  const char *name;
  uint8_t total;

  if (index == 0U)
  {
    d = &s_cap_motor;
    name = "motor";
  }
  else if (index == 1U)
  {
    d = &s_cap_alert;
    name = "alert";
  }
  else
  {
    return 0U;
  }
  total = (uint8_t)(sizeof(tnb_cap_desc_t) + d->name_len);
  if (cap < total)
  {
    return 0U;
  }
  memcpy(buf, d, sizeof(tnb_cap_desc_t));
  memcpy(&buf[sizeof(tnb_cap_desc_t)], name, d->name_len);
  return total;
}

static void write_reg(uint8_t reg, uint8_t val)
{
  switch (reg)
  {
    case TNB_REG_ALERT_CTRL:
      s_node.alert_ctrl = val;
      alert_apply();
      break;
    case TNB_REG_ALERT_CLEAR:
      if (val != 0U)
      {
        s_node.fault_flags = 0U;
        s_node.status &= (uint8_t)~(TNB_ST_FAULT | TNB_ST_OVERTEMP | TNB_ST_ALERT);
        TnbMotor_ClearFault();
      }
      break;
    case TNB_REG_CAP_INDEX:
      s_node.cap_index = val;
      break;
    case TNB_REG_MOT_MODE:
      TnbMotor_SetMode(val);
      break;
    case TNB_REG_MOT_CMD:
      if ((val & TNB_MOT_CMD_FAULT_CLR) != 0U)
      {
        TnbMotor_ClearFault();
        s_node.fault_flags = 0U;
        s_node.status &= (uint8_t)~(TNB_ST_FAULT | TNB_ST_OVERTEMP | TNB_ST_ALERT);
      }
      if ((val & TNB_MOT_CMD_RUN) != 0U)
      {
        TnbMotor_RequestRun();
      }
      else
      {
        TnbMotor_RequestStop();
      }
      break;
    case TNB_REG_MOT_DIR:
      TnbMotor_SetDir(val);
      break;
    default:
      s_node.last_err = TNB_ERR_UNSUPPORTED;
      break;
  }
}

static void write_payload(uint8_t reg, const uint8_t *data, uint16_t len)
{
  uint16_t i = 0U;

  if ((reg == TNB_REG_CMD) && (len >= 2U))
  {
    if (data[0] == TNB_CMD_PING)
    {
      s_node.resp[0] = TNB_OK;
      s_node.resp[1] = 4U;
      s_node.resp[2] = (uint8_t)'T';
      s_node.resp[3] = (uint8_t)'N';
      s_node.resp[4] = (uint8_t)'B';
      s_node.resp[5] = (uint8_t)'1';
      s_node.resp_len = 6U;
    }
    else
    {
      s_node.resp[0] = TNB_ERR_UNSUPPORTED;
      s_node.resp[1] = 0U;
      s_node.resp_len = 2U;
      s_node.last_err = TNB_ERR_UNSUPPORTED;
    }
    return;
  }

  while (i < len)
  {
    uint8_t r = (uint8_t)(reg + i);
    if ((r == TNB_REG_MOT_IQ_REF_MA) && ((uint16_t)(len - i) >= 2U))
    {
      TnbMotor_SetIqMa((int16_t)get16(&data[i]));
      i += 2U;
      continue;
    }
    if ((r == TNB_REG_MOT_W_REF_EPS) && ((uint16_t)(len - i) >= 2U))
    {
      TnbMotor_SetWRefEps((int16_t)get16(&data[i]));
      i += 2U;
      continue;
    }
    write_reg(r, data[i]);
    i++;
  }
}

static void refresh_health(void)
{
  uint16_t mf = TnbMotor_Fault();
  uint32_t now = HAL_GetTick();

  s_node.uptime_s = now / 1000U;
  s_node.heartbeat++;
  if ((mf & TNB_MOT_FLT_OTEMP) != 0U)
  {
    s_node.fault_flags |= TNB_FLT_OVERTEMP;
    s_node.status |= (uint8_t)(TNB_ST_OVERTEMP | TNB_ST_FAULT);
  }
  else
  {
    s_node.fault_flags &= (uint16_t)~TNB_FLT_OVERTEMP;
    s_node.status &= (uint8_t)~TNB_ST_OVERTEMP;
  }
  if ((mf & TNB_MOT_FLT_NFAULT) != 0U)
  {
    s_node.fault_flags |= TNB_FLT_PWM;
    s_node.status |= TNB_ST_FAULT;
  }
  else
  {
    s_node.fault_flags &= (uint16_t)~TNB_FLT_PWM;
  }
  if ((mf & TNB_MOT_FLT_I2C) != 0U)
  {
    s_node.fault_flags |= TNB_FLT_I2C;
    s_node.status |= TNB_ST_FAULT;
  }
  else
  {
    s_node.fault_flags &= (uint16_t)~TNB_FLT_I2C;
  }
  if (mf != 0U)
  {
    s_node.status |= TNB_ST_FAULT;
  }
  else
  {
    s_node.status &= (uint8_t)~TNB_ST_FAULT;
  }
  alert_apply();
  s_view[TNB_REG_STATUS] = s_node.status;
  put16(&s_view[TNB_REG_FAULT_FLAGS], s_node.fault_flags);
  put32(&s_view[TNB_REG_UPTIME_S], s_node.uptime_s);
  put16(&s_view[TNB_REG_TEMP_C10], (uint16_t)s_temp_c10);
  put16(&s_view[TNB_REG_VDDA_MV], s_vdda_mv);
  s_view[TNB_REG_HEARTBEAT] = s_node.heartbeat;
  s_view[TNB_REG_LAST_ERR] = s_node.last_err;
  s_view[TNB_REG_I2C_OK] = s_node.i2c_ok;
}

void TnbNode_RefreshMot(void)
{
  uint16_t theta = 0U;
  int16_t w = 0;
  int16_t iq = 0;
  int16_t id = 0;
  uint16_t vbus = 0U;
  uint8_t hall = 0U;
  int32_t acc;

  refresh_health();
  TnbMotor_Telemetry(&theta, &w, &iq, &id, &vbus, &hall);
  s_view[TNB_REG_MOT_MODE] = TnbMotor_Mode();
  s_view[TNB_REG_MOT_DIR] = TnbMotor_Dir();
  put16(&s_view[TNB_REG_MOT_IQ_REF_MA], (uint16_t)TnbMotor_IqRefMa());
  put16(&s_view[TNB_REG_MOT_W_REF_EPS], (uint16_t)TnbMotor_WRefEps());
  put16(&s_view[TNB_REG_MOT_THETA_Q16], theta);
  put16(&s_view[TNB_REG_MOT_W_MEAS_EPS], (uint16_t)w);
  put16(&s_view[TNB_REG_MOT_IQ_MA], (uint16_t)iq);
  put16(&s_view[TNB_REG_MOT_ID_MA], (uint16_t)id);
  put16(&s_view[TNB_REG_MOT_VBUS_MV], vbus);
  put16(&s_view[TNB_REG_MOT_FAULT], TnbMotor_Fault());
  s_view[TNB_REG_MOT_HALL] = hall;
  s_view[TNB_REG_MOT_STATE] = TnbMotor_State();
  acc = TnbMotor_ThetaAcc();
  put32(&s_view[TNB_REG_MOT_THETA_ACC], (uint32_t)acc);
  put16(&s_view[TNB_REG_MOT_IQ_MAX_MA], (uint16_t)TnbMotor_IqMaxMa());
  put16(&s_view[TNB_REG_MOT_VBOOST_MV], s_vboost_mv);
  put16(&s_view[TNB_REG_MOT_T_BOARD_C10], (uint16_t)s_t_board_c10);
  put16(&s_view[TNB_REG_MOT_T_CASE_C10], (uint16_t)TNB_TEMP_NONE);
  put16(&s_view[TNB_REG_MOT_T_MCU_C10], (uint16_t)s_temp_c10);
  put16(&s_view[TNB_REG_MOT_CLOCK_KHZ],
        (uint16_t)(SystemCoreClock / 1000U));
  {
    uint16_t sns = 0U;
    int32_t vd;
    int32_t vq;

    if ((BOARD_NFAULT_PORT->IDR & BOARD_NFAULT_PIN) != 0U)
    {
      sns |= TNB_MOT_SNS_NFAULT;
    }
    if ((BOARD_NOTEMP_PORT->IDR & BOARD_NOTEMP_PIN) != 0U)
    {
      sns |= TNB_MOT_SNS_NOTEMP;
    }
    if (SystemCoreClock >= 40000000U)
    {
      sns |= TNB_MOT_SNS_CLK_OK;
    }
    if (s_t_board_c10 != TNB_TEMP_NONE)
    {
      sns |= TNB_MOT_SNS_TBOARD;
    }
    if (s_tmcu_ok != 0U)
    {
      sns |= TNB_MOT_SNS_TMCU;
    }
    if (TnbMotor_Reversing() != 0U)
    {
      sns |= TNB_MOT_SNS_REV;
    }
    if (g_motor_foc_fx.sat != 0U)
    {
      sns |= TNB_MOT_SNS_SAT;
    }
    put16(&s_view[TNB_REG_MOT_SENSE], sns);
    vd = g_motor_foc_fx.vd_uv / 1000;
    vq = g_motor_foc_fx.vq_uv / 1000;
    if (vd > 32767)
    {
      vd = 32767;
    }
    if (vd < -32768)
    {
      vd = -32768;
    }
    if (vq > 32767)
    {
      vq = 32767;
    }
    if (vq < -32768)
    {
      vq = -32768;
    }
    put16(&s_view[TNB_REG_MOT_VD_MV], (uint16_t)(int16_t)vd);
    put16(&s_view[TNB_REG_MOT_VQ_MV], (uint16_t)(int16_t)vq);
  }
}

static void apply_rail_cal(uint16_t vref_now, uint16_t ts_raw)
{
  uint16_t vref_cal;
  int32_t ts_cal1;
  int32_t ts_cal2;
  int32_t raw;
  int32_t t_c10;

  vref_cal = *VREFINT_CAL_ADDR;
  if ((vref_now != 0U) && (vref_cal != 0U) && (vref_cal != 0xFFFFU))
  {
    s_vdda_mv = (uint16_t)(((uint32_t)VREFINT_CAL_VREF * vref_cal) / vref_now);
  }
  else if (s_vdda_mv == 0U)
  {
    s_vdda_mv = VREFINT_CAL_VREF;
  }

  ts_cal1 = (int32_t)(*TS_CAL1_ADDR);
  ts_cal2 = (int32_t)(*TS_CAL2_ADDR);
  raw = (int32_t)ts_raw;
  if (s_vdda_mv > 0U)
  {
    raw = (raw * (int32_t)s_vdda_mv) / (int32_t)VREFINT_CAL_VREF;
  }
  if ((ts_cal2 != ts_cal1) && (ts_cal2 > 0) && (ts_cal2 != 0xFFFF) &&
      (ts_cal1 > 0) && (ts_cal1 != 0xFFFF))
  {
    t_c10 = ((int32_t)800 * (raw - ts_cal1)) / (ts_cal2 - ts_cal1) + 300;
  }
  else if ((ts_cal1 > 0) && (ts_cal1 != 0xFFFF))
  {
    t_c10 = 300 + (((ts_cal1 - raw) * 10) / 34);
  }
  else
  {
    return;
  }
  if (t_c10 < -400)
  {
    t_c10 = -400;
  }
  if (t_c10 > 1500)
  {
    t_c10 = 1500;
  }
  s_temp_c10 = (int16_t)t_c10;
  s_tmcu_ok = 1U;
}

/* PA0 = RT1 10k NTC to +3V3, R21 10k to GND. B≈3435. Code rises as it heats. */
static int16_t ntc_board_c10(uint16_t code)
{
  static const uint16_t codes[] = {
      781U, 1130U, 1476U, 1818U, 2048U, 2260U, 2640U,
      2970U, 3250U, 3485U, 3680U, 3830U, 3940U
  };
  static const int16_t c10s[] = {
      -100, 0, 100, 200, 250, 300, 400, 500, 600, 700, 800, 900, 1000
  };
  const unsigned n = 13U;
  unsigned i;

  if ((code < 200U) || (code > 4000U))
  {
    return TNB_TEMP_NONE;
  }
  if (code <= codes[0])
  {
    return c10s[0];
  }
  if (code >= codes[n - 1U])
  {
    return c10s[n - 1U];
  }
  for (i = 1U; i < n; i++)
  {
    if (code <= codes[i])
    {
      int32_t dc = (int32_t)codes[i] - (int32_t)codes[i - 1U];
      int32_t dt = (int32_t)c10s[i] - (int32_t)c10s[i - 1U];
      return (int16_t)(c10s[i - 1U] +
                       (dt * (int32_t)(code - codes[i - 1U])) / dc);
    }
  }
  return TNB_TEMP_NONE;
}

static void apply_aux(uint16_t ntc_raw, uint16_t vboost_raw)
{
  uint32_t vdda = (s_vdda_mv != 0U) ? s_vdda_mv : VREFINT_CAL_VREF;
  uint32_t mv;

  s_t_board_c10 = ntc_board_c10(ntc_raw);
  mv = ((uint32_t)vboost_raw * vdda * (uint32_t)BOARD_VBOOST_DIV_NUM) /
       (4095UL * (uint32_t)BOARD_VBOOST_DIV_DEN);
  if (mv > 65535U)
  {
    mv = 65535U;
  }
  s_vboost_mv = (uint16_t)mv;
}

void TnbNode_NoteCrc(void)
{
  s_node.last_err = TNB_ERR_CRC;
}

void TnbNode_NoteI2c(void)
{
  s_i2c_t = HAL_GetTick();
  s_i2c_seen = 1U;
}

uint8_t TnbNode_I2cTimedOut(uint32_t timeout_ms)
{
  if (s_i2c_seen == 0U)
  {
    return 0U;
  }
  return ((HAL_GetTick() - s_i2c_t) > timeout_ms) ? 1U : 0U;
}

uint8_t TnbNode_RegPtr(void)
{
  return s_node.reg_ptr;
}

void TnbNode_SampleRails(void)
{
  ADC_HandleTypeDef hadc = {0};
  ADC_ChannelConfTypeDef ch = {0};
  GPIO_InitTypeDef gpio = {0};
  uint16_t vref_now = 0U;
  uint16_t ts_raw = 0U;
  uint16_t ntc_raw = 0U;
  uint16_t vboost_raw = 0U;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_5;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  __HAL_RCC_ADC1_CLK_ENABLE();
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc.Init.SamplingTimeCommon = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    return;
  }
  if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
  {
    (void)HAL_ADC_DeInit(&hadc);
    return;
  }

  ADC->CCR |= ADC_CCR_VREFEN | ADC_CCR_TSEN;
  HAL_Delay(1);

  ch.Channel = ADC_CHANNEL_VREFINT;
  ch.Rank = ADC_RANK_CHANNEL_NUMBER;
  (void)HAL_ADC_ConfigChannel(&hadc, &ch);
  if (HAL_ADC_Start(&hadc) == HAL_OK)
  {
    if (HAL_ADC_PollForConversion(&hadc, 10U) == HAL_OK)
    {
      vref_now = (uint16_t)HAL_ADC_GetValue(&hadc);
    }
    (void)HAL_ADC_Stop(&hadc);
  }
  ch.Rank = ADC_RANK_NONE;
  (void)HAL_ADC_ConfigChannel(&hadc, &ch);

  ch.Channel = ADC_CHANNEL_TEMPSENSOR;
  ch.Rank = ADC_RANK_CHANNEL_NUMBER;
  (void)HAL_ADC_ConfigChannel(&hadc, &ch);
  if (HAL_ADC_Start(&hadc) == HAL_OK)
  {
    if (HAL_ADC_PollForConversion(&hadc, 10U) == HAL_OK)
    {
      ts_raw = (uint16_t)HAL_ADC_GetValue(&hadc);
    }
    (void)HAL_ADC_Stop(&hadc);
  }

  ch.Rank = ADC_RANK_NONE;
  (void)HAL_ADC_ConfigChannel(&hadc, &ch);
  ch.Channel = BOARD_ADC_NTC_CH;
  ch.Rank = ADC_RANK_CHANNEL_NUMBER;
  (void)HAL_ADC_ConfigChannel(&hadc, &ch);
  if (HAL_ADC_Start(&hadc) == HAL_OK)
  {
    if (HAL_ADC_PollForConversion(&hadc, 10U) == HAL_OK)
    {
      ntc_raw = (uint16_t)HAL_ADC_GetValue(&hadc);
    }
    (void)HAL_ADC_Stop(&hadc);
  }
  ch.Rank = ADC_RANK_NONE;
  (void)HAL_ADC_ConfigChannel(&hadc, &ch);
  ch.Channel = BOARD_ADC_VBOOST_CH;
  ch.Rank = ADC_RANK_CHANNEL_NUMBER;
  (void)HAL_ADC_ConfigChannel(&hadc, &ch);
  if (HAL_ADC_Start(&hadc) == HAL_OK)
  {
    if (HAL_ADC_PollForConversion(&hadc, 10U) == HAL_OK)
    {
      vboost_raw = (uint16_t)HAL_ADC_GetValue(&hadc);
    }
    (void)HAL_ADC_Stop(&hadc);
  }

  ADC->CCR &= ~(ADC_CCR_VREFEN | ADC_CCR_TSEN);
  (void)HAL_ADC_DeInit(&hadc);
  apply_rail_cal(vref_now, ts_raw);
  apply_aux(ntc_raw, vboost_raw);
}

void TnbNode_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  memset(&s_node, 0, sizeof(s_node));
  memset(s_view, 0, sizeof(s_view));
  s_node.addr = TNB_ADDR_FULL_FROM_BOARD_ID(TNB_BOARD_ID);
  s_node.alert_ctrl = TNB_ALERT_CTRL_EN;
  s_node.status = TNB_ST_READY;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin = BOARD_TP_CYCLE_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_TP_CYCLE_PORT, &gpio);
  HAL_GPIO_WritePin(BOARD_TP_CYCLE_PORT, BOARD_TP_CYCLE_PIN, GPIO_PIN_SET);

  TnbMotor_Init();
  refresh_view();
  TnbNode_ApplyI2cAddress();
}

uint8_t TnbNode_PrimaryAddr(void)
{
  return s_node.addr;
}

void TnbNode_Task(void)
{
  static uint32_t rails_t;
  uint16_t vref = 0U;
  uint16_t ts = 0U;

  TnbMotor_Task();
  if ((TnbMotor_Reversing() == 0U) &&
      ((HAL_GetTick() - rails_t) >= 500U))
  {
    rails_t = HAL_GetTick();
    if (MotorAdc_PollRails(&vref, &ts) != 0)
    {
      apply_rail_cal(vref, ts);
      apply_aux(MotorAdc_NtcRaw(), MotorAdc_VboostRaw());
    }
  }
  refresh_view();
}

void TnbNode_OnWrite(tnb_match_t kind, const uint8_t *data, uint16_t len)
{
  if ((data == 0) || (len == 0U))
  {
    return;
  }
  if (kind == TNB_MATCH_GENERAL)
  {
    if (data[0] == TNB_GC_RESET)
    {
      NVIC_SystemReset();
    }
    return;
  }
  s_node.reg_ptr = data[0];
  if (len > 1U)
  {
    write_payload(data[0], &data[1], (uint16_t)(len - 1U));
  }
  s_node.i2c_ok++;
  TnbNode_NoteI2c();
}

uint16_t TnbNode_OnRead(tnb_match_t kind, uint8_t *buf, uint16_t cap)
{
  if ((buf == 0) || (cap == 0U) || (kind == TNB_MATCH_GENERAL))
  {
    return 0U;
  }
  if (s_node.reg_ptr == TNB_REG_CAP_DESC)
  {
    uint16_t n = cap_desc(s_node.cap_index, buf, cap);

    TnbNode_NoteI2c();
    return n;
  }
  {
    uint16_t start = s_node.reg_ptr;
    uint16_t n = 0U;
    while ((n < cap) && ((start + n) < TNB_REG_SPACE_SIZE))
    {
      buf[n] = s_view[start + n];
      n++;
    }
    s_node.i2c_ok++;
    TnbNode_NoteI2c();
    return n;
  }
}
