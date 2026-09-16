#include "tnb_motor.h"
#include "tnb_node.h"
#include "tnb_protocol.h"
#include "uart_log.h"
#include "motor_app.h"
#include "motor_adc.h"
#include "motor_angle.h"
#include "motor_foc_fx.h"
#include "motor_hall.h"
#include "motor_pwm.h"
#include "hall6.h"
#include "board_pins.h"
#include "main.h"

#define MA_PER_LSB_NUM   1617
#define MA_PER_LSB_DEN   1000
#define VQMAX_LINEAR_UV  6580000
#define START_TIMEOUT_MS 8000U
#define HALL6_DUTY_PCT   25U
#define HALL6_READY_EDGES 6U
#define HALL6_STALL_TICKS 160000U
#define REV_W_EPS         12
#define REV_SLEW_MS       5U
#define REV_FLIP_MS       40U
#define REV_STALL_MS      800U
#define REV_IQ_LIM_MA     800
#define REV_DECEL         1U
#define REV_FLIP          2U
#define REV_ACCEL         3U
#define REV_HALL6         4U
#define VBUS_LIVE_MV      8000U
#ifndef TNB_I2C_TIMEOUT_MS
#define TNB_I2C_TIMEOUT_MS 3000U
#endif

static volatile uint8_t  s_mode = TNB_MOT_MODE_SPEED;
static volatile uint8_t  s_dir;
static volatile uint8_t  s_dir_app;
static volatile uint8_t  s_rev;
static volatile uint32_t s_rev_t0;
static volatile uint32_t s_rev_slew_t;
static volatile int16_t  s_rev_w;
static volatile int16_t  s_iq_ma = 300;
static volatile int16_t  s_w_eps = 80;
static volatile uint8_t  s_state = TNB_MOT_ST_IDLE;
static volatile uint16_t s_fault;
static volatile uint8_t  s_run_req;
static volatile uint8_t  s_stop_req;
static volatile uint32_t s_t0;

static int32_t ma_to_lsb(int16_t ma)
{
  int32_t lsb = ((int32_t)ma * MA_PER_LSB_DEN) / MA_PER_LSB_NUM;
  if (lsb > (int32_t)MOTOR_FOC_IQ_LSB)
  {
    lsb = (int32_t)MOTOR_FOC_IQ_LSB;
  }
  else if (lsb < -(int32_t)MOTOR_FOC_IQ_LSB)
  {
    lsb = -(int32_t)MOTOR_FOC_IQ_LSB;
  }
  return lsb;
}

static int16_t lsb_to_ma(int32_t lsb)
{
  int32_t ma = (lsb * MA_PER_LSB_NUM) / MA_PER_LSB_DEN;
  if (ma > 32767)
  {
    ma = 32767;
  }
  else if (ma < -32768)
  {
    ma = -32768;
  }
  return (int16_t)ma;
}

static int32_t w_abs_eps(void)
{
  int32_t om = MotorHall6_FocOmegaQ8();
  int32_t w;

  if (om < 0)
  {
    om = -om;
  }
  w = (om * 625) >> 19;
  return w;
}

static void apply_dir_now(void)
{
  s_dir_app = s_dir;
  MotorHall6_SetDirection((int)s_dir_app);
}

static uint16_t vbus_mv_now(void)
{
  int32_t vmv = ((int32_t)g_motor_adc_raw[MOTOR_ADC_VBUS] * 4058) >> 10;

  if (vmv < 0)
  {
    vmv = 0;
  }
  if (vmv > 65535)
  {
    vmv = 65535;
  }
  return (uint16_t)vmv;
}

static void sample_rail_faults(void)
{
  if (vbus_mv_now() < VBUS_LIVE_MV)
  {
    s_fault &= (uint16_t)~(TNB_MOT_FLT_OTEMP | TNB_MOT_FLT_NFAULT);
    return;
  }
  if ((BOARD_NOTEMP_PORT->IDR & BOARD_NOTEMP_PIN) == 0U)
  {
    s_fault |= TNB_MOT_FLT_OTEMP;
  }
  else
  {
    s_fault &= (uint16_t)~TNB_MOT_FLT_OTEMP;
  }
  if ((BOARD_NFAULT_PORT->IDR & BOARD_NFAULT_PIN) == 0U)
  {
    s_fault |= TNB_MOT_FLT_NFAULT;
  }
  else
  {
    s_fault &= (uint16_t)~TNB_MOT_FLT_NFAULT;
  }
}

static void apply_refs(void)
{
  int32_t lsb = ma_to_lsb(s_iq_ma);
  int32_t w = s_w_eps;
  int32_t slew;
  int32_t lim;

  if (w < 0)
  {
    w = -w;
  }
  if (w > 260)
  {
    w = 260;
  }
  if ((s_rev == REV_DECEL) || (s_rev == REV_ACCEL))
  {
    w = s_rev_w;
    if (w < 0)
    {
      w = -w;
    }
    lim = ma_to_lsb(REV_IQ_LIM_MA);
    if (lsb > lim)
    {
      lsb = lim;
    }
  }
  else if ((s_rev == REV_FLIP) || (s_rev == REV_HALL6))
  {
    lsb = 0;
    w = 0;
  }
  g_motor_foc_iq_ref = lsb;
  g_motor_foc_iq_lim = lsb;
  g_motor_foc_w_ref_eps = w;
  g_motor_foc_vq_max_uv = VQMAX_LINEAR_UV;
  slew = g_motor_foc_park_slew_q16;
  if (w > 180)
  {
    slew = w * 4;
    if (slew < 600)
    {
      slew = 600;
    }
    if (slew > 4000)
    {
      slew = 4000;
    }
    g_motor_foc_park_slew_q16 = slew;
  }
  if ((s_rev != REV_FLIP) && (s_rev != REV_HALL6) &&
      (s_mode == TNB_MOT_MODE_SPEED))
  {
    g_motor_foc_spd_on = 1U;
  }
  else
  {
    g_motor_foc_spd_on = 0U;
  }
}

static uint8_t hall6_spinning(const motor_hall6_snapshot_t *snap)
{
  return ((snap->kick == 0U) && (snap->hall_changes >= HALL6_READY_EDGES)) ? 1U : 0U;
}

static void start_hall6(uint8_t reset_acc)
{
  MotorApp_ApplyDefaults();
  MotorHall6_SetKickDutyPct(HALL6_DUTY_PCT);
  MotorHall6_SetDutyPct(HALL6_DUTY_PCT);
  s_rev = 0U;
  apply_dir_now();
  MotorFocFx_SetMode(MOTOR_FOC_FX_OBSERVE);
  if (MotorHall6_Enable(1) == 0)
  {
    s_fault |= TNB_MOT_FLT_STALL;
    s_state = TNB_MOT_ST_FAULT;
    return;
  }
  s_state = TNB_MOT_ST_HALL6;
  s_t0 = HAL_GetTick();
  if (reset_acc != 0U)
  {
    MotorAngle_ResetAcc();
  }
  MotorPwm_MoeEnable();
  apply_refs();
}

static void do_stop(void)
{
  s_rev = 0U;
  g_motor_foc_w_meas_eps = 0;
  MotorApp_Stop();
  s_state = TNB_MOT_ST_IDLE;
  s_run_req = 0U;
  UartLog_Puts("停机");
  UartLog_Nl();
}

static void rev_slew_step(int16_t dest)
{
  uint32_t now = HAL_GetTick();

  if ((now - s_rev_slew_t) < REV_SLEW_MS)
  {
    return;
  }
  s_rev_slew_t = now;
  if (s_rev_w > dest)
  {
    s_rev_w--;
  }
  else if (s_rev_w < dest)
  {
    s_rev_w++;
  }
}

static void enter_run(void)
{
  g_motor_foc_vq_max_uv = VQMAX_LINEAR_UV;
  apply_refs();
  if (s_mode == TNB_MOT_MODE_TORQUE)
  {
    s_state = TNB_MOT_ST_TORQUE;
    UartLog_Puts("力矩 ");
    UartLog_PutI(s_iq_ma);
    UartLog_Puts(" mA");
    UartLog_Nl();
  }
  else
  {
    s_state = TNB_MOT_ST_SPEED;
    UartLog_Puts("速度 ");
    UartLog_PutI(s_w_eps);
    UartLog_Puts(" 电周期/秒");
    UartLog_Nl();
  }
}

void TnbMotor_Init(void)
{
  s_mode = TNB_MOT_MODE_SPEED;
  s_dir = 0U;
  s_dir_app = 0U;
  s_rev = 0U;
  s_iq_ma = 1200;
  s_w_eps = 40;
  s_state = TNB_MOT_ST_IDLE;
  s_fault = 0U;
}

void TnbMotor_RequestRun(void)
{
  s_run_req = 0U;
  s_stop_req = 0U;
  if (SystemCoreClock < 40000000U)
  {
    s_fault |= TNB_MOT_FLT_STALL;
    s_state = TNB_MOT_ST_FAULT;
    UartLog_Puts("时钟未到 48M，不转");
    UartLog_Nl();
    return;
  }
  if ((s_state == TNB_MOT_ST_IDLE) || (s_state == TNB_MOT_ST_FAULT))
  {
    s_fault = 0U;
    start_hall6(1U);
  }
}

void TnbMotor_PollIsr(void)
{
  motor_hall6_snapshot_t snap;

  sample_rail_faults();
  if (MotorPwm_IsLatched() != 0U)
  {
    s_fault |= TNB_MOT_FLT_PWM;
    if ((BOARD_NFAULT_PORT->IDR & BOARD_NFAULT_PIN) != 0U)
    {
      MotorPwm_MoeEnable();
    }
  }
  else
  {
    s_fault &= (uint16_t)~TNB_MOT_FLT_PWM;
  }
  if (((s_fault & TNB_MOT_FLT_OTEMP) != 0U) &&
      (s_state != TNB_MOT_ST_IDLE) &&
      (s_state != TNB_MOT_ST_FAULT))
  {
    s_rev = 0U;
    MotorApp_Stop();
    s_state = TNB_MOT_ST_FAULT;
    return;
  }

  if (s_rev == REV_DECEL)
  {
    rev_slew_step(0);
    apply_refs();
    g_motor_foc_w_meas_eps = w_abs_eps();
    if ((g_motor_foc_w_meas_eps < REV_W_EPS) && (s_rev_w <= REV_W_EPS))
    {
      apply_dir_now();
      s_rev = REV_FLIP;
      s_rev_t0 = HAL_GetTick();
      g_motor_foc_w_meas_eps = 0;
      apply_refs();
    }
  }
  else if (s_rev == REV_FLIP)
  {
    apply_refs();
    if ((HAL_GetTick() - s_rev_t0) >= REV_FLIP_MS)
    {
      s_rev = REV_ACCEL;
      s_rev_w = 8;
      s_rev_slew_t = HAL_GetTick();
      s_rev_t0 = s_rev_slew_t;
      apply_refs();
    }
  }
  else if (s_rev == REV_ACCEL)
  {
    int16_t dest = s_w_eps;

    if (dest < 0)
    {
      dest = (int16_t)-dest;
    }
    rev_slew_step(dest);
    apply_refs();
    if ((s_rev_w >= dest) && (w_abs_eps() > 10))
    {
      s_rev = 0U;
      apply_refs();
    }
    else if ((HAL_GetTick() - s_rev_t0) >= REV_STALL_MS)
    {
      s_rev = REV_HALL6;
    }
  }

  if ((s_state == TNB_MOT_ST_TORQUE) || (s_state == TNB_MOT_ST_SPEED))
  {
    apply_refs();
    return;
  }
  if (s_state != TNB_MOT_ST_HALL6)
  {
    return;
  }

  MotorHall6_GetSnapshot(&snap);
  if (hall6_spinning(&snap) != 0U)
  {
    if ((g_motor_angle.valid == 0U) &&
        (g_motor_angle.sector < MOTOR_ANGLE_SECTORS))
    {
      g_motor_angle.valid = 1U;
    }
    if (g_motor_angle.valid != 0U)
    {
      g_motor_foc_handover = 1U;
    }
  }

  if (g_motor_foc_fx.mode == MOTOR_FOC_FX_CURRENT)
  {
    apply_refs();
    if (s_mode == TNB_MOT_MODE_TORQUE)
    {
      s_state = TNB_MOT_ST_TORQUE;
    }
    else
    {
      s_state = TNB_MOT_ST_SPEED;
    }
    return;
  }

  if (snap.loop_count > HALL6_STALL_TICKS)
  {
    s_rev = 0U;
    MotorApp_Stop();
    s_fault |= TNB_MOT_FLT_STALL;
    s_state = TNB_MOT_ST_FAULT;
  }
}

void TnbMotor_RequestStop(void)
{
  s_stop_req = 1U;
  s_run_req = 0U;
  s_rev = 0U;
  g_motor_foc_handover = 2U;
  g_motor_foc_w_meas_eps = 0;
  MotorApp_Stop();
  s_state = TNB_MOT_ST_IDLE;
}

void TnbMotor_ClearFault(void)
{
  s_fault = 0U;
  if (s_state == TNB_MOT_ST_FAULT)
  {
    s_state = TNB_MOT_ST_IDLE;
  }
}

void TnbMotor_SetMode(uint8_t mode)
{
  if (mode > TNB_MOT_MODE_SPEED)
  {
    return;
  }
  s_mode = mode;
  if (mode == TNB_MOT_MODE_STOP)
  {
    TnbMotor_RequestStop();
    return;
  }
  if ((s_state == TNB_MOT_ST_TORQUE) || (s_state == TNB_MOT_ST_SPEED))
  {
    enter_run();
  }
}

void TnbMotor_SetDir(uint8_t dir)
{
  s_dir = (dir != 0U) ? 1U : 0U;
  if (s_dir == s_dir_app)
  {
    s_rev = 0U;
    if ((s_state == TNB_MOT_ST_TORQUE) || (s_state == TNB_MOT_ST_SPEED))
    {
      apply_refs();
    }
    return;
  }
  /* Idle / hall6: table flip is safe. FOC: slew w through zero, flip
   * Park at low speed, then ramp back. Instant 180° at 80 latches PWM. */
  if ((s_state == TNB_MOT_ST_IDLE) ||
      (s_state == TNB_MOT_ST_FAULT) ||
      (s_state == TNB_MOT_ST_HALL6))
  {
    s_rev = 0U;
    apply_dir_now();
    return;
  }
  s_rev = REV_DECEL;
  s_rev_w = (int16_t)w_abs_eps();
  if (s_rev_w < 8)
  {
    s_rev_w = s_w_eps;
    if (s_rev_w < 0)
    {
      s_rev_w = (int16_t)-s_rev_w;
    }
  }
  s_rev_t0 = HAL_GetTick();
  s_rev_slew_t = s_rev_t0;
  apply_refs();
}

void TnbMotor_SetIqMa(int16_t ma)
{
  s_iq_ma = ma;
  if ((s_state != TNB_MOT_ST_IDLE) && (s_state != TNB_MOT_ST_FAULT))
  {
    apply_refs();
  }
}

void TnbMotor_SetWRefEps(int16_t eps)
{
  s_w_eps = eps;
  if ((s_state != TNB_MOT_ST_IDLE) && (s_state != TNB_MOT_ST_FAULT))
  {
    apply_refs();
  }
}

uint8_t  TnbMotor_Mode(void)     { return s_mode; }
uint8_t  TnbMotor_Dir(void)      { return s_dir; }
int16_t  TnbMotor_IqRefMa(void)  { return s_iq_ma; }
int16_t  TnbMotor_WRefEps(void)  { return s_w_eps; }
uint8_t  TnbMotor_State(void)    { return s_state; }
uint8_t  TnbMotor_Reversing(void){ return (s_rev != 0U) ? 1U : 0U; }
uint16_t TnbMotor_Fault(void)    { return s_fault; }
int32_t  TnbMotor_ThetaAcc(void) { return MotorAngle_ThetaAcc(); }
int16_t  TnbMotor_IqMaxMa(void)  { return lsb_to_ma((int32_t)MOTOR_FOC_IQ_LSB); }

void TnbMotor_Telemetry(uint16_t *theta_q16, int16_t *w_eps,
                        int16_t *iq_ma, int16_t *id_ma,
                        uint16_t *vbus_mv, uint8_t *hall)
{
  int32_t vmv;

  if (theta_q16 != 0)
  {
    *theta_q16 = g_motor_angle.theta;
  }
  if (w_eps != 0)
  {
    if ((s_state == TNB_MOT_ST_IDLE) || (s_state == TNB_MOT_ST_FAULT))
    {
      *w_eps = (int16_t)w_abs_eps();
    }
    else
    {
      *w_eps = (int16_t)g_motor_foc_w_meas_eps;
    }
  }
  if (iq_ma != 0)
  {
    *iq_ma = lsb_to_ma(g_motor_foc_fx.iq_lsb);
  }
  if (id_ma != 0)
  {
    *id_ma = lsb_to_ma(g_motor_foc_fx.id_lsb);
  }
  if (vbus_mv != 0)
  {
    vmv = ((int32_t)g_motor_adc_raw[MOTOR_ADC_VBUS] * 4058) >> 10;
    if (vmv < 0)
    {
      vmv = 0;
    }
    if (vmv > 65535)
    {
      vmv = 65535;
    }
    *vbus_mv = (uint16_t)vmv;
  }
  if (hall != 0)
  {
    *hall = MotorHall_ReadRaw();
  }
}

void TnbMotor_Task(void)
{
  static uint8_t rev_seen;

  if (s_rev == REV_HALL6)
  {
    s_rev = 0U;
    rev_seen = 0U;
    s_fault = 0U;
    UartLog_Puts("换向重起");
    UartLog_Nl();
    MotorApp_Stop();
    start_hall6(0U);
  }
  else if (s_rev == REV_DECEL)
  {
    if (rev_seen != 1U)
    {
      rev_seen = 1U;
      UartLog_Puts("换向减速");
      UartLog_Nl();
    }
  }
  else if (s_rev == REV_FLIP)
  {
    if (rev_seen != 2U)
    {
      rev_seen = 2U;
      UartLog_Puts("换向");
      UartLog_Nl();
    }
  }
  else if (s_rev == REV_ACCEL)
  {
    if (rev_seen != 3U)
    {
      rev_seen = 3U;
      UartLog_Puts("换向加速");
      UartLog_Nl();
    }
  }
  else if (rev_seen != 0U)
  {
    rev_seen = 0U;
    UartLog_Puts("换向完成");
    UartLog_Nl();
  }

  sample_rail_faults();

  if (((s_state == TNB_MOT_ST_HALL6) ||
       (s_state == TNB_MOT_ST_TORQUE) ||
       (s_state == TNB_MOT_ST_SPEED)) &&
      (TnbNode_I2cTimedOut(TNB_I2C_TIMEOUT_MS) != 0U))
  {
    s_fault |= TNB_MOT_FLT_I2C;
    do_stop();
    s_state = TNB_MOT_ST_FAULT;
    UartLog_Puts("I2C 超时停机");
    UartLog_Nl();
    return;
  }

  if (s_stop_req != 0U)
  {
    s_stop_req = 0U;
    do_stop();
    return;
  }

  if (((s_fault & (uint16_t)~TNB_MOT_FLT_PWM) != 0U) &&
      (s_state != TNB_MOT_ST_IDLE) &&
      (s_state != TNB_MOT_ST_FAULT))
  {
    do_stop();
    s_state = TNB_MOT_ST_FAULT;
    UartLog_Puts("故障 ");
    UartLog_PutHex16(s_fault);
    UartLog_Nl();
    return;
  }

  if ((s_run_req != 0U) && (s_state == TNB_MOT_ST_IDLE))
  {
    s_run_req = 0U;
    start_hall6(1U);
    return;
  }

  if (s_state == TNB_MOT_ST_HALL6)
  {
    motor_hall6_snapshot_t snap;

    MotorHall6_GetSnapshot(&snap);
    if ((hall6_spinning(&snap) != 0U) && (g_motor_angle.valid != 0U))
    {
      g_motor_foc_iq_ref = ma_to_lsb(s_iq_ma);
      g_motor_foc_handover = 1U;
      if (g_motor_foc_fx.mode != MOTOR_FOC_FX_CURRENT)
      {
        UartLog_Puts("交接电流环");
        UartLog_Nl();
      }
    }
    if (g_motor_foc_fx.mode == MOTOR_FOC_FX_CURRENT)
    {
      enter_run();
      return;
    }
    if ((HAL_GetTick() - s_t0) > START_TIMEOUT_MS)
    {
      do_stop();
      s_fault |= TNB_MOT_FLT_STALL;
      s_state = TNB_MOT_ST_FAULT;
      UartLog_Puts("起转超时");
      UartLog_Nl();
    }
  }
}
