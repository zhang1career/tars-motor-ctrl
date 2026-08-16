#include "hall6.h"
#include "motor_hall.h"
#include "motor_pwm.h"
#include "motor_tick.h"

#define MOTOR_HALL6_DEFAULT_DUTY   6U
#define MOTOR_HALL6_DEFAULT_RUN    7U
#define MOTOR_HALL6_MAX_RUN_DUTY   25U
#define MOTOR_HALL6_KICK_INTERVAL  600U    /* 20 kHz -> 30 ms/step (TARS) */
#define MOTOR_HALL6_KICK_SYNC_MIN  2U
#define MOTOR_HALL6_KICK_MAX_STEPS 40U    /* ~1.2 s open-loop kick before hall run */
#define MOTOR_HALL6_MOE_MIN_STEPS  2U

typedef motor_phase_mode_t hall6_phase_mode_t;

typedef struct {
  hall6_phase_mode_t u;
  hall6_phase_mode_t v;
  hall6_phase_mode_t w;
} hall6_step_t;

static const hall6_step_t s_table_cw[8] = {
  { MOTOR_PHASE_OFF, MOTOR_PHASE_OFF, MOTOR_PHASE_OFF },
  { MOTOR_PHASE_PWM, MOTOR_PHASE_OFF, MOTOR_PHASE_LOW },
  { MOTOR_PHASE_LOW, MOTOR_PHASE_PWM, MOTOR_PHASE_OFF },
  { MOTOR_PHASE_OFF, MOTOR_PHASE_PWM, MOTOR_PHASE_LOW },
  { MOTOR_PHASE_LOW, MOTOR_PHASE_OFF, MOTOR_PHASE_PWM },
  { MOTOR_PHASE_PWM, MOTOR_PHASE_LOW, MOTOR_PHASE_OFF },
  { MOTOR_PHASE_OFF, MOTOR_PHASE_LOW, MOTOR_PHASE_PWM },
  { MOTOR_PHASE_OFF, MOTOR_PHASE_OFF, MOTOR_PHASE_OFF },
};

static const uint8_t s_seq_cw[6] = { 5U, 1U, 3U, 2U, 6U, 4U };
static const uint8_t s_seq_ccw[6] = { 5U, 4U, 6U, 2U, 3U, 1U };

static motor_hall6_snapshot_t s_snap;
static volatile uint8_t s_enable;
static uint8_t s_initialized;
static uint8_t s_last_gpio_hall;
static uint8_t s_kick_active;
static uint8_t s_kick_seq_idx;
static uint32_t s_kick_div;
static uint32_t s_kick_steps;
static uint8_t s_kick_sync;
static uint8_t s_phase_offset;
static uint8_t s_kick_duty_pct;
static uint8_t s_run_duty_pct;
static uint8_t s_moe_pending;

static void hall6_store(const motor_hall6_snapshot_t *src);
static void hall6_commutate(uint8_t table_hall);
static void hall6_moe_release(void);

static void hall6_disable_outputs(void)
{
  s_enable = 0U;
  s_kick_active = 0U;
  s_moe_pending = 0U;
  MotorTick_Stop();
  MotorPwm_Stop();
}

static void hall6_store(const motor_hall6_snapshot_t *src)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_snap = *src;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static const hall6_step_t *hall6_lookup(uint8_t hall)
{
  if (hall >= 8U)
  {
    return &s_table_cw[0];
  }

  return &s_table_cw[hall];
}

static const uint8_t *hall6_seq(uint8_t ccw)
{
  /* Match TARS bench wiring: host CCW uses seq_cw. */
  return (ccw != 0U) ? s_seq_cw : s_seq_ccw;
}

static uint8_t hall6_seq_index(const uint8_t *seq, uint8_t hall)
{
  uint8_t i;

  for (i = 0U; i < 6U; i++)
  {
    if (seq[i] == hall)
    {
      return i;
    }
  }
  return 0U;
}

static uint8_t hall6_map_hall(uint8_t raw)
{
  const uint8_t *seq;

  if ((raw == 0U) || (raw == 7U))
  {
    return raw;
  }

  seq = hall6_seq(s_snap.direction);
  return seq[(hall6_seq_index(seq, raw) + s_phase_offset) % 6U];
}

static void hall6_kick_begin(uint8_t hall)
{
  const uint8_t *seq = hall6_seq(s_snap.direction);

  s_kick_active = 1U;
  s_kick_seq_idx = hall6_seq_index(seq, hall);
  s_kick_div = 0U;
  s_kick_steps = 0U;
  s_kick_sync = 0U;
  s_last_gpio_hall = hall;
}

static void hall6_kick_step(void)
{
  const uint8_t *seq = hall6_seq(s_snap.direction);
  motor_hall6_snapshot_t snap;

  s_kick_seq_idx = (uint8_t)((s_kick_seq_idx + 1U) % 6U);
  s_kick_steps++;
  hall6_commutate(hall6_map_hall(seq[s_kick_seq_idx]));

  snap = s_snap;
  snap.kick = 1U;
  hall6_store(&snap);

  if ((s_kick_sync >= MOTOR_HALL6_KICK_SYNC_MIN) ||
      (s_kick_steps >= MOTOR_HALL6_KICK_MAX_STEPS))
  {
    s_kick_active = 0U;
    snap.kick = 0U;
    s_snap.duty_pct = s_run_duty_pct;
    snap.duty_pct = s_run_duty_pct;
    hall6_store(&snap);
    if (s_moe_pending != 0U)
    {
      hall6_moe_release();
    }
  }
}

static void hall6_moe_release(void)
{
  uint8_t hall;

  if (s_moe_pending == 0U)
  {
    return;
  }

  s_moe_pending = 0U;
  MotorPwm_MoeEnable();
  hall = MotorHall_ReadRaw();
  hall6_commutate(hall6_map_hall(hall));
}

static void hall6_commutate(uint8_t table_hall)
{
  const hall6_step_t *st;
  motor_hall6_snapshot_t snap;
  const uint8_t *seq;

  if ((table_hall == 0U) || (table_hall == 7U))
  {
    snap = s_snap;
    snap.fault = 1U;
    hall6_store(&snap);
    return;
  }

  st = hall6_lookup(table_hall);
  {
    TIM_TypeDef *tim = htim1.Instance;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t pulse = (arr * (uint32_t)s_snap.duty_pct) / 100U;

    if (pulse > arr)
    {
      pulse = arr;
    }

    /* Direct CCER/CCR writes (TARS path); avoid SetPhase EGR_UG in ISR. */
    switch (st->u)
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
    switch (st->v)
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
    switch (st->w)
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
  }

  seq = hall6_seq(s_snap.direction);
  snap = s_snap;
  snap.fault = 0U;
  snap.step = hall6_seq_index(seq, table_hall);
  hall6_store(&snap);
}

void MotorHall6_Init(void)
{
  if (s_initialized != 0U)
  {
    return;
  }

  s_snap.duty_pct = MOTOR_HALL6_DEFAULT_DUTY;
  s_kick_duty_pct = MOTOR_HALL6_DEFAULT_DUTY;
  s_run_duty_pct = MOTOR_HALL6_DEFAULT_RUN;
  s_phase_offset = 3U;
  s_snap.phase = 3U;
  s_snap.direction = 0U;
  s_enable = 0U;
  s_initialized = 1U;
}

int MotorHall6_IsEnabled(void)
{
  return (s_enable != 0U) ? 1 : 0;
}

void MotorHall6_SetDutyPct(uint8_t pct)
{
  if (pct > MOTOR_HALL6_MAX_RUN_DUTY)
  {
    pct = MOTOR_HALL6_MAX_RUN_DUTY;
  }
  s_run_duty_pct = pct;
  if (s_kick_active == 0U)
  {
    s_snap.duty_pct = pct;
  }
}

void MotorHall6_SetKickDutyPct(uint8_t pct)
{
  if (pct > MOTOR_HALL6_MAX_RUN_DUTY)
  {
    pct = MOTOR_HALL6_MAX_RUN_DUTY;
  }
  s_kick_duty_pct = pct;
  if (s_kick_active != 0U)
  {
    s_snap.duty_pct = pct;
  }
}

void MotorHall6_SetPhaseOffset(uint8_t offset)
{
  s_phase_offset = (uint8_t)(offset % 6U);
  s_snap.phase = s_phase_offset;
}

void MotorHall6_SetDirection(int ccw)
{
  s_snap.direction = (ccw != 0) ? 1U : 0U;
}

void MotorHall6_GetSnapshot(motor_hall6_snapshot_t *out)
{
  uint32_t primask;

  if (out == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *out = s_snap;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

int MotorHall6_Enable(int enable)
{
  motor_hall6_snapshot_t snap;

  if (enable != 0)
  {
    uint8_t hall;
    uint8_t mapped;

    s_moe_pending = 1U;
    MotorPwm_HardwareSafe();
    if (MotorPwm_ArmOutputs() == 0)
    {
      s_moe_pending = 0U;
      return 0;
    }

    s_enable = 1U;
    s_last_gpio_hall = 0xFFU;
    snap = s_snap;
    snap.enabled = 1U;
    snap.fault = 0U;
    snap.loop_count = 0U;
    snap.hall_changes = 0U;
    snap.kick = 1U;
    s_snap.duty_pct = s_kick_duty_pct;
    snap.duty_pct = s_kick_duty_pct;
    hall6_store(&snap);

    hall = MotorHall_ReadRaw();
    mapped = hall6_map_hall(hall);
    snap.hall_raw = hall;
    hall6_store(&snap);
    hall6_kick_begin(hall);
    hall6_commutate(mapped);
    hall6_moe_release();
    if (s_moe_pending != 0U)
    {
      /* Break active or MOE not latched; force once more after BIF clear. */
      MotorPwm_MoeEnable();
      s_moe_pending = 0U;
    }

    MotorTick_Start();
    return 1;
  }

  hall6_disable_outputs();
  snap = s_snap;
  snap.enabled = 0U;
  snap.kick = 0U;
  hall6_store(&snap);
  return 1;
}

void MotorHall6_ControlLoopISR(void)
{
  motor_hall6_snapshot_t snap;
  uint8_t hall;
  uint8_t hall_edge = 0U;

  if (s_enable == 0U)
  {
    return;
  }

  hall = MotorHall_ReadRaw();

  if (hall != s_last_gpio_hall)
  {
    motor_hall6_snapshot_t edge = s_snap;
    edge.hall_changes++;
    hall6_store(&edge);

    if (s_kick_active != 0U)
    {
      s_kick_sync++;
    }
    s_last_gpio_hall = hall;
    hall_edge = 1U;
  }

  if (s_kick_active != 0U)
  {
    s_kick_div++;
    if (s_kick_div >= MOTOR_HALL6_KICK_INTERVAL)
    {
      s_kick_div = 0U;
      hall6_kick_step();
    }

    if ((s_moe_pending != 0U) &&
        ((s_kick_steps >= MOTOR_HALL6_MOE_MIN_STEPS) || (s_kick_sync >= 1U)))
    {
      hall6_moe_release();
    }
  }
  else if (hall_edge != 0U)
  {
    if (s_moe_pending != 0U)
    {
      hall6_moe_release();
    }
    hall6_commutate(hall6_map_hall(hall));
  }

  snap = s_snap;
  snap.hall_raw = hall;
  snap.kick = s_kick_active;
  snap.phase = s_phase_offset;
  snap.loop_count++;
  hall6_store(&snap);
}
