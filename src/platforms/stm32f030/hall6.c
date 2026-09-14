#include "hall6.h"
#include "motor_hall.h"
#include "motor_pwm.h"
#include "motor_tick.h"
#if defined(MOTOR_TRACE) && (MOTOR_TRACE != 0)
#include "motor_trace.h"
#endif

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
  /* 4 and 6 were swapped vs a +60° field on s_seq_cw (5,1,3,2,6,4):
   * old 4=210° / 6=270° made the 6→4 step −60°. FocTheta reads this
   * table, so the rows and the Park stairs have to stay together. */
  { MOTOR_PHASE_OFF, MOTOR_PHASE_LOW, MOTOR_PHASE_PWM },
  { MOTOR_PHASE_PWM, MOTOR_PHASE_LOW, MOTOR_PHASE_OFF },
  { MOTOR_PHASE_LOW, MOTOR_PHASE_OFF, MOTOR_PHASE_PWM },
  { MOTOR_PHASE_OFF, MOTOR_PHASE_OFF, MOTOR_PHASE_OFF },
};

static const uint8_t s_seq_cw[6] = { 5U, 1U, 3U, 2U, 6U, 4U };
static const uint8_t s_seq_ccw[6] = { 5U, 4U, 6U, 2U, 3U, 1U };

volatile uint16_t g_hall6_sector_dwell[8];

static motor_hall6_snapshot_t s_hall6_snap;
static volatile uint8_t s_hall6_enable;
static uint8_t s_initialized;
static uint8_t s_last_gpio_hall;
static uint8_t s_kick_active;
static uint8_t s_kick_seq_idx;
static uint32_t s_kick_div;
static uint32_t s_kick_steps;
static uint8_t s_kick_sync;
static uint8_t s_phase_offset;
/* Control ticks since the last hall edge. One hall sector is 60 electrical
 * degrees, so this is the raw material for the continuous angle in stage D --
 * and on its own it already gives per-sector speed. */
static uint16_t s_ticks_since_edge;
static uint8_t s_kick_duty_pct;
static uint8_t s_run_duty_pct;
static uint8_t s_moe_pending;

static void hall6_store(const motor_hall6_snapshot_t *src);
static void hall6_commutate(uint8_t table_hall);
static void hall6_moe_release(void);

static void hall6_disable_outputs(void)
{
  s_hall6_enable = 0U;
  s_kick_active = 0U;
  s_moe_pending = 0U;
  MotorTick_Stop();
  MotorPwm_Stop();
}

void MotorHall6_ReleasePwm(void)
{
  s_hall6_enable = 0U;
  s_kick_active = 0U;
  s_moe_pending = 0U;
  s_hall6_snap.enabled = 0U;
  s_hall6_snap.kick = 0U;
}

static void hall6_store(const motor_hall6_snapshot_t *src)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_hall6_snap = *src;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/*
 * Reversing via the hall sequence alone cannot work at phase offsets 0 and 3:
 * s_seq_ccw is s_seq_cw reversed about index 0, so mapping through it is
 * equivalent to offset -p, and -p == p (mod 6) exactly at those two offsets -
 * which are the only efficient ones. Reverse torque instead by swapping which
 * phase sources and which sinks, same as ol_lookup_step().
 */
static const hall6_step_t *hall6_lookup(uint8_t hall, uint8_t reverse)
{
  static hall6_step_t rev;
  const hall6_step_t *st;

  if (hall >= 8U)
  {
    return &s_table_cw[0];
  }

  st = &s_table_cw[hall];
  if (reverse == 0U)
  {
    return st;
  }

  rev = *st;
  if (rev.u == MOTOR_PHASE_PWM) { rev.u = MOTOR_PHASE_LOW; }
  else if (rev.u == MOTOR_PHASE_LOW) { rev.u = MOTOR_PHASE_PWM; }
  if (rev.v == MOTOR_PHASE_PWM) { rev.v = MOTOR_PHASE_LOW; }
  else if (rev.v == MOTOR_PHASE_LOW) { rev.v = MOTOR_PHASE_PWM; }
  if (rev.w == MOTOR_PHASE_PWM) { rev.w = MOTOR_PHASE_LOW; }
  else if (rev.w == MOTOR_PHASE_LOW) { rev.w = MOTOR_PHASE_PWM; }
  return &rev;
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

  seq = hall6_seq(s_hall6_snap.direction);
  return seq[(hall6_seq_index(seq, raw) + s_phase_offset) % 6U];
}

static void hall6_kick_begin(uint8_t hall)
{
  const uint8_t *seq = hall6_seq(s_hall6_snap.direction);

  s_kick_active = 1U;
  s_kick_seq_idx = hall6_seq_index(seq, hall);
  s_kick_div = 0U;
  s_kick_steps = 0U;
  s_kick_sync = 0U;
  s_last_gpio_hall = hall;
}

static void hall6_kick_step(void)
{
  const uint8_t *seq = hall6_seq(s_hall6_snap.direction);
  motor_hall6_snapshot_t snap;

  s_kick_seq_idx = (uint8_t)((s_kick_seq_idx + 1U) % 6U);
  s_kick_steps++;
  hall6_commutate(hall6_map_hall(seq[s_kick_seq_idx]));

  snap = s_hall6_snap;
  snap.kick = 1U;
  hall6_store(&snap);

  if ((s_kick_sync >= MOTOR_HALL6_KICK_SYNC_MIN) ||
      (s_kick_steps >= MOTOR_HALL6_KICK_MAX_STEPS))
  {
    s_kick_active = 0U;
    snap.kick = 0U;
    s_hall6_snap.duty_pct = s_run_duty_pct;
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
    snap = s_hall6_snap;
    snap.fault = 1U;
    hall6_store(&snap);
    return;
  }

  st = hall6_lookup(table_hall, s_hall6_snap.direction);
  {
    TIM_TypeDef *tim = htim1.Instance;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t pulse = (arr * (uint32_t)s_hall6_snap.duty_pct) / 100U;

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

  seq = hall6_seq(s_hall6_snap.direction);
  snap = s_hall6_snap;
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

  s_hall6_snap.duty_pct = MOTOR_HALL6_DEFAULT_DUTY;
  s_kick_duty_pct = MOTOR_HALL6_DEFAULT_DUTY;
  s_run_duty_pct = MOTOR_HALL6_DEFAULT_RUN;
  s_phase_offset = 3U;
  s_hall6_snap.phase = 3U;
  s_hall6_snap.direction = 0U;
  s_hall6_enable = 0U;
  s_initialized = 1U;
}

int MotorHall6_IsEnabled(void)
{
  return (s_hall6_enable != 0U) ? 1 : 0;
}

uint16_t MotorHall6_FocTheta(void)
{
  static uint16_t last;
  const hall6_step_t *st;
  uint8_t mapped;
  int32_t v_deg;

  mapped = hall6_map_hall(MotorHall_ReadRaw());
  st = hall6_lookup(mapped, s_hall6_snap.direction);

  /* Voltage angle of PWM=+ / LOW=− (U at 0°, V at 120°, W at 240°).
   * FOC +vq sits at theta+90°, so return V−90°. */
  if ((st->u == MOTOR_PHASE_PWM) && (st->w == MOTOR_PHASE_LOW))
  {
    v_deg = 30;
  }
  else if ((st->v == MOTOR_PHASE_PWM) && (st->u == MOTOR_PHASE_LOW))
  {
    v_deg = 150;
  }
  else if ((st->v == MOTOR_PHASE_PWM) && (st->w == MOTOR_PHASE_LOW))
  {
    v_deg = 90;
  }
  else if ((st->w == MOTOR_PHASE_PWM) && (st->u == MOTOR_PHASE_LOW))
  {
    v_deg = 210;
  }
  else if ((st->u == MOTOR_PHASE_PWM) && (st->v == MOTOR_PHASE_LOW))
  {
    v_deg = 330;
  }
  else if ((st->w == MOTOR_PHASE_PWM) && (st->v == MOTOR_PHASE_LOW))
  {
    v_deg = 270;
  }
  else
  {
    return last;
  }

  last = (uint16_t)(((v_deg - 90) * 65536) / 360);
  return last;
}

static uint8_t s_ft_stair;
static uint8_t s_ft_have_stair;
static uint16_t s_ft_anchor;
static uint16_t s_ft_ticks;
static uint16_t s_ft_last_ticks[6];
static int32_t s_ft_omega_q8; /* dest-stair pred; interpolator only */
static int32_t s_ft_omega_spd_q8; /* 6-sector Σwidth/Σticks; speed PI */
static int16_t s_ft_spd_step[6];
static uint16_t s_ft_spd_ticks[6];
/* Physical sector widths, Q16. From foc_ang dwell fractions across
 * the 2026-09-14 speed-hold traces (tick 4/5, both dirs). Stairs are
 * FocTheta/10923, not raw hall. Sum is 65536. Equal 60° was 10923
 * and made per-sector ω swing with the 51–68° halls; the mean was
 * still high because integer tick counts floor. */
static const uint16_t s_ft_width_q16[6] = {
    12302U, 9420U, 11316U, 11587U, 9470U, 11441U
};
static uint8_t s_ft_spd_n;
static uint8_t s_ft_spd_i;
static int16_t s_ft_step;
static uint8_t s_ft_have_omega;

/* FocTheta is 60° stairs. 65536/6 = 10922.67; 0..10922 → 0, wrap → 0. */
static uint8_t foc_theta_stair(uint16_t now)
{
  uint8_t s = (uint8_t)((uint32_t)now / 10923U);

  return (s > 5U) ? 0U : s;
}

uint16_t MotorHall6_FocThetaInterp(uint16_t now)
{
  uint8_t stair = foc_theta_stair(now);
  int32_t travelled;
  int32_t lim;

  /* Edge = FocTheta stair change only. A second MotorHall_ReadRaw() in
   * the same tick can disagree with the read inside FocTheta when the
   * GPIO edge falls between them: step and pred then blow up and dth
   * sits on ±60° for the whole next hall. Key dest-stair dwell by
   * `now`, not a second GPIO sample. */
  if (now != s_ft_anchor)
  {
    if ((s_ft_have_stair != 0U) && (s_ft_ticks > 0U))
    {
      uint16_t pred;
      int32_t om;
      int32_t sum_step;
      uint32_t sum_ticks;
      uint8_t k;

      s_ft_step = (int16_t)(now - s_ft_anchor);
      s_ft_last_ticks[s_ft_stair] = s_ft_ticks;
      /* This stair's last dwell predicts this stair. Arithmetic mean of
       * 60°/dwell reads high when halls are unequal (2026-09-13 wrap
       * sat 2–5% under w_meas). Speed uses Σstep/Σticks over one turn. */
      pred = s_ft_last_ticks[stair];
      if (pred == 0U)
      {
        pred = s_ft_ticks;
      }
      om = ((int32_t)s_ft_step << 8) / (int32_t)pred;
      if (s_ft_have_omega != 0U)
      {
        s_ft_omega_q8 = (s_ft_omega_q8 * 3 + om) / 4;
      }
      else
      {
        s_ft_omega_q8 = om;
      }
      /* Sign follows the FocTheta stair; magnitude is the calibrated
       * physical width of the stair we just left. */
      if (s_ft_step >= 0)
      {
        s_ft_spd_step[s_ft_spd_i] = (int16_t)s_ft_width_q16[s_ft_stair];
      }
      else
      {
        s_ft_spd_step[s_ft_spd_i] = -(int16_t)s_ft_width_q16[s_ft_stair];
      }
      /* Inclusive of the starting-edge tick. The counter only advances
       * while now == anchor, so a raw store drops one tick/sector and
       * w_meas reads 1.5–3% high vs wrap (2026-09-14). Widths that
       * still sum to 65536 cannot cancel that. */
      s_ft_spd_ticks[s_ft_spd_i] = (uint16_t)(s_ft_ticks + 1U);
      s_ft_spd_i++;
      if (s_ft_spd_i >= 6U)
      {
        s_ft_spd_i = 0U;
      }
      if (s_ft_spd_n < 6U)
      {
        s_ft_spd_n++;
      }
      sum_step = 0;
      sum_ticks = 0U;
      for (k = 0U; k < s_ft_spd_n; k++)
      {
        sum_step += (int32_t)s_ft_spd_step[k];
        sum_ticks += (uint32_t)s_ft_spd_ticks[k];
      }
      if (sum_ticks > 0U)
      {
        /* dwell+1 already counts the starting-edge tick. n/2 was a
         * leftover floor correction and overshot +0.37 tick/sector
         * (wrap 0.9–1.4% above w_ref, 2026-09-14). */
        s_ft_omega_spd_q8 = (sum_step << 8) / (int32_t)sum_ticks;
      }
      s_ft_have_omega = 1U;
    }
    s_ft_anchor = now;
    s_ft_stair = stair;
    s_ft_have_stair = 1U;
    s_ft_ticks = 0U;
    if ((s_ft_have_omega == 0U) || (s_ft_step == 0))
    {
      return now;
    }
    /* Sector-centered: start half a step behind the new stair so the
     * sector-mean of (interp - stair) is 0. Continuous with the previous
     * sector's end (old+30 = new-30). Starting at the stair and walking
     * 3/4 of the way left dth mean -23 deg; Park then added +23 deg. */
    return (uint16_t)((int32_t)now - ((int32_t)s_ft_step / 2));
  }

  if (s_ft_ticks < 0xFFFFU)
  {
    s_ft_ticks++;
  }

  if ((s_ft_have_omega == 0U) || (s_ft_step == 0))
  {
    return now;
  }

  /* Full walk across this stair, clamped to the step. Centered by the
   * -step/2 term so Park can sit on FocTheta with park_off = 0. */
  travelled = (s_ft_omega_q8 * (int32_t)s_ft_ticks) >> 8;
  if (s_ft_step >= 0)
  {
    lim = (int32_t)s_ft_step;
    if (travelled > lim)
    {
      travelled = lim;
    }
    if (travelled < 0)
    {
      travelled = 0;
    }
  }
  else
  {
    lim = (int32_t)s_ft_step;
    if (travelled < lim)
    {
      travelled = lim;
    }
    if (travelled > 0)
    {
      travelled = 0;
    }
  }

  return (uint16_t)((int32_t)s_ft_anchor - ((int32_t)s_ft_step / 2) + travelled);
}

int32_t MotorHall6_FocOmegaQ8(void)
{
  if (s_ft_have_omega == 0U)
  {
    return 0;
  }
  /* One-turn Σstep/Σticks. Dest-stair pred stays on the interpolator. */
  return s_ft_omega_spd_q8;
}

uint8_t MotorHall6_FloatPhase(void)
{
  const hall6_step_t *st = hall6_lookup(hall6_map_hall(MotorHall_ReadRaw()),
                                        s_hall6_snap.direction);

  if (st->u == MOTOR_PHASE_OFF)
  {
    return 0U;
  }
  if (st->v == MOTOR_PHASE_OFF)
  {
    return 1U;
  }
  if (st->w == MOTOR_PHASE_OFF)
  {
    return 2U;
  }
  return 0xFFU;
}

void MotorHall6_ApplyFocDuty(uint8_t duty_pct)
{
  if (duty_pct > MOTOR_HALL6_MAX_RUN_DUTY)
  {
    duty_pct = MOTOR_HALL6_MAX_RUN_DUTY;
  }
  s_hall6_snap.duty_pct = duty_pct;
  hall6_commutate(hall6_map_hall(MotorHall_ReadRaw()));
}

void MotorHall6_ApplyFocVoltages(int32_t vu, int32_t vv, int32_t vw,
                                 int32_t vdc_mv)
{
  const hall6_step_t *st;
  int32_t vpwm = 0;
  int32_t vlow = 0;
  int32_t duty;

  st = hall6_lookup(hall6_map_hall(MotorHall_ReadRaw()), s_hall6_snap.direction);
  if (st->u == MOTOR_PHASE_PWM)
  {
    vpwm = vu;
  }
  else if (st->u == MOTOR_PHASE_LOW)
  {
    vlow = vu;
  }
  if (st->v == MOTOR_PHASE_PWM)
  {
    vpwm = vv;
  }
  else if (st->v == MOTOR_PHASE_LOW)
  {
    vlow = vv;
  }
  if (st->w == MOTOR_PHASE_PWM)
  {
    vpwm = vw;
  }
  else if (st->w == MOTOR_PHASE_LOW)
  {
    vlow = vw;
  }

  if (vdc_mv < 1000)
  {
    vdc_mv = 1000;
  }
  {
    int32_t vline = vpwm - vlow;

    if (vline < 0)
    {
      vline = -vline;
    }
    /* 57 ≈ 100/sqrt(3): two-phase line is sqrt(3)*vq when aligned. */
    duty = (vline * 57) / vdc_mv;
  }
  if (duty < 8)
  {
    duty = 8;
  }
  else if (duty > (int32_t)MOTOR_HALL6_MAX_RUN_DUTY)
  {
    duty = (int32_t)MOTOR_HALL6_MAX_RUN_DUTY;
  }
  MotorHall6_ApplyFocDuty((uint8_t)duty);
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
    s_hall6_snap.duty_pct = pct;
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
    s_hall6_snap.duty_pct = pct;
  }
}

void MotorHall6_SetPhaseOffset(uint8_t offset)
{
  s_phase_offset = (uint8_t)(offset % 6U);
  s_hall6_snap.phase = s_phase_offset;
}

void MotorHall6_SetDirection(int ccw)
{
  s_hall6_snap.direction = (ccw != 0) ? 1U : 0U;
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
  *out = s_hall6_snap;
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

    s_hall6_enable = 1U;
    s_last_gpio_hall = 0xFFU;
    snap = s_hall6_snap;
    snap.enabled = 1U;
    snap.fault = 0U;
    snap.loop_count = 0U;
    snap.hall_changes = 0U;
    snap.kick = 1U;
    s_hall6_snap.duty_pct = s_kick_duty_pct;
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
  snap = s_hall6_snap;
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

  if (s_hall6_enable == 0U)
  {
    return;
  }

  hall = MotorHall_ReadRaw();

  if (hall != s_last_gpio_hall)
  {
    motor_hall6_snapshot_t edge = s_hall6_snap;
    edge.hall_changes++;
    hall6_store(&edge);

    if ((s_last_gpio_hall >= 1U) && (s_last_gpio_hall <= 6U) &&
        (s_ticks_since_edge > 0U))
    {
      g_hall6_sector_dwell[s_last_gpio_hall] = s_ticks_since_edge;
    }

    if (s_kick_active != 0U)
    {
      s_kick_sync++;
    }
    s_last_gpio_hall = hall;
    hall_edge = 1U;
    s_ticks_since_edge = 0U;
  }
  else if (s_ticks_since_edge < 0x7FFFU)
  {
    s_ticks_since_edge++;
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

  snap = s_hall6_snap;
  snap.hall_raw = hall;
  snap.kick = s_kick_active;
  snap.phase = s_phase_offset;
  snap.loop_count++;
  hall6_store(&snap);

#if defined(MOTOR_TRACE) && (MOTOR_TRACE != 0)
  /* One producer per capture: motor_tick.c pushes the current channels when the
   * host arms a different source. */
  if (g_motor_trace.source == MOTOR_TRACE_SRC_HALL6)
  {
    MotorTrace_Push((int16_t)hall, (int16_t)snap.step,
                    (int16_t)s_ticks_since_edge, (int16_t)snap.kick);
  }
#endif
}
