#include "hall6.h"
#include "fx_udiv.h"
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
static uint16_t s_last_dwell;
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
  /* Index and phase offset are each 0..5, so the sum is 0..10.
   * One subtract matches % 6. The open-loop kick keeps its own % 6. */
  {
    uint8_t idx = (uint8_t)(hall6_seq_index(seq, raw) + s_phase_offset);

    if (idx >= 6U)
    {
      idx = (uint8_t)(idx - 6U);
    }
    return seq[idx];
  }
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
  /* Leave the all-low precharge in place until its window ends. */
  if (MotorPwm_BootstrapReady() == 0)
  {
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

static struct {
  uint16_t last;
  uint8_t have;
  uint8_t hall_held;
  uint8_t dir_held;
} s_ftheta;

/* Angle for each raw hall at phase offset 3. Built off the chip from
 * hall6_map_hall + hall6_lookup + the pattern tests below, for both
 * directions. Hall 0 and 7 are illegal and stay 0; the caller keeps
 * the previous angle. Any other phase offset still uses foc_theta_walk,
 * which is not on the tick while the offset stays 3. */
static const uint16_t s_theta_hall[2][8] = {
    { 0U, 21675U, 43422U, 32990U, 439U, 11243U, 54070U, 0U },
    { 0U, 54070U, 11243U, 439U, 32990U, 43422U, 21675U, 0U },
};

static uint16_t foc_theta_walk(uint8_t hall, uint8_t dir) __attribute__((noinline));
static uint16_t foc_theta_walk(uint8_t hall, uint8_t dir)
{
  const hall6_step_t *st;
  uint8_t mapped;
  uint8_t idx;
  /* Sector centers, pattern order (not stair order). Measured widths,
   * cumulative, then shifted so their mean matches the old voltage
   * angles. Each value still lands in its own foc_theta_stair bucket.
   * Pattern 0..5 → stairs 5, 1, 0, 2, 4, 3. */
  static const uint16_t theta_q16[6] = {
      54070U, 11243U, 439U, 21675U, 43422U, 32990U
  };

  s_ftheta.hall_held = hall;
  s_ftheta.dir_held = dir;
  s_ftheta.have = 1U;

  mapped = hall6_map_hall(hall);
  st = hall6_lookup(mapped, s_hall6_snap.direction);

  /* Voltage angle of PWM=+ / LOW=− (U at 0°, V at 120°, W at 240°).
   * FOC +vq sits at theta+90°, so return V−90°. */
  if ((st->u == MOTOR_PHASE_PWM) && (st->w == MOTOR_PHASE_LOW))
  {
    idx = 0U;
  }
  else if ((st->v == MOTOR_PHASE_PWM) && (st->u == MOTOR_PHASE_LOW))
  {
    idx = 1U;
  }
  else if ((st->v == MOTOR_PHASE_PWM) && (st->w == MOTOR_PHASE_LOW))
  {
    idx = 2U;
  }
  else if ((st->w == MOTOR_PHASE_PWM) && (st->u == MOTOR_PHASE_LOW))
  {
    idx = 3U;
  }
  else if ((st->u == MOTOR_PHASE_PWM) && (st->v == MOTOR_PHASE_LOW))
  {
    idx = 4U;
  }
  else if ((st->w == MOTOR_PHASE_PWM) && (st->v == MOTOR_PHASE_LOW))
  {
    idx = 5U;
  }
  else
  {
    return s_ftheta.last;
  }

  s_ftheta.last = theta_q16[idx];
  return s_ftheta.last;
}

uint16_t MotorHall6_FocTheta(void)
{
  uint8_t hall;
  uint8_t dir;

  /* The stair is constant between hall edges. Recomputing the commutation
   * table every 50 us was a few hundred cycles and did not change theta. */
  hall = MotorHall_ReadRaw();
  dir = s_hall6_snap.direction;
  if ((s_ftheta.have != 0U) && (hall == s_ftheta.hall_held) &&
      (dir == s_ftheta.dir_held))
  {
    return s_ftheta.last;
  }
  /* Same cost every hall change: one load. The search through the
   * commutation list used to run here, so this tick's length depended
   * on which hall had just arrived. */
  s_ftheta.hall_held = hall;
  s_ftheta.dir_held = dir;
  s_ftheta.have = 1U;
  if ((s_phase_offset == 3U) && (hall >= 1U) && (hall <= 6U))
  {
    s_ftheta.last = s_theta_hall[dir & 1U][hall];
    return s_ftheta.last;
  }
  return foc_theta_walk(hall, dir);
}

/* 40 ms at 20 kHz. A held rotor sits longer than this. A turning
 * sector at a few electrical revolutions per second does not. */
#define HALL6_STALL_TICKS 800U

/* One object: separate symbols were a literal-pool load per access.
 * The width table stays const. */
static struct {
  uint16_t anchor;
  uint16_t ticks;
  int32_t omega_q8; /* dest-stair pred; interpolator only */
  int16_t step;
  uint8_t have_omega;
  uint8_t stair;
  uint8_t have_stair;
  uint8_t spd_dirty; /* edge records a new sector; the reader divides */
  uint8_t spd_n;
  uint8_t spd_i;
  int32_t omega_spd_q8; /* 6-sector Σwidth/Σticks; speed PI */
  uint16_t last_ticks[6];
  int16_t spd_step[6];
  uint16_t spd_ticks[6];
  uint16_t stall_ticks; /* dwell past this is a stop, not a slow sector */
} s_ft;

/* Physical sector widths, Q16, indexed by foc_theta_stair. Sum is 65536.
 * From the 2026-10-15 200 mA torque run, 19 electrical revolutions, the
 * 3/4 dwell mean in last_ticks. Degrees: 65.61, 53.09, 61.51, 62.81,
 * 51.80, 65.18. The edge step uses these widths. The steady tick does
 * not load the table. */
static const uint16_t s_ft_width_q16[6] = {
    11944U, 9665U, 11198U, 11433U, 9430U, 11866U
};

/* Nearest 60° stair. Truncation (now/10923) put both 0° and 60° in
 * bucket 0 and never returned 5. Boundaries are half a stair:
 * 5461, 16384, 27307, 38229, 49152, 60075. The six FocTheta angles
 * land in 0..5 one each. */
static uint8_t foc_theta_stair(uint16_t now)
{
  uint32_t n = (uint32_t)now;

  if ((n >= 60075U) || (n < 5461U))
  {
    return 0U;
  }
  if (n < 16384U)
  {
    return 1U;
  }
  if (n < 27307U)
  {
    return 2U;
  }
  if (n < 38229U)
  {
    return 3U;
  }
  if (n < 49152U)
  {
    return 4U;
  }
  return 5U;
}

/* Edge work (division, sector history) is not in the steady prologue.
 * A shared function was pushing eight registers on every 50 us tick. */
static uint16_t fx_interp_edge(uint16_t now) __attribute__((noinline));
static uint16_t fx_interp_edge(uint16_t now)
{
  uint8_t stair;

  stair = foc_theta_stair(now);

  /* Edge = FocTheta stair change only. A second MotorHall_ReadRaw() in
   * the same tick can disagree with the read inside FocTheta when the
   * GPIO edge falls between them: step and pred then blow up and dth
   * sits on ±60° for the whole next hall. Key dest-stair dwell by
   * `now`, not a second GPIO sample. */
  if ((s_ft.have_stair != 0U) && (s_ft.ticks > 0U))
  {
    uint16_t pred;
    int32_t om;
    uint8_t stalled;

    /* Sign from the stair centers. Magnitude is this sector's width,
     * so the walk meets the next boundary instead of assuming 60°. */
    {
      int16_t raw_step = (int16_t)(now - s_ft.anchor);
      uint16_t width = s_ft_width_q16[stair];

      if (raw_step >= 0)
      {
        s_ft.step = (int16_t)width;
      }
      else
      {
        s_ft.step = (int16_t)(-(int32_t)width);
      }
    }
    /* A held shaft runs the dwell up to the counter limit. Folding
     * that sample in makes the next sector predict a stop, and the
     * old speed then walks the angle to the sector end and locks.
     * Forget that speed and hold the hall center. */
    stalled = (uint8_t)((s_ft.stall_ticks != 0U) &&
                        (s_ft.ticks > s_ft.stall_ticks));
    if (stalled != 0U)
    {
      uint8_t i;

      s_ft.have_omega = 0U;
      s_ft.omega_q8 = 0;
      s_ft.spd_n = 0U;
      s_ft.spd_i = 0U;
      s_ft.spd_dirty = 0U;
      s_ft.omega_spd_q8 = 0;
      for (i = 0U; i < 6U; i++)
      {
        s_ft.last_ticks[i] = 0U;
      }
    }
    else
    {
    /* Recent mean of this stair's dwell. Zero means not seen since
     * boot, so the first sample is stored raw. The predictor below
     * reads the same words: later visits use that mean, not the last
     * single dwell. Edge only, so the steady tick does not pay for it. */
    {
      uint16_t prev = s_ft.last_ticks[s_ft.stair];
      uint16_t raw = s_ft.ticks;

      if (prev == 0U)
      {
        s_ft.last_ticks[s_ft.stair] = raw;
      }
      else
      {
        s_ft.last_ticks[s_ft.stair] =
            (uint16_t)((((uint32_t)prev * 3U) + (uint32_t)raw) >> 2);
      }
    }
    /* This stair's recent mean predicts this stair. Arithmetic mean of
     * 60°/dwell reads high when halls are unequal (2026-09-13 wrap
     * sat 2–5% under w_meas). Speed uses Σstep/Σticks over one turn. */
    pred = s_ft.last_ticks[stair];
    if (pred == 0U)
    {
      pred = s_ft.ticks;
    }
    /* Reciprocal, no Newton. At most 0.4% fast. The walk is clamped
     * to this sector, so the far end arrives at most about 0.2° early. */
    om = Fx_QuotS((int32_t)s_ft.step << 8, (int32_t)pred);
    if (s_ft.have_omega != 0U)
    {
      s_ft.omega_q8 = (s_ft.omega_q8 * 3 + om) / 4;
    }
    else
    {
      s_ft.omega_q8 = om;
    }
    /* |omega| * 65535 must fit in int32. A compare on every steady tick
     * pushed that tick past 1200 cycles, so the bound is applied here.
     * Faster than this, the walk lags and the sector clamp still holds. */
    if (s_ft.omega_q8 > 32767)
    {
      s_ft.omega_q8 = 32767;
    }
    else if (s_ft.omega_q8 < -32767)
    {
      s_ft.omega_q8 = -32767;
    }
    /* Twice the predicted dwell, and never shorter than a real stop.
     * The steady tick only compares against this. */
    {
      uint32_t lim = (uint32_t)pred << 1;

      if (lim < HALL6_STALL_TICKS)
      {
        lim = HALL6_STALL_TICKS;
      }
      if (lim > 65535U)
      {
        lim = 65535U;
      }
      s_ft.stall_ticks = (uint16_t)lim;
    }
    /* Sign follows the FocTheta stair; magnitude is the calibrated
     * physical width of the stair we just left. */
    if (s_ft.step >= 0)
    {
      s_ft.spd_step[s_ft.spd_i] = (int16_t)s_ft_width_q16[s_ft.stair];
    }
    else
    {
      s_ft.spd_step[s_ft.spd_i] = -(int16_t)s_ft_width_q16[s_ft.stair];
    }
    /* Inclusive of the starting-edge tick. The counter only advances
     * while now == anchor, so a raw store drops one tick/sector and
     * w_meas reads 1.5–3% high vs wrap (2026-09-14). Widths that
     * still sum to 65536 cannot cancel that. */
    s_ft.spd_ticks[s_ft.spd_i] = (uint16_t)(s_ft.ticks + 1U);
    s_ft.spd_i++;
    if (s_ft.spd_i >= 6U)
    {
      s_ft.spd_i = 0U;
    }
    if (s_ft.spd_n < 6U)
    {
      s_ft.spd_n++;
    }
    /* The six-sector speed quotient is not an input to the current
     * PIs. Summing and dividing here made every hall edge a long
     * tick. The reader computes it. */
    s_ft.spd_dirty = 1U;
    s_ft.have_omega = 1U;
    }
  }
  s_ft.anchor = now;
  s_ft.stair = stair;
  s_ft.have_stair = 1U;
  s_ft.ticks = 0U;
  if ((s_ft.have_omega == 0U) || (s_ft.step == 0))
  {
    return now;
  }
  /* Start at this sector's boundary (center - width/2). The next
   * sector starts where this one ends, because the centers were built
   * from the measured widths. */
  return (uint16_t)((int32_t)now - ((int32_t)s_ft.step / 2));
}

uint16_t MotorHall6_FocThetaInterp(uint16_t now)
{
  int32_t travelled;
  int32_t lim;

  if (now != s_ft.anchor)
  {
    return fx_interp_edge(now);
  }

  if (s_ft.ticks < 0xFFFFU)
  {
    s_ft.ticks++;
  }

  if ((s_ft.have_omega == 0U) || (s_ft.step == 0))
  {
    return now;
  }

  /* Hold the hall center. Keeping the old speed walks the angle to
   * the sector end and the next release cannot cross a hall. */
  if ((s_ft.stall_ticks != 0U) && (s_ft.ticks > s_ft.stall_ticks))
  {
    s_ft.have_omega = 0U;
    s_ft.omega_q8 = 0;
    return now;
  }

  /* Full walk across this stair, clamped to the step. Centered by the
   * -step/2 term so Park can sit on FocTheta with park_off = 0.
   * omega is bounded at the edge so this product fits in int32. */
  travelled = (s_ft.omega_q8 * (int32_t)s_ft.ticks) >> 8;
  if (s_ft.step >= 0)
  {
    lim = (int32_t)s_ft.step;
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
    lim = (int32_t)s_ft.step;
    if (travelled < lim)
    {
      travelled = lim;
    }
    if (travelled > 0)
    {
      travelled = 0;
    }
  }

  return (uint16_t)((int32_t)s_ft.anchor - ((int32_t)s_ft.step / 2) + travelled);
}

uint16_t MotorHall6_LastDwell(void)
{
  return s_last_dwell;
}

/* The stall path leaves a huge dwell in the interpolator. The next edge
 * is then thrown away and the angle sits on the hall center, so a rotor
 * that six-step just got moving stops again. Arm from the sector that
 * actually happened. */
void MotorHall6_FocArm(uint16_t dwell_ticks)
{
  uint8_t hall;
  uint8_t dir;
  uint8_t i;
  uint16_t now;
  uint32_t lim;

  hall = MotorHall_ReadRaw();
  dir = (uint8_t)(s_hall6_snap.direction & 1U);
  now = MotorHall6_FocTheta();
  s_ft.anchor = now;
  s_ft.ticks = 0U;
  s_ft.have_stair = 1U;
  s_ft.stair = foc_theta_stair(now);
  for (i = 0U; i < 6U; i++)
  {
    s_ft.last_ticks[i] = 0U;
  }
  if ((dwell_ticks < 40U) || (dwell_ticks > 2000U) ||
      (hall < 1U) || (hall > 6U))
  {
    s_ft.have_omega = 0U;
    s_ft.omega_q8 = 0;
    s_ft.stall_ticks = HALL6_STALL_TICKS;
    return;
  }

  {
    const uint8_t *seq = hall6_seq(dir);
    uint8_t idx = hall6_seq_index(seq, hall);
    uint8_t nxt = seq[(uint8_t)((idx + 1U) % 6U)];
    int16_t raw = (int16_t)(s_theta_hall[dir][nxt] - s_theta_hall[dir][hall]);
    uint16_t width = s_ft_width_q16[s_ft.stair];
    int16_t step = (raw >= 0) ? (int16_t)width : (int16_t)(-(int32_t)width);
    int32_t om = Fx_QuotS((int32_t)step << 8, (int32_t)dwell_ticks);

    if (om > 32767)
    {
      om = 32767;
    }
    else if (om < -32767)
    {
      om = -32767;
    }
    s_ft.step = step;
    s_ft.omega_q8 = om;
    s_ft.have_omega = 1U;
    lim = (uint32_t)dwell_ticks << 1;
    if (lim < HALL6_STALL_TICKS)
    {
      lim = HALL6_STALL_TICKS;
    }
    if (lim > 65535U)
    {
      lim = 65535U;
    }
    s_ft.stall_ticks = (uint16_t)lim;
  }
}

int32_t MotorHall6_FocOmegaQ8(void)
{
  if (s_ft.have_omega == 0U)
  {
    return 0;
  }
  /* No stair for 40 ms ≈ 4 elec/s. The six-sector mean otherwise
   * freezes at the last running speed and the PI holds iq=0. */
  if (s_ft.ticks > HALL6_STALL_TICKS)
  {
    s_ft.omega_spd_q8 = 0;
    s_ft.spd_n = 0U;
    s_ft.spd_i = 0U;
    s_ft.spd_dirty = 0U;
    return 0;
  }
  /* One-turn Σstep/Σticks. Not on the current-loop tick: only a reader
   * (host, or the speed PI) pays the division, and only after an edge. */
  if (s_ft.spd_dirty != 0U)
  {
    int32_t sum_step = 0;
    uint32_t sum_ticks = 0U;
    uint8_t k;

    for (k = 0U; k < s_ft.spd_n; k++)
    {
      sum_step += (int32_t)s_ft.spd_step[k];
      sum_ticks += (uint32_t)s_ft.spd_ticks[k];
    }
    if (sum_ticks > 0U)
    {
      s_ft.omega_spd_q8 = Fx_DivS(sum_step << 8, (int32_t)sum_ticks, 1U);
    }
    s_ft.spd_dirty = 0U;
  }
  return s_ft.omega_spd_q8;
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

static int hall6_prepare_outputs(void)
{
  s_moe_pending = 1U;
  MotorPwm_HardwareSafe();
  if (MotorPwm_ArmOutputs() == 0)
  {
    s_moe_pending = 0U;
    return 0;
  }
  MotorPwm_ChargeBootstrap();
  if ((htim1.Instance->BDTR & TIM_BDTR_MOE) == 0U)
  {
    s_moe_pending = 0U;
    return 0;
  }
  return 1;
}

int MotorHall6_IsForward(uint8_t prev, uint8_t now)
{
  const uint8_t *seq;
  uint8_t i;

  if ((prev < 1U) || (prev > 6U) || (now < 1U) || (now > 6U))
  {
    return 0;
  }
  seq = hall6_seq((uint8_t)(s_hall6_snap.direction & 1U));
  for (i = 0U; i < 6U; i++)
  {
    if (seq[i] == prev)
    {
      return (seq[(uint8_t)((i + 1U) % 6U)] == now) ? 1 : 0;
    }
  }
  return 0;
}

int MotorHall6_Enable(int enable)
{
  motor_hall6_snapshot_t snap;

  if (enable != 0)
  {
    uint8_t hall;
    uint8_t mapped;

    if (hall6_prepare_outputs() == 0)
    {
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
    s_last_dwell = 0U;
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

int MotorHall6_ResumeSync(void)
{
  motor_hall6_snapshot_t snap;
  uint8_t hall;
  uint8_t mapped;

  if (hall6_prepare_outputs() == 0)
  {
    return 0;
  }

  s_hall6_enable = 1U;
  s_kick_active = 0U;
  s_ticks_since_edge = 0U;
  /* Drop the stale sector time. The edge count stays, so a rotor that
   * is already turning does not start the open-loop kick again. */
  s_last_dwell = 0U;

  snap = s_hall6_snap;
  snap.enabled = 1U;
  snap.fault = 0U;
  snap.loop_count = 0U;
  snap.kick = 0U;
  snap.duty_pct = s_run_duty_pct;
  s_hall6_snap.duty_pct = s_run_duty_pct;
  hall = MotorHall_ReadRaw();
  s_last_gpio_hall = hall;
  mapped = hall6_map_hall(hall);
  snap.hall_raw = hall;
  hall6_store(&snap);
  hall6_commutate(mapped);
  hall6_moe_release();
  if (s_moe_pending != 0U)
  {
    MotorPwm_MoeEnable();
    s_moe_pending = 0U;
  }

  MotorTick_Start();
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
      s_last_dwell = s_ticks_since_edge;
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
    /* The 40 ms bootstrap window used to swallow the first kick step:
     * the index advanced, but commutate returned and left the bridge
     * in all-low. Hold the due step and apply it once the window ends. */
    if (s_kick_div < MOTOR_HALL6_KICK_INTERVAL)
    {
      s_kick_div++;
    }
    if ((s_kick_div >= MOTOR_HALL6_KICK_INTERVAL) &&
        (MotorPwm_BootstrapReady() != 0))
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
