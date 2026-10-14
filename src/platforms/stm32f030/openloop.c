#include "openloop.h"
#include "motor_pwm.h"
#include "motor_hall.h"
#include "motor_tick.h"

#define MOTOR_OPENLOOP_DEFAULT_MS 45U
/* 1.3 ohm phase-to-phase at 12 V: dead time still subtracts from the HS pulse,
 * so ~20% commanded duty is needed for ~1 A of winding current. */
#define MOTOR_OPENLOOP_MAX_DUTY  25U

typedef motor_phase_mode_t ol_phase_mode_t;

typedef struct {
  ol_phase_mode_t u;
  ol_phase_mode_t v;
  ol_phase_mode_t w;
} ol_step_t;

static const ol_step_t s_table_cw[8] = {
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

static motor_openloop_snapshot_t s_ol_snap;
static volatile uint8_t s_ol_enable;
static uint8_t s_initialized;
static uint8_t s_hall_sync;
static uint8_t s_hall_stable;
static uint8_t s_hall_candidate;
static uint8_t s_hall_debounce;
static uint8_t s_hall_invert;
static uint8_t s_kick_invert;
static int8_t s_hall_spin;
static uint8_t s_hall_wrong;
static uint8_t s_hall_phase;
static uint8_t s_uvw_perm;
/* 0 = derive pulse from duty_pct. 1% of ARR is 12 counts here, too coarse near
 * the dead-time threshold, so allow the raw CCR value instead. */
static uint16_t s_pulse_counts;
#define OL_PULSE_COUNTS_MAX 360U
static uint8_t s_hall_good_edges;
static uint8_t s_hall_locked;
static uint32_t s_step_div;
static uint32_t s_hall_last_edge_loop;

/* Map TIM1 U/V/W (PA8/PA9/PA10) to logical commutation phases. */
static const uint8_t s_uvw_perm_lut[6][3] = {
  { 0U, 1U, 2U },
  { 0U, 2U, 1U },
  { 1U, 0U, 2U },
  { 1U, 2U, 0U },
  { 2U, 0U, 1U },
  { 2U, 1U, 0U },
};

#define OL_HALL_DEBOUNCE     12U
#define OL_HALL_LOCK_EDGES   3U
#define OL_HALL_STALL_MS    120U
#define OL_HALL_STALL_TICKS ((OL_HALL_STALL_MS * MOTOR_CTRL_ISR_HZ) / 1000U)

static void ol_store(const motor_openloop_snapshot_t *src)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_ol_snap = *src;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void ol_apply_phase(TIM_TypeDef *tim, uint32_t ccer_e, uint32_t ccer_ne,
                           volatile uint32_t *ccr, ol_phase_mode_t mode,
                           uint32_t pulse, uint32_t arr)
{
  switch (mode)
  {
  case MOTOR_PHASE_OFF:
    tim->CCER &= ~(ccer_e | ccer_ne);
    *ccr = 0U;
    break;
  case MOTOR_PHASE_PWM:
#if defined(MOTOR_HS_ONLY) && (MOTOR_HS_ONLY != 0)
    /* Diagnostic: high side alone, so this leg's own low side cannot conduct. */
    tim->CCER &= ~ccer_ne;
    tim->CCER |= ccer_e;
#else
    tim->CCER |= (ccer_e | ccer_ne);
#endif
    *ccr = pulse;
    break;
  case MOTOR_PHASE_LOW:
    /* With CCxE cleared the OCxN pin follows OCxREF directly - no complement and
     * no dead time - so CCR must keep OCxREF permanently active to hold the low
     * side on. CCR=0 would leave it off and the leg could not sink current. */
    tim->CCER &= ~ccer_e;
    tim->CCER |= ccer_ne;
    *ccr = arr + 1U;
    break;
  default:
    tim->CCER &= ~(ccer_e | ccer_ne);
    *ccr = 0U;
    break;
  }
}

static void ol_apply_uvw_tim(ol_phase_mode_t u, ol_phase_mode_t v, ol_phase_mode_t w, uint8_t duty_pct)
{
  TIM_TypeDef *tim = htim1.Instance;
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  uint32_t pulse;

  if (s_pulse_counts != 0U)
  {
    pulse = (uint32_t)s_pulse_counts;
  }
  else
  {
    pulse = (arr * (uint32_t)duty_pct) / 100U;
  }

  if (pulse > arr)
  {
    pulse = arr;
  }

  ol_apply_phase(tim, TIM_CCER_CC1E, TIM_CCER_CC1NE, &tim->CCR1, u, pulse, arr);
  ol_apply_phase(tim, TIM_CCER_CC2E, TIM_CCER_CC2NE, &tim->CCR2, v, pulse, arr);
  ol_apply_phase(tim, TIM_CCER_CC3E, TIM_CCER_CC3NE, &tim->CCR3, w, pulse, arr);
}

static void ol_apply_uvw(ol_phase_mode_t u, ol_phase_mode_t v, ol_phase_mode_t w, uint8_t duty_pct)
{
  const uint8_t *map = s_uvw_perm_lut[s_uvw_perm % 6U];
  ol_phase_mode_t logical[3] = { u, v, w };

  ol_apply_uvw_tim(logical[map[0]], logical[map[1]], logical[map[2]], duty_pct);
}

static const ol_step_t *ol_lookup_step(uint8_t hall, uint8_t ccw)
{
  const ol_step_t *st;

  if (hall >= 8U)
  {
    return &s_table_cw[0];
  }

  st = &s_table_cw[hall];
  if (ccw == 0U)
  {
    return st;
  }

  static ol_step_t rev;
  rev.u = st->u;
  rev.v = st->v;
  rev.w = st->w;
  if (rev.u == MOTOR_PHASE_PWM) { rev.u = MOTOR_PHASE_LOW; }
  else if (rev.u == MOTOR_PHASE_LOW) { rev.u = MOTOR_PHASE_PWM; }
  if (rev.v == MOTOR_PHASE_PWM) { rev.v = MOTOR_PHASE_LOW; }
  else if (rev.v == MOTOR_PHASE_LOW) { rev.v = MOTOR_PHASE_PWM; }
  if (rev.w == MOTOR_PHASE_PWM) { rev.w = MOTOR_PHASE_LOW; }
  else if (rev.w == MOTOR_PHASE_LOW) { rev.w = MOTOR_PHASE_PWM; }
  return &rev;
}

static const uint8_t *ol_seq6(uint8_t ccw)
{
  return (ccw != 0U) ? s_seq_cw : s_seq_ccw;
}

static uint8_t ol_seq_index(const uint8_t *seq, uint8_t hall)
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

static uint8_t ol_map_hall(uint8_t raw)
{
  const uint8_t *seq;

  if ((raw == 0U) || (raw == 7U))
  {
    return raw;
  }

  seq = ol_seq6(s_ol_snap.direction);
  return seq[(ol_seq_index(seq, raw) + s_hall_phase) % 6U];
}

static uint8_t ol_lookup_commute(void)
{
  return s_hall_invert;
}

static const uint8_t *ol_kick_seq(void)
{
  if (s_ol_snap.direction != 0U)
  {
    return s_kick_invert ? s_seq_ccw : s_seq_cw;
  }
  return s_kick_invert ? s_seq_cw : s_seq_ccw;
}

static int8_t ol_hall_spin_detect(uint8_t prev, uint8_t next)
{
  uint8_t ip;
  uint8_t in;

  if ((prev == 0U) || (prev == 7U) || (next == 0U) || (next == 7U))
  {
    return 0;
  }

  ip = ol_seq_index(s_seq_cw, prev);
  in = ol_seq_index(s_seq_cw, next);
  if (in == (uint8_t)((ip + 1U) % 6U))
  {
    return 1;
  }
  if (in == (uint8_t)((ip + 5U) % 6U))
  {
    return -1;
  }
  return 0;
}

static int ol_hall_edge_ok(int8_t spin)
{
  if (spin == 0)
  {
    return 0;
  }

  s_hall_spin = spin;
  if (s_ol_snap.direction != 0U)
  {
    return (spin > 0) ? 1 : 0;
  }
  return (spin < 0) ? 1 : 0;
}

static void ol_hall_wrong_edge(void)
{
  if (s_hall_locked != 0U)
  {
    return;
  }

  s_hall_wrong++;
  if (s_hall_wrong >= 2U)
  {
    s_hall_invert ^= 1U;
    s_kick_invert ^= 1U;
    s_hall_wrong = 0U;
  }
}

static void ol_hall_good_edge(void)
{
  s_hall_last_edge_loop = s_ol_snap.loop_count;

  if (s_hall_good_edges < 255U)
  {
    s_hall_good_edges++;
  }
  if (s_hall_good_edges >= OL_HALL_LOCK_EDGES)
  {
    s_hall_locked = 1U;
  }
}

static void ol_hall_stall_check(void)
{
  if ((s_hall_locked != 0U) &&
      ((s_ol_snap.loop_count - s_hall_last_edge_loop) > OL_HALL_STALL_TICKS))
  {
    s_hall_locked = 0U;
    s_hall_good_edges = 0U;
    s_step_div = 0U;
  }
}

static void ol_apply_hall_commute(uint8_t table_hall)
{
  const ol_step_t *st;

  if ((table_hall == 0U) || (table_hall == 7U))
  {
    return;
  }

  st = ol_lookup_step(table_hall, ol_lookup_commute());
  ol_apply_uvw(st->u, st->v, st->w, s_ol_snap.duty_pct);
}

static void ol_apply_step6_kick(uint8_t step_idx, uint8_t duty_pct)
{
  const uint8_t *seq = ol_kick_seq();
  uint8_t hall = seq[(step_idx + s_hall_phase) % 6U];
  const ol_step_t *st = ol_lookup_step(hall, 0U);

  ol_apply_uvw(st->u, st->v, st->w, duty_pct);
}

static void ol_apply_step6(uint8_t step_idx, uint8_t duty_pct)
{
  const uint8_t *seq = ol_seq6(s_ol_snap.direction);
  uint8_t hall = seq[(step_idx + s_hall_phase) % 6U];
  const ol_step_t *st = ol_lookup_step(hall, 0U);

  ol_apply_uvw(st->u, st->v, st->w, duty_pct);
}

static void ol_apply_step3(uint8_t step, uint8_t duty_pct)
{
  ol_phase_mode_t u = MOTOR_PHASE_OFF;
  ol_phase_mode_t v = MOTOR_PHASE_OFF;
  ol_phase_mode_t w = MOTOR_PHASE_OFF;

  switch (step % 3U)
  {
  case 0U: u = MOTOR_PHASE_PWM; break;
  case 1U: v = MOTOR_PHASE_PWM; break;
  default: w = MOTOR_PHASE_PWM; break;
  }

  ol_apply_uvw(u, v, w, duty_pct);
}

static void ol_apply_step(uint8_t step, uint8_t duty_pct)
{
  if (s_ol_snap.mode == MOTOR_OPENLOOP_MODE_6STEP)
  {
    ol_apply_step6(step, duty_pct);
  }
  else
  {
    ol_apply_step3(step, duty_pct);
  }
}

static uint8_t ol_next_step(uint8_t step)
{
  if (s_ol_snap.mode == MOTOR_OPENLOOP_MODE_6STEP)
  {
    return (uint8_t)((step + 1U) % 6U);
  }

  if (s_hall_sync != 0U)
  {
    return (uint8_t)((step + 1U) % 6U);
  }

  if (s_ol_snap.direction != 0U)
  {
    return (uint8_t)((step + 1U) % 3U);
  }
  return (uint8_t)((step + 2U) % 3U);
}

/* ISR ticks between commutations; 0 means hold the present step. */
static uint32_t ol_step_ticks(void);

static uint16_t ol_effective_step_ms(void)
{
  uint32_t elapsed_ms;

  if (s_ol_snap.ramp_ms == 0U)
  {
    return s_ol_snap.step_ms;
  }

  elapsed_ms = s_ol_snap.loop_count / (MOTOR_CTRL_ISR_HZ / 1000U);
  if (elapsed_ms >= (uint32_t)s_ol_snap.ramp_ms)
  {
    return s_ol_snap.ramp_end_ms;
  }

  {
    uint32_t start = (uint32_t)s_ol_snap.ramp_start_ms;
    uint32_t end = (uint32_t)s_ol_snap.ramp_end_ms;
    uint32_t span = (uint32_t)s_ol_snap.ramp_ms;
    int32_t delta = (int32_t)end - (int32_t)start;
    return (uint16_t)(start + ((delta * (int32_t)elapsed_ms) / (int32_t)span));
  }
}

static uint32_t ol_step_ticks(void)
{
  uint16_t ms = ol_effective_step_ms();
  uint32_t ticks;

  if (ms == 0U)
  {
    return 0U;
  }

  ticks = ((uint32_t)ms * MOTOR_CTRL_ISR_HZ) / 1000U;
  return (ticks < 1U) ? 1U : ticks;
}

void MotorOpenloop_Init(void)
{
  if (s_initialized != 0U)
  {
    return;
  }

  s_ol_snap.duty_pct = 6U;
  s_ol_snap.step_ms = MOTOR_OPENLOOP_DEFAULT_MS;
  s_ol_snap.ramp_start_ms = 0U;
  s_ol_snap.ramp_end_ms = 0U;
  s_ol_snap.ramp_ms = 0U;
  s_ol_snap.direction = 0U;
  s_ol_snap.mode = MOTOR_OPENLOOP_MODE_3STEP;
  s_ol_snap.step = 0U;
  s_uvw_perm = 0U;
  s_hall_sync = 0U;
  s_hall_stable = 0U;
  s_hall_candidate = 0U;
  s_hall_debounce = 0U;
  s_hall_invert = 0U;
  s_kick_invert = 0U;
  s_hall_spin = 0;
  s_hall_wrong = 0U;
  s_hall_good_edges = 0U;
  s_hall_locked = 0U;
  s_hall_last_edge_loop = 0U;
  s_hall_phase = 3U;
  s_ol_enable = 0U;
  s_initialized = 1U;
}

int MotorOpenloop_IsEnabled(void)
{
  return (s_ol_enable != 0U) ? 1 : 0;
}

void MotorOpenloop_SetDutyPct(uint8_t pct)
{
  if (pct > MOTOR_OPENLOOP_MAX_DUTY)
  {
    pct = MOTOR_OPENLOOP_MAX_DUTY;
  }
  s_ol_snap.duty_pct = pct;
}

void MotorOpenloop_SetStepMs(uint16_t ms)
{
  /* 0 holds the present step indefinitely (bench diagnostics); a non-zero rate
   * is clamped to a range the rotor can follow. */
  if (ms != 0U)
  {
    if (ms < 5U) { ms = 5U; }
    if (ms > 500U) { ms = 500U; }
  }
  s_ol_snap.step_ms = ms;
}

void MotorOpenloop_SetRampMs(uint16_t start_ms, uint16_t end_ms, uint16_t ramp_ms)
{
  if (start_ms < 5U) { start_ms = 5U; }
  if (end_ms < 5U) { end_ms = 5U; }
  if (start_ms > 500U) { start_ms = 500U; }
  if (end_ms > 500U) { end_ms = 500U; }
  s_ol_snap.ramp_start_ms = start_ms;
  s_ol_snap.ramp_end_ms = end_ms;
  s_ol_snap.ramp_ms = ramp_ms;
  s_ol_snap.step_ms = end_ms;
}

void MotorOpenloop_SetDirection(int ccw)
{
  s_ol_snap.direction = (ccw != 0) ? 1U : 0U;
}

void MotorOpenloop_SetMode(uint8_t mode)
{
  s_ol_snap.mode = (mode == MOTOR_OPENLOOP_MODE_6STEP) ?
                MOTOR_OPENLOOP_MODE_6STEP : MOTOR_OPENLOOP_MODE_3STEP;
}

void MotorOpenloop_SetHallSync(int enable)
{
  s_hall_sync = (enable != 0) ? 1U : 0U;
  s_hall_stable = 0U;
  s_hall_candidate = 0U;
  s_hall_debounce = 0U;
  s_hall_invert = 0U;
  s_kick_invert = 0U;
  s_hall_spin = 0;
  s_hall_wrong = 0U;
  s_hall_good_edges = 0U;
  s_hall_locked = 0U;
}

void MotorOpenloop_SetHallPhase(uint8_t phase)
{
  s_hall_phase = (uint8_t)(phase % 6U);
}

void MotorOpenloop_SetUvPerm(uint8_t perm)
{
  s_uvw_perm = (uint8_t)(perm % 6U);
}

void MotorOpenloop_SetPulseCounts(uint16_t counts)
{
  if (counts > OL_PULSE_COUNTS_MAX)
  {
    counts = OL_PULSE_COUNTS_MAX;
  }
  s_pulse_counts = counts;
}

void MotorOpenloop_GetSnapshot(motor_openloop_snapshot_t *out)
{
  uint32_t primask;

  if (out == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *out = s_ol_snap;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

int MotorOpenloop_Enable(int enable)
{
  motor_openloop_snapshot_t snap;

  if (enable != 0)
  {
    MotorPwm_Stop();

    if (MotorPwm_Start() == 0)
    {
      MotorPwm_Stop();
      return 0;
    }

    s_step_div = 0U;
    s_ol_enable = 1U;
    s_hall_invert = 0U;
    s_kick_invert = 0U;
    s_hall_spin = 0;
    s_hall_wrong = 0U;
    s_hall_good_edges = 0U;
    s_hall_locked = 0U;
    s_hall_last_edge_loop = 0U;

    snap = s_ol_snap;
    snap.enabled = 1U;
    snap.step = 0U;
    snap.step_count = 0U;
    snap.loop_count = 0U;
    ol_store(&snap);

    ol_apply_step(0U, s_ol_snap.duty_pct);
    if (s_hall_sync != 0U)
    {
      uint8_t hall = MotorHall_ReadRaw();

      if ((hall != 0U) && (hall != 7U))
      {
        s_hall_stable = hall;
        s_hall_candidate = hall;
        s_hall_debounce = 0U;
        snap.step = ol_seq_index(ol_seq6(s_ol_snap.direction), hall);
        ol_store(&snap);
        ol_apply_hall_commute(ol_map_hall(hall));
      }
    }

    MotorTick_Start();
    return 1;
  }

  s_ol_enable = 0U;
  MotorTick_Stop();
  MotorPwm_Stop();

  snap = s_ol_snap;
  snap.enabled = 0U;
  ol_store(&snap);
  return 1;
}

void MotorOpenloop_ControlLoopISR(void)
{
  motor_openloop_snapshot_t snap;
  uint32_t ticks;

  if (s_ol_enable == 0U)
  {
    return;
  }

  if (s_hall_sync != 0U)
  {
    uint8_t hall = MotorHall_ReadRaw();
    uint8_t hall_stepped = 0U;

    if ((hall != 0U) && (hall != 7U))
    {
      if (hall == s_hall_stable)
      {
        s_hall_debounce = 0U;
      }
      else if (hall == s_hall_candidate)
      {
        s_hall_debounce++;
        if (s_hall_debounce >= OL_HALL_DEBOUNCE)
        {
          int8_t spin = ol_hall_spin_detect(s_hall_stable, hall);

          if (ol_hall_edge_ok(spin) != 0)
          {
            s_hall_wrong = 0U;
            ol_hall_good_edge();
            snap = s_ol_snap;
            snap.step = ol_seq_index(ol_seq6(s_ol_snap.direction), hall);
            snap.step_count++;
            ol_store(&snap);
            ol_apply_hall_commute(ol_map_hall(hall));
            hall_stepped = 1U;
          }
          else if (spin != 0)
          {
            ol_hall_wrong_edge();
          }
          s_hall_stable = hall;
          s_hall_debounce = 0U;
        }
      }
      else
      {
        s_hall_candidate = hall;
        s_hall_debounce = 1U;
      }
    }

    ol_hall_stall_check();

    if ((hall_stepped == 0U) && (s_hall_locked == 0U))
    {
      ticks = ol_step_ticks();

      if (ticks != 0U)
      {
        s_step_div++;
        if (s_step_div >= ticks)
        {
          s_step_div = 0U;
          snap = s_ol_snap;
          snap.step = ol_next_step(snap.step);
          snap.step_count++;
          ol_store(&snap);
          ol_apply_step6_kick(s_ol_snap.step, s_ol_snap.duty_pct);
        }
      }
    }

    snap = s_ol_snap;
    snap.hall_raw = hall;
    snap.hall_spin = s_hall_spin;
    snap.hall_invert = s_hall_invert;
    snap.hall_sync_on = 1U;
    snap.hall_locked = s_hall_locked;
    snap.loop_count++;
    ol_store(&snap);
    return;
  }

  ticks = ol_step_ticks();

  if (ticks != 0U)
  {
    s_step_div++;
  }
  if ((ticks != 0U) && (s_step_div >= ticks))
  {
    s_step_div = 0U;
    snap = s_ol_snap;
    snap.step = ol_next_step(snap.step);
    snap.step_count++;
    ol_store(&snap);
    ol_apply_step(s_ol_snap.step, s_ol_snap.duty_pct);
  }

  snap = s_ol_snap;
  snap.loop_count++;
  ol_store(&snap);

#if defined(MOTOR_PHASE_SWEEP) && (MOTOR_PHASE_SWEEP != 0)
  if ((s_ol_snap.step_count > 0U) && ((s_ol_snap.step_count % 50U) == 0U))
  {
    s_hall_phase = (uint8_t)((s_hall_phase + 1U) % 6U);
    ol_apply_step(s_ol_snap.step, s_ol_snap.duty_pct);
  }
#endif
#if defined(MOTOR_UVW_SWEEP) && (MOTOR_UVW_SWEEP != 0)
  if ((s_ol_snap.step_count > 0U) && ((s_ol_snap.step_count % 100U) == 0U))
  {
    s_uvw_perm = (uint8_t)((s_uvw_perm + 1U) % 6U);
    ol_apply_step(s_ol_snap.step, s_ol_snap.duty_pct);
  }
#endif
}
