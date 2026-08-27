#include "motor_angle.h"

volatile motor_angle_state_t g_motor_angle;

/*
 * Sector index rises with electrical angle in the direction hall6 calls
 * direction 0. The code order 6,2,3,1,5,4 was read off the target (stage A1
 * capture) and is the canonical 120-degree hall pattern:
 *   (A,B,C) = 110 -> 010 -> 011 -> 001 -> 101 -> 100
 */
static const uint8_t s_hall_to_sector[8] = {
    0xFFU, /* 0 invalid */
    3U,    /* 1 = 001 */
    1U,    /* 2 = 010 */
    2U,    /* 3 = 011 */
    5U,    /* 4 = 100 */
    4U,    /* 5 = 101 */
    0U,    /* 6 = 110 */
    0xFFU  /* 7 invalid */
};

/* Rounded so the six anchors span exactly one revolution. */
static const uint16_t s_anchor[MOTOR_ANGLE_SECTORS] = {
    0U, 10923U, 21845U, 32768U, 43691U, 54613U
};

static const int16_t s_anchor_trim[MOTOR_ANGLE_SECTORS] = MOTOR_ANGLE_ANCHOR_TRIM;

#define SECTOR_SPAN 10923U /* 65536 / 6, rounded */

/* Duration of each sector on its most recent pass. The sector widths are
 * unequal but repeat to 1.5% per revolution, so last revolution's value for
 * *this* sector predicts far better than the sector just left: predicting from
 * the neighbour gives edge jumps up to 15 degrees, this gives about 1. */
static uint16_t s_sector_ticks[MOTOR_ANGLE_SECTORS];

void MotorAngle_Reset(void)
{
  uint8_t i;

  for (i = 0U; i < MOTOR_ANGLE_SECTORS; i++)
  {
    s_sector_ticks[i] = 0U;
  }

  g_motor_angle.theta = 0U;
  g_motor_angle.omega_q8 = 0;
  g_motor_angle.ticks_in_sector = 0U;
  g_motor_angle.sector = 0xFFU;
  g_motor_angle.hall = 0U;
  g_motor_angle.dir = 0;
  g_motor_angle.valid = 0U;
  g_motor_angle.edge_jump = 0U;
  g_motor_angle.bad_edges = 0U;
  g_motor_angle.edges = 0U;
}

static uint16_t angle_anchor(uint8_t sector)
{
  return (uint16_t)((int32_t)s_anchor[sector] + (int32_t)s_anchor_trim[sector] +
                    (int32_t)MOTOR_ANGLE_OFFSET_Q16);
}

static uint16_t angle_abs_diff(uint16_t a, uint16_t b)
{
  uint16_t d = (uint16_t)(a - b);

  return (d > 32768U) ? (uint16_t)(0U - d) : d;
}

static void angle_on_edge(uint8_t sector)
{
  uint8_t prev = g_motor_angle.sector;
  uint16_t measured = g_motor_angle.ticks_in_sector;
  uint16_t predicted;
  int8_t dir;
  uint16_t edge_angle;

  g_motor_angle.edges++;

  if (prev == 0xFFU)
  {
    /* First edge: nothing to compare against, so only the anchor is known. */
    g_motor_angle.sector = sector;
    g_motor_angle.theta = angle_anchor(sector);
    g_motor_angle.ticks_in_sector = 0U;
    g_motor_angle.omega_q8 = 0;
    return;
  }

  if (sector == ((prev + 1U) % MOTOR_ANGLE_SECTORS))
  {
    dir = 1;
  }
  else if (prev == ((sector + 1U) % MOTOR_ANGLE_SECTORS))
  {
    dir = -1;
  }
  else
  {
    /* Two or more sectors at once cannot happen at any speed this loop can
     * follow: a sector is at least 16 ticks even at 3000 rpm. So this is a
     * missed edge or a glitch -- record it and resynchronise to the anchor
     * rather than extrapolating from a bogus interval. */
    g_motor_angle.bad_edges++;
    g_motor_angle.sector = sector;
    g_motor_angle.theta = angle_anchor(sector);
    g_motor_angle.ticks_in_sector = 0U;
    g_motor_angle.omega_q8 = 0;
    g_motor_angle.dir = 0;
    return;
  }

  /* The boundary just crossed: going forward it is the entered sector's anchor,
   * going backward it is the anchor of the sector being left. */
  edge_angle = (dir > 0) ? angle_anchor(sector) : angle_anchor(prev);
  g_motor_angle.edge_jump = angle_abs_diff(g_motor_angle.theta, edge_angle);

  /* Scale this sector's previous duration by however much the sector just left
   * changed since its own previous pass. That separates speed (from the fresh
   * measurement) from geometry (from the per-sector memory). */
  predicted = s_sector_ticks[sector];
  if ((predicted != 0U) && (s_sector_ticks[prev] != 0U) && (measured != 0U))
  {
    uint32_t scaled = ((uint32_t)predicted * (uint32_t)measured) /
                      (uint32_t)s_sector_ticks[prev];

    predicted = (scaled == 0U) ? 1U : (uint16_t)((scaled > 0xFFFFU) ? 0xFFFFU : scaled);
  }
  else
  {
    predicted = (measured != 0U) ? measured : 1U;
  }

  s_sector_ticks[prev] = measured;

  /* One division per edge, not per tick: at 24 electrical rev/s that is 144 a
   * second, which is nothing even without a hardware divider. */
  g_motor_angle.omega_q8 =
      (int32_t)(((uint32_t)SECTOR_SPAN << 8) / (uint32_t)predicted);
  if (dir < 0)
  {
    g_motor_angle.omega_q8 = -g_motor_angle.omega_q8;
  }

  g_motor_angle.dir = dir;
  g_motor_angle.sector = sector;
  g_motor_angle.theta = edge_angle;
  g_motor_angle.ticks_in_sector = 0U;
  g_motor_angle.valid = 1U;
}

void MotorAngle_Update(uint8_t hall)
{
  uint8_t sector = s_hall_to_sector[hall & 7U];

  g_motor_angle.hall = hall;

  if (sector == 0xFFU)
  {
    /* Invalid code: hold the last angle rather than jumping somewhere wrong. */
    g_motor_angle.valid = 0U;
    return;
  }

  if (sector != g_motor_angle.sector)
  {
    angle_on_edge(sector);
    return;
  }

  if (g_motor_angle.ticks_in_sector < 0xFFFFU)
  {
    g_motor_angle.ticks_in_sector++;
  }

  if (g_motor_angle.ticks_in_sector > MOTOR_ANGLE_STALL_TICKS)
  {
    g_motor_angle.valid = 0U;
    g_motor_angle.omega_q8 = 0;
    return;
  }

  if (g_motor_angle.valid == 0U)
  {
    return;
  }

  {
    int32_t travelled =
        (g_motor_angle.omega_q8 * (int32_t)g_motor_angle.ticks_in_sector) >> 8;
    int32_t limit = (int32_t)SECTOR_SPAN - 1;
    uint16_t base;

    /* Never extrapolate past the next anchor. Overrunning it would make the
     * angle run away while decelerating, and a wrong angle is worse than a
     * stale one because it feeds straight into the Park transform. */
    if (travelled > limit)
    {
      travelled = limit;
    }
    else if (travelled < -limit)
    {
      travelled = -limit;
    }

    base = (g_motor_angle.dir > 0) ? angle_anchor(g_motor_angle.sector)
                                   : angle_anchor((uint8_t)((g_motor_angle.sector + 1U) %
                                                            MOTOR_ANGLE_SECTORS));
    g_motor_angle.theta = (uint16_t)((int32_t)base + travelled);
  }
}
