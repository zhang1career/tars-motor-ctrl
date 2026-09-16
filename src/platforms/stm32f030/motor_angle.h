#ifndef MOTOR_ANGLE_H
#define MOTOR_ANGLE_H

#include <stdint.h>

/*
 * Continuous rotor electrical angle from the three hall sensors.
 *
 * The halls give six anchors per electrical revolution; FOC needs an angle at
 * every tick, so the angle is extrapolated between anchors and corrected to the
 * anchor when an edge arrives.
 *
 * Units: 65536 counts per electrical revolution. That wraps for free in a
 * uint16_t and the top bits index a sin/cos table directly.
 *
 * The anchors are placed at uniform 60 degrees. The six sectors are NOT actually
 * uniform -- measured 53.6 to 68.1 degrees, repeatable to 1.5% -- but that
 * measurement is contaminated by commutation torque ripple at the same order as
 * the geometry it would be calibrating (docs/measurement-validity.md 3.6). The
 * offsets get calibrated later against FOC's own id, which responds to angle
 * error directly. MOTOR_ANGLE_OFFSET_Q16 is where that result goes.
 */

#define MOTOR_ANGLE_SECTORS 6U

/*
 * Two distinct corrections, both calibration results rather than constants,
 * both zero until stage E measures them against id:
 *
 * OFFSET_Q16 is the alignment between hall code and electrical zero. It depends
 * on where the sensors sit relative to the magnets and on the phase wiring, and
 * it is one number common to all six anchors.
 *
 * ANCHOR_TRIM is the anchor *spacing* error. Uniform 60-degree anchors are
 * wrong because the real sectors measure 53.6 to 68.1 degrees (A1.1): the true
 * angle of anchor k is the cumulative sum of the real widths, so accumulating
 * those measurements puts anchor 2 about 10 degrees off. Six numbers are needed,
 * and a global offset cannot substitute for them.
 *
 * The A1.1 timing figures are deliberately NOT baked in here: they are
 * contaminated by commutation torque ripple at the same order as the geometry
 * (measurement-validity 3.6), so they would trade a known error for an unknown
 * one.
 */
#ifndef MOTOR_ANGLE_OFFSET_Q16
/* Left at 0. After moving the tick to ADC DMA TC (2026-08-27) the
 * shunts show the hall-synchronous current. 2 s acc at OFFSET=0:
 *   stair  CW (id,iq)=(+0.3,+298) mA   CCW (+7.9,−269) mA
 *   interp CW (+148,+250)              CCW (−108,−235)
 * Staircase id ≈ 0 and iq flips with the torque sign, so the anchors
 * already sit on the 6-step voltage. The interpolated ~−29° is the
 * mean of a 60° sweep of that same vector, not an alignment error.
 * Do not write −5243. Do not write +4190 (+23 deg) here either: that
 * number cancelled MotorHall6_FocThetaInterp lag, a different angle
 * path. CURRENT Park uses hall6 FocTheta, not these anchors.
 * Do not close the current loop on TRIM yet. */
#define MOTOR_ANGLE_OFFSET_Q16 (0)
#endif

#ifndef MOTOR_ANGLE_ANCHOR_TRIM
#define MOTOR_ANGLE_ANCHOR_TRIM { 0, 0, 0, 0, 0, 0 }
#endif

/* No hall edge for this many ticks means the extrapolation is meaningless.
 * 4000 ticks at 20 kHz is 200 ms per 60 degrees, i.e. below ~0.8 electrical
 * rev/s. */
#ifndef MOTOR_ANGLE_STALL_TICKS
#define MOTOR_ANGLE_STALL_TICKS 4000U
#endif

typedef struct
{
  uint16_t theta;          /* electrical angle, 65536 per revolution */
  int32_t  omega_q8;       /* angle counts per tick, x256; signed by direction */
  uint16_t ticks_in_sector;
  uint8_t  sector;         /* 0..5, or 0xFF before the first valid edge */
  uint8_t  hall;
  int8_t   dir;            /* +1, -1, or 0 while unknown */
  uint8_t  valid;          /* 0 when stalled or not yet synchronised */
  uint16_t edge_jump;      /* |extrapolated - anchor| at the last edge; the
                            * direct measure of extrapolation quality */
  uint16_t bad_edges;      /* hall jumped by more than one sector: a missed edge
                            * or a glitch, never normal rotation */
  uint16_t edges;
} motor_angle_state_t;

extern volatile motor_angle_state_t g_motor_angle;

void MotorAngle_Reset(void);
void MotorAngle_ResetAcc(void);
int32_t MotorAngle_ThetaAcc(void);

/* While FOC is producing torque, electrical direction must follow the
 * commanded iq, not the last hall edge. A single reverse edge is chatter
 * at a boundary (the 0.3 A close sat there and jittered). 0 = follow edges. */
void MotorAngle_SetDirHint(int8_t dir);

/* Call once per control tick with the raw hall code. */
void MotorAngle_Update(uint8_t hall);

/* Current sector's anchor (uniform 60° + TRIM + OFFSET). Holds still
 * between edges; used to Park 6-step current as a DC vector. */
uint16_t MotorAngle_SectorAnchor(void);

#endif /* MOTOR_ANGLE_H */
