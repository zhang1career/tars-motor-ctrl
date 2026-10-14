#ifndef MOTOR_FOC_FX_H
#define MOTOR_FOC_FX_H

#include <stdint.h>

/*
 * Fixed-point Hall-sensored FOC. Same structure as motor_foc.c (Clarke, Park,
 * two current PIs, inverse Park, SVPWM) but integer throughout so it fits the
 * 20 kHz control ISR. Spec: docs/tasks/fixed-point-foc.md.
 *
 * Currents stay in ADC LSB (1.617 mA). Voltages stay in uV until inverse Park,
 * then VU = 1024 uV. sin/cos are Q15. No soft-float in MotorFocFx_Step.
 */

enum
{
  MOTOR_FOC_FX_OFF = 0U,
  MOTOR_FOC_FX_OBSERVE = 1U,
  MOTOR_FOC_FX_CURRENT = 2U
};

typedef struct
{
  int32_t id_lsb;
  int32_t iq_lsb;
  int32_t vd_uv;
  int32_t vq_uv;
  int32_t iq_ref_lsb;
  uint16_t theta;
  uint16_t ccr[3];
  uint32_t steps;
  uint8_t mode;
  uint8_t sat;
  int16_t dth;           /* FocThetaInterp - FocTheta, signed Q16 */
  uint16_t theta_interp; /* discrete FocTheta */
} motor_foc_fx_snapshot_t;

extern volatile motor_foc_fx_snapshot_t g_motor_foc_fx;

/* Running sum of id/iq, reset from the host after the rotor is spinning so
 * the mean is not contaminated by the open-loop kick. n is the sample count.
 * Host mean = acc / n. 4 s at 20 kHz stays inside int32.
 *
 * CURRENT Park uses discrete FocTheta. id/iq are in that frame.
 * id_st/iq_st use the motor_angle sector anchor.
 * id_ip/iq_ip use the motor_angle interpolator (observe only).
 * dth is the sum of (FocThetaInterp - FocTheta). */
typedef struct
{
  int32_t id;
  int32_t iq;
  uint32_t n;
  int32_t id_st;
  int32_t iq_st;
  int32_t dth;
  int32_t id_ip;
  int32_t iq_ip;
  int32_t i0;            /* sum of (ia+ib+ic)/3 before removal */
} motor_foc_fx_acc_t;

extern volatile motor_foc_fx_acc_t g_motor_foc_fx_acc;

/* 300 mA / 1.617 mA per LSB. Matches hall6-observe staircase iq. */
#ifndef MOTOR_FOC_IQ_LSB
#define MOTOR_FOC_IQ_LSB 186
#endif
#ifndef MOTOR_FOC_ID_LSB
#define MOTOR_FOC_ID_LSB MOTOR_FOC_IQ_LSB
#endif

/* Host writes 1 after the rotor is spinning: ISR releases hall6 and
 * enters the accepted CURRENT baseline (DPWMMIN, id on, DT on,
 * FocThetaInterp, |vd| 2.4 V, vq 2.4 V). 3.6 V after interp is locked
 * is OK (2026-08-28); the old BKIN was Park sitting at −60°. Write 2
 * to freeze PWM; motor_off then clears MOE. */
extern volatile uint8_t g_motor_foc_handover;
/* Host may write while CURRENT is running. ISR clamps to 0..MOTOR_FOC_IQ_LSB
 * each tick (sign ignored). */
extern volatile int32_t g_motor_foc_iq_ref;
/* Speed-loop |iq| ceiling in LSB. apply_refs writes this; ISR clamps
 * to 0..MOTOR_FOC_IQ_LSB. Empty-load default is MOTOR_FOC_IQ_LSB. */
extern volatile int32_t g_motor_foc_iq_lim;
/* 0 = Hi-Z two-phase, 1 = DPWMMIN, 2 = midrail. Handover sets 1. */
extern volatile uint8_t g_motor_foc_leg3;
/* 1 = Park follows FocThetaInterp after the next hall edge. Handover
 * sets 1. Leave 0 only to compare discrete stairs. */
extern volatile uint8_t g_motor_foc_interp;
/* 1 = id PI on. Handover sets 1. */
extern volatile uint8_t g_motor_foc_id_on;
/* Host may write while CURRENT is running. ISR clamps to
 * ±MOTOR_FOC_IQ_LSB. Handover leaves 0 (id PI holds zero). Negative
 * is field weakening. Do not add this to g_motor_foc_fx — mode is
 * at byte +32 and the handover poll uses that offset. */
extern volatile int32_t g_motor_foc_id_ref;
/* vq ceiling in µV. Handover 2.4e6. ISR clamps 1.2e6..10e6.
 * 12 V linear is 6.58e6; 15 V linear is Vdc/√3 ≈ 8.66e6.
 * Six-step is ~2 Vdc/π and needs g_motor_foc_overmod=1. */
extern volatile int32_t g_motor_foc_vq_max_uv;
/* |vd| ceiling in µV. Handover 2.4e6. ISR clamps 1.2e6..2.4e6. */
extern volatile int32_t g_motor_foc_vd_max_uv;
/* Park rate limit, Q16 per tick. Default 600 = 3.30 deg/tick = 183
 * elec/s at 20 kHz (clears empty-load 5.0 V / 121 elec/s). 300 was
 * the 3.6 V follow rate and became the speed limit. ISR clamps 60..4000. */
extern volatile int32_t g_motor_foc_park_slew_q16;
/* Added to the Park angle, Q16. Default +7 deg (1274): nulls
 * empty-load |vd| after FocThetaInterp is sector-centered
 * (2026-09-13). +23 deg (4190) was the old interpolator-lag
 * cancel. −30 deg (−5461) sat +vd and punched the V driver.
 * Host may overwrite. ISR clamps ±16384. */
extern volatile int32_t g_motor_foc_park_off_q16;
/* 1 = speed PI writes iq_ref. Handover leaves 0 so the accepted
 * CURRENT baseline is unchanged (iq = MOTOR_FOC_IQ_LSB, vq pegs).
 * Empty-load free-run is voltage-limited; hold below that so vq
 * leaves the ceiling and the current loop can regulate. */
extern volatile uint8_t g_motor_foc_spd_on;
/* |electrical rev/s|. ISR clamps 0..260. Default Park 600 Q16 is
 * 183 elec/s — raise g_motor_foc_park_slew_q16 before asking above
 * that. Sign ignored; +iq still follows hall6. Wrap is the score. */
extern volatile int32_t g_motor_foc_w_ref_eps;
/* |electrical rev/s| from FocOmegaQ8, updated every CURRENT tick. */
extern volatile int32_t g_motor_foc_w_meas_eps;
/* 1 = skip the Vdc/√3 circle and clamp iPark to the hexagon
 * (span ≤ Vdc). Linear 6.58 V already holds 174 @ 175; this is
 * only past that. Hexagon sat rewinds Ki to the delivered vd/vq. */
extern volatile uint8_t g_motor_foc_overmod;

void MotorFocFx_Init(void);
void MotorFocFx_SetMode(uint8_t mode);
void MotorFocFx_SetIqRefLsb(int32_t lsb);
void MotorFocFx_ResetAcc(void);
void MotorFocFx_Handover(void);
void MotorFocFx_Step(void);

#endif /* MOTOR_FOC_FX_H */
