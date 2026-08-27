#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include <stdint.h>

/*
 * Hall-sensored field-oriented control, single precision.
 *
 * Structure follows sim/foc_controller_step.m steps 1, 3, 5, 6 and 8 -- three
 * shunt zero-sequence removal and Clarke, Park, two current PIs, inverse Park,
 * SVPWM. The sensorless flux observer, the PLL and the I/F startup (steps 2, 4
 * and 7 there) are gone: the rotor angle comes from motor_angle.c.
 *
 * Dropping the observer is what makes this fit at all. The generated float
 * controller measured 15000-18000 cycles on this M0 (roadmap 3.3.1), which does
 * not fit any usable loop rate; the observer and PLL are the bulk of it, and
 * they also need the PM flux linkage, which on this motor is an estimate rather
 * than a measurement.
 *
 * Float rather than fixed point on purpose: the measured headroom is 4237
 * cycles at 10 kHz (roadmap A2.1), enough for this, and the unknowns worth
 * spending debugging effort on are angle alignment and current calibration --
 * not Q-format scaling bugs, which look identical to angle errors from the
 * outside. Fixed point becomes worthwhile only if 20 kHz is needed later.
 */

enum
{
  MOTOR_FOC_OFF = 0U,
  /* Compute id/iq from the measured currents and the hall angle but drive
   * nothing. The motor runs on hall6 meanwhile, so angle alignment and current
   * calibration can be checked without the risk of a closed loop.
   * roadmap stage E step 1. */
  MOTOR_FOC_OBSERVE = 1U,
  /* Current loop closed, SVPWM on all three phases. */
  MOTOR_FOC_CURRENT = 2U
};

/* Control ticks per FOC step. The ISR runs at 20 kHz; 2 gives a 10 kHz loop,
 * which the cycle bench says fits with margin while 20 kHz does not. */
#ifndef MOTOR_FOC_DECIM
#define MOTOR_FOC_DECIM 2U
#endif

/*
 * Current loop bandwidth. Pole-zero cancellation tuning: Kp = wc*Ld and
 * Ki = wc*Rs place the PI zero on the motor pole.
 *
 * COUPLED TO THE LOOP RATE. Usable bandwidth is about f_s/10, so 10 kHz allows
 * about 1 kHz and 800 Hz leaves margin. mc_params.m uses 1200 Hz because it
 * assumes a 20 kHz loop; leaving 1200 Hz here while sampling at 10 kHz eats the
 * phase margin.
 */
#ifndef MOTOR_FOC_WC_HZ
#define MOTOR_FOC_WC_HZ 800.0f
#endif

/* Measured, not datasheet: 1.3 ohm and 874.7 uH phase-to-phase, halved. */
#define MOTOR_FOC_RS 0.65f
#define MOTOR_FOC_LD 437.3e-6f
#define MOTOR_FOC_LQ 437.3e-6f

/*
 * Back-EMF feed-forward gain, we * lambda. Left at zero because the PM flux
 * linkage in sim/mc_params.m (0.0085 Wb) is an estimate, and at 24 electrical
 * rev/s the term would be 1.28 V out of a 6.93 V ceiling -- too large to inject
 * on a guess. The integrator absorbs it instead. Set this once Ke is measured
 * in stage F.
 */
#ifndef MOTOR_FOC_LAMBDA
#define MOTOR_FOC_LAMBDA 0.0f
#endif

/* Peak phase current the loop is allowed to command. */
#ifndef MOTOR_FOC_IMAX
#define MOTOR_FOC_IMAX 2.0f
#endif

typedef struct
{
  /* Scaled integers so the trace and SWD reads stay cheap and unambiguous. */
  int16_t id_ma;
  int16_t iq_ma;
  int16_t vd_mv;
  int16_t vq_mv;
  int16_t iq_ref_ma;
  uint16_t theta;        /* copy of the angle used this step */
  uint16_t duty_q12[3];  /* applied duty, 4096 = 100% */
  uint32_t steps;
  uint8_t mode;
  uint8_t sat;           /* 1 when the voltage circle limit was active */
} motor_foc_snapshot_t;

extern volatile motor_foc_snapshot_t g_motor_foc;

void MotorFoc_Init(void);
void MotorFoc_SetMode(uint8_t mode);
void MotorFoc_SetIqRefMa(int16_t ma);

/*
 * One FOC step over the current globals.
 *
 * NOT for the control ISR: measured at 16173 cycles, against a 2400-cycle tick
 * (roadmap 3.3.2). A computation that long cannot sit in the ISR at any
 * decimation, because the tick it lands on overruns and the following ticks are
 * lost. The fixed-point implementation is what runs in the ISR.
 *
 * Call this from the background loop instead. In MOTOR_FOC_OBSERVE that is
 * exactly what stage E step 1 needs: hall6 keeps driving the motor while id/iq
 * are computed for inspection, and the sector rate is only 144 Hz, so the few
 * kHz the background loop manages is ample.
 */
void MotorFoc_Step(void);

#endif /* MOTOR_FOC_H */
