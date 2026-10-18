#ifndef MOTOR_ADC_H
#define MOTOR_ADC_H

#include <stdint.h>

/*
 * Phase current + bus voltage sampling, synchronised to the PWM.
 *
 * Why synchronisation is the whole point: the shunts are on the low side, so a
 * phase current only appears across its shunt while that phase's low-side FET
 * conducts. Sampling at an arbitrary instant returns a mixture of "conducting"
 * and "not conducting" -- the unsynchronised probe used during bring-up read a
 * 341 mA winding as anything between -339 mA and 0.
 *
 * Centre-aligned counting makes this easy: at the counter peak every CCRx is
 * below CNT, so all three low sides conduct at once. That is the sampling
 * instant, and it is also the middle of the widest conduction window available.
 *
 * Trigger chain: TIM3 counts one PWM period, edge-aligned, and OC1REF rises
 * MOTOR_ADC_LEAD_COUNTS before the TIM1 peak. TIM3 TRGO is that edge. TIM1
 * CH4 is the boost switch on PA11, so it is not the ADC trigger. CC1E stays
 * off and PB4/PB5 stay GPIO. DMA TC still runs the control ISR on the four
 * samples.
 *
 * The conversions are sequential (F030 has one ADC and no injected group), so
 * the three phases are sampled 1.67 us apart. With L/R = 673 us the current
 * moves ~23 mA in that time, comparable to the noise floor.
 */

#define MOTOR_ADC_CH_COUNT 4U

enum
{
  MOTOR_ADC_IU = 0U,   /* PA1, ADC_IN1 */
  MOTOR_ADC_IV = 1U,   /* PA2, ADC_IN2 */
  MOTOR_ADC_IW = 2U,   /* PA3, ADC_IN3 */
  MOTOR_ADC_VBUS = 3U  /* PA4, ADC_IN4. Sample kept; see MotorAdc_VbusMv. */
};

/*
 * PA4 is the boost feedback, R30 18k / R31 1k. The motor law does not
 * treat that code as vdc. MOTOR_VBUS_SENSE stays 0 and PWM scaling uses
 * MOTOR_VBUS_FIXED_MV. The conversion stays in the scan so the current
 * sample window does not move; the boost loop reads the same sample.
 *
 * While sense is off, PWM scaling uses MOTOR_VBUS_FIXED_MV.
 */
#ifndef MOTOR_VBUS_SENSE
#define MOTOR_VBUS_SENSE 0
#endif
#ifndef MOTOR_VBUS_FIXED_MV
#define MOTOR_VBUS_FIXED_MV 24000
#endif

/* Counts before the peak at which the sequence starts. 48 MHz timer clock.
 * Four conversions take 6.7 us. At vq 5.0 V the low-side window still
 * fits a 168-count (3.5 us) lead. At vq 6.2 V the window shrinks to
 * ~99 counts, so 168 starts before all three lows conduct and i0 walks
 * to −27 mA (2026-09-13). 72 counts = 1.5 us sits inside that window.
 *
 * Empty-load i0 (iu+iv+iw)/3, id_ref 0:
 *   vq 5.0 V (2026-09-11)
 *     lead   120    168    176    224    336    504
 *     i0    +2.7   +1.6   +1.5   +0.5  -35.7  -76.0  mA
 *   vq 6.2 V dir0 (2026-09-13)
 *     lead    36     48     72     96    144    168
 *     i0   -11.9   -9.9   -9.0  -12.7  -23.1  -27.3  mA
 * Floor ~−9 mA at 48–72 is leftover stagger/offset, not the knee. */
#ifndef MOTOR_ADC_LEAD_COUNTS
#define MOTOR_ADC_LEAD_COUNTS 72U
#endif

/* DMA target. Index order follows the channel numbers because the F030 scans
 * them in ascending order. */
extern volatile uint16_t g_motor_adc_raw[MOTOR_ADC_CH_COUNT];

void MotorAdc_Init(void);
int  MotorAdc_Start(void);
void MotorAdc_Stop(void);
/* Pause the FOC sequence, convert VREFINT + TEMP, restore TRGO DMA.
 * Returns 1 if both samples are non-zero. Task only; ~1 ms. */
int  MotorAdc_PollRails(uint16_t *vref_now, uint16_t *ts_raw);
/* Last PA0 / PA5 codes from PollRails. 0 if never sampled. */
uint16_t MotorAdc_NtcRaw(void);
uint16_t MotorAdc_VboostRaw(void);

/* DMA TC is the control-tick source (MotorTick_Start / Stop). */
void MotorAdc_EnableTick(void);
void MotorAdc_DisableTick(void);
void MotorAdc_DmaIrq(void);

/* Zero-current code is 2048 by construction: the INA240 mid-rail reference is
 * divided from the same +3V3A that is the ADC reference, so it tracks. Do not
 * rescale it with a measured VDDA. */
static inline int16_t MotorAdc_ShuntLsb(uint8_t idx)
{
  return (int16_t)((int32_t)g_motor_adc_raw[idx] - 2048);
}

/* Millivolts the modulator should treat as DC bus.
 * Sense on: 3.9639 mV/LSB * 1024 = 4058, R30/R31 = 4.9, result in mV.
 * Sense off: the fixed power-stage voltage, not the PA4 code. */
static inline int32_t MotorAdc_VbusMv(void)
{
#if MOTOR_VBUS_SENSE
  int32_t vmv = ((int32_t)g_motor_adc_raw[MOTOR_ADC_VBUS] * 4058) >> 10;

  if (vmv < 0)
  {
    vmv = 0;
  }
  if (vmv > 65535)
  {
    vmv = 65535;
  }
  return vmv;
#else
  return (int32_t)MOTOR_VBUS_FIXED_MV;
#endif
}

#endif /* MOTOR_ADC_H */
