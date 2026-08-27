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
 * Trigger chain: TIM1 CH4 in PWM2 mode produces exactly one OC4REF rising edge
 * per PWM period, as the counter passes CCR4 on the way up. CCR4 is placed
 * MOTOR_ADC_LEAD_US before the peak so the conversions finish by the time the
 * control ISR runs. OC4REF drives TRGO, which triggers the ADC. CC4E stays off,
 * so PA11 remains a plain GPIO held low for the charge pump.
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
  MOTOR_ADC_VBUS = 3U  /* PA4, ADC_IN4 */
};

/* Counts before the peak at which the sequence starts. 48 MHz timer clock, so
 * 336 counts = 7 us: four conversions at 1.67 us each finish just before the
 * peak. The low-side conduction half-window is (1236 - CCR) counts, so a 7 us
 * lead holds up to about 70% duty; above that this must shrink, at the cost of
 * the samples being taken further from the peak. */
#ifndef MOTOR_ADC_LEAD_COUNTS
#define MOTOR_ADC_LEAD_COUNTS 336U
#endif

/* DMA target. Index order follows the channel numbers because the F030 scans
 * them in ascending order. */
extern volatile uint16_t g_motor_adc_raw[MOTOR_ADC_CH_COUNT];

void MotorAdc_Init(void);
int  MotorAdc_Start(void);
void MotorAdc_Stop(void);

#if defined(MOTOR_ADC_STROBE) && (MOTOR_ADC_STROBE != 0)
/*
 * Pulses TP_CYCLE (PA15, TP1) when the conversion sequence completes, so a scope
 * can show where sampling lands relative to the low-side conduction window --
 * the one stage C acceptance item that cannot be checked from the target alone.
 * The sequence started MOTOR_ADC_CH_COUNT * 1.67 us before the pulse and the
 * counter peak is roughly MOTOR_ADC_LEAD_COUNTS/48 us after its start.
 *
 * Costs one interrupt per PWM period, so it is opt-in rather than always on.
 */
void MotorAdc_DmaIrq(void);
#endif

/* Zero-current code is 2048 by construction: the INA240 mid-rail reference is
 * divided from the same +3V3A that is the ADC reference, so it tracks. Do not
 * rescale it with a measured VDDA. */
static inline int16_t MotorAdc_ShuntLsb(uint8_t idx)
{
  return (int16_t)((int32_t)g_motor_adc_raw[idx] - 2048);
}

#endif /* MOTOR_ADC_H */
