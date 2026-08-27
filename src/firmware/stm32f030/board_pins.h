#ifndef BOARD_PINS_H
#define BOARD_PINS_H

/*
 * motor-ctrl mainboard v0.1 — STM32F030K6Tx (LQFP-32)
 * Pin map for firmware. Truth source is docs/mainboard-spec.md section 3;
 * keep both in sync with the KiCad schematic.
 */

/* ---- TIM1 half-bridge (3-phase complementary PWM) ---- */
#define BOARD_PWM_U_H_PIN      GPIO_PIN_8   /* PA8  TIM1_CH1  */
#define BOARD_PWM_U_H_PORT     GPIOA
#define BOARD_PWM_U_L_PIN      GPIO_PIN_7   /* PA7  TIM1_CH1N */
#define BOARD_PWM_U_L_PORT     GPIOA

#define BOARD_PWM_V_H_PIN      GPIO_PIN_9   /* PA9  TIM1_CH2  */
#define BOARD_PWM_V_H_PORT     GPIOA
#define BOARD_PWM_V_L_PIN      GPIO_PIN_0   /* PB0  TIM1_CH2N */
#define BOARD_PWM_V_L_PORT     GPIOB

#define BOARD_PWM_W_H_PIN      GPIO_PIN_10  /* PA10 TIM1_CH3  */
#define BOARD_PWM_W_H_PORT     GPIOA
#define BOARD_PWM_W_L_PIN      GPIO_PIN_1   /* PB1  TIM1_CH3N */
#define BOARD_PWM_W_L_PORT     GPIOB

/* ---- Fault inputs (LM339 open-drain wired-OR, 4k7 pull-up to +3V3) ---- */
#define BOARD_NFAULT_PIN       GPIO_PIN_6   /* PA6  TIM1_BKIN ← overcurrent window
                                             * comparator, via JP2. Low = fault. */
#define BOARD_NFAULT_PORT      GPIOA

#define BOARD_NOTEMP_PIN       GPIO_PIN_12  /* PA12 ← remote-NTC comparators, TP5.
                                             * Also OR'd into nFAULT through D5, so
                                             * this pin is what tells overtemp from
                                             * overcurrent apart. Low = overtemp. */
#define BOARD_NOTEMP_PORT      GPIOA

/* ---- Charge pump ----
 * PA11 drives Q3's base through R25 2k2, so a floating pin biases the pump
 * half-on. It must be an explicit push-pull output. Keep it LOW: the pump
 * would take V_BOOST to ~21 V, and with no HB-HS zener on the half-bridge
 * boards that overruns the UCC27211 20 V maximum at low duty
 * (mainboard-spec.md section 9.5.4).
 */
#define BOARD_CP_DRIVE_PIN     GPIO_PIN_11  /* PA11 TIM1_CH4 → CP_DRIVE, TP2 */
#define BOARD_CP_DRIVE_PORT    GPIOA

/* ---- ADC (12-bit, reference = VDDA) ---- */
#define BOARD_ADC_NTC_CH       ADC_CHANNEL_0   /* PA0 board NTC, 10k/10k divider */
#define BOARD_ADC_IU_CH        ADC_CHANNEL_1   /* PA1 ISENSE_U ← INA240_U.OUT */
#define BOARD_ADC_IV_CH        ADC_CHANNEL_2   /* PA2 ISENSE_V ← INA240_V.OUT */
#define BOARD_ADC_IW_CH        ADC_CHANNEL_3   /* PA3 ISENSE_W ← INA240_W.OUT */
#define BOARD_ADC_VBUS_CH      ADC_CHANNEL_4   /* PA4 VSENSE_12V, R30 39k / R31 10k */
#define BOARD_ADC_VBOOST_CH    ADC_CHANNEL_5   /* PA5 VSENSE_V_BOOST, R32 68k / R33 10k */

/* Current front end: INA240A2 (gain 50) across the 10 mOhm half-bridge shunt,
 * mid-rail referenced by dividing +3V3A internally.
 *
 * The reference and the ADC share VDDA, so the zero-current code is 2048
 * whatever VDDA does. Do not rescale it by a measured VDDA -- that would
 * reintroduce the error the ratiometric reference cancels. Measured 2026-08-27:
 * 2045 / 2048 / 2048, i.e. -4.9 / 0 / 0 mA of residual offset.
 */
#define BOARD_ISENSE_ZERO_CODE   2048
#define BOARD_ISENSE_GAIN        50
#define BOARD_SHUNT_MILLIOHM     10

/* Divider ratios as integer fractions, for scaling without floating point. */
#define BOARD_VBUS_DIV_NUM       49   /* (39k + 10k) / 10k */
#define BOARD_VBUS_DIV_DEN       10
#define BOARD_VBOOST_DIV_NUM     78   /* (68k + 10k) / 10k */
#define BOARD_VBOOST_DIV_DEN     10

/* ---- Hall (digital, 3.3 V): yellow / green / blue ---- */
#define BOARD_HALL_A_PIN       GPIO_PIN_3   /* PB3 — HALL_A (yellow Ha) */
#define BOARD_HALL_A_PORT      GPIOB
#define BOARD_HALL_B_PIN       GPIO_PIN_4   /* PB4 — HALL_B (green Hb) */
#define BOARD_HALL_B_PORT      GPIOB
#define BOARD_HALL_C_PIN       GPIO_PIN_5   /* PB5 blue Hc */
#define BOARD_HALL_C_PORT      GPIOB

/* ---- I2C1 (TNB / host) ---- */
#define BOARD_I2C_SCL_PIN      GPIO_PIN_6   /* PB6 */
#define BOARD_I2C_SCL_PORT     GPIOB
#define BOARD_I2C_SDA_PIN      GPIO_PIN_7   /* PB7 */
#define BOARD_I2C_SDA_PORT     GPIOB

/* ---- Debug ---- */
#define BOARD_SWDIO_PIN        GPIO_PIN_13  /* PA13 */
#define BOARD_SWCLK_PIN        GPIO_PIN_14  /* PA14 */
#define BOARD_TP_CYCLE_PIN     GPIO_PIN_15  /* PA15 — TP1, net TP_CYCLE (scope strobe) */
#define BOARD_TP_CYCLE_PORT    GPIOA

#endif /* BOARD_PINS_H */
