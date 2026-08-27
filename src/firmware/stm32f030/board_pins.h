#ifndef BOARD_PINS_H
#define BOARD_PINS_H

/*
 * motor-ctrl PCB — STM32F030K6Tx (LQFP-32)
 * Pin map for firmware; keep in sync with pcb/ schematic.
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

#define BOARD_PWM_BKIN_PIN     GPIO_PIN_6   /* PA6  TIM1_BKIN — floating, break unused */
#define BOARD_PWM_BKIN_PORT    GPIOA

#define BOARD_CHARGE_PUMP_PIN  GPIO_PIN_11  /* PA11 TIM1_CH4 — unused */
#define BOARD_CHARGE_PUMP_PORT GPIOA

#define BOARD_TIM1_ETR_PIN     GPIO_PIN_12  /* PA12 TIM1_ETR — unused */
#define BOARD_TIM1_ETR_PORT    GPIOA

/* ---- ADC (12-bit): temp + 3× low-side shunt to GND ---- */
#define BOARD_ADC_TEMP_CH      ADC_CHANNEL_0   /* PA0 unused (NTC later) */
#define BOARD_ADC_IU_CH        ADC_CHANNEL_1   /* PA1 board1 I_SENSE */
#define BOARD_ADC_IV_CH        ADC_CHANNEL_2   /* PA2 board2 I_SENSE */
#define BOARD_ADC_IW_CH        ADC_CHANNEL_3   /* PA3 board3 I_SENSE */

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
