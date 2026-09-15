/*
 * foc_app.c  --  STM32F429 integration template for the generated FOC controller
 *
 * This is HAND-WRITTEN reference glue (a template), NOT generated code.  It
 * shows how to drive the portable controller produced by MATLAB/Embedded
 * Coder (codegen_stm32/foc_step_stm32.c) from a CubeMX/HAL project.
 *
 * Control timing:
 *   - TIM1 in center-aligned PWM mode at 20 kHz (Fpwm = mc_params.Fpwm).
 *   - ADC samples three phase-current shunts + DC-bus voltage, triggered by
 *     TIM1, converted by the time the update/EOC ISR fires.
 *   - foc_step_stm32() is called ONCE per PWM period and returns three duty
 *     cycles in [0,1] which are written to TIM1 CCR1..CCR3.
 *
 * Cortex-M4F: build with the hard-float FPU enabled so the single-precision
 * math in foc_step_stm32.c maps to VFP instructions:
 *   -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -O2 -ffast-math
 * and enable the FPU in SystemInit (CPACR: set CP10/CP11 full access).
 */

#include "main.h"                 /* CubeMX: hadc1/2, htim1 handles, etc. */
#include "foc_step_stm32.h"       /* generated controller entry point     */
#include "foc_step_stm32_initialize.h"

/* ------------------------------------------------------------------ */
/* Board / sensor scaling  --  EDIT THESE FOR YOUR POWER STAGE         */
/* ------------------------------------------------------------------ */
#define ADC_VREF        3.3f      /* ADC reference voltage [V]               */
#define ADC_FULL        4095.0f   /* 12-bit ADC full scale                   */

/* Phase-current shunt + amplifier chain:
 *   Vadc = Voffset + Iphase * Rshunt * Gain
 * Convert ADC counts -> amps.  Offset is the zero-current ADC code (~mid). */
#define ISHUNT_OHM      0.010f    /* shunt resistance [ohm]                  */
#define IAMP_GAIN       20.0f     /* current-sense amplifier gain            */
#define I_COUNTS_TO_A   (ADC_VREF / ADC_FULL / (ISHUNT_OHM * IAMP_GAIN))
static uint16_t ia_offset = 2048; /* calibrate at startup (zero current)     */
static uint16_t ib_offset = 2048;
static uint16_t ic_offset = 2048;

/* DC-bus divider:  Vdc = Vadc * (Rtop + Rbot) / Rbot                         */
#define VBUS_DIV        11.0f     /* e.g. Rtop=100k, Rbot=10k -> 11           */
#define V_COUNTS_TO_V   (ADC_VREF / ADC_FULL * VBUS_DIV)

/* ------------------------------------------------------------------ */
/* Application state                                                   */
/* ------------------------------------------------------------------ */
static volatile float g_speed_ref_rpm = 1500.0f;   /* set from UI/CAN/etc. */
static volatile float g_enable        = 1.0f;

/* telemetry (read from a low-priority loop / debugger) */
volatile float dbg_theta, dbg_speed, dbg_id, dbg_iq;

/* ------------------------------------------------------------------ */
/* One-time init  --  call after MX_*_Init(), before starting TIM1      */
/* ------------------------------------------------------------------ */
void foc_init(void)
{
    foc_step_stm32_initialize();      /* zeroes controller state          */

    /* TODO: calibrate current offsets here by averaging N ADC samples
     * with the inverter disabled (all low-side off or 50% duty, no torque). */

    /* Start PWM with 50% duty (zero phase voltage) and the ADC trigger. */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    /* enable TIM1 update interrupt -> HAL_TIM_PeriodElapsedCallback below */
    HAL_TIM_Base_Start_IT(&htim1);
}

/* ------------------------------------------------------------------ */
/* Current control ISR  --  runs at Fpwm = 20 kHz                       */
/*                                                                      */
/* Hook this to the TIM1 update event (or the ADC injected-EOC ISR).    */
/* Budget: at 20 kHz you have 50 us; this controller is ~1.5 KB and     */
/* runs in a few microseconds on the 180 MHz F429 with the FPU on.      */
/* ------------------------------------------------------------------ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1) return;

    /* 1) read raw conversions (CubeMX: place phase currents on injected
     *    channels triggered by TIM1, Vdc on a regular/3rd injected ch).    */
    uint16_t raw_ia = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint16_t raw_ib = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    uint16_t raw_ic = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
    uint16_t raw_vdc= (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

    /* 2) scale to physical units (single precision) */
    float ia  = ((float)raw_ia - (float)ia_offset) * I_COUNTS_TO_A;
    float ib  = ((float)raw_ib - (float)ib_offset) * I_COUNTS_TO_A;
    float ic  = ((float)raw_ic - (float)ic_offset) * I_COUNTS_TO_A;
    float vdc = (float)raw_vdc * V_COUNTS_TO_V;
    if (vdc < 6.0f) vdc = 24.0f;     /* guard before bus is up */

    /* 3) run one FOC tick (zero-sequence removal inside foc_step_stm32) */
    float da, db, dc, th, spd, id, iq;
    foc_step_stm32(ia, ib, ic, vdc, g_speed_ref_rpm, g_enable,
                   &da, &db, &dc, &th, &spd, &id, &iq);

    /* 4) write duties to the PWM compare registers.
     *    ARR is the timer auto-reload (period).  duty in [0,1].             */
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(da * (float)arr));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(db * (float)arr));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dc * (float)arr));

    /* 5) telemetry */
    dbg_theta = th; dbg_speed = spd; dbg_id = id; dbg_iq = iq;
}
