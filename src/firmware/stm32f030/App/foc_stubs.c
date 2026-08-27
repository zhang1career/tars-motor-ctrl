#include "rtwtypes.h"

/*
 * Hall-assist hooks referenced by sim/codegen_stm32/foc_step_stm32.c.
 * Disabled for the stage-B timing bench (sensorless path inside codegen).
 */
volatile uint8_T g_tars_foc_hall_en = 0U;
volatile real32_T g_tars_foc_hall_theta = 0.0f;
volatile real32_T g_tars_foc_hall_w_est = 0.0f;
volatile real32_T g_tars_foc_iq_lim = 3.0f;
