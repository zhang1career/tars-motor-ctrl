#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"

void Error_Handler(void);
void MotorClock_Retry(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
