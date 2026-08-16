#ifndef STM32F0XX_IT_H
#define STM32F0XX_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void SysTick_Handler(void);
void TIM1_BRK_UP_TRG_COM_IRQHandler(void);
void TIM3_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32F0XX_IT_H */
