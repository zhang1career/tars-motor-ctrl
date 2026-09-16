#ifndef UART_LOG_H
#define UART_LOG_H

#include <stdint.h>

void UartLog_Init(void);
void UartLog_Puts(const char *s);
void UartLog_PutU(uint32_t v);
void UartLog_PutI(int32_t v);
void UartLog_PutHex16(uint16_t v);
void UartLog_Nl(void);

#endif /* UART_LOG_H */
