#ifndef TNB_NODE_H
#define TNB_NODE_H

#include <stdint.h>

typedef enum {
  TNB_MATCH_PRIMARY = 0,
  TNB_MATCH_GENERAL = 2
} tnb_match_t;

void     TnbNode_Init(void);
/* 在 MotorApp_Init 之前采一次内部温度 / VDDA。之后 ADC 归电流环。 */
void     TnbNode_SampleRails(void);
void     TnbNode_Task(void);
uint8_t  TnbNode_PrimaryAddr(void);
void     TnbNode_OnWrite(tnb_match_t kind, const uint8_t *data, uint16_t len);
uint16_t TnbNode_OnRead(tnb_match_t kind, uint8_t *buf, uint16_t cap);
void     TnbNode_NoteI2c(void);
uint8_t  TnbNode_I2cTimedOut(uint32_t timeout_ms);
void     TnbNode_NoteCrc(void);
uint8_t  TnbNode_RegPtr(void);
void     TnbNode_RefreshMot(void);
void     TnbNode_ApplyI2cAddress(void);

#endif /* TNB_NODE_H */
