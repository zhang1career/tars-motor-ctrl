#ifndef TNB_MOTOR_H
#define TNB_MOTOR_H

#include <stdint.h>

void     TnbMotor_Init(void);
void     TnbMotor_PollIsr(void);
void     TnbMotor_Task(void);
void     TnbMotor_RequestRun(void);
void     TnbMotor_RequestStop(void);
void     TnbMotor_ClearFault(void);
void     TnbMotor_SetMode(uint8_t mode);
void     TnbMotor_SetDir(uint8_t dir);
void     TnbMotor_SetIqMa(int16_t ma);
void     TnbMotor_SetWRefEps(int16_t eps);
uint8_t  TnbMotor_Mode(void);
uint8_t  TnbMotor_Dir(void);
int16_t  TnbMotor_IqRefMa(void);
int16_t  TnbMotor_WRefEps(void);
uint8_t  TnbMotor_State(void);
uint8_t  TnbMotor_Reversing(void);
uint16_t TnbMotor_Fault(void);
void     TnbMotor_Telemetry(uint16_t *theta_q16, int16_t *w_eps,
                            int16_t *iq_ma, int16_t *id_ma,
                            uint16_t *vbus_mv, uint8_t *hall);
int32_t  TnbMotor_ThetaAcc(void);
int16_t  TnbMotor_IqMaxMa(void);

#endif /* TNB_MOTOR_H */
