#ifndef __JY60_H_
#define __JY60_H_

#include "board.h"
#include "JY60_USART.h"
#include "wit_c_sdk.h"
#include "OLED.h"

#define ACC_UPDATE		0x01    // <--- ??????
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

extern float fAcc[3], fGyro[3], fAngle[3];
static void CmdProcess(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
void JY60_Init();
void JY60_GetData();
#endif
// ... ????