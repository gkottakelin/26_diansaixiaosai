#ifndef __GWHD_H_
#define __GWHD_H_

#include "ti_msp_dl_config.h"
#include "OLED.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "board.h"
#include "Key.h"

static No_MCU_Sensor sensor;
static unsigned char rx_buff[256]={0};
static unsigned short Anolog[8]={0};
static unsigned short white[8]={3219,3212,3266,3143,3073,3219,3147,3071};
static unsigned short black[8]={2069,1646,1890,1578,1005,1946,1292,1263};
static unsigned short Normal[8];
static  uint8_t Digtal = 0b10101010; // 定义一个八位二进制数;

void GWHD_Init();
void GWHD_Jiaozhun();
void GWHD_Work();
uint16_t GWHD_AnalyzeData(uint16_t num);

#endif