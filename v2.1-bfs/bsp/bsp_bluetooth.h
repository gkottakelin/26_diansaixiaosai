#ifndef __BSP_BLUETOOTH_H
#define __BSP_BLUETOOTH_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* 全局变量声明 */
extern volatile char Bluetooth_RxPacket[100]; // 接收数据包缓冲区，数据包格式 "@MSG\r\n"
extern volatile uint8_t Bluetooth_RxFlag;     // 接收完成标志位

/* 蓝牙发送函数声明 */
void Bluetooth_SendChar(char ch);
void Bluetooth_SendString(char* str);

#endif /* BLUETOOTH_H */