#include "bsp_bluetooth.h"


/* 变量定义 */
volatile char Bluetooth_RxPacket[100];
volatile uint8_t Bluetooth_RxFlag = 0;

/**
 * @brief 发送单个字符到蓝牙
 * @param ch 要发送的字符
 */
void Bluetooth_SendChar(char ch)
{
    // 等待UART空闲
    while(DL_UART_isBusy(UART_3_INST) == true);
    // 发送数据
    DL_UART_Main_transmitData(UART_3_INST, ch);
}

/**
 * @brief 发送字符串到蓝牙
 * @param str 要发送的字符串首地址
 */
void Bluetooth_SendString(char* str)
{
    while (*str != '\0')
    {
        Bluetooth_SendChar(*str++);
    }
}

/**
 * @brief 串口0中断服务函数（处理蓝牙接收）
 * 注意：如果你的 SysConfig 中断名称不是 UART_3_INST_IRQHandler，请根据实际情况修改
 */
//void UART_3_INST_IRQHandler(void)
//{
//    static uint8_t RxState = 0;     // 状态机：表示当前接收状态
//    static uint8_t pRxPacket = 0;   // 数组指针：表示当前接收位置

//    switch(DL_UART_getPendingInterrupt(UART_3_INST))
//    {
//        case DL_UART_IIDX_RX:
//        {
//            // 读取数据寄存器，获取接收到的数据字节
//            uint8_t RxData = DL_UART_Main_receiveData(UART_3_INST);            
//            
//            /* 使用状态机思路处理不同接收情况 */
//            
//            /* 当前状态为0，等待数据包头 '@' */
//            if (RxState == 0)
//            {
//                if (RxData == '@' && Bluetooth_RxFlag == 0) // 确认是包头，并且上一个数据包已处理完
//                {
//                    RxState = 1;       // 进入下一个状态
//                    pRxPacket = 0;     // 数据包位置归零
//                }
//            }
//            /* 当前状态为1，接收数据包内容，同时判断是否接收到了第一个包尾 '\r' */
//            else if (RxState == 1)
//            {
//                if (RxData == '\r')    // 接收到第一个包尾
//                {
//                    RxState = 2;       // 进入下一个状态
//                }
//                else                   // 接收到普通数据
//                {
//                    Bluetooth_RxPacket[pRxPacket] = RxData; // 将数据存入数据包缓冲区指定位置
//                    pRxPacket++;       // 数据包位置递增
//                }
//            }
//            /* 当前状态为2，等待接收第二个包尾 '\n' */
//            else if (RxState == 2)
//            {
//                if (RxData == '\n')    // 接收到第二个包尾
//                {
//                    RxState = 0;       // 状态归零，准备接收下一个包
//                    Bluetooth_RxPacket[pRxPacket] = '\0'; // 接收到的字符串末尾添加结束符
//                    Bluetooth_RxFlag = 1; // 数据包标志位置1，表示成功接收一个完整的数据包
//                }
//            }
//            break;
//        }
//        default:
//            break;
//    }
//}
/*  
#include "ti_msp_dl_config.h"
#include "board.h"
#include "Key.h"
#include "bluetooth.h"  // 引入蓝牙模块头文件

int main(void)
{
    SYSCFG_DL_init();
    
    NVIC_ClearPendingIRQ(UART_3_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);

    // 测试发送功能
    Bluetooth_SendString("Bluetooth Init OK!\r\n");

    while (1)
    {
        if (Bluetooth_RxFlag == 1) // 如果接收到完整的蓝牙数据包
        {
            // 解析数据：假设发来的指令是 "1" 就亮灯，否则灭灯
            if(Bluetooth_RxPacket[0] == '1') {
                DL_GPIO_setPins(LED_PORT, LED_LED1_PIN);
                Bluetooth_SendString("LED ON\r\n"); // 回复状态
            } else {
                DL_GPIO_clearPins(LED_PORT, LED_LED1_PIN);
                Bluetooth_SendString("LED OFF\r\n");
            }
            
            Bluetooth_RxFlag = 0; // 处理完成后，务必将标志位清零，否则无法接收后续数据包
        }
    }
}
*/