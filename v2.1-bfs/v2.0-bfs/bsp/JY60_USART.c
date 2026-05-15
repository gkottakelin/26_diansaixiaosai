#include "JY60_USART.h"
#include "wit_c_sdk.h"
#include <stdint.h>
#define UART2_BUF_SIZE 256
static volatile uint8_t  uart2_buf[UART2_BUF_SIZE];
static volatile uint16_t uart2_head = 0;  /* 中断写 */
static volatile uint16_t uart2_tail = 0;  /* 主循环读 */
// 发送函数（适配 UART_0）
void Uart2Send(uint8_t *p_data, uint32_t uiSize)
{   
    for(uint32_t i = 0; i < uiSize; i++)
    {
        // 阻塞发送单个字节
        DL_UART_Main_transmitDataBlocking(UART_2_INST, p_data[i]);
    }
}

// 串口初始化接口（配合 AutoScanSensor 使用）
void Uart2Init(unsigned int uiBaud)
{
    // 基础引脚和外设初始化已由 main.c 中的 SYSCFG_DL_init() 完成。
    // JY60 默认波特率通常是 9600，如果在 SysConfig 中固定了 9600 波特率，
    // 这里其实不需要任何操作。如果必须支持自动扫描动态修改波特率，
    // 需要调用 DL_UART_Main_setBaudRate()，但通常为了求稳建议固定 9600。
}

// UART2 接收中断服务函数
void UART_2_INST_IRQHandler(void)
{
    switch(DL_UART_getPendingInterrupt(UART_2_INST))
    {
        case DL_UART_IIDX_RX:
        case DL_UART_IIDX_RX_TIMEOUT_ERROR:
            while (DL_UART_isRXFIFOEmpty(UART_2_INST) == false)
            {
                uint8_t d = DL_UART_Main_receiveData(UART_2_INST);
                uint16_t next = (uart2_head + 1) % UART2_BUF_SIZE;
                if (next != uart2_tail)   /* buffer未满才存，满了直接丢弃 */
                {
                    uart2_buf[uart2_head] = d;
                    uart2_head = next;
                }
            }
            break;
 
        case DL_UART_IIDX_OVERRUN_ERROR:
        case DL_UART_IIDX_FRAMING_ERROR:
        case DL_UART_IIDX_PARITY_ERROR:
            DL_UART_clearInterruptStatus(UART_2_INST,
                DL_UART_INTERRUPT_OVERRUN_ERROR |
                DL_UART_INTERRUPT_FRAMING_ERROR |
                DL_UART_INTERRUPT_PARITY_ERROR);
            while (DL_UART_isRXFIFOEmpty(UART_2_INST) == false)
                DL_UART_Main_receiveData(UART_2_INST);
            break;
 
        default:
            break;
    }
}
void uart2_process(void)
{
    while (uart2_tail != uart2_head)
    {
        uint8_t d = uart2_buf[uart2_tail];
        uart2_tail = (uart2_tail + 1) % UART2_BUF_SIZE;
        WitSerialDataIn(d);   /* 库函数在主循环调用，不占中断时间 */
    }
}
 