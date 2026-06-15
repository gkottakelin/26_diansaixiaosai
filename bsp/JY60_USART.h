#ifndef __JY60_USART_H
#define __JY60_USART_H
#include "ti_msp_dl_config.h"
#define UART2_BUF_SIZE 256
static volatile uint8_t  uart2_buf[UART2_BUF_SIZE];
static volatile uint16_t uart2_head;  /* ÖÐ¶ÏÐ´ */
static volatile uint16_t uart2_tail;  /* Ö÷Ñ­»·¶Á */
void Uart2Init(unsigned int uiBaud);
void Uart2Send(unsigned char *p_data, unsigned int uiSize);
void uart2_process(void);
#endif