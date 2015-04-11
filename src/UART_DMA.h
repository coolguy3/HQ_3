#ifndef __UART_DMA_H__
#define __UART_DMA_H__

#include "common.h"
#include "uart.h"
#include "dma.h"

/* ����ʹ�õ�DMAͨ���� */
#define DMA_SEND_CH             HW_DMA_CH1
#define DMA_REV_CH              HW_DMA_CH2




void UART_DMASendInit(uint32_t uartInstnace, uint8_t dmaChl, uint8_t * txBuf);
void UART_DMARevInit(uint32_t uartInstnace, uint8_t dmaChl, uint8_t * rxBuf);
uint32_t UART_SendWithDMA(uint32_t dmaChl, const uint8_t *buf, uint32_t size);
void UART_DMA_Array_Report(uint8_t cnt , void * Array_Width_Six);
void UART_DMA_CCD_Report(void);

#endif
