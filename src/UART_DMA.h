#ifndef __UART_DMA_H__
#define __UART_DMA_H__

#include "common.h"
#include "uart.h"
#include "dma.h"

/* 定义使用的DMA通道号 */
#define DMA_SEND_CH             HW_DMA_CH1
#define DMA_REV_CH              HW_DMA_CH2


/******条件编译选择Report的曲线*****/
//	( 3 , float )	Ang_Acc ; Ang_Gyro ; Ang; 
//#define __UART_DMA_IMU_Report__

//	( 6 , float )	PID_Stand.target ; Ang ; -Gyro_v ; PID_Stand.outP ; PID_Stand.outD ; PID_Stand.PID_out/10.0
//#define __UART_DMA_PID_Stand_Report__	

//	( 3 , float )	Speed_Left ; Speed_Right ; Speed_Car
//#define __UART_DMA_Speed_Measure_Report__

//	( 6 , float )	PID_Speed.target ; Speed_Car ; PID_Speed.outP ; PID_Speed.outI ; PID_Speed.outD ; output
//#define __UART_DMA_PID_Speed_Report__

//	(	6 , float)	PID_Turn.error ; Deta_error ; PID_Turn.outP ; PID_Turn.outD ; output ; PID_Turn.PID_Avg_out
#define __UART_DMA_PID_Turn_Report__

//	CCD
#define __UART_DMA_CCD_Report__


void UART_DMASendInit(uint32_t uartInstnace, uint8_t dmaChl, uint8_t * txBuf);
void UART_DMARevInit(uint32_t uartInstnace, uint8_t dmaChl, uint8_t * rxBuf);
uint32_t UART_SendWithDMA(uint32_t dmaChl, const uint8_t *buf, uint32_t size);

void Uart2_Init(void);
void UART_DMA_Array_Report(uint8_t cnt , void * Array_Width_Six);
//void UART_DMA_CCD_Report(void);
void CCD_Report(void);

#endif
