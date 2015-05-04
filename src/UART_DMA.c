#include "UART_DMA.h"


/* UART 模块数据寄存器 */
static const void* UART_DataPortAddrTable[] = 
{
    (void*)&UART0->D,
    (void*)&UART1->D,
    (void*)&UART2->D,
    (void*)&UART3->D,
    (void*)&UART4->D,
    (void*)&UART5->D,    
};

/* UART 发送触发源编号 */
static const uint32_t UART_SendDMATriggerSourceTable[] = 
{
    UART0_TRAN_DMAREQ,
    UART1_TRAN_DMAREQ,
    UART2_TRAN_DMAREQ,
    UART3_TRAN_DMAREQ,
    UART4_TRAN_DMAREQ,
    UART5_TRAN_DMAREQ,
};

/* UART 接收触发源编号 */
static const uint32_t UART_RevDMATriggerSourceTable[] = 
{
    UART0_REV_DMAREQ,
    UART1_REV_DMAREQ,
    UART2_REV_DMAREQ,
    UART3_REV_DMAREQ,
    UART4_REV_DMAREQ,
    UART5_REV_DMAREQ,
};

/* DMA 串口发送 配置 */
void UART_DMASendInit(uint32_t uartInstnace, uint8_t dmaChl, uint8_t * txBuf)
{
    DMA_InitTypeDef DMA_InitStruct1 = {0};
    DMA_InitStruct1.chl = dmaChl;
    DMA_InitStruct1.chlTriggerSource = UART_SendDMATriggerSourceTable[uartInstnace];
    DMA_InitStruct1.triggerSourceMode = kDMA_TriggerSource_Normal;
    DMA_InitStruct1.minorLoopByteCnt = 1;
    DMA_InitStruct1.majorLoopCnt = 1;
    
    DMA_InitStruct1.sAddr = (uint32_t)txBuf;
    DMA_InitStruct1.sLastAddrAdj = -1; 
    DMA_InitStruct1.sAddrOffset = 1;
    DMA_InitStruct1.sDataWidth = kDMA_DataWidthBit_8;
    DMA_InitStruct1.sMod = kDMA_ModuloDisable;
    
    DMA_InitStruct1.dAddr = (uint32_t)UART_DataPortAddrTable[uartInstnace]; 
    DMA_InitStruct1.dLastAddrAdj = 0;
    DMA_InitStruct1.dAddrOffset = 0;
    DMA_InitStruct1.dDataWidth = kDMA_DataWidthBit_8;
    DMA_InitStruct1.dMod = kDMA_ModuloDisable;
    DMA_Init(&DMA_InitStruct1);
    DMA_EnableRequest(dmaChl);
}

/* DMA 串口接收 配置 */
void UART_DMARevInit(uint32_t uartInstnace, uint8_t dmaChl, uint8_t * rxBuf)
{
    DMA_InitTypeDef DMA_InitStruct1 = {0};
    DMA_InitStruct1.chl = dmaChl;
    DMA_InitStruct1.chlTriggerSource = UART_RevDMATriggerSourceTable[uartInstnace];
    DMA_InitStruct1.triggerSourceMode = kDMA_TriggerSource_Normal;
    DMA_InitStruct1.minorLoopByteCnt = 1;
    DMA_InitStruct1.majorLoopCnt = 1;
    
    DMA_InitStruct1.sAddr = (uint32_t)&UART0->D;
    DMA_InitStruct1.sLastAddrAdj = 0; 
    DMA_InitStruct1.sAddrOffset = 0;
    DMA_InitStruct1.sDataWidth = kDMA_DataWidthBit_8;
    DMA_InitStruct1.sMod = kDMA_ModuloDisable;
    
    DMA_InitStruct1.dAddr = (uint32_t)rxBuf; 
    DMA_InitStruct1.dLastAddrAdj = 0;
    DMA_InitStruct1.dAddrOffset = 0;
    DMA_InitStruct1.dDataWidth = kDMA_DataWidthBit_8;
    DMA_InitStruct1.dMod = kDMA_ModuloDisable;
    DMA_Init(&DMA_InitStruct1); 
    /* 完成 Major Loop 后不停止 Request 继续等待DMA硬件触发源触发 */
    DMA_EnableAutoDisableRequest(dmaChl, false);
}

/* DMA 发送函数 */
uint32_t UART_SendWithDMA(uint32_t dmaChl, const uint8_t *buf, uint32_t size)
{
    DMA_SetSourceAddress(dmaChl, (uint32_t)buf);
    DMA_SetMajorLoopCounter(dmaChl, size);
    /* 启动传输 */
    DMA_EnableRequest(dmaChl);
    return 0;
}

void Uart2_Init(void)
{
	UART_QuickInit(UART2_RX_PD02_TX_PD03 , 115200); 			
//	UART_EnableTxFIFO(HW_UART2, true);
//	UART_SetTxFIFOWatermark(HW_UART2, UART_GetTxFIFOSize(HW_UART2));
	UART_ITDMAConfig(HW_UART2, kUART_DMA_Tx, true);
	UART_DMASendInit(HW_UART2, DMA_SEND_CH, NULL);
	
}

/*
*			功能：用串口DMA发送任意型数据(1~8个)到上位机，显示实时曲线(上位机选择波形数目要看传输的数组长度)
*
*			参数：数组字节数(用sizeof)、数组指针
*/
void UART_DMA_Array_Report(uint8_t cnt , void * Array_Width_Six)
{
	uint8_t temp[99] = {0}, i = 0 ;
	
	temp[0] = 0x03;		temp[1] = 0xfc;			//帧头
	for( i = 0 ; i < cnt ; i++ )
	{
		temp[i + 2] = * ( (uint8_t *)Array_Width_Six + i);
	}
	temp[cnt + 2] = 0xfc;	temp[cnt + 3] = 0x03;  	//帧尾
	
	while(DMA_IsMajorLoopComplete(DMA_SEND_CH)){;}
  UART_SendWithDMA(DMA_SEND_CH, (const uint8_t*)temp, cnt + 4);		
	
}

void CCD_Report(void)
{
		extern uint8_t UART_Buffer_CCD[132];
		//只要重复调用这个函数就可以发送不同的数组，但必须注意要确保上一次已经发送完
		while(DMA_IsMajorLoopComplete(DMA_SEND_CH));
		UART_SendWithDMA(DMA_SEND_CH, (const uint8_t*)UART_Buffer_CCD, sizeof(UART_Buffer_CCD));		

}

//测试用UART_DMA_CCD_Report发送一直有乱码出现
//void UART_DMA_CCD_Report(void)	//100ms
//{
//	extern uint8_t Pixel[];
//	uint8_t temp[132] = {0}, i = 0 ;
//	
//	temp[0] = 0x02;		temp[1] = 0xfd;			//帧头
//	for( i = 0 ; i < 128 ; i++ )
//	{
//		temp[i + 2] = * ( (uint8_t *)Pixel + i);
//	}
//	temp[128 + 2] = 0xfd;	temp[128 + 3] = 0x02;  	//帧尾
//	
//	while(DMA_IsMajorLoopComplete(DMA_SEND_CH)){;}
//  UART_SendWithDMA(DMA_SEND_CH, (const uint8_t*)temp, 128 + 4);

//}
