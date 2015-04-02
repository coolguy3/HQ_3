#include "UART_DMA.h"


/* UART ģ�����ݼĴ��� */
static const void* UART_DataPortAddrTable[] = 
{
    (void*)&UART0->D,
    (void*)&UART1->D,
    (void*)&UART2->D,
    (void*)&UART3->D,
    (void*)&UART4->D,
    (void*)&UART5->D,    
};

/* UART ���ʹ���Դ��� */
static const uint32_t UART_SendDMATriggerSourceTable[] = 
{
    UART0_TRAN_DMAREQ,
    UART1_TRAN_DMAREQ,
    UART2_TRAN_DMAREQ,
    UART3_TRAN_DMAREQ,
    UART4_TRAN_DMAREQ,
    UART5_TRAN_DMAREQ,
};

/* UART ���մ���Դ��� */
static const uint32_t UART_RevDMATriggerSourceTable[] = 
{
    UART0_REV_DMAREQ,
    UART1_REV_DMAREQ,
    UART2_REV_DMAREQ,
    UART3_REV_DMAREQ,
    UART4_REV_DMAREQ,
    UART5_REV_DMAREQ,
};

/* DMA ���ڷ��� ���� */
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

/* DMA ���ڽ��� ���� */
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
    /* ��� Major Loop ��ֹͣ Request �����ȴ�DMAӲ������Դ���� */
    DMA_EnableAutoDisableRequest(dmaChl, false);
}

/* DMA ���ͺ��� */
uint32_t UART_SendWithDMA(uint32_t dmaChl, const uint8_t *buf, uint32_t size)
{
    DMA_SetSourceAddress(dmaChl, (uint32_t)buf);
    DMA_SetMajorLoopCounter(dmaChl, size);
    /* �������� */
    DMA_EnableRequest(dmaChl);
    return 0;
}



/*
*			���ܣ��ô���DMA����Float������(1~6��)����λ������ʾʵʱ����
*
*			������Float������ָ��
*/
void UART_DMA_Array_Width_Six(float * Array_Width_Six)
{
	uint8_t temp[28] = {0} , i = 0 , j = 0 , k = 2;
	temp[0] = 0x03;		temp[1] = 0xfc;			//֡ͷ
	temp[26] = 0xfc;	temp[27] = 0x03;  	//֡β

	for( i = 0 ; i < 6 ; i++ )
	{
		for( j = 0 ; j < 4 ; j++,k++ )
		{
			temp[k] = * ( (uint8_t *)(Array_Width_Six+i) + j );
		}
	}
	while(DMA_IsMajorLoopComplete(DMA_SEND_CH));
  UART_SendWithDMA(DMA_SEND_CH, (const uint8_t*)temp, sizeof(temp));		
	
}

