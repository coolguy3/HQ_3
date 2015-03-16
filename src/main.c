#include "common.h"
#include "gpio.h"
#include "uart.h"
#include "pit.h"
#include "IIC.h"
#include "IMU.h"
#include "CCD.h"

uint8_t Pixel[128] = {0};
uint8_t TIME1flag_20ms = 0 ;
uint8_t send_data_cnt = 0;

//**************1ms��ʱ�ж�*****************
static void PIT0_ISR(void)
{
//		static uint8_t IMU_3ms = 0;
//		IMU_3ms++;
//		if(IMU_3ms == 3)
//		{
//			IMU_3ms = 0;
//			IMU_Update();   //�� ����IMU_Report����ʱ��2~2.5ms֮��
//		}
}

//**************0.2ms��ʱ�ж�***************
static void PIT1_ISR(void)
{
  extern uint8_t IntegrationTime;       //�ع�ʱ��      
  static uint8_t TimerCnt20ms = 0;
  uint8_t integration_piont = 0;
   
  TimerCnt20ms++;

  //�����ع�ʱ������ع�㡢���ع��ִ��CCD��������
  integration_piont = 100 - IntegrationTime;    //100 * 0.2 = 20ms
  if(integration_piont >= 2) 
	{     
    if(integration_piont == TimerCnt20ms)
    StartIntegration();         	//��ʼ�ع�
  }

  if(TimerCnt20ms >= 100) 				//��ʱ20ms   100 * 0.2 = 20ms
	{
    TimerCnt20ms = 0;
    TIME1flag_20ms = 1;
		ImageCapture(Pixel);					//CCD��������,�������в������������ʱ�����SI��CLK����AD�ɼ�����������Pixel����
		CalculateIntegrationTime();		//���ݲɼ����AO(�ɱ�ʾ��ǿ)�����������ع�ʱ��
  }
	
}

int main(void)
{
	uint8_t i = 0;
	
	DelayInit();
	//***********��ʼ��LED**************************
	GPIO_QuickInit(HW_GPIOA, 17, kGPIO_Mode_OPP);		

	//***********��ʼ������**************************
	UART_QuickInit(UART0_RX_PA15_TX_PA14 , 115200); 			
	UART_EnableTxFIFO(HW_UART0, true);
	UART_SetTxFIFOWatermark(HW_UART0, UART_GetTxFIFOSize(HW_UART0));

	//***********��ʼ��ģ��IIC: SDA--E24 SCL--E25 �������ǡ����ٶȼ�******
//	IMU_Init();		

	//***********��ʼ��PITģ��***********************
	PIT_QuickInit(HW_PIT_CH0, 1*1000);				 //��ʱ1ms
	PIT_CallbackInstall(HW_PIT_CH0, PIT0_ISR); //PIT0_ISR���Զ����жϺ�����
	PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE);

	PIT_QuickInit(HW_PIT_CH1, 1*200);				 	 //��ʱ0.2ms
	PIT_CallbackInstall(HW_PIT_CH1, PIT1_ISR); 
	PIT_ITDMAConfig(HW_PIT_CH1, kPIT_IT_TOF,ENABLE);
	
	//***********��ʼ�� SI--E4 CLK--E5 ��ADC0_SE6B_PD5 8bits����**********
	CCD_Init(); 
	
	while(1)
	{
	//		DelayMs(500);
	//		GPIO_ToggleBit(HW_GPIOA, 17);	
		if(TIME1flag_20ms == 1)
		{
			TIME1flag_20ms = 0; 
			

			if(++send_data_cnt >= 5) 
			{
				send_data_cnt = 0;
				UART_WriteByte(HW_UART0, 0x02);		//֡ͷ
				UART_WriteByte(HW_UART0, 0xfd);
				for(i = 0;i < 128;i++)
				{
					UART_WriteByte(HW_UART0, Pixel[i] );
				}
				UART_WriteByte(HW_UART0, 0xfd);		//֡β
				UART_WriteByte(HW_UART0, 0x02);
				
			}
			
		} 
		
	}	//while end
	
}	//main end

