/******��ͷ�ļ�*********/
#include "common.h"
#include "gpio.h"
#include "uart.h"
#include "pit.h"
#include "ftm.h"
#include "dma.h"
/******�Զ���ͷ�ļ�*****/
#include "IIC.h"
#include "IMU.h"
#include "CCD.h"
#include "PID.h"
#include "KEY.h"
#include "UART_DMA.h"
#include "CTRL.h"

#define SPEED_CTRL_PEROID		75	//ms
extern struct Quad_PID PID_Stand;
/**************1ms��ʱ�ж�*****************/
static void PIT0_ISR(void)
{
		static uint8_t Count_Flag = 0;		//5ms��־λ	
		static uint8_t Speed_Flag = 0;
	
		Count_Flag++;
	
		if(Count_Flag == 0)
		{
			Speed_Flag++;
			if(Speed_Flag == 4)
			{
				Speed_Flag = 0;
				Speed_Measure();	//25msִ��һ��
			}		
		}
		if(Count_Flag == 1)
		{

		}
		if(Count_Flag == 2)		//ֱ��	5ms����		5ms����
		{
			IMU_Update();   //����̬���������˲�������Ƕ�
			PID_Stand_Update();
			Motor_Set();
		}
		if(Count_Flag == 3)		//�ٶ�	75ms����	5ms����
		{
			
		}
		if(Count_Flag == 4)		//����	10ms����	5ms����
		{
			Count_Flag = 0;
		}

}

/**************0.2ms��ʱ�ж�***************/
static void PIT1_ISR(void)
{
  extern uint8_t IntegrationTime;       //�ع�ʱ��  
	extern uint8_t UART_Buffer_CCD[132];	//��Ųɼ�ADֵ
	extern uint8_t TIME1flag_20ms;	//20ms��־λ
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
		ImageCapture(UART_Buffer_CCD + 2);					//CCD��������,�������в������������ʱ�����SI��CLK����AD�ɼ�����������Pixel����
		CalculateIntegrationTime();		//���ݲɼ����AO(�ɱ�ʾ��ǿ)�����������ع�ʱ��
  }
	
}

int main(void)
{
	enum Key { Key_Up,Key_Down,Key_Left,Key_Right,Key_Enter,No_Key	} Key;
	int16_t Duty1 = 5000 , Duty2 = 5000;
//	float Kp,Kd;
	
	DelayInit();
	
	/***********��ʼ��PTM0������·PWM:���1 PA6  ���2 PA7*******/
	FTM_PWM_QuickInit(FTM0_CH3_PA06, kPWM_EdgeAligned, 5000);		//	���ֵ�λHz���������ʼ��ռ�ձ�Ϊ50%
	FTM_PWM_QuickInit(FTM0_CH4_PA07, kPWM_EdgeAligned, 5000);	

	/***********��ʼ������������������*******/
	FTM_QD_QuickInit(FTM1_QD_PHA_PA12_PHB_PA13, kFTM_QD_NormalPolarity, kQD_PHABEncoding);	
	FTM_QD_QuickInit(FTM2_QD_PHA_PA10_PHB_PA11, kFTM_QD_NormalPolarity, kQD_PHABEncoding);
		
	/***********��ʼ�������ͷ�����*****************/
	Key_Init();
	
	/***********��ʼ��UART_Tx_DMA*************************/
	UART_QuickInit(UART2_RX_PD02_TX_PD03 , 115200); 			
//	UART_EnableTxFIFO(HW_UART2, true);
//	UART_SetTxFIFOWatermark(HW_UART2, UART_GetTxFIFOSize(HW_UART2));
	UART_ITDMAConfig(HW_UART2, kUART_DMA_Tx, true);
	UART_DMASendInit(HW_UART2, DMA_SEND_CH, NULL);
		
	/**********��������һ��ʾ��Ҫ��ʼ��ʼ��IMU��������������ƫ����ģ�豣�־�ֹ������**********/
	GPIO_ToggleBit(HW_GPIOB, 22);
	DelayMs(200);
	GPIO_ToggleBit(HW_GPIOB, 22);		
		
	/***********��ʼ��ģ��IIC: SDA--C11 SCL--C10 ������ƫ������ֱ���ǶȺ�PID����********/
	IMU_Init();		//��ʼ����ͨ����������ѭ��	
	pidSetTarget(&PID_Stand,43.0);	//���ó�ֱ�������
	pidSetKp(&PID_Stand, 73);			
	pidSetKd(&PID_Stand, 0.6);
	
	/***********��ʼ�� SI--C4 CLK--C5 ��ADC1_SE4B_PC8��8bits����************************/
	CCD_Init(); 

	/***********��ʼ��PIT��ʱģ��***********************/
	PIT_QuickInit(HW_PIT_CH0, 1*1000);					//��ʱ1ms
	PIT_CallbackInstall(HW_PIT_CH0, PIT0_ISR);	//PIT0_ISR���Զ����жϺ�����
	PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE);

	PIT_QuickInit(HW_PIT_CH1, 0.2*1000);				//��ʱ0.2ms
	PIT_CallbackInstall(HW_PIT_CH1, PIT1_ISR); 
	PIT_ITDMAConfig(HW_PIT_CH1, kPIT_IT_TOF,ENABLE);

	/**********�������ڶ���ʾ���ʼ�����****************/
	GPIO_ToggleBit(HW_GPIOB, 22);
	DelayMs(100);
	GPIO_ToggleBit(HW_GPIOB, 22);		

	while(1)
	{
				
		Key = (enum Key)Key_Scan();
		switch(Key)
		{
			case Key_Up			:	Duty1 += 100;	
												if(Duty1 > 10000)	Duty1 = 10000;
												FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3 , Duty1);		break;	// 0-10000 ��Ӧռ�ձ� 0-100%
			case Key_Down		: Duty1 -= 100;
												if(Duty1 < 0)			Duty1 = 0;			
												FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3 , Duty1);		break;	//����
			case Key_Left		: Duty2 += 100;	
												if(Duty2 > 10000)	Duty2 = 10000;
												FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4 , Duty2);		break;	//����
			case Key_Right	: Duty2 -= 100;	
												if(Duty2 < 0)			Duty2 = 0;			
												FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4 , Duty2);		break;
			case Key_Enter	: 		 break;
			default :  break;	//DelayMs(10);	
		}
		
	//	CCD_Report();

	}	//while end
	
}	//main end

