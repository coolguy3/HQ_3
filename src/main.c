#include "common.h"			
#include "IMU.h"
#include "CCD.h"
#include "PID.h"
#include "KEY.h"
#include "UART_DMA.h"			//��UART_DMA.h��ע�ͻ������غ궨�� ��ѡ��Report������
#include "CTRL.h"
#include "OLED.h"

 								
int main(void)
{
	DelayInit();	
	FTM_PWM_Encoder_Init();
	Key_Init();
	DIP_Init();
	Bee_Init();
	Uart2_Init();
	CCD_Init();
	OLED_Init();
	MKP512FlashInit();		//�ڳ�ʼ���ж���FLASH����Ĳ���
	
	DelayMs(200);	//������̬ģ���ϵ�ʱ�䣬֮��ʼ��ʼ��IMU��������������ƫ����ʱ��ģ�豣�־�ֹ������	
	IMU_Init();		//��ʼ��ģ��IIC: SDA--C11 SCL--C10 ������ƫ������ֱ���ǶȺ�PID��������ʼ����ͨ����������ѭ��	

	PIT_Timer_Init();			//���г�ʼ�����ſ���ʱ�ж�
	GPIO_ToggleBit(HW_GPIOB, 22);	DelayMs(100);	GPIO_ToggleBit(HW_GPIOB, 22);		//������ʾ���ʼ�����
	
	while(1)
	{
		OLED_UI();
		
		#ifdef __UART_DMA_CCD_Report__
		CCD_Report();
		#endif
		
	}	//while end
	
}	//main end

