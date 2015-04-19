#include "common.h"			
#include "IMU.h"
#include "CCD.h"
#include "PID.h"
#include "KEY.h"
#include "UART_DMA.h"			//��UART_DMA.h��ע�ͻ������غ궨�� ��ѡ��Report������
#include "CTRL.h"
#include "OLED.h"

 	//1.�ٶ�PID�ĳ�λ��ʽ������ַ���ͻ����޷� 2.CCD�ع�ʱ�����Ϊ1ms-15ms 3.���ת��PID��15ms����һ�Σ�5ms�������һ�Ρ�(��û�ù�����������)
	//4 ������ǰ��ӵ�����ԣ���������٣�Enter���˳�   5.���PWMƵ�ʸ�Ϊ13K(����5900������5700�ſ�ʼת) 
	//	6. ��ֱ����ֹ��PID�͵��������ѹ�кܴ��ϵ	7.ĿǰPID��ֱ��P��190���� D��20����    �ٶ�P��100���� i�� 10����  D��10����
	//8.��OLED���²���ͬʱRESET����ֵ��������֮ǰֵ��Ӱ�죨���������һֱ�ۼӵģ�
	
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
	
	Motor_Test();			//������Գ��򣬷�������٣�Enter���˳�  
	DelayMs(200);			//��ʱ��ֹEnter����������
	
	while(1)
	{
		OLED_UI();		//��PID��������

	}	//while end
	
}	//main end

