#include "KEY.h"

void Key_Init(void)
{
	//���򿪹�	��������
	GPIO_QuickInit(HW_GPIOE, 8, kGPIO_Mode_IPU);	
	GPIO_QuickInit(HW_GPIOE, 9, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOE, 10, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOE, 11, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOE, 12, kGPIO_Mode_IPU);
	//���뿪��	��������
	GPIO_QuickInit(HW_GPIOD, 11, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOD, 12, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOD, 13, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOD, 14, kGPIO_Mode_IPU);
	//������		�������
	GPIO_QuickInit(HW_GPIOB, 22, kGPIO_Mode_OPP);
	GPIO_SetBit(HW_GPIOB, 22);			//�رշ�����
}

uint8_t Key_Scan(uint8_t mode)
{	 
	static uint8_t key_up=1;//�������ɿ���־
	if(mode)key_up=1;  //֧������		
	if( key_up && (PEin(8)==0||PEin(12)==0||PEin(11)==0||PEin(10)==0||PEin(9)==0) )
	{
		DelayMs(10);//ȥ���� 
		key_up=0;
		if(PEin(8)==0)return K1;					//��
		else if(PEin(12)==0)return K2;		//��
		else if(PEin(11)==0)return K3;		//��
		else if(PEin(10)==0)return K4;		//��
		else if(PEin(9)==0)return K5;			//��
	}
	else if( PEin(8)==1 && PEin(12)==1 && PEin(11)==1 && PEin(10)==1 && PEin(9)==1 )
		key_up=1;
 	return 5;// �ް�������
}


