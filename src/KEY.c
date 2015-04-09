#include "KEY.h"

void Key_Init(void)
{
	//五向开关	上拉输入
	GPIO_QuickInit(HW_GPIOE, 8, kGPIO_Mode_IPU);	
	GPIO_QuickInit(HW_GPIOE, 9, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOE, 10, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOE, 11, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOE, 12, kGPIO_Mode_IPU);
	//拨码开关	上拉输入
	GPIO_QuickInit(HW_GPIOD, 11, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOD, 12, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOD, 13, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOD, 14, kGPIO_Mode_IPU);
	//蜂鸣器		推挽输出
	GPIO_QuickInit(HW_GPIOB, 22, kGPIO_Mode_OPP);
	GPIO_SetBit(HW_GPIOB, 22);			//关闭蜂鸣器
}

uint8_t Key_Scan(uint8_t mode)
{	 
	static uint8_t key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		
	if( key_up && (PEin(8)==0||PEin(12)==0||PEin(11)==0||PEin(10)==0||PEin(9)==0) )
	{
		DelayMs(10);//去抖动 
		key_up=0;
		if(PEin(8)==0)return K1;					//上
		else if(PEin(12)==0)return K2;		//下
		else if(PEin(11)==0)return K3;		//左
		else if(PEin(10)==0)return K4;		//右
		else if(PEin(9)==0)return K5;			//中
	}
	else if( PEin(8)==1 && PEin(12)==1 && PEin(11)==1 && PEin(10)==1 && PEin(9)==1 )
		key_up=1;
 	return 5;// 无按键按下
}


