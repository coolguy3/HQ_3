#include "common.h"			
#include "IMU.h"
#include "CCD.h"
#include "PID.h"
#include "KEY.h"
#include "UART_DMA.h"			//在UART_DMA.h中注释或添加相关宏定义 来选择Report的曲线
#include "CTRL.h"
#include "OLED.h"

extern float Car_Status[6] ;	

	
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
	MKP512FlashInit();		//在初始化中读回FLASH保存的参数
	
	DelayMs(200);	//给足姿态模块上电时间，之后开始初始化IMU并计算陀螺仪零偏、此时车模需保持静止！！！	
	IMU_Init();		//初始化模拟IIC: SDA--C11 SCL--C10 、算零偏、设置直立角度和PID参数，初始化不通过会陷入死循环	

	PIT_Timer_Init();			//所有初始化完后才开定时中断
	GPIO_ToggleBit(HW_GPIOB, 22);	DelayMs(100);	GPIO_ToggleBit(HW_GPIOB, 22);		//蜂鸣器示意初始化完成
	
//	Motor_Test();			//电机调试程序，方向键调速，Enter键退出  
	
	while(1)
	{
		OLED_UI();		//调PID参数界面
		//UART_DMA_Array_Report(sizeof(Car_Status),Car_Status);
	}	//while end
	
}	//main end

