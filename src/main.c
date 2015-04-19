#include "common.h"			
#include "IMU.h"
#include "CCD.h"
#include "PID.h"
#include "KEY.h"
#include "UART_DMA.h"			//在UART_DMA.h中注释或添加相关宏定义 来选择Report的曲线
#include "CTRL.h"
#include "OLED.h"

 	//1.速度PID改成位置式方便积分分离和积分限幅 2.CCD曝光时间调整为1ms-15ms 3.添加转向PID，15ms计算一次，5ms输出控制一次。(还没用过，可能有误)
	//4 调参数前添加电机测试，方向键调速，Enter键退出   5.电机PWM频率改为13K(左轮5900，右轮5700才开始转) 
	//	6. 调直立静止的PID和电机死区电压有很大关系	7.目前PID：直立P：190左右 D：20左右    速度P：100以上 i： 10以下  D：10以下
	//8.用OLED更新参数同时RESET其他值，以免受之前值得影响（比如积分是一直累加的）
	
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
	
	Motor_Test();			//电机调试程序，方向键调速，Enter键退出  
	DelayMs(200);			//延时防止Enter键连续触发
	
	while(1)
	{
		OLED_UI();		//调PID参数界面

	}	//while end
	
}	//main end

