#include "CTRL.h"
#include "OLED.h"
#include "UART_DMA.h"

//因机械摩擦存在电机死区、宏定义轮子开始转的占空比、需实验测得    竞赛时都取5000

#define	MOTOT_DEADLINE_LEFT_POS	5100
#define	MOTOT_DEADLINE_LEFT_NEG	4900
#define	MOTOT_DEADLINE_RIGHT_POS	5100
#define	MOTOT_DEADLINE_RIGHT_NEG	4900
	

float Speed_Car = 0;	

//int Speed_Left_Filtered = 0,Speed_Right_Filtered = 0; 	//记录正交脉冲个数 
//static uint8_t sgn(int32_t a)
//{
//	if(a > 0) 
//		return 0;
//	else	if(a < 0)
//					return 1;
//				else
//					return 2;
//		
//}

void Speed_Measure()
{
	int32_t Speed_Left = 0,Speed_Right = 0; 	//记录正交脉冲个数 
  uint8_t Dir_Left = 0,Dir_Right = 0;		//记录编码器旋转方向1 
	
//	static int32_t Speed_Left_Filter[10] = {0},Speed_Right_Filter[10] = {0};		//记录编码器旋转方向1 
//	int32_t Sum_Left = 0,Sum_Right = 0;
//	uint8_t  i = 0,Cnt_Left = 0,Cnt_Right = 0;
//	static uint8_t	num = 0 ;
	
	int32_t temp[3] = {0};
	
	FTM_QD_GetData(HW_FTM1, &Speed_Left, &Dir_Left);
	FTM_QD_GetData(HW_FTM2, &Speed_Right, &Dir_Right);
	
	if(Speed_Left > 50000)
	{
		Speed_Left += -65536;
	}
	if(Speed_Right > 50000)
	{
		Speed_Right = 65536 - Speed_Right;
	}
	else 
		Speed_Right = -Speed_Right;
	
	Speed_Car = (Speed_Left + Speed_Right) / 2.0 ;
	
//	//看过曲线，深度为3滤波效果不错		平均滤波正负速度会抵消		感觉不需要滤波、等跑起来再看
//	Speed_Left_Filter[num] = Speed_Left;
//	Speed_Right_Filter[num] = Speed_Right;
//	for(i=0;i<3;i++)
//	{
//		if( sgn(Speed_Left_Filter[i]) == sgn(Speed_Left) )
//		{
//			Cnt_Left++;
//			Sum_Left += Speed_Left_Filter[i];
//		}
//		if( sgn(Speed_Left_Filter[i]) == sgn(Speed_Right) )
//		{
//			Cnt_Right++;
//			Sum_Right += Speed_Right_Filter[i];
//		}
//	}
//	if(Speed_Left == 0)
//		Speed_Left_Filtered = 0;
//	else
//		Speed_Left_Filtered =  Sum_Left / Cnt_Left ;
//	if(Speed_Right == 0)
//		Speed_Right_Filtered = 0;
//	else
//		Speed_Right_Filtered = Sum_Right / Cnt_Right ;
//	num = (num + 1) % 3;
//	


//	temp[0] = Speed_Left;
//	temp[1] = Speed_Right;
//	temp[2] = Speed_Car;
//	UART_DMA_Array_Report(sizeof(temp) ,temp);
	
	FTM_QD_ClearCount(HW_FTM1); 	//如测量频率则需要定时清除Count值 ，定时获取数据 
	FTM_QD_ClearCount(HW_FTM2);
	
}	


void Motor_Set()
{
	extern float Ang;			//用于倾角异常停机
	extern struct Quad_PID PID_Stand , PID_Speed;
	int16_t Motor_Left = 0,Motor_Right = 0,Duty_Left = 0,Duty_Right = 0;
	
	PID_Speed.PID_out += PID_Speed.PID_Avg_out;
	
	Motor_Left = PID_Stand.PID_out - PID_Speed.PID_out; 
	Motor_Right = PID_Stand.PID_out - PID_Speed.PID_out;

	//左轮
	if( Motor_Left > 0)
	{
		Duty_Left = Motor_Left + MOTOT_DEADLINE_LEFT_POS;
		if( Duty_Left > 9900)
			Duty_Left = 9900;
	}
	else if( Motor_Left < 0)
	{
		Duty_Left = Motor_Left + MOTOT_DEADLINE_LEFT_NEG;
		if( Duty_Left < 100)
			Duty_Left = 100;
	}
	else
	{
		Duty_Left = 5000;
	}
	
	//右轮
	if( Motor_Right > 0)
	{
		Duty_Right = Motor_Right + MOTOT_DEADLINE_RIGHT_POS;
		if( Duty_Right > 9900)
			Duty_Right = 9900;
	}
	else if( Motor_Right < 0)
	{
		Duty_Right = Motor_Right + MOTOT_DEADLINE_RIGHT_NEG;
		if( Duty_Right < 100)
			Duty_Right = 100;
	}
	else
	{
		Duty_Right = 5000;
	}
	
	//车子角度失常停电机
	if( Ang > 75 || Ang < 25)
	{
		Duty_Left = 5000;
		Duty_Right = 5000;
	}
	
	FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4 , Duty_Left);
	FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3 , Duty_Right);
	
}

