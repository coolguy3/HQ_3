#include "CTRL.h"

extern struct Quad_PID PID_Stand , PID_Speed , PID_Turn;
extern struct Parameter{
	float Stand_Kp , Stand_Kd ;
	float Speed_Kp , Speed_Ki , Speed_Kd;
	float Ang_Set , Speed_Set ;
}	Flash_Parameter;	

//因机械摩擦存在电机死区、宏定义轮子开始转的占空比、需实验测得    竞赛时都取5000 ???
#define	MOTOT_DEADLINE_LEFT_POS	5500			//加电机死区
#define	MOTOT_DEADLINE_LEFT_NEG	4500
#define	MOTOT_DEADLINE_RIGHT_POS	5400
#define	MOTOT_DEADLINE_RIGHT_NEG	4600

//#define	MOTOT_DEADLINE_LEFT_POS	5000		//不加电机死区
//#define	MOTOT_DEADLINE_LEFT_NEG	5000
//#define	MOTOT_DEADLINE_RIGHT_POS	5000
//#define	MOTOT_DEADLINE_RIGHT_NEG	5000
	
float Speed_Car = 0;	
//int Speed_Left_Filtered = 0,Speed_Right_Filtered = 0; 	//记录正交脉冲个数 
	
	
/**************1ms*****************/
static void PIT0_ISR(void)
{
		static uint8_t Count_Flag = 0;		//5ms标志位
		static uint8_t Speed_Flag = 0 , Speed_Ctrl_Flag = 0;
		static uint16_t Speed_Ctrl_Cnt;
		extern uint8_t IntegrationTime;       //曝光时间
		extern uint8_t UART_Buffer_CCD[132];						//存放采集AD值
		static uint8_t TimerCnt15ms = 0;
		uint8_t integration_piont = 0;
		extern uint8_t Motor_Set_Flag ;			//按Enter控制，是否开启电机控制

		TimerCnt15ms++;

		//根据曝光时间计算曝光点，执行CCD启动程序
		integration_piont = 15 - IntegrationTime;    //100 * 0.2 = 20ms     
		if(integration_piont == TimerCnt15ms)
			StartIntegration();         	//开始曝光
	
		if(Count_Flag == 1)
		{

		}
		
		if(Count_Flag == 2)		//直立	5ms计算	5ms控制
		{
			IMU_Update();   		//读姿态传感器、滤波、算角度
			PID_Stand_Update();
			if(Motor_Set_Flag)
			{
				Motor_Set();
				Speed_Ctrl_Cnt++;
				if(Speed_Ctrl_Cnt == 400)		//站两秒再跑
				{
					//改成跑起来的参数
					pidInit(&PID_Speed, Flash_Parameter.Speed_Kp , Flash_Parameter.Speed_Ki , Flash_Parameter.Speed_Kd ,Flash_Parameter.Speed_Set);
				}	
				if(Speed_Ctrl_Cnt > 500) Speed_Ctrl_Cnt = 500;
			}
		}
		
		if(Count_Flag == 3)		
		{

		}
		
		if(Count_Flag == 4)		//速度	75ms计算	5ms控制
		{	
			Speed_Flag++;
			if(Speed_Flag == 7)
			{
				Speed_Flag = 0;
				Speed_Measure();	
				PID_Speed_Update();
			}
			if(Speed_Flag == 15)
			{
				Speed_Flag = 0;
			}			
		}
		
		if(Count_Flag == 5)		//方向	20ms控计算	5ms控制
		{
			if(TimerCnt15ms >= 15) 				//定时15ms
			{
				TimerCnt15ms = 0;
				ImageCapture(UART_Buffer_CCD + 2);					//CCD采样程序,函数里有参数需调整
				CalculateIntegrationTime();		//根据采集的平均电压(代替光强)，反馈计算曝光时间
				Recognize_Road();
		//		PID_Turn_Update();
				#ifdef __UART_DMA_CCD_Report__
				CCD_Report();
				#endif
			}
			Count_Flag = 0;
		}
		
		Count_Flag++;

}


	
void PIT_Timer_Init()
{
	/***********初始化PIT定时模块***********************/
	PIT_QuickInit(HW_PIT_CH0, 1*1000);					//定时1ms
	PIT_CallbackInstall(HW_PIT_CH0, PIT0_ISR);	//PIT0_ISR是自定义中断函数名
	PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE);

}

void FTM_PWM_Encoder_Init()
{
	/***********初始化PTM0产生两路PWM********/
	FTM_PWM_QuickInit(FTM0_CH3_PA06, kPWM_EdgeAligned, 13000);		//	数字单位Hz、函数里初始化占空比为50%
	FTM_PWM_QuickInit(FTM0_CH4_PA07, kPWM_EdgeAligned, 13000);	

	/***********初始化编码器的正交解码*******/
	FTM_QD_QuickInit(FTM1_QD_PHA_PA12_PHB_PA13, kFTM_QD_NormalPolarity, kQD_PHABEncoding);	
	FTM_QD_QuickInit(FTM2_QD_PHA_PA10_PHB_PA11, kFTM_QD_NormalPolarity, kQD_PHABEncoding);

}

void Speed_Measure()
{
	int32_t Speed_Left = 0,Speed_Right = 0; 	//记录正交脉冲个数 
  uint8_t Dir_Left = 0,Dir_Right = 0;		//记录编码器旋转方向1 
	
//	static int32_t Speed_Left_Filter[10] = {0},Speed_Right_Filter[10] = {0};		//记录编码器旋转方向1 
//	int32_t Sum_Left = 0,Sum_Right = 0;
//	uint8_t  i = 0,Cnt_Left = 0,Cnt_Right = 0;
//	static uint8_t	num = 0 ;
	
	float temp[3] = {0};
	
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

	#ifdef __UART_DMA_Speed_Measure_Report__
	temp[0] = Speed_Left;
	temp[1] = Speed_Right;
	temp[2] = Speed_Car;
	UART_DMA_Array_Report(sizeof(temp) ,temp);
	#endif
	
	FTM_QD_ClearCount(HW_FTM1); 	//如测量频率则需要定时清除Count值 ，定时获取数据 
	FTM_QD_ClearCount(HW_FTM2);
	
}	

void Motor_Set()
{
	extern float Ang;			//用于倾角异常停机
	extern struct Quad_PID PID_Stand , PID_Speed , PID_Turn;
	extern uint8_t Motor_Set_Flag;
	int16_t Motor_Left = 0,Motor_Right = 0,Duty_Left = 0,Duty_Right = 0;
	
	PID_Speed.PID_out += PID_Speed.PID_Avg_out;
	PID_Turn.PID_out += PID_Turn.PID_Avg_out;
	
	Motor_Left = PID_Stand.PID_out - PID_Speed.PID_out + PID_Turn.PID_out; 
	Motor_Right = PID_Stand.PID_out - PID_Speed.PID_out - PID_Turn.PID_out;

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
		Motor_Set_Flag = 0;
	}
	
	FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4 , Duty_Left);
	FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3 , Duty_Right);
	
}

