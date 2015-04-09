/******库头文件*********/
#include "common.h"
#include "gpio.h"
#include "uart.h"
#include "pit.h"
#include "ftm.h"
#include "dma.h"
/******自定义头文件*****/
#include "IIC.h"
#include "IMU.h"
#include "CCD.h"
#include "PID.h"
#include "KEY.h"
#include "UART_DMA.h"
#include "CTRL.h"
#include "OLED.h"

#define SPEED_CTRL_PEROID		75	//ms
extern struct Quad_PID PID_Stand;
uint8_t Motor_Set_Flag = 0;			//按键Enter键控制，是否开启电机控制
/**************1ms定时中断*****************/
static void PIT0_ISR(void)
{
		static uint8_t Count_Flag = 0;		//5ms标志位	
		static uint8_t Speed_Flag = 0;
	
		Count_Flag++;
	
		if(Count_Flag == 0)
		{
			Speed_Flag++;
			if(Speed_Flag == 4)
			{
				Speed_Flag = 0;
				Speed_Measure();	//25ms执行一次
			}		
		}
		if(Count_Flag == 1)
		{

		}
		if(Count_Flag == 2)		//直立	5ms计算		5ms控制
		{
			IMU_Update();   //读姿态传感器、滤波、算出角度
			PID_Stand_Update();
			if(Motor_Set_Flag)
			Motor_Set();
		}
		if(Count_Flag == 3)		//速度	75ms计算	5ms控制
		{
			
		}
		if(Count_Flag == 4)		//方向	10ms计算	5ms控制
		{
			Count_Flag = 0;
		}

}

/**************0.2ms定时中断***************/
static void PIT1_ISR(void)
{
  extern uint8_t IntegrationTime;       //曝光时间  
	extern uint8_t UART_Buffer_CCD[132];	//存放采集AD值
	extern uint8_t TIME1flag_20ms;	//20ms标志位
  static uint8_t TimerCnt20ms = 0;
  uint8_t integration_piont = 0;
   
  TimerCnt20ms++;

  //根据曝光时间计算曝光点、在曝光点执行CCD启动程序
  integration_piont = 100 - IntegrationTime;    //100 * 0.2 = 20ms
  if(integration_piont >= 2) 
	{     
    if(integration_piont == TimerCnt20ms)
    StartIntegration();         	//开始曝光
  }

  if(TimerCnt20ms >= 100) 				//定时20ms   100 * 0.2 = 20ms
	{
    TimerCnt20ms = 0;
    TIME1flag_20ms = 1;
		ImageCapture(UART_Buffer_CCD + 2);					//CCD采样程序,函数里有参数需调整，按时序控制SI、CLK进行AD采集，结果存放在Pixel数组
		CalculateIntegrationTime();		//根据采集输出AO(可表示光强)，反馈计算曝光时间
  }
	
}

int main(void)
{
	enum Key { Key_Up,Key_Down,Key_Left,Key_Right,Key_Enter,No_Key	} Key;
	int16_t Duty1 = 5000 , Duty2 = 5000;

	static struct Parameter
	{
		float Stand_Kp,Stand_Kd;
	}Flash_Parameter;
	
	DelayInit();
	
	/***********初始化PTM0产生两路PWM:电机1 PA6  电机2 PA7*******/
	FTM_PWM_QuickInit(FTM0_CH3_PA06, kPWM_EdgeAligned, 5000);		//	数字单位Hz、函数里初始化占空比为50%
	FTM_PWM_QuickInit(FTM0_CH4_PA07, kPWM_EdgeAligned, 5000);	

	/***********初始化编码器的正交解码*******/
	FTM_QD_QuickInit(FTM1_QD_PHA_PA12_PHB_PA13, kFTM_QD_NormalPolarity, kQD_PHABEncoding);	
	FTM_QD_QuickInit(FTM2_QD_PHA_PA10_PHB_PA11, kFTM_QD_NormalPolarity, kQD_PHABEncoding);
		
	/***********初始化按键和蜂鸣器*****************/
	Key_Init();
	
	/***********初始化UART_Tx_DMA*************************/
	UART_QuickInit(UART2_RX_PD02_TX_PD03 , 115200); 			
//	UART_EnableTxFIFO(HW_UART2, true);
//	UART_SetTxFIFOWatermark(HW_UART2, UART_GetTxFIFOSize(HW_UART2));
	UART_ITDMAConfig(HW_UART2, kUART_DMA_Tx, true);
	UART_DMASendInit(HW_UART2, DMA_SEND_CH, NULL);
					
	/***********初始化 SI--C4 CLK--C5 、ADC1_SE4B_PC8、8bits精度************************/
	CCD_Init();
	
	/***********初始化OLED和片内Flash**********************/
	OLED_Init();
	MKP512FlashInit();
	Flash_Parameter = * ( (struct Parameter *)TEST_ADDR_BEIGN );
	
	/**********蜂鸣器第一声示意要开始初始化IMU并计算陀螺仪零偏、车模需保持静止！！！**********/
	GPIO_ToggleBit(HW_GPIOB, 22);
	DelayMs(200);
	GPIO_ToggleBit(HW_GPIOB, 22);		

	/***********初始化模拟IIC: SDA--C11 SCL--C10 、算零偏、设置直立角度和PID参数********/
	IMU_Init();		//初始化不通过会陷入死循环	
	pidSetTarget(&PID_Stand,43.0);	//设置车直立的倾角

	/***********初始化PIT定时模块***********************/
	PIT_QuickInit(HW_PIT_CH0, 1*1000);					//定时1ms
	PIT_CallbackInstall(HW_PIT_CH0, PIT0_ISR);	//PIT0_ISR是自定义中断函数名
	PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE);

	PIT_QuickInit(HW_PIT_CH1, 0.2*1000);				//定时0.2ms
	PIT_CallbackInstall(HW_PIT_CH1, PIT1_ISR); 
	PIT_ITDMAConfig(HW_PIT_CH1, kPIT_IT_TOF,ENABLE);

	/**********蜂鸣器第二声示意初始化完成****************/
	GPIO_ToggleBit(HW_GPIOB, 22);
	DelayMs(100);
	GPIO_ToggleBit(HW_GPIOB, 22);		

	while(1)
	{
				
//		Key = (enum Key)Key_Scan();
//		switch(Key)
//		{
//			case Key_Up			:	Duty1 += 100;				// 0-10000 对应占空比 0-100%
//												if(Duty1 > 10000)	Duty1 = 10000;
//												FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3 , Duty1);		
//												OLED_Show_Data(0,0,Duty1);
//												break;	
//			case Key_Down		: Duty1 -= 100;
//												if(Duty1 < 0)			Duty1 = 0;			
//												FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3 , Duty1);		
//												OLED_Show_Data(0,0,Duty1);
//												break;	//右轮
//			case Key_Left		: Duty2 += 100;	
//												if(Duty2 > 10000)	Duty2 = 10000;
//												FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4 , Duty2);		
//												OLED_Show_Data(0,1,Duty2);
//												break;	//左轮
//			case Key_Right	: Duty2 -= 100;	
//												if(Duty2 < 0)			Duty2 = 0;			
//												FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4 , Duty2);		
//												OLED_Show_Data(0,1,Duty2);
//												break;
//			case Key_Enter	: 		 break;
//			default :  break;	//DelayMs(10);	
//		}

		Key = (enum Key)Key_Scan(!Boma_1);	//拨码开关1---0,不支持连续按;1,支持连续按
		switch(Key)
		{
			case Key_Up			:	Flash_Parameter.Stand_Kp += 0.1;
												OLED_P6x8Str(0,0,"Kp:");
												OLED_Show_Float(3,0,Flash_Parameter.Stand_Kp);
												break;	
			case Key_Down		: Flash_Parameter.Stand_Kp -= 0.1;
												OLED_P6x8Str(0,0,"Kp:");
												OLED_Show_Float(3,0,Flash_Parameter.Stand_Kp);
												break;	//右轮
			case Key_Left		: Flash_Parameter.Stand_Kd += 0.1;
												OLED_P6x8Str(0,1,"Kd:");
												OLED_Show_Float(3,1,Flash_Parameter.Stand_Kd);
												break;	//左轮
			case Key_Right	: Flash_Parameter.Stand_Kd -= 0.1;
												OLED_P6x8Str(0,1,"Kd:");
												OLED_Show_Float(3,1,Flash_Parameter.Stand_Kd);
												break;
			case Key_Enter	:	EraseSector(TEST_ADDR_BEIGN);
												ProgramPage(TEST_ADDR_BEIGN, sizeof(Flash_Parameter), (void*)&Flash_Parameter);
												pidSetKp(&PID_Stand, Flash_Parameter.Stand_Kp);			
												pidSetKd(&PID_Stand, Flash_Parameter.Stand_Kd);		//0.6
												Motor_Set_Flag = 1;
												break;
			default :  break;	//DelayMs(10);	
		}

		
		CCD_Report();

	}	//while end
	
}	//main end

