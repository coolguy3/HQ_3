#include "common.h"
#include "gpio.h"
#include "uart.h"
#include "pit.h"
#include "IIC.h"
#include "IMU.h"
#include "CCD.h"

uint8_t Pixel[128] = {0};
uint8_t TIME1flag_20ms = 0 ;
uint8_t send_data_cnt = 0;

//**************1ms定时中断*****************
static void PIT0_ISR(void)
{
//		static uint8_t IMU_3ms = 0;
//		IMU_3ms++;
//		if(IMU_3ms == 3)
//		{
//			IMU_3ms = 0;
//			IMU_Update();   //有 串口IMU_Report，耗时在2~2.5ms之间
//		}
}

//**************0.2ms定时中断***************
static void PIT1_ISR(void)
{
  extern uint8_t IntegrationTime;       //曝光时间      
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
		ImageCapture(Pixel);					//CCD采样程序,函数里有参数需调整，按时序控制SI、CLK进行AD采集，结果存放在Pixel数组
		CalculateIntegrationTime();		//根据采集输出AO(可表示光强)，反馈计算曝光时间
  }
	
}

int main(void)
{
	uint8_t i = 0;
	
	DelayInit();
	//***********初始化LED**************************
	GPIO_QuickInit(HW_GPIOA, 17, kGPIO_Mode_OPP);		

	//***********初始化串口**************************
	UART_QuickInit(UART0_RX_PA15_TX_PA14 , 115200); 			
	UART_EnableTxFIFO(HW_UART0, true);
	UART_SetTxFIFOWatermark(HW_UART0, UART_GetTxFIFOSize(HW_UART0));

	//***********初始化模拟IIC: SDA--E24 SCL--E25 、陀螺仪、加速度计******
//	IMU_Init();		

	//***********初始化PIT模块***********************
	PIT_QuickInit(HW_PIT_CH0, 1*1000);				 //定时1ms
	PIT_CallbackInstall(HW_PIT_CH0, PIT0_ISR); //PIT0_ISR是自定义中断函数名
	PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE);

	PIT_QuickInit(HW_PIT_CH1, 1*200);				 	 //定时0.2ms
	PIT_CallbackInstall(HW_PIT_CH1, PIT1_ISR); 
	PIT_ITDMAConfig(HW_PIT_CH1, kPIT_IT_TOF,ENABLE);
	
	//***********初始化 SI--E4 CLK--E5 、ADC0_SE6B_PD5 8bits精度**********
	CCD_Init(); 
	
	while(1)
	{
	//		DelayMs(500);
	//		GPIO_ToggleBit(HW_GPIOA, 17);	
		if(TIME1flag_20ms == 1)
		{
			TIME1flag_20ms = 0; 
			

			if(++send_data_cnt >= 5) 
			{
				send_data_cnt = 0;
				UART_WriteByte(HW_UART0, 0x02);		//帧头
				UART_WriteByte(HW_UART0, 0xfd);
				for(i = 0;i < 128;i++)
				{
					UART_WriteByte(HW_UART0, Pixel[i] );
				}
				UART_WriteByte(HW_UART0, 0xfd);		//帧尾
				UART_WriteByte(HW_UART0, 0x02);
				
			}
			
		} 
		
	}	//while end
	
}	//main end

