#include "CCD.h"

uint8_t IntegrationTime = 2;	//曝光时间  
uint8_t UART_Buffer_CCD[132] = {0};	//存放Report的AD值
uint8_t	Pixel[128] = {0};	//存放采集的AD值
//中线位置，CCD均值，跳边沿阈值(22-30间)，左右线,赛道宽度
uint8_t PixelAverageValue,CCD_Threshold=15,RoadWith,Guide_Threshold=0;
int16_t Left,Right,Guide_Left,Guide_Right, Left_Pre[3] = {0}  , Right_Pre[3] = {0} ;
float  Mid=64 , Mid_Pre[3] = {64} , RoadWith_Pre[3] = {90} ;
uint8_t Road_State , Last_Road_State , Guide_Line = 0;
float	Mid_Filter[3] = {0} , Mid_Filtered = 0;

//************求均值**************************
uint8_t PixelAverage(uint8_t len, uint8_t *data) 
{
  uint8_t i;
  uint16_t sum = 0;
  for(i = 0; i < len; i++) 
	{
    sum += *data++;
  }
  return ( (uint8_t)(sum/len) );
}

//************CCD延时程序 200ns***************
void SamplingDelay(void)
{
	volatile uint8_t i ;
	for(i = 0; i < 1; i++) 
	{
    __asm("nop");
    __asm("nop");
	}
}

//*************CCD初始化**********************
void CCD_Init(void)
{
  GPIO_QuickInit(HW_GPIOC, 4, kGPIO_Mode_OPP);	//SI
	GPIO_QuickInit(HW_GPIOC, 5, kGPIO_Mode_OPP);	//CLK		
	PCout(4) = 1;
	PCout(5) = 1;
	
	ADC_QuickInit(ADC1_SE4B_PC8, kADC_SingleDiff8or9);	//8位精度

	UART_Buffer_CCD[0] = 0x02;	UART_Buffer_CCD[1] = 0xfd;			//帧头
	UART_Buffer_CCD[130] = 0xfd;	UART_Buffer_CCD[131] = 0x02;	//帧尾
}

//***************CCD启动程序******************
void StartIntegration(void) 
{
	uint8_t i;

	SI_SetVal();            /* SI  = 1 */
	SamplingDelay();
	CLK_SetVal();           /* CLK = 1 */
	SamplingDelay();
	SI_ClrVal();            /* SI  = 0 */
	SamplingDelay();
	CLK_ClrVal();           /* CLK = 0 */

	for(i=0; i<127; i++) 
	{
		SamplingDelay();
		SamplingDelay();
		CLK_SetVal();       /* CLK = 1 */
		SamplingDelay();
		SamplingDelay();
		CLK_ClrVal();       /* CLK = 0 */
	}
	
	SamplingDelay();
	SamplingDelay();
	CLK_SetVal();           /* CLK = 1 */
	SamplingDelay();
	SamplingDelay();
	CLK_ClrVal();           /* CLK = 0 */
	
}

//***************CCD采样程序******************************
void ImageCapture(uint8_t * ImageData) 
{
	uint8_t i , *p = Pixel;
	extern uint8_t AtemP ;

	SI_SetVal();            /* SI  = 1 */
	SamplingDelay();
	CLK_SetVal();           /* CLK = 1 */
	SamplingDelay();
	SI_ClrVal();            /* SI  = 0 */
	SamplingDelay();

	//Delay 10us for sample the first pixel																									！！！
	for(i = 0; i < 250; i++)                     //更改250，让CCD的图像看上去比较平滑
	{
		SamplingDelay() ;  //200ns                 //把该值改大或者改小达到自己满意的结果。
	}

	*ImageData =  ADC_QuickReadValue(ADC1_SE4B_PC8);
	*p = *ImageData;
	ImageData ++ ;
	p++ ;
	
	CLK_ClrVal();         /* CLK = 0 */
	for(i=0; i<127; i++) 
	{
		SamplingDelay();
		SamplingDelay();
		CLK_SetVal();       /* CLK = 1 */
		SamplingDelay();
		SamplingDelay();
		*ImageData =  ADC_QuickReadValue(ADC1_SE4B_PC8);
		*p = *ImageData;
		ImageData ++ ;
		p++ ;
		CLK_ClrVal();       /* CLK = 0 */
	}
	
	SamplingDelay();
	SamplingDelay();
	CLK_SetVal();           /* CLK = 1 */
	SamplingDelay();
	SamplingDelay();
	CLK_ClrVal();           /* CLK = 0 */
}

//***************计算曝光时间，单位ms **************************
void CalculateIntegrationTime(void) 
{
		extern uint8_t PixelAverageValue;		//AD均值
		uint8_t PixelAverageVoltage;	//实际电压均值 电压都放大10倍
		int16_t TargetPixelAverageVoltage = 25;		//目标电压均值    ！！！
		int16_t PixelAverageVoltageError = 0;			//偏差
		int16_t TargetPixelAverageVoltageAllowError = 2;		//死区	！！！

    PixelAverageValue = PixelAverage(128,UART_Buffer_CCD + 2);
 //   PixelAverageVoltage = (unsigned char)((int)PixelAverageValue * 25 / 194);
		PixelAverageVoltage = (unsigned char)( ((int)PixelAverageValue * 33 ) >> 8 );
	
    PixelAverageVoltageError = TargetPixelAverageVoltage - PixelAverageVoltage;
	
    if(PixelAverageVoltageError < -TargetPixelAverageVoltageAllowError)
    {
      PixelAverageVoltageError = 0- PixelAverageVoltageError ;
      PixelAverageVoltageError /= 2;				//  ??
      if(PixelAverageVoltageError > 10 )		//防止调节太快			！！！
         PixelAverageVoltageError = 10 ;
       IntegrationTime -= PixelAverageVoltageError/6.666;
    }
    if(PixelAverageVoltageError > TargetPixelAverageVoltageAllowError)
    { 
        PixelAverageVoltageError /= 2;
        if(PixelAverageVoltageError > 10 )
           PixelAverageVoltageError = 10 ;
        IntegrationTime += PixelAverageVoltageError/6.666;}
 
    if(IntegrationTime <= 1)			//积分时间限幅   ！！！
        IntegrationTime = 1;
    if(IntegrationTime >= 15)
        IntegrationTime = 15;
		
}

//识别黑线，计算中线
void Get_Line(void) 
{
//	extern uint8_t PixelAverageValue;
//	CCD_Threshold = PixelAverageValue/2-24;
	
	uint8_t Maxfind = RoadWith_Pre[2]/2.0 + 10 , i = 0;
	int16_t Looptmp1 , Looptmp2;
	Guide_Left = Guide_Right =0;
	
	//找左线
	if( Mid_Pre[2]> 124)
	{
		Looptmp1 = 124;
	}
	else
	{
		Looptmp1 = Mid_Pre[2];
	}
	
	//Maxfind稍大于最近几次赛道宽度
//	if( Mid_Pre[2] - Maxfind > 3)
//	{
//		Looptmp2 = Mid_Pre[2] - Maxfind;
//	}
//	else
//	{
		Looptmp2 = 3;
//	}
	
	Left = 0;//检测不到黑线时都为0
	for( i = Looptmp1 ; i >= Looptmp2 ; i--)
	{
		if(			Guide_Line == 0
				&&	Pixel[i]+4 <= Pixel[i+1]
				&&	Pixel[i+1]+4 <= Pixel[i+2]
				&& 	Pixel[i+2]+4 <= Pixel[i+3] 
				&& 	Pixel[i+3]-Pixel[i] >= CCD_Threshold
			)
		{
			Left = i ;
			if(Guide_Right == 0) Guide_Right = Left;
			Guide_Line = 1;
			Guide_Threshold = Pixel[i+3]-Pixel[i];
		}
		if(			Guide_Line == 1
				&&	Pixel[i]+4 <= Pixel[i-1]
				&& 	Pixel[i-1]+4 <= Pixel[i-2] 
				&& 	Pixel[i-2]+4 <= Pixel[i-3]
				&& 	Pixel[i-3]-Pixel[i] >= Guide_Threshold-3
			)
		{
			Guide_Left = i ;
			break;
		}
	}	
	Left_Pre[0] =  Left_Pre[1] ;
	Left_Pre[1] =  Left_Pre[2] ;
	Left_Pre[2] =  Left;
	Guide_Line = 0;
	if(Guide_Left == 0 || Guide_Right == 0)
	{
		Guide_Left = Guide_Right = 0;
	}
	
	//找右线
	if( Mid_Pre[2] < 3)
	{
		Looptmp1 = 3;
	}
	else
	{
		Looptmp1 = Mid_Pre[2];
	}
	
	//Maxfind稍大于最近几次赛道宽度的一半
//	if( Mid_Pre[2] + Maxfind < 125)
//	{
//		Looptmp2 = Mid_Pre[2] + Maxfind;
//	}
//	else
//	{
		Looptmp2 = 125;
//	}
	
	Right = 0;//检测不到黑线时都为0
	for(i = Looptmp1 ; i <= Looptmp2 ; i++)
	{
		if(			Pixel[i]+4 <= Pixel[i-1]
				&& 	Pixel[i-1]+4 <= Pixel[i-2] 
				&& 	Pixel[i-2]+4 <= Pixel[i-3]
				&& 	Pixel[i-3]-Pixel[i] >= CCD_Threshold
			)
		{
			Right = i;
			if(Guide_Left == 0) Guide_Left = Right;
			Guide_Line = 1;
			Guide_Threshold = Pixel[i-3]-Pixel[i];
		}
		if(			Guide_Line == 1
				&&	Pixel[i]+4 <= Pixel[i+1]
				&&	Pixel[i+1]+4 <= Pixel[i+2]
				&& 	Pixel[i+2]+4 <= Pixel[i+3] 
				&& 	Pixel[i+3]-Pixel[i] >= Guide_Threshold-3
			)
		{
			Guide_Right = i ;
			break;
		}
	}
	Right_Pre[0] =  Right_Pre[1] ;
	Right_Pre[1] =  Right_Pre[2] ;
	Right_Pre[2] =  Right;
	Guide_Line = 0;
	if(Guide_Left == 0 || Guide_Right == 0)
	{
		Guide_Left = Guide_Right = 0;
	}
	if(Guide_Left != 0 && Guide_Right != 0)
	{
		Left = Guide_Left;
		Right = Guide_Right;
	}
}


//均值数据滤波
void Filter_Pixel(void)
{
	unsigned char i;
	for(i=1;i<127;i++)
	{
		if(UART_Buffer_CCD[i+2]==0&&UART_Buffer_CCD[i+2]!=UART_Buffer_CCD[i+1]&&UART_Buffer_CCD[i]!=UART_Buffer_CCD[i+3])
		{
			UART_Buffer_CCD[i+2]=250;
		} 
		else if(UART_Buffer_CCD[i+2]==250&&UART_Buffer_CCD[i+2]!=UART_Buffer_CCD[i+1]&&UART_Buffer_CCD[i+2]!=UART_Buffer_CCD[i+3])
		{
			UART_Buffer_CCD[i+2]=0;
		}

		if(UART_Buffer_CCD[i+2]==0&&UART_Buffer_CCD[i+2]!=UART_Buffer_CCD[i+1]&&UART_Buffer_CCD[i+2]!=UART_Buffer_CCD[i+3])
		{
			UART_Buffer_CCD[i+2]=250;
		} 
		else if(UART_Buffer_CCD[i+2]==250&&UART_Buffer_CCD[i+2]!=UART_Buffer_CCD[i+1]&&UART_Buffer_CCD[i+2]!=UART_Buffer_CCD[i+3])
		{
			UART_Buffer_CCD[i+2]=0;
		}
	}

	//黑线上的点数超过3时使用
	for(i=1;i<126;i++)
	{
		if(UART_Buffer_CCD[i]==0&&UART_Buffer_CCD[i]==UART_Buffer_CCD[i+1]&&UART_Buffer_CCD[i]!=UART_Buffer_CCD[i-1]&&UART_Buffer_CCD[i]!=UART_Buffer_CCD[i+2])
		{
			UART_Buffer_CCD[i]=250;
			UART_Buffer_CCD[i+1]=250;
		}
		if(UART_Buffer_CCD[i]==250&&UART_Buffer_CCD[i]==UART_Buffer_CCD[i+1]&&UART_Buffer_CCD[i]!=UART_Buffer_CCD[i-1]&&UART_Buffer_CCD[i]!=UART_Buffer_CCD[i+2])            
		{
			UART_Buffer_CCD[i]=0;
			UART_Buffer_CCD[i+1]=0;
		}       
	}
}

void Recognize_Road(void)
{
	static int num = 0;  
	int i=0;
	float sum=0;
	
	Get_Line();
	Mid_Pre[0] = Mid_Pre[1];
	Mid_Pre[1] = Mid_Pre[2];
	Mid_Pre[2] = Mid_Filtered;
	RoadWith_Pre[0] = RoadWith_Pre[1];
	RoadWith_Pre[1] = RoadWith_Pre[2];
	RoadWith_Pre[2] = RoadWith;
	Last_Road_State = Road_State;
	
	if(Left == 0 && Right == 0 && PixelAverageValue<100) 
	{
		Road_State = 1;		//全黑
		if(Last_Road_State == 3)
		{
			Mid = Mid_Pre[2]; //数字待定
			RoadWith = RoadWith_Pre[2];
		}
		if(Last_Road_State == 4)
		{
			Mid = Mid_Pre[2] + 0; //数字待定
			RoadWith = RoadWith_Pre[2];
		}
		if(Last_Road_State == 5)
		{
			Mid = Mid_Pre[2] - 0; //数字待定
			RoadWith = RoadWith_Pre[2];
		}
	}
	if(Left == 0 && Right == 0 && PixelAverageValue>100)
	{
		Road_State = 2;		//全白
		Mid = Mid_Pre[2];
		RoadWith = RoadWith_Pre[2];
	}
	if(Left > 0 && Right > 0 && PixelAverageValue>60)
	{
		Road_State = 3;		//没丢线  直道   ?弯道
		Mid = ( Left + Right ) / 2.0;
		RoadWith = Right - Left;
	}
	if(Left == 0 && Right > 0 && PixelAverageValue>60)
	{
		Road_State = 4;		//左丢线
		Mid = Right - RoadWith_Pre[2]/2 - 0;
		RoadWith = RoadWith_Pre[2];
	}
	if(Left > 0 && Right == 0 && PixelAverageValue>60)
	{
		Road_State = 5;		//右丢线
		Mid = Left + RoadWith_Pre[2]/2 + 0;
		RoadWith = RoadWith_Pre[2];
	}
	
	
	Mid_Filter[num] = Mid;
	for(i=0;i<3;i++)
		 sum += Mid_Filter[i];
	Mid_Filtered =  sum / 3.0 ;
	num = (num + 1) % 3;
	

}

