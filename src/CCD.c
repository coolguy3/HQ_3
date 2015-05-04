#include "CCD.h"

uint8_t IntegrationTime = 2;	//�ع�ʱ��  
uint8_t UART_Buffer_CCD[132] = {0};	//���Report��ADֵ
uint8_t	Pixel[128] = {0};	//��Ųɼ���ADֵ
//����λ�ã�CCD��ֵ����������ֵ(22-30��)��������,�������
uint8_t Mid=64,PixelAverageValue,CCD_Threshold=10,RoadWith;
int16_t Left,Right;
uint8_t Mid_Pre[3] = {64} , Left_Pre[3] = {0}  , Right_Pre[3] = {0} , RoadWith_Pre[3] = {0} ;
uint8_t road_state;
//************���ֵ**************************
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

//************CCD��ʱ���� 200ns***************
void SamplingDelay(void)
{
	volatile uint8_t i ;
	for(i = 0; i < 1; i++) 
	{
    __asm("nop");
    __asm("nop");
	}
}

//*************CCD��ʼ��**********************
void CCD_Init(void)
{
  GPIO_QuickInit(HW_GPIOC, 4, kGPIO_Mode_OPP);	//SI
	GPIO_QuickInit(HW_GPIOC, 5, kGPIO_Mode_OPP);	//CLK		
	PCout(4) = 1;
	PCout(5) = 1;
	
	ADC_QuickInit(ADC1_SE4B_PC8, kADC_SingleDiff8or9);	//8λ����

	UART_Buffer_CCD[0] = 0x02;	UART_Buffer_CCD[1] = 0xfd;			//֡ͷ
	UART_Buffer_CCD[130] = 0xfd;	UART_Buffer_CCD[131] = 0x02;	//֡β
}

//***************CCD��������******************
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

//***************CCD��������******************************
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

	//Delay 10us for sample the first pixel																									������
	for(i = 0; i < 250; i++)                     //����250����CCD��ͼ����ȥ�Ƚ�ƽ��
	{
		SamplingDelay() ;  //200ns                 //�Ѹ�ֵ�Ĵ���߸�С�ﵽ�Լ�����Ľ����
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

//***************�����ع�ʱ�䣬��λms **************************
void CalculateIntegrationTime(void) 
{
		extern uint8_t PixelAverageValue;		//AD��ֵ
		uint8_t PixelAverageVoltage;	//ʵ�ʵ�ѹ��ֵ ��ѹ���Ŵ�10��
		int16_t TargetPixelAverageVoltage = 25;		//Ŀ���ѹ��ֵ    ������
		int16_t PixelAverageVoltageError = 0;			//ƫ��
		int16_t TargetPixelAverageVoltageAllowError = 2;		//����	������

    PixelAverageValue = PixelAverage(128,UART_Buffer_CCD + 2);
 //   PixelAverageVoltage = (unsigned char)((int)PixelAverageValue * 25 / 194);
		PixelAverageVoltage = (unsigned char)( ((int)PixelAverageValue * 33 ) >> 8 );
	
    PixelAverageVoltageError = TargetPixelAverageVoltage - PixelAverageVoltage;
	
    if(PixelAverageVoltageError < -TargetPixelAverageVoltageAllowError)
    {
      PixelAverageVoltageError = 0- PixelAverageVoltageError ;
      PixelAverageVoltageError /= 2;				//  ??
      if(PixelAverageVoltageError > 10 )		//��ֹ����̫��			������
         PixelAverageVoltageError = 10 ;
       IntegrationTime -= PixelAverageVoltageError/6.666;
    }
    if(PixelAverageVoltageError > TargetPixelAverageVoltageAllowError)
    { 
        PixelAverageVoltageError /= 2;
        if(PixelAverageVoltageError > 10 )
           PixelAverageVoltageError = 10 ;
        IntegrationTime += PixelAverageVoltageError/6.666;}
 
    if(IntegrationTime <= 1)			//����ʱ���޷�   ������
        IntegrationTime = 1;
    if(IntegrationTime >= 15)
        IntegrationTime = 15;
		
}

//ʶ����ߣ���������
void Get_Line(void) 
{
//	extern uint8_t PixelAverageValue;
//	CCD_Threshold = PixelAverageValue/2-24;
	uint8_t Maxfind = 33 , i = 0;
	int16_t Looptmp1 , Looptmp2;
	
	//������
	if( Mid_Pre[2] > 124)
	{
		Looptmp1 = 124;
	}
	else
	{
		Looptmp1 = Mid_Pre[2];
	}
	
	//Maxfind�Դ�����������������
	if( Mid_Pre[2] - Maxfind > 3)
	{
		Looptmp2 = Mid_Pre[2] - Maxfind;
	}
	else
	{
		Looptmp2 = 3;
	}
	
	Left = -1;//��ⲻ������ʱ��Ϊ-1
	for( i = Looptmp1 ; i >= Looptmp2 ; i--)
	{
		if(			Pixel[i]+4 <= Pixel[i+1]
				&&	Pixel[i+1]+4 <= Pixel[i+2]
				&& 	Pixel[i+2]+4 <= Pixel[i+3] 
				&& 	Pixel[i+3]-Pixel[i] >= CCD_Threshold
				&& 	i <= Mid_Pre[2]		)
		{
			Left = i ;
			break;
		}
	}	
	Left_Pre[0] =  Left_Pre[1] ;
	Left_Pre[1] =  Left_Pre[2] ;
	Left_Pre[2] =  Left;
	
	//������
	if( Mid_Pre[2] < 3)
	{
		Looptmp1 = 3;
	}
	else
	{
		Looptmp1 = Mid_Pre[2];
	}
	
	//Maxfind�Դ����������������ȵ�һ��
	if( Mid_Pre[2] + Maxfind < 125)
	{
		Looptmp2 = Mid_Pre[2] + Maxfind;
	}
	else
	{
		Looptmp2 = 125;
	}
	
	Right = -1;//��ⲻ������ʱ��Ϊ-1
	for(i = Looptmp1 ; i <= Looptmp2 ; i++)
	{
		if(			Pixel[i]+4 <= Pixel[i-1]
				&& 	Pixel[i-1]+4 <= Pixel[i-2] 
				&& 	Pixel[i-2]+4 <= Pixel[i-3]
				&& 	Pixel[i-3]-Pixel[i] >= CCD_Threshold
				&&  i >= Mid_Pre[2]		 )
		{
			Right = i ;
			break;
		}
	}	
	Right_Pre[0] =  Right_Pre[1] ;
	Right_Pre[1] =  Right_Pre[2] ;
	Right_Pre[2] =  Right;
	
}


//��ֵ�����˲�
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

	//�����ϵĵ�������3ʱʹ��
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
	Get_Line();
	Mid_Pre[0] = Mid_Pre[1];
	Mid_Pre[1] = Mid_Pre[2];
	Mid_Pre[2] = Mid;
	RoadWith_Pre[0] = RoadWith_Pre[1];
	RoadWith_Pre[1] = RoadWith_Pre[2];
	RoadWith_Pre[2] = RoadWith;
	
	if(Left < 0 && Right < 0 && PixelAverageValue<60) 
	{
		road_state = 1;		//ȫ��
		
	}
	if(Left < 0 && Right < 0 && PixelAverageValue>60)
	{
		road_state = 2;		//ȫ��
	}
	if(Left > 0 && Right > 0 && PixelAverageValue>60)
	{
		road_state = 3;		//û����  ֱ��   ?���
		Mid = ( Left + Right ) / 2;
		RoadWith = (Right - Left) / 2;
	}
	if(Left < 0 && Right > 0 && PixelAverageValue>60)
	{
		road_state = 4;		//����
		Mid = Right - RoadWith_Pre[2];
		RoadWith = RoadWith_Pre[2];
	}
	if(Left > 0 && Right < 0 && PixelAverageValue>60)
	{
		road_state = 5;		//�Ҷ���
		Mid = Left + RoadWith_Pre[2];
		RoadWith = RoadWith_Pre[2];
	}
	

	
	

}

