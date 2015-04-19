#include "CCD.h"


uint8_t IntegrationTime = 2;	//�ع�ʱ��  
uint8_t UART_Buffer_CCD[132] = {0};	//��Ųɼ�ADֵ

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
	uint8_t i;
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
	ImageData ++ ;
	CLK_ClrVal();         /* CLK = 0 */
	for(i=0; i<127; i++) 
	{
		SamplingDelay();
		SamplingDelay();
		CLK_SetVal();       /* CLK = 1 */
		SamplingDelay();
		SamplingDelay();
		*ImageData =  ADC_QuickReadValue(ADC1_SE4B_PC8);
		ImageData ++ ;
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
		extern uint8_t Pixel[128];
		uint8_t PixelAverageValue;		//AD��ֵ
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


//*********************�Լ���Ӧ�����λ���㷨*************************************
#define LINEBREADTH    10 
#define LINECONCAT     8
void AccommodFondLine(int8_t *PixelAryy ,uint8_t PixelCount , int16_t *LastLeftPixelStation,int16_t *LastRingtPixelStation,uint8_t FAVAULE)
{
  static uint8_t NOLeftCount,NORightCout ;
  int16_t temp0B ,temp1B,temp2B,temp3B;
  uint8_t *LineStation ,LineCount ,*LineLeftStation,*LineRightStation;
  int16_t LeftMIN,LeftMAX,RightMIN,RightMAX;
  LineCount = 0 ;
  for(temp0B = 0 ; temp0B < PixelCount ; temp0B ++)
  {
    temp1B = temp0B ;
    temp2B = 0 ;
    /***********
    ������߰���
    ***********/
    while(temp2B <= LINEBREADTH) {
      temp1B -- ;
      if(temp1B < 0)
        break ;
      if( PixelAryy[temp1B] -  PixelAryy[temp0B] > FAVAULE )
      { temp2B ++ ;}
      else if(temp2B)
      { break ; }
   
    }
     
    /***********
    �����ұ߰���
    ***********/
    temp1B = temp0B ;
    temp3B = 0 ;
    while(temp3B <= LINEBREADTH)
    {
      temp1B ++ ;
      if(temp1B > PixelCount)
      { break ; }
      if( PixelAryy[temp1B] -  PixelAryy[temp0B] > FAVAULE )
      { temp3B ++ ;}
      else if(temp3B)
      { break ; }
    }
    /***********
    ��¼����λ��
    ***********/    
    if(temp2B >= LINEBREADTH ){
      *LineStation = temp0B ;
      LineCount ++ ;
    }else if(temp3B >= LINEBREADTH ){
      *LineStation = temp0B ;
      LineCount ++ ;
    }
		
	}
	
  /**********
  ���������Բ������Һ���λ��
  **********/
  if(LineCount)
  {
    temp2B = PixelCount >> 1 ;
    temp1B = NOLeftCount << 1;
    temp1B += LINECONCAT;
    LeftMIN = *LastLeftPixelStation - temp1B ;
    LeftMAX = *LastRingtPixelStation + temp1B ;
    if(LeftMIN < 0)
      LeftMIN = 0 ;

    if(LeftMAX > (temp2B + 1))
      LeftMAX  = temp2B + 1 ;
    
    RightMIN = *LastRingtPixelStation - temp1B ;
    RightMAX = *LastRingtPixelStation + temp1B ;
    if(RightMAX > PixelCount)
       RightMAX = PixelCount ;
    if(RightMIN < (temp2B - 1))
		{
       RightMIN = temp2B - 1 ;
    }
    temp2B = 0 ;
    temp3B = 0 ;
		for(temp1B = 0 ;temp1B < LineCount ;temp1B ++ )
		{
			if( (LeftMIN < LineStation[temp1B])&&(LineStation[temp1B]<LeftMAX))
			{
				LineLeftStation[temp2B] = LineStation[temp1B] ;
				temp2B ++ ;
			}else if( (RightMIN < LineStation[temp1B])&&(LineStation[temp1B]<RightMAX))
			{
				LineRightStation[temp3B] = LineStation[temp1B] ;
				temp3B ++ ;
			} 
		}
    
  }
	else 
  {
    NOLeftCount ++ ;
    NORightCout ++ ;
  }
  
  if(temp2B)
  {
    NOLeftCount = 0 ;
  }
     
}



