#include "CTRL.h"

extern struct Quad_PID PID_Stand , PID_Speed , PID_Turn ;
extern float Ang , Mid_Filtered , Mid;
extern struct Parameter
	{
	float Stand_Kp , Stand_Kd ;
	float Speed_Kp , Speed_Ki , Speed_Kd;
	float Ang_Set , Speed_Set ;
	float Turn_Kp , Turn_Kd ;
}	
Flash_Parameter;	

//���еĦ�����ڵ���������궨�����ӿ�ʼת��ռ�ձȡ���ʵ����    ����ʱ��ȡ5000 ???


#define	MOTOT_DEADLINE_LEFT_POS	5600			//�ӵ������
#define	MOTOT_DEADLINE_LEFT_NEG	4350
#define	MOTOT_DEADLINE_RIGHT_POS	5450
#define	MOTOT_DEADLINE_RIGHT_NEG	4500

//#define	MOTOT_DEADLINE_LEFT_POS	5550			//�ӵ������
//#define	MOTOT_DEADLINE_LEFT_NEG	4450
//#define	MOTOT_DEADLINE_RIGHT_POS	5400
//#define	MOTOT_DEADLINE_RIGHT_NEG	4600

//#define	MOTOT_DEADLINE_LEFT_POS	5700			//�ӵ������
//#define	MOTOT_DEADLINE_LEFT_NEG	4300
//#define	MOTOT_DEADLINE_RIGHT_POS	5550
//#define	MOTOT_DEADLINE_RIGHT_NEG	4450


	
float Speed_Car = 0;	
int Speed_Left_Filtered = 0,Speed_Right_Filtered = 0; 	//��¼����������� 
float Car_Status[8] = {0};	
	
uint8_t sgn(int32_t num)
{
	if(num >= 0)
		return 1;
	else
		return 0;
}	
	
/**************1ms*****************/
static void PIT0_ISR(void)
{
		static uint8_t Count_Flag = 0;		//5ms��־λ
		static uint8_t Speed_Flag = 0 , Speed_Ctrl_Flag = 0;
		static uint16_t Speed_Ctrl_Cnt = 0;
		extern uint8_t IntegrationTime;       //�ع�ʱ��
		extern uint8_t UART_Buffer_CCD[132];						//��Ųɼ�ADֵ
		static uint8_t TimerCnt15ms = 0;
		uint8_t integration_piont = 0;
		extern uint8_t Motor_Set_Flag ;			//��Enter���ƣ��Ƿ����������



	
		TimerCnt15ms++;

		//�����ع�ʱ������ع�㣬ִ��CCD��������
		integration_piont = 15 - IntegrationTime;    //100 * 0.2 = 20ms     
		if(integration_piont == TimerCnt15ms)
			StartIntegration();         	//��ʼ�ع�
	
		if(Count_Flag == 1)
		{

		}
		
		if(Count_Flag == 2)		//ֱ��	5ms����	5ms����
		{
			IMU_Update();   		//����̬���������˲�����Ƕ�
			PID_Stand_Update();
			
			if(Motor_Set_Flag)
			{
				Speed_Ctrl_Cnt++;
				if(Speed_Ctrl_Cnt == 200)		//վ��������		400 500
				{
					//�ĳ��������Ĳ���
					pidInit(&PID_Stand, Flash_Parameter.Stand_Kp , 0, Flash_Parameter.Stand_Kd ,Flash_Parameter.Ang_Set);	//REset pid��ֵ�������²���
					pidInit(&PID_Speed, Flash_Parameter.Speed_Kp , Flash_Parameter.Speed_Ki , Flash_Parameter.Speed_Kd ,Flash_Parameter.Speed_Set);
					pidInit(&PID_Turn, Flash_Parameter.Turn_Kp , 0 , Flash_Parameter.Turn_Kd , 0);	
					
//					pidInit(&PID_Stand, Flash_Parameter.Stand_Kp , 0, Flash_Parameter.Stand_Kd ,Flash_Parameter.Ang_Set);	//REset pid��ֵ�������²���
//					pidInit(&PID_Speed, Flash_Parameter.Speed_Kp , Flash_Parameter.Speed_Ki , Flash_Parameter.Speed_Kd ,0);
//					pidInit(&PID_Turn, Flash_Parameter.Turn_Kp , 0 , Flash_Parameter.Turn_Kd , 0);	
					
				}	
				if(Speed_Ctrl_Cnt > 300) Speed_Ctrl_Cnt = 300;
				Motor_Set();
			}
			

			
		}
		
		if(Count_Flag == 3)		
		{		
			
			#ifdef __UART_DMA_Car_Status_Report__
//			Car_Status[0] = Ang * 100;
//			Car_Status[1] = Flash_Parameter.Ang_Set * 100;
			Car_Status[0] = Speed_Car * 100;
			Car_Status[1] = Flash_Parameter.Speed_Set * 100;
//			Car_Status[2] = Mid_Filtered;
			Car_Status[2] = PID_Stand.PID_out;
//			Car_Status[5] = PID_Speed.outP;
//			Car_Status[6] = PID_Speed.outI;
			Car_Status[3] = PID_Speed.PID_out;
			Car_Status[4] = Mid;
			Car_Status[5] = PID_Turn.outP;
			Car_Status[6] = PID_Turn.outD;
			Car_Status[7] = PID_Turn.PID_out;	
			UART_DMA_Array_Report(28,Car_Status);
			#endif
			
//			#ifdef __UART_DMA_Car_Status_Report__
//			Car_Status[0] = Mid_Filtered;
//			Car_Status[1] = PID_Turn.error;
////			Car_Status[2] = Deta_error;
//			Car_Status[2] = PID_Turn.outP ;
//			Car_Status[3] = PID_Turn.outD ;
//			Car_Status[4] = PID_Turn.PID_out;	
//			UART_DMA_Array_Report(20,Car_Status);
//			#endif

			
		}
		
		if(Count_Flag == 4)		//�ٶ�	75ms����	5ms����
		{	
			
			Speed_Flag++;
//			if(Speed_Flag == 7)
//			{
//				Speed_Measure();	
//				PID_Speed_Update();
//			}
			if(Speed_Flag == 15)
			{
				Speed_Flag = 0;
				Speed_Measure();	
				PID_Speed_Update();
			}			
		}
		
		if(Count_Flag == 5)		//����	20ms�ؼ���	5ms����
		{
			if(TimerCnt15ms >= 15) 				//��ʱ15ms
			{
				TimerCnt15ms = 0;
				ImageCapture(UART_Buffer_CCD + 2);					//CCD��������,�������в��������
				CalculateIntegrationTime();		//���ݲɼ���ƽ����ѹ(�����ǿ)�����������ع�ʱ��
				Recognize_Road();
				PID_Turn_Update();
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
	/***********��ʼ��PIT��ʱģ��***********************/
	PIT_QuickInit(HW_PIT_CH0, 1*1000);					//��ʱ1ms
	PIT_CallbackInstall(HW_PIT_CH0, PIT0_ISR);	//PIT0_ISR���Զ����жϺ�����
	PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE);

}

void FTM_PWM_Encoder_Init()
{
	/***********��ʼ��PTM0������·PWM********/
	FTM_PWM_QuickInit(FTM0_CH3_PA06, kPWM_EdgeAligned, 25000);		//	���ֵ�λHz���������ʼ��ռ�ձ�Ϊ50%
	FTM_PWM_QuickInit(FTM0_CH4_PA07, kPWM_EdgeAligned, 25000);	

	/***********��ʼ������������������*******/
	FTM_QD_QuickInit(FTM1_QD_PHA_PA12_PHB_PA13, kFTM_QD_NormalPolarity, kQD_PHABEncoding);	
	FTM_QD_QuickInit(FTM2_QD_PHA_PA10_PHB_PA11, kFTM_QD_NormalPolarity, kQD_PHABEncoding);

}

void Speed_Measure()
{
	int32_t Speed_Left = 0,Speed_Right = 0; 	//��¼����������� 
  uint8_t Dir_Left = 0,Dir_Right = 0;		//��¼��������ת����1 
	
	static int32_t Speed_Left_Filter[3] = {0},Speed_Right_Filter[3] = {0};	//�˲���
	int32_t Sum_Left = 0,Sum_Right = 0;
	uint8_t  i = 0,Cnt_Left = 0,Cnt_Right = 0;
	static uint8_t	num = 0 ;
	
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
		
	//�������ߣ����Ϊ3�˲�Ч������		ƽ���˲������ٶȻ����		�о�����Ҫ�˲������������ٿ�
	Speed_Left_Filter[num] = Speed_Left;
	Speed_Right_Filter[num] = Speed_Right;
	num = (num + 1) % 3;
	for(i=0;i<3;i++)
	{
		if( sgn(Speed_Left_Filter[i]) == sgn(Speed_Left) )
		{
			Cnt_Left++;
			Sum_Left += Speed_Left_Filter[i];
		}
		if( sgn(Speed_Right_Filter[i]) == sgn(Speed_Right) )
		{
			Cnt_Right++;
			Sum_Right += Speed_Right_Filter[i];
		}
	}
	
	if(Cnt_Left != 0)
	{
		if(Speed_Left == 0)
			Speed_Left_Filtered = 0;
		else
			Speed_Left_Filtered =  Sum_Left / Cnt_Left ;
	}
	else
		Speed_Left_Filtered = Speed_Left;
	
	if(Cnt_Right != 0)
	{
		if(Speed_Right == 0)
			Speed_Right_Filtered = 0;
		else
			Speed_Right_Filtered = Sum_Right / Cnt_Right ;
	}
	else
		Speed_Right_Filtered = Speed_Right;
	

	//Speed_Car = (Speed_Left + Speed_Right) / 2.0 ;
	Speed_Car = ( Speed_Left_Filtered + Speed_Right_Filtered ) / 2;
	

	#ifdef __UART_DMA_Speed_Measure_Report__
	temp[0] = Speed_Left;
	temp[1] = Speed_Right;
	temp[2] = Speed_Car;
	UART_DMA_Array_Report(sizeof(temp) ,temp);
	#endif
	
	FTM_QD_ClearCount(HW_FTM1); 	//�����Ƶ������Ҫ��ʱ���Countֵ ����ʱ��ȡ���� 
	FTM_QD_ClearCount(HW_FTM2);
	
}	

void Motor_Set()
{
	extern float Ang;			//��������쳣ͣ��
	extern struct Quad_PID PID_Stand , PID_Speed , PID_Turn;
	extern uint8_t Motor_Set_Flag;
	int16_t Motor_Left = 0,Motor_Right = 0,Duty_Left = 0,Duty_Right = 0;
	
	PID_Speed.PID_out += PID_Speed.PID_Avg_out;
	PID_Turn.PID_out += PID_Turn.PID_Avg_out;
	
	if(!DIP_2)	//����ֱ���ٶȿ���
	{
		PID_Turn.PID_out = 0;
	}
	if(!DIP_3)	//����ת�����
	{
		PID_Speed.PID_out = 0;
	}
	if(!DIP_4)	//����ֱ������
	{
		PID_Turn.PID_out = 0;
		PID_Speed.PID_out = 0;
	}
	
	Motor_Left = PID_Stand.PID_out - PID_Speed.PID_out + PID_Turn.PID_out; 
	Motor_Right = PID_Stand.PID_out - PID_Speed.PID_out - PID_Turn.PID_out;

	//����
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
	
	//����
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
	
	//���ӽǶ�ʧ��ͣ���
	if( Ang > 75 || Ang < 13)
	{
		Duty_Left = 5000;
		Duty_Right = 5000;
		Motor_Set_Flag = 0;
	}
	
	FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4 , Duty_Left);
	FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3 , Duty_Right);
	
}

