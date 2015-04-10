#include "CTRL.h"
#include "OLED.h"

//���еĦ�����ڵ���������궨�����ӿ�ʼת��ռ�ձȡ���ʵ����
//#define	MOTOT_DEADLINE_LEFT_POS	5400
//#define	MOTOT_DEADLINE_LEFT_NEG	4600
//#define	MOTOT_DEADLINE_RIGHT_POS	5300
//#define	MOTOT_DEADLINE_RIGHT_NEG	4600

#define	MOTOT_DEADLINE_LEFT_POS	5100
#define	MOTOT_DEADLINE_LEFT_NEG	4900
#define	MOTOT_DEADLINE_RIGHT_POS	5000
#define	MOTOT_DEADLINE_RIGHT_NEG	5000

int Speed_Left_Filtered = 0,Speed_Right_Filtered = 0; 	//��¼����������� 

void Speed_Measure()
{
	int Speed_Left = 0,Speed_Right = 0; 	//��¼����������� 
  uint8_t Dir_Left = 0,Dir_Right = 0;		//��¼��������ת����1 
	static uint16_t Speed_Left_Filter[10] = {0},Speed_Right_Filter[10] = {0};		//��¼��������ת����1 
	uint16_t Sum_Left = 0,Sum_Right = 0;
	uint8_t  i = 0;
	static uint8_t	num = 0 ;
	static uint8_t Time = 0;
	
	FTM_QD_GetData(HW_FTM1, &Speed_Left, &Dir_Left);
	FTM_QD_GetData(HW_FTM2, &Speed_Right, &Dir_Right);
	
//	Time++;
//	if(Time > 40)
//	{
//		Time = 0;
		OLED_Show_Data(3,0,Speed_Left);
		OLED_Show_Data(3,1,Speed_Right);	
//	}
	
//	printf("value:%6x dir:%d  \r", Speed_Left, Dir_Left);
	
	FTM_QD_ClearCount(HW_FTM1); 	//�����Ƶ������Ҫ��ʱ���Countֵ ����ʱ��ȡ���� 
	FTM_QD_ClearCount(HW_FTM2);
	

	
	Speed_Left_Filter[num] = Speed_Left;
	Speed_Right_Filter[num] = Speed_Right;
	for(i=0;i<3;i++)
	{
		Sum_Left += Speed_Left_Filter[i];
		Sum_Right += Speed_Right_Filter[i];
	}
	Speed_Left_Filtered =  Sum_Left / 3.0 ;
	Speed_Right_Filtered = Sum_Right / 3.0 ;
	num = (num + 1) % 3;
	
}	


void Motor_Set()
{
	extern float Ang;
	extern struct Quad_PID PID_Stand;
	int16_t Motor_Left = 0,Motor_Right = 0,Duty_Left = 0,Duty_Right = 0;
	Motor_Left = PID_Stand.PID_out; 
	Motor_Right = PID_Stand.PID_out;

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
	
//	//���ӽǶ�ʧ��ͣ���
//	if( Ang > 75 || Ang < 25)
//	{
//		Duty_Left = 5000;
//		Duty_Right = 5000;
//	}
	
	FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4 , Duty_Left);
	FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3 , Duty_Right);
	
}

