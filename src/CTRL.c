#include "CTRL.h"


//���еĦ�����ڵ���������궨�����ӿ�ʼת��ռ�ձȡ���ʵ����
#define	MOTOT_DEADLINE_LEFT_POS	5400
#define	MOTOT_DEADLINE_LEFT_NEG	4600
#define	MOTOT_DEADLINE_RIGHT_POS	5400
#define	MOTOT_DEADLINE_RIGHT_NEG	4600

void Speed_Measure()
{
	int Speed_Left,Speed_Right; 	//��¼����������� 
  uint8_t Dir_Left,Dir_Right;		//��¼��������ת����1 

	FTM_QD_GetData(HW_FTM1, &Speed_Left, &Dir_Left);
	FTM_QD_GetData(HW_FTM2, &Speed_Right, &Dir_Right);
	//printf("value:%6x dir:%d  \r", value, dir);
	FTM_QD_ClearCount(HW_FTM1); 	//�����Ƶ������Ҫ��ʱ���Countֵ ����ʱ��ȡ���� 
	FTM_QD_ClearCount(HW_FTM2);
	
//	Gyro_Filter[num] = * GY;
//	for(i=0;i<10;i++)
//		 sum += Gyro_Filter[i];
//	Gyro_Y_Filtered = ( sum / 10.0 ) - Gy_Offset;
//	num = (num + 1) % 10;
//	
	
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

