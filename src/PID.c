#include "PID.h"


struct Quad_PID PID_Stand , PID_Speed , PID_Turn;

void pidInit(struct Quad_PID* pid, float kp,float ki, float kd , float target)
{
	pid->error = 0;
  pid->d_error = 0;
	pid->dd_error = 0;	
	pid->outP	 = 0;        
  pid->outI = 0;         
  pid->outD = 0;         
	pid->PID_out = 0;   		
	pid->PID_Lastout = 0;  
	pid->PID_Avg_out = 0;	
	
	pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  pid->target = target;
	
}

float PID_Stand_Update()
{
  extern float Ang;
	extern float Gyro_v;
	float temp[6] = {0};
	float output = 0;
	
  PID_Stand.error = PID_Stand.target - Ang;
	
	PID_Stand.outP = PID_Stand.Kp * PID_Stand.error;
	PID_Stand.outD = PID_Stand.Kd * -Gyro_v;
	
	PID_Stand.PID_out = output = 	PID_Stand.outP +
														PID_Stand.outD;

	#ifdef __UART_DMA_PID_Stand_Report__
	temp[0] = PID_Stand.target;
	temp[1] = Ang;
	temp[2] = -Gyro_v;
	temp[3] = PID_Stand.outP;
	temp[4] = PID_Stand.outD;
	temp[5] = PID_Stand.PID_out/10.0;	
	UART_DMA_Array_Report(sizeof(temp),temp);
	#endif
	
  return output;
}

//float PID_Speed_Update()			//增量式
//{
//  extern float Speed_Car;
//	float	Error,D_Error,DD_Error;
//	static float output = 0;
//	
//	float temp[6] = {0};
//	
//	Error = PID_Speed.target - Speed_Car;
//	D_Error = Error - PID_Speed.d_error;
//	DD_Error = D_Error - PID_Speed.dd_error;
//	
//	PID_Speed.d_error = Error;
//	PID_Speed.dd_error = D_Error;
//	
//	if( Error > 0 || Error < 0)
//	{
//		PID_Speed.outP = PID_Speed.Kp * D_Error;
//		PID_Speed.outI = PID_Speed.Ki * Error;
//		PID_Speed.outD = PID_Speed.Kd * DD_Error;
//		
//		PID_Speed.PID_Avg_out = output = 	( PID_Speed.outP + PID_Speed.outI + PID_Speed.outD ) / 15.0;	//增量平均输出
//	}
//	else
//		PID_Speed.PID_Avg_out = output = 	0;
//	
//	#ifdef __UART_DMA_PID_Speed_Report__
//	temp[0] = PID_Speed.target;
//	temp[1] = Speed_Car;
//	temp[2] = PID_Speed.outP ;
//	temp[3] = PID_Speed.outI ;
//	temp[4] = PID_Speed.outD ;
//	temp[5] = PID_Speed.PID_Avg_out;	
//	UART_DMA_Array_Report(sizeof(temp),temp);
//	#endif
//	
//  return output;
//}

#define integ_limit 30000
float PID_Speed_Update()
{
  extern float Speed_Car;
	float Deta_error ;
	static float output = 0;
	
	float temp[6] = {0};
	
	PID_Speed.error = PID_Speed.target - Speed_Car;
	Deta_error = PID_Speed.error - PID_Speed.d_error;
	
	if( PID_Speed.error < 3000 || PID_Speed.error >-3000 || Deta_error < 3000 || Deta_error > -3000)//比例偏差微分偏差较大不输出，数字实验看曲线定  
	{
		PID_Speed.outP = PID_Speed.Kp * PID_Speed.error;
		
	//	if( PID_Speed.error < 300 || PID_Speed.error > -300)	//积分分离  数字实验看曲线定 
			PID_Speed.outI += PID_Speed.Ki * PID_Speed.error;
//		if( PID_Speed.outI > integ_limit)			//积分限幅	 数字实验看曲线定 
//			PID_Speed.outI = integ_limit;
//		if( PID_Speed.outI < -integ_limit)
//			PID_Speed.outI = -integ_limit;
			
		PID_Speed.outD = PID_Speed.Kd * ( Deta_error / 0.075 );
		
		output =  PID_Speed.outP + PID_Speed.outI + PID_Speed.outD ;	
		PID_Speed.PID_Avg_out = ( output - PID_Speed.PID_Lastout ) / 15.0;
	}
	else
		PID_Speed.PID_Avg_out = 0;
	
	
	PID_Speed.d_error = PID_Speed.error;
	PID_Speed.PID_out = PID_Speed.PID_Lastout;
	PID_Speed.PID_Lastout = output;
	
	#ifdef __UART_DMA_PID_Speed_Report__
	temp[0] = PID_Speed.target;
	temp[1] = Speed_Car;
	temp[2] = PID_Speed.outP ;
	temp[3] = PID_Speed.outI ;
	temp[4] = PID_Speed.outD ;
	temp[5] = output;	
	UART_DMA_Array_Report(sizeof(temp),temp);
	#endif
	
  return output;
}

float PID_Turn_Update()
{
  float Line_Now , Line_Last;
	float Deta_error;
	static float output = 0;
	
	float temp[6] = {0};
	
	PID_Turn.error = Line_Now - Line_Last;
	Deta_error = PID_Turn.error - PID_Turn.d_error;
	
	if( PID_Turn.error < 3000 || PID_Turn.error >-3000 || Deta_error < 3000 || Deta_error > -3000)//比例偏差微分偏差较大不输出，数字实验看曲线定  
	{
		PID_Turn.outP = PID_Turn.Kp * PID_Turn.error;			
		PID_Turn.outD = PID_Turn.Kd * Deta_error;
		
		output =  PID_Turn.outP + PID_Turn.outD ;	
		PID_Turn.PID_Avg_out = ( output - PID_Turn.PID_Lastout ) / 3.0;
	}
	else
		PID_Turn.PID_Avg_out = 0;
	
	
	PID_Turn.d_error = PID_Turn.error;
	PID_Turn.PID_out = PID_Speed.PID_Lastout;
	PID_Turn.PID_Lastout = output;
	
	#ifdef __UART_DMA_PID_Turn_Report__
	temp[0] = Line_Now;
	temp[1] = Line_Last;
	temp[2] = PID_Turn.outP ;
	temp[3] = PID_Turn.outI ;
	temp[4] = PID_Turn.outD ;
	temp[5] = output;	
	UART_DMA_Array_Report(sizeof(temp),temp);
	#endif
	
  return output;
}

void pidSetError(struct Quad_PID* pid, float err)
{
  pid->error = err;
}



void pidSetTarget(struct Quad_PID* pid, float target)
{
  pid->target = target;
}


void pidSetKp(struct Quad_PID* pid, float kp)
{
  pid->Kp = kp;
}

void pidSetKi(struct Quad_PID* pid, float ki)
{
  pid->Ki = ki;
}

void pidSetKd(struct Quad_PID* pid, float kd)
{
  pid->Kd = kd;
}


