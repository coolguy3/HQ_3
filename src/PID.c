#include "PID.h"


struct Quad_PID PID_Stand , PID_Speed , PID_Turn;
extern struct Parameter{
	float Stand_Kp , Stand_Kd ;
	float Speed_Kp , Speed_Ki , Speed_Kd;
	float Ang_Set , Speed_Set ;
}	Flash_Parameter;	

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
  extern float Speed_Car , Ang;
	float Deta_error;
	static float output = 0;
	static uint8_t Integ_Start = 0 , Deriv_Start = 0 , Speedup_Finish = 0;
	
	float temp[6] = {0};
	
	if(PID_Speed.target > 0)	//速度目标值为0是起跑前的静止直立
	{
		if(!Speedup_Finish)
		{	
			
			if( Speed_Car < Flash_Parameter.Speed_Set * 1/3 )
			{
				PID_Speed.target = Flash_Parameter.Speed_Set * 1/3;
			}
			
//			if( Speed_Car < Flash_Parameter.Speed_Set * 1/6 )
//      {
//				PID_Speed.target = Flash_Parameter.Speed_Set * 1/6;
//      }
//			else if( Speed_Car < Flash_Parameter.Speed_Set * 2/6 )
//			{
//				PID_Speed.target = Flash_Parameter.Speed_Set * 2/6;
//			}
			
			else if( Speed_Car < Flash_Parameter.Speed_Set * 3/6 )
			{
				PID_Speed.target = Flash_Parameter.Speed_Set * 3/6;
			}
			else if( Speed_Car < Flash_Parameter.Speed_Set * 4/6 )
			{
				PID_Speed.target = Flash_Parameter.Speed_Set * 4/6;
			}
			else if( Speed_Car < Flash_Parameter.Speed_Set * 5/6 )
			{
				PID_Speed.target = Flash_Parameter.Speed_Set * 5/6;
			}
			else
			{		
//				pidInit(&PID_Stand, Flash_Parameter.Stand_Kp , 0 , Flash_Parameter.Stand_Kd , 39.5);	//REset pid各值，并更新参数
//				pidInit(&PID_Speed, Flash_Parameter.Speed_Kp , Flash_Parameter.Speed_Ki , Flash_Parameter.Speed_Kd ,Flash_Parameter.Speed_Set);
				PID_Speed.target = Flash_Parameter.Speed_Set;
				Integ_Start = 1;
				Speedup_Finish = 1;
			}
		}
	}
	else
	{	
		Integ_Start = 1;		//静止用PID
	}


				
	PID_Speed.error = PID_Speed.target - Speed_Car;
	Deta_error = PID_Speed.error - PID_Speed.d_error;
	
//	if( PID_Speed.error < 40 && PID_Speed.error > -40 && Deta_error < 80 && Deta_error > -80)//跳轮 等速度稳定后再加上比例偏差微分偏差较大不输出，数字实验看曲线定  
//	{
		PID_Speed.outP = PID_Speed.Kp * PID_Speed.error;
		
		if( PID_Speed.error < 20 || PID_Speed.error > -20)	//积分分离  数字实验看曲线定 
		{
			if(Integ_Start)
			{
				PID_Speed.outI += PID_Speed.Ki * PID_Speed.error/10.0;
			}
			else
			{
				PID_Speed.outI = 0;		//积分清0
			}
//		if( PID_Speed.outI > integ_limit)			//积分限幅	 数字实验看曲线定 
//			PID_Speed.outI = integ_limit;
//		if( PID_Speed.outI < -integ_limit)
//			PID_Speed.outI = -integ_limit;
		}
			
		PID_Speed.outD = PID_Speed.Kd * Deta_error;
		
		output =  PID_Speed.outP + PID_Speed.outI + PID_Speed.outD;	
//		if( output > 1000)	output = 1000;
//		if( output < -1000)	output = -1000;
		PID_Speed.PID_Avg_out = ( output - PID_Speed.PID_Lastout ) / 15.0;
//	}
//	else
//		PID_Speed.PID_Avg_out = 0;
	
	PID_Speed.d_error = PID_Speed.error;
//	PID_Speed.PID_out = PID_Speed.PID_Lastout;
	PID_Speed.PID_Lastout = output;
	
	#ifdef __UART_DMA_PID_Speed_Report__
	temp[0] = PID_Speed.target;
	temp[1] = Speed_Car;
	temp[2] = PID_Speed.outP ;
	temp[3] = PID_Speed.outI ;
	temp[4] = output;	
	UART_DMA_Array_Report(sizeof(temp),temp);
	#endif
	
  return output;
}




float PID_Turn_Update()	//	滤波
{
  extern float Mid_Pre[3]  , Mid_Filtered , Mid;
	static float Deta_error = 0;
	static float output = 0;
	
	float temp[6] = {0};
	
	PID_Turn.error = Mid_Filtered - 64;
	//PID_Turn.error = Mid - 64;
	
	if(PID_Turn.error - PID_Turn.d_error > 4)
	{
		PID_Turn.error = PID_Turn.d_error + 4;
	}
	if(PID_Turn.error - PID_Turn.d_error < -4)
	{
		PID_Turn.error = PID_Turn.d_error - 4;
	}
	
//	if(PID_Turn.error - PID_Turn.d_error > 10)
//	{
//		PID_Turn.error = PID_Turn.d_error + 10;
//	}
//	if(PID_Turn.error - PID_Turn.d_error < -10)
//	{
//		PID_Turn.error = PID_Turn.d_error - 10;
//	}
	
	Deta_error = 0.5 * Deta_error + 0.5 * (PID_Turn.error - PID_Turn.d_error);
	//Deta_error = PID_Turn.error - PID_Turn.d_error;
	
	PID_Turn.outP = PID_Turn.Kp * PID_Turn.error;			
	PID_Turn.outD = PID_Turn.Kd * (Deta_error / 0.015);
	if( PID_Turn.outD > 1000)	PID_Turn.outD = 1000;
	if( PID_Turn.outD < -1000)	PID_Turn.outD = -1000;
	output =  PID_Turn.outP + PID_Turn.outD ;	

	PID_Turn.PID_Avg_out = ( output - PID_Turn.PID_Lastout ) / 3.0;

	PID_Turn.d_error = PID_Turn.error;
	PID_Turn.PID_out = PID_Turn.PID_Lastout;
	PID_Turn.PID_Lastout = output;
	
	#ifdef __UART_DMA_PID_Turn_Report__
	temp[0] = Mid_Filtered;
	temp[1] = PID_Turn.error;
	temp[2] = Deta_error;
	temp[3] = PID_Turn.outP ;
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


