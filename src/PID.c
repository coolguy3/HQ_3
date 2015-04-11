#include "PID.h"
#include "UART_DMA.h"

struct Quad_PID PID_Stand , PID_Speed;

void pidInit(struct Quad_PID* pid, float kp,float ki, float kd)
{
  pid->error = 0;
  pid->d_error = 0;
	pid->dd_error = 0;
  pid->Integrator = 0;
  pid->deriv = 0;
  pid->target = 0;
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  
  pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;
}

float PID_Stand_Update()
{
  extern float Ang;
	extern float Gyro_v;
	float temp[6] = {0};
	float output = 0;
	
  PID_Stand.current = Ang;
  PID_Stand.error = PID_Stand.target - Ang;
	
	PID_Stand.deriv = -Gyro_v;

	PID_Stand.outP = PID_Stand.Kp * PID_Stand.error;
	PID_Stand.outD = PID_Stand.Kd * PID_Stand.deriv;
	
	PID_Stand.PID_out = output = 	PID_Stand.outP +
														PID_Stand.outD;


//	temp[0] = PID_Stand.target;
//	temp[1] = Ang;
//	temp[2] = PID_Stand.PID_out/10.0;
//	temp[3] = -Gyro_v;
//	UART_DMA_Array_Report(sizeof(temp),temp);
	
  return output;
}

float PID_Speed_Update()
{
  extern float Speed_Car;
	float	Error,D_Error,DD_Error;
	static float output = 0;
	
	float temp[6] = {0};
	
	Error = PID_Speed.target - Speed_Car;
	D_Error = Error - PID_Speed.d_error;
	DD_Error = D_Error - PID_Speed.dd_error;
	
	PID_Speed.d_error = Error;
	PID_Speed.dd_error = D_Error;
	
	if( Error > 0 || Error < 0)
	{
		PID_Speed.outP = PID_Speed.Kp * D_Error;
		PID_Speed.outI = PID_Speed.Ki * Error;
		PID_Speed.outD = PID_Speed.Kd * DD_Error;
		
		PID_Speed.PID_Avg_out = output = 	( PID_Speed.outP + PID_Speed.outI + PID_Speed.outD ) / 15.0;	//增量平均输出
	}
	else
		PID_Speed.PID_Avg_out = output = 	0;
	
//	UART_DMA_Array_Report(sizeof(temp),temp);
	
  return output;
}

void pidSetError(struct Quad_PID* pid, float err)
{
  pid->error = err;
}

void pidSetIntegralLimit(struct Quad_PID* pid, float limit)
{
  pid->iLimit = limit;
}

void pidReset(struct Quad_PID* pid)
{
  pid->error = 0;
  pid->d_error = 0;
	pid->dd_error = 0;
  pid->Integrator = 0;
  pid->deriv = 0;
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

void pidSetMeasured(struct Quad_PID* pid, float measured)
{
  pid->current = measured;
}
