#include "PID.h"
#include "UART_DMA.h"

struct Quad_PID PID_Stand;

void pidInit(struct Quad_PID* pid, float kp,float ki, float kd)
{
  pid->merror = 0;
  pid->last_error = 0;
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
	float output;
	
  PID_Stand.current = Ang;
  PID_Stand.merror = PID_Stand.target - Ang;
	
	if( PID_Stand.merror > 0.3 || PID_Stand.merror < -0.3)	//µ÷½ÚËÀÇø
	{
		PID_Stand.deriv = -Gyro_v;

		PID_Stand.outP = PID_Stand.Kp * PID_Stand.merror;
		PID_Stand.outD = PID_Stand.Kd * PID_Stand.deriv;
		
		PID_Stand.PID_out = output = 	PID_Stand.outP +
															PID_Stand.outD;
	}

	temp[0] = PID_Stand.target;
	temp[1] = Ang;
	temp[2] = PID_Stand.PID_out/10.0;
	UART_DMA_Array_Width_Six(temp);
	
  return output;
}

void pidSetError(struct Quad_PID* pid, float err)
{
  pid->merror = err;
}

void pidSetIntegralLimit(struct Quad_PID* pid, float limit)
{
  pid->iLimit = limit;
}

void pidReset(struct Quad_PID* pid)
{
  pid->merror = 0;
  pid->last_error = 0;
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
