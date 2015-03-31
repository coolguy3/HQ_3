#include "PID.h"

extern float Gyro_v;

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

float PID_Stand_Update(struct Quad_PID* pid, float measured)
{
  float output;
	
  pid->current = measured;
  pid->merror = pid->target - measured;

  pid->Integrator += pid->merror * 0.005;
  if (pid->Integrator > pid->iLimit)
  {
    pid->Integrator = pid->iLimit;
  }
  else if (pid->Integrator < -pid->iLimit)
  {
    pid->Integrator = -pid->iLimit;
  }

  pid->deriv = Gyro_v;

  pid->outP = pid->Kp * pid->merror;
  pid->outI = pid->Ki * pid->Integrator;
  pid->outD = pid->Kd * pid->deriv;
  pid->PID_out = output = 	pid->outP +
														pid->outI +
														pid->outD;

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
