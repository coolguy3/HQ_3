#ifndef __PID_H__
#define __PID_H__

#include "common.h"
#include "UART_DMA.h"

struct Quad_PID
{
	float target;  		// 目标值
	float current; 		// 当前值
	float error;			//当前偏差
	float d_error;		//上次偏差
	float dd_error;		//上上次偏差
	float Integrator;	//当前积分值(未乘上Ki)
	float deriv;			//当前微分值(未乘上Kd)
	float iLimit;			//积分限制值
	float Kp;					//比例系数
	float Ki;					//积分系数
	float Kd;					//微分系数

	float outP;         //< proportional output (debugging)
  float outI;         //< integral output (debugging)
  float outD;         //< derivative output (debugging)
	float PID_out;   		//当前PID的输出
	float PID_Avg_out;	//增量平均输出
		
};

#define DEFAULT_PID_INTEGRATION_LIMIT  1000.0

void pidInit(struct Quad_PID* pid, const float kp,
             const float ki, const float kd);
float PID_Stand_Update(void);
float PID_Speed_Update(void);
void pidSetIntegralLimit(struct Quad_PID* pid, float limit);
void pidSetError(struct Quad_PID* pid, float err);
void pidReset(struct Quad_PID* pid);
void pidSetTarget(struct Quad_PID* pid, float target);
void pidSetKp(struct Quad_PID* pid, float kp);
void pidSetKi(struct Quad_PID* pid, float ki);
void pidSetKd(struct Quad_PID* pid, float kd);
void pidSetMeasured(struct Quad_PID* pid, float measured);

#endif
