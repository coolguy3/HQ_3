#ifndef __CTRL_H__
#define	__CTRL_H__

#include "common.h"
#include "pit.h"
#include "ftm.h"

#include "OLED.h"
#include "UART_DMA.h"
#include "IMU.h"
#include "CCD.h"
#include "PID.h"

void PIT_Timer_Init(void);
void FTM_PWM_Encoder_Init(void);
void Motor_Set(void);
void Speed_Measure(void);


#endif
