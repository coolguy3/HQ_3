#ifndef __KEY_H__
#define __KEY_H__

#include "common.h"
#include "gpio.h"

#define DIP_1  PDin(14)
#define DIP_2  PDin(13)
#define DIP_3  PDin(12)
#define DIP_4	 PDin(11)

#define K1			0
#define K2			1
#define K3			2
#define K4			3
#define K5			4

void Key_Init(void);
void DIP_Init(void);
void Bee_Init(void);
uint8_t Key_Scan(uint8_t mode);

#endif
