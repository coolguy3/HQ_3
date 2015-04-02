#ifndef __CCD_H__
#define __CCD_H__
 
#include "common.h"

#define SI_SetVal()   PCout(4) = 1
#define SI_ClrVal()   PCout(4) = 0
#define CLK_ClrVal()  PCout(5) = 0
#define CLK_SetVal()  PCout(5) = 1

void CCD_Init(void);
void StartIntegration(void);   
void ImageCapture(uint8_t * ImageData);
void SamplingDelay(void);
void CalculateIntegrationTime(void);
uint8_t PixelAverage(uint8_t len, uint8_t *data);
void CCD_Report(void);

#endif
