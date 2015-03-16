#include "common.h"

#ifndef  calculation_H
#define  calculation_H  

#define SI_SetVal()   PEout(4) = 1
#define SI_ClrVal()   PEout(4) = 0
#define CLK_ClrVal()  PEout(5) = 0
#define CLK_SetVal()  PEout(5) = 1

void CCD_Init(void);
void StartIntegration(void);   
void ImageCapture(uint8_t * ImageData);
void SamplingDelay(void);
void CalculateIntegrationTime(void);
uint8_t PixelAverage(uint8_t len, uint8_t *data);



#endif
