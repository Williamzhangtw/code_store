#if !defined CODE_STORE_DEVICE_ST_STM32F1XX_UTILS_DELAY_H_
#define CODE_STORE_DEVICE_ST_STM32F1XX_UTILS_DELAY_H_
	
#include "../hal/hal.h"


extern volatile int32_t microseconds;

extern volatile int32_t milliseconds;



void UTILS_MilliSecondInit(void);

void UTILS_MicroSecondInit(void); 

void UTILS_MicroSecondDelay(uint32_t count);

void UTILS_MilliSecondDelay(uint32_t count);

 
 
#endif /* CODE_STORE_DEVICE_ST_STM32F1XX_UTILS_DELAY_H_ */
