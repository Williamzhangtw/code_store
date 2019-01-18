#if !defined CODE_STORE_DEVICE_ST_STM32F1XX_UTILS_SOC_INIT_H_
	#define CODE_STORE_DEVICE_ST_STM32F1XX_UTILS_SOC_INIT_H_

#include "../hal/hal.h"

extern LL_RCC_ClocksTypeDef rcc_clocks;

void UTILS_SystemClockInit(void);
void UTILS_McoInit(void);
void UTILS_NvicInit(void);

	
#endif  /* CODE_STORE_DEVICE_ST_STM32F1XX_UTILS_SOC_INIT_H_ */
