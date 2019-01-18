#if !defined CODE_STORE_DEVICE_ST_STM32F1XX_HAL_HAL_H_
 #define CODE_STORE_DEVICE_ST_STM32F1XX_HAL_HAL_H_
/* HAL : hardware abstraction layer */
 
 
/*
 * standard C
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


/*
 * HAL
 */

#if defined USE_HAL_DRIVER
/* HAL library */
#include "./stm32f1xx_hal.h"
/*will include "hal_config.h" file */
#endif

#if defined USE_FULL_LL_DRIVER
#include "./stm32f1xx_ll_conf.h"
/* will include "ll_config.h" file */
#endif

#include "redefine_io.h"  /* placed at BSP folder */

#endif /* CODE_STORE_DEVICE_ST_STM32F1XX_HAL_HAL.H_ */
