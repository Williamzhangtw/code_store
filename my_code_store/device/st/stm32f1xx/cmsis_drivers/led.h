
/* Define to prevent recursive inclusion -------------------------------------*/
#if !defined CODE_STORE_DEVICE_ST_STM32F1XX_CMSIS_DRIVER_LED_H_
 #define CODE_STORE_DEVICE_ST_STM32F1XX_CMSIS_DRIVER_LED_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

#include "../utilities/utilities.h"
#include "./common/arm_led.h"

#define LED_INIT            ((uint8_t)0x01)   // LED initialized

#define LED_BLINK_FAST  200 /* 200 ms period*/
#define LED_BLINK_SLOW  500

/* LED GPIO Configuration */
typedef const struct _LED_IO {
  GPIO_TypeDef* gpio_port;
  uint32_t      gpio_pin;
}LED_IO;

/* I2C Information (Run-Time) */
typedef struct _LED_INFO {
	ARM_LED_STATUS  status;
//uint8_t               reserved[2];          // Reserved
}LED_INFO;

/* I2C Resources definition */
typedef const struct _LED_RESOURCES{
  LED_IO        io;                   // LED Input/Output pins
  LED_INFO      *info;                 // Run-Time information
}LED_RESOURCES;


#if defined USE_LED1
extern ARM_LED arm_led1;
extern LED_RESOURCES led1_resources;
#endif

#if defined USE_LED2
extern LED_RESOURCES led1_resources;
extern ARM_LED arm_led2;
#endif

#if defined USE_LED3
extern ARM_LED arm_led3;
extern LED_RESOURCES led1_resources;
#endif

#if defined USE_LED4
extern ARM_LED arm_led4;
extern LED_RESOURCES led1_resources;
#endif


#ifdef __cplusplus
}
#endif
  
#endif  /* CODE_STORE_DEVICE_ST_STM32F1XX_CMSIS_DRIVER_LED_H_ */

