
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CODE_STORE_DEVICE_ST_STM32F1XX_CMSIS_DRIVER_BUTTON_H_
#define CODE_STORE_DEVICE_ST_STM32F1XX_CMSIS_DRIVER_BUTTON_H_

#ifdef __cplusplus
 extern "C" {
#endif 

#include "../utilities/utilities.h"
#include "./common/arm_button.h"


#define BUTTON_INIT            ((uint8_t)0x01)   // BUTTON initialized


/* BUTTON GPIO Configuration */
typedef const struct _BUTTON_IO {
  GPIO_TypeDef       *gpio_port;
  uint32_t            gpio_pin;
  uint8_t             pull;
} BUTTON_IO;

/* I2C Information (Run-Time) */
typedef struct _BUTTON_INFO {
  ARM_BUTTON_SIGNAL_EVENT cb_event;  
  uint8_t volatile      flags;                // Current BUTTON state flags
//  IRQn_Type             irq_num;           // I2C Event IRQ Number
//  uint8_t               reserved[2];          // Reserved
} BUTTON_INFO;

/* I2C Resources definition */
typedef struct {
  BUTTON_IO        io;                   // BUTTON Input/Output pins
  BUTTON_INFO      *info;                 // Run-Time information

} const BUTTON_RESOURCES;


extern ARM_BUTTON arm_button1;
extern ARM_BUTTON arm_button2;
extern ARM_BUTTON arm_button3;


#ifdef __cplusplus
}
#endif
  
#endif   /* CODE_STORE_DEVICE_ST_STM32F1XX_CMSIS_DRIVER_BUTTON_H_ */

