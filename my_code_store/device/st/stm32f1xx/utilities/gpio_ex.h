#if !defined CODE_STORE_DEVICE_ST_STM32F1XX_UTILS_GPIO_EX_H_
	#define CODE_STORE_DEVICE_ST_STM32F1XX_UTILS_GPIO_EX_H_
	
#include "../hal/hal.h"

/*
#define BITBAND_ADDR(GPIOx,bitnum) (volatile uint32_t *)(((uint32_t)&GPIOx->ODR & 0xF0000000) + 0x2000000+ \
	(((uint32_t)&GPIOx->ODR& 0xFFFFF)<<5)+ (bitnum <<2)) // Convert the address as a pointer
	*BITBAND_ADDR(LED_ORANGE_Port,LED_ORANGE_BIT) = 0; //reset pin
*/

/*
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
 */
#define BITBAND_ADDR(GPIOx,Pin) (__IO uint32_t *)(((uint32_t)&GPIOx->ODR & 0xF0000000) + 0x2000000+  \
	(((uint32_t)&GPIOx->ODR & 0xFFFFF)*32)+ (POSITION_VAL(Pin >> GPIO_PIN_MASK_POS) *4)) 



 
void UTILS_GpioClockEnable(GPIO_TypeDef *gpio);



#endif  /* CODE_STORE_DEVICE_ST_STM32F1XX_UTILS_GPIO_EX_H_  */
