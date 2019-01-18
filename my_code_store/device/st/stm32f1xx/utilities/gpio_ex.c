#include "gpio_ex.h"

/*
 * belief: enable the GPIO clock according to the GPIO port \
 * the function will return the alternate function EXTI port \
 * in case the corresponding pin needs to set up EXTI mode
 */
void UTILS_GpioClockEnable(GPIO_TypeDef *gpio)
{
	uint32_t periphs_clock_enable_bit_mask =0;
	switch ((uint32_t)gpio)
	{
		case (uint32_t)GPIOA:
		    periphs_clock_enable_bit_mask = RCC_APB2ENR_IOPAEN;
			break;
		case (uint32_t)GPIOB:
			periphs_clock_enable_bit_mask = RCC_APB2ENR_IOPBEN;
			break;
		case (uint32_t)GPIOC:
			periphs_clock_enable_bit_mask = RCC_APB2ENR_IOPCEN;
			break;
  #if defined(GPIOD)
		case (uint32_t)GPIOD:
			periphs_clock_enable_bit_mask = RCC_APB2ENR_IOPDEN;
			break;
  #endif
  #if defined(GPIOF)
    case (uint32_t)GPIOF:
      periphs_clock_enable_bit_mask = RCC_APB2ENR_IOPFEN;
      break;
  #endif
  #if defined(GPIOG)
    case (uint32_t)GPIOG:
      periphs_clock_enable_bit_mask = RCC_APB2ENR_IOPGEN;
      break;
  #endif
  #if defined(GPIOH)
    case (uint32_t)GPIOH:
      periphs_clock_enable_bit_mask = RCC_APB2ENR_IOPHEN;
      break;
  #endif
		default:
			break;
	}
	//  LL_APB2_GRP1_EnableClock(periphs_clock_enable_bit_mask);
	 RCC->APB2ENR |= periphs_clock_enable_bit_mask;//  SET_BIT(RCC->APB2ENR, Periphs);
	 __NOP(); __NOP(); __NOP(); __NOP();
}



