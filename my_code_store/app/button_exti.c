#include "bsp.h"

/*
 * 
 */
 
 

 
void GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  UNUSED(GPIO_Pin);
  arm_i2c1.Control(ARM_I2C_ABORT_TRANSFER,0);
}


//Wakeup button
void EXTI0_IRQHandler(void)
{
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
  GPIO_EXTI_Callback(0);
}
//Key Button
void EXTI9_5_IRQHandler(void)
{


	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
//
  }  
  
}

//Temper Button
void EXTI15_10_IRQHandler(void)
{

	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
  }  
  
 
}
