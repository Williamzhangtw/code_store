#include "button_exti.h"

/*
 * 
 */
 
void Button_EXTI_Init(void)
{
  BSP_Button_IT_Falling_Init(BUTTON_USER_KEY);
}
 
 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  UNUSED(GPIO_Pin);

  if(LL_GPIO_IsInputPinSet(BUTTON_USER_KEY_PORT,BUTTON_USER_KEY_PIN))
  { 
    BSP_LED_Toggle(LED4);
  }
}
