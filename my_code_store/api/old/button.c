
/* Includes ------------------------------------------------------------------*/
#include "button.h"

extern  GPIO_TypeDef* const BUTTON_PORT[];

extern const uint32_t BUTTON_PIN[];



void BSP_Button_IT_Falling_Init(Button_TypeDef Button)
{
  #if defined USE_HAL_DRIVER
  ZTW_GPIO_Clock_Enable(BUTTON_PORT[Button]);
  GPIO_InitTypeDef button_gpio;
  
  button_gpio.Pin = BUTTON_PIN[Button];
  button_gpio.Mode = GPIO_MODE_IT_FALLING;
  button_gpio.Pull = GPIO_PULLUP;
  button_gpio.Speed = GPIO_SPEED_FREQ_LOW;
  
  HAL_GPIO_Init(BUTTON_PORT[Button],&button_gpio);

  /* Enable the corresponding Push Button clock */
	
  //uint32_t temp = (uint32_t)POSITION_VAL(BUTTON_PIN[Button]>>GPIO_PIN_MASK_POS);
   uint32_t temp = POSITION_VAL(BUTTON_PIN[Button]);
  if(temp<5)
  {
    NVIC_SetPriority((IRQn_Type)(temp+6), NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    NVIC_EnableIRQ((IRQn_Type)(temp+6));
  }
  else if(temp<10)
  {
    NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    NVIC_EnableIRQ(EXTI9_5_IRQn);
  }
  else
  {
    NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    NVIC_EnableIRQ(EXTI15_10_IRQn);
  }

  #else
  ZTW_GPIO_Clock_Enable(BUTTON_PORT[Button]);
  LL_GPIO_SetPinMode(BUTTON_PORT[Button],BUTTON_PIN[Button],LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(BUTTON_PORT[Button],BUTTON_PIN[Button],LL_GPIO_PULL_UP);
  

  
  uint32_t temp = (uint32_t)POSITION_VAL(BUTTON_PIN[Button]>>GPIO_PIN_MASK_POS);
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  
  
  uint32_t exti_gpio_port =0;
	switch ((uint32_t)BUTTON_PORT[Button])
	{
		case (uint32_t)GPIOA:
			exti_gpio_port = LL_GPIO_AF_EXTI_PORTA;
			break;
		case (uint32_t)GPIOB:
			exti_gpio_port = LL_GPIO_AF_EXTI_PORTB;
			break;
		case (uint32_t)GPIOC:
			exti_gpio_port = LL_GPIO_AF_EXTI_PORTC;
			break;
		case (uint32_t)GPIOD:
			exti_gpio_port = LL_GPIO_AF_EXTI_PORTD;
			break;
		case (uint32_t)GPIOE:
			exti_gpio_port = LL_GPIO_AF_EXTI_PORTE;
			break;
		
		#if defined STM32F103xE
		case (uint32_t)GPIOF:
			exti_gpio_port = LL_GPIO_AF_EXTI_PORTF;
			break;
		case (uint32_t)GPIOG:
			exti_gpio_port = LL_GPIO_AF_EXTI_PORTG;
			break;
		#endif
		default:
			break;
	}
  uint32_t af_exti_line = (temp/4)|(0x000FU << 16U<<((temp%4))*4);
  LL_GPIO_AF_SetEXTISource(exti_gpio_port,af_exti_line);
  

  if(temp<5)
  {
    NVIC_SetPriority((IRQn_Type)(temp+6), NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    NVIC_EnableIRQ((IRQn_Type)(temp+6));
  }
  else if(temp<10)
  {
    NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    NVIC_EnableIRQ(EXTI9_5_IRQn);
  }
  else
  {
    NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    NVIC_EnableIRQ(EXTI15_10_IRQn);
  }
  uint32_t exti_line = 1<<temp;
  LL_EXTI_EnableFallingTrig_0_31(exti_line);
  LL_EXTI_DisableRisingTrig_0_31(exti_line);
  LL_EXTI_EnableIT_0_31(exti_line);  
  
	#endif
  
  
  
  
}

void BSP_Button_IT_Rising_Init(Button_TypeDef Button)
{
#if defined USE_HAL_DRIVER
	ZTW_GPIO_Clock_Enable(BUTTON_PORT[Button]);
  GPIO_InitTypeDef button_gpio;
  
  button_gpio.Pin = BUTTON_PIN[Button];
  button_gpio.Mode = GPIO_MODE_IT_RISING;
  button_gpio.Pull = GPIO_PULLDOWN;
  button_gpio.Speed = GPIO_SPEED_FREQ_LOW;
  
  HAL_GPIO_Init(BUTTON_PORT[Button],&button_gpio);


  /* Enable the corresponding Push Button clock */
	
  uint32_t temp = POSITION_VAL(BUTTON_PIN[Button]);//(uint32_t)POSITION_VAL(BUTTON_PIN[Button]>>GPIO_PIN_MASK_POS);

  if(temp<5)
  {
    NVIC_SetPriority((IRQn_Type)(temp+6), NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    NVIC_EnableIRQ((IRQn_Type)(temp+6));
  }
  else if(temp<10)
  {
    NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    NVIC_EnableIRQ(EXTI9_5_IRQn);
  }
  else
  {
    NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    NVIC_EnableIRQ(EXTI15_10_IRQn);
  }
#else
  ZTW_GPIO_Clock_Enable(BUTTON_PORT[Button]);
  LL_GPIO_SetPinMode(BUTTON_PORT[Button],BUTTON_PIN[Button],LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(BUTTON_PORT[Button],BUTTON_PIN[Button],LL_GPIO_PULL_DOWN);
  
  uint32_t temp = (uint32_t)POSITION_VAL(BUTTON_PIN[Button]>>GPIO_PIN_MASK_POS);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  
  uint32_t exti_gpio_port =0;
	switch ((uint32_t)BUTTON_PORT[Button])
	{
		case (uint32_t)GPIOA:
			exti_gpio_port = LL_GPIO_AF_EXTI_PORTA;
			break;
		case (uint32_t)GPIOB:
			exti_gpio_port = LL_GPIO_AF_EXTI_PORTB;
			break;
		case (uint32_t)GPIOC:
			exti_gpio_port = LL_GPIO_AF_EXTI_PORTC;
			break;
		case (uint32_t)GPIOD:
			exti_gpio_port = LL_GPIO_AF_EXTI_PORTD;
			break;
		case (uint32_t)GPIOE:
			exti_gpio_port = LL_GPIO_AF_EXTI_PORTE;
			break;
		
		#if defined STM32F103xE
		case (uint32_t)GPIOF:
			exti_gpio_port = LL_GPIO_AF_EXTI_PORTF;
			break;
		case (uint32_t)GPIOG:
			exti_gpio_port = LL_GPIO_AF_EXTI_PORTG;
			break;
		#endif
		default:
			break;
	}
  uint32_t af_exti_line = (temp/4)|(0x000FU << 16U<<((temp%4))*4);
  LL_GPIO_AF_SetEXTISource(exti_gpio_port,af_exti_line);
 
  if(temp<5)
  {
    NVIC_SetPriority((IRQn_Type)(temp+6), NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    NVIC_EnableIRQ((IRQn_Type)(temp+6));
  }
  else if(temp<10)
  {
    NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    NVIC_EnableIRQ(EXTI9_5_IRQn);
  }
  else
  {
    NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    NVIC_EnableIRQ(EXTI15_10_IRQn);
  }
  uint32_t exti_line = 1<<temp;
  LL_EXTI_EnableFallingTrig_0_31(exti_line);
  LL_EXTI_DisableRisingTrig_0_31(exti_line);
  LL_EXTI_EnableIT_0_31(exti_line);  
#endif
	
}

void BSP_Button_DeInit(Button_TypeDef Button)
{
#if defined USE_HAL_DRIVER
	uint32_t temp = POSITION_VAL(BUTTON_PIN[Button]);
	LL_GPIO_SetPinMode(BUTTON_PORT[Button], BUTTON_PIN[Button],LL_GPIO_MODE_FLOATING);
	if(temp<5)
	{
		NVIC_DisableIRQ((IRQn_Type)(temp+6));
	}
	else if(temp<10)
	{
		NVIC_DisableIRQ(EXTI9_5_IRQn);
	}
	else
	{
		NVIC_DisableIRQ(EXTI15_10_IRQn);
	}
	uint32_t exti_line = 1<<temp;
	LL_EXTI_DisableIT_0_31(exti_line);
#else
	uint32_t temp = (uint32_t)POSITION_VAL(BUTTON_PIN[Button]>>GPIO_PIN_MASK_POS);
	LL_GPIO_SetPinMode(BUTTON_PORT[Led], BUTTON_PIN[Led],LL_GPIO_MODE_FLOATING);
	if(temp<5)
	{
		NVIC_DisableIRQ((IRQn_Type)(temp+6));
	}
	else if(temp<10)
	{
		NVIC_DisableIRQ(EXTI9_5_IRQn);
	}
	else
	{
		NVIC_DisableIRQ(EXTI15_10_IRQn);
	}
	uint32_t exti_line = 1<<temp;
	LL_EXTI_DisableIT_0_31(exti_line);
#endif
}

/**
  \fn          int32_t Buttons_DeInitialize (void)
  \brief       De-initialize buttons
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t Buttons_Deinitialize (void) {
	for(uint32_t index =0;index<BUTTONn;index++)
	{
		BSP_Button_DeInit(index);
	}
  return 0;
}

/**
  * @brief  Returns the selected button state.
  * @param  Button: Button to be checked.
  *   This parameter can be one of the following values:
  *     @arg BUTTON_TAMPER: Key/Tamper Push Button 
  * @retval Button state
  */
uint32_t BSP_BUTTON_GetState(Button_TypeDef Button)
{
	return LL_GPIO_IsInputPinSet(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}


/**
  \fn          uint32_t Buttons_GetCount (void)
  \brief       Get number of available buttons
  \return      Number of available buttons
*/
uint32_t Buttons_GetCount (void) {
  return BUTTONn;
}

  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
