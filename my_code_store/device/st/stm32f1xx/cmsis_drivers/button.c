/* Includes ------------------------------------------------------------------*/
#include "./button.h"


extern void UTILS_GpioClockEnable(GPIO_TypeDef *gpio);

#define ARM_BUTTON_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)    /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION driver_version = {
  ARM_BUTTON_API_VERSION,
  ARM_BUTTON_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_BUTTON_CAPABILITIES driver_capabilities = { 
  0U                    // Does not support 10-bit addressing
};

/**
  \fn          ARM_DRV_VERSION I2C_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION ButtonGetVersion (void) {
  return driver_version;
}

/**
  \fn          ARM_I2C_CAPABILITIES I2C_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_I2C_CAPABILITIES
*/
static ARM_BUTTON_CAPABILITIES ButtonGetCapabilities (void) {
  return driver_capabilities;
}

int32_t ButtonInitialize(BUTTON_RESOURCES* button_resources,ARM_BUTTON_SIGNAL_EVENT cb_event)
{
  if((button_resources->info->flags& BUTTON_INIT) ==0)
  {
    GPIO_TypeDef *port = button_resources->io.gpio_port;
    uint32_t pin = button_resources->io.gpio_pin;
    uint8_t pull = button_resources->io.pull;

    UTILS_GpioClockEnable(port);
    LL_GPIO_SetPinMode(port,pin,LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(port,pin,pull);

    uint32_t temp = (uint32_t)POSITION_VAL(pin>>GPIO_PIN_MASK_POS);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);

    uint32_t exti_gpio_port =0;
    switch ((uint32_t)port)
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
      #if defined GPIOE
      case (uint32_t)GPIOE:
        exti_gpio_port = LL_GPIO_AF_EXTI_PORTE;
        break;
      #endif

      #if defined GPIOF
      case (uint32_t)GPIOF:
        exti_gpio_port = LL_GPIO_AF_EXTI_PORTF;
        break;
      #endif
      #if defined GPIOG
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

    if (pull==LL_GPIO_PULL_UP)
    {
      LL_EXTI_EnableFallingTrig_0_31(exti_line);
      LL_EXTI_DisableRisingTrig_0_31(exti_line);
    }
    else
    {
      LL_EXTI_EnableRisingTrig_0_31(exti_line);
      LL_EXTI_DisableFallingTrig_0_31(exti_line);
    }

    LL_EXTI_EnableIT_0_31(exti_line);

      button_resources->info->flags  = 0U;
      button_resources->info->cb_event=cb_event;
  }
  return ARM_DRIVER_OK;
}
/**
  \fn          int32_t Buttons_DeInitialize (void)
  \brief       De-initialize buttons
  \returns
   - \b  0: function succeeded
   - \b -1: function faibutton
*/
int32_t ButtonUninitialize(BUTTON_RESOURCES *button_resources)
{
  if(button_resources->info->flags& BUTTON_INIT)
  {
    GPIO_TypeDef *port = button_resources->io.gpio_port;
    uint32_t pin = button_resources->io.gpio_pin;

    uint32_t temp = POSITION_VAL(pin);
    LL_GPIO_SetPinMode(port, pin,LL_GPIO_MODE_FLOATING);
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
    button_resources->info->flags &= ~BUTTON_INIT;
    button_resources->info->cb_event=NULL;
  }
	return ARM_DRIVER_OK;
}

/**
  * @brief  Returns the selected button state.
  * @param  Button: Button to be checked.
  *   This parameter can be one of the following values:
  *     @arg BUTTON_TAMPER: Key/Tamper Push Button 
  * @retval Button state
  */
BUTTON_STATE ButtonGetState(BUTTON_RESOURCES *button_resources)
{
  GPIO_TypeDef *port = button_resources->io.gpio_port;
  uint32_t pin = button_resources->io.gpio_pin;
  uint8_t pull = button_resources->io.pull;
  uint32_t result = LL_GPIO_IsInputPinSet(port, pin);
 if(pull == 0)//aka if(pull ==LL_GPIO_PULL_Down)
 {
    if (result ==PRESSED)
    {
      result = RELEASED;
    }
    else
    {
      result = PRESSED;
    }
 }
  return result;
}



#if defined USE_BUTTON1

/* I2C1 Information (Run-Time) */
static BUTTON_INFO button1_info = {
  NULL,
  0U,
};

/* I2C1 Resources */

BUTTON_RESOURCES button1_resources= {
  {
    BUTTON1_GPIO_PORT,
    BUTTON1_GPIO_PIN,
    BUTTON1_GPIO_PULL,
  },
  &button1_info,
};

static int32_t Button1Initialize(ARM_BUTTON_SIGNAL_EVENT cb_event)
{
  return ButtonInitialize(&button1_resources,cb_event);
}

static int32_t Button1Uninitialize(void){
  return ButtonUninitialize(&button1_resources);
}

static BUTTON_STATE Button1GetState(void)
{
  return ButtonGetState(&button1_resources);
}
ARM_BUTTON arm_button1 =
{
	ButtonGetVersion,
	ButtonGetCapabilities,
	Button1Initialize,
	Button1Uninitialize,
	Button1GetState,
};

  
#endif

#if defined USE_BUTTON2

/* I2C1 Information (Run-Time) */
static BUTTON_INFO button2_info = {
  NULL,
  0U,
};

/* I2C1 Resources */

BUTTON_RESOURCES button2_resources= {
  {
    BUTTON2_GPIO_PORT,
    BUTTON2_GPIO_PIN,
    BUTTON2_GPIO_PULL,
  },
  &button2_info,
};

static int32_t Button2Initialize(ARM_BUTTON_SIGNAL_EVENT cb_event)
{
  return ButtonInitialize(&button2_resources,cb_event);
}

static int32_t Button2Uninitialize(void){
  return ButtonUninitialize(&button2_resources);
}

static BUTTON_STATE Button2GetState(void)
{
  return ButtonGetState(&button2_resources);
}
ARM_BUTTON arm_button2 =
{
	ButtonGetVersion,
	ButtonGetCapabilities,
	Button2Initialize,
	Button2Uninitialize,
	Button2GetState,
};

  
#endif

#if defined USE_BUTTON3

/* I2C1 Information (Run-Time) */
static BUTTON_INFO button3_info = {
  NULL,
  0U,
};

/* I2C1 Resources */

BUTTON_RESOURCES button3_resources= {
  {
    BUTTON3_GPIO_PORT,
    BUTTON3_GPIO_PIN,
    BUTTON3_GPIO_PULL,
  },
  &button3_info,
};

static int32_t Button3Initialize(ARM_BUTTON_SIGNAL_EVENT cb_event)
{
  return ButtonInitialize(&button3_resources,cb_event);
}

static int32_t Button3Uninitialize(void){
  return ButtonUninitialize(&button3_resources);
}

static BUTTON_STATE Button3GetState(void)
{
  return ButtonGetState(&button3_resources);
}
ARM_BUTTON arm_button3 =
{
	ButtonGetVersion,
	ButtonGetCapabilities,
	Button3Initialize,
	Button3Uninitialize,
	Button3GetState,
};

  
#endif




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
