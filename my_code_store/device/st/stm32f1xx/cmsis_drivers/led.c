/* Includes ------------------------------------------------------------------*/
#include "led.h"
extern void UTILS_GpioClockEnable(GPIO_TypeDef *gpio);

#define ARM_LedDRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)    /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION driver_version = {
  ARM_LED_API_VERSION,
  ARM_LedDRV_VERSION
};



/**
  \fn          ARM_DRV_VERSION I2C_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION LedGetVersion (void) {
  return driver_version;
}



ARM_DRIVER_STATUS LedInitialize (LED_RESOURCES *led_resources) {

  if (led_resources->info->status.initialized == false)
  {
    GPIO_TypeDef * port = led_resources->io.gpio_port;
    uint32_t pin = led_resources->io.gpio_pin;

    UTILS_GpioClockEnable(port);
    // reset the output pin
    LL_GPIO_ResetOutputPin(port, pin);
    // if you don't reset. if the pinOutReg=1. after GPIO has been configured, the pin will output high.

    LL_GPIO_SetPinMode(port, pin,LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(port, pin,LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(port,pin,LL_GPIO_SPEED_FREQ_LOW);

    led_resources->info->status.initialized = true;
    
  }
  return ARM_DRIVER_OK;
}

ARM_DRIVER_STATUS LedUninitialize (LED_RESOURCES *led_resources) {

	if (led_resources->info->status.initialized == true){
    GPIO_TypeDef * port = led_resources->io.gpio_port;
    uint32_t pin = led_resources->io.gpio_pin;
    LL_GPIO_SetPinMode(port,pin,LL_GPIO_MODE_FLOATING);
    led_resources->info->status.initialized =false;

  }
  return ARM_DRIVER_OK;
}
/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LedNUMBER (depend on how many LED on the board)linked to BSP
  */

static ARM_DRIVER_STATUS LedOn(LED_RESOURCES *led_resources){
  GPIO_TypeDef * port = led_resources->io.gpio_port;
  uint32_t pin = led_resources->io.gpio_pin;
  LL_GPIO_SetOutputPin(port, pin);
  return ARM_DRIVER_OK;
}
/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LedNUMBER (depend on how many LED on the board)linked to BSP
  */

static ARM_DRIVER_STATUS LedOff(LED_RESOURCES *led_resources){
  GPIO_TypeDef * port = led_resources->io.gpio_port;
  uint32_t pin = led_resources->io.gpio_pin;
  LL_GPIO_ResetOutputPin(port, pin);
  return ARM_DRIVER_OK;
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LedNUMBER (depend on how many LED on the board)linked to BSP
  * @retval None
  */
static ARM_DRIVER_STATUS LedToggle(LED_RESOURCES *led_resources){
  GPIO_TypeDef * port = led_resources->io.gpio_port;
  uint32_t pin = led_resources->io.gpio_pin;
  	if(LL_GPIO_IsOutputPinSet(port, pin)==0)
	{
			LL_GPIO_SetOutputPin(port, pin);
	}
	else
	{
			LL_GPIO_ResetOutputPin(port,pin);
	}
  return ARM_DRIVER_OK;
}


#if defined USE_LED1

/*  Information (Run-Time) */
static LED_INFO led1_info;

/* LED1 Resources */
 LED_RESOURCES led1_resources = {
  {
    LED1_GPIO_PORT,
    LED1_GPIO_PIN
  },

   &led1_info
};


static ARM_DRIVER_STATUS Led1Initialize ( void) {
  return LedInitialize(&led1_resources);
}
static ARM_DRIVER_STATUS Led1Uninitialize (void) {
  return LedUninitialize(&led1_resources);
}
static ARM_DRIVER_STATUS Led1On (void) {
  return LedOn(&led1_resources);
}
static ARM_DRIVER_STATUS Led1Off (void) {
  return LedOff(&led1_resources);
}
static ARM_DRIVER_STATUS Led1Toggle (void) {
  return LedToggle(&led1_resources);
}

ARM_LED arm_led1 = {
  LedGetVersion,
  Led1Initialize,
  Led1Uninitialize,
  Led1On,
  Led1Off,
  Led1Toggle,
};



#endif

#if defined USE_LED2
/* I2C1 Information (Run-Time) */
static LED_INFO led2_info ;

/* I2C1 Resources */
LED_RESOURCES led2_resources = {
  {
    LED2_GPIO_PORT,
    LED2_GPIO_PIN
  },

   &led2_info
};


static ARM_DRIVER_STATUS Led2Initialize (void) {
  return LedInitialize(&led2_resources);
}
static ARM_DRIVER_STATUS Led2Uninitialize (void) {
  return LedUninitialize(&led2_resources);
}


static ARM_DRIVER_STATUS Led2On (void) {

  return LedOn(&led2_resources);
}

static ARM_DRIVER_STATUS Led2Off (void) {
  return LedOff(&led2_resources);
}

static ARM_DRIVER_STATUS Led2Toggle (void) {
  return LedToggle(&led2_resources);
}
ARM_LED arm_led2 = {
  LedGetVersion,
  Led2Initialize,
  Led2Uninitialize,
  Led2On,
  Led2Off,
  Led2Toggle,
};
#endif



#if defined USE_LED3
/* I2C1 Information (Run-Time) */
static LED_INFO led3_info ;

/* I2C1 Resources */
LED_RESOURCES led3_resources = {
  {
    LED3_GPIO_PORT,
    LED3_GPIO_PIN
  },

   &led3_info
};

static ARM_DRIVER_STATUS Led3Initialize (void) {
  return LedInitialize(&led3_resources);
}

static ARM_DRIVER_STATUS Led3Uninitialize (void) {
  return LedUninitialize(&led3_resources);
}

static ARM_DRIVER_STATUS Led3On (void) {
  return LedOn(&led3_resources);
}

static ARM_DRIVER_STATUS Led3Off (void) {
  return LedOff(&led3_resources);
}

static ARM_DRIVER_STATUS Led3Toggle (void) {
  return LedToggle(&led3_resources);
}

ARM_LED arm_led3 = {
  LedGetVersion,
  Led3Initialize,
  Led3Uninitialize,
  Led3On,
  Led3Off,
  Led3Toggle,
};
#endif

#if defined USE_LED4
/* I2C1 Information (Run-Time) */
static LED_INFO led4_info;
/* I2C1 Resources */
LED_RESOURCES led4_resources = {
  {
    LED4_GPIO_PORT,
    LED4_GPIO_PIN
  },

   &led4_info
};

static ARM_DRIVER_STATUS Led4Initialize (void) {
  return LedInitialize(&led4_resources);
}

static ARM_DRIVER_STATUS Led4Uninitialize (void) {
  return LedUninitialize(&led4_resources);
}

static ARM_DRIVER_STATUS Led4On (void) {
  return LedOn(&led4_resources);
}

static ARM_DRIVER_STATUS Led4Off (void) {
  return LedOff(&led4_resources);
}

static ARM_DRIVER_STATUS Led4Toggle (void) {
  return LedToggle(&led4_resources);
}

ARM_LED arm_led4 = {
  LedGetVersion,
  Led4Initialize,
  Led4Uninitialize,
  Led4On,
  Led4Off,
  Led4Toggle,
};
#endif
