#if !defined CODE_STORE_BSP_OPEN103Z_DEVICE_RENAME_H_
#define CODE_STORE_BSP_OPEN103Z_DEVICE_RENAME_H_

#define LED1_GPIO_PORT  GPIOF
#define LED1_GPIO_PIN  LL_GPIO_PIN_6

#define LED2_GPIO_PORT  GPIOF
#define LED2_GPIO_PIN  LL_GPIO_PIN_7

#define LED3_GPIO_PORT  GPIOF
#define LED3_GPIO_PIN  LL_GPIO_PIN_8

#define LED4_GPIO_PORT  GPIOF
#define LED4_GPIO_PIN  LL_GPIO_PIN_9


#define BUTTON1_GPIO_PORT BUTTON_USER_KEY_PORT
#define BUTTON1_GPIO_PIN  BUTTON_USER_KEY_PIN
#define BUTTON1_GPIO_PULL LL_GPIO_PULL_UP

#define BUTTON_USER_KEY_PORT  GPIOC
#define BUTTON_USER_KEY_PIN  LL_GPIO_PIN_6



/** 	I2C driver														<I2C.c>*/

//I2C1 pins
#define I2C_SW_SCL_PORT	GPIOB
#define I2C_SW_SCL_PIN		LL_GPIO_PIN_6

#define I2C_SW_SDA_PORT	GPIOB
#define I2C_SW_SDA_PIN		LL_GPIO_PIN_7






#define I2C1_SCL_GPIO_PORT	GPIOB
#define I2C1_SCL_GPIO_PIN		LL_GPIO_PIN_6

#define I2C1_SDA_GPIO_PORT	GPIOB
#define I2C1_SDA_GPIO_PIN		LL_GPIO_PIN_7



 
   //TIM3 Channel1 Output pin
  #define TIM3_CHANNEL_1_GPIO_Clock RCC_APB2ENR_IOPCEN
  #define TIM3_CHANNEL_1_Pin 		LL_GPIO_PIN_6
  #define TIM3_CHANNEL_1_Port 		GPIOC

  //TIM3 Channel2 Input pin
  #define TIM3_CHANNEL_2_GPIO_Clock RCC_APB2ENR_IOPAEN
  #define TIM3_CHANNEL_2_Pin 		LL_GPIO_PIN_7
  #define TIM3_CHANNEL_2_Port 		GPIOA

  //TIM4 Channel1 input pin
  #define TIM4_CHANNEL_1_GPIO_Clock RCC_APB2ENR_IOPDEN
  #define TIM4_CHANNEL_1_Pin 		LL_GPIO_PIN_12
  #define TIM4_CHANNEL_1_Port 		GPIOD
	
	
	


#endif /* CODE_STORE_BSP_OPEN103Z_DEVICE_RENAME_H_ */