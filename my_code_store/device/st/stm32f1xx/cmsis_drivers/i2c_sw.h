/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#include "device_header.h"


#define SCL BITBAND_ADDR(I2C_SW_SCL_GPIO_PORT,I2C_SW_SCL_GPIO_PIN)
#define SDA BITBAND_ADDR(I2C_SW_SDA_GPIO_PORT,I2C_SW_SDA_GPIO_PIN)

#define SCL_IN()  	SCL_read()
#define SDA_IN()  	SDA_read()
#define SCL_HIGH()  SCL_OutputHigh()
#define SDA_HIGH()  SDA_OutputHigh()
#define SCL_LOW()  	SCL_OutputLow()
#define SDA_LOW()  	SDA_OutputLow()



void SCL_OutputHigh(void);
void SDA_OutputHigh(void);
void SCL_OutputLow(void);
void SDA_OutputLow(void);
void I2C_SW_Start(void);
void I2C_SW_Stop(void);

void I2C_SW_Init(void);
uint32_t SCL_read(void);
uint32_t SDA_read(void);
uint32_t I2C_SW_Receive(uint32_t ack);
uint32_t I2C_SW_Transmit(uint32_t data);
/*
 * extern function indicate the function is defined in other C file
 */

extern void uDelay(uint32_t);


#endif


