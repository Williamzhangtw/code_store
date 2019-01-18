#if !defined I2C_EEPROM_H_
 #define I2C_EEPROM_H_
 
#include "bsp.h"
 
//#ifndef TSC_I2C_PORT
//#define TSC_I2C_PORT    1               /* I2C Port number                    */
//#endif
//
//#define TSC_I2C_ADDR    0x41            /* 7-bit I2C address                  */
//
//
//#define _I2C_Driver_(n)  Driver_I2C##n
//#define  I2C_Driver_(n) _I2C_Driver_(n)
//extern ARM_DRIVER_I2C    I2C_Driver_(TSC_I2C_PORT);
//#define ptrI2C         (&I2C_Driver_(TSC_I2C_PORT))
 
 
 int32_t EepromInit (bool pooling) ;
 int32_t EepromReadPool (uint16_t addr, uint8_t *buf, uint32_t len);
 int32_t EepromReadEvent (uint16_t addr, uint8_t *buf, uint32_t len) ;
 void I2cTest(void);
#endif /*I2C_EEPROM_H_*/
