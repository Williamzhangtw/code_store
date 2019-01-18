#include "i2c_eeprom.h"

#define EEPROM_I2C_ADDR       0x51      /* EEPROM I2C address */
const uint8_t MESSAGE[] = "HELLO";
uint8_t buffer[15]={0};
/* I2C driver instance */

static ARM_DRIVER_I2C *i2c_drv = &arm_i2c1;

static volatile uint32_t i2c_event;

/* I2C Signal Event function callback */
void I2cSignalEvent (uint32_t event) {

  /* Save received events */
  i2c_event |= event;

  /* Optionally, user can define specific actions for an event */

  if (event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) {
    /* Less data was transferred than requested */
  }

  if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
    /* Transfer or receive is finished */
  }

  if (event & ARM_I2C_EVENT_ADDRESS_NACK) {
    /* Slave address was not acknowledged */
  }

  if (event & ARM_I2C_EVENT_ARBITRATION_LOST) {
    /* Master lost bus arbitration */
  }

  if (event & ARM_I2C_EVENT_BUS_ERROR) {
    /* Invalid start/stop position detected */
  }

  if (event & ARM_I2C_EVENT_BUS_CLEAR) {
    /* Bus clear operation completed */
  }

  if (event & ARM_I2C_EVENT_GENERAL_CALL) {
    /* Slave was addressed with a general call address */
  }

  if (event & ARM_I2C_EVENT_SLAVE_RECEIVE) {
    /* Slave addressed as receiver but SlaveReceive operation is not started */
  }

  if (event & ARM_I2C_EVENT_SLAVE_TRANSMIT) {
    /* Slave addressed as transmitter but SlaveTransmit operation is not started */
  }
}
/* Read I2C connected EEPROM (event driven example) */
int32_t EepromReadEvent (uint16_t addr, uint8_t *buf, uint32_t len) {
  uint8_t a[2];

  a[0] = (uint8_t)(addr >> 8);
  a[1] = (uint8_t)(addr & 0xFF);
  /* Clear event flags before new transfer */
  i2c_event = 0U;

  i2c_drv->MasterTransmit (EEPROM_I2C_ADDR, a, 2, true);

  /* Wait until transfer completed */
  while ((i2c_event & ARM_I2C_EVENT_TRANSFER_DONE) == 0U);
  /* Check if all data transferred */
  if ((i2c_event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) != 0U) return -1;

  /* Clear event flags before new transfer */
  i2c_event = 0U;

  i2c_drv->MasterReceive (EEPROM_I2C_ADDR, buf, len, false);

  /* Wait until transfer completed */
  while ((i2c_event & ARM_I2C_EVENT_TRANSFER_DONE) == 0U);
  /* Check if all data transferred */
  if ((i2c_event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) != 0U) return -1;

  return 0;
}
/* Read I2C connected EEPROM (pooling example) */
int32_t EEPROM_Read_Pool (uint16_t addr, uint8_t *buf, uint32_t len) {
  uint8_t a[2];

  a[0] = (uint8_t)(addr >> 8);
  a[1] = (uint8_t)(addr & 0xFF);

  i2c_drv->MasterTransmit (EEPROM_I2C_ADDR, a, 2, true);

  /* Wait until transfer completed */
  while (i2c_drv->GetStatus().busy);
  /* Check if all data transferred */
  if (i2c_drv->GetDataCount () != len) return -1;

  i2c_drv->MasterReceive (EEPROM_I2C_ADDR, buf, len, false);

  /* Wait until transfer completed */
  while (i2c_drv->GetStatus().busy);
  /* Check if all data transferred */
  if (i2c_drv->GetDataCount () != len) return -1;
   

  return 0;
}
void I2cTest(void)
{
//   i2c_drv->MasterTransmit (EEPROM_I2C_ADDR, MESSAGE, 5, true);
//    /* Wait until transfer completed */
//    while (i2c_drv->GetStatus().busy);
//
//    i2c_drv->MasterReceive (EEPROM_I2C_ADDR, buffer, 5, false);
//    /* Wait until transfer completed */
//    while (i2c_drv->GetStatus().busy);

    i2c_drv->SlaveTransmit(MESSAGE,5);
    /* Wait until transfer completed */
    while (i2c_drv->GetStatus().busy);

//    i2c_drv->SlaveReceive(buffer,5);
//    while (i2c_drv->GetStatus().busy);
}


/* Initialize I2C connected EEPROM */
int32_t EepromInit (bool pooling) {
  int32_t status=0;

  if (pooling == true) {
    i2c_drv->Initialize (NULL);
  } else {
    i2c_drv->Initialize (I2cSignalEvent);
  }
  i2c_drv->PowerControl (ARM_POWER_FULL);
  i2c_drv->Control      (ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
  i2c_drv->Control      (ARM_I2C_BUS_CLEAR, 0);
//  i2c_drv->Control(ARM_I2C_ABORT_TRANSFER,0);


//  /* Check if EEPROM can be accessed */
//  if (pooling == true) {
//    status = EEPROM_Read_Pool (0x00, &val, 1);
//  } else {
//    status = EEPROM_Read_Event (0x00, &val, 1);
//  }

  return (status);
}
