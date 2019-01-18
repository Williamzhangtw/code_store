#include "cmsis_i2c_test.h"

/*
 * Round 1
 *TRANSMITTER(MASTER): write 'hello' to slave
 *RECEIVER(Slave): receive the data and compare it with 'hello', if no error, turn on green led (led1), else turn on red led(led4)
 *
 * Round 2
 * receiver (master) : write read to slave (read 5 bytes) / compare the data received with 'hello', if no error, turn on led1, else turn on led4
 * transmitter (slave) : send 'hello world' to master
 */

#define TRANSMITTER 0
#define RECEIVER 1

#define TEST1 0
#define TEST2 1



#define CMSIS_I2C_MODE       TRANSMITER /* EEPROM I2C address */
static uint8_t Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength);
const uint8_t TEST_DATA[] = 'hello world';

char receive_buffer[0xf]={0};
uint8_t test = TEST1;
/* I2C driver instance */

static ARM_DRIVER_I2C *I2Cdrv = &Driver_I2C1;

static volatile uint32_t I2C_Event;

/* I2C Signal Event function callback */
void I2C_SignalEvent (uint32_t event) {

  /* Save received events */
  I2C_Event |= event;

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
int32_t EEPROM_Read_Event (uint16_t addr, uint8_t *buf, uint32_t len) {
  uint8_t a[2];

  a[0] = (uint8_t)(addr >> 8);
  a[1] = (uint8_t)(addr & 0xFF);
  /* Clear event flags before new transfer */
  I2C_Event = 0U;

  I2Cdrv->MasterTransmit (EEPROM_I2C_ADDR, a, 2, true);

  /* Wait until transfer completed */
  while ((I2C_Event & ARM_I2C_EVENT_TRANSFER_DONE) == 0U);
  /* Check if all data transferred */
  if ((I2C_Event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) != 0U) return -1;

  /* Clear event flags before new transfer */
  I2C_Event = 0U;

  I2Cdrv->MasterReceive (EEPROM_I2C_ADDR, buf, len, false);

  /* Wait until transfer completed */
  while ((I2C_Event & ARM_I2C_EVENT_TRANSFER_DONE) == 0U);
  /* Check if all data transferred */
  if ((I2C_Event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) != 0U) return -1;

  return 0;
}
/* Read I2C connected EEPROM (pooling example) */
int32_t EEPROM_Read_Pool (uint16_t addr, uint8_t *buf, uint32_t len) {
  uint8_t a[2];

  a[0] = (uint8_t)(addr >> 8);
  a[1] = (uint8_t)(addr & 0xFF);

  I2Cdrv->MasterTransmit (EEPROM_I2C_ADDR, a, 2, true);

  /* Wait until transfer completed */
  while (I2Cdrv->GetStatus().busy);
  /* Check if all data transferred */
  if (I2Cdrv->GetDataCount () != len) return -1;

  I2Cdrv->MasterReceive (EEPROM_I2C_ADDR, buf, len, false);

  /* Wait until transfer completed */
  while (I2Cdrv->GetStatus().busy);
  /* Check if all data transferred */
  if (I2Cdrv->GetDataCount () != len) return -1;

  return 0;
}

/* Initialize I2C connected EEPROM */
int32_t CMSIS_I2C_Initialize (bool pooling) {
  int32_t status;
  uint8_t val;

  if (pooling == true) {
    I2Cdrv->Initialize (NULL);
  } else {
    I2Cdrv->Initialize (I2C_SignalEvent);
  }
  I2Cdrv->PowerControl (ARM_POWER_FULL);
  I2Cdrv->Control      (ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
  I2Cdrv->Control      (ARM_I2C_BUS_CLEAR, 0);

  /* Check if EEPROM can be accessed */
  if (pooling == true) {
    status = EEPROM_Read_Pool (0x00, &val, 1);
  } else {
    status = EEPROM_Read_Event (0x00, &val, 1);
  }
#if CMSIS_I2C_MODE == TRANSMITTER
  I2Cdrv->MasterTransmit (I2C_SLAVE_ADDR, TESTDATA, 5, true);
#endif

#if CMSIS_I2C_MODE == RECEIVER
  I2Cdrv->SlaveReceive(receive_buffer, 5);
  if(Buffercmp8(receive_buffer,TESTDATA,5))
  {
    arm_led->LED_On(LED_RED);
  }
  else
  {
    arm_led->LED_On(LED_GREEN);
  }
#endif

  return (status);
}



// 'LED 2' off indicate the test you are going to execute is task 1: Transmitter(master) & Receiver(slave), press 'KEY' to execute test
// 'LED 2' on indicate the test you are going to execute is task 2: Receiver(master) & Transmitter(slave), press 'KEY'to execute test

void CMSIS_I2C_Test(void)
{
  WaitForButtonPress();
  if(button)
  {
    button = 0;

  }
}


/**
  * @brief  Compares two 8-bit buffers and returns the comparison result.
  * @param  pBuffer1: pointer to the source buffer to be compared to.
  * @param  pBuffer2: pointer to the second source buffer to be compared to the first.
  * @param  BufferLength: buffer's length.
  * @retval 0: Comparison is OK (the two Buffers are identical)
  *         Value different from 0: Comparison is NOK (Buffers are different)
  */
static uint8_t Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return 1;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}
