#include "i2c_tp_rit.h"


 uint8_t Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength);


void Slave_Reception_Callback(void);
void Slave_Complete_Callback(void);


volatile uint32_t ubButtonPress = 0;
const uint8_t aLedOn[]           = "LED ON";

/**
  * @brief Variables related to SlaveReceive process
  */
uint8_t      aReceiveBuffer[0xF] = {0};
volatile uint8_t ubReceiveIndex      = 0;

/**
  * @brief Variables related to MasterTransmit process
  */
volatile uint8_t  ubNbDataToTransmit = sizeof(aLedOn);
uint8_t*      pTransmitBuffer    = (uint8_t*)aLedOn;

/**
  * @brief Slave settings
  */
#define SLAVE_OWN_ADDRESS                       0x5A /* This value is a left shift of a real 7 bits of a slave address
                                                        value which can find in a Datasheet as example: b0101101
                                                        mean in uint8_t equivalent at 0x2D and this value can be
                                                        seen in the OAR1 register in bits ADD[1:7] */

/**
  * @brief Master Transfer Request Direction
  */
#define I2C_REQUEST_WRITE                       0x00
#define I2C_REQUEST_READ                        0x01
/* 				 							(I2C1)*/
/* I2C SPEEDCLOCK define to max value: 400 KHz */
#define I2C_SPEEDCLOCK                 400000
#define I2C_DUTYCYCLE            LL_I2C_DUTYCYCLE_2

/**
  * @brief  This function configures I2C1 in Master mode.
  * @note   This function is used to :
  *         -1- Enables GPIO clock.
  *         -2- Enable the I2C1 peripheral clock and configures the I2C1 pins.
  *         -3- Configure I2C1 functional parameters.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void Configure_I2C_Master(void)
{

  /* (1) Enables GPIO clock  **********************/

  /* Enable the peripheral clock of GPIOB */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /* (2) Enable the I2C1 peripheral clock *************************************/

  /* Enable the peripheral clock for I2C1 */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* Configure SCL Pin as : Alternate function, High Speed, Open drain, Pull up */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_OPENDRAIN);

  /* Configure SDA Pin as : Alternate function, High Speed, Open drain, Pull up */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_OPENDRAIN);

  /* (3) Configure I2C1 functional parameters ********************************/
  
  /* Disable I2C1 prior modifying configuration registers */
  LL_I2C_Disable(I2C1);
  
  /* Retrieve Clock frequencies */
  LL_RCC_GetSystemClocksFreq(&rcc_clocks);

  /* Configure the SCL Clock Speed */
  LL_I2C_ConfigSpeed(I2C1, rcc_clocks.PCLK1_Frequency, I2C_SPEEDCLOCK, I2C_DUTYCYCLE);
  
  /* Configure the Own Address1                   */
  /* Reset Values of :
   *     - OwnAddress1 is 0x00
   *     - OwnAddrSize is LL_I2C_OWNADDRESS1_7BIT
   */
  //LL_I2C_SetOwnAddress1(I2C1, 0x00, LL_I2C_OWNADDRESS1_7BIT);

  /* Enable Clock stretching */
  /* Reset Value is Clock stretching enabled */
  //LL_I2C_EnableClockStretching(I2C1);

  
  /* Enable General Call                  */
  /* Reset Value is General Call disabled */
  //LL_I2C_EnableGeneralCall(I2C1);

  /* Configure the 7bits Own Address2     */
  /* Reset Values of :
   *     - OwnAddress2 is 0x00
   *     - Own Address2 is disabled
   */
  //LL_I2C_SetOwnAddress2(I2C1, 0x00);
  //LL_I2C_DisableOwnAddress2(I2C1);

  /* Enable Peripheral in I2C mode */
  /* Reset Value is I2C mode */
  //LL_I2C_SetMode(I2C1, LL_I2C_MODE_I2C);
//void I2C1_Init(void)

  /*
  * Clock enable
  */
//  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
//  /* Add delay related to RCC workaround */
//  while (READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN) != RCC_APB1ENR_I2C1EN) {};


//  /*
//  * GPIO config
//  */
//  LL_APB2_GRP1_EnableClock(I2C1_SCL_CLOCK);
//  LL_APB2_GRP1_EnableClock(I2C1_SDA_CLOCK);
//  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
//  LL_GPIO_SetPinMode(I2C1_SCL_PORT,I2C1_SCL_PIN,LL_GPIO_MODE_ALTERNATE);
//  LL_GPIO_SetPinOutputType(I2C1_SCL_PORT,I2C1_SCL_PIN,LL_GPIO_OUTPUT_OPENDRAIN);
//  LL_GPIO_SetPinSpeed(I2C1_SCL_PORT,I2C1_SCL_PIN,LL_GPIO_SPEED_FREQ_HIGH);

//  LL_GPIO_SetPinMode(I2C1_SDA_PORT,I2C1_SDA_PIN,LL_GPIO_MODE_ALTERNATE);
//  LL_GPIO_SetPinOutputType(I2C1_SDA_PORT,I2C1_SDA_PIN,LL_GPIO_OUTPUT_OPENDRAIN);
//  LL_GPIO_SetPinSpeed(I2C1_SDA_PORT,I2C1_SDA_PIN,LL_GPIO_SPEED_FREQ_HIGH);
////  /* Force the I2C Periheral Clock Reset */
//  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
//__NOP(); __NOP(); __NOP(); __NOP(); 
////  /* Release the I2C Periheral Clock Reset */
//  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);

//  /*I2C Initialization */
//  /* Disable the selected I2Cx Peripheral */
//  LL_I2C_Disable(I2C1);
//  /*
//  * I2C mode, aka standard mode
//  */
//  LL_I2C_SetMode(I2C1,LL_I2C_MODE_I2C);
//  // @note   When disabled the Address 0x00 is NACKed.
//  LL_I2C_DisableGeneralCall(I2C1);
//  LL_I2C_EnableClockStretching(I2C1);

//  LL_I2C_SetClockSpeedMode(I2C1,LL_I2C_CLOCK_SPEED_STANDARD_MODE);

////  //?????????

////  LL_I2C_SetPeriphClock(I2C1,rcc_clocks.PCLK1_Frequency);
////  LL_I2C_SetClockPeriod(I2C1,4);
////  LL_I2C_SetDutyCycle(I2C1,);
////  //100000 is the frequency of SCL
////  LL_I2C_SetRiseTime(I2C1,__LL_I2C_RISE_TIME(__LL_I2C_FREQ_HZ_TO_MHZ(LL_I2C_GetPeriphClock(I2C1)),100000));

//  LL_I2C_ConfigSpeed(I2C1, rcc_clocks.PCLK1_Frequency, 100000, LL_I2C_DUTYCYCLE_2);
//  LL_I2C_SetOwnAddress1(I2C1,0x123,LL_I2C_OWNADDRESS1_7BIT);
//  LL_I2C_SetOwnAddress2(I2C1, 0);
//  LL_I2C_DisableOwnAddress2(I2C1);


//  /* I2C1 interrupt Init */

//  LL_I2C_DisableDMAReq_RX(I2C1);
//  LL_I2C_DisableLastDMA(I2C1);

//  LL_I2C_EnableIT_EVT(I2C1);
//  LL_I2C_EnableIT_ERR(I2C1);
//  //TxE=1|RxNE=1 generate event interrupt
//  LL_I2C_EnableIT_BUF(I2C1);

//  /*
//  * event interrupt
//  */
//  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),14, 0));
//  NVIC_EnableIRQ(I2C1_EV_IRQn);
//  /*
//  * error interrupt
//  */
//  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),14, 0));
//  NVIC_EnableIRQ(I2C1_ER_IRQn);

  LL_I2C_Enable(I2C1);

//  LL_I2C_AcknowledgeNextData(I2C1,LL_I2C_ACK);

 
}




/**
  * @brief  This function configures I2C2 in Slave mode.
  * @note   This function is used to :
  *         -1- Enables GPIO clock.
  *         -2- Enable the I2C2 peripheral clock and configures the I2C2 pins.
  *         -3- Configure NVIC for I2C2.
  *         -4- Configure I2C2 functional parameters.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void Configure_I2C_Slave(void)
{
  /* (1) Enables GPIO clock */
  /* Enable the peripheral clock of GPIOB */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /* (2) Enable the I2C2 peripheral clock *************************************/

  /* Enable the peripheral clock for I2C2 */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

  /* Configure SCL Pin as : Alternate function, High Speed, Open drain, Pull up */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

  /* Configure SDA Pin as : Alternate function, High Speed, Open drain, Pull up */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);

  /* (3) Configure NVIC for I2C2 **********************************************/

  /* Configure Event IT:
   *  - Set priority for I2C2_EV_IRQn
   *  - Enable I2C2_EV_IRQn
   */
  NVIC_SetPriority(I2C2_EV_IRQn, 0);
  NVIC_EnableIRQ(I2C2_EV_IRQn);

  /* Configure Error IT:
   *  - Set priority for I2C2_ER_IRQn
   *  - Enable I2C2_ER_IRQn
   */
  NVIC_SetPriority(I2C2_ER_IRQn, 0);
  NVIC_EnableIRQ(I2C2_ER_IRQn);

  /* (4) Configure I2C2 functional parameters ***********************/

  /* Disable I2C2 prior modifying configuration registers */
  LL_I2C_Disable(I2C2);

  /* Configure the Own Address1 :
   *  - OwnAddress1 is SLAVE_OWN_ADDRESS
   *  - OwnAddrSize is LL_I2C_OWNADDRESS1_7BIT
   */
  LL_I2C_SetOwnAddress1(I2C2, SLAVE_OWN_ADDRESS, LL_I2C_OWNADDRESS1_7BIT);

  /* Enable Clock stretching */
  /* Reset Value is Clock stretching enabled */
  //LL_I2C_EnableClockStretching(I2C2);

  /* Enable General Call                  */
  /* Reset Value is General Call disabled */
  //LL_I2C_EnableGeneralCall(I2C2);

  /* Configure the 7bits Own Address2     */
  /* Reset Values of :
   *     - OwnAddress2 is 0x00
   *     - Own Address2 is disabled
   */
  //LL_I2C_SetOwnAddress2(I2C2, 0x00);
  //LL_I2C_DisableOwnAddress2(I2C2);

  /* Enable Peripheral in I2C mode */
  /* Reset Value is I2C mode */
  //LL_I2C_SetMode(I2C2, LL_I2C_MODE_I2C);
}

void Handle_I2C_Slave(void)
{
  /* (1) Prepare acknowledge for Slave address reception **********************/
  LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);
}

/**
  * @brief  This Function handle Master events to perform a transmission process
  * @note  This function is composed in different steps :
  *        -1- Prepare acknowledge for Master data reception.
  *        -2- Initiate a Start condition to the Slave device.
  *        -3- Loop until Start Bit transmitted (SB flag raised).
  *        -4- Send Slave address with a 7-Bit SLAVE_OWN_ADDRESS for a write request.
  *        -5- Loop until Address Acknowledgement received (ADDR flag raised).
  *        -6- Clear ADDR flag and loop until end of transfer (ubNbDataToTransmit == 0).
  *             -6.1 Transmit data (TXE flag raised).
  *        -7- End of tranfer, Data consistency are checking into Slave process.
  * @param  None
  * @retval None
  */
void Handle_I2C_Master(void)
{
  /* (1) Prepare acknowledge for Master data reception ************************/
//  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

  /* (2) Initiate a Start condition to the Slave device ***********************/
  /* Master Generate Start condition */
  LL_I2C_GenerateStartCondition(I2C1);

  /* (3) Loop until Start Bit transmitted (SB flag raised) ********************/

#if (USE_TIMEOUT == 1)
  Timeout = I2C_SEND_TIMEOUT_SB_MS;
#endif /* USE_TIMEOUT */

  /* Loop until SB flag is raised  */
  while(!LL_I2C_IsActiveFlag_SB(I2C1))
  {
#if (USE_TIMEOUT == 1)
    /* Check Systick counter flag to decrement the time-out value */
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      if(Timeout-- == 0)
      {
        /* Time-out occurred. Set LED2 to blinking mode */
        LED_Blinking(LED_BLINK_SLOW);
      }
    }
#endif /* USE_TIMEOUT */
  }

  /* (4) Send Slave address with a 7-Bit SLAVE_OWN_ADDRESS for a write request */
    LL_I2C_TransmitData8(I2C1, SLAVE_OWN_ADDRESS | I2C_REQUEST_WRITE);

  /* (5) Loop until Address Acknowledgement received (ADDR flag raised) *******/

#if (USE_TIMEOUT == 1)
  Timeout = I2C_SEND_TIMEOUT_ADDR_MS;
#endif /* USE_TIMEOUT */

  /* Loop until ADDR flag is raised  */
  while(!LL_I2C_IsActiveFlag_ADDR(I2C1))
  {
#if (USE_TIMEOUT == 1)
    /* Check Systick counter flag to decrement the time-out value */
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      if(Timeout-- == 0)
      {
        /* Time-out occurred. Set LED2 to blinking mode */
        LED_Blinking(LED_BLINK_SLOW);
      }
    }
#endif /* USE_TIMEOUT */
  }

  /* (6) Clear ADDR flag and loop until end of transfer (ubNbDataToTransmit == 0) */

  /* Clear ADDR flag value in ISR register */
  LL_I2C_ClearFlag_ADDR(I2C1);

#if (USE_TIMEOUT == 1)
  Timeout = I2C_SEND_TIMEOUT_TXE_MS;
#endif /* USE_TIMEOUT */

  /* Loop until TXE flag is raised  */
  volatile uint32_t index =ubNbDataToTransmit ;
  uint8_t*      buffer = pTransmitBuffer  ;
  while(index > 0)
  {
    /* (6.1) Transmit data (TXE flag raised) **********************************/

    /* Check TXE flag value in ISR register */
    if(LL_I2C_IsActiveFlag_TXE(I2C1))
    {
      /* Write data in Transmit Data register.
      TXE flag is cleared by writing data in TXDR register */
      LL_I2C_TransmitData8(I2C1, (*buffer++));

      index--;

#if (USE_TIMEOUT == 1)
      Timeout = I2C_SEND_TIMEOUT_TXE_MS;
#endif /* USE_TIMEOUT */
    }
    

#if (USE_TIMEOUT == 1)
    /* Check Systick counter flag to decrement the time-out value */
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      if(Timeout-- == 0)
      {
        /* Time-out occurred. Set LED2 to blinking mode */
        LED_Blinking(LED_BLINK_SLOW);
      }
    }
#endif /* USE_TIMEOUT */
  }

  /* (7) End of tranfer, Data consistency are checking into Slave process *****/
  /* Generate Stop condition */
  LL_I2C_GenerateStopCondition(I2C1);

}



void I2C1_Transfer_Master_Callback(void)
{
   if(LL_I2C_IsActiveFlag_SB(I2C1))
  {
      LL_I2C_TransmitData8(I2C1, SLAVE_OWN_ADDRESS | I2C_REQUEST_WRITE);
  }
  else if(LL_I2C_IsActiveFlag_ADDR(I2C1))
  {
     LL_I2C_ClearFlag_ADDR(I2C1);
  }
  else if(LL_I2C_IsActiveFlag_BTF(I2C1))
  {
    /* Call function Master Complete Callback */
    if(ubNbDataToTransmit==0)
    {
      ubNbDataToTransmit = sizeof(aLedOn);
      pTransmitBuffer    = (uint8_t*)aLedOn;
      LL_I2C_GenerateStopCondition(I2C1);
    }
    else
    {
    /* Write data in Transmit Data register.
    TXE flag is cleared by writing data in TXDR register */
    LL_I2C_TransmitData8(I2C1, (*pTransmitBuffer++));

    ubNbDataToTransmit--;
    }
  }
  /* Check TXE flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_TXE(I2C1))
  {
    if(ubNbDataToTransmit==0)
    {
      ubNbDataToTransmit = sizeof(aLedOn);
      pTransmitBuffer    = (uint8_t*)aLedOn;
      LL_I2C_GenerateStopCondition(I2C1);
    }
    else
    {
    /* Write data in Transmit Data register.
    TXE flag is cleared by writing data in TXDR register */
    LL_I2C_TransmitData8(I2C1, (*pTransmitBuffer++));

    ubNbDataToTransmit--;
    }
  }

}

/**
  * @brief  This function Activate I2C2 peripheral (Slave)
  * @note   This function is used to :
  *         -1- Enable I2C2.
  *         -2- Enable I2C2 transfer event/error interrupts.
  * @param  None
  * @retval None
  */
void Activate_I2C_Slave(void)
{
  /* (1) Enable I2C2 **********************************************************/
  LL_I2C_Enable(I2C2);

  /* (2) Enable I2C2 transfer event/error interrupts:
   *  - Enable Events interrupts
   *  - Enable Errors interrupts
   */
  LL_I2C_EnableIT_EVT(I2C2);
  LL_I2C_EnableIT_ERR(I2C2);

  LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);
}

/**
  * @brief  This function Activate I2C1 peripheral (Master)
  * @note   This function is used to :
  *         -1- Enable I2C1.
  * @param  None
  * @retval None
  */
void Activate_I2C_Master(void)
{
  /* (1) Enable I2C1 **********************************************************/
  LL_I2C_Enable(I2C1);
}



/**
  * @brief  Function called from I2C IRQ Handler when RXNE flag is set
  *         Function is in charge of retrieving received byte on I2C lines.
  * @param  None
  * @retval None
  */
void Slave_Reception_Callback(void)
{
  /* Read character in Receive Data register.
  RXNE flag is cleared by reading data in RXDR register */
  aReceiveBuffer[ubReceiveIndex++] = LL_I2C_ReceiveData8(I2C2);
}

/**
  * @brief  Function called from I2C IRQ Handler when STOP flag is set
  *         Function is in charge of checking data received,
  *         LED2 is On if data are correct.
  * @param  None
  * @retval None
  */
void Slave_Complete_Callback(void)
{
  /* Check if datas request to turn on the LED2 */
  if(Buffercmp8((uint8_t*)aReceiveBuffer, (uint8_t*)aLedOn, (ubReceiveIndex-1)) == 0)
  {
    /* Turn LED2 On */
    /* Expected bytes have been received */
    /* Slave Rx sequence completed successfully*/
    arm_led2.On();
    ubReceiveIndex =0;
  }
  else
  {
    /* Call Error function */
    Diagnose_Error();
  }
}


/**
  * @brief  Wait for User push-button press to start transfer.
  * @param  None
  * @retval None
  */
  /*  */
void WaitForUserButtonPress(void)
{
  while (ubButtonPress == 0)
  {
	  arm_led3.Toggle();
	  mDelay(100);
  }
  /* Turn LED2 off */
  arm_led3.Off();
}





/**
  * @brief  Compares two 8-bit buffers and returns the comparison result.
  * @param  pBuffer1: pointer to the source buffer to be compared to.
  * @param  pBuffer2: pointer to the second source buffer to be compared to the first.
  * @param  BufferLength: buffer's length.
  * @retval 0: Comparison is OK (the two Buffers are identical)
  *         Value different from 0: Comparison is NOK (Buffers are different)
  */
uint8_t Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength)
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

