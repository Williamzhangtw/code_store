/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2018 Arm Limited
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * $Date:        5. February 2018
 * $Revision:    V2.8
 *
 * Driver:       arm_i2c1, arm_i2c2, arm_i2c3
 * Configured:   via device_config.h & rename_device.h
 * Project:      CMSIS I2C Driver for ST STM32F1xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                 Value   I2C Interface
 *   ---------------------                 -----   -------------
 *   Connect to hardware via arm_i2c# = 1       use I2C1
 *   Connect to hardware via arm_i2c# = 2       use I2C2
 *   Connect to hardware via arm_i2c# = 3       use I2C3
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 2.8
 *    Added support for ARM Compiler 6
 *  Version 2.7
 *    Slave event handling reworked.
 *  Version 2.6
 *    Corrected rise time setting when using ARM_I2C_BUS_SPEED_FAST
 *  Version 2.5
 *    Corrected PowerControl function for:
 *      - Unconditional Power Off
 *      - Conditional Power full (driver must be initialized)
 *  Version 2.4
 *    Added support for STM32F410xx
 *    Corrected 3 byte reception and POS bit handling in master mode
 *    Corrected acknowledge handling in slave mode
 *  Version 2.3
 *    Updated initialization, uninitialization and power procedures
 *    Added support for STM32F446xx
 *    Limitation of FREQ[5:0] bits in I2C->CR2 set to 50MHz
 *  Version 2.2
 *    Corrected transfer issues after ARM_I2C_EVENT_ADDRESS_NACK.
 *    Corrected slave address parameter checking.
 *  Version 2.1
 *    Corrected 10-bit addressing mode
 *    Slave operation mode issues fixed
 *    STM32CubeMX generated code can also be used to configure the driver.
 *  Version 2.0
 *    Updated to the CMSIS Driver API V2.02
 *  Version 1.2
 *    Bugfix (corrected I2C register access)
 *  Version 1.1
 *    Based on API V1.10 (namespace prefix ARM_ added)
 *  Version 1.0
 *    Initial release
 */

/*! \page stm32f1_i2c CMSIS-Driver I2C Setup

The CMSIS-Driver I2C requires:
  - Setup of I2Cx input clock
  - Setup of I2Cx in I2C mode with optional DMA for Rx and Tx transfers

Valid settings for various evaluation boards are listed in the table below:

Peripheral Resource | stm32f107         | STM32F4-Discovery | 32F401C-Discovery | 32F429I-Discovery
:-------------------|:------------------|:------------------|:------------------|:------------------
I2C Mode            | I2C1: <b>I2C</b>  | I2C1: <b>I2C</b>  | I2C1: <b>I2C</b>  | I2C3: <b>I2C</b>
SCL pin             | PB6               | PB6               | PB6               | PA8
SDA pin             | PB7               | PB9               | PB9               | PC9

For different boards, refer to the hardware schematics to reflect correct setup values.

The STM32CubeMX configuration for MCBSTM32F400 with steps for Pinout, Clock, and System Configuration are
listed below. Enter the values that are marked \b bold.

Pinout tab
----------
  1. Configure mode
    - Peripherals \b I2C1: Mode=<b>I2C</b>

Clock Configuration tab
-----------------------
  1. Configure APB1 clock
    - Setup "APB1 peripheral clocks (MHz)" to match application requirements

Configuration tab
-----------------
  1. Under Connectivity open \b I2C1 Configuration:
     - \e optional <b>DMA Settings</b>: setup DMA transfers for Rx and Tx\n
       \b Add - Select \b I2C1_RX: Stream=DMA1 Stream 0, Direction=Peripheral to Memory, Priority=Low
          DMA Request Settings         | Label             | Peripheral | Memory
          :----------------------------|:------------------|:-----------|:-------------
          Mode: Normal                 | Increment Address | OFF        |\b ON
          Use Fifo OFF Threshold: Full | Data Width        |\b Byte     | Byte
          .                            | Burst Size        | Single     | Single
       \b Add - Select \b I2C1_TX: Stream=DMA1 Stream 6, Direction=Memory to Peripheral, Priority=Low
          DMA Request Settings         | Label             | Peripheral | Memory
          :----------------------------|:------------------|:-----------|:-------------
          Mode: Normal                 | Increment Address | OFF        |\b ON
          Use FIFO OFF Threshold: Full | Data Width        |\b Byte     | Byte
          .                            | Burst Size        | Single     | Single

     - <b>GPIO Settings</b>: review settings, no changes required
          Pin Name | Signal on Pin | GPIO mode | GPIO Pull-up/Pull..| Maximum out | User Label
          :--------|:--------------|:----------|:-------------------|:------------|:----------
          PB8      | I2C1_SCL      | Alternate | Pull-up            | High        |.
          PB9      | I2C1_SDA      | Alternate | Pull-up            | High        |.

     - <b>NVIC Settings</b>: enable interrupts
          Interrupt Table                      | Enable | Preemption Priority | Sub Priority
          :------------------------------------|:-------|:--------------------|:--------------
          DMA1 stream0 global interrupt        |   ON   | 0                   | 0
          DMA1 stream6 global interrupt        |   ON   | 0                   | 0
          I2C1 event interrupt                 |\b ON   | 0                   | 0
          I2C1 error interrupt                 |\b ON   | 0                   | 0

     - Parameter Settings: not used
     - User Constants: not used

     Click \b OK to close the I2C1 Configuration dialog
*/

/*! \cond */

/*
 * Mode selection
 *  - master transmitter
 *  - master receiver
 *  - slave receiver
 *  - slave transmitter
 *
 *  By default, I2C operate in slave mode. The interface automatically switches from slave to
 *  master,after it generates a START condition and from master to slave, if an arbitration
 *  loss or a STOP generation occurs, allowing multi-master capability.
 *
 *  Communication flow:
 *          [master mode]
 *            the I2C interface initiates a data transfer and generates the clock signal.A serial
 *            data transfer always begins with a start condition and ends with a stop condition.
 *            Both start and stop condition are generated by software in master mode. Master mode
 *            is selected as soon as Start condition is generated.
 *            - Program the peripheral clock
 *            - configure the clock control registers
 *            - configure the rise time register
 *            - enable I2C
 *            - Set Start bit
 *            - When BUSY bit is cleared, interface switch to Master(MSL bit set)
 *            - When already in master mode, the interface generate a Re-start condition at the
 *              end of current byte transfer.
 *            - Start condition sent
 *            - [SB event/interrupt]
 *            - cleared by reading SR1 followed by writing DR with slave address
 *            - [ADDR event/interrupt]
 *            - cleared by reading SR1 followed by reading SR2
 *            [master transmitter]
 *            - the master waits until first byte is written into DR
 *            - when receive ACK from slave, [TxE event/interrupt]
 *            - TxE is cleared by writing to DR
 *            - ...
 *            - if TxE is set and DR was not written before the end of next data transmission, BTF
 *              is set and the interface waits until BTF is cleared by a read from SR1 followed by a
 *              write to DR, stretching SCL low.
 *            - last byte is written to the DR
 *            - [TxE event/interrupt] & [BTF event/interrupt]
 *            - the stop bit is set by software to generate a stop condition. TxE and BTF are cleared
 *              by the stop condition. interface automatically goes back to slave mode(MSL bit cleared)
 *            [master receiver]
 *            - if one byte is received, ACK
 *            - [RxNE event/interrupt]
 *            - RxNE is cleared by reading DR
 *            - if RxNE bit is set and the DR is not read before the next byte reception, the BTF bit is set and interface waits
 *              until BTF is cleared by a read in the SR1 followed by a read in the DR register, stretching SCL low.
 *            - closing the communication
 *              - Method 1(for I2C interrupt with Highest priority)
 *                - [RxNE event/interrupt] before last byte
 *                - reset ACK and set STOP bit
 *                - (after slave receiving NACK, it releases the control of SCL and SDA)
 *                - send a stop condition
 *                - [RxNE event/interrupt]
 *                - clear by reading DR
 *             - Method 2(polling or low I2C interrupt priority)
 *                (when receiving number is >2)
 *                - last third data is available to read, but we don't read.
 *                - last second data is available [RxNE event/interrupt], don't read
 *                - [BTF event/interrupt]
 *                - reset ACK bit and set STOP bit, read DR
 *                (when receiving number is 2)
 *                - Set POS bit and reset ACK bit
 *                - [ADDR event/interrupt]
 *                - clear ADDR, reset
 *                - [BTF event/interrupt]
 *                - Set stop bit
 *                - [RxNE event/interrupt]
 *                - clear by read DR
 *                (when receiving number is 1)
 *                - [ADDR event/interrupt]
 *                - clear ADDR, Reset ACK and Set stop bit
 *                - [RxNE event/interrupt]
 *                - clear by read DR
 *
 *
 *          [slave mode] (enable EVT_IT, Set ACK,
 *            - a start condition is detected
 *            - the address is received from the SDA line
 *            - compared with OAR1 or OAR2 or General Call Address.
 *            - header/address not matched. the I2C ignores it and waits for another Start condition.
 *            - header matched, ACK and wait for another 8 bit data(10 bit address).
 *            - address matched,
 *                - ACK
 *                - [ADDR event/interrupt]
 *                  - clear by reading SR1 followed by reading SR2
 *                - (if ENDUAL=1) Check OAR1 and OAR2 to know which is acknowledge
 *              [Slave transmitter]
 *                - the slave stretch SCL low until ADDR is clear and DR has been written
 *                - [TxE event/interrupt]
 *                - TxE clear when shift register not empty and a write to DR
 *                - a acknowledge pulse is received.
 *                - [TxE event/interrupt]
 *                - if TxE is set and not written DR before the end of the next data transmission,
 *                  the BTF bit is set and the interface waits until BTF is cleared by a read to SR1
 *                  followed by a write to DR, stretching SCL low
 *                  [BTF event/interrupt]
 *              [Slave receiver]
 *                - receive bytes from the SDA line into DR register.
 *                - When DR is full
 *                  - ACK
 *                  - [RxNE event/interrupt]
 *                  - RxNE is cleared by reading DR
 *                  - if RxNE is set and not read DR before the end of the next data reception, the
 *                    BTF bit is set and the interface waits until BTF is cleared by a read from SR1
 *                    followed by a read to DR, stretching SCL low
 *                    [BTF event/interrupt]
 *            - after last data byte is transferred a Stop condition is generated by master.
 *            - slave detects stop condition
 *            - [STOPF event/interrupt]
 *            - STOPF is clear by a read of SR1 followed by a write to CR1
 *
 *    @add  TRA bit indicates whether the slave is in Receiver or Transmitter mode, and is decided by last bit of address
 *
 *
 */

#include "i2c.h"


extern void UTILS_MilliSecondDelay(uint32_t n_m_second);
extern void UTILS_GpioClockEnable(GPIO_TypeDef *gpio);

#define ARM_I2C_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,8)    /* driver version */

#if defined(MX_I2C1_RX_DMA_Instance) && defined(MX_I2C1_TX_DMA_Instance)
	#if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
	static DMA_HandleTypeDef hdma_i2c1_rx;
	#else
	extern DMA_HandleTypeDef hdma_i2c1_rx;
	#endif


	#if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
		static DMA_HandleTypeDef hdma_i2c1_tx;
	#else
		extern DMA_HandleTypeDef hdma_i2c1_tx;
	#endif

	static void I2C1_RX_DMA_Complete(DMA_HandleTypeDef *hdma);
	static void I2C1_RX_DMA_Error   (DMA_HandleTypeDef *hdma);
	static void I2C1_TX_DMA_Complete(DMA_HandleTypeDef *hdma);
	static void I2C1_TX_DMA_Error   (DMA_HandleTypeDef *hdma);
#endif

#if defined(MX_I2C2_RX_DMA_Instance) && defined(MX_I2C2_TX_DMA_Instance)
  #if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
  static DMA_HandleTypeDef hdma_i2c2_rx;
  #else
  extern DMA_HandleTypeDef hdma_i2c2_rx;
  #endif

  #if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
  static DMA_HandleTypeDef hdma_i2c2_tx;
  #else
  extern DMA_HandleTypeDef hdma_i2c2_tx;
  #endif

  static void I2C2_RX_DMA_Complete(DMA_HandleTypeDef *hdma);
  static void I2C2_RX_DMA_Error   (DMA_HandleTypeDef *hdma);
  static void I2C2_TX_DMA_Complete(DMA_HandleTypeDef *hdma);
  static void I2C2_TX_DMA_Error   (DMA_HandleTypeDef *hdma);
#endif

#if defined(MX_I2C3_RX_DMA_Instance) && defined(MX_I2C3_TX_DMA_Instance)
  #if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
  static DMA_HandleTypeDef hdma_i2c3_rx;
  #else
  extern DMA_HandleTypeDef hdma_i2c3_rx;
  #endif

  #if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
  static DMA_HandleTypeDef hdma_i2c3_tx;
  #else
  extern DMA_HandleTypeDef hdma_i2c3_tx;
  #endif

  static void I2C3_RX_DMA_Complete(DMA_HandleTypeDef *hdma);
  static void I2C3_RX_DMA_Error   (DMA_HandleTypeDef *hdma);
  static void I2C3_TX_DMA_Complete(DMA_HandleTypeDef *hdma);
  static void I2C3_TX_DMA_Error   (DMA_HandleTypeDef *hdma);
#endif


/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_I2C_API_VERSION,
  ARM_I2C_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_I2C_CAPABILITIES DriverCapabilities = { 
  0U                    // Does not support 10-bit addressing
};

/*
 * @belief    Get driver version.
 * @retval    ARM_DRV_VERSION
 */
static ARM_DRIVER_VERSION I2CX_GetVersion (void) {
  return (DriverVersion);
}

/*
 * @belief    Get driver capabilities.
 * @retval    ARM_I2C_CAPABILITIES
 */
static ARM_I2C_CAPABILITIES I2CX_GetCapabilities (void) {

  return (DriverCapabilities);
}

/*
 * @belief    Initialize The GPIO of I2C Interface.
 * @param     i2c_resource
 *      @arg  arm_i2c1
 *      @arg  arm_i2c2
 *      @arg  arm_i2c3
 * @retval    execution_status
 */
static ARM_DRIVER_STATUS I2C_Initialize (ARM_I2C_SignalEvent_t cb_event, I2C_RESOURCES *i2c_resource) {

  // if i2c already had been initialized
  if (i2c_resource->info->status.initialized ==true ) { return ARM_DRIVER_OK; }

    /* Setup I2C pin configuration */

    /* Configure SCL Pin */
  UTILS_GpioClockEnable (i2c_resource->io.scl_port);
  LL_GPIO_SetPinMode(i2c_resource->io.scl_port,i2c_resource->io.scl_pin,LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinOutputType(i2c_resource->io.scl_port,i2c_resource->io.scl_pin,LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinSpeed(i2c_resource->io.scl_port,i2c_resource->io.scl_pin,LL_GPIO_SPEED_FREQ_MEDIUM);
  /* Configure SDA Pin */
  UTILS_GpioClockEnable (i2c_resource->io.sda_port);
  LL_GPIO_SetPinMode(i2c_resource->io.sda_port,i2c_resource->io.sda_pin,LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinOutputType(i2c_resource->io.sda_port,i2c_resource->io.sda_pin,LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinSpeed(i2c_resource->io.sda_port,i2c_resource->io.sda_pin,LL_GPIO_SPEED_FREQ_MEDIUM);

//    __HAL_AFIO_REMAP_I2C1_ENABLE();


  /* Reset Run-Time information structure */
  memset (i2c_resource->info, 0x00, sizeof (I2C_INFO));
//  i2c_resource->h->Instance = i2c_resource->reg;
  i2c_resource->info->cb_event = cb_event;
  i2c_resource->info->status.initialized  = true;
  return (ARM_DRIVER_OK);
}

/*
 * @belief    De-initialize I2C Interface's GPIO.
 * @param     i2c_resource
 *      @arg  arm_i2c1
 *      @arg  arm_i2c2
 *      @arg  arm_i2c3
 * @retval    execution_status
 */
static ARM_DRIVER_STATUS I2C_Uninitialize (I2C_RESOURCES *i2c_resource) {

  /* Unconfigure SCL and SDA Pins */
  if (i2c_resource->info->status.initialized ==false ) { return ARM_DRIVER_OK; }

  LL_GPIO_SetPinMode(i2c_resource->io.scl_port,i2c_resource->io.scl_pin,LL_GPIO_MODE_FLOATING);
  LL_GPIO_SetPinMode(i2c_resource->io.sda_port,i2c_resource->io.sda_pin,LL_GPIO_MODE_FLOATING);

  LL_GPIO_AF_DisableRemap_I2C1();
  //  if (i2c_resource->dma_rx != NULL) { i2c_resource->dma_rx->h->Instance = NULL; }
  //  if (i2c_resource->dma_tx != NULL) { i2c_resource->dma_tx->h->Instance = NULL; }

//  i2c_resource->h->Instance = NULL;
  i2c_resource->info->status.initialized = false;
  return (ARM_DRIVER_OK);
}

/*
 * @belief    Peripheral power control
 * @param     state Power state
 *      @arg  ARM_POWER_OFF   Shutdown the peripheral
 *      @arg  ARM_POWER_LOW   Set peripheral in battery saver mode
 *      @arg  ARM_POWER_FULL  Set peripheral in standard mode
 * @param     i2c_resource
 *      @arg  arm_i2c1
 *      @arg  arm_i2c2
 *      @arg  arm_i2c3
 * @retval
 * @note      By default, I2C operate in slave mode. The interface automatically switches from slave to\
 *            master,after it generates a START condition and from master to slave, if an arbitration  \
 *            loss or a STOP generation occurs, allowing multi-master capability.
 *
 */
static ARM_DRIVER_STATUS I2C_PowerControl (ARM_POWER_STATE state, I2C_RESOURCES *i2c_resource) {
  switch (state) {
    case ARM_POWER_OFF:
    if (i2c_resource->info->status.powered ==false ) { return (ARM_DRIVER_OK); }
    /* Enable I2C clock */
    if (i2c_resource->reg == I2C1)      { LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_I2C1); }
    #if defined(I2C2)
    else if (i2c_resource->reg == I2C2)      { LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_I2C2); }
    #endif
    #if defined(I2C3)
    else if (i2c_resource->reg == I2C3)      { LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_I2C3); }
    #endif
    else { return (ARM_DRIVER_ERROR); }

    /* Disable I2C peripheral */
//    i2c_resource->reg->CR1 = 0;
    LL_I2C_Disable(i2c_resource->reg);

    /* Disable I2C IRQ */
    NVIC_DisableIRQ(i2c_resource->ev_irq_num);
    NVIC_DisableIRQ(i2c_resource->er_irq_num);

    i2c_resource->info->status.busy             = 0U;
    i2c_resource->info->status.mode             = 0U;
    i2c_resource->info->status.direction        = 0U;
    i2c_resource->info->status.general_call     = 0U;
    i2c_resource->info->status.arbitration_lost = 0U;
    i2c_resource->info->status.bus_error        = 0U;

    i2c_resource->info->status.powered =false;
    break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if (i2c_resource->info->status.initialized ==false ) { return (ARM_DRIVER_ERROR); }
      if (i2c_resource->info->status.powered ==true ) { return (ARM_DRIVER_OK); }

      /* Enable I2C clock */
      if (i2c_resource->reg == I2C1)      { LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1); }
      #if defined(I2C2)
      else if (i2c_resource->reg == I2C2)      { LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2); }
      #endif
      #if defined(I2C3)
      else if (i2c_resource->reg == I2C3)      { LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3); }
      #endif
      else { return (ARM_DRIVER_ERROR); }

      /* Clear and Enable I2C IRQ */
      NVIC_ClearPendingIRQ(i2c_resource->ev_irq_num);
      NVIC_ClearPendingIRQ(i2c_resource->er_irq_num);
      NVIC_EnableIRQ(i2c_resource->ev_irq_num);
      NVIC_EnableIRQ(i2c_resource->er_irq_num);

      /* Reset the peripheral */
      if (i2c_resource->reg == I2C1) {
        LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
        __NOP(); __NOP(); __NOP(); __NOP();
        LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);
      }
    #if defined (I2C2)
      else if (i2c_resource->reg == I2C2) {
        LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C2);
        __NOP(); __NOP(); __NOP(); __NOP();
        LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C2);
      }
    #endif
    #if defined (I2C3)
      else if (i2c_resource->reg == I2C3) {
        LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C3);
        __NOP(); __NOP(); __NOP(); __NOP();
        LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C3);
      }
    #endif
      else { return (ARM_DRIVER_ERROR); }


      /* Enable event and error interrupts */
//      i2c_resource->reg->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
      LL_I2C_EnableIT_EVT(i2c_resource->reg);
      LL_I2C_EnableIT_ERR(i2c_resource->reg);
      /* Disable buffer interrupts */
//      i2c_resource->reg->CR2 &= ~I2C_CR2_ITBUFEN;
      LL_I2C_DisableIT_BUF(i2c_resource->reg);
      /* Enable clock stretching */
      LL_I2C_EnableClockStretching(i2c_resource->reg);

      /* Enable I2C peripheral */
      LL_I2C_Enable(i2c_resource->reg);
//      i2c_resource->reg->CR1 |= I2C_CR1_PE;

      /* Enable acknowledge */
//      i2c_resource->reg->CR1 |= I2C_CR1_ACK;
      LL_I2C_AcknowledgeNextData(i2c_resource->reg,LL_I2C_ACK);
      /* Ready for operation */
      i2c_resource->info->status.powered =true;
      break;
    default: 
      break;
    }

  return ARM_DRIVER_OK;
}


/*
 * @belief Master transmitter
 * @param   addr    Slave address (7-bit or 10-bit)
 * @param   *data   Pointer to buffer with data to send to I2C Slave
 * @param   num     Number of data bytes to send
 * @param   xfer_pending  Transfer operation is pending - Stop condition will not be generated
 *      @arg    true   enable pending
 *      @arg    false  disable pending
 * @param   i2c_resource           Pointer to I2C resources
 *      @arg    arm_i2c1
 *      @arg    arm_i2c2
 *      @arg    arm_i2c3
 * @retval  execution_status
 * @note    master features: clock generation; start and stop generation;
 */
static ARM_DRIVER_STATUS I2C_MasterTransmit (uint32_t       addr,
                                   const uint8_t *data,
                                   uint32_t       num,
                                   bool           xfer_pending,
                                   I2C_RESOURCES *i2c_resource) {

  if ((data == NULL) || (num == 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((addr & ~((uint32_t)ARM_I2C_ADDRESS_10BIT | (uint32_t)ARM_I2C_ADDRESS_GC)) > 0x3FFU) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

   if (num > UINT16_MAX) {
       /* HAL does not handle 32-bit count in transfer */
      return ARM_DRIVER_ERROR;
    }

  if (i2c_resource->info->status.busy == true) {
    return (ARM_DRIVER_ERROR_BUSY);
  }

  /*
   * @ author NewLogic
   * when XFER_CTRL_XPENDING == true, I2C bus always busy
   */
  if ((i2c_resource->info->transfer_flags & XFER_CTRL_XPENDING) == 0U) {
    /* New transfer */
    while (LL_I2C_IsActiveFlag_BUSY(i2c_resource->reg)) {
      ; /* Wait until bus released */
    }
  }

  i2c_resource->info->status.busy             = true;
  i2c_resource->info->status.mode             = I2C_MODE_MASTER;
  i2c_resource->info->status.direction        = I2C_DIR_TX;
  i2c_resource->info->status.bus_error        = false;
  i2c_resource->info->status.arbitration_lost = false;

  i2c_resource->info->tx_info.xfer_size  = num;
  i2c_resource->info->tx_info.count  = 0U;
  i2c_resource->info->tx_info.data = (uint8_t *)((uint32_t)data);

  i2c_resource->info->addr = (uint16_t)(addr);
  i2c_resource->info->transfer_flags = 0U;

  if (xfer_pending == true) {
    i2c_resource->info->transfer_flags |= XFER_CTRL_XPENDING;
  }

//  if (i2c_resource->dma_tx) {
//    /* Enable stream */
//    if (HAL_DMA_Start_IT (i2c_resource->dma_tx->h, (uint32_t)data, (uint32_t)&(i2c_resource->reg->DR), num) != HAL_OK) {
//      return ARM_DRIVER_ERROR;
//    }
//  }

  /* Generate start and enable event interrupts */
  LL_I2C_DisableIT_EVT(i2c_resource->reg);
  LL_I2C_GenerateStartCondition(i2c_resource->reg);
  LL_I2C_EnableIT_EVT(i2c_resource->reg);

  return ARM_DRIVER_OK;
}

/*
 * @belief Master receiver
 * @param   addr    Slave address (7-bit or 10-bit)
 * @param   *data   Pointer to buffer with data to send to I2C Slave
 * @param   num     Number of data bytes to send
 * @param   xfer_pending  Transfer operation is pending - Stop condition will not be generated
 *      @arg    true   enable pending
 *      @arg    false  disable pending
 * @param   i2c_resource           Pointer to I2C resources
 *      @arg    arm_i2c1
 *      @arg    arm_i2c2
 *      @arg    arm_i2c3
 * @retval  execution_status
 * @note    master features: clock generation; start and stop generation;
 */
static ARM_DRIVER_STATUS I2C_MasterReceive (uint32_t       addr,
                                  uint8_t       *data,
                                  uint32_t       num,
                                  bool           xfer_pending,
                                  I2C_RESOURCES *i2c_resource) {

  if ((data == NULL) || (num == 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if ((addr & ~((uint32_t)ARM_I2C_ADDRESS_10BIT | (uint32_t)ARM_I2C_ADDRESS_GC)) > 0x3FFU) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (i2c_resource->info->status.busy == true) {
    return (ARM_DRIVER_ERROR_BUSY);
  }

  if ((i2c_resource->info->transfer_flags & XFER_CTRL_XPENDING) == 0U) {
    /* New transfer */
//    while (i2c_resource->reg->SR2 & I2C_SR2_BUSY)
    while (LL_I2C_IsActiveFlag_BUSY(i2c_resource->reg))
    {
      ; /* Wait until bus released */
    }
  }

  i2c_resource->info->status.busy             = true;
  i2c_resource->info->status.mode             = I2C_MODE_MASTER;
  i2c_resource->info->status.direction        = I2C_DIR_RX;
  i2c_resource->info->status.bus_error        = false;
  i2c_resource->info->status.arbitration_lost = false;

  i2c_resource->info->rx_info.xfer_size  = num;
  i2c_resource->info->rx_info.count  = 0U;
  i2c_resource->info->rx_info.data = (uint8_t *)data;

  i2c_resource->info->addr = (uint16_t)(addr);
  i2c_resource->info->transfer_flags = 0U;

  if (xfer_pending) {
    i2c_resource->info->transfer_flags |= XFER_CTRL_XPENDING;
  }

  /* Enable acknowledge generation */
//  i2c_resource->reg->CR1 |= I2C_CR1_ACK;
  LL_I2C_AcknowledgeNextData(i2c_resource->reg,LL_I2C_ACK);
//  if (i2c_resource->dma_rx) {
//    /* Enable stream */
//    if (HAL_DMA_Start_IT (i2c_resource->dma_rx->h, (uint32_t)&(i2c_resource->reg->DR), (uint32_t)data, num) != HAL_OK) {
//      return ARM_DRIVER_ERROR;
//    }
//    /* Permit generation of a NACK on the last received data */
//    i2c_resource->reg->CR2 |= I2C_CR2_LAST;
//  }

  /* Generate start and enable event interrupts */
  LL_I2C_DisableIT_EVT(i2c_resource->reg);
  LL_I2C_GenerateStartCondition(i2c_resource->reg);
//  i2c_resource->reg->CR2 |=  I2C_CR2_ITEVTEN;
  LL_I2C_EnableIT_EVT(i2c_resource->reg);
  return ARM_DRIVER_OK;
}

/*
 * @belief Slave transmitter
 * @param   *data   Pointer to buffer with data to send to I2C Slave
 * @param   num     Number of data bytes to send
 * @param   i2c_resource           Pointer to I2C resources
 *      @arg    arm_i2c1
 *      @arg    arm_i2c2
 *      @arg    arm_i2c3
 * @retval  execution_status
 * @note    slave features: programmable I2C address detection; Dual addressing capability to acknowledge
 *          2 slave address; stop bit detection
 */
static ARM_DRIVER_STATUS I2C_SlaveTransmit (const uint8_t *data, uint32_t num, I2C_RESOURCES *i2c_resource) {

  if ((data == NULL) || (num == 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (i2c_resource->info->status.busy ==true) {
    return (ARM_DRIVER_ERROR_BUSY);
  }

  i2c_resource->info->status.bus_error    = false;
  i2c_resource->info->status.general_call = false;

  i2c_resource->info->tx_info.xfer_size  = num;
  i2c_resource->info->tx_info.count  = 0;
  i2c_resource->info->tx_info.data = (uint8_t *)((uint32_t)data);

  i2c_resource->info->transfer_flags = 0U;

//  if (i2c_resource->dma_tx) {
//    /* Enable stream */
//    if (HAL_DMA_Start_IT (i2c_resource->dma_tx->h, (uint32_t)data, (uint32_t)&(i2c_resource->reg->DR), num) != HAL_OK) {
//      return ARM_DRIVER_ERROR;
//    }
//  }

  /* Enable acknowledge */
  LL_I2C_AcknowledgeNextData(i2c_resource->reg,LL_I2C_ACK);
  /* Enable event interrupts */
  LL_I2C_EnableIT_EVT(i2c_resource->reg);

  return ARM_DRIVER_OK;
}

/*
 * @belief Slave receiver
 * @param   *data   Pointer to buffer with data to send to I2C Slave
 * @param   num     Number of data bytes to receive
 * @param   i2c_resource           Pointer to I2C resources
 *      @arg    arm_i2c1
 *      @arg    arm_i2c2
 *      @arg    arm_i2c3
 * @retval  ARM_DRIVER_STATUS
 * @note    slave features: programmable I2C address detection; Dual addressing capability to acknowledge
 *          2 slave address; stop bit detection
 */
static ARM_DRIVER_STATUS I2C_SlaveReceive (uint8_t *data, uint32_t num, I2C_RESOURCES *i2c_resource) {

  if ((data == NULL) || (num == 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (i2c_resource->info->status.busy == true) {
    return (ARM_DRIVER_ERROR_BUSY);
  }

  i2c_resource->info->status.bus_error    = false;
  i2c_resource->info->status.general_call = false;

  i2c_resource->info->rx_info.data = data;
  i2c_resource->info->rx_info.xfer_size  = num;
  i2c_resource->info->rx_info.count  = 0;

  i2c_resource->info->transfer_flags = 0U;

//  if (i2c_resource->dma_rx) {
//    /* Enable stream */
//    if (HAL_DMA_Start_IT (i2c_resource->dma_rx->h, (uint32_t)&(i2c_resource->reg->DR), (uint32_t)data, num) != HAL_OK) {
//      return ARM_DRIVER_ERROR;
//    }
//  }

  /* Enable acknowledge */
  LL_I2C_AcknowledgeNextData(i2c_resource->reg,LL_I2C_ACK);

  /* Enable event interrupts */
  LL_I2C_EnableIT_EVT(i2c_resource->reg);
  return ARM_DRIVER_OK;
}

/*
 * @belief Get transferred data count
 * @param   i2c_resource           Pointer to I2C resources
 *      @arg    arm_i2c1
 *      @arg    arm_i2c2
 *      @arg    arm_i2c3
 * @retval   number of data bytes transferred; -1 when Slave is not addressed by Master
 */
static int32_t I2C_GetDataCount (I2C_RESOURCES *i2c_resource) {
  int32_t val;

  if (i2c_resource->info->status.direction == I2C_DIR_TX) {
    val = i2c_resource->info->tx_info.count;
  } else {
    val = i2c_resource->info->rx_info.count;
  }

  return (val);
}


/*
 * @brief Control I2C Interface
 * @param control operation
 *      @arg @ref ARM_I2C_OWN_ADDRESSS
 *      @arg @ref ARM_I2C_BUS_SPEED
 *      @arg @ref ARM_I2C_BUS_CLEAR
 *      @arg @ref ARM_I2C_ABORT_TRANSFER
 * @param arg argument of operation (optional)
 * @param   i2c_resource           Pointer to I2C resources
 *      @arg    arm_i2c1
 *      @arg    arm_i2c2
 *      @arg    arm_i2c3
 * @retval  execution_status
 */
ARM_DRIVER_STATUS I2C_Control (uint32_t control, uint32_t arg, I2C_RESOURCES *i2c_resource) {

  uint32_t state;
  uint32_t i, pclk;
  uint32_t ccr;
  uint32_t trise;
  LL_RCC_ClocksTypeDef rcc_clocks;


  if ((i2c_resource->info->status.powered) == 0U) {
    /* I2C not powered */
    return ARM_DRIVER_ERROR;
  }

  switch (control)
  {
    case ARM_I2C_OWN_ADDRESS:
      /* Enable/Disable General call */
      if (arg & ARM_I2C_ADDRESS_GC)
      {
        LL_I2C_EnableGeneralCall(i2c_resource->reg);
      }
      else
      {
        LL_I2C_DisableGeneralCall(i2c_resource->reg);
      }
      /* Set own address and its length */
      LL_I2C_SetOwnAddress1(i2c_resource->reg,((arg << 1) & 0x03FFU) |
                                              (1U << 14)             |
                                              ((arg & ARM_I2C_ADDRESS_10BIT) ? (1U << 15) : (0U)),\
                                              LL_I2C_OWNADDRESS1_7BIT);
//      i2c->reg->OAR1 = ((arg << 1) & 0x03FFU) |
//                            (1U << 14)             |
//                            ((arg & ARM_I2C_ADDRESS_10BIT) ? (1U << 15) : (0U));

      break;

    case ARM_I2C_BUS_SPEED:
        
      LL_RCC_GetSystemClocksFreq(&rcc_clocks);

//      pclk = HAL_RCC_GetPCLK1Freq();
      pclk = rcc_clocks.PCLK1_Frequency;
      switch (arg) {
        case ARM_I2C_BUS_SPEED_STANDARD:
          /* Clock = 100kHz,  Rise Time = 1000ns */
          if (pclk > 50000000U) { return ARM_DRIVER_ERROR_UNSUPPORTED; }
          if (pclk <  2000000U) { return ARM_DRIVER_ERROR_UNSUPPORTED; }
          ccr   = (pclk /  100000U) / 2U;
          trise = (pclk / 1000000U) + 1U;
          LL_I2C_SetClockSpeedMode(i2c_resource->reg,LL_I2C_CLOCK_SPEED_STANDARD_MODE);
          break;
        case ARM_I2C_BUS_SPEED_FAST:
          /* Clock = 400kHz,  Rise Time = 300ns */
          LL_I2C_SetClockSpeedMode(i2c_resource->reg,LL_I2C_CLOCK_SPEED_FAST_MODE);
          if (pclk > 50000000U) { return ARM_DRIVER_ERROR_UNSUPPORTED; }
          if (pclk <  4000000U) { return ARM_DRIVER_ERROR_UNSUPPORTED; }
          if ((pclk >= 10000000U) && ((pclk % 10000000U) == 0U)) {
            ccr =   I2C_CCR_DUTY | ((pclk / 400000U) / 25U);
          } else {
            ccr =   (pclk / 400000U) / 3U;
          }
          trise = (((pclk / 1000000U) * 300U) / 1000U) + 1U;
          break;
        default:
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      /* Disable I2C peripheral */
      LL_I2C_Disable(i2c_resource->reg);
//      i2c_resource->reg->CR2   &= ~I2C_CR2_FREQ;
//      i2c_resource->reg->CR2   |=  pclk / 1000000U;
      LL_I2C_SetPeriphClock(i2c_resource->reg,pclk);
      LL_I2C_SetClockPeriod(i2c_resource->reg,ccr);
//      i2c_resource->reg->TRISE  =  trise;
      LL_I2C_SetRiseTime(i2c_resource->reg,trise);
      /* Enable I2C peripheral */
      LL_I2C_Enable(i2c_resource->reg);
      /* Enable acknowledge    */
      LL_I2C_AcknowledgeNextData(i2c_resource->reg,LL_I2C_ACK);
      break;

    case ARM_I2C_BUS_CLEAR:
      /*
       * A 9th clock pulse follows the 8 clock cycles of a byte transfer, during which, the receiver must
       * Send an acknowledge bit to the master
       */
      /* Configure SCl and SDA pins as GPIO pin */

      LL_GPIO_SetPinMode(i2c_resource->io.scl_port,i2c_resource->io.scl_pin,LL_GPIO_MODE_OUTPUT);
      LL_GPIO_SetPinOutputType(i2c_resource->io.scl_port,i2c_resource->io.scl_pin,LL_GPIO_OUTPUT_OPENDRAIN);
      LL_GPIO_SetPinSpeed(i2c_resource->io.scl_port,i2c_resource->io.scl_pin,LL_GPIO_SPEED_FREQ_LOW);


      LL_GPIO_SetPinMode(i2c_resource->io.sda_port,i2c_resource->io.sda_pin,LL_GPIO_MODE_OUTPUT);
      LL_GPIO_SetPinOutputType(i2c_resource->io.sda_port,i2c_resource->io.sda_pin,LL_GPIO_OUTPUT_OPENDRAIN);
      LL_GPIO_SetPinSpeed(i2c_resource->io.sda_port,i2c_resource->io.sda_pin,LL_GPIO_SPEED_FREQ_LOW);
    

      /* Output SCL and SDA high */
      LL_GPIO_SetOutputPin(i2c_resource->io.scl_port, i2c_resource->io.scl_pin);    
      LL_GPIO_SetOutputPin(i2c_resource->io.sda_port, i2c_resource->io.sda_pin);
    
    
      UTILS_MilliSecondDelay (I2C_BUS_CLEAR_CLOCK_PERIOD);

      for (i = 0U; i < 9U; i++) {
        if (LL_GPIO_IsInputPinSet(i2c_resource->io.sda_port, i2c_resource->io.sda_pin)) {
          /* Break if slave released SDA line */
          break;
        }
        /* Clock high */
        LL_GPIO_SetOutputPin(i2c_resource->io.scl_port, i2c_resource->io.scl_pin);
        UTILS_MilliSecondDelay (I2C_BUS_CLEAR_CLOCK_PERIOD/2);

        /* Clock low */
        LL_GPIO_ResetOutputPin (i2c_resource->io.scl_port, i2c_resource->io.scl_pin);
        UTILS_MilliSecondDelay (I2C_BUS_CLEAR_CLOCK_PERIOD/2);
      }

      /* Check SDA state */
      state = LL_GPIO_IsInputPinSet(i2c_resource->io.sda_port, i2c_resource->io.sda_pin);

      /* Configure SDA and SCL pins as I2C peripheral pins */
      LL_GPIO_SetPinMode(i2c_resource->io.scl_port,i2c_resource->io.scl_pin,LL_GPIO_MODE_ALTERNATE);
      LL_GPIO_SetPinSpeed(i2c_resource->io.scl_port,i2c_resource->io.scl_pin,LL_GPIO_SPEED_FREQ_MEDIUM);
      LL_GPIO_SetPinOutputType(i2c_resource->io.scl_port,i2c_resource->io.scl_pin,LL_GPIO_OUTPUT_OPENDRAIN);

      LL_GPIO_SetPinMode(i2c_resource->io.sda_port,i2c_resource->io.sda_pin,LL_GPIO_MODE_ALTERNATE);
      LL_GPIO_SetPinSpeed(i2c_resource->io.sda_port,i2c_resource->io.sda_pin,LL_GPIO_SPEED_FREQ_MEDIUM);
      LL_GPIO_SetPinOutputType(i2c_resource->io.sda_port,i2c_resource->io.sda_pin,LL_GPIO_OUTPUT_OPENDRAIN);

      return (state == 1) ? ARM_DRIVER_OK : ARM_DRIVER_ERROR;

    case ARM_I2C_ABORT_TRANSFER:
      /* Disable DMA requests and I2C interrupts */
//      i2c_resource->reg->CR2 &= ~(I2C_CR2_DMAEN | I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN);
      LL_I2C_DisableIT_EVT(i2c_resource->reg);
      LL_I2C_DisableIT_BUF(i2c_resource->reg);
      LL_I2C_DisableDMAReq_RX(i2c_resource->reg);
      LL_I2C_DisableDMAReq_TX(i2c_resource->reg);

//      if ((i2c_resource->dma_rx != NULL) && (i2c_resource->dma_tx != NULL)) {
//        /* Disable DMA Streams */
//        if (HAL_DMA_Abort (i2c_resource->dma_rx->h) != HAL_OK) {
//          return ARM_DRIVER_ERROR;
//        }
//        if (HAL_DMA_Abort (i2c_resource->dma_tx->h) != HAL_OK) {
//          return ARM_DRIVER_ERROR;
//        }
//      }
      /* Generate stop */
      /* Master generates stop after the current byte transfer */
      /* Slave releases SCL and SDA after the current byte transfer */
//      i2c_resource->reg->CR1 |= I2C_CR1_STOP;
      LL_I2C_GenerateStopCondition(i2c_resource->reg);

      i2c_resource->info->rx_info.xfer_size  = 0U;
      i2c_resource->info->rx_info.count  = 0U;
      i2c_resource->info->tx_info.xfer_size  = 0U;
      i2c_resource->info->tx_info.count  = 0U;

      i2c_resource->info->addr = 0U;
      i2c_resource->info->transfer_flags = 0U;

      i2c_resource->info->status.busy             = 0U;
      i2c_resource->info->status.mode             = 0U;
      i2c_resource->info->status.direction        = 0U;
      i2c_resource->info->status.general_call     = 0U;
      i2c_resource->info->status.arbitration_lost = 0U;
      i2c_resource->info->status.bus_error        = 0U;

      /* Disable and re-enable peripheral to clear some flags */
//      i2c_resource->reg->CR1 &= ~I2C_CR1_PE;
      LL_I2C_Disable(i2c_resource->reg);
//      i2c_resource->reg->CR1 |=  I2C_CR1_PE;
      LL_I2C_Enable(i2c_resource->reg);
      /* Enable acknowledge */
//      i2c_resource->reg->CR1 |=  I2C_CR1_ACK;
      LL_I2C_AcknowledgeNextData(i2c_resource->reg,LL_I2C_ACK);
      break;

    default: return ARM_DRIVER_ERROR;
  }
  return ARM_DRIVER_OK;
}


/*
 * @belief Get I2C status.
 * @param   i2c_resource           Pointer to I2C resources
 *      @arg    arm_i2c1
 *      @arg    arm_i2c2
 *      @arg    arm_i2c3
 * @retval   I2C status
 *      @arg  @ref ARM_I2C_STATUS
 */
static ARM_I2C_STATUS I2C_GetStatus (I2C_RESOURCES *i2c_resource) {
  return (i2c_resource->info->status);
}

/*
 * @belief  I2C Event Interrupt handler. For successful address/data communication
 * @param   i2c_resource           Pointer to I2C resources
 *      @arg    arm_i2c1
 *      @arg    arm_i2c2
 *      @arg    arm_i2c3
 * @retval   none
 * @note     status flags: Transmitter/Receiver mode flag; End-of-Byte transmission flag; I2C busy flag
 */
static void I2C_EV_IRQHandler (I2C_RESOURCES *i2c_resource) {
  I2C_TRANSFER_INFO volatile *rx = &i2c_resource->info->rx_info;
  I2C_TRANSFER_INFO volatile *tx = &i2c_resource->info->tx_info;
  I2C_INFO          volatile *tr = i2c_resource->info;
  uint8_t  data;
  uint16_t sr1;
  uint16_t sr2 = 0;
  uint32_t event;

  sr1 = (uint16_t)i2c_resource->reg->SR1;

  if (sr1 & I2C_SR1_SB){
    /* (EV5): start bit generated, send address */

    if (tr->addr & ARM_I2C_ADDRESS_10BIT){
      /*10 bit address mode, header matched*/
      /* 10-bit addressing mode */
      data = (uint8_t)(0xF0U | ((tr->addr >> 7) & 0x06U));
    }
    else{
      /* 7-bit addressing mode */
      data  = (uint8_t)(tr->addr << 1);
      data |= (uint8_t)tr->status.direction;
    }
    i2c_resource->reg->DR = data;
  }
  else if (sr1 & I2C_SR1_ADD10){
    /* (EV9): 10-bit address header sent, send device address LSB */
    i2c_resource->reg->DR = (uint8_t)tr->addr;

    if (tr->status.direction == I2C_DIR_RX) {
      /* Master receiver generates repeated start in 10-bit addressing mode */
      tr->transfer_flags |= XFER_CTRL_RSTART;
    }
  }
  else if (sr1 & I2C_SR1_ADDR){
    /* (EV6): addressing complete */

    if ((tr->status.mode == I2C_MODE_MASTER) && (tr->status.direction == I2C_DIR_RX))
    {
      /* Master receiver */

      /* Clear ADDR flag */
      LL_I2C_ClearFlag_ADDR(i2c_resource->reg);

      if (rx->xfer_size == 1U)/* master receiver mode when receiving number is 1*/
      {
        LL_I2C_AcknowledgeNextData(i2c_resource->reg, LL_I2C_NACK);
      }

      if (tr->transfer_flags & XFER_CTRL_RSTART) /* 10 bit address,?? */
      {
        tr->transfer_flags &= ~XFER_CTRL_RSTART;
        /* Generate repeated start */
        LL_I2C_GenerateStartCondition(i2c_resource->reg);
      }
      else
      {
        if (rx->xfer_size == 1U) {
          if ((tr->transfer_flags & XFER_CTRL_XPENDING) == 0U) {
            LL_I2C_GenerateStopCondition(i2c_resource->reg);
          }
        }
        else if (rx->xfer_size == 2U) {
          LL_I2C_AcknowledgeNextData(i2c_resource->reg,LL_I2C_NACK);
          LL_I2C_EnableBitPOS(i2c_resource->reg);
          /* Wait until BTF == 1 */
          tr->transfer_flags |= XFER_CTRL_WAIT_BTF;
        }
        else {
          if (rx->xfer_size == 3U) {
            /* Wait until BTF == 1 */
            tr->transfer_flags |= XFER_CTRL_WAIT_BTF;
          }
        }
      }
    }
    else
    {
      /* Master transmitter | Slave transmitter | Slave receiver */
      LL_I2C_ClearFlag_ADDR(i2c_resource->reg);

      /*
       * (MSL: 0 stands slave mode, 1 stands master mode. Set when start condition happen,
       * Cleared when stop condition happen or loss of arbitration(ARLO=1) or PE=0
       */
//      LL_I2C_IsActiveFlag_MSL(i2c_resource->reg);
      if ((sr2 & I2C_SR2_MSL) == 0U)
      {
        /* Slave mode */

        /*
         * (TRA: 0 stands Data byte received, 1 stands for Data byte transmitted. TRA bit is
         * set depending on the end of address byte. Cleared when detect a stop condition or
         * repeated Start condition, ARLO=1,or PE =0
         */
        if (sr2 & I2C_SR2_TRA)
        {
          /* TRA bit ==1 */ /* Transmitter */


          /* Slave transmitter */
          tx->count = 0;
          tr->status.direction = I2C_DIR_TX;
        }
        else
        {
          /* TRA bit == 0 */ /* receiver */
          /* Slave receiver */
          rx->count = 0;
          tr->status.direction = I2C_DIR_RX;
        }

        if (sr2 & I2C_SR2_GENCALL)
        {
          tr->status.general_call = 1U;
        } else
        {
          tr->status.general_call = 0U;
        }

        if (tr->cb_event != NULL)
        {
          /* program signal I2C by event is set*/
          event = 0U; /* Reset event flag */

          if (tr->status.general_call)
          {
            event |= ARM_I2C_EVENT_GENERAL_CALL;
          }

          if (tr->status.direction == I2C_DIR_TX)
          {
            if (tx->xfer_size == 0)
            {
              tr->cb_event (event | ARM_I2C_EVENT_SLAVE_TRANSMIT);
            }
          }
          else /* status.direction == I2C_DIR_RX */
          {
            if (rx->xfer_size == 0)
            {
              tr->cb_event (event | ARM_I2C_EVENT_SLAVE_RECEIVE);

              /* Check if receive operation was set */
              if (rx->xfer_size == 0) {
                /* Nothing to receive, disable ACK */
//                i2c_resource->reg->CR1 &= ~I2C_CR1_ACK;
                LL_I2C_AcknowledgeNextData(i2c_resource->reg,LL_I2C_NACK);

              }
            }
          }
        }
        /* Driver is busy */
        tr->status.busy = true;
      }
    }

    tr->transfer_flags |= XFER_CTRL_ADDR_DONE | XFER_CTRL_XACTIVE;

//    if ((i2c_resource->dma_rx != NULL) && (i2c_resource->dma_tx != NULL)) {
//      /* Enable DMA data transfer */
//      i2c_resource->reg->CR2 |= I2C_CR2_DMAEN;

//    }
 //   else {
      /* Enable IRQ data transfer */
//      i2c_resource->reg->CR2 |= I2C_CR2_ITBUFEN;
      LL_I2C_EnableIT_BUF(i2c_resource->reg);
  //  }
  }
  else if (sr1 & I2C_SR1_STOPF) {
    /* Slave mode: STOP condition detected */
    /* Set after ACK, not set after NACK   */
    event = 0U;

    if (tr->status.direction == I2C_DIR_RX) {
      /* slave receiver */
      if ((uint32_t)rx->count < rx->xfer_size) {
        event = ARM_I2C_EVENT_TRANSFER_DONE | ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
      }
    }
    else {
      if ((uint32_t)tx->count < tx->xfer_size) {
        event = ARM_I2C_EVENT_TRANSFER_DONE | ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
      }
    }

    /* Re-enable ACK */
    LL_I2C_AcknowledgeNextData(i2c_resource->reg,LL_I2C_ACK);
    tr->transfer_flags = 0U;
    tr->status.busy = false;

    if ((event != 0U) && (tr->cb_event != NULL)) {
      tr->cb_event (event);
    }
  }
  else if (tr->transfer_flags & XFER_CTRL_XACTIVE) {
    /* BTF, RxNE or TxE interrupt */
    if (tr->transfer_flags & XFER_CTRL_DMA_DONE) {
      /* BTF triggered this event */
      if (tr->status.mode == I2C_MODE_MASTER) {
        if (tr->transfer_flags & XFER_CTRL_XPENDING) {
          /* Disable event interrupt */
          i2c_resource->reg->CR2 &= ~I2C_CR2_ITEVTEN;
        }
        else {
          /* Generate stop condition */
          i2c_resource->reg->CR1 |= I2C_CR1_STOP;
        }

        tr->transfer_flags &= ~XFER_CTRL_XACTIVE;

        tr->status.busy = 0U;
        tr->status.mode = 0U;

        if (tr->cb_event) {
          tr->cb_event (ARM_I2C_EVENT_TRANSFER_DONE);
        }
      }
    }
    else if (sr1 & I2C_SR1_TXE){
      /* Transmitter mode */
      if (tr->status.mode == I2C_MODE_MASTER){
        /* Master transmitter */
        if (tr->transfer_flags & XFER_CTRL_WAIT_BTF){
          /* Closing Communication */
          if (sr1 & I2C_SR1_BTF){
            /* End master transmit operation */
            LL_I2C_DisableIT_BUF(i2c_resource->reg);
            if (tr->transfer_flags & XFER_CTRL_XPENDING){
              LL_I2C_DisableIT_EVT(i2c_resource->reg);
            }
            else{
              LL_I2C_GenerateStopCondition(i2c_resource->reg);
            }

            tr->transfer_flags &= ~XFER_CTRL_XACTIVE;
            /* after a stop condition, The interface automatically goes back to slave mode */
            tr->status.busy = false;
            tr->status.mode = I2C_MODE_SLAVE;

            if (tr->cb_event) {
              tr->cb_event (ARM_I2C_EVENT_TRANSFER_DONE);
            }
          }
        }
        else
        { /* Transfering active */
          /*EV8_1: Shift register empty, DR empty*/
          /*EV8: Shift register not empty, DR empty*/
          i2c_resource->reg->DR = tx->data[tx->count];
          tx->count++;
          if ((uint32_t)tx->count == tx->xfer_size) {
            /* all data has been sent to DR */
            tr->transfer_flags |= XFER_CTRL_WAIT_BTF;
          }
        }
      }
      else {
        /* Slave transmitter */
        if (tr->transfer_flags & XFER_CTRL_WAIT_BTF) {
          if (sr1 & I2C_SR1_BTF) {
            /* Master requests more data */
            event = ARM_I2C_EVENT_SLAVE_TRANSMIT;

            if (tr->status.general_call) {
              event |= ARM_I2C_EVENT_GENERAL_CALL;
            }

            if (tr->cb_event != NULL) {
              tr->cb_event (event);
            }
          }
        }

        if (tx->xfer_size != 0) {
          i2c_resource->reg->DR = tx->data[tx->count];

          tx->count++;
          if ((uint32_t)tx->count == tx->xfer_size) {
            tx->xfer_size = 0;

            tr->transfer_flags |= XFER_CTRL_XDONE | XFER_CTRL_WAIT_BTF;
          }
        }
        else {
          /* sent 0xff to signal the master that slave have receive enough data(>num) */
          i2c_resource->reg->DR = (uint8_t)0xFF;
        }
      }
    }
    else if (sr1 & I2C_SR1_RXNE) {
      /* Receiver mode */
      if (i2c_resource->info->status.mode == I2C_MODE_MASTER){
        /* Master receiver */
        if (tr->transfer_flags & XFER_CTRL_WAIT_BTF){
          /* Closing Communication */
          if (sr1 & I2C_SR1_BTF){
//            if ((rx->xfer_size == 2U) || ((uint32_t)rx->count == (rx->xfer_size - 2U)))
            if (rx->xfer_size == 2U){
              /* Two bytes  */
//              LL_I2C_DisableIT_BUF(i2c_resource->reg);
              if (tr->transfer_flags & XFER_CTRL_XPENDING) {
                i2c_resource->reg->CR2 &= ~I2C_CR2_ITEVTEN;
              }
              else{
                LL_I2C_GenerateStopCondition(i2c_resource->reg);
              }

              /* Read data N-1 and N (last 2 data)*/
              rx->data[rx->count++] = LL_I2C_ReceiveData8(i2c_resource->reg);
              rx->data[rx->count++] = LL_I2C_ReceiveData8(i2c_resource->reg);

              tr->transfer_flags &= ~XFER_CTRL_XACTIVE;

              tr->status.busy = false;
              tr->status.mode = I2C_MODE_SLAVE;

              LL_I2C_DisableBitPOS(i2c_resource->reg);
              if (tr->cb_event)
              {
                tr->cb_event (ARM_I2C_EVENT_TRANSFER_DONE);
              }
            }
            else /* rx->xfer_size > 2 */
            {
              /* Three bytes remaining */
              i2c_resource->reg->CR1 &= ~I2C_CR1_ACK;
              tr->transfer_flags &= ~XFER_CTRL_WAIT_BTF;
              /* Read data N-2 */
              rx->data[rx->count++] = (uint8_t)i2c_resource->reg->DR;
            }
          }
        }
        else {
          /* Transfering active */
          if (rx->xfer_size == 1U) {
            /* Single byte transfer completed */
            i2c_resource->reg->CR2 &= ~I2C_CR2_ITBUFEN;

            if (tr->transfer_flags & XFER_CTRL_XPENDING) {
              i2c_resource->reg->CR2 &= ~I2C_CR2_ITEVTEN;
            }
            /* (STOP was already sent during ADDR phase) */

            tr->transfer_flags &= ~XFER_CTRL_XACTIVE;

            tr->status.busy = false;
            tr->status.mode = I2C_MODE_SLAVE;

            if (tr->cb_event) {
              tr->cb_event (ARM_I2C_EVENT_TRANSFER_DONE);
            }
          }
          else if ((uint32_t)rx->count == (rx->xfer_size - 3U)) {
              /* N > 2 byte reception, begin N-2 data reception */
//              i2c_resource->reg->CR2 &= ~I2C_CR2_ITBUFEN;
              /* Wait until BTF == 1 */
              tr->transfer_flags |= XFER_CTRL_WAIT_BTF;
          }
          else if ((uint32_t)rx->count == (rx->xfer_size -1U)){
              if (tr->transfer_flags & XFER_CTRL_XPENDING) {
                i2c_resource->reg->CR2 &= ~I2C_CR2_ITEVTEN;
              }
              else{
                LL_I2C_GenerateStopCondition(i2c_resource->reg);
              }
          }
          rx->data[rx->count++] = (uint8_t)i2c_resource->reg->DR;
        }
        
      }
      else {
        /* Slave receiver */
        if (tr->transfer_flags & XFER_CTRL_XDONE){
          tr->transfer_flags &= ~XFER_CTRL_XDONE;

          /* Master sends more data then required */
          event = ARM_I2C_EVENT_SLAVE_RECEIVE;

          if (tr->status.general_call) {
            event |= ARM_I2C_EVENT_GENERAL_CALL;
          }

          if (tr->cb_event) {
            tr->cb_event (event);
          }

//          if (rx->xfer_size == 0) {
//            /* Nothing to receive, disable ACK */
//            i2c_resource->reg->CR1 &= ~I2C_CR1_ACK;
//          }
        }

        data = (uint8_t)i2c_resource->reg->DR;

        if (rx->xfer_size != 0){
          /* Receive data */
          rx->data[rx->count] = data;

          rx->count++;
          if ((uint32_t)rx->count == rx->xfer_size) {
            rx->xfer_size = 0;

            tr->transfer_flags |= XFER_CTRL_XDONE;

            tr->status.busy = 0U;

            event = ARM_I2C_EVENT_TRANSFER_DONE;

            if (tr->status.general_call) {
              event |= ARM_I2C_EVENT_GENERAL_CALL;
            }

            if (tr->cb_event) {
              tr->cb_event (event);
            }
          }
        }
      }
    }
  }
}

/*
 * @belief  I2C Error Interrupt handler. for I2C error condition
 * @param   i2c_resource           Pointer to I2C resources
 *      @arg    arm_i2c1
 *      @arg    arm_i2c2
 *      @arg    arm_i2c3
 * @retval   none
 * @note     Error flags: arbitration lost condition for master mode; acknowledgment failure after adderss/data \
 *           transmission; detection of misplaced start or stop condition; overrun\underrun if clock stretching \
 *           is disabled
 */
static void I2C_ER_IRQHandler (I2C_RESOURCES *i2c_resource) {
  uint32_t sr1 = i2c_resource->reg->SR1;
  uint32_t evt = 0U;
  uint32_t err = 0U;

  if (sr1 & I2C_SR1_SMBALERT) {
    /* SMBus alert */
    err |= I2C_SR1_SMBALERT;
  }
  if (sr1 & I2C_SR1_TIMEOUT) {
    /* Timeout - SCL remained LOW for 25ms */
    err |= I2C_SR1_TIMEOUT;
  }
  if (sr1 & I2C_SR1_PECERR) {
    /* PEC Error in reception */
    err |= I2C_SR1_PECERR;
  }
  if (sr1 & I2C_SR1_OVR) {
    /* Overrun/Underrun */
    err |= I2C_SR1_OVR;
  }

  if (sr1 & I2C_SR1_AF) {
    /* Acknowledge failure */
    err |= I2C_SR1_AF;

    /* Reset the communication */
    i2c_resource->reg->CR1 |= I2C_CR1_STOP;

    i2c_resource->info->status.busy = 0U;
    i2c_resource->info->status.mode = 0U;

    evt = ARM_I2C_EVENT_TRANSFER_DONE;

    if ((i2c_resource->info->transfer_flags & XFER_CTRL_ADDR_DONE) == 0U) {
      /* Addressing not done */
      evt |= ARM_I2C_EVENT_ADDRESS_NACK;
    }
    else {
      /* Check if all data transmitted */
      if ((i2c_resource->info->transfer_flags & XFER_CTRL_XDONE) == 0U) {
        evt |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
      }
    }
    i2c_resource->info->tx_info.xfer_size = 0U;
    i2c_resource->info->transfer_flags   = 0U;
  }

  if (sr1 & I2C_SR1_ARLO) {
    /* Arbitration lost */
    err |= I2C_SR1_ARLO;

    /* Switch to slave mode */
    i2c_resource->info->status.busy             = 0U;
    i2c_resource->info->status.mode             = 0U;
    i2c_resource->info->status.arbitration_lost = 1U;

    evt = ARM_I2C_EVENT_TRANSFER_DONE | ARM_I2C_EVENT_ARBITRATION_LOST;

    /* Check if all data transmitted */
    if ((i2c_resource->info->transfer_flags & XFER_CTRL_XDONE) == 0U) {
      evt |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
    }

    i2c_resource->info->transfer_flags = 0U;
  }

  if (sr1 & I2C_SR1_BERR) {
    /* Bus error - misplaced start/stop */
    err |= I2C_SR1_BERR;

    i2c_resource->info->status.bus_error = 1U;

    evt = ARM_I2C_EVENT_TRANSFER_DONE | ARM_I2C_EVENT_BUS_ERROR;

    /* Check if all data transmitted */
    if ((i2c_resource->info->transfer_flags & XFER_CTRL_XDONE) == 0U) {
      evt |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
    }

    if (i2c_resource->info->status.mode == 0U) {
      /* Lines are released in slave mode */
      i2c_resource->info->status.busy = 0U;

      i2c_resource->info->transfer_flags = 0U;
    }
  }
  /* Abort DMA streams */
//  if ((i2c_resource->dma_tx != NULL) && (i2c_resource->dma_rx != NULL)) {
//    HAL_DMA_Abort (i2c_resource->dma_tx->h);
//    HAL_DMA_Abort (i2c_resource->dma_rx->h);
//  }

  /* Clear error flags */
  i2c_resource->reg->SR1 &= ~err;

  if ((evt != 0) && (i2c_resource->info->cb_event != NULL)) {
    i2c_resource->info->cb_event (evt);
  }
}




#if (defined(MX_I2C1_TX_DMA_Instance) || \
     defined(MX_I2C2_TX_DMA_Instance) || \
     defined(MX_I2C3_TX_DMA_Instance))
/**
  \fn          void I2C_DMA_TxEvent (uint32_t event, I2C_RESOURCES *i2c_resource)
  \brief       I2C DMA Transmit Event handler
  \param[in]   i2c_resource  Pointer to I2C resources
*/
static void I2C_DMA_TxEvent (uint32_t event, I2C_RESOURCES *i2c_resource) {
  (void) event;

  i2c_resource->reg->CR2 &= ~I2C_CR2_DMAEN;

  i2c_resource->info->tx_info.count = (int32_t)(i2c_resource->info->tx_info.xfer_size - __HAL_DMA_GET_COUNTER(i2c_resource->dma_tx->h));

  if (i2c_resource->info->status.mode) {
    /* Master transmitter: Wait for BTF in I2C EV IRQ handler */
    i2c_resource->info->transfer_flags |= XFER_CTRL_DMA_DONE;
  }
}
#endif


#if (defined(MX_I2C1_RX_DMA_Instance) || \
     defined(MX_I2C2_RX_DMA_Instance) || \
     defined(MX_I2C3_RX_DMA_Instance))
/**
  \fn          void I2C_DMA_RxEvent (uint32_t event, I2C_RESOURCES *i2c_resource)
  \brief       I2C DMA Receive Event handler
  \param[in]   i2c_resource  Pointer to I2C resources
*/
static void I2C_DMA_RxEvent (uint32_t event, I2C_RESOURCES *i2c_resource) {
  (void) event;

  i2c_resource->reg->CR2 &= ~I2C_CR2_DMAEN;

  i2c_resource->info->rx_info.count = (int32_t)(i2c_resource->info->rx_info.xfer_size - __HAL_DMA_GET_COUNTER(i2c_resource->dma_rx->h));

  if (i2c_resource->info->status.mode) {
    /* Master mode */
    if (i2c_resource->info->transfer_flags & XFER_CTRL_XPENDING) {
      /* Transfer pending */
      i2c_resource->reg->CR2 &= ~I2C_CR2_ITEVTEN;
    }
    else {
      if (i2c_resource->info->rx_info.xfer_size != 1U) {
        i2c_resource->reg->CR1 |= I2C_CR1_STOP;
      }
    }

    i2c_resource->info->status.busy = 0U;
    i2c->info->status.mode = 0U;

    if (i2c_resource->info->cb_event) {
      i2c_resource->info->cb_event (ARM_I2C_EVENT_TRANSFER_DONE);
    }
  }
}
#endif


#if (defined(USE_I2C1)&&defined(I2C1))
/* Function prototypes */
  void I2C1_EV_IRQHandler (void);
  void I2C1_ER_IRQHandler (void);

  /* I2C1 DMA */
  #if defined(MX_I2C1_RX_DMA_Instance) && defined(MX_I2C1_TX_DMA_Instance)
  static const I2C_DMA I2C1_RX_DMA = {
    &hdma_i2c1_rx,
    &I2C1_RX_DMA_Complete,
    &I2C1_RX_DMA_Error,
    #if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
    MX_I2C1_RX_DMA_Instance,
    MX_I2C1_RX_DMA_IRQn,
    { 0U, 0U, 0U },
    MX_I2C1_RX_DMA_Channel,
    MX_I2C1_RX_DMA_Priority
    #endif
  };
  static const I2C_DMA I2C1_TX_DMA = {
    &hdma_i2c1_tx,
    &I2C1_TX_DMA_Complete,
    &I2C1_TX_DMA_Error,
    #if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
    MX_I2C1_TX_DMA_Instance,
    MX_I2C1_TX_DMA_IRQn,
    { 0U, 0U, 0U },
    MX_I2C1_TX_DMA_Channel,
    MX_I2C1_TX_DMA_Priority
    #endif
  };
  #endif

  /* I2C1 Information (Run-Time) */
  static I2C_INFO I2C1_Info;

  /* I2C1 Resources */
  static I2C_RESOURCES I2C1_Resources = {

    I2C1,
//  #if defined(MX_I2C1_RX_DMA_Instance) && defined(MX_I2C1_TX_DMA_Instance)
//    &I2C1_RX_DMA,
//    &I2C1_TX_DMA,
//  #else
//    NULL,
//    NULL,
//  #endif
    {
      I2C1_SCL_GPIO_PORT,
      I2C1_SDA_GPIO_PORT,
      I2C1_SCL_GPIO_PIN,
      I2C1_SDA_GPIO_PIN,
    },
    I2C1_EV_IRQn,
    I2C1_ER_IRQn,
//    0U,
    &I2C1_Info
  };
/* MX_I2C1 */


/* I2C1 Driver wrapper functions */
static ARM_DRIVER_STATUS I2C1_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return I2C_Initialize(cb_event, &I2C1_Resources);
}
static ARM_DRIVER_STATUS I2C1_Uninitialize (void) {
  return I2C_Uninitialize(&I2C1_Resources);
}
static ARM_DRIVER_STATUS I2C1_PowerControl (ARM_POWER_STATE state) {
  return I2C_PowerControl(state, &I2C1_Resources);
}
static ARM_DRIVER_STATUS I2C1_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterTransmit(addr, data, num, xfer_pending, &I2C1_Resources);
}
static ARM_DRIVER_STATUS I2C1_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterReceive(addr, data, num, xfer_pending, &I2C1_Resources);
}
static ARM_DRIVER_STATUS I2C1_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return I2C_SlaveTransmit(data, num, &I2C1_Resources);
}
static ARM_DRIVER_STATUS I2C1_SlaveReceive (uint8_t *data, uint32_t num) {
  return I2C_SlaveReceive(data, num, &I2C1_Resources);
}
static int32_t I2C1_GetDataCount (void) {
  return I2C_GetDataCount(&I2C1_Resources);
}
ARM_DRIVER_STATUS I2C1_Control (uint32_t control, uint32_t arg) {
  return I2C_Control(control, arg, &I2C1_Resources);
}
static ARM_I2C_STATUS I2C1_GetStatus (void) {
  return I2C_GetStatus(&I2C1_Resources);
}
void I2C1_EV_IRQHandler (void) {
  I2C_EV_IRQHandler(&I2C1_Resources);
}
void I2C1_ER_IRQHandler (void) {
  I2C_ER_IRQHandler(&I2C1_Resources);
}

#if defined(MX_I2C1_RX_DMA_Instance) && defined(MX_I2C1_TX_DMA_Instance)
#if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
void I2C1_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_I2C1_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
}
void I2C1_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_I2C1_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
}
#endif
void I2C1_RX_DMA_Complete(DMA_HandleTypeDef *hdma) {
  (void)hdma;
  I2C_DMA_RxEvent (DMA_COMPLETED, &I2C1_Resources);
}
void I2C1_RX_DMA_Error(DMA_HandleTypeDef *hdma) {
  (void)hdma;
  I2C_DMA_RxEvent (DMA_ERROR, &I2C1_Resources);
}
void I2C1_TX_DMA_Complete(DMA_HandleTypeDef *hdma) {
  (void)hdma;
  I2C_DMA_TxEvent (DMA_COMPLETED, &I2C1_Resources);
}
void I2C1_TX_DMA_Error(DMA_HandleTypeDef *hdma) {
  (void)hdma;
  I2C_DMA_TxEvent (DMA_ERROR, &I2C1_Resources);
}
#endif

/* I2C1 Driver Control Block */
ARM_DRIVER_I2C arm_i2c1 = {
  I2CX_GetVersion,
  I2CX_GetCapabilities,
  I2C1_Initialize,
  I2C1_Uninitialize,
  I2C1_PowerControl,
  I2C1_MasterTransmit,
  I2C1_MasterReceive,
  I2C1_SlaveTransmit,
  I2C1_SlaveReceive,
  I2C1_GetDataCount,
  I2C1_Control,
  I2C1_GetStatus
};

#endif


#if defined(MX_I2C2)
/* I2C2 Driver wrapper functions */
static int32_t I2C2_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return I2C_Initialize(cb_event, &I2C2_Resources);
}
static int32_t I2C2_Uninitialize (void) {
  return I2C_Uninitialize(&I2C2_Resources);
}
static int32_t I2C2_PowerControl (ARM_POWER_STATE state) {
  return I2C_PowerControl(state, &I2C2_Resources);
}
static int32_t I2C2_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterTransmit(addr, data, num, xfer_pending, &I2C2_Resources);
}
static int32_t I2C2_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterReceive(addr, data, num, xfer_pending, &I2C2_Resources);
}
static int32_t I2C2_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return I2C_SlaveTransmit(data, num, &I2C2_Resources);
}
static int32_t I2C2_SlaveReceive (uint8_t *data, uint32_t num) {
  return I2C_SlaveReceive(data, num, &I2C2_Resources);
}
static int32_t I2C2_GetDataCount (void) {
  return I2C_GetDataCount(&I2C2_Resources);
}
static int32_t I2C2_Control (uint32_t control, uint32_t arg) {
  return I2C_Control(control, arg, &I2C2_Resources);
}
static ARM_I2C_STATUS I2C2_GetStatus (void) {
  return I2C_GetStatus(&I2C2_Resources);
}
void I2C2_EV_IRQHandler (void) {
  I2C_EV_IRQHandler(&I2C2_Resources);
}
void I2C2_ER_IRQHandler (void) {
  I2C_ER_IRQHandler(&I2C2_Resources);
}

#if defined(MX_I2C2_RX_DMA_Instance) && defined(MX_I2C2_TX_DMA_Instance)
#if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
void I2C2_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_I2C2_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
}
void I2C2_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_I2C2_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_i2c2_tx);
}
#endif
void I2C2_RX_DMA_Complete(DMA_HandleTypeDef *hdma) {
  (void)hdma;
  I2C_DMA_RxEvent (DMA_COMPLETED, &I2C2_Resources);
}
void I2C2_RX_DMA_Error(DMA_HandleTypeDef *hdma) {
  (void)hdma;
  I2C_DMA_RxEvent (DMA_ERROR, &I2C2_Resources);
}
void I2C2_TX_DMA_Complete(DMA_HandleTypeDef *hdma) {
  (void)hdma;
  I2C_DMA_TxEvent (DMA_COMPLETED, &I2C2_Resources);
}
void I2C2_TX_DMA_Error(DMA_HandleTypeDef *hdma) {
  (void)hdma;
  I2C_DMA_TxEvent (DMA_ERROR, &I2C2_Resources);
}
#endif


/* I2C2 Driver Control Block */
ARM_DRIVER_I2C Driver_I2C2 = {
  I2CX_GetVersion,
  I2CX_GetCapabilities,
  I2C2_Initialize,
  I2C2_Uninitialize,
  I2C2_PowerControl,
  I2C2_MasterTransmit,
  I2C2_MasterReceive,
  I2C2_SlaveTransmit,
  I2C2_SlaveReceive,
  I2C2_GetDataCount,
  I2C2_Control,
  I2C2_GetStatus
};
#endif


#if defined(MX_I2C3)
/* I2C3 Driver wrapper functions */
static int32_t I2C3_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return I2C_Initialize(cb_event, &I2C3_Resources);
}
static int32_t I2C3_Uninitialize (void) {
  return I2C_Uninitialize(&I2C3_Resources);
}
static int32_t I2C3_PowerControl (ARM_POWER_STATE state) {
  return I2C_PowerControl(state, &I2C3_Resources);
}
static int32_t I2C3_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterTransmit(addr, data, num, xfer_pending, &I2C3_Resources);
}
static int32_t I2C3_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return I2C_MasterReceive(addr, data, num, xfer_pending, &I2C3_Resources);
}
static int32_t I2C3_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return I2C_SlaveTransmit(data, num, &I2C3_Resources);
}
static int32_t I2C3_SlaveReceive (uint8_t *data, uint32_t num) {
  return I2C_SlaveReceive(data, num, &I2C3_Resources);
}
static int32_t I2C3_GetDataCount (void) {
  return I2C_GetDataCount(&I2C3_Resources);
}
static int32_t I2C3_Control (uint32_t control, uint32_t arg) {
  return I2C_Control(control, arg, &I2C3_Resources);
}
static ARM_I2C_STATUS I2C3_GetStatus (void) {
  return I2C_GetStatus(&I2C3_Resources);
}
void I2C3_EV_IRQHandler (void) {
  I2C_EV_IRQHandler(&I2C3_Resources);
}
void I2C3_ER_IRQHandler (void) {
  I2C_ER_IRQHandler(&I2C3_Resources);
}

#if defined(MX_I2C3_RX_DMA_Instance) && defined(MX_I2C3_TX_DMA_Instance)
#if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
void I2C3_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_I2C3_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_i2c3_rx);
}
void I2C3_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_I2C3_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_i2c3_tx);
}
#endif
void I2C3_RX_DMA_Complete(DMA_HandleTypeDef *hdma) {
  (void)hdma;
  I2C_DMA_RxEvent (DMA_COMPLETED, &I2C3_Resources);
}
void I2C3_RX_DMA_Error(DMA_HandleTypeDef *hdma) {
  (void)hdma;
  I2C_DMA_RxEvent (DMA_ERROR, &I2C3_Resources);
}
void I2C3_TX_DMA_Complete(DMA_HandleTypeDef *hdma) {
  (void)hdma;
  I2C_DMA_TxEvent (DMA_COMPLETED, &I2C3_Resources);
}
void I2C3_TX_DMA_Error(DMA_HandleTypeDef *hdma) {
  (void)hdma;
  I2C_DMA_TxEvent (DMA_ERROR, &I2C3_Resources);
}
#endif


/* I2C3 Driver Control Block */
ARM_DRIVER_I2C Driver_I2C3 = {
  I2CX_GetVersion,
  I2CX_GetCapabilities,
  I2C3_Initialize,
  I2C3_Uninitialize,
  I2C3_PowerControl,
  I2C3_MasterTransmit,
  I2C3_MasterReceive,
  I2C3_SlaveTransmit,
  I2C3_SlaveReceive,
  I2C3_GetDataCount,
  I2C3_Control,
  I2C3_GetStatus
};
#endif


#if defined(MX_I2C2)
/* Function prototypes */
void I2C2_EV_IRQHandler (void);
void I2C2_ER_IRQHandler (void);

#if defined(RTE_DEVICE_FRAMEWORK_CUBE_MX)
extern I2C_HandleTypeDef hi2c2;
#endif

/* I2C2 DMA */
#if defined(MX_I2C2_RX_DMA_Instance) && defined(MX_I2C2_TX_DMA_Instance)
static const I2C_DMA I2C2_RX_DMA = {
  &hdma_i2c2_rx,
  I2C2_RX_DMA_Complete,
  I2C2_RX_DMA_Error,
  #if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
  MX_I2C2_RX_DMA_Instance,
  MX_I2C2_RX_DMA_IRQn,
  { 0U, 0U, 0U },
  MX_I2C2_RX_DMA_Channel,
  MX_I2C2_RX_DMA_Priority
  #endif
};
static const I2C_DMA I2C2_TX_DMA = {
  &hdma_i2c2_tx,
  I2C2_TX_DMA_Complete,
  I2C2_TX_DMA_Error,
  #if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
  MX_I2C2_TX_DMA_Instance,
  MX_I2C2_TX_DMA_IRQn,
  { 0U, 0U, 0U },
  MX_I2C2_TX_DMA_Channel,
  MX_I2C2_TX_DMA_Priority
  #endif
};
#endif

/* I2C2 Information (Run-Time) */
static I2C_INFO I2C2_Info;

/* I2C2 Resources */
static I2C_RESOURCES I2C2_Resources = {
#if defined(RTE_DEVICE_FRAMEWORK_CUBE_MX)
  &hi2c2,
#endif
  I2C2,
#if defined(MX_I2C2_RX_DMA_Instance) && defined(MX_I2C2_TX_DMA_Instance)
  &I2C2_RX_DMA,
  &I2C2_TX_DMA,
#else
  NULL,
  NULL,
#endif
  {
    MX_I2C2_SCL_GPIOx,
    MX_I2C2_SDA_GPIOx,
    MX_I2C2_SCL_GPIO_Pin,
    MX_I2C2_SDA_GPIO_Pin,
    MX_I2C2_SCL_GPIO_PuPdOD,
    MX_I2C2_SDA_GPIO_PuPdOD,
    MX_I2C2_SCL_GPIO_AF,
    MX_I2C2_SDA_GPIO_AF
  },
  I2C2_EV_IRQn,
  I2C2_ER_IRQn,
  0U,
  &I2C2_Info
};

#endif /* MX_I2C2 */

#if defined(MX_I2C3)
/* Function prototypes */
void I2C3_EV_IRQHandler (void);
void I2C3_ER_IRQHandler (void);

#if defined(RTE_DEVICE_FRAMEWORK_CUBE_MX)
extern I2C_HandleTypeDef hi2c3;
#endif

/* I2C3 DMA */
#if defined(MX_I2C3_RX_DMA_Instance) && defined(MX_I2C3_TX_DMA_Instance)
static const I2C_DMA I2C3_RX_DMA = {
  &hdma_i2c3_rx,
  I2C3_RX_DMA_Complete,
  I2C3_RX_DMA_Error,
  #if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
  MX_I2C3_RX_DMA_Instance,
  MX_I2C3_RX_DMA_IRQn,
  { 0U, 0U, 0U },
  MX_I2C3_RX_DMA_Channel,
  MX_I2C3_RX_DMA_Priority
  #endif
};
static const I2C_DMA I2C3_TX_DMA = {
  &hdma_i2c3_tx,
  I2C3_TX_DMA_Complete,
  I2C3_TX_DMA_Error,
  #if defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
  MX_I2C3_TX_DMA_Instance,
  MX_I2C3_TX_DMA_IRQn,
  { 0U, 0U, 0U },
  MX_I2C3_TX_DMA_Channel,
  MX_I2C3_TX_DMA_Priority
  #endif
};
#endif

/* I2C3 Information (Run-Time) */
static I2C_INFO I2C3_Info;

/* I2C3 Resources */
static I2C_RESOURCES I2C3_Resources = {
#if defined(RTE_DEVICE_FRAMEWORK_CUBE_MX)
  &hi2c3,
#endif
  I2C3,
#if defined(MX_I2C3_RX_DMA_Instance) && defined(MX_I2C3_TX_DMA_Instance)
  &I2C3_RX_DMA,
  &I2C3_TX_DMA,
#else
  NULL,
  NULL,
#endif
  {
    MX_I2C3_SCL_GPIOx,
    MX_I2C3_SDA_GPIOx,
    MX_I2C3_SCL_GPIO_Pin,
    MX_I2C3_SDA_GPIO_Pin,
    MX_I2C3_SCL_GPIO_PuPdOD,
    MX_I2C3_SDA_GPIO_PuPdOD,
    MX_I2C3_SCL_GPIO_AF,
    MX_I2C3_SDA_GPIO_AF
  },
  I2C3_EV_IRQn,
  I2C3_ER_IRQn,
  0U,
  &I2C3_Info
};

#endif /* MX_I2C3 */
