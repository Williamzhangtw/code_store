/* -----------------------------------------------------------------------------
* Copyright (c) 2013-2017 ARM Ltd.
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
* $Date:        10. November 2017
* $Revision:    V1.0
*
* Project:      I2C Driver definitions for STMicroelectronics STM32F0xx
* -------------------------------------------------------------------------- */

#if !defined I2C_STM32F1XX_H_
#define I2C_STM32F1XX_H_


#include "../utilities/utilities.h"
#include "./common/driver_i2c.h"


/* Bus Clear clock period definition */
#define I2C_BUS_CLEAR_CLOCK_PERIOD      (2)   // I2C bus clock period (ms)

/*
* @brief Current driver status flag definition
* @note used to the status of I2C peripheral, suplimental for i2c_resource->info->status
*
*/
//  typedef enum _I2C_FLAGS
//  {
//      I2C_INIT = ((uint8_t)0x01),   // I2C initialized
//      I2C_POWER = ((uint8_t)0x02),   // I2C powered on
//      I2C_SETUP = ((uint8_t)0x04),   // I2C configured
//      I2C_DMA_TX = ((uint8_t)0x08),   // Tx DMA enabled
//      I2C_DMA_RX = ((uint8_t)0x10),   // Rx DMA enabled
//      I2C_XFER_SET = ((uint8_t)0x20),   // Transfer buffer registered
//      I2C_XFER_FIRST = ((uint8_t)0x40),
//      I2C_XFER_NEXT = ((uint8_t)0x80),
//  }I2C_FLAGS;

#define I2C_MODE_SLAVE      ((bool)0)   // Mode: slave
#define I2C_MODE_MASTER     ((bool)1)   // Mode: master

#define I2C_DIR_TX          ((bool)0)   // Direction: transmitter
#define I2C_DIR_RX          ((bool)1)   // Direction: receiver


/*
* @brief Transfering status flags
* @note  Use to control the interrupt handler workflow
*/
typedef enum _I2C_TRANSFER_FLAGS
{
XFER_CTRL_XPENDING  = ((uint8_t)0x01),   // Transfer pending
XFER_CTRL_XACTIVE =  ((uint8_t)0x02),   // Transfer active
XFER_CTRL_XDONE  =   ((uint8_t)0x04),   // Transfer done
XFER_CTRL_RSTART  =  ((uint8_t)0x08),   // Generate repeated start and readdress
XFER_CTRL_ADDR_DONE = ((uint8_t)0x10),   // Addressing done
XFER_CTRL_DMA_DONE = ((uint8_t)0x20),   // DMA transfer done
XFER_CTRL_WAIT_BTF = ((uint8_t)0x40),   // Wait for byte transfer finished
}I2C_TRANSFER_FLAGS;


/* DMA Event definitions */
#define DMA_COMPLETED             0U
#define DMA_ERROR                 1U

//  /* DMA Callback functions */
//  typedef void (*DMA_Callback_t) (DMA_HandleTypeDef *hdma);

//  /* DMA Information definitions */
//  typedef struct _I2C_DMA {
//    DMA_HandleTypeDef    *h;
//    DMA_Callback_t        cb_complete;
//    DMA_Callback_t        cb_error;
//  //#if defined USE_HAL
//  //  DMA_Stream_TypeDef   *stream;               // Stream register interface
//  //  IRQn_Type             irq_num;              // Stream IRQ number
//  //  uint8_t               reserved[3];          // Reserved
//  //  uint32_t              channel;              // Channel number
//  //  uint32_t              priority;             // Stream priority
//  //#endif
//  } const I2C_DMA;


/* I2C Input/Output Configuration */
typedef const struct _I2C_IO {
GPIO_TypeDef         *scl_port;             // SCL IO Port
GPIO_TypeDef         *sda_port;             // SDA IO Port
uint16_t              scl_pin;              // SCL IO Pin
uint16_t              sda_pin;              // SDA IO Pin
} I2C_IO;

/* I2C Transfer Information (Run-Time) */
typedef struct _I2C_TRANSFER_INFO {
uint8_t              *data;                 // Data pointer
uint32_t              xfer_size;                  // Number of data to transfer
int32_t               count;                  // Data transfer counter
} I2C_TRANSFER_INFO;


/* I2C Information (Run-Time) */
typedef struct _I2C_INFO {
ARM_I2C_SignalEvent_t cb_event;             // Event Callback
ARM_I2C_STATUS        status;               // Status flags
I2C_TRANSFER_INFO     rx_info;                   // Rx transfer information
I2C_TRANSFER_INFO     tx_info;                   // Tx transfer information
uint16_t              addr;                 // Transfer address
I2C_TRANSFER_FLAGS    transfer_flags;                 // Transfer control
//    I2C_FLAGS             i2c_flags;                // Current I2C state flags
} I2C_INFO;

/* I2C Resources definition */
typedef struct {
I2C_TypeDef          *reg;                  // I2C peripheral register interface
//    I2C_DMA              *dma_rx;               // I2C DMA Configuration
//    I2C_DMA              *dma_tx;               // I2C DMA Configuration
I2C_IO                io;                   // I2C Input/Output pins
IRQn_Type             ev_irq_num;           // I2C Event IRQ Number
IRQn_Type             er_irq_num;           // I2C Error IRQ Number
//  uint16_t              reserved;             // Reserved
I2C_INFO             *info;                 // Run-Time information
} const I2C_RESOURCES;


// Global functions and variables exported by driver .c module



#if (defined(USE_I2C1)&&defined(I2C1))
extern ARM_DRIVER_I2C arm_i2c1;
extern void I2C1_EV_IRQHandler  (void);
extern void I2C1_ER_IRQHandler  (void);
extern void I2C1_RX_DMA_Handler (void);
extern void I2C1_TX_DMA_Handler (void);
#endif

#ifdef MX_I2C2
extern void I2C2_EV_IRQHandler  (void);
extern void I2C2_ER_IRQHandler  (void);
extern void I2C2_RX_DMA_Handler (void);
extern void I2C2_TX_DMA_Handler (void);

extern ARM_DRIVER_I2C Driver_I2C2;
#endif

#ifdef MX_I2C3
extern void I2C3_EV_IRQHandler  (void);
extern void I2C3_ER_IRQHandler  (void);
extern void I2C3_RX_DMA_Handler (void);
extern void I2C3_TX_DMA_Handler (void);

extern ARM_DRIVER_I2C Driver_I2C3;
#endif

#if defined(MX_I2C1_RX_DMA_Instance)
extern DMA_HandleTypeDef hdma_i2c1_rx;
#endif
#if defined(MX_I2C1_TX_DMA_Instance)
extern DMA_HandleTypeDef hdma_i2c1_tx;
#endif



#if defined(MX_I2C2_RX_DMA_Instance)
extern DMA_HandleTypeDef hdma_i2c2_rx;
#endif
#if defined(MX_I2C2_TX_DMA_Instance)
extern DMA_HandleTypeDef hdma_i2c2_tx;
#endif


#endif /* I2C_STM32F1XX_H_ */

