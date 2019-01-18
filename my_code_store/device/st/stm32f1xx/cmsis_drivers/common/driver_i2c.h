/*
 * Copyright (c) 2013-2017 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * $Date:        2. Feb 2017
 * $Revision:    V2.3
 *
 * Project:      I2C (Inter-Integrated Circuit) Driver definitions
 */

/* History:
 *  Version 2.3
 *    ARM_I2C_STATUS made volatile
 *  Version 2.2
 *    Removed function ARM_I2C_MasterTransfer in order to simplify drivers
 *      and added back parameter "xfer_pending" to functions
 *      ARM_I2C_MasterTransmit and ARM_I2C_MasterReceive
 *  Version 2.1
 *    Added function ARM_I2C_MasterTransfer and removed parameter "xfer_pending"
 *      from functions ARM_I2C_MasterTransmit and ARM_I2C_MasterReceive
 *    Added function ARM_I2C_GetDataCount
 *    Removed flag "address_nack" from ARM_I2C_STATUS
 *    Replaced events ARM_I2C_EVENT_MASTER_DONE and ARM_I2C_EVENT_SLAVE_DONE
 *      with event ARM_I2C_EVENT_TRANSFER_DONE
 *    Added event ARM_I2C_EVENT_TRANSFER_INCOMPLETE
 *    Removed parameter "arg" from function ARM_I2C_SignalEvent
 *  Version 2.0
 *    New simplified driver:
 *      complexity moved to upper layer (especially data handling)
 *      more unified API for different communication interfaces
 *    Added:
 *      Slave Mode
 *    Changed prefix ARM_DRV -> ARM_DRIVER
 *  Version 1.10
 *    Namespace prefix ARM_ added
 *  Version 1.00
 *    Initial release
 */

#ifndef DRIVER_I2C_H_
#define DRIVER_I2C_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include "driver_common.h"

#define ARM_I2C_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,3)  /* API version */


/****** I2C Control Codes *****/

#define ARM_I2C_OWN_ADDRESS             (0x01)      ///< Set Own Slave Address; arg = address 
#define ARM_I2C_BUS_SPEED               (0x02)      ///< Set Bus Speed; arg = speed
#define ARM_I2C_BUS_CLEAR               (0x03)      ///< Execute Bus clear: send nine clock pulses
#define ARM_I2C_ABORT_TRANSFER          (0x04)      ///< Abort Master/Slave Transmit/Receive

/*----- I2C Bus Speed -----*/
#define ARM_I2C_BUS_SPEED_STANDARD      (0x01)      ///< Standard Speed (100kHz)
#define ARM_I2C_BUS_SPEED_FAST          (0x02)      ///< Fast Speed     (400kHz)
#define ARM_I2C_BUS_SPEED_FAST_PLUS     (0x03)      ///< Fast+ Speed    (  1MHz)
#define ARM_I2C_BUS_SPEED_HIGH          (0x04)      ///< High Speed     (3.4MHz)


/****** I2C Address Flags *****/

#define ARM_I2C_ADDRESS_10BIT           (0x0400)    ///< 10-bit address flag
#define ARM_I2C_ADDRESS_GC              (0x8000)    ///< General Call flag



/*
 * @brief indicate the status of I2C peripheral
 */
typedef volatile struct _ARM_I2C_STATUS {
  uint32_t busy             : 1;        ///< Busy flag
  uint32_t mode             : 1;        ///< Mode: 0=Slave, 1=Master
  uint32_t direction        : 1;        ///< Direction: 0=Transmitter, 1=Receiver
  uint32_t general_call     : 1;        ///< General Call indication (cleared on start of next Slave operation)
  uint32_t arbitration_lost : 1;        ///< Master lost arbitration (cleared on start of next Master operation)
  uint32_t bus_error        : 1;        ///< Bus error detected (cleared on start of next Master/Slave operation)

  uint32_t initialized      : 1;        /* GPIO initialized flag */
  uint32_t powered          : 1;        /* I2C powered flag */
  uint32_t reserved         : 24;
} ARM_I2C_STATUS;


/****** I2C Event *****/
#define ARM_I2C_EVENT_TRANSFER_DONE       (1UL << 0)
    /*
     * Occurs after Master/Slave Transmit/Receive operation has finished (Stop condition,)
     */
#define ARM_I2C_EVENT_TRANSFER_INCOMPLETE (1UL << 1)
    /*
     * Occurs together with ARM_I2C_EVENT_TRANSFER_DONE when less data is transferred then requested
     */
#define ARM_I2C_EVENT_SLAVE_TRANSMIT      (1UL << 2)
    /*
     * Occurs when addressed as Slave Transmitter and ARM_I2C_SlaveTransmit has not been started.
     * (num = 0(no byte need to transfer)|XFER_CTRL_WAIT_BTF (do not need tranfer any more))
     */
#define ARM_I2C_EVENT_SLAVE_RECEIVE       (1UL << 3)
    /* Occurs when addressed as Slave Receiver and ARM_I2C_SlaveReceive has not been started
     * (XFER_CTRL_WAIT_BTF(do not need read DR any more)
     *
     * Occurs in Slave receiver mode, when 'num' of data has been all received
     */
#define ARM_I2C_EVENT_ADDRESS_NACK        (1UL << 4)
    /*
     * Occurs in master mode when address is not acknowledged from slave.
     */
#define ARM_I2C_EVENT_GENERAL_CALL        (1UL << 5)
    /*
     * Indicates General Call in slave mode together with ARM_I2C_EVENT_TRANSFER_DONE, ARM_I2C_EVENT_SLAVE_TRANSMIT
     * and ARM_I2C_EVENT_SLAVE_RECEIVE
     */
#define ARM_I2C_EVENT_ARBITRATION_LOST    (1UL << 6)
    /*
     * Occurs in master mode when arbitration is lost.
     */
#define ARM_I2C_EVENT_BUS_ERROR           (1UL << 7)
    /*
     * Occurs when bus error is detected.(START/STOP at illegal position)
     */
#define ARM_I2C_EVENT_BUS_CLEAR           (1UL << 8)
    /*
     * Occurs after ARM_I2C_BUS_CLEAR Control operation has finished.
     */

/*
 * The I2C driver generates call back events that are notified via the function ARM_I2C_SignalEvent
 */
typedef void (*ARM_I2C_SignalEvent_t) (uint32_t event);  ///< Pointer to \ref ARM_I2C_SignalEvent : Signal I2C Event.


/**
\brief I2C Driver Capabilities.
*/
typedef struct _ARM_I2C_CAPABILITIES {
  uint32_t address_10_bit : 1;          ///< supports 10-bit addressing
  uint32_t reserved       : 31;         ///< Reserved (must be zero)
} ARM_I2C_CAPABILITIES;


/**
\brief Access structure of the I2C Driver.
*/
typedef struct _ARM_DRIVER_I2C {
  ARM_DRIVER_VERSION (*GetVersion) (void);
  ARM_I2C_CAPABILITIES (*GetCapabilities)(void);
  ARM_DRIVER_STATUS (*Initialize) (ARM_I2C_SignalEvent_t cb_event);
  ARM_DRIVER_STATUS (*Uninitialize) (void);
  ARM_DRIVER_STATUS (*PowerControl) (ARM_POWER_STATE state);
  ARM_DRIVER_STATUS (*MasterTransmit) (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending);
  ARM_DRIVER_STATUS (*MasterReceive) (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending);
  ARM_DRIVER_STATUS (*SlaveTransmit) (const uint8_t *data, uint32_t num);
  ARM_DRIVER_STATUS (*SlaveReceive) (uint8_t *data, uint32_t num);
  int32_t (*GetDataCount) (void);
  ARM_DRIVER_STATUS (*Control) (uint32_t control, uint32_t arg);
  ARM_I2C_STATUS (*GetStatus) (void);
} const ARM_DRIVER_I2C;

#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_I2C_H_ */
