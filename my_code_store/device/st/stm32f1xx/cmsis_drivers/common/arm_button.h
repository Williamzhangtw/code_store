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

#ifndef ARM_BUTTON_H_
#define ARM_BUTTON_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include "driver_common.h"
#define ARM_BUTTON_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,3)  /* API version */


typedef struct _ARM_BUTTON_CAPABILITIES {
  uint32_t reserved       : 32;         ///< Reserved (must be zero)
} ARM_BUTTON_CAPABILITIES;

typedef enum _BUTTON_STATE
{
  PRESSED = 0,
  RELEASED =1,
}BUTTON_STATE;




typedef void(*ARM_BUTTON_SIGNAL_EVENT) (uint32_t event);

/**
\brief Access structure of the I2C Driver.
*/
typedef struct _ARM_BUTTON{
  ARM_DRIVER_VERSION   (*GetVersion)     (void);
  ARM_BUTTON_CAPABILITIES (*GetCapabilities)(void);

  int32_t              (*Initialize)     (ARM_BUTTON_SIGNAL_EVENT cb_event);
  int32_t              (*Uninitialize)   (void);
//  int32_t              (*PowerControl)   (ARM_POWER_STATE state);
  BUTTON_STATE              (*GetState)         (void);
} const ARM_BUTTON;

#ifdef  __cplusplus
}
#endif

#endif
