#if !defined __I2C_TP_RIT_H__
 #define __I2C_TP_RIT_H__
 
 
#include "api.h" 

extern uint8_t*      pTransmitBuffer ;

 void Configure_I2C_Master(void);
 void Configure_I2C_Slave(void);
 void Handle_I2C_Slave(void);
 void Activate_I2C_Slave(void);
 void Slave_Reception_Callback(void);
 void Slave_Complete_Callback(void);
 void Activate_I2C_Master(void);
 void Handle_I2C_Master(void);
 
 void I2C1_Transfer_Master_Callback(void);
 
#endif
