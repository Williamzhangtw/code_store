#include "i2c.h"
//STMPE811 Timing
// SCL clock frequency max 400khz
//delay
//SCL falling edge the Level low period should last at lease 1.3µs  // uDelay(2);
//SCL rising edge the level high period should last at lease 600ns  // uDelay(1);
//tBuf the time the bus must be free before Start signal at lease 1.3µs



/* I2C software version */

void I2C_SW_Init(void)
{
	ZTW_GPIO_Clock_Enable(I2C_SW_SCL_GPIO_PORT);
	ZTW_GPIO_Clock_Enable(I2C_SW_SDA_GPIO_PORT);

	LL_GPIO_SetPinMode(I2C_SW_SCL_GPIO_PORT,I2C_SW_SCL_GPIO_PIN,LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(I2C_SW_SCL_GPIO_PORT,I2C_SW_SCL_GPIO_PIN,LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinSpeed(I2C_SW_SCL_GPIO_PORT,I2C_SW_SCL_GPIO_PIN,LL_GPIO_SPEED_FREQ_HIGH);

	LL_GPIO_SetPinMode(I2C_SW_SDA_GPIO_PORT,I2C_SW_SDA_GPIO_PIN,LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(I2C_SW_SDA_GPIO_PORT,I2C_SW_SDA_GPIO_PIN,LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinSpeed(I2C_SW_SDA_GPIO_PORT,I2C_SW_SDA_GPIO_PIN,LL_GPIO_SPEED_FREQ_HIGH);
}

void SCL_OutputHigh(void)
{
	*SCL = 1;
}
void SDA_OutputHigh(void)
{
	*SDA = 1;
}
void SCL_OutputLow(void)
{
	*SCL = 0;
}
void SDA_OutputLow(void)
{
	*SDA = 0;
}

uint32_t SCL_read(void){
	uint32_t SCL_level;
	*SCL = 1;
	SCL_level = LL_GPIO_IsInputPinSet(I2C_SW_SCL_GPIO_PORT,I2C_SW_SCL_GPIO_PIN);
	return SCL_level;
}

uint32_t SDA_read(void){
	uint32_t SDA_level;
	*SDA = 1;
	SDA_level = LL_GPIO_IsInputPinSet(I2C_SW_SDA_GPIO_PORT,I2C_SW_SDA_GPIO_PIN);
	return SDA_level;
}

void I2C_SW_Start(void)
{
	SDA_HIGH();
	uDelay(1);//add
	SCL_HIGH();
	uDelay(2);
	SDA_LOW();
	//SDA high -> low
	uDelay(2);
}

void I2C_SW_Stop(void)
{
	 SDA_LOW();
	 uDelay(1);//add
	 SCL_HIGH();
	 uDelay(1);//add
	 SDA_HIGH();
	 //	 SDA low -> high
	 uDelay(1);
}

uint32_t I2C_SW_Receive(uint32_t ack)
{
	uint32_t index =0;
	uint32_t data = 0;
	// release I2C bus
	SCL_HIGH();
	SDA_HIGH();
	for(index = 0; index <8; index++)
	{
		data=data<<1;

		while(SCL_IN()==0)
		{
			
		}
		if(SDA_IN())
		{
			data|=1;
		}
//		SCL_Out = 0; wrong??
	}
	//because the time of SCL==1 is very short.300ns
	//but it still may cause conflict
	//it has to make sure the SCL==0 at this point
	//maybe it don't have to.
	//because the bus is control by the slave. The slave won't recognize that was a start signal
	if(ack)
	{
		SDA_HIGH();
	}
	else
	{
		SDA_LOW();
	}
	while(SCL_IN()==0)
	{
		
	}
	// delay
	SCL_LOW(); // occupied the bus
	return data;
}


uint32_t I2C_SW_Transmit(uint32_t data)
{
	uint32_t ack;
	uint32_t index = 0;
	SCL_LOW();
	uDelay(2);//add
	for (index = 0;index<8;index++)
	{
		//prepare data in the data bus(SDA)
		//MSB first
		if(data&0x80)
		{
			SDA_HIGH();
		}
		else
		{
			SDA_LOW();
		}
		data=data<<1;
		//acknowledge the slave to read the data
		uDelay(1);//add
		SCL_HIGH();
		// delay
		uDelay(1);//add
		SCL_LOW();
		uDelay(2);//add
	}
	//prepare to receive ack signal from slave(release the SDA bus)
	SDA_HIGH();
	//acknowledge the slave to send the ack (may already send)
	SCL_HIGH();
	ack=SDA_IN(); //read the ack
	uDelay(1);
	SCL_LOW(); // end the SCL clock cycle as well as occupying the bus
	uDelay(2);
	return ack;
}



