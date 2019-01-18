/**
  ******************************************************************************
  * @file    stm3210c_eval.c
  * @author  MCD Application Team
  * @version V6.1.0
  * @date    14-April-2017
  * @brief   This file provides a set of firmware functions to manage Leds, 
  *          push-button and COM ports for STM3210C_EVAL
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "./bsp.h"



/* BSP */
/*
  * support (function source code) & (definitions)  for *.c files in <API> folder
  * support (function source code) & (definitions)  for *.c files in <Components> folder
  */

//
///*
// * 	LED
// */
///* PD.07--LED--Ground*/
///* PD.13--LED--Ground*/
///* PD.03--LED--Ground*/
///* PD.04--LED--Ground*/
// GPIO_TypeDef* const led_port[LED_NUMBER] = {LED_GREEN_PORT,LED_ORANGE_PORT,LED_RED_PORT,LED_BLUE_PORT};
//const uint32_t led_pin[LED_NUMBER] = {LED_GREEN_PIN,LED_ORANGE_PIN,LED_RED_PIN,LED_BLUE_PIN};
//
//
//
//
///*
// *   BUTTON
// */
//// PA0--Button--3.3v (wakeup)
//// PB9--Button--Ground (key)
//// PC13--Button--Ground (Temper)
//GPIO_TypeDef* const button_port[BUTTON_NUMBER] = {BUTTON_KEY_PORT,BUTTON_TAMPER_PORT,BUTTON_WAKEUP_PORT};
//const uint32_t button_pin[BUTTON_NUMBER] = {BUTTON_KEY_PIN,BUTTON_TAMPER_PIN,BUTTON_WAKEUP_PIN};
//const uint8_t button_pull[BUTTON_NUMBER] = {LL_GPIO_PULL_UP,LL_GPIO_PULL_UP,LL_GPIO_PULL_DOWN};

















/*
 *     SOC I2C
 */
/**
  * @brief  This Function handle Slave events to perform a transmission process
  * @note  This function is composed in one step :
  *        -1- Prepare acknowledge for Slave address reception.
  * @param  None
  * @retval None
  */






/*
 * 		SOC		peripheral
 */





/*
*  									(ADC1)
*/
volatile uint32_t adc_value;

/* ADC1 init function */
void ADC1_Init(void)
{

/* Clock source */
LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);

/* Peripheral clock enable */
//
LL_ADC_Enable(ADC1);
/**ADC1 GPIO Configuration
PC4   ------> ADC1_IN14
*/


LL_GPIO_SetPinMode(POTENTION_METER_Port,POTENTION_METER_Pin,LL_GPIO_MODE_ANALOG);


/*Common config */
LL_ADC_SetDataAlignment(ADC1,LL_ADC_DATA_ALIGN_RIGHT);
LL_ADC_SetSequencersScanMode(ADC1,LL_ADC_SEQ_SCAN_DISABLE);
LL_ADC_SetMultimode(__LL_ADC_COMMON_INSTANCE(ADC1),LL_ADC_MULTI_INDEPENDENT);
LL_ADC_REG_SetTriggerSource(ADC1,LL_ADC_REG_TRIG_SOFTWARE);
LL_ADC_REG_SetSequencerLength(ADC1,LL_ADC_REG_SEQ_SCAN_DISABLE);
LL_ADC_REG_SetSequencerDiscont(ADC1,LL_ADC_REG_SEQ_DISCONT_DISABLE);
LL_ADC_REG_SetContinuousMode(ADC1,LL_ADC_REG_CONV_CONTINUOUS);
LL_ADC_REG_SetDMATransfer(ADC1,LL_ADC_REG_DMA_TRANSFER_NONE);

/*Configure Regular Channel*/
LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_14);
LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_14, LL_ADC_SAMPLINGTIME_1CYCLE_5);

LL_ADC_SetAnalogWDMonitChannels(ADC1,LL_ADC_AWD_CHANNEL_14_REG);

LL_ADC_SetAnalogWDThresholds(ADC1,LL_ADC_AWD_THRESHOLD_HIGH,4000);
LL_ADC_SetAnalogWDThresholds(ADC1,LL_ADC_AWD_THRESHOLD_LOW,200);

LL_ADC_EnableIT_AWD1(ADC1);
LL_ADC_EnableIT_EOS(ADC1);


/* ADC1 interrupt Init */
NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));
NVIC_EnableIRQ(ADC1_2_IRQn);


LL_ADC_REG_StartConversionSWStart(ADC1);
}


//
///*										(I2C1)
//*
//*/
//void I2C1_Init(void)
//{
///*
//* Clock enable
//*/
//LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
///* Add delay related to RCC workaround */
//while (READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN) != RCC_APB1ENR_I2C1EN) {};
//
//
///*
//* GPIO config
//*/
//LL_APB2_GRP1_EnableClock(I2C1_SCL_Clock);
//LL_APB2_GRP1_EnableClock(I2C1_SDA_Clock);
//LL_GPIO_SetPinMode(I2C1_SCL_Port,I2C1_SCL_Pin,LL_GPIO_MODE_ALTERNATE);
//LL_GPIO_SetPinOutputType(I2C1_SCL_Port,I2C1_SCL_Pin,LL_GPIO_OUTPUT_OPENDRAIN);
//LL_GPIO_SetPinSpeed(I2C1_SCL_Port,I2C1_SCL_Pin,LL_GPIO_SPEED_FREQ_HIGH);
//
//LL_GPIO_SetPinMode(I2C1_SDA_Port,I2C1_SDA_Pin,LL_GPIO_MODE_ALTERNATE);
//LL_GPIO_SetPinOutputType(I2C1_SDA_Port,I2C1_SDA_Pin,LL_GPIO_OUTPUT_OPENDRAIN);
//LL_GPIO_SetPinSpeed(I2C1_SDA_Port,I2C1_SDA_Pin,LL_GPIO_SPEED_FREQ_HIGH);
//
///* Force the I2C Periheral Clock Reset */
////EVAL_I2Cx_FORCE_RESET();
//
///* Release the I2C Periheral Clock Reset */
////EVAL_I2Cx_RELEASE_RESET();
//
///*I2C Initialization */
///* Disable the selected I2Cx Peripheral */
//LL_I2C_Disable(I2C1);
///*
//* I2C mode, aka standard mode
//*/
//LL_I2C_SetMode(I2C1,LL_I2C_MODE_I2C);
//// @note   When disabled the Address 0x00 is NACKed.
//LL_I2C_DisableGeneralCall(I2C1);
//LL_I2C_EnableClockStretching(I2C1);
//
//LL_I2C_SetClockSpeedMode(I2C1,LL_I2C_CLOCK_SPEED_STANDARD_MODE);
//
////?????????
//LL_I2C_SetClockPeriod(I2C1,4);
//LL_I2C_SetDutyCycle(I2C1,LL_I2C_DUTYCYCLE_2);
//LL_I2C_SetPeriphClock(I2C1,rcc_clocks.PCLK1_Frequency);
//
////100000 is the frequency of SCL
//LL_I2C_SetRiseTime(I2C1,__LL_I2C_RISE_TIME(__LL_I2C_FREQ_HZ_TO_MHZ(LL_I2C_GetPeriphClock(I2C1)),100000));
//
//
//LL_I2C_SetOwnAddress1(I2C1,0x123,LL_I2C_OWNADDRESS1_7BIT);
//LL_I2C_SetOwnAddress2(I2C1, 0);
//LL_I2C_DisableOwnAddress2(I2C1);
//
//
///* I2C1 interrupt Init */
//
//LL_I2C_DisableDMAReq_RX(I2C1);
//LL_I2C_DisableLastDMA(I2C1);
//
//LL_I2C_EnableIT_EVT(I2C1);
//LL_I2C_EnableIT_ERR(I2C1);
////TxE=1|RxNE=1 generate event interrupt
//LL_I2C_EnableIT_BUF(I2C1);
//
///*
//* event interrupt
//*/
//NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),14, 0));
//NVIC_EnableIRQ(I2C1_EV_IRQn);
///*
//* error interrupt
//*/
//NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),14, 0));
//NVIC_EnableIRQ(I2C1_ER_IRQn);
//
//LL_I2C_Enable(I2C1);
//
//LL_I2C_AcknowledgeNextData(I2C1,LL_I2C_ACK);
//
//}












/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
