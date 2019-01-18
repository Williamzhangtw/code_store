#include "./delay.h"




#if defined DWT
uint32_t microsecond_divider;
uint32_t millisecond_divider;
#else
volatile int32_t	milliseconds;
volatile int32_t microseconds;
#endif



void UTILS_MicroSecondInit(void)
{
#if defined DWT
	LL_RCC_ClocksTypeDef rcc_clocks;
	LL_RCC_GetSystemClocksFreq(&rcc_clocks);
	microsecond_divider = rcc_clocks.HCLK_Frequency/1000000;
#else /* Using timer to generate microsecnod delay */
/* General-purpose timers (TIM2 to TIM5) */
#if !defined TIMER3_ALTERNATE_GPIO
//	#define TIMER3_ALTERNATE_GPIO
#endif
#if defined TIMER3_ALTERNATE_GPIO
//AFIO GPIO settings
/**TIM3 GPIO Configuration  PC6   ------> TIM3_CH1 */
//	LL_APB2_GRP1_EnableClock(TIM3_CHANNEL_1_GPIO_Clock);
//	LL_APB2_GRP1_EnableClock(RCC_APB2ENR_AFIOEN);
//	LL_GPIO_AF_EnableRemap_TIM3();
//	LL_GPIO_SetPinMode(TIM3_CHANNEL_1_Port,TIM3_CHANNEL_1_Pin,LL_GPIO_MODE_ALTERNATE);
//	LL_GPIO_SetPinOutputType(TIM3_CHANNEL_1_Port,TIM3_CHANNEL_1_Pin,LL_GPIO_OUTPUT_PUSHPULL);
//	LL_GPIO_SetPinSpeed(TIM3_CHANNEL_1_Port,TIM3_CHANNEL_1_Pin,LL_GPIO_SPEED_FREQ_HIGH);

/**TIM3 GPIO Configuration  PC5   ------> TIM3_CH2 */
LL_APB2_GRP1_EnableClock(TIM3_CHANNEL_2_GPIO_Clock);
LL_GPIO_SetPinMode(TIM3_CHANNEL_2_Port,TIM3_CHANNEL_2_Pin,LL_GPIO_MODE_FLOATING);
#endif

/* Timer peripheral settings */

/*basic settings*/
#if !defined TIMER3_BASIC_Settings
#define TIMER3_BASIC_Settings
#endif
#if defined TIMER3_BASIC_Settings
/* Peripheral clock enable */
LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

#if !defined TIMER3_CLOCK_SOURCE
/*
* @param  TIMER3_CLOCK_SOURCE This parameter can be a combination of the following values:
*    @arg @ref 1	CLOCKSOURCE_INTERNAL
*    @arg @ref 2	CLOCKSOURCE_EXT_MODE1
*    @arg @ref 3	CLOCKSOURCE_EXT_MODE2
*/
#define TIMER3_CLOCK_SOURCE 1
#endif

#if TIMER3_CLOCK_SOURCE ==1
LL_TIM_SetClockSource(TIM3,LL_TIM_CLOCKSOURCE_INTERNAL);
#endif

#if TIMER3_CLOCK_SOURCE ==2
LL_TIM_SetClockSource(TIM3,LL_TIM_CLOCKSOURCE_EXT_MODE1);
/*
1. channel 2 is configured as input, IC2 is mapped on TI2 by writting CC2S='01'@TIMx_CCMR1 register
*/
LL_TIM_IC_SetActiveInput(TIM3,LL_TIM_CHANNEL_CH2,LL_TIM_ACTIVEINPUT_DIRECTTI);
/*
2. Configure the input filter duration by writting the IC2F[3:0] bits @TIMx_CCMR1 register
*/
/*????????????????????*/
LL_TIM_IC_SetFilter(TIM3,LL_TIM_CHANNEL_CH2,LL_TIM_ETR_FILTER_FDIV1);
/*
3. Select rising edge polarity by writting CC2P=0 @TIMx_CCER register
*/
LL_TIM_IC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH2,LL_TIM_IC_POLARITY_RISING);

/*
* select TI2 as the input source by writting TS='110'@TIMx_SMCR register
*/
LL_TIM_SetTriggerInput(TIM3,LL_TIM_TS_TI2FP2);

/*
* When a active edge(rising or falling) on TI2, the counter counts once and the TIF flag is set
*/

#endif

#if TIMER3_CLOCK_SOURCE ==3
LL_TIM_SetClockSource(TIM3,LL_TIM_CLOCKSOURCE_EXT_MODE2);
/* ?????????LL_TIM_ETR_FILTER_FDIV1*/
LL_TIM_ConfigETR(TIM3,LL_TIM_ETR_POLARITY_NONINVERTED,LL_TIM_ETR_PRESCALER_DIV8,LL_TIM_ETR_FILTER_FDIV1);
LL_TIM_EnableExternalClock(TIM3);
#endif

/* frequency formula of timer output
OutPut(hz) = CK_PSC/((PSR+1)*(ARR+1))*/
LL_TIM_SetPrescaler(TIM3,((rcc_clocks.PCLK1_Frequency<<1)/2000000-1));
//	LL_TIM_SetPrescaler(TIM3,(HSE_VALUE/5000000-1));


/* ARR */
LL_TIM_SetAutoReload(TIM3,(2-1));
LL_TIM_SetCounterMode(TIM3,LL_TIM_COUNTERMODE_DOWN);
/* enable ARR shadow register */
LL_TIM_EnableARRPreload(TIM3);
//	LL_TIM_DisableARRPreload(TIM3);

/* set the division ratio between the timer clock(CK_INT)
frequency and sampling clock used by the digital filters
????????????????????????????????????????????????????????
digital filter clk settings
*/
LL_TIM_SetClockDivision(TIM3,LL_TIM_CLOCKDIVISION_DIV1);

LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
LL_TIM_DisableMasterSlaveMode(TIM3);


#endif

/* input channel settings */
#if !defined TIMER3_INPUT_CHANNEL_Settings
//		#define TIMER3_INPUT_CHANNEL_Settings
#endif
#if defined TIMER3_INPUT_CHANNEL_Settings
/*
*  configure input source for timer channelx
*/
LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
/*
* filter the noise created by signal toggling
*/
LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
/*
* LL_TIM_IC_POLARITY_RISING:Trigger signal generated when Rising edge occurs(Level high)
* LL_TIM_IC_POLARITY_FALLING:Trigger signal generated when Falling edge occurs(Level Low)
*/
LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);

LL_TIM_IC_SetPrescaler(TIM3,LL_TIM_CHANNEL_CH2,LL_TIM_ICPSC_DIV1);
/*
* Timer channel Capture enable
*/
LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH2);
//	LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH2);
#endif

/* output channel settings */

//	LL_TIM_OC_SetCompareCH1(TIM3,0xffff);
//	LL_TIM_OC_SetCompareCH2(TIM3,0xffff);
//	LL_TIM_OC_SetCompareCH3(TIM3,0xffff);
//	LL_TIM_OC_SetCompareCH4(TIM3,0xffff);


#if !defined TIMER3_OUTPUT_CHANNEL_Settings
//		#define TIMER3_OUTPUT_CHANNEL_Settings
#endif
#if defined TIMER3_OUTPUT_CHANNEL_Settings
LL_TIM_OC_SetCompareCH1(TIM3,0xffff);

LL_TIM_OC_SetMode(TIM3,LL_TIM_CHANNEL_CH1,LL_TIM_OCMODE_PWM1);
/*
* When preload enable, the shadow register will be updated until next UEV
*/
LL_TIM_OC_EnablePreload(TIM3,LL_TIM_CHANNEL_CH1);
/*
* OCx active high:LL_TIM_OCPOLARITY_HIGH
* OCx active low:LL_TIM_OCPOLARITY_LOW
*/
LL_TIM_OC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
/*
* Use in OnePulse situation to reduce the time between pulse start trigger and pulse
*/
LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH1);
/*
* OCx signal is output on the corresponding output pin
*/
//	LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH1);
//	LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH2);

#endif

/* interrupt */
NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),6, 0));
NVIC_EnableIRQ(TIM3_IRQn);
//	LL_TIM_EnableIT_CC1(TIM3);
LL_TIM_EnableIT_UPDATE(TIM3);
LL_TIM_EnableCounter(TIM3);
#endif
}

void UTILS_MilliSecondInit(void)
{
#if defined DWT
	LL_RCC_ClocksTypeDef rcc_clocks;
	LL_RCC_GetSystemClocksFreq(&rcc_clocks);
	millisecond_divider = rcc_clocks.HCLK_Frequency/1000;

#else
	// Systick
	SysTick_Config(SystemCoreClock/1000);
	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
#endif
}
/**
* @brief  Systick delay function (interrupt mode)
* @param  n_m_second: Specifies how many milisecond you want to delay.
*/
void UTILS_MilliSecondDelay(uint32_t count)
{
#if defined DWT
	uint32_t tickstart = DWT->CYCCNT;
	uint32_t wait = count*millisecond_divider;
	while ((DWT->CYCCNT- tickstart)<wait);
#else
  uint32_t tickstart = milliseconds;
  uint32_t wait = count;
  while ((milliseconds - tickstart) < wait);
#endif
}

/*
*  uDelay								(TIM3)
*/
/**
* @brief  Timer3 delay function (interrupt mode)
* @param  n_m_second: Specifies how many milisecond you want to delay.
*/
void UTILS_MicroSecondDelay(uint32_t count)
{
#if defined DWT
	uint32_t tickstart = DWT->CYCCNT;
	/* Go to number of cycles for system */
	uint32_t wait =count* microsecond_divider;
	/* Delay till end */
	while ((DWT->CYCCNT - tickstart) < wait);      // change value of 15 to adjust timing (microseconds-x)
#else
	uint32_t tickstart = microseconds;
	uint32_t wait = count;
	while ((microseconds - tickstart) < wait);
#endif
}

#if !defined DWT
void SysTick_Handler(void)
{
	
	if(LL_SYSTICK_IsActiveCounterFlag())
	{
		milliseconds++;
	}
	
}

void TIM3_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_UPDATE(TIM3))
	{
		LL_TIM_ClearFlag_UPDATE(TIM3);
		microseconds++;
	}
}
#endif
