#include "soc_init.h"
#include "diagnose.h"



/*
system_timer
NVIC
*/



/**
  * @brief System Clock Configuration
  * @retval None
  */

void UTILS_NvicInit(void)
{
	/* PREFETCH_ENABLE */
  #if defined USE_HAL_DRIVER
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE(); 
  #else  
  LL_FLASH_EnablePrefetch();
  #endif

	// already enable by default
	
  /* Set Interrupt Group Priority */
	/*
	 * all 4 bit for pre-emption
	 * 0(high priority) ->15(low priority)
	 */
  NVIC_SetPriorityGrouping(0x00000003U);

  /* System interrupt init*/

  NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
}


void UTILS_SystemClockInit(void)
{

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

   if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
     UTILS_Error(__FILE__ , __LINE__); 
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_HSE_EnableCSS();
	
	#if defined STM32F107xC|STM32F105xC
	
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_PLL2_DIV_5, LL_RCC_PLL_MUL_9);

  LL_RCC_PLL_ConfigDomain_PLL2(LL_RCC_HSE_PREDIV2_DIV_5, LL_RCC_PLL2_MUL_8);

  LL_RCC_PLL2_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL2_IsReady() != 1)
  {
    
  }
	#else
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1,LL_RCC_PLL_MUL_9);
#endif
  LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);

  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  } 
	
		// SystemCoreClock Check	
  LL_RCC_ClocksTypeDef rcc_clocks;
	LL_RCC_GetSystemClocksFreq(&rcc_clocks);
  if (SystemCoreClock != rcc_clocks.HCLK_Frequency)  {        
		 UTILS_Error(__FILE__ , __LINE__);// Error Handling
  }
	
	
}


void UTILS_McoInit(void)
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_8,LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinOutputType(GPIOA,LL_GPIO_PIN_8,LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOA,LL_GPIO_PIN_8,LL_GPIO_SPEED_FREQ_HIGH);
	LL_RCC_ConfigMCO(RCC_CFGR_MCOSEL_HSE);
}


