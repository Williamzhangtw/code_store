#include "diagnose.h"
/* 	error flag  						(TIM7)
*
*/


volatile uint32_t err_flag=0;


void UTILS_DiagnoseInit(void)
{
/* Peripheral clock enable */
LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

LL_TIM_ClearFlag_UPDATE(TIM7);
LL_RCC_ClocksTypeDef rcc_clocks;
LL_RCC_GetSystemClocksFreq(&rcc_clocks);
LL_TIM_SetPrescaler(TIM7,((rcc_clocks.PCLK1_Frequency<<1)/10000-1));
LL_TIM_SetAutoReload(TIM7,4999);
LL_TIM_EnableIT_UPDATE(TIM7);

/* TIM6 interrupt Init */
NVIC_SetPriority(TIM7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
NVIC_EnableIRQ(TIM7_IRQn);

LL_TIM_EnableCounter(TIM7);
}



void UTILS_DiagnoseError(void)
{
	err_flag = 1;
}


void UTILS_DiagnoseHandler(char *file, int line)
{

	//(char *file, int line)
	err_flag = 1;
	//		__BKPT(7);  /* Error: Call debugger or replace with custom error handling */


}


int32_t UTILS_DiagnoseDetect(void)
{
//	if(err_flag )
//	{
//	//		__BKPT(7);  /* Error: Call debugger or replace with custom error handling */
//		while(1)
//		{
//		}
//  }
	return err_flag;
}

void UTILS_DiagnoseReset(void)
{
	err_flag =0;
}

void TIM7_IRQHandler(void)
{
	LL_TIM_ClearFlag_UPDATE(TIM7);
	if(err_flag)
	{
//		arm_led4.On();
	}
}

//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t* file, uint32_t line)
//{ 
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//	printf("Wrong parameters value: file %s on line %d\r\n", file, line);
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */




