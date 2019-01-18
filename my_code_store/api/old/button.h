
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUTTON_H
#define __BUTTON_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "bsp.h"
#include "rename_api.h"
/** @defgroup STM3210C_EVAL_Exported_Types STM3210C EVAL Exported Types
  * @{
  */

/**
 * @brief BUTTON Types Definition
 */
typedef enum 
{
	BUTTON1 =0,

	BUTTON2 = 1,

	BUTTON3 = 2,

} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;



void BSP_Button_IT_Falling_Init(Button_TypeDef Button);
void BSP_Button_IT_Rising_Init(Button_TypeDef Button);
void BSP_Button_DeInit(Button_TypeDef Button);
int32_t Buttons_Deinitialize (void);
uint32_t BSP_BUTTON_GetState(Button_TypeDef Button);



#ifdef __cplusplus
}
#endif
  
#endif /* __STM3210C_EVAL_H */

/**
  * @}
  */
  
/**
  * @}
  */
  
/**
  * @}
  */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
