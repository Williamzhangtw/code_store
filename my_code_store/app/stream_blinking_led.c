#include "stream_blinking_led.h"

typedef struct _NODE
{
  uint8_t rank;
  void* handler;
  struct _NODE * next;
}NODE;


NODE *led_head_node = NULL ;

void AddNode(NODE* node,void * handler)
{
  NODE *link = (NODE*) malloc(sizeof(NODE));
  link->handler= handler;
  link->next =led_head_node;
  led_head_node=link;
}


void LedNodeInit(void)
{
  led_head_node = NULL;
#if defined USE_LED1
  AddNode(led_head_node,(void*)&arm_led1);
#endif
  
#if defined USE_LED2
 AddNode(led_head_node,(void*)&arm_led2);
#endif
  #if defined USE_LED3
  AddNode(led_head_node,(void*)&arm_led3);
#endif
  #if defined USE_LED4
  AddNode(led_head_node,(void*)&arm_led4);
#endif
 


}

void LedNodeUninit(void)
{

   free(led_head_node->next->next->next);
   free(led_head_node->next->next);
   free(led_head_node->next);   
   free(led_head_node);
}


void LedStreamBlinking(void)
{
  NODE *current_node = led_head_node;

  while(current_node!=NULL)
  {
    ((ARM_LED*)current_node->handler)->Toggle();
    UTILS_MilliSecondDelay(100);
    current_node = current_node->next;
	}
  

}
 
void LedStreamBlinkingBitband(void)
{
//	volatile uint32_t * bitband_alia;
//	for(uint32_t index=0;index<LED_NUMBER;index++)
//	{
//		bitband_alia = BITBAND_ADDR(led_port[index],led_pin[index]);
//		if (*bitband_alia)
//		{
//			*bitband_alia=0;
//		}
//		else
//		{
//			*bitband_alia=1;
//		}
//		mDelay(LED_BLINK_FAST);
//	}
}

