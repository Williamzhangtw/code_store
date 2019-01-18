#if !defined __STREAM_BLINKING_LED_H__
 #define __STREAM_BLINKING_LED_H__
 
 
#include "bsp.h"
 
 
void LedNodeInit(void);
void LedStreamBlinkingBitband(void);
void LedStreamBlinking(void);
void LedNodeUninit(void);

#endif
