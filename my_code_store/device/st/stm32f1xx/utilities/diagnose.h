#if !defined CODE_STORE_DEVICE_ST_STM32F1XX_UTILS_DIAGNOSE_H_
	#define CODE_STORE_DEVICE_ST_STM32F1XX_UTILS_DIAGNOSE_H_
	

#include "../hal/hal.h"


/* Race condiction protection */
 static inline void UTILS_EnterCriticalSection(void){
  	__asm__(
  	 "CPSID i\n\t"
  //	 "bx lr\n\t"
  	 );
  }

  //static __attribute__((always_inline)) void Exit_Critical_Section(void){

 static inline void UTILS_ExitCriticalSection(void){
  		__asm__(
  	 "CPSIE i\n\t"
  //	 "bx lr\n\t"
  	 );
  }

static inline void UTILS_Error(char *file, int line)
{
	printf("error: file %s on line %d\r\n",file , line);
  #if defined DEBUG
			__BKPT(7);  /* Error: Call debugger or replace with custom error handling */
  #endif
		while(1)
		{
		}
  
}
// Error(__FILE__ , __LINE__)
 

/*  error Detect  								(TIM7)*/
 extern volatile uint32_t err_flag;
  
	



void UTILS_DiagnoseInit(void);


void UTILS_DiagnoseHandler(char *file, int line);


 
int32_t UTILS_DiagnoseDetect(void);
void UTILS_DiagnoseReset(void);
void UTILS_DiagnoseError(void);



	
	
#endif /* CODE_STORE_DEVICE_ST_STM32F1XX_UTILS_DIAGNOSE_H_ */
