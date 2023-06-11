#include "Judge_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "UI.h"


void Judge_task(void const * argument)
{
  /* USER CODE BEGIN Judge_task */
  /* Infinite loop */
  for(;;)
  {
		Client_task();
		
    osDelay(1);
  }
}
