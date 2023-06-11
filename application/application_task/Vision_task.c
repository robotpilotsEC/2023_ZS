#include "Vision_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "zs_rp_config.h"

void Start_Vision_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Vision_Task */
  /* Infinite loop */
  for(;;)
  {
		if(VISION_MD)
		{
		 Vision_Get();
		 Vision_TX();	
		}
		vision_heart_beat(&vision_sensor);
    osDelay(7);
  }
  /* USER CODE END Start_Vision_Task */
}
