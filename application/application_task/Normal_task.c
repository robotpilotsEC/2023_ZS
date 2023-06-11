#include "Normal_task.h"
#include "cmsis_os.h"
#include "main.h"
void Normal_tasks(void const * argument)
{
  /* USER CODE BEGIN StartTask10 */
  /* Infinite loop */
  for(;;)
  {	
		judge_sensor.heart_beat(&judge_sensor);
		CAP_RP_2023.heart_beat();
		#if POSITION_N == 0		
			slave_heart_beat();
			Down_Send(&judge_sensor);
		
		#endif
		
		#if POSITION_N == 1		
			Up_Send();
		 
		#endif
		
    osDelay(5);
		
  }
  /* USER CODE END StartTask10 */
}
