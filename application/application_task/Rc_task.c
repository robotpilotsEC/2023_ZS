#include "Rc_task.h"
#include "cmsis_os.h"
#include "main.h"
extern int32_t change_F_B_time;             //键盘模式一键换头计算的时间
void rc_task(void const * argument)
{
  /* USER CODE BEGIN rc_task */
  /* Infinite loop */
	Rc_Init();
	Key_Init();
  for(;;)
  {	

		#if POSITION_N == 1
			rc_sensor_heart_beat(&rc);		  
			RC_Offline_Sleep();	 	
	 
			if(MODE == MODE_KEY)
			{		
				if(fast_180_flag == 0 ||  change_F_B_time >= 400)             //键盘模式一键换头计算的时间)
				{
				  Mouse_X_UPDATA = Mouse_X_Speed();
	        Mouse_Y_UPDATA = Mouse_Y_Speed();
				  Mouse_FS();
				}
					KB_CTRL();		
			}		
			offline_LED_UP();
		
		#endif
		
		
		#if POSITION_N == 0
			offline_LED_DOWN();
		#endif
		
		
    osDelay(1);
  }
  /* USER CODE END rc_task */
}
