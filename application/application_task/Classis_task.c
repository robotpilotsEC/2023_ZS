#include "Classis_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "zs_rp_config.h"

void Start_chassis_task(void const * argument)
{
  /* USER CODE BEGIN Start_chassis_task */
  /* Infinite loop */
	chassis_speed_init(&chassis_speed);
	Chassis_motor_init();
	Chassis_motor_follow_init();	
  for(;;)
  {
		
	 if(CHASSIS_MD)
	 {
		if(RC_ONLINE)
		{
			if(MODE_MAC_OR_TOP == MODE_TOP)
			{
				if(Offline_flag == 0)
				{							  											
          Chassis_speed_set(&chassis_speed);				
          Chassis_motor(&chassis_speed);						
				}
			}
			else if(MODE_MAC_OR_TOP == MODE_MAC)
			{
				  Chassis_speed_set(&chassis_speed);				
			    Chassis_motor(&chassis_speed);
			}
				
		}
		else if(RC_OFFLINE)
		{
			CAN_cmd_chassis(0,0,0,0);//底盘电机停转卸力
			
		}
		
		
	}
    osDelay(2);
  }
  /* USER CODE END Start_chassis_task */
}

void Supercap_task(void const * argument)
{
  /* USER CODE BEGIN Supercap_task */
  /* Infinite loop */
  for(;;)
  {
		#if POSITION_N == 0
			CAP_RP_2023.ctrl();		
		#endif
		osDelay(4);
  }
  /* USER CODE END Supercap_task */
}
