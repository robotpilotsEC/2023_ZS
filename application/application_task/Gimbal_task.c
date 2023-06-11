#include "Gimbal_task.h"
#include "cmsis_os.h"
#include "main.h"

void Start_gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Start_gimbal_Task */
	Gimbal_angle_struct_Init(&Gimbal_angle);
	Gimbal_motor_mac_init();
	Gimbal_motor_follow_init();	
	
  /* Infinite loop */
  for(;;)
 {
	
	if(GIMBAL_MD)
	{	
		imu_kp_change();
		if(RC_ONLINE)
		{			
			vision_enter_RC();	
			Change_Target();
			Gimbal_Auto_Ctrl();
			Gimbal_Pos_Init();
			
			if(MODE_MAC_OR_TOP == MODE_TOP && ((vision_enter == OFF && vision_enter_rc == ON) || (vision_enter == OFF && vision_enter_rc == OFF)))	
			{			
				if(Offline_flag == 0)
				{												                        
          Gimbal_follow_motor();						 
				}
			}
			else if(MODE_MAC_OR_TOP == MODE_MAC)
			{
				Gimbal_mac_motor();
			}
			else if(vision_enter == ON )
			{	
	   
				if(Offline_flag == 0)
				{	
				  Gimbal_motor_vision_init ();
					Gimbal_vision();
				}
			} 
			CAN_cmd_gimbal(gimbal_yaw_motor_current, gimbal_pitch_motor_current, 0, 0);	
		}
		else if(RC_OFFLINE)
		{
			CAN_cmd_gimbal(0,0,0,0);//底盘电机停转卸力		
		}
		
	}
    osDelay(2);
}
  /* USER CODE END Start_gimbal_Task */
}
