#include "Shoot_task.h"
#include "cmsis_os.h"
#include "main.h"

void Start_shoot_task(void const * argument)
{
  /* USER CODE BEGIN Start_shoot_task */
  /* Infinite loop */
   shoot_skip_init(&shoot_skip);
   shoot_flag_init(&shoot_flag ,&change_barr);
	 shoot_left_friction_wheel_init();
	 shoot_right_friction_wheel_init();
	 shoot_flick_wheel_jam_handle_init();				 
	 shoot_flick_wheel_follow_init(); //连发
	 shoot_flick_wheel_mac_init();    //单发
	 shoot_flick_wheel_stop_init();   //停止射击
	 shoot_change_barrel_init();
  for(;;)
  {  
		if(SHOOT_MD)
		{
		
		 if(RC_ONLINE)
		 {		
			 Fri_Stop_Judge();
			 if(MODE == MODE_KEY)
			 {
				 R_Fast_Shoot();
			 }			
			 Auto_Shoot_Ctrl();
			 shoot_motor_running_fire();
			 shoot_motor_single_fire();
			 
			 Shoot_Speed_Test();
			 
			 
			 
			 #if CAR_MODE == 1 && BARR == 2
				 change_clock(&change_barr); //锁定枪管
				 change_barrel_motor(&change_barr); //枪管切换
			   
			 #endif
			 
			 shoot_stop();
			 
			 //PWM函数
			 if(MODE == MODE_RC && MODE_MAC_OR_TOP == MODE_MAC)
			 {
				 COVER_WEAK();
			   Box_Cover_Control();			
			 }
			 
			  if(MODE == MODE_RC && MODE_MAC_OR_TOP == MODE_TOP)
			 {
				 COVER_WEAK();
				 Cover_Close();
			 }
			 
			 if(MODE == MODE_KEY )
			 {
				 COVER_WEAK();
			   Box_Cover_Control();	
			 }
			 
			  #if CAR_MODE == 1 && BARR == 2
			   CAN_cmd_shoot( pid_shoot_left_friction_wheel_spd.pos_out,
											pid_shoot_right_friction_wheel_spd.pos_out,
											flick_motor_current,
											change_barr.change_barral_motor_current);
			 #endif
			 
			
			 #if CAR_MODE == 2 || (BARR == 1 && CAR_MODE == 1 )
			   CAN_cmd_shoot( pid_shoot_left_friction_wheel_spd.pos_out,
											  pid_shoot_right_friction_wheel_spd.pos_out,
											  flick_motor_current,
										  	0);
			 #endif
		 }	
		 else if(RC_OFFLINE)
		{
			CAN_cmd_shoot(0,0,0,0);//底盘电机停转卸力		
		}
		
	 }	
    osDelay(3);
  }
  /* USER CODE END Start_shoot_task */
}
