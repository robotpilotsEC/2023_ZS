#include "Shoot.h"

/*
	��Ħ����PID��ʼ��
*/	
void shoot_left_friction_wheel_init()
{		
	PID_struct_init(&pid_shoot_left_friction_wheel_spd,
	SPD_PID_SHOOT_FRICTION_MAXOUTPUT,
	SPD_PID_SHOOT_FRICTION_INTEGRALIMIT,
	SPD_PID_SHOOT_FRICTION_KP,
	SPD_PID_SHOOT_FRICTION_KI,
	SPD_PID_SHOOT_FRICTION_KD);  			
}

/*
	��Ħ����PID��ʼ��
*/	
void shoot_right_friction_wheel_init()
{		
	PID_struct_init(&pid_shoot_right_friction_wheel_spd,
	SPD_PID_SHOOT_FRICTION_MAXOUTPUT,
	SPD_PID_SHOOT_FRICTION_INTEGRALIMIT,
	SPD_PID_SHOOT_FRICTION_KP,
	SPD_PID_SHOOT_FRICTION_KI,
	SPD_PID_SHOOT_FRICTION_KD); 	
}

/*
	����������ģʽPID��ʼ��
*/	
void shoot_flick_wheel_follow_init()
{			
		PID_struct_init(&pid_shoot_flick_wheel_follow_spd,10000,5000,10.0f,0.5,0);  		
}

/*
	�����ֵ���ģʽPID��ʼ��
*/	
void shoot_flick_wheel_mac_init()
{			
	    PID_struct_init(&pid_shoot_flick_wheel_mac_spd,10000,2000,12.0f,0.8f,0);  //17 1.2 
			PID_struct_init(&pid_shoot_flick_wheel_mac_pos,10000,2000,0.244f,0,0);  	//	0.244
}
/*
	��ǹ��PID��ʼ��
*/	
void shoot_change_barrel_init(void)
{			
	    PID_struct_init(&pid_shoot_change_barrel_spd,8000,1000,5.0f,0.1f,0);  
			PID_struct_init(&pid_shoot_change_barrel_pos,8000,1000,0.1f,0,0);  		
}

/*
	������ֹͣ���ģʽPID��ʼ��
*/	
void shoot_flick_wheel_stop_init()
{			
	    PID_struct_init(&pid_shoot_flick_wheel_stop_spd,8000,1000,4.0f,0.1f,0);  
			PID_struct_init(&pid_shoot_flick_wheel_stop_pos,3000,1000,0.1f,0,0);  		
}
/*
	�����ֿ�������ģʽPID��ʼ��
*/	
void shoot_flick_wheel_jam_handle_init()
{			
	    PID_struct_init(&pid_shoot_flick_wheel_jam_spd,8000,1000,4.0f,0.1f,0);  
			PID_struct_init(&pid_shoot_flick_wheel_jam_pos,8000,1000,0.3f,0,0);  		
}

