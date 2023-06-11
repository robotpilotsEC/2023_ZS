/*
*  ����  __RP_CONFIG_H����CARR_MODE����
*  
*  
*/

#include "Chassis.h"
#include "arm_math.h"


/*
	���̵��PID��ʼ��
*/
void Chassis_motor_init()
{
			
	PID_struct_init(&pid_spd[Left_front_spd],
									PID_CHASSIS_MAXOUTPUT,
									PID_CHASSIS_INTEGRALIMIT,
									PID_CHASSIS_KP,
									PID_CHASSIS_KI,
									PID_CHASSIS_KD);  
	
	PID_struct_init(&pid_spd[Right_front_spd],
									PID_CHASSIS_MAXOUTPUT,
									PID_CHASSIS_INTEGRALIMIT,
									PID_CHASSIS_KP,
									PID_CHASSIS_KI,
									PID_CHASSIS_KD);  	
	
	PID_struct_init(&pid_spd[Left_back_spd],
									PID_CHASSIS_MAXOUTPUT,
									PID_CHASSIS_INTEGRALIMIT,
									PID_CHASSIS_KP,
									PID_CHASSIS_KI,
									PID_CHASSIS_KD);  	
	
	PID_struct_init(&pid_spd[Right_back_spd],
									PID_CHASSIS_MAXOUTPUT,
									PID_CHASSIS_INTEGRALIMIT,
									PID_CHASSIS_KP,
									PID_CHASSIS_KI,
									PID_CHASSIS_KD);  

		
}



/*
	���̸������PID��ʼ��
*/
void Chassis_motor_follow_init()
{
		
	PID_struct_init(&pid_spd_follow,
									PID_CHASSIS_FOLLOW_MAXOUTPUT,
									PID_CHASSIS_FOLLOW_INTEGRALIMIT,
									PID_CHASSIS_FOLLOW_KP,
									PID_CHASSIS_FOLLOW_KI,
									PID_CHASSIS_FOLLOW_KD);  
}


/*ֹͣ���ƽӿ�*/
void Chassis_Motor_Stop(void)
{
	
	rc.info->ch0 = 0;
	rc.info->ch1 = 0;
	rc.info->ch2 = 0;
	rc.info->ch3 = 0;
	rc.info->s1 = 4;
	rc.info->s2 = 4;
}
