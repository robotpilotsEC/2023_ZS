#include "gimbal.h"


/*
	云台PID初始化（跟随模式）
*/
void Gimbal_motor_follow_init()
{
		
		//yaw云台	
		PID_struct_init(&pid_spd[Gimbal_follow_motor_yaw_spd],
										SPD_PID_GIMBAL_YAW_GYRO_MAXOUTPUT,
										SPD_PID_GIMBAL_YAW_GYRO_INTEGRALIMIT,
										SPD_PID_GIMBAL_YAW_GYRO_KP,
										SPD_PID_GIMBAL_YAW_GYRO_KI,
										SPD_PID_GIMBAL_YAW_GYRO_KD);
	
		PID_struct_init(&pid_pos[Gimbal_follow_motor_yaw_pos],
										POS_PID_GIMBAL_YAW_GYRO_MAXOUTPUT,
										POS_PID_GIMBAL_YAW_GYRO_INTEGRALIMIT,
								  	POS_PID_GIMBAL_YAW_GYRO_KP,
										POS_PID_GIMBAL_YAW_GYRO_KI,
										POS_PID_GIMBAL_YAW_GYRO_KD);		         
		//pitch云台	
		PID_struct_init(&pid_spd[Gimbal_follow_motor_pitch_spd],
										SPD_PID_GIMBAL_PITCH_GYRO_MAXOUTPUT,
										SPD_PID_GIMBAL_PITCH_GYRO_INTEGRALIMIT,
								  	SPD_PID_GIMBAL_PITCH_GYRO_KP,
										SPD_PID_GIMBAL_PITCH_GYRO_KI,
										SPD_PID_GIMBAL_PITCH_GYRO_KD);
		PID_struct_init(&pid_pos[Gimbal_follow_motor_pitch_pos],
										POS_PID_GIMBAL_PITCH_GYRO_MAXOUTPUT,
										POS_PID_GIMBAL_PITCH_GYRO_INTEGRALIMIT,
										POS_PID_GIMBAL_PITCH_GYRO_KP,
										POS_PID_GIMBAL_PITCH_GYRO_KI,
										POS_PID_GIMBAL_PITCH_GYRO_KD);	
 								
}

/*
	云台PID初始化（机械模式）
*/
void Gimbal_motor_mac_init()
{
		//PID初始化
		//下云台	
		PID_struct_init(&pid_spd[Gimbal_mac_motor_yaw_spd],
										SPD_PID_GIMBAL_YAW_MAC_MAXOUTPUT,
										SPD_PID_GIMBAL_YAW_MAC_INTEGRALIMIT,
										SPD_PID_GIMBAL_YAW_MAC_KP,
										SPD_PID_GIMBAL_YAW_MAC_KI,
										SPD_PID_GIMBAL_YAW_MAC_KD);
	
		PID_struct_init(&pid_pos[Gimbal_mac_motor_yaw_pos],
										POS_PID_GIMBAL_YAW_MAC_MAXOUTPUT,
										POS_PID_GIMBAL_YAW_MAC_INTEGRALIMIT,
										POS_PID_GIMBAL_YAW_MAC_KP,
										POS_PID_GIMBAL_YAW_MAC_KI,
										POS_PID_GIMBAL_YAW_MAC_KD);		         	
		//上云台	
		PID_struct_init(&pid_spd[Gimbal_mac_motor_pitch_spd],
										POS_PID_GIMBAL_PITCH_GYRO_MAXOUTPUT,
										SPD_PID_GIMBAL_PITCH_MAC_INTEGRALIMIT,
										SPD_PID_GIMBAL_PITCH_MAC_KP,
									  SPD_PID_GIMBAL_PITCH_MAC_KI,
										SPD_PID_GIMBAL_PITCH_MAC_KD);
		PID_struct_init(&pid_pos[Gimbal_mac_motor_pitch_pos],
										POS_PID_GIMBAL_PITCH_MAC_MAXOUTPUT,
										POS_PID_GIMBAL_PITCH_MAC_INTEGRALIMIT,
										POS_PID_GIMBAL_PITCH_MAC_KP,
										POS_PID_GIMBAL_PITCH_MAC_KI,
										POS_PID_GIMBAL_PITCH_MAC_KD);		   
}

/*
	云台PID初始化（视觉模式）
*/
void Gimbal_motor_vision_init()
{	

		PID_struct_init(&pid_spd[Gimbal_follow_motor_yaw_spd],
										SPD_PID_GIMBAL_YAW_GYRO_MAXOUTPUT,
										SPD_PID_GIMBAL_YAW_GYRO_INTEGRALIMIT,
										SPD_PID_GIMBAL_YAW_GYRO_KP,
										SPD_PID_GIMBAL_YAW_GYRO_KI,
										SPD_PID_GIMBAL_YAW_GYRO_KD);
	
		PID_struct_init(&pid_pos[Gimbal_follow_motor_yaw_pos],
										POS_PID_GIMBAL_YAW_GYRO_MAXOUTPUT,
										POS_PID_GIMBAL_YAW_GYRO_INTEGRALIMIT,
								  	POS_PID_GIMBAL_YAW_GYRO_KP,
										POS_PID_GIMBAL_YAW_GYRO_KI,
										POS_PID_GIMBAL_YAW_GYRO_KD);		         
		//pitch云台	
		PID_struct_init(&pid_spd[Gimbal_follow_motor_pitch_spd],
										SPD_PID_GIMBAL_PITCH_GYRO_MAXOUTPUT,
										SPD_PID_GIMBAL_PITCH_GYRO_INTEGRALIMIT,
								  	SPD_PID_GIMBAL_PITCH_GYRO_KP,
										SPD_PID_GIMBAL_PITCH_GYRO_KI,
										SPD_PID_GIMBAL_PITCH_GYRO_KD);
		PID_struct_init(&pid_pos[Gimbal_follow_motor_pitch_pos],
										POS_PID_GIMBAL_PITCH_GYRO_MAXOUTPUT,
										POS_PID_GIMBAL_PITCH_GYRO_INTEGRALIMIT,
										POS_PID_GIMBAL_PITCH_GYRO_KP,
										POS_PID_GIMBAL_PITCH_GYRO_KI,
										POS_PID_GIMBAL_PITCH_GYRO_KD);	
}


