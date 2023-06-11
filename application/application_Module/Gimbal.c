#include "gimbal.h"


/*
	��̨PID��ʼ��������ģʽ��
*/
void Gimbal_motor_follow_init()
{
		
		//yaw��̨	
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
		//pitch��̨	
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
	��̨PID��ʼ������еģʽ��
*/
void Gimbal_motor_mac_init()
{
		//PID��ʼ��
		//����̨	
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
		//����̨	
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
	��̨PID��ʼ�����Ӿ�ģʽ��
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
		//pitch��̨	
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


