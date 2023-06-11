/********************************************************************************
  * @file			gimbal_motor.c
	* @author      RobotPilots@2022
  * @version		V1.0.0
  * @date			
  * @brief   		
  *******************************************************************************/
#include "gimbal_motor.h"

char vision_enter = 0;
char vision_enter_rc = 0;
int16_t follow_to_mac = 0;
int16_t mac_to_follow = 0;
float gimbal_yaw_motor_current = 0;
float gimbal_pitch_motor_current = 0;
Gimbal_angle_t  Gimbal_angle;


/*
	�Ӿ�ƫ��
*/
//�������
#define DF_YAW_ADD 0//20
#define DF_PIT_ADD 0//-30

//���鲿��
#define AIM_YAW_ADD 0
#define AIM_PIT_ADD 0



//���ģʽ�µ�ƫ���������
float Yaw_Target_Add;
float Pit_Target_Add;
//float VISION_KP;
void imu_kp_change(void)
{
 /*-�л�������Kp-*/
	    if(HAL_GetTick() > 4000)
			{
				Kp = IMU_NORM_KP;	
			}
	    else 									
			{				
				Kp = IMU_INIT_KP;	
			}
}
/*
	��̨�Ƕȳ�ʼ��
*/
void Gimbal_angle_struct_Init(Gimbal_angle_t* gimbal_angle)
{
	 gimbal_angle->angle_target_follow_yaw = 0;         //yaw��̨Ŀ��ֵ������ģʽ��
	 gimbal_angle->angle_lessen_follow_yaw = 0;         //pitch��̨ң��������ֵ��С������ģʽ��
	 gimbal_angle->angle_target_follow_pitch = 0;       //yaw��̨Ŀ��ֵ������ģʽ��
	 gimbal_angle->angle_lessen_follow_pitch = 0;       //pitch��̨ң��������ֵ��С������ģʽ��
	 gimbal_angle->angle_target_gimbal_pitch_mac = PITCH_GIMBAL_MAC_MID_VALUE;//pitch��̨ң��������ֵ��С(��еģʽ)
	 gimbal_angle->angle_lessen_gimbal_pitch_mac = 0;   //pitch��̨ң��������ֵ��С����еģʽ��
	 gimbal_angle->gimbal_pitch_follow_mac_angle = 0;   //pitch��̨�ڸ���ģʽʱ�ĵ����е�Ƕ�
	 gimbal_angle->angle_mac_follow = CHASSIS_MAC_FRONT_MID_VALUE; //���̸���YAW�Ļ�е�Ƕ�
	 gimbal_angle->pitch_mac = 0;	//��еģʽ�µ�gyro pitchֵ
}

/*
	��ң����̨λ�ó�ʼ������еģʽͷ��λ+pitch��λ��
*/
void Gimbal_Pos_Init()

{
	if(Offline_flag == 1)
	{
		Gimbal_mac_motor();
	
		if(abs(motor_chassis_gimbal[Gimbal_yaw].angle - Gimbal_angle.angle_mac_follow) <= 50 && motor_chassis_gimbal[Gimbal_yaw].angle - Gimbal_angle.angle_mac_follow >= -50)
		{
			if(abs(motor_chassis_gimbal[Gimbal_pitch].angle - Gimbal_angle.angle_target_gimbal_pitch_mac)<=50)
			{
				MODE_MAC_OR_TOP = MODE_TOP;	
				Offline_flag = 0;
			}
		}
	}
}
/*
	����ģʽ
*/	
void Gimbal_follow_motor(void)
{  	
	//����	
	  follow_to_mac = 1;
	//ʵʱ��������̨��е�Ƕ�
	  Gimbal_angle.gimbal_pitch_follow_mac_angle = motor_chassis_gimbal[Gimbal_pitch].angle;
	
	//��е�л��ظ��棬����̨�Ƕ�Ҫ�ͻ�еһ��
	 if(mac_to_follow == 1)
	 {
	  Gimbal_angle.angle_target_follow_pitch = Gimbal_angle.pitch_mac;
		mac_to_follow = 0;
	 }
	 
	 if(MODE == MODE_RC || (vision_enter == OFF && vision_enter_rc == ON ))
	 {
		Gimbal_angle.angle_lessen_follow_yaw = (CH0_VALUE) * ANGLE_LIM_GYRO_YAW_RATE;
		Gimbal_angle.angle_target_follow_yaw += Gimbal_angle.angle_lessen_follow_yaw;
		Gimbal_angle.angle_lessen_follow_pitch = (CH1_VALUE) * ANGLE_LIM_GYRO_PITCH_RATE;
		Gimbal_angle.angle_target_follow_pitch += Gimbal_angle.angle_lessen_follow_pitch;
		shoot_flag.cover_flag = OFF;  //ң����ģʽ�²��ص���ֱ������������ģʽĬ�ϵ��ֹر�
	 }
	 else if(MODE == MODE_KEY)
	{
		Gimbal_angle.angle_lessen_follow_yaw = (CH2_VALUE_K) * ANGLE_LIM_GYRO_YAW_RATE;
		Gimbal_angle.angle_target_follow_yaw += Gimbal_angle.angle_lessen_follow_yaw;
		Gimbal_angle.angle_lessen_follow_pitch = (CH3_VALUE_K) * ANGLE_LIM_GYRO_PITCH_RATE;
		Gimbal_angle.angle_target_follow_pitch += Gimbal_angle.angle_lessen_follow_pitch;
	 }
	 if(shoot_flag.cover_flag == ON)//����һ��������̨�Զ�ˮƽ������ң��������
	 {
		 Gimbal_angle.angle_target_follow_pitch = 0;
	 }
	  
	//����̨��̬�޷��������ڴ���������λ��ʱ��
	  if(Gimbal_angle.angle_target_follow_pitch >= (PITCH_GIMBAL_GYRO_VALUE_MAX + pitch - (motor_chassis_gimbal[Gimbal_pitch].angle - PITCH_GIMBAL_MAC_MID_VALUE)/22.755f) )
		{
			Gimbal_angle.angle_target_follow_pitch = PITCH_GIMBAL_GYRO_VALUE_MAX +  pitch - (motor_chassis_gimbal[Gimbal_pitch].angle - PITCH_GIMBAL_MAC_MID_VALUE)/22.755f;
		}
		if(Gimbal_angle.angle_target_follow_pitch <= (PITCH_GIMBAL_GYRO_VALUE_MIN + pitch - (motor_chassis_gimbal[Gimbal_pitch].angle - PITCH_GIMBAL_MAC_MID_VALUE)/22.755f))
		{
			Gimbal_angle.angle_target_follow_pitch = PITCH_GIMBAL_GYRO_VALUE_MIN + pitch - (motor_chassis_gimbal[Gimbal_pitch].angle - PITCH_GIMBAL_MAC_MID_VALUE)/22.755f;
		}
	

  //λ�û���measureΪ�����ǵ�yawֵ��-180��180��	
  //�ٶȻ���measureΪ�����ǵĽ��ٶ�ֵ
		cascade_pid_ctrl_gimbal_yaw(&pid_pos[Gimbal_follow_motor_yaw_pos], &pid_spd[Gimbal_follow_motor_yaw_spd], yaw , ggz, Gimbal_angle.angle_target_follow_yaw);	
		cascade_pid_ctrl_gimbal_pitch(&pid_pos[Gimbal_follow_motor_pitch_pos], &pid_spd[Gimbal_follow_motor_pitch_spd], pitch, ggy, Gimbal_angle.angle_target_follow_pitch);
		
		gimbal_yaw_motor_current	= pid_spd[Gimbal_yaw].pos_out;
		gimbal_pitch_motor_current = pid_spd[Gimbal_pitch].pos_out;
		
		
											 
}

/*
	��еģʽ
*/	
void Gimbal_mac_motor()
{	
		//��е
	  mac_to_follow = 1;
	//ʵʱ��������̨pitch�Ƕ�
	  Gimbal_angle.pitch_mac = pitch;
	
		if(follow_to_mac == 1)
		{
			Gimbal_angle.angle_target_gimbal_pitch_mac = Gimbal_angle.gimbal_pitch_follow_mac_angle;
			if(Offline_flag == 1)
			{
				Gimbal_angle.angle_target_gimbal_pitch_mac = PITCH_GIMBAL_MAC_MID_VALUE;//��ʼ��ˮƽpitch
			}
			follow_to_mac = 0;
		}
	  	
	  Gimbal_angle.angle_mac_follow = angle_target_gyro;		
	
	  Gimbal_angle.angle_target_follow_yaw = yaw;//���¸���ģʽ������yaw
	  
			
		 if(MODE == MODE_RC)
		 {											
			 Gimbal_angle.angle_lessen_gimbal_pitch_mac = (CH1_VALUE) * ANGLE_LIM_MAC_PITCH_RATE*3;
			 Gimbal_angle.angle_target_gimbal_pitch_mac = Gimbal_angle.angle_target_gimbal_pitch_mac + Gimbal_angle.angle_lessen_gimbal_pitch_mac;
		 }
			else if(MODE == MODE_KEY)
			{
				Gimbal_angle.angle_lessen_gimbal_pitch_mac = (CH3_VALUE_K) * ANGLE_LIM_MAC_PITCH_RATE*3;
				Gimbal_angle.angle_target_gimbal_pitch_mac = Gimbal_angle.angle_target_gimbal_pitch_mac +Gimbal_angle. angle_lessen_gimbal_pitch_mac;
			}
			if(shoot_flag.cover_flag == 1)//����һ��������̨�Զ�ˮƽ������ң��������
		 {
			 Gimbal_angle.angle_target_gimbal_pitch_mac = PITCH_GIMBAL_MAC_MID_VALUE;
		 }
	  
	
	  if(Gimbal_angle.angle_target_gimbal_pitch_mac <= PITCH_GIMBAL_MAC_VALUE_MIN )
		{
			Gimbal_angle.angle_target_gimbal_pitch_mac = PITCH_GIMBAL_MAC_VALUE_MIN;
		}
		if(Gimbal_angle.angle_target_gimbal_pitch_mac >= PITCH_GIMBAL_MAC_VALUE_MAX )
		{
			Gimbal_angle.angle_target_gimbal_pitch_mac = PITCH_GIMBAL_MAC_VALUE_MAX;
		}
		
		//λ�û���measureΪ����̨����Ļ�е�Ƕ�ֵ����Լ��2000
		//�ٶȻ���measureΪ�����ǵĽ��ٶ�ֵ
	
		cascade_pid_ctrl_mac_yaw(&pid_pos[Gimbal_mac_motor_yaw_pos], 
		                     &pid_spd[Gimbal_mac_motor_yaw_spd], 
		                     motor_chassis_gimbal[Gimbal_yaw].angle, 
		                     ggz, 
		                     Gimbal_angle.angle_mac_follow);
		
		cascade_pid_ctrl(&pid_pos[Gimbal_mac_motor_pitch_pos], 
										 &pid_spd[Gimbal_mac_motor_pitch_spd], 
										 motor_chassis_gimbal[Gimbal_pitch].angle, 
										 ggy, 
										 Gimbal_angle.angle_target_gimbal_pitch_mac);
		
		gimbal_yaw_motor_current	= pid_spd[Gimbal_mac_motor_yaw_spd].pos_out;
		gimbal_pitch_motor_current = pid_spd[Gimbal_mac_motor_pitch_spd].pos_out;
	
}



float angle_target_vision_gimbal_yaw = 0;
float angle_target_vision_gimbal_pitch = 0;
/*
	�Ӿ�ģʽ
*/
void vision_enter_RC()
{
	/*�Ӿ�������ԣ�ң��������*/
	  if(SW1_UP && SW2_DOWN && TW_VALUE > 500)
		{
			vision_enter_rc = ON;		
			Vision_Mode = CMD_AIM_AUTO;
			
		}
		if(SW1_UP && SW2_DOWN && TW_VALUE < -500)
		{
			vision_enter_rc = OFF;	
      Vision_Mode = CMD_AIM_AUTO;			
		}
		
		/*�Ӿ��������������ԣ�ң��������*/
		if(SW1_UP && SW2_UP && TW_VALUE > 500)
		{
			vision_enter_rc = ON;		
			Vision_Mode = CMD_AIM_BIG_BUFF;
		}
		if(SW1_UP && SW2_UP && TW_VALUE <- 500)
		{
			vision_enter_rc = OFF;		
			Vision_Mode = CMD_AIM_OFF;
		}
		
}

/*
		�л�Ŀ��
*/
extern uint16_t change_target_flag;
extern uint16_t change_target;
void Change_Target()
{
	if(SW1_UP && SW2_UP && TW_VALUE > 500)
	{
		 if(change_target_flag == 0)
		{
			change_target += 1;	
			change_target_flag = 1;				
		}	
	}
	
	if(SW1_UP && SW2_UP && TW_VALUE == 0)
	{
		 change_target_flag = 0;
	}
	
}
void Gimbal_vision()
{
		
			angle_target_vision_gimbal_pitch = (vision_sensor.info->RxPacket.RxData.pitch_angle / 22.7555f) - 180.0f;
			angle_target_vision_gimbal_yaw = (vision_sensor.info->RxPacket.RxData.yaw_angle / 22.7555f) - 180.0f;
			
			cascade_pid_ctrl_gimbal_yaw(&pid_pos[Gimbal_follow_motor_yaw_pos], 
																	 &pid_spd[Gimbal_follow_motor_yaw_spd], 
																	 yaw, 
																	 ggz, 
																	 angle_target_vision_gimbal_yaw);	
		
			cascade_pid_ctrl_gimbal_pitch(&pid_pos[Gimbal_follow_motor_pitch_pos], 
																		&pid_spd[Gimbal_follow_motor_pitch_spd], 
																		pitch, 
																		ggy, 
																		angle_target_vision_gimbal_pitch);
				
			gimbal_yaw_motor_current	= pid_spd[Gimbal_follow_motor_yaw_spd].pos_out;
			gimbal_pitch_motor_current = pid_spd[Gimbal_follow_motor_pitch_spd].pos_out;
		
	
} 

void Gimbal_Auto_Ctrl(void)
{
	Vision_Cmd_Id_t Cmd = vision_sensor.info->TxPacket.FrameHeader.cmd_id;
	Vision_Rx_Packet_t *Pack = &vision_sensor.info->RxPacket;
	
	if(VISION_ONLINE && Cmd && Find_Tar())
	{
			if((Pack->FrameHeader.cmd_id == CMD_AIM_SMALL_BUFF || Pack->FrameHeader.cmd_id == CMD_AIM_BIG_BUFF || Pack->FrameHeader.cmd_id == CMD_AIM_ANDF))
			{
				Yaw_Target_Add = DF_YAW_ADD;
			  Pit_Target_Add = DF_PIT_ADD;	
				vision_enter = ON;	
			}
			if((Pack->FrameHeader.cmd_id == CMD_AIM_AUTO || Pack->FrameHeader.cmd_id == CMD_AIM_ANTOP ) && (MOUSE_RIGH || vision_enter_rc == 1) )
			{
				Yaw_Target_Add = AIM_YAW_ADD;
			  Pit_Target_Add = AIM_PIT_ADD;
				vision_enter = ON;					
			}		
	}
	else
	{
		vision_enter = OFF;		
	}
	if(vision_enter_rc == 0)
	{
		if((Pack->FrameHeader.cmd_id == CMD_AIM_AUTO || Pack->FrameHeader.cmd_id == CMD_AIM_ANTOP) && MOUSE_RIGH == 0)
		{
			vision_enter = OFF;	
		}
	}
	
}

