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
	视觉偏置
*/
//打符部分
#define DF_YAW_ADD 0//20
#define DF_PIT_ADD 0//-30

//自瞄部分
#define AIM_YAW_ADD 0
#define AIM_PIT_ADD 0



//打符模式下的偏置与积分项
float Yaw_Target_Add;
float Pit_Target_Add;
//float VISION_KP;
void imu_kp_change(void)
{
 /*-切换陀螺仪Kp-*/
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
	云台角度初始化
*/
void Gimbal_angle_struct_Init(Gimbal_angle_t* gimbal_angle)
{
	 gimbal_angle->angle_target_follow_yaw = 0;         //yaw云台目标值（跟随模式）
	 gimbal_angle->angle_lessen_follow_yaw = 0;         //pitch云台遥控器给的值缩小（跟随模式）
	 gimbal_angle->angle_target_follow_pitch = 0;       //yaw云台目标值（跟随模式）
	 gimbal_angle->angle_lessen_follow_pitch = 0;       //pitch云台遥控器给的值缩小（跟随模式）
	 gimbal_angle->angle_target_gimbal_pitch_mac = PITCH_GIMBAL_MAC_MID_VALUE;//pitch云台遥控器给的值缩小(机械模式)
	 gimbal_angle->angle_lessen_gimbal_pitch_mac = 0;   //pitch云台遥控器给的值缩小（机械模式）
	 gimbal_angle->gimbal_pitch_follow_mac_angle = 0;   //pitch云台在跟随模式时的电机机械角度
	 gimbal_angle->angle_mac_follow = CHASSIS_MAC_FRONT_MID_VALUE; //底盘跟随YAW的机械角度
	 gimbal_angle->pitch_mac = 0;	//机械模式下的gyro pitch值
}

/*
	开遥控云台位置初始化（机械模式头归位+pitch归位）
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
	跟随模式
*/	
void Gimbal_follow_motor(void)
{  	
	//跟随	
	  follow_to_mac = 1;
	//实时更新上云台机械角度
	  Gimbal_angle.gimbal_pitch_follow_mac_angle = motor_chassis_gimbal[Gimbal_pitch].angle;
	
	//机械切换回跟随，上云台角度要和机械一致
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
		shoot_flag.cover_flag = OFF;  //遥控器模式下不关弹仓直接切入陀螺仪模式默认弹仓关闭
	 }
	 else if(MODE == MODE_KEY)
	{
		Gimbal_angle.angle_lessen_follow_yaw = (CH2_VALUE_K) * ANGLE_LIM_GYRO_YAW_RATE;
		Gimbal_angle.angle_target_follow_yaw += Gimbal_angle.angle_lessen_follow_yaw;
		Gimbal_angle.angle_lessen_follow_pitch = (CH3_VALUE_K) * ANGLE_LIM_GYRO_PITCH_RATE;
		Gimbal_angle.angle_target_follow_pitch += Gimbal_angle.angle_lessen_follow_pitch;
	 }
	 if(shoot_flag.cover_flag == ON)//弹仓一开，上云台自动水平，脱离遥控器控制
	 {
		 Gimbal_angle.angle_target_follow_pitch = 0;
	 }
	  
	//上云台动态限幅（适用于处于上下坡位置时）
	  if(Gimbal_angle.angle_target_follow_pitch >= (PITCH_GIMBAL_GYRO_VALUE_MAX + pitch - (motor_chassis_gimbal[Gimbal_pitch].angle - PITCH_GIMBAL_MAC_MID_VALUE)/22.755f) )
		{
			Gimbal_angle.angle_target_follow_pitch = PITCH_GIMBAL_GYRO_VALUE_MAX +  pitch - (motor_chassis_gimbal[Gimbal_pitch].angle - PITCH_GIMBAL_MAC_MID_VALUE)/22.755f;
		}
		if(Gimbal_angle.angle_target_follow_pitch <= (PITCH_GIMBAL_GYRO_VALUE_MIN + pitch - (motor_chassis_gimbal[Gimbal_pitch].angle - PITCH_GIMBAL_MAC_MID_VALUE)/22.755f))
		{
			Gimbal_angle.angle_target_follow_pitch = PITCH_GIMBAL_GYRO_VALUE_MIN + pitch - (motor_chassis_gimbal[Gimbal_pitch].angle - PITCH_GIMBAL_MAC_MID_VALUE)/22.755f;
		}
	

  //位置环的measure为陀螺仪的yaw值（-180，180）	
  //速度环的measure为陀螺仪的角速度值
		cascade_pid_ctrl_gimbal_yaw(&pid_pos[Gimbal_follow_motor_yaw_pos], &pid_spd[Gimbal_follow_motor_yaw_spd], yaw , ggz, Gimbal_angle.angle_target_follow_yaw);	
		cascade_pid_ctrl_gimbal_pitch(&pid_pos[Gimbal_follow_motor_pitch_pos], &pid_spd[Gimbal_follow_motor_pitch_spd], pitch, ggy, Gimbal_angle.angle_target_follow_pitch);
		
		gimbal_yaw_motor_current	= pid_spd[Gimbal_yaw].pos_out;
		gimbal_pitch_motor_current = pid_spd[Gimbal_pitch].pos_out;
		
		
											 
}

/*
	机械模式
*/	
void Gimbal_mac_motor()
{	
		//机械
	  mac_to_follow = 1;
	//实时更新上云台pitch角度
	  Gimbal_angle.pitch_mac = pitch;
	
		if(follow_to_mac == 1)
		{
			Gimbal_angle.angle_target_gimbal_pitch_mac = Gimbal_angle.gimbal_pitch_follow_mac_angle;
			if(Offline_flag == 1)
			{
				Gimbal_angle.angle_target_gimbal_pitch_mac = PITCH_GIMBAL_MAC_MID_VALUE;//初始化水平pitch
			}
			follow_to_mac = 0;
		}
	  	
	  Gimbal_angle.angle_mac_follow = angle_target_gyro;		
	
	  Gimbal_angle.angle_target_follow_yaw = yaw;//更新跟随模式陀螺仪yaw
	  
			
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
			if(shoot_flag.cover_flag == 1)//弹仓一开，上云台自动水平，脱离遥控器控制
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
		
		//位置环的measure为下云台电机的机械角度值：大约在2000
		//速度环的measure为陀螺仪的角速度值
	
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
	视觉模式
*/
void vision_enter_RC()
{
	/*视觉自瞄测试：遥控器操作*/
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
		
		/*视觉打符（大符）测试：遥控器操作*/
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
		切换目标
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

