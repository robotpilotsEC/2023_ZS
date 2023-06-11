/********************************************************************************
  * @file			classis_motor.c
	* @author      RobotPilots@2022
  * @version		V1.0.0
  * @date			
  * @brief   		
  *******************************************************************************/
#include "classis_motor.h"

int16_t angle_target_gyro = CHASSIS_MAC_FRONT_MID_VALUE;       //������Ŀ��Ƕ�
int16_t angle_target_gyro_now = CHASSIS_MAC_FRONT_MID_VALUE ;  //������Ŀ��Ƕȣ�ʵʱ���£�
int16_t Small_Top_Mode_Flag;                                   //С����ģʽ������־λ
int16_t Small_Top_Mode;           														 //С����ģʽ����/��/�죩

//ң����0/����ģʽ1
int16_t MODE ;
//������ģʽ/��еģʽ
int16_t MODE_MAC_OR_TOP;
 
int32_t change_F_B_time;             //����ģʽһ����ͷ�����ʱ��
int16_t Top_Count_T;                 //����С����ģʽʱ��T
float Delta_Mec_Degree_Yaw_45;       //����ģʽ����45��ת��YAW��е�ǶȲ�
float Delta_Mec_Degree_Yaw;          //��ͨ�˶���С����YAW�ǶȲ�

int16_t current_limit[4] = {0};      //�洫�����㷨�޹���������鶨��

_chassis_speed_t chassis_speed;

/*
	�����ٶȱ�����ʼ��
*/
void chassis_speed_init(_chassis_speed_t *speed_packet)
{
	 chassis_speed.rate_limit = 0;//�����ٶ�
	 chassis_speed.shift_enlarge_speed_rate = 1.5;//����shift�ӵ�����ٶ�
	 chassis_speed.speed_max = CHASSIS_SPEED_MAX;
	 for(int i=0; i<4; i++)
	 {
	 chassis_speed.Speed_xy[i]=0;
	 chassis_speed.Speed_xyz[i]=0;
	 }
	 chassis_speed.Speed_x=0;
	 chassis_speed.Speed_y=0;
	 chassis_speed.Speed_z=0;

	 chassis_speed.Speed_X_out=0;
	 chassis_speed.Speed_Y_out=0;

	 chassis_speed.Speed_x_45 = 0;
	 chassis_speed.Speed_y_45 = 0;
	 chassis_speed.sin_now_45 = 0;
	 chassis_speed.cos_now_45 = 0;

	 chassis_speed.x = 0;//С����x����
	 chassis_speed.y = 0;//С����y����
	 chassis_speed.sin_now = 0;
	 chassis_speed.cos_now = 0;
}
/*
	��̨180��תͷ
*/

void Gimbal_half_turn_Chassis_speed_set(_chassis_speed_t *chassis_speed)
{
	
		//180��ͷ
			if(fast_180_flag == 0)
				{
				  if(vision_enter_rc == OFF )				
			 
					{
						pid_calc_follow(&pid_spd_follow,motor_chassis_gimbal[Gimbal_yaw].angle,angle_target_gyro);
		        chassis_speed->Speed_z = pid_spd_follow.pos_out;
					}
				}
				
			else if(fast_180_flag == 1)
				{
					chassis_speed->Speed_z = 0;
					change_F_B_time++;
					
					if(abs(motor_chassis_gimbal[Gimbal_yaw].angle - CHASSIS_MAC_FRONT_MID_VALUE) <= 100 && change_F_B_time >= 400 )
					{
						angle_target_gyro_now = CHASSIS_MAC_FRONT_MID_VALUE;
						fast_180_flag = 0;
						change_F_B_time = 0;
					}
					if(abs(motor_chassis_gimbal[Gimbal_yaw].angle - CHASSIS_MAC_BACK_MID_VALUE) <= 100  && change_F_B_time >= 400)
					{
						angle_target_gyro_now = CHASSIS_MAC_BACK_MID_VALUE;
						fast_180_flag = 0;
						change_F_B_time = 0;
					}
					if(abs(motor_chassis_gimbal[Gimbal_yaw].angle - (CHASSIS_MAC_FRONT_MID_VALUE - TURN_45_MAC_ANGLE_VALUE)) <= 100  && change_F_B_time >= 400 && TURN_45_FLAG == 0)
					{
						angle_target_gyro_now = (CHASSIS_MAC_FRONT_MID_VALUE - TURN_45_MAC_ANGLE_VALUE);
						fast_180_flag = 0;
						change_F_B_time = 0;
					}
					if(abs(motor_chassis_gimbal[Gimbal_yaw].angle - (CHASSIS_MAC_BACK_MID_VALUE - TURN_45_MAC_ANGLE_VALUE)) <= 100 && change_F_B_time >= 400 && TURN_45_FLAG == 0)
					{
						angle_target_gyro_now = (CHASSIS_MAC_BACK_MID_VALUE - TURN_45_MAC_ANGLE_VALUE);
						fast_180_flag = 0;
						change_F_B_time = 0;
					}
					
				}
}


/*
	С����(�������͹ر�)
*/
float top_flg = 0;
void Small_top_Chassis_speed_set(_chassis_speed_t *chassis_speed)
{

	
		if(TW_VALUE > 500 && SW1_MID)
		{		
			Small_Top_Mode_Flag = Little_TOP_ON; //��־С����ģʽ����
		}
		if(TW_VALUE < -500 && SW1_MID)
		{
			Small_Top_Mode_Flag = Little_TOP_OFF; //��־С����ģʽ�ر�
		}		
		
		if(Small_Top_Mode_Flag == Little_TOP_OFF)//�ж�С����ģʽ�ر�
			{
				angle_target_gyro = angle_target_gyro_now;
				
				if(angle_target_gyro == CHASSIS_MAC_FRONT_MID_VALUE )
				{
					if(MODE == MODE_RC)
					{
					 chassis_speed->Speed_X_out = CH3_VALUE;
					 chassis_speed->Speed_Y_out = CH2_VALUE;
					}
					else if(MODE == MODE_KEY)
					{
						chassis_speed->Speed_X_out = CH1_VALUE_K;
					  chassis_speed->Speed_Y_out = CH0_VALUE_K;
					}
					if(shoot_flag.cover_flag == 1)
				  {					
		       chassis_speed->Speed_X_out = CH1_VALUE_K*0.1f;
		       chassis_speed->Speed_Y_out = CH0_VALUE_K*0.1f;
				  }
				 
        Delta_Mec_Degree_Yaw = angle_target_gyro - motor_chassis_gimbal[Gimbal_yaw].angle;				
	      chassis_speed->sin_now = sin((Delta_Mec_Degree_Yaw)*3.1415f/4096.0f);
				chassis_speed->cos_now = cos((Delta_Mec_Degree_Yaw)*3.1415f/4096.0f);
				
			  chassis_speed->Speed_x = chassis_speed->Speed_X_out*chassis_speed->cos_now -  chassis_speed->Speed_Y_out*chassis_speed->sin_now;
			  chassis_speed->Speed_y = chassis_speed->Speed_X_out*chassis_speed->sin_now +  chassis_speed->Speed_Y_out*chassis_speed->cos_now;				 
				 
				}
				else if(angle_target_gyro == CHASSIS_MAC_BACK_MID_VALUE )
				{
					if(MODE == MODE_RC)
					{
					   chassis_speed->Speed_X_out = -CH3_VALUE;
					   chassis_speed->Speed_Y_out = -CH2_VALUE;
					}
					else if(MODE == MODE_KEY)
					{
						 chassis_speed->Speed_X_out = - CH1_VALUE_K;
					   chassis_speed->Speed_Y_out = - CH0_VALUE_K;
					}
					if(shoot_flag.cover_flag == 1)
				 {					
		       chassis_speed->Speed_X_out = -CH1_VALUE_K*0.1f;
		       chassis_speed->Speed_Y_out = -CH0_VALUE_K*0.1f;
				 }
				 
        Delta_Mec_Degree_Yaw = angle_target_gyro - motor_chassis_gimbal[Gimbal_yaw].angle;				
	      chassis_speed->sin_now = sin((Delta_Mec_Degree_Yaw)*3.1415f/4096.0f);
				chassis_speed->cos_now = cos((Delta_Mec_Degree_Yaw)*3.1415f/4096.0f);
				
			  chassis_speed->Speed_x = chassis_speed->Speed_X_out*chassis_speed->cos_now -  chassis_speed->Speed_Y_out*chassis_speed->sin_now;
			  chassis_speed->Speed_y = chassis_speed->Speed_X_out*chassis_speed->sin_now +  chassis_speed->Speed_Y_out*chassis_speed->cos_now;				 
				 
				}
				else if(TURN_45_FLAG == 0) //yaw��45��ת��
				{			
					if(MODE == MODE_KEY)
					{					
						if( angle_target_gyro == (CHASSIS_MAC_FRONT_MID_VALUE - TURN_45_MAC_ANGLE_VALUE))
						{  
							 chassis_speed->Speed_x_45 = CH1_VALUE_K;
					     chassis_speed->Speed_y_45 = CH0_VALUE_K;

							 Delta_Mec_Degree_Yaw_45 = CHASSIS_MAC_FRONT_MID_VALUE - motor_chassis_gimbal[Gimbal_yaw].angle ;
						}
						else if(angle_target_gyro == (CHASSIS_MAC_BACK_MID_VALUE - TURN_45_MAC_ANGLE_VALUE))
						{
							chassis_speed->Speed_x_45 = - CH1_VALUE_K;
					    chassis_speed->Speed_y_45 = - CH0_VALUE_K;
							Delta_Mec_Degree_Yaw_45 = CHASSIS_MAC_BACK_MID_VALUE  - motor_chassis_gimbal[Gimbal_yaw].angle;
						}
						
						chassis_speed->sin_now_45 = sin((Delta_Mec_Degree_Yaw_45)*3.1415f/4096.0f);
						chassis_speed->cos_now_45 = cos((Delta_Mec_Degree_Yaw_45)*3.1415f/4096.0f);
						
						chassis_speed->Speed_x = chassis_speed->Speed_x_45*chassis_speed->cos_now_45 - chassis_speed->Speed_y_45*chassis_speed->sin_now_45;
						chassis_speed->Speed_y = chassis_speed->Speed_x_45*chassis_speed->sin_now_45 + chassis_speed->Speed_y_45*chassis_speed->cos_now_45;
					}					
				}
			}
			
			 if(Small_Top_Mode_Flag == Little_TOP_ON)//�ж�С����ģʽ����
		  {		
        				
       if(MODE == MODE_RC)
			 {
         if(angle_target_gyro == CHASSIS_MAC_FRONT_MID_VALUE)		
				 {					 
					chassis_speed->x = CH3_VALUE;
					chassis_speed->y = CH2_VALUE;
				 }
				 else if(angle_target_gyro == CHASSIS_MAC_BACK_MID_VALUE)		
				 {					 
					chassis_speed->x = -CH3_VALUE;
					chassis_speed->y = -CH2_VALUE;
				 }
				chassis_speed->Speed_z = SMALL_TOP_SPEED_FAST;
			 }
			 if(MODE == MODE_KEY)
			 {
				
				 if(angle_target_gyro == CHASSIS_MAC_FRONT_MID_VALUE)		
				 {					 
					chassis_speed->x = CH1_VALUE_K;
					chassis_speed->y = CH0_VALUE_K;
				 }
				  else if(angle_target_gyro == CHASSIS_MAC_BACK_MID_VALUE)		
				 {					 
					chassis_speed->x = -CH1_VALUE_K;
					chassis_speed->y = -CH0_VALUE_K;
				 }
			 } 
				if(JUDGE_ONLINE && Small_Top_Mode == SMALL_TOP_MODE_FAST)
				{
					if((BUFFER > 55 &&  (!chassis_speed->Speed_x && !chassis_speed->Speed_y))||KEY_SHIFT)
					{
						if(top_flg > 500)top_flg = 500;
						else
							top_flg++;
					}
					else
					{
						if(top_flg <= 0)top_flg = 0;
						else
							top_flg--;
					}

					chassis_speed->Speed_z = SMALL_TOP_SPEED_FAST + top_flg / 1.5f;
					
					if     (chassis_speed->Speed_z < SMALL_TOP_SPEED_FAST)chassis_speed->Speed_z = SMALL_TOP_SPEED_FAST;
					else if(chassis_speed->Speed_z > 600                 )chassis_speed->Speed_z = 600;	
				}

				
				Delta_Mec_Degree_Yaw = angle_target_gyro - motor_chassis_gimbal[Gimbal_yaw].angle ;
				
	      chassis_speed->sin_now = sin((Delta_Mec_Degree_Yaw)*3.1415f/4096.0f);
				chassis_speed->cos_now = cos((Delta_Mec_Degree_Yaw)*3.1415f/4096.0f);
				
			  chassis_speed->Speed_x =  chassis_speed->x*chassis_speed->cos_now -  chassis_speed->y*chassis_speed->sin_now;
			  chassis_speed->Speed_y =  chassis_speed->x*chassis_speed->sin_now +  chassis_speed->y*chassis_speed->cos_now;	
			}
}

/*
	�ٶ�����
*/
void Chassis_speed_set(_chassis_speed_t *chassis_speed)
{
	//��е
		if (MODE_MAC_OR_TOP == MODE_MAC)
		{
			if(Gimbal_angle.angle_mac_follow == CHASSIS_MAC_FRONT_MID_VALUE)
			{
				if(MODE == MODE_RC)
				{
					chassis_speed->Speed_z = CH0_VALUE;
		      chassis_speed->Speed_x = CH3_VALUE;
		      chassis_speed->Speed_y = CH2_VALUE;
				}
				else if(MODE == MODE_KEY)
				{
					//�����ƶ�
					if(KEY_Q_Speed_z == 1)
					{
						chassis_speed->Speed_z = -80;
					}
					if(KEY_E_Speed_z == 1)
					{
						chassis_speed->Speed_z = +80;
					}				
					else if(KEY_Q_Speed_z == 0 && KEY_E_Speed_z == 0)
					{
						chassis_speed->Speed_z = CH2_VALUE_K;
					}
		      chassis_speed->Speed_x = CH1_VALUE_K;
		      chassis_speed->Speed_y = CH0_VALUE_K;
				}
				//�����ֵ����ٶ�����
				if(shoot_flag.cover_flag == 1)
				{
					chassis_speed->Speed_z = CH2_VALUE_K*0.1f;
		      chassis_speed->Speed_x = CH1_VALUE_K*0.1f;
		      chassis_speed->Speed_y = CH0_VALUE_K*0.1f;
				}
			}
			else if(Gimbal_angle.angle_mac_follow == CHASSIS_MAC_BACK_MID_VALUE)
			{
				if(MODE == MODE_RC)
				{
					chassis_speed->Speed_z = CH0_VALUE;
		      chassis_speed->Speed_x = -CH3_VALUE;
		      chassis_speed->Speed_y = -CH2_VALUE;
				}
				else if (MODE == MODE_KEY)
				{
					//�����ƶ�
					if(KEY_Q_Speed_z == 1)
					{
						chassis_speed->Speed_z = -80;
					}
					if(KEY_E_Speed_z == 1)
					{
						chassis_speed->Speed_z = +80;
					}				
					else if(KEY_Q_Speed_z == 0 && KEY_E_Speed_z == 0)
					{
						chassis_speed->Speed_z = CH2_VALUE_K;
					}
		      chassis_speed->Speed_x = -CH1_VALUE_K;
		      chassis_speed->Speed_y = -CH0_VALUE_K;
				}
				//�����ֵ����ٶ�����
				if(shoot_flag.cover_flag == 1)
				{
					chassis_speed->Speed_z = -CH2_VALUE_K*0.1f;
		      chassis_speed->Speed_x = -CH1_VALUE_K*0.1f;
		      chassis_speed->Speed_y = -CH0_VALUE_K*0.1f;
				}
			}		
			   if(CAR_MODE == 1 && Small_Top_Mode_Flag == Little_TOP_OFF)
				{
						//һֱ����ͷβ�жϣ��������ݴ洢����ʱ�м������
					if(motor_chassis_gimbal[Gimbal_yaw].angle > 4500 )
					{
						angle_target_gyro = CHASSIS_MAC_BACK_MID_VALUE;
					}
					
					if(motor_chassis_gimbal[Gimbal_yaw].angle < 4500 )
					{
						angle_target_gyro = CHASSIS_MAC_FRONT_MID_VALUE;
					}
			  }
				if(CAR_MODE == 2 && Small_Top_Mode_Flag == Little_TOP_OFF)
				{
					if(motor_chassis_gimbal[Gimbal_yaw].angle > 2500 &&  motor_chassis_gimbal[Gimbal_yaw].angle < 7000)
					{
						angle_target_gyro = CHASSIS_MAC_FRONT_MID_VALUE;
					}
					else 
					{
						angle_target_gyro = CHASSIS_MAC_BACK_MID_VALUE;
					}
				}
		}
		
		//���棬�õ�ǰ����������̨����Ƕ� - �趨ֵ
		if(MODE_MAC_OR_TOP == MODE_TOP)
	  {
			if(TURN_45_FLAG == 1) 
			{
				if(CAR_MODE == 1 && Small_Top_Mode_Flag == Little_TOP_OFF)
				{
						//һֱ����ͷβ�жϣ��������ݴ洢����ʱ�м������
					if(motor_chassis_gimbal[Gimbal_yaw].angle > 4500 )
					{
						angle_target_gyro_now = CHASSIS_MAC_BACK_MID_VALUE;
					}
					
					if(motor_chassis_gimbal[Gimbal_yaw].angle < 4500 )
					{
						angle_target_gyro_now = CHASSIS_MAC_FRONT_MID_VALUE;
					}
			  }
				if(CAR_MODE == 2 && Small_Top_Mode_Flag == Little_TOP_OFF)
				{
					if(motor_chassis_gimbal[Gimbal_yaw].angle > 1305 &&  motor_chassis_gimbal[Gimbal_yaw].angle < 5512)
					{
						angle_target_gyro_now = CHASSIS_MAC_FRONT_MID_VALUE;
					}
					else 
					{
						angle_target_gyro_now = CHASSIS_MAC_BACK_MID_VALUE;
					}
				}
				
	  	}
			//180��תͷ 
			Gimbal_half_turn_Chassis_speed_set(chassis_speed);
			//С����
			Small_top_Chassis_speed_set(chassis_speed);
		
		}
}

/*
	����
*/

float Speed_all[4];

void Chassis_motor(_chassis_speed_t *chassis_speed)
{   
		Vision_Rx_Packet_t *Pack = &vision_sensor.info->RxPacket;
		chassis_speed->Speed_xy[0] =  chassis_speed->Speed_x  + chassis_speed->Speed_y;
		chassis_speed->Speed_xy[1] = -chassis_speed->Speed_x  + chassis_speed->Speed_y;
		chassis_speed->Speed_xy[2] =  chassis_speed->Speed_x  - chassis_speed->Speed_y;
		chassis_speed->Speed_xy[3] = -chassis_speed->Speed_x  - chassis_speed->Speed_y;
		
				//shift�ж�
	
	 if(MODE == MODE_RC)
		{
			for(int i = 0; i < 4; i++)
			{
				chassis_speed->Speed_xyz[0] = (chassis_speed->Speed_xy[0] + chassis_speed->Speed_z)*ENLARGE_SPEED* SHIFT_HIGH_SPD_RATE *1.5f ;
	   	  chassis_speed->Speed_xyz[1] = (chassis_speed->Speed_xy[1] + chassis_speed->Speed_z)*ENLARGE_SPEED* SHIFT_HIGH_SPD_RATE *1.5f ;
	    	chassis_speed->Speed_xyz[2] = (chassis_speed->Speed_xy[2] + chassis_speed->Speed_z)*ENLARGE_SPEED* SHIFT_HIGH_SPD_RATE *1.5f ;
		    chassis_speed->Speed_xyz[3] = (chassis_speed->Speed_xy[3] + chassis_speed->Speed_z)*ENLARGE_SPEED* SHIFT_HIGH_SPD_RATE *1.5f ;

				Cap_output_limit = CAP_OUTPUT_POWER_LIMIT;
			}	
		}
    if(KEY_SHIFT != 0 && MODE == MODE_KEY )
		{
			for(int i = 0; i < 4; i++)
			{
				chassis_speed->Speed_xyz[0] = (chassis_speed->Speed_xy[0] + chassis_speed->Speed_z)*ENLARGE_SPEED* SHIFT_HIGH_SPD_RATE;
	   	  chassis_speed->Speed_xyz[1] = (chassis_speed->Speed_xy[1] + chassis_speed->Speed_z)*ENLARGE_SPEED* SHIFT_HIGH_SPD_RATE;
	    	chassis_speed->Speed_xyz[2] = (chassis_speed->Speed_xy[2] + chassis_speed->Speed_z)*ENLARGE_SPEED* SHIFT_HIGH_SPD_RATE;
		    chassis_speed->Speed_xyz[3] = (chassis_speed->Speed_xy[3] + chassis_speed->Speed_z)*ENLARGE_SPEED* SHIFT_HIGH_SPD_RATE;

				Cap_output_limit = CAP_OUTPUT_POWER_LIMIT;
			}	
		}
		else if(KEY_SHIFT == 0 && MODE == MODE_KEY)		
		{
		
			for(int i = 0; i < 4; i++)
			{
				chassis_speed->Speed_xyz[0] = (chassis_speed->Speed_xy[0] + chassis_speed->Speed_z)*ENLARGE_SPEED* SHIFT_LOW_SPD_RATE;
				chassis_speed->Speed_xyz[1] = (chassis_speed->Speed_xy[1] + chassis_speed->Speed_z)*ENLARGE_SPEED* SHIFT_LOW_SPD_RATE;
				chassis_speed->Speed_xyz[2] = (chassis_speed->Speed_xy[2] + chassis_speed->Speed_z)*ENLARGE_SPEED* SHIFT_LOW_SPD_RATE;
				chassis_speed->Speed_xyz[3] = (chassis_speed->Speed_xy[3] + chassis_speed->Speed_z)*ENLARGE_SPEED* SHIFT_LOW_SPD_RATE;
				Cap_output_limit = CAP_OUTPUT_POWER_LIMIT*0.50f;
			}
		}			
		//�����ٶ�
		for(int i = 0; i < 4; i++)
		{
			 if (abs(chassis_speed->Speed_xyz[i]) >= CHASSIS_SPEED_MAX)		 
			 {			
				 if(chassis_speed->Speed_xyz[i] < 0)
				 {
					 chassis_speed->Speed_xyz[i] *= - (chassis_speed->speed_max / chassis_speed->Speed_xyz[i]);
				 }
				 else
				 {
				   chassis_speed->Speed_xyz[i] *= (chassis_speed->speed_max / chassis_speed->Speed_xyz[i]);
				 } 
			 }				 
	   }
		
		pid_calc(&pid_spd[Left_front_spd],  motor_chassis_gimbal[Left_front].speed_rpm,  chassis_speed->Speed_xyz[0]);
		pid_calc(&pid_spd[Right_front_spd], motor_chassis_gimbal[Right_front].speed_rpm, chassis_speed->Speed_xyz[1]);
		pid_calc(&pid_spd[Left_back_spd],   motor_chassis_gimbal[Left_back].speed_rpm,   chassis_speed->Speed_xyz[2]);
		pid_calc(&pid_spd[Right_back_spd],  motor_chassis_gimbal[Right_back].speed_rpm,  chassis_speed->Speed_xyz[3]);
	
		//��ԭ���ĵ����ٶ�����floatת��Ϊint16_t�����̹����޷�		
		for(int i = 0; i < 4; i++)
		{		
			current_limit[i] = (int16_t)pid_spd[i].pos_out ;					
		}		
		
		//���̹����޷�
		Chassis_Motor_Power_Limit(current_limit);
		
		//����ϵͳʧ���������˶�

		if(JUDGE_OFFLINE || CAP_RP_2023.work_state == DEV_OFFLINE )
		{ 
			for(int i = 0; i < 4; i++)
			{
				pid_spd[i].MaxOutput = SLOW_PID_CHASSIS_MAXOUTPUT;
			}
				CAN_cmd_chassis ( pid_spd[0].pos_out, 
													pid_spd[1].pos_out,
													pid_spd[2].pos_out,
													pid_spd[3].pos_out);
		}
		//����ϵͳ�����������˶�
		if(JUDGE_ONLINE && CAP_RP_2023.work_state == DEV_ONLINE)
		{
			if(vision_enter_rc == OFF && (Small_Top_Mode_Flag == Little_TOP_ON || !(Pack->FrameHeader.cmd_id == CMD_AIM_SMALL_BUFF || Pack->FrameHeader.cmd_id == CMD_AIM_BIG_BUFF || Pack->FrameHeader.cmd_id == CMD_AIM_ANDF)))
			{		
				CAN_cmd_chassis ( current_limit[0], 
													current_limit[1],
													current_limit[2],
													current_limit[3]);
			}
	    
			if(vision_enter_rc == ON ||((Pack->FrameHeader.cmd_id == CMD_AIM_SMALL_BUFF || Pack->FrameHeader.cmd_id == CMD_AIM_BIG_BUFF || Pack->FrameHeader.cmd_id == CMD_AIM_ANDF) && Small_Top_Mode_Flag == Little_TOP_OFF))
			{		
				CAN_cmd_chassis (0,0,0,0);
			}
		}
		
  }

