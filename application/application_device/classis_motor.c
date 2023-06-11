/********************************************************************************
  * @file			classis_motor.c
	* @author      RobotPilots@2022
  * @version		V1.0.0
  * @date			
  * @brief   		
  *******************************************************************************/
#include "classis_motor.h"

int16_t angle_target_gyro = CHASSIS_MAC_FRONT_MID_VALUE;       //陀螺仪目标角度
int16_t angle_target_gyro_now = CHASSIS_MAC_FRONT_MID_VALUE ;  //陀螺仪目标角度（实时更新）
int16_t Small_Top_Mode_Flag;                                   //小陀螺模式开启标志位
int16_t Small_Top_Mode;           														 //小陀螺模式（慢/中/快）

//遥控器0/键盘模式1
int16_t MODE ;
//陀螺仪模式/机械模式
int16_t MODE_MAC_OR_TOP;
 
int32_t change_F_B_time;             //键盘模式一键换头计算的时间
int16_t Top_Count_T;                 //变速小陀螺模式时间T
float Delta_Mec_Degree_Yaw_45;       //键盘模式底盘45°转向：YAW机械角度差
float Delta_Mec_Degree_Yaw;          //普通运动中小陀螺YAW角度差

int16_t current_limit[4] = {0};      //祖传功率算法限功率输出数组定义

_chassis_speed_t chassis_speed;

/*
	底盘速度变量初始化
*/
void chassis_speed_init(_chassis_speed_t *speed_packet)
{
	 chassis_speed.rate_limit = 0;//限制速度
	 chassis_speed.shift_enlarge_speed_rate = 1.5;//按下shift加到最大速度
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

	 chassis_speed.x = 0;//小陀螺x坐标
	 chassis_speed.y = 0;//小陀螺y坐标
	 chassis_speed.sin_now = 0;
	 chassis_speed.cos_now = 0;
}
/*
	云台180°转头
*/

void Gimbal_half_turn_Chassis_speed_set(_chassis_speed_t *chassis_speed)
{
	
		//180掉头
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
	小陀螺(含开启和关闭)
*/
float top_flg = 0;
void Small_top_Chassis_speed_set(_chassis_speed_t *chassis_speed)
{

	
		if(TW_VALUE > 500 && SW1_MID)
		{		
			Small_Top_Mode_Flag = Little_TOP_ON; //标志小陀螺模式开启
		}
		if(TW_VALUE < -500 && SW1_MID)
		{
			Small_Top_Mode_Flag = Little_TOP_OFF; //标志小陀螺模式关闭
		}		
		
		if(Small_Top_Mode_Flag == Little_TOP_OFF)//判断小陀螺模式关闭
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
				else if(TURN_45_FLAG == 0) //yaw轴45°转向
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
			
			 if(Small_Top_Mode_Flag == Little_TOP_ON)//判断小陀螺模式开启
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
	速度设置
*/
void Chassis_speed_set(_chassis_speed_t *chassis_speed)
{
	//机械
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
					//缓慢移动
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
				//开弹仓底盘速度设置
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
					//缓慢移动
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
				//开弹仓底盘速度设置
				if(shoot_flag.cover_flag == 1)
				{
					chassis_speed->Speed_z = -CH2_VALUE_K*0.1f;
		      chassis_speed->Speed_x = -CH1_VALUE_K*0.1f;
		      chassis_speed->Speed_y = -CH0_VALUE_K*0.1f;
				}
			}		
			   if(CAR_MODE == 1 && Small_Top_Mode_Flag == Little_TOP_OFF)
				{
						//一直进行头尾判断，并将数据存储到临时中间变量中
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
		
		//跟随，用当前反馈的下云台电机角度 - 设定值
		if(MODE_MAC_OR_TOP == MODE_TOP)
	  {
			if(TURN_45_FLAG == 1) 
			{
				if(CAR_MODE == 1 && Small_Top_Mode_Flag == Little_TOP_OFF)
				{
						//一直进行头尾判断，并将数据存储到临时中间变量中
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
			//180°转头 
			Gimbal_half_turn_Chassis_speed_set(chassis_speed);
			//小陀螺
			Small_top_Chassis_speed_set(chassis_speed);
		
		}
}

/*
	底盘
*/

float Speed_all[4];

void Chassis_motor(_chassis_speed_t *chassis_speed)
{   
		Vision_Rx_Packet_t *Pack = &vision_sensor.info->RxPacket;
		chassis_speed->Speed_xy[0] =  chassis_speed->Speed_x  + chassis_speed->Speed_y;
		chassis_speed->Speed_xy[1] = -chassis_speed->Speed_x  + chassis_speed->Speed_y;
		chassis_speed->Speed_xy[2] =  chassis_speed->Speed_x  - chassis_speed->Speed_y;
		chassis_speed->Speed_xy[3] = -chassis_speed->Speed_x  - chassis_speed->Speed_y;
		
				//shift判断
	
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
		//限制速度
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
	
		//把原来的底盘速度类型float转换为int16_t：底盘功率限幅		
		for(int i = 0; i < 4; i++)
		{		
			current_limit[i] = (int16_t)pid_spd[i].pos_out ;					
		}		
		
		//底盘功率限幅
		Chassis_Motor_Power_Limit(current_limit);
		
		//裁判系统失联，低速运动

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
		//裁判系统正常，正常运动
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

