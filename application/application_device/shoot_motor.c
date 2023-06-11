/********************************************************************************
  * @file			gimbal_motor.c
	* @author      RobotPilots@2022
  * @version		V1.0.0
  * @date			
  * @brief   		
  *******************************************************************************/

#include "shoot_motor.h"

//��ǹ��־λ
char change_barrel_rc = 1;
char change_barrel_rc_flag = 1;

float friction_speed_tar =  FRICTION_SPEED;                //Ħ����Ŀ���ٶ�
float flick_speed_tar = FLICK_SPEED;                        //������Ŀ���ٶ�
float flick_pos_target = 0;                                //����ģʽ�²�����λ�û���ʼĿ��ֵ
float flick_motor_current = 0;                             //�����ֵ�����������2006��
float flick_pos_target_jam = 0;                            //��תλ�û�Ŀ��ֵ

_shoot_skip_t shoot_skip;
_shoot_flag_t shoot_flag;
_change_barr_t change_barr;


/*
	���������ʼ��
*/	


void shoot_skip_init(_shoot_skip_t *shoot_skip_t)
{
	shoot_skip_t->s2_friction_last = 0;        
	shoot_skip_t->s2_friction_now = 0;

	shoot_skip_t->s2_flick_last = 0;
	shoot_skip_t->s2_flick_now = 0;

	shoot_skip_t->s2_cover_last = 0;
	shoot_skip_t->s2_cover_now = 0;
}
/*
	�����־λ��ʼ��
*/	
void shoot_flag_init(_shoot_flag_t *shoot_flag_t, _change_barr_t *change_barr_t)
{
	 shoot_flag_t->friction_flag_top  = OFF;            //����ģʽĦ���ֿ��ر�־λ
	 shoot_flag_t->flick_flag_top  = OFF ;              //����ģʽ�����ֿ��ر�־λ

	 shoot_flag_t->friction_flag_mac  = OFF;            //��еģʽĦ���ֿ��ر�־λ
	 shoot_flag_t->flick_flag_mac  = OFF;               //��еģʽ�����ֿ��ر�־λ

	 shoot_flag_t->cover_flag = OFF;
	
	 change_barr_t->no_barrel_clock_time = 0;
	 change_barr_t->barrel_clock_time = 0;
	 change_barr_t->barrel_clock = ON;
	 change_barr_t->barrel_1_pos = 0;
	 change_barr_t->barrel_2_pos = 0;
	 change_barr_t->change_barral_pos_target = 0;  
	 change_barr_t->change_barral_speed_tar = -CHANGE_BARR_SPEED;
	 change_barr_t->change_barral_state = 0; 
	 change_barr_t->change_barral_state_2_to_1 = 0;  
	 change_barr_t->change_barral_state_1_to_2 = 0;  
	 change_barr_t->change_barral_motor_current = 0;                     //��ǹ�ܵ�����������2006��
	 change_barr_t->barrel_ID = 1	;                                      //ǹ��Ĭ��ID��
}




/*
	 ����ģʽ������ģʽ��ң�������ƣ�
   
*/	
void shoot_motor_running_fire_rc(rc_sensor_t *rc)
{
	if(SW1_MID)
	{
		if(SW2_UP)
		{
			shoot_flag.flick_flag_top = ON; //�����ֿ���	
		}
		else
		{
			shoot_flag.flick_flag_top = OFF; //�����ֹر�	
		}
	}
}

/*
	 ��ת����
*/	
int16_t judge_jam_time = 0;
int16_t judge_jam_flag = 0;
void shoot_motor_fire_jam_judge()
{
	//������������
		if(abs(motor_shoot[flick].real_current) >= 7000 && shoot_flag.flick_flag_mac == ON)		
		{		
			judge_jam_flag++;
		}

}

/*
	 �Զ����
*/	
int16_t Auto_Shoot_Flag;
int16_t Auto_Shoot_Time;
extern char Shoot_Permit;
extern char Rx_Cmd_id;
char Auto_Shoot_Single_flag;


void Auto_Shoot_Ctrl(void)
{
	if(Auto_Shoot_Flag == 1 &&  Rx_Cmd_id == 10)
	{
		if(shoot_flag.flick_flag_mac  == 0 && Auto_Shoot_Single_flag == 1)
		{
			 if(Shoot_Permit == ON  && !STOP_SHOOT_1 && motor_shoot[R_friction].speed_rpm >= 3000)
			 {				
				 single_shoot_flag = OFF;	
				 shoot_flag.flick_flag_top = ON;	 
			 }				 
    }	
	}
	else if(Auto_Shoot_Flag == 0 &&  Rx_Cmd_id == 10)
	{
		Auto_Shoot_Single_flag = 0;
  	shoot_flag.flick_flag_top = OFF;	 
	}	 
			
}
/*
	 Ħ���ֶϵ��ж�
   
*/	
void Fri_Stop_Judge(void)
{
	if(motor_offline_check.shoot_motor_offline->offline_cnt_R >= motor_offline_check.shoot_motor_offline->offline_cnt_max 
		&& motor_offline_check.shoot_motor_offline->offline_cnt_L >= motor_offline_check.shoot_motor_offline->offline_cnt_max)
	{
		shoot_flag.friction_flag_top = OFF;
	}
}
/*
	 ����ģʽ������ģʽ�£�
   
*/	

void shoot_motor_running_fire()
{
	shoot_motor_running_fire_rc(&rc);
	
	if(shoot_flag.flick_flag_top == ON && shoot_flag.friction_flag_top == ON && change_barr.change_barral_state == OFF  ) //�Ҳ�ť��,���Ħ����û�п�������ô�����ֲ��ܿ���

	{		
		shoot_flag.flick_flag_mac  = OFF;//������Σ�ֻҪ��������������Ϊ�رյ���ģʽ
		//��ת����
		shoot_motor_fire_jam_judge();	
		if(judge_jam_flag >= 100)
		{ 
			judge_jam_time++;
			pid_calc_shoot_continuous_flick(&pid_shoot_flick_wheel_follow_spd, motor_shoot[flick].speed_rpm, -flick_speed_tar);
			if(judge_jam_time >= 25)
			{
				judge_jam_flag = 0;
				judge_jam_time = 0;
			}
		}
		
		if(judge_jam_flag < 100)
		{
			pid_calc_shoot_continuous_flick(&pid_shoot_flick_wheel_follow_spd, motor_shoot[flick].speed_rpm, flick_speed_tar);
		}
			
	}
	
	if(shoot_flag.flick_flag_top == OFF) //�Ҳ�ť��
	{		
		pid_calc_shoot_continuous_flick(&pid_shoot_flick_wheel_follow_spd, motor_shoot[flick].speed_rpm, 0);				
	}
	
	//Ħ����������
	if((MODE == MODE_RC && MODE_MAC_OR_TOP == MODE_TOP) || (SW1_MID && vision_enter_rc == ON ))
	{
		shoot_skip.s2_friction_now = SW2;
		
		if(shoot_skip.s2_friction_last != shoot_skip.s2_friction_now && SW2_DOWN)
		{
			shoot_flag.friction_flag_top =! shoot_flag.friction_flag_top;
			
		}	
		shoot_skip.s2_friction_last = shoot_skip.s2_friction_now;
  }
	
 
	Shoot_heat_limit(&change_barr);
  Fri_Spd_adjust();
	Shoot_speed_limit();
	
	 if(shoot_flag.flick_flag_top == ON  && shoot_flag.friction_flag_top == ON)
	{
		flick_pos_target  = motor_shoot[flick].total_angle;//�����ܽǶ��뵥��Ŀ��ֵ���
		flick_motor_current = pid_shoot_flick_wheel_follow_spd.pos_out;
	}	
	
	
}	
		 
int32_t delta_angle = 0;
int16_t single_shoot_heat_control = 1;//ԭ����0
char Buff_Hand_Shoot_flag;
extern float usTms; //���ʱ

/*
	 ����ģʽ��������Ҫʹ��λ�û�����������ת��˳ʱ�벦�����ĽǶ�Ϊ45��
   
*/
void shoot_motor_single_fire()
{
	//����ģʽ����
	shoot_flag.friction_flag_mac = ON;
		
	//������������
	if(judge_jam_flag < 100)
	{
		
		if(((MODE == MODE_RC && MODE_MAC_OR_TOP == MODE_MAC) || vision_enter_rc == ON )&& shoot_flag.friction_flag_top == ON)
		{
			shoot_skip.s2_flick_now = SW2;
			if(shoot_skip.s2_flick_last != shoot_skip.s2_flick_now && SW2_UP && single_shoot_heat_control == 1 && shoot_flag.flick_flag_mac == OFF)
			{
				shoot_flag.flick_flag_mac  = ON;			
				flick_pos_target -= SINGLE_SHOOT_ANGLE;//����һ�Σ�λ�û�Ŀ���һ��		
			}	
			shoot_skip.s2_flick_last = shoot_skip.s2_flick_now;
			
		}	
		if(MODE == MODE_KEY && single_shoot_flag && change_barr.change_barral_state == OFF && shoot_flag.friction_flag_top == ON )		
		{
			shoot_skip.s2_flick_now = MOUSE_LEFT;
		
			if(shoot_skip.s2_flick_last != shoot_skip.s2_flick_now && MOUSE_LEFT == 1 && single_shoot_heat_control == 1  && shoot_flag.flick_flag_mac  == OFF )
			{
				shoot_flag.flick_flag_mac  = ON;
				flick_pos_target -= SINGLE_SHOOT_ANGLE;//����һ�Σ�λ�û�Ŀ���һ��	
			}	
			shoot_skip.s2_flick_last = shoot_skip.s2_flick_now;
		}		
		else if(MODE == MODE_KEY  && change_barr.change_barral_state == OFF && Rx_Cmd_id == 10 && shoot_flag.friction_flag_top == ON && Auto_Shoot_Single_flag == 0)		
		{
			shoot_skip.s2_flick_now = Auto_Shoot_Flag;
			if(shoot_skip.s2_flick_last != shoot_skip.s2_flick_now && Auto_Shoot_Flag == 1 && single_shoot_heat_control == 1 && shoot_flag.flick_flag_mac  == OFF)
			{
				shoot_flag.flick_flag_mac  = ON;
				flick_pos_target -= SINGLE_SHOOT_ANGLE;//����һ�Σ�λ�û�Ŀ���һ��	
				Auto_Shoot_Single_flag = 1;			
			}	
			shoot_skip.s2_flick_last = shoot_skip.s2_flick_now;
		}		
		
		
		//�Զ����
		 Vision_Rx_Packet_t *Pack = &vision_sensor.info->RxPacket;
		 if(MODE == MODE_KEY && Pack->FrameHeader.cmd_id == CMD_AIM_SMALL_BUFF  && change_barr.change_barral_state == OFF 
			 && vision_enter == ON && shoot_flag.friction_flag_top == ON && Buff_Hand_Shoot_flag == 0)
		{		
			if(single_shoot_heat_control == 1 && usTms>= 650
				
			) //�ݶ�600����,����500������������̫�У�
			{			
				shoot_flag.friction_flag_top = ON;				
				shoot_flag.flick_flag_mac  = ON;
				flick_pos_target -= SINGLE_SHOOT_ANGLE;//����һ�Σ�λ�û�Ŀ���һ��	      	
				usTms = 0;
			}			
		}			
		else if(MODE == MODE_KEY && Pack->FrameHeader.cmd_id == CMD_AIM_BIG_BUFF  && change_barr.change_barral_state == OFF 
			     && vision_enter == ON && shoot_flag.friction_flag_top == ON && Buff_Hand_Shoot_flag == 0)
		{		
			if(single_shoot_heat_control == 1 && usTms>= 1000) //�ݶ�600����
			{			
				shoot_flag.friction_flag_top = ON;				
				shoot_flag.flick_flag_mac  = ON;
				flick_pos_target -= SINGLE_SHOOT_ANGLE;//����һ�Σ�λ�û�Ŀ���һ��	      	
				usTms = 0;
			}			
		}		
	}		
	
	//��ת�ж� 
  shoot_motor_fire_jam_judge();
	//������PID���㣬��Ҫ�����ж�:���Ħ����û�п�������ô�����ֲ��ܿ���
	if(shoot_flag.friction_flag_top == ON && judge_jam_flag < 100)
	{		
			cascade_pid_ctrl_shoot(&pid_shoot_flick_wheel_mac_pos, 
														 &pid_shoot_flick_wheel_mac_spd,
														 motor_shoot[flick].total_angle,
														 motor_shoot[flick].speed_rpm,
														 flick_pos_target); 		
		delta_angle = abs(motor_shoot[flick].total_angle - flick_pos_target );
		//�жϵ��ε�������	
		if(delta_angle < 1000 && judge_jam_flag  < 100) 
		{
			shoot_flag.flick_flag_mac  = OFF;
		}
	}
	if(judge_jam_flag >= 500 && delta_angle >= 1000)
	{
		  judge_jam_time++;		 
			pid_calc_shoot_continuous_flick(&pid_shoot_flick_wheel_follow_spd, motor_shoot[flick].speed_rpm, -flick_speed_tar);
			if(judge_jam_time >= 25)
			{
				judge_jam_flag = 0;
				judge_jam_time = 0;
			}
	}
  
	Shoot_heat_limit(&change_barr);
	Shoot_speed_limit();
	Fri_Spd_adjust();
		
	if(shoot_flag.flick_flag_mac == ON && shoot_flag.friction_flag_top == ON && judge_jam_flag < 100)
	{
	 flick_motor_current = pid_shoot_flick_wheel_mac_spd.pos_out;
	}
	if(judge_jam_flag >= 100 && shoot_flag.friction_flag_top == ON)
	{
		flick_motor_current = pid_shoot_flick_wheel_follow_spd.pos_out;
	}
	if(change_barr.change_barral_state == ON)
	{
		shoot_flag.flick_flag_top = OFF;
	}
}


/*
	 ���ֹͣPID���ƣ���ж����
*/
extern char Key_R_Fast_Shoot;
int32_t R_Shoot_pos;
void shoot_stop()
{
		if(shoot_flag.flick_flag_top == OFF && shoot_flag.flick_flag_mac == OFF && judge_jam_flag < 100 && Key_R_Fast_Shoot == OFF)
		{
			cascade_pid_ctrl_shoot( &pid_shoot_flick_wheel_stop_pos, 
														 &pid_shoot_flick_wheel_stop_spd,
														 motor_shoot[flick].total_angle,
														 motor_shoot[flick].speed_rpm,
														 flick_pos_target); 	
			flick_motor_current = pid_shoot_flick_wheel_stop_spd.pos_out;
		}
}
/*
	R������Ƶ�������
*/
int32_t bullet_num_permit;

char heat_update;
char Next_R_Flag;
int16_t delay_time;


void R_Fast_Shoot(void)
{	
	if(change_barr.change_barral_state == OFF)//��ǹʱ�������
	{
		if(Key_R_Fast_Shoot == ON && Next_R_Flag == 0)
		{
			if(heat_update == 1 )
			{
				bullet_num_permit = (int)(R_SHOOT);
				bullet_num_permit = bullet_num_permit / 10;
				flick_pos_target = motor_shoot[flick].total_angle - (bullet_num_permit) * SINGLE_SHOOT_ANGLE ;
				Next_R_Flag = 1;		
				heat_update = 0;		
			}	
		}
		if(shoot_flag.friction_flag_top == ON && judge_jam_flag < 100 && Key_R_Fast_Shoot == ON)
		{		
				cascade_pid_ctrl_shoot(&pid_shoot_flick_wheel_mac_pos, 
																 &pid_shoot_flick_wheel_mac_spd,
																 motor_shoot[flick].total_angle,
																 motor_shoot[flick].speed_rpm,
																 flick_pos_target); 		
				flick_motor_current = pid_shoot_flick_wheel_mac_spd.pos_out;
		}
		if(abs(flick_pos_target - motor_shoot[flick].total_angle) <= 1000)
		{
			delay_time++;
			if(delay_time >= 100)
			{
				Key_R_Fast_Shoot = OFF;
				Next_R_Flag  = 0;	
				
			}	
		}
		if(Key_R_Fast_Shoot == OFF)
		{
			delay_time = 0;
			heat_update = 1;		
			if(!MOUSE_LEFT)
			{
				shoot_flag.flick_flag_top = OFF;		
			}
		}
	}
}
/*
	ǹ����������
*/

void Shoot_heat_limit(_change_barr_t *change_barr_t)
{
	if(JUDGE_ONLINE)
	{

	  //ǹ��1����
		if(START_SHOOT_1 && change_barr_t->change_barral_state_2_to_1 == OFF && change_barr_t->change_barral_state_1_to_2 == OFF && change_barr_t->barrel_ID == 1)
		{
			//����ģʽ
			if(shoot_flag.flick_flag_top == ON)
			{
				//�����ٶȵ���
				if(Key_R_Fast_Shoot == OFF || Auto_Shoot_Flag == 0)
				{
					if(SLOW_SHOOT_1 || SLOW_SHOOT_2)
					{
						flick_speed_tar = FLICK_SPEED_DEC;
					}
					if(HIGH_SHOOT_1 || HIGH_SHOOT_2)
					{
						flick_speed_tar = FLICK_SPEED;
					}
				}
				if(Key_R_Fast_Shoot == ON || Auto_Shoot_Flag == 1)
				flick_speed_tar= FLICK_SPEED_HIGH; 	
			}
			
			//����ģʽ
			if(shoot_flag.friction_flag_mac == ON )
			{
				single_shoot_heat_control = 1;
			}
		}
		
		#if  CAR_MODE == 2 && DOUBLE_BARR == 2
		//ǹ��2����
		if(START_SHOOT_2 && change_barr_t->change_barral_state_1_to_2 == OFF && change_barr_t->change_barral_state_2_to_1 == OFF && change_barr_t->barrel_ID == 2)
		{
			//����ģʽ
			if(shoot_flag.flick_flag_top == ON)
			{
				if(Key_R_Fast_Shoot == OFF)
				{
					//�����ٶȵ���
					if(SLOW_SHOOT_1 || SLOW_SHOOT_2)
					{
						flick_speed_tar = FLICK_SPEED_DEC;
					}
					if(HIGH_SHOOT_1 || HIGH_SHOOT_2)
					{
						flick_speed_tar = FLICK_SPEED;
					}
			  }
			}
			if(Key_R_Fast_Shoot == ON)
				flick_speed_tar= FLICK_SPEED_HIGH; 	
			
			//����ģʽ
			if(shoot_flag.friction_flag_mac == ON )
			{
				single_shoot_heat_control = 1;
			}
		}
		#endif
		
		
		
		//ǹ��1ֹͣ���
		if(STOP_SHOOT_1 && change_barr_t->barrel_ID == 1)
		{
			//����ģʽ
			if(shoot_flag.flick_flag_top == ON)
			{
				flick_speed_tar =0;
			}
			//����ģʽ
			if(shoot_flag.friction_flag_mac == ON )
			{
				single_shoot_heat_control = 0;
			}
		}
 
		#if  CAR_MODE == 2 && DOUBLE_BARR == 2
	  //ǹ��2ֹͣ���
		if(STOP_SHOOT_2 && change_barr_t->barrel_ID == 2)
		{
			//����ģʽ
			if(shoot_flag.flick_flag_top == ON)
			{
				flick_speed_tar =0;
			}
			//����ģʽ
			if(shoot_flag.friction_flag_mac == ON )
			{
				single_shoot_heat_control = 0;
			}
		}	
    #endif		
	}
} 

float shoot_limit;

/*
	�������������
*/	
float shoot_speed_last = FRICTION_SPEED;
float shoot_speed_now;
float shoot_speed_limit_last;
float shoot_speed_limit_now;
char  shoot_speed_change_flag;
char  speed_dec_flag;
char  speed_add_flag;
char  low_speed_time_num;
float shoot_adot;
float speed_high_flg;

void Fri_Spd_adjust(void)
{	
	if(CAR_MODE == 1 && BARR == 2)
	{
		if(change_barr.barrel_ID == 1)
		{
			shoot_limit = SHOOT_SPEED_LIMIT_1;
		}
		else if(change_barr.barrel_ID == 2)
		{
			shoot_limit = SHOOT_SPEED_LIMIT_2;
		}
	}
	else
	{
		shoot_limit = SHOOT_SPEED_LIMIT_1;
	}
	
	//������������ʱ
	if(shoot_speed_limit_last != shoot_limit )
	{
		shoot_speed_change_flag = ON;
		speed_dec_flag = 0;
		speed_add_flag = 0;
		low_speed_time_num = 0;		
	}
	
	shoot_speed_now	= SHOOT_SPEED;  /*���´�ʱ����*/
	
	if(shoot_speed_last != shoot_speed_now)
	{
		shoot_speed_change_flag = OFF;
	}
  
	if( shoot_speed_last != shoot_speed_now && shoot_speed_change_flag == OFF)
	{		
		if(shoot_speed_now < (shoot_limit - 2.2f))//������ٵ���27.8m/s
		{
			low_speed_time_num++;
		}		
		
		/*�����ж�*/ 		/*�����ж�*/
		if(shoot_limit - 0.5 <= shoot_speed_now ||low_speed_time_num == 3 )
		{	
			speed_high_flg = speed_high_flg + (shoot_limit - 1 - shoot_speed_now) * 100;
			low_speed_time_num = 0;		
		}		
		if(shoot_speed_now <= (shoot_limit - 0.9f))
		{
			speed_dec_flag ++;
		  speed_add_flag = 0;
		}
		if(shoot_speed_now >= (shoot_limit - 1.35f))
		{
			speed_dec_flag = 0;
		  speed_add_flag++;
		}		
	}
	
	
	
	if(speed_dec_flag == 2)
	{
		shoot_adot++;
		speed_dec_flag = 0;
		speed_add_flag = 0;		
	}
	if(speed_add_flag == 2)
	{
		shoot_adot--;
		speed_dec_flag = 0;
		speed_add_flag = 0;		
	}
	shoot_speed_last	= shoot_speed_now;
	shoot_speed_limit_last = shoot_limit;
}	
	
	

void Shoot_speed_limit(void)
{
	
	/*����ٶ�*/
	int16_t Fri_Speed = 0;
  char temR,temL;
	int16_t tem_err_L,tem_err_R;
	/*ʧ������*/
	if(JUDGE_OFFLINE)
	{
		Fri_Speed = Fri_spd_15;
	}
	/*���ߴ���*/
	else
	{		
		/*����ٶ�*/
		switch((uint16_t)shoot_limit)
		{
			case 0:
				Fri_Speed = 0;
				break;
			case 15:
				Fri_Speed = Fri_spd_15;
				break; 
			case 18:
				Fri_Speed = Fri_spd_18;
				break;		
			case 20:
				Fri_Speed = Fri_spd_20;
				break;	
			case 22:
				Fri_Speed = Fri_spd_22;
				break;			
			case 30:
				Fri_Speed = Fri_spd_30;
				break;	
			default:
				if(shoot_limit < 40 && shoot_limit)
					Fri_Speed = Fri_spd_15 + (shoot_limit - 15) * 110;
				else
					Fri_Speed = Fri_spd_30;
				break;
		}	 
		friction_speed_tar = Fri_Speed;     /*�����Ӿ��ĵ���*/
			/*���ٴ��� ���ʹ���*/
		Fri_Speed = Fri_Speed - speed_high_flg;
		Fri_Speed = Fri_Speed - shoot_adot * 10;

		/*Ħ�����¶ȿ���ת��*/

	  temL = motor_shoot[L_friction].hall;
	  temR = motor_shoot[R_friction].hall;
		
	  tem_err_L = temL - 30;
	  tem_err_R = temR - 30;
		
	  //pid_shoot_left_friction_wheel_spd.target = (Fri_Speed + 80 * tem_err_L/10);
    //pid_shoot_right_friction_wheel_spd.target = -(Fri_Speed + 80 * tem_err_R/10);
		pid_shoot_left_friction_wheel_spd.target = (Fri_Speed);
    pid_shoot_right_friction_wheel_spd.target = -(Fri_Speed);
				
	if(shoot_flag.friction_flag_top == ON) /*�Ҳ�ť��*/	
	{  
  	pid_calc_shoot_friction(&pid_shoot_left_friction_wheel_spd, motor_shoot[L_friction].speed_rpm,  pid_shoot_left_friction_wheel_spd.target);
	  pid_calc_shoot_friction(&pid_shoot_right_friction_wheel_spd, motor_shoot[R_friction].speed_rpm, pid_shoot_right_friction_wheel_spd.target);	   
	}
	else if(shoot_flag.friction_flag_top == OFF || shoot_limit == 0)
	{	   
     pid_calc_shoot_friction(&pid_shoot_left_friction_wheel_spd, motor_shoot[L_friction].speed_rpm, 0);
		 pid_calc_shoot_friction(&pid_shoot_right_friction_wheel_spd, motor_shoot[R_friction].speed_rpm, 0);
	}	

 }
}


/*-------------------------------------------------------------------------------------------------------
���ֲ��� �����ر�
-------------------------------------------------------------------------------------------------------*/
void Cover_Open(void)
{	
	if(CAR_MODE == 1)
	{
		COVER_PwmOut(50);
	}
  else if(CAR_MODE == 2)
	{	
		COVER_PwmOut(80);
	}	
}
void Cover_Close(void)
{	
	if(CAR_MODE == 1)
	{
		COVER_PwmOut(230);
	}
	else if(CAR_MODE == 2)
	{
	  COVER_PwmOut(210);   
	}		
}

/*
	���ֿ���
*/
void Box_Cover_Control_RC(void)
{
	/*���ֿ���������*/
	if(SW1_DOWN)
	{
		shoot_skip.s2_cover_now = SW2;
		if(shoot_skip.s2_cover_last != shoot_skip.s2_cover_now && SW2_DOWN)
		{
			shoot_flag.cover_flag =! shoot_flag.cover_flag;
			
		}	
		shoot_skip.s2_cover_last = shoot_skip.s2_cover_now;
	}
}

void Box_Cover_Control()
{
	
	Box_Cover_Control_RC();
	if(shoot_flag.cover_flag == 1)
	{
		Cover_Open();
	}
  if(shoot_flag.cover_flag == 0)
	{
		Cover_Close();
	}
		
}

/*
#define SHOOT_SPEED_LIMIT_1       judge_info.game_robot_status.shooter_id1_17mm_speed_limit
#define SHOOT_SPEED_LIMIT_2   judge_info.game_robot_status.shooter_id2_17mm_speed_limit

#define BARREL_SPEED_1   judge_info.game_robot_status.shooter_id1_17mm_speed_limit
#define BARREL_SPEED_2   judge_info.game_robot_status.shooter_id2_17mm_speed_limit

#define SHOOTING_HEAT_1        judge_sensor.info->power_heat_data.shooter_id1_17mm_shooting_heat
#define SHOOTING_HEAT_2    judge_sensor.info->power_heat_data.shooter_id2_17mm_shooting_heat

#define SHOOTING_HEAT_LIMIT_1      judge_sensor.info->game_robot_status.shooter_id1_17mm_shooting_limit
#define SHOOTING_HEAT_LIMIT_2  judge_sensor.info->game_robot_status.shooter_id2_17mm_shooting_limit


#define STOP_SHOOT_1    (SHOOTING_HEAT_LIMIT_1 - SHOOTING_HEAT_1 <= 20)
#define START_SHOOT_1   (SHOOTING_HEAT_LIMIT_1 - SHOOTING_HEAT_1 > 20)
#define STOP_SHOOT_2    (SHOOTING_HEAT_LIMIT_2 - SHOOTING_HEAT_2 <= 20)
#define START_SHOOT_2   (SHOOTING_HEAT_LIMIT_2 - SHOOTING_HEAT_2 > 20)
#define SLOW_SHOOT_1    (SHOOTING_HEAT_LIMIT_1 - SHOOTING_HEAT_1 <= 80)
#define SLOW_SHOOT_2    (SHOOTING_HEAT_LIMIT_2 - SHOOTING_HEAT_2 <= 80��
#define HIGH_SHOOT_1    (SHOOTING_HEAT_LIMIT_1 - SHOOTING_HEAT_1 > 80)
#define HIGH_SHOOT_2    (SHOOTING_HEAT_LIMIT_2 - SHOOTING_HEAT_2 > 80)
*/


/*
��ң��/�ϵ�

*/
void change_clock(_change_barr_t *change_barr_t)
{
	if(motor_offline_check.shoot_motor_offline->offline_cnt_f >= motor_offline_check.shoot_motor_offline->offline_cnt_max )
	{
		change_barr_t->barrel_clock = ON;
	}
 if(change_barr_t->barrel_clock == ON)
 {
	 change_barr_t->barrel_clock_time++;
	if(change_barr_t->barrel_clock_time <= 5000)
	{
		 change_barr_t->change_barral_speed_tar = -CHANGE_BARR_SPEED;
		 pid_calc_shoot_continuous_flick(&pid_shoot_change_barrel_spd, motor_shoot[change_barrel].speed_rpm, change_barr_t->change_barral_speed_tar);	
		 change_barr_t->change_barral_motor_current = pid_shoot_change_barrel_spd.pos_out;
	}
	if(abs(motor_shoot[change_barrel].real_current) >= 5000 && abs(motor_shoot[change_barrel].speed_rpm) <= 10) //��λ�ж϶�ת
	{
		change_barr_t->barrel_1_pos = motor_shoot[change_barrel].total_angle;
		change_barr_t->change_barral_pos_target = change_barr_t->barrel_1_pos;
		change_barr_t->barrel_clock = OFF;
		change_barr_t->barrel_clock_time = 0;
	}
 }	
 
}


char LOCK1 = 0;
char LOCK2 = 0;
void change_barrel_motor(_change_barr_t *change_barr_t)
{
	 /********ң������ǹ*****************************/
	 if(TW_VALUE > 500)
	 {
		 if(change_barrel_rc == 1 && change_barrel_rc_flag == 0 )
		 {
				change_barrel_rc = 2;
			  change_barrel_rc_flag = 1;
		 }
		 if(change_barrel_rc == 2 && change_barrel_rc_flag == 0 )
		 {
				change_barrel_rc = 1;
			  change_barrel_rc_flag = 1;
		 }
	 }
	 if(TW_VALUE == 0)
	 {
		 change_barrel_rc_flag = 0;
		
	 }	 
	 /********ң������ǹ******************************/ 
	 
	 
	if(change_barr_t->barrel_clock == OFF)
	{		
		/*
		  ǹ��1�������� && ǹ��2��ʱ������  �л� ǹ��2
		*/
		if((STOP_SHOOT_1 && START_SHOOT_2 && change_barr_t->change_barral_state_2_to_1 == OFF && LOCK1 == 0) 
			|| (change_barrel_rc == 2 && LOCK1 == 0) )
		{
			change_barr_t->change_barral_state_1_to_2 = ON;	
      LOCK1 = 1;
			LOCK2 = 0;
      change_barr_t->barrel_ID	= 2;		
			
		}	
		if(change_barr_t->change_barral_state_1_to_2 == ON)
		{
			change_barr_t->barrel_clock_time++;
			if(change_barr_t->barrel_clock_time <=5000)
			{
				change_barr_t->change_barral_speed_tar = CHANGE_BARR_SPEED_FAST;
				pid_calc_shoot_continuous_flick(&pid_shoot_change_barrel_spd, motor_shoot[change_barrel].speed_rpm, change_barr_t->change_barral_speed_tar);	
			
			}
			if(abs(motor_shoot[change_barrel].real_current) >= 7000 && abs(motor_shoot[change_barrel].speed_rpm) <= 200) //�ж϶�ת
			{
				change_barr_t->barrel_2_pos = motor_shoot[change_barrel].total_angle;
				change_barr_t->change_barral_pos_target = change_barr_t->barrel_2_pos;
				change_barr_t->change_barral_state_1_to_2 = OFF;
				
				
				change_barr_t->barrel_clock_time = 0;
			}
			 change_barr_t->change_barral_pos_target = change_barr_t->barrel_2_pos ;
		}
	
		
		/*		
     ǹ��2�������� && ǹ��1��ʱ������  �л� ǹ��1
		*/

		if((STOP_SHOOT_2 && START_SHOOT_1  && change_barr_t->change_barral_state_1_to_2 == OFF && LOCK2 == 0)
			 || (change_barrel_rc == 1 && LOCK2 == 0))

		{
      change_barr_t->change_barral_state_2_to_1 = ON;
			LOCK1 = 0;
			LOCK2 = 1;
			change_barr_t->barrel_ID	 = 1;	
		}	
		
		if( change_barr_t->change_barral_state_2_to_1 == ON)
		{
			change_barr_t->barrel_clock_time++;
			if(change_barr_t->barrel_clock_time <= 5000)
			{
				change_barr_t->change_barral_speed_tar = - CHANGE_BARR_SPEED_FAST;
				pid_calc_shoot_continuous_flick(&pid_shoot_change_barrel_spd, motor_shoot[change_barrel].speed_rpm, change_barr_t->change_barral_speed_tar);	
			}
			if(abs(motor_shoot[change_barrel].real_current) >= 5000 && abs(motor_shoot[change_barrel].speed_rpm) <= 200) //�ж϶�ת
			{
				change_barr_t->barrel_1_pos = motor_shoot[change_barrel].total_angle;
				change_barr_t->change_barral_pos_target = change_barr_t->barrel_1_pos;
				change_barr_t->change_barral_state_2_to_1 = OFF;
				change_barr_t->barrel_clock_time = 0;
				
			}
			 change_barr_t->change_barral_pos_target =  change_barr_t->barrel_1_pos ;
		}
	
     
		if(	change_barr_t->change_barral_state_1_to_2 == ON || change_barr_t->change_barral_state_2_to_1 == ON)
		{
			change_barr_t->change_barral_state = ON;
		}
		else
		{
			change_barr_t->change_barral_state = OFF;
		}

    if(change_barr_t->change_barral_state_2_to_1 == OFF && change_barr_t->change_barral_state_1_to_2 == OFF)
		{
      cascade_pid_ctrl_shoot( &pid_shoot_change_barrel_pos, 
														  &pid_shoot_change_barrel_spd,
														  motor_shoot[change_barrel].total_angle,
														  motor_shoot[change_barrel].speed_rpm,
														  change_barr_t->change_barral_pos_target); 	
		}
															
		change_barr_t->change_barral_motor_current = pid_shoot_change_barrel_spd.pos_out;	
	}

}



/**
 * ����ͳ��
 **/
shoot_recode_t shoot_recode = {

 .min = 30,
 .max = 0,
};

void Shoot_Speed_Test(void)
{	
		shoot_recode.pre_speed = shoot_recode.speed;
    shoot_recode.speed     = judge_sensor.info->shoot_data.bullet_speed;
	
		if(shoot_recode.pre_speed != shoot_recode.speed && !shoot_speed_change_flag)
		{
			
		  if(shoot_recode.speed<12 && shoot_recode.speed>=11)shoot_recode.shoot_speed.cnt_11++;
			if(shoot_recode.speed<13 && shoot_recode.speed>=12)shoot_recode.shoot_speed.cnt_12++;
			if(shoot_recode.speed<14 && shoot_recode.speed>=13)shoot_recode.shoot_speed.cnt_13++;
			if(shoot_recode.speed<15 && shoot_recode.speed>=14)shoot_recode.shoot_speed.cnt_14++;
			if(shoot_recode.speed<16 && shoot_recode.speed>=15)shoot_recode.shoot_speed.cnt_15++;	
			
		  if(shoot_recode.speed<17 && shoot_recode.speed>=16)shoot_recode.shoot_speed.cnt_16++;
			if(shoot_recode.speed<18 && shoot_recode.speed>=17)shoot_recode.shoot_speed.cnt_17++;
			if(shoot_recode.speed<19 && shoot_recode.speed>=18)shoot_recode.shoot_speed.cnt_18++;
			if(shoot_recode.speed<20 && shoot_recode.speed>=19)shoot_recode.shoot_speed.cnt_19++;
			if(shoot_recode.speed<21 && shoot_recode.speed>=20)shoot_recode.shoot_speed.cnt_20++;	
					
		  if(shoot_recode.speed<22 && shoot_recode.speed>=21)shoot_recode.shoot_speed.cnt_21++;
			if(shoot_recode.speed<23 && shoot_recode.speed>=22)shoot_recode.shoot_speed.cnt_22++;
			if(shoot_recode.speed<24 && shoot_recode.speed>=23)shoot_recode.shoot_speed.cnt_23++;
			if(shoot_recode.speed<25 && shoot_recode.speed>=24)shoot_recode.shoot_speed.cnt_24++;
			if(shoot_recode.speed<26 && shoot_recode.speed>=25)shoot_recode.shoot_speed.cnt_25++;	
				
			if(shoot_recode.speed<27 && shoot_recode.speed>=26)shoot_recode.shoot_speed.cnt_26++;
			if(shoot_recode.speed<28 && shoot_recode.speed>=27)shoot_recode.shoot_speed.cnt_27++;
			if(shoot_recode.speed<29 && shoot_recode.speed>=28)shoot_recode.shoot_speed.cnt_28++;
			if(shoot_recode.speed<30 && shoot_recode.speed>=29)shoot_recode.shoot_speed.cnt_29++;
			if(shoot_recode.speed>=30)shoot_recode.shoot_speed.cnt_30++;
					
			shoot_recode.cnt++;
			
			if(shoot_recode.cnt)
			{
				shoot_recode.sp_cnt += shoot_recode.speed;
				shoot_recode.pj   = shoot_recode.sp_cnt / shoot_recode.cnt;
			}
			
	    if(shoot_recode.speed > shoot_recode.max)
				shoot_recode.max = shoot_recode.speed;
			if(shoot_recode.speed < shoot_recode.min && shoot_recode.speed != 0)
				shoot_recode.min = shoot_recode.speed;
			
			shoot_recode.jicha = shoot_recode.max - shoot_recode.min;
			
		}
}


