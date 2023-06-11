/* 
*	   ʧ������Լ�����
*/
#include "Offline_check.h"

int16_t Offline_flag = 1;
ALL_Reset_t ALL_Reset;
//��¼˯��ʱ��
sleep_t RC_Sleep = 
{
	.time = 0,
};

void All_Reset_Judge(ALL_Reset_t *ALL_Reset)
{
	ALL_Reset->KeyZ_ON = rc_info.kb.bit.Z;
	ALL_Reset->KeyX_ON = rc_info.kb.bit.X;
	ALL_Reset->KeyCtrl_ON = rc_info.kb.bit.CTRL;
}
	
void Sleep_Mode(void)
{
	Chassis_Motor_Stop();
}
/*
	ң����ʧ���¿�����˯��ģʽ
	2���������е��ͣת 4���ж��
*/
void RC_Offline_Sleep(void)
{
	if(RC_OFFLINE)
	{	
	
			COVER_SLEEP();				
		
		//��ʼ���Ƚ����еģʽ
		MODE_MAC_OR_TOP = MODE_MAC;
		//��̨λ�ó�ʼ����־λ ��1	
		Offline_flag = ON;
    //���������־λ����
    shoot_flag_init(&shoot_flag,&change_barr);
		//С����ģʽ��־λ�ر�
		Small_Top_Mode_Flag = OFF;
		//���²������ϵ�Ŀ��Ƕ�
		flick_pos_target =  motor_shoot[flick].total_angle;
		//ң�����Ӿ���־λ����
		
		vision_enter_rc = OFF;
		
		

			
	}
	else if(RC_ONLINE)
	{
		ENEMY();
		RC_Sleep.time = HAL_GetTick();
		All_Reset_Judge(&ALL_Reset);
		if(ALL_Reset.KeyZ_ON && ALL_Reset.KeyX_ON && ALL_Reset.KeyCtrl_ON)
		{
			System_Reset();
		}
		//�����κ�ģʽ�����û�еģʽ����̨��λ		
		//ģʽ״̬�л�
		if(SW1_MID && vision_enter_rc ==OFF)
		{
			 MODE = MODE_RC;
			 MODE_MAC_OR_TOP = MODE_TOP;								
		}
		else if(SW1_DOWN && vision_enter_rc ==OFF)
		{
			MODE = MODE_RC;
			MODE_MAC_OR_TOP = MODE_MAC;		
		}		
		else if(SW1_UP && vision_enter_rc ==OFF)
		{
			//�������ģʽ
      MODE = MODE_KEY;
		}

	}
}


//ң��ʧ����λ������״̬��ʼ��
void rc_offline_handle(void)
{
	RC_ResetData(&rc);
	motor_offline_handle();
}
void motor_offline_handle(void)
{
	pid_spd[Left_front_spd].pos_out = 0;
	pid_spd[Right_front_spd].pos_out = 0;
	pid_spd[Left_back_spd].pos_out = 0;
	pid_spd[Right_back_spd].pos_out = 0;
	pid_spd[Gimbal_yaw].pos_out = 0;
	pid_spd[Gimbal_pitch].pos_out=0;
//	pid_shoot_left_friction_wheel_spd.pos_out = 0;
//	pid_shoot_right_friction_wheel_spd.pos_out = 0;
//	flick_motor_current = 0;//������Can��0
	
	
	
	CAN_cmd_chassis ( pid_spd[Left_front_spd].pos_out, 
										pid_spd[Right_front_spd].pos_out,
										pid_spd[Left_back_spd].pos_out,
										pid_spd[Right_back_spd].pos_out);
										
	CAN_cmd_gimbal(pid_spd[Gimbal_yaw].pos_out, 
								 pid_spd[Gimbal_pitch].pos_out,
								 0, 
								 0);	
  
//	//CAN_cmd_shoot( pid_shoot_left_friction_wheel_spd.pos_out,
//								 pid_shoot_right_friction_wheel_spd.pos_out,
//								 flick_motor_current,
//								 0);							 
}

//�����λ����
void System_Reset(void) 
{
   __set_FAULTMASK(1); //�ر������ж�
    NVIC_SystemReset(); //���������λ
}

uint16_t current_led_cnt;

void offline_LED_UP(void)
{
	
		if((motor_offline_check.chassis_motor_offline->offline_cnt_L_F < motor_offline_check.chassis_motor_offline->offline_cnt_max)
			&&(motor_offline_check.chassis_motor_offline->offline_cnt_L_B < motor_offline_check.chassis_motor_offline->offline_cnt_max)
		  &&(motor_offline_check.chassis_motor_offline->offline_cnt_R_F < motor_offline_check.chassis_motor_offline->offline_cnt_max)
		  &&(motor_offline_check.chassis_motor_offline->offline_cnt_R_B < motor_offline_check.chassis_motor_offline->offline_cnt_max)
			&&(motor_offline_check.gimbal_motor_offline->offline_cnt_Pitch < motor_offline_check.gimbal_motor_offline->offline_cnt_max)
	  	&&(motor_offline_check.gimbal_motor_offline->offline_cnt_Yaw < motor_offline_check.gimbal_motor_offline->offline_cnt_max)
	   	&&((motor_offline_check.shoot_motor_offline->offline_cnt_L || motor_offline_check.shoot_motor_offline->offline_cnt_R)<motor_offline_check.shoot_motor_offline->offline_cnt_max)
		  &&(motor_offline_check.shoot_motor_offline->offline_cnt_f < motor_offline_check.shoot_motor_offline->offline_cnt_max)
		  )
	{
		 current_led_cnt++;
		if(current_led_cnt < 100)
		{
			 HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		}
		else if(current_led_cnt >= 100 && current_led_cnt <200)
		{
			 HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		}
		else if(current_led_cnt >= 200 && current_led_cnt < 300)
		{
			 HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		}
		else if(current_led_cnt >= 300 && current_led_cnt < 400
			
		)
		{
			 HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
		}
		else
		{
			current_led_cnt = 0;
		}
	}
	
	if((motor_offline_check.chassis_motor_offline->offline_cnt_L_F >= motor_offline_check.chassis_motor_offline->offline_cnt_max)
			&&(motor_offline_check.chassis_motor_offline->offline_cnt_L_B >= motor_offline_check.chassis_motor_offline->offline_cnt_max)
		  &&(motor_offline_check.chassis_motor_offline->offline_cnt_R_F >= motor_offline_check.chassis_motor_offline->offline_cnt_max)
		  &&(motor_offline_check.chassis_motor_offline->offline_cnt_R_B >= motor_offline_check.chassis_motor_offline->offline_cnt_max))
	{
		   HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	}
	else
	{
		motor_offline_check.chassis_motor_offline->offline_cnt_L_F ++;
		motor_offline_check.chassis_motor_offline->offline_cnt_L_B ++;
		motor_offline_check.chassis_motor_offline->offline_cnt_R_F ++;
		motor_offline_check.chassis_motor_offline->offline_cnt_R_B ++;

  }
	
	if((motor_offline_check.gimbal_motor_offline->offline_cnt_Pitch >= motor_offline_check.gimbal_motor_offline->offline_cnt_max)
		&&(motor_offline_check.gimbal_motor_offline->offline_cnt_Yaw >= motor_offline_check.gimbal_motor_offline->offline_cnt_max))
	{
		   HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	}
	else
	{
		motor_offline_check.gimbal_motor_offline->offline_cnt_Pitch ++;
		motor_offline_check.gimbal_motor_offline->offline_cnt_Yaw ++;

  }
	
	if((motor_offline_check.shoot_motor_offline->offline_cnt_L || motor_offline_check.shoot_motor_offline->offline_cnt_R)>= motor_offline_check.shoot_motor_offline->offline_cnt_max)
	{
		   HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	}
	else
	{
		motor_offline_check.shoot_motor_offline->offline_cnt_L++;
		motor_offline_check.shoot_motor_offline->offline_cnt_R++;
  }
	
	if(motor_offline_check.shoot_motor_offline->offline_cnt_f >= motor_offline_check.shoot_motor_offline->offline_cnt_max)
	{
		   HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	}
	else
	{
		motor_offline_check.shoot_motor_offline->offline_cnt_f++;
  }
	if(JUDGE_OFFLINE)
	{
		   HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	}
	
}

void offline_LED_DOWN(void)
{	
		if(CAP_RP_2023.offline_cnt < CAP_RP_2023.offline_max_cnt)
		{
			current_led_cnt++;
			if(current_led_cnt < 100)
			{
				 HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
			}
			else if(current_led_cnt >= 100 && current_led_cnt <200)
			{
				 HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
			}
			else if(current_led_cnt >= 200 && current_led_cnt < 300)
			{
				 HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
			}
			else if(current_led_cnt >= 300 && current_led_cnt < 400)
			{
				 HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
			}
			else
			{
				current_led_cnt = 0;
			}
		}
		else
		{
			   HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
		}
	
	
	
	
}	
