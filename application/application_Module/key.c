/*
*	  �����ж����Ӧ����
*	  ����ң����
*/

#include "key.h"
#include "Auto_Aim.h"
#include "judge_sensor.h"

uint8_t Enemy;
void ENEMY(void)
{
	if(SW2_UP)  Enemy = RED;
	if(SW2_MID) Enemy = BLUE;
	if(SW2_DOWN)Enemy = AUTO;
}
/*====================================================*/
/*===================�Զ���λ=========================*/
/*
 * ��ĳЩ�����������Զ���λ
	 �����ֹ��λ����������δ�������������
		
		��ָ����ģʽ��
		����������������ģʽ�µ�λ ��굥��
		�����������������ģʽ�µ�λ ��곤��	
*/

/*-�ƶ�ģʽ��ֹ��λ������������С���ݺͻ�еģʽ��-*/
void Move_Reset(void)
{
//	if(State.Move.state.CHAS_FIRST == NO && State.Move.state.TOP == NO)
//		
//	 State.move_mode(&State.Move, GIMB_FIRST);
}

/*�����Զ���λ*/
void Auto_Reset(void)
{
	if(!MOUSE_RIGH  && !KEY_Z && !KEY_X)
		{
		
		Vision_Mode = CMD_AIM_AUTO;	
		
	}
}

/*====================�Զ���λ=======================*/
/*====================================================*/


		
char single_shoot_flag = 0;
char Key_R_Fast_Shoot = 0;
char Shoot_Permit = 0;
extern char Rx_Cmd_id;
extern int16_t Auto_Shoot_Flag;
extern char Buff_Hand_Shoot_flag;

//���
void Key_Mouse_L(void)//
{
  KEY_State_Judge(&Keyboard.Mouse_L ,MOUSE_LEFT, CHANGE_TIM , LONG_CHANGE_TIM_MOUSE_L); 
	
  switch(Keyboard.Mouse_L.State)
  {
    case UP:
	  	Auto_Shoot_Flag = 0;
		  single_shoot_flag = OFF;	
		
		  if(Key_R_Fast_Shoot == OFF)
			{
		   shoot_flag.flick_flag_top = OFF;	
			}
		
			if(shoot_flag.friction_flag_top == OFF)
			{
		    Shoot_Permit = OFF; 
			}			
			 if(shoot_flag.friction_flag_top == ON)
			{
		    Shoot_Permit = ON; 
			}
    
			
      break;
    case PRESS:	
			/*����������ص���*/
			shoot_flag.cover_flag = OFF;
		
			break;
    case SHORT_DOWN:
			 /*���ֱ�ӽ����ֶ����*/
		
			Buff_Hand_Shoot_flag = 1;
		
		   /*�����Ħ����*/
		  if(Shoot_Permit == OFF)
			{
		    shoot_flag.friction_flag_top = ON;
			}	
			
			if(Shoot_Permit == ON  && !STOP_SHOOT_1 && motor_shoot[R_friction].speed_rpm >= 3000  && Key_R_Fast_Shoot == OFF && Auto_Shoot_Flag == 0)	
     {
			  single_shoot_flag = ON;			
			}
      break;  
    case DOWN:
      
		  single_shoot_flag = OFF;
			if(Shoot_Permit == ON  && !STOP_SHOOT_1 && motor_shoot[R_friction].speed_rpm >= 3000 &&  Rx_Cmd_id!= 10 && Key_R_Fast_Shoot == OFF && Auto_Shoot_Flag == 0)
			{				 			 	
				 shoot_flag.flick_flag_top = ON;				
			}		
		
			if( Shoot_Permit == ON  && !STOP_SHOOT_1 && motor_shoot[R_friction].speed_rpm >= 3000 && Auto_Shoot() && Rx_Cmd_id == 10 && Key_R_Fast_Shoot == OFF && Auto_Shoot_Flag == 0 )
			{
				Auto_Shoot_Flag = 1;		
			}

			else
			{
				Auto_Shoot_Flag = 0;
			}		
		
			if(STOP_SHOOT_1)
			{
			  shoot_flag.flick_flag_top = OFF;			
			}
      break;
    case RELAX:			
      break;
  }
}
//����
void Key_Mouse_R(void)//
{
  KEY_State_Judge(&Keyboard.Mouse_R ,MOUSE_RIGH, CHANGE_TIM , LONG_CHANGE_TIM_MOUSE_R); 
  switch(Keyboard.Mouse_R.State)
  {
    case UP:
      break;
    case PRESS:		
			
				Vision_Mode = CMD_AIM_AUTO;			
				
      break;
    case SHORT_DOWN:
      break;  
    case DOWN:
      break;
    case RELAX:
      break;
  }
}

/***=================================================***/

void Key_W(void)
{
  KEY_State_Judge(&Keyboard.W ,KEY_W , CHANGE_TIM , LONG_CHANGE_TIM_W);
  switch(Keyboard.W.State)
  {
    case UP:			
      break;
    case PRESS:
      break;
    case SHORT_DOWN:			
      break;
    case DOWN:
      break;
    case RELAX:
      break;
  }
}

void Key_S(void)
{
  KEY_State_Judge(&Keyboard.S ,KEY_S, CHANGE_TIM , LONG_CHANGE_TIM_S);
  switch(Keyboard.S.State)
  {
    case UP:
      break;
    case PRESS:
      break;
    case SHORT_DOWN:
      break;
    case DOWN:
      break;
    case RELAX:
      break;
  }
}

void Key_D(void)
{
  KEY_State_Judge(&Keyboard.D ,KEY_D, CHANGE_TIM , LONG_CHANGE_TIM_D);
  switch(Keyboard.D.State)
  {
    case UP:
      break;
    case PRESS:
      break;
    case SHORT_DOWN:
      break;
    case DOWN:
      break;
    case RELAX:		
      break;
  }
}

void Key_A(void)
{
  KEY_State_Judge(&Keyboard.A ,KEY_A, CHANGE_TIM , LONG_CHANGE_TIM_A);
  switch(Keyboard.A.State)
  {
    case UP:
      break;
    case PRESS:
      break;
    case SHORT_DOWN:
      break;
    case DOWN:
      break;
    case RELAX:	
      break;
  }
}
/*����ֵתΪң��ͨ��ֵ end*/



/*���ܼ� begin*/
int16_t KEY_Q_Speed_z = 0;
void Key_Q(void)//
{
	if(abs(pid_pos[Gimbal_follow_motor_yaw_pos].err[NOW]) < 5 && Q_FLAG)Q_FLAG = 0;
	
  KEY_State_Judge(&Keyboard.Q ,KEY_Q, CHANGE_TIM , LONG_CHANGE_TIM_Q);
  switch(Keyboard.Q.State)
  {
    case UP: //̧��״̬
			KEY_Q_Speed_z = 0;
      break;
    case PRESS: //����˲��
			Q_FLAG = !Q_FLAG;
			E_FLAG = 0;
			C_FLAG = 0;			
	
			if(Q_FLAG)
			{
				Gimbal_angle.angle_target_follow_yaw += 90; 
				//�����ƶ���־λ
				if(MODE_MAC_OR_TOP == MODE_MAC)
				{
					KEY_Q_Speed_z = 1;
				}
			}
			
			else
				Gimbal_angle.angle_target_follow_yaw = yaw;
      break;
    case SHORT_DOWN: //�̰�			
      break;
    case DOWN: //����
      break;
    case RELAX: //�ɿ�˲��
      break;
  }
}

int16_t KEY_E_Speed_z = 0;
void Key_E(void)//
{
	if(abs(pid_pos[Gimbal_follow_motor_yaw_pos].err[NOW]) < 5 && E_FLAG)E_FLAG = 0;
	
  KEY_State_Judge(&Keyboard.E ,KEY_E, CHANGE_TIM , LONG_CHANGE_TIM_E);
  switch(Keyboard.E.State)
  {
    case UP: //̧��״̬
			KEY_E_Speed_z = 0;
      break;
    case PRESS: //����˲��
			Q_FLAG = 0;
			E_FLAG = !E_FLAG;
			C_FLAG = 0;
		
			if(E_FLAG)
			{
				Gimbal_angle.angle_target_follow_yaw -= 90; 
				//�����ƶ���־λ
				if(MODE_MAC_OR_TOP == MODE_MAC)
				{
					KEY_E_Speed_z = 1;
				}
			}
			else
				Gimbal_angle.angle_target_follow_yaw = yaw;
			
      break;
    case SHORT_DOWN: //�̰�			
      break;
    case DOWN: //����
      break;
    case RELAX: //�ɿ�˲��
      break;
  }
}

char fast_180_flag = 0;
void Key_C(void)//
{
	if(abs(pid_pos[Gimbal_follow_motor_yaw_pos].err[NOW]) < 5 && C_FLAG)C_FLAG = 0;
	
  KEY_State_Judge(&Keyboard.C ,KEY_C, CHANGE_TIM , LONG_CHANGE_TIM_C);
  switch(Keyboard.C.State)
  {
 		case UP:				
      break;
    case PRESS:
			Q_FLAG = 0;
			E_FLAG = 0;
			C_FLAG = !C_FLAG;
		
			if(C_FLAG)
			{
				Gimbal_angle.angle_target_follow_yaw -=  180 ;	
        fast_180_flag = 1;				
			}
			else
				Gimbal_angle.angle_target_follow_yaw = yaw;
      break;
    case SHORT_DOWN:
      break;
    case DOWN:
      break;
    case RELAX:
      break;
  }
}



//////////////////////////////////////////////
char PressF = 0;
void Key_F(void)//
{
  KEY_State_Judge(&Keyboard.F ,KEY_F, CHANGE_TIM , LONG_CHANGE_TIM_F);
  switch(Keyboard.F.State)
  {
    case UP: //̧��״̬

      break;
    case PRESS: //����˲��
			
			F_FLAG++;
      break;
    case SHORT_DOWN: //�̰�
			if( Small_Top_Mode_Flag == Little_TOP_OFF)		
			{				
		    Small_Top_Mode_Flag = Little_TOP_ON;
				Small_Top_Mode = SMALL_TOP_MODE_FAST;
	
			}
      break;
    case DOWN: //����
      break;
    case RELAX: //�ɿ�˲��
      break;
  }
}



char slow_move_flag = 0;
void Key_B(void)
{
  KEY_State_Judge(&Keyboard.B ,KEY_B, CHANGE_TIM , LONG_CHANGE_TIM_B);
  switch(Keyboard.B.State)
  {
    case UP:				
      break;
    case PRESS:		
			B_FLAG++;			

      break;
    case SHORT_DOWN:
			if(MODE == MODE_KEY && MODE_MAC_OR_TOP == MODE_TOP)
			{
				Shoot_Permit = OFF; 
			  shoot_flag.cover_flag = 1;
				shoot_flag.friction_flag_top	= 0;	
			}
      break;
    case DOWN:
			if(MODE == MODE_KEY && MODE_MAC_OR_TOP == MODE_TOP)
			{
			  shoot_flag.cover_flag = 0;
			}
			
      break;
    case RELAX:
      break;
  }
}

uint32_t r_time1 = 0;

void Key_R(void)//����������
{
  KEY_State_Judge(&Keyboard.R ,KEY_R, CHANGE_TIM , LONG_CHANGE_TIM_R);
  switch(Keyboard.R.State)
  {
    case UP:
      break;
    case PRESS:
			shoot_flag.cover_flag = OFF;//�ص���
      break;
    case SHORT_DOWN:	
			
			if(Key_R_Fast_Shoot == OFF)
			{
				Key_R_Fast_Shoot = ON;
				shoot_flag.friction_flag_top	= ON;  //�̰�����Ħ����
			}
			  
      break;
    case DOWN:
			Key_R_Fast_Shoot = OFF;
		  shoot_flag.flick_flag_top = OFF;   //�رղ�����
      break;
    case RELAX:
      break;
  }
}

extern float usTms; //���ʱ
void Key_X(void)
{
  KEY_State_Judge(&Keyboard.X ,KEY_X, CHANGE_TIM , LONG_CHANGE_TIM_X);
  switch(Keyboard.X.State)
  {
    case UP:		
      break;
			
    case PRESS:
			usTms = 0; //���ʱ
			  Buff_Hand_Shoot_flag = 0;
		   shoot_flag.friction_flag_top = ON;		
				Vision_Mode = CMD_AIM_SMALL_BUFF;			
      break;
    case SHORT_DOWN:
      break;
    case DOWN:
      break;
    case RELAX:
      break;
  }
}

void Key_Z(void)//
{
  KEY_State_Judge(&Keyboard.Z ,KEY_Z, CHANGE_TIM , LONG_CHANGE_TIM_Z);
  switch(Keyboard.Z.State)
  {
    case UP:	
      break;
			
    case PRESS:
			  usTms = 0; //���ʱ
			  Buff_Hand_Shoot_flag = 0;
	    	shoot_flag.friction_flag_top = ON;		
				Vision_Mode = CMD_AIM_BIG_BUFF;				
      break;
    case SHORT_DOWN:
      break;
    case DOWN:
      break;
    case RELAX:
      break;
  }
}

void Key_G(void)
{
  KEY_State_Judge(&Keyboard.G ,KEY_G, CHANGE_TIM , LONG_CHANGE_TIM_G);
  switch(Keyboard.G.State)
  {
    case UP:			
      break;
			
    case PRESS:	

      break;
    case SHORT_DOWN:	
     		
      break;
    case DOWN:
      break;
    case RELAX:
      break;
  }
}


char TURN_NORMAL = 0;
char TURN_45_FLAG = 1;//��ʼ������Ϊ1����ֹctrl��
char TURN_45 = 0;
uint16_t change_target_flag;
uint16_t change_target;
void Key_V(void)
{
  KEY_State_Judge(&Keyboard.V ,KEY_V, CHANGE_TIM , LONG_CHANGE_TIM_V);
  switch(Keyboard.V.State)
  {
		case UP:		
			if(vision_enter_rc == 0)
			{
			  change_target_flag = 0;		
			}
      break;
    case PRESS:
//			if(MODE_MAC_OR_TOP == MODE_TOP)
//			{
//				V_FLAG++;			
//				TURN_45 = !TURN_45;	    				
//			}
      break;
    case SHORT_DOWN:
			 if(change_target_flag == 0)
				{
					 change_target += 1;	
					 change_target_flag = 1;				
				}			
//      if(TURN_45_FLAG == 1 && MODE_MAC_OR_TOP == MODE_TOP)
//			{
//		  	angle_target_gyro_now = Limit_Target((float)(angle_target_gyro_now - TURN_45_MAC_ANGLE_VALUE));	
//  			TURN_45_FLAG = 0; //V����һ��֮��V����ʧЧ
//			}	
//			
      break;
    case DOWN:
			
			 	
      break;
    case RELAX:
      break;
  }
}



void Key_ctrl(void)//
{
  KEY_State_Judge(&Keyboard.CTRL ,KEY_CTRL, CHANGE_TIM , LONG_CHANGE_TIM_CTRL);
  switch(Keyboard.CTRL.State)
  {
    case UP:
      break;
    case PRESS:
			if(shoot_flag.cover_flag == 1)//�ص���
			{
				shoot_flag.cover_flag = 0;
			}
			else if(MODE_MAC_OR_TOP == MODE_MAC)
			{
        Gimbal_angle.angle_mac_follow = CHASSIS_MAC_FRONT_MID_VALUE;	
			}			
			shoot_flag.flick_flag_top = OFF; 

      break;
    case SHORT_DOWN:
						
			MODE_MAC_OR_TOP = MODE_TOP;
		  Small_Top_Mode_Flag = Little_TOP_OFF;//�ر�С����
		
			if(TURN_45_FLAG == 0)
			{
				angle_target_gyro_now = Limit_Target((float)(angle_target_gyro_now + TURN_45_MAC_ANGLE_VALUE));
        TURN_45_FLAG = 1;				
			}
				
		
			break;
    case DOWN:
			
      break;
    case RELAX:			
      break;
  }
}

char High_Spd = 0;
char Low_Spd = 1;

void Key_SHIFT(void)
{
  KEY_State_Judge(&Keyboard.SHIFT ,KEY_SHIFT, CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);
  switch(Keyboard.SHIFT.State)
  {
    case UP:
			High_Spd = 0;
		  Low_Spd  = 1;
      break;
    case PRESS:
			High_Spd = 1;
		  Low_Spd  = 0;
      break;
    case SHORT_DOWN:
			
      break;  
    case DOWN:

      break;
    case RELAX:
      break;
  }
}
/*���ܼ� end*/


void KB_CTRL(void)
{
		if(SW2_MID && TW_VALUE < -300)NVIC_SystemReset();
	
	  Key_Channel_Update();//����ģ��ͨ��ֵ
	  
		Key_Q();
		Key_E();
		Key_C();
		
		Key_B();
		Key_V();	
	
		Key_F();
		
		Key_Mouse_L();
		Key_Mouse_R();
		Key_Z();	
		Key_X();	
		Key_G();	
		
	  if(KEY_Z && KEY_X)Vision_Mode = CMD_AIM_ANDF;
	  if(KEY_CTRL && KEY_G) MODE_MAC_OR_TOP = MODE_MAC;
	
		Key_ctrl();
	  
		Key_SHIFT();

		Key_R();

		Move_Reset();
		
		if(vision_enter_rc == 0)
		{
		  Auto_Reset();
		}
}
