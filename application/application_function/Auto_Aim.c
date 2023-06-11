#include "Auto_Aim.h"

extern Vision_Cmd_Id_t Vision_Cmd_Mode;

/*视觉模式
	CMD_AIM_OFF 		    = 0x00,	// 不启动自瞄
	CMD_AIM_AUTO		    = 0x01,	// 启动自瞄
	CMD_AIM_SMALL_BUFF	= 0x02,	  // 识别小符
	CMD_AIM_BIG_BUFF	  = 0x03,	// 识别大符
	CMD_AIM_ANTOP	   	= 0x04,	  // 击打哨兵
	CMD_AIM_ANDF		    = 0x05	  // 吊射基地

*/
uint8_t Vision_Mode = 1;


/*是否到达位置*/
bool ARRIVE(void)
{
  if(MOUSE_RIGH
		&& Find_Tar()
		&& abs(pid_pos[Gimbal_follow_motor_yaw_pos].err[2]) < 20
  	&& abs(pid_pos[Gimbal_follow_motor_pitch_pos].err[2]) < 20)
		return 1;
	else
		return 0;
}
/*是否可以自动射击*/
bool Auto_Shoot(void)
{
	Vision_Rx_Data_t *Pack = &vision_sensor.info->RxPacket.RxData;	
	
	if(Pack->zm_shoot_enable && vision_enter )
		return 1;
	else
		return 0;
}
/*处于自动的状态*/
State_t DF_STATE(void)
{
	Vision_Rx_Packet_t *Pack = &vision_sensor.info->RxPacket;
	
	if((Pack->FrameHeader.cmd_id == CMD_AIM_SMALL_BUFF || Pack->FrameHeader.cmd_id == CMD_AIM_BIG_BUFF || Pack->FrameHeader.cmd_id == CMD_AIM_ANDF))
		return ING;
	else
		return NO;
}

State_t ZM_STATE(void)
{
	Vision_Rx_Packet_t *Pack = &vision_sensor.info->RxPacket;
	
	if((Pack->FrameHeader.cmd_id == CMD_AIM_AUTO || Pack->FrameHeader.cmd_id == CMD_AIM_ANTOP))
		return ING;
	else
		return NO;
}

/*是否找到目标*/
bool Find_Tar(void)
{
	Vision_Rx_Data_t *Pack = &vision_sensor.info->RxPacket.RxData;	
	
	if(Pack->is_find_Dafu || Pack->is_find_Target)
		return 1;
	else
		return 0;
}

char Color_Record = 3;
char Get_Color(void)
{ 
	//假设本车不确定是红方还是蓝方
	char Correct_Color = 3;	
	//获取裁判系统我方ID
  char Our_id = judge_info.game_robot_status.robot_id;
	//如果裁判系统离线
	if(JUDGE_OFFLINE)
	{
		//如果遥控器设定敌方颜色为红色，则我方颜色为蓝色
		if(Enemy == RED)Correct_Color = BLUE;
		//如果遥控器设定敌方颜色为蓝色，则我方颜色为红色
		else if(Enemy == BLUE)Correct_Color = RED;
		//极端情况：如果遥控器不可以确定是红色还是蓝色，就赌一把，50%的概率。
		else 
		{		
			if(Color_Record != 3)Correct_Color = RED;		
		}
		
	}
	//如果裁判系统正常
	if(JUDGE_ONLINE)
	{
		//如果裁判系统我方ID为3/4/5，则为蓝方步兵ID
		if(Our_id == 3 || Our_id == 4 || Our_id == 5)Correct_Color = RED;
		//如果裁判系统我方ID为103/104/105，则为红方步兵ID
		else if(Our_id == 103 || Our_id == 104 || Our_id == 105)Correct_Color = BLUE;  
		//如果我方正确颜色为红/蓝
		if(Correct_Color == BLUE || Correct_Color == RED)
		{ 
			//如果颜色记录与设定值一样，记录正确颜色
			if(Color_Record == 3)Color_Record = Correct_Color;
			//如果颜色记录与正确颜色不一样（颜色突然发生更改），再次记录正确颜色
			if(Color_Record != Correct_Color)Color_Record = Correct_Color;
		}
		//如果不确定我方正确颜色（极端情况）
		else
		{
	    //如果颜色记录与设定值不一样
			if(Color_Record != 3)Correct_Color = Color_Record;
			 //如果颜色记录与设定值一样
			else
			{
				if(Enemy == RED)Correct_Color = 1;
				else if(Enemy == BLUE)Correct_Color = 0;				
			}
		}
	}
	return Correct_Color;
}


uint8_t Get_Speed_Limit(void)
{
	uint8_t limit;
	
	if(JUDGE_ONLINE)
	{
		limit = SHOOT_SPEED_LIMIT_1;
		
	}
	else
	{
		switch((uint16_t)friction_speed_tar)
		{
			case Fri_15:
			limit = 15;	
			break;
		
			case Fri_18:
			limit = 18;	
			break;

			case Fri_20:
			limit = 20;	
			break;

			case Fri_22:
			limit = 22;	
			break;		

			case Fri_30:
			limit = 30;	
			break;				
		}
	}
	return limit;
}
