#include "Auto_Aim.h"

extern Vision_Cmd_Id_t Vision_Cmd_Mode;

/*�Ӿ�ģʽ
	CMD_AIM_OFF 		    = 0x00,	// ����������
	CMD_AIM_AUTO		    = 0x01,	// ��������
	CMD_AIM_SMALL_BUFF	= 0x02,	  // ʶ��С��
	CMD_AIM_BIG_BUFF	  = 0x03,	// ʶ����
	CMD_AIM_ANTOP	   	= 0x04,	  // �����ڱ�
	CMD_AIM_ANDF		    = 0x05	  // �������

*/
uint8_t Vision_Mode = 1;


/*�Ƿ񵽴�λ��*/
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
/*�Ƿ�����Զ����*/
bool Auto_Shoot(void)
{
	Vision_Rx_Data_t *Pack = &vision_sensor.info->RxPacket.RxData;	
	
	if(Pack->zm_shoot_enable && vision_enter )
		return 1;
	else
		return 0;
}
/*�����Զ���״̬*/
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

/*�Ƿ��ҵ�Ŀ��*/
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
	//���豾����ȷ���Ǻ췽��������
	char Correct_Color = 3;	
	//��ȡ����ϵͳ�ҷ�ID
  char Our_id = judge_info.game_robot_status.robot_id;
	//�������ϵͳ����
	if(JUDGE_OFFLINE)
	{
		//���ң�����趨�з���ɫΪ��ɫ�����ҷ���ɫΪ��ɫ
		if(Enemy == RED)Correct_Color = BLUE;
		//���ң�����趨�з���ɫΪ��ɫ�����ҷ���ɫΪ��ɫ
		else if(Enemy == BLUE)Correct_Color = RED;
		//������������ң����������ȷ���Ǻ�ɫ������ɫ���Ͷ�һ�ѣ�50%�ĸ��ʡ�
		else 
		{		
			if(Color_Record != 3)Correct_Color = RED;		
		}
		
	}
	//�������ϵͳ����
	if(JUDGE_ONLINE)
	{
		//�������ϵͳ�ҷ�IDΪ3/4/5����Ϊ��������ID
		if(Our_id == 3 || Our_id == 4 || Our_id == 5)Correct_Color = RED;
		//�������ϵͳ�ҷ�IDΪ103/104/105����Ϊ�췽����ID
		else if(Our_id == 103 || Our_id == 104 || Our_id == 105)Correct_Color = BLUE;  
		//����ҷ���ȷ��ɫΪ��/��
		if(Correct_Color == BLUE || Correct_Color == RED)
		{ 
			//�����ɫ��¼���趨ֵһ������¼��ȷ��ɫ
			if(Color_Record == 3)Color_Record = Correct_Color;
			//�����ɫ��¼����ȷ��ɫ��һ������ɫͻȻ�������ģ����ٴμ�¼��ȷ��ɫ
			if(Color_Record != Correct_Color)Color_Record = Correct_Color;
		}
		//�����ȷ���ҷ���ȷ��ɫ�����������
		else
		{
	    //�����ɫ��¼���趨ֵ��һ��
			if(Color_Record != 3)Correct_Color = Color_Record;
			 //�����ɫ��¼���趨ֵһ��
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
