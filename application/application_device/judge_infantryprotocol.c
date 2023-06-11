/********************************************************************************
  * @file		judge_infantryprotocol.c
  * @version		V1.0.0
  * @date			
  * @brief   		
  *******************************************************************************/

#include "judge_infantryprotocol.h"
#include "vision_potocal.h"
#include "vision_sensor.h"
#include "judge_sensor.h"

#include "crc.h"
#include "string.h"
#include "uart_drv.h"
#include "usart.h"

#define JUDGE_FRAME_HEADER	0xA5  //此为帧头的SOF,帧头分为 SOF,length,Seq,CRC8


/**********偏移位置**********/
//帧字节（单位/字节）
enum judge_frame_offset_t {//帧头占五个字节，命令码占两个字节，从第七个开始为数据帧
	FRAME_HEADER	= 0,
	CMD_ID			= 5,
	DATA_SEG		= 7   
};
//帧头字节（单位/bit）
enum judge_frame_header_offset_t {//帧头的详细位置
	SOF			= 0,
	DATA_LENGTH	= 1,
	SEQ			= 3,
	CRC8		= 4
};//FRAME_HEADER

enum //车间通信
{
	//哨兵请求支援
	infantry3_Rescue_sentryID = 0x0201,
	infantry4_Rescue_sentryID = 0x0202,
	infantry5_Rescue_sentryID = 0x0203,
	hero_Rescue_sentryID      = 0x0204,
	engineer_Rescue_sentryID  = 0x0205,
};


void judge_sensor_init(judge_sensor_t *judge_sen)
{
	judge_sen->info->offline_cnt = judge_sen->info->offline_max_cnt+1;
	judge_sen->work_state= DEV_OFFLINE;
	
	if(judge_sen->id == DEV_ID_JUDGE) {
		judge_sen->errno = NONE_ERR;
	}
	else {
		judge_sen->errno = DEV_ID_ERR;
	}
}


void judge_sensor_update(judge_sensor_t *judge_sen, uint8_t *rxBuf)
{
	uint8_t  res = false;
	uint16_t frame_length;
	uint16_t cmd_id;
  uint16_t data_cmd_id;
	judge_info_t *judge_info = judge_sen->info;


	if( rxBuf == NULL )
	{
		judge_info->data_valid = false;
		return;
	}
	
	memcpy(&judge_info->fream_header, rxBuf, LEN_FRAME_HEAD);//5个字节
	
	/* 帧首字节是否为0xA5 */
	if(rxBuf[SOF] == JUDGE_FRAME_HEADER) 
	{
		
		/* 帧头CRC8校验 */
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true) 
		{
			/* 统计一帧的总数据长度，用于CRC16校验 */     // 长度两个字节
			frame_length = LEN_FRAME_HEAD + LEN_CMD_ID + judge_info->fream_header.data_length + LEN_FRAME_TAIL;
			
			if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == true)
			{
				res = true;
				
				cmd_id = (rxBuf[CMD_ID+1] << 8 | rxBuf[CMD_ID]);
				
				switch(cmd_id)
				{
					
					case ID_robot_interactive_header_data:
					{
						data_cmd_id = (rxBuf[DATA_SEG+1] << 8 | rxBuf[DATA_SEG]);
						switch (data_cmd_id)
						{
							case infantry3_Rescue_sentryID:
							case infantry4_Rescue_sentryID:
							case infantry5_Rescue_sentryID:
							case hero_Rescue_sentryID:
							case engineer_Rescue_sentryID:
								memcpy(&judge_info->sentry_Info_rescue,(rxBuf+DATA_SEG+LEN_CMD_ID),LEN_SENTRY_INFO);
							break;
						}
					}
					
					case ID_game_state: 
						memcpy(&judge_info->game_status, (rxBuf+DATA_SEG), LEN_GAME_STATUS);
						break;
					
					case ID_game_result: 
						memcpy(&judge_info->game_result, (rxBuf+DATA_SEG), LEN_GAME_RESULT);
						break;
					
					case ID_game_robot_HP: 
						memcpy(&judge_info->game_robot_HP, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_HP);
						break;//所有机器人的血量都可以获得
					
				
					case ID_ICRA_buff_debuff_zone_status:
						memcpy(&judge_info->ICRA_buff,(rxBuf+DATA_SEG),LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS);
						break;
					
					case ID_event_data: 
						memcpy(&judge_info->event_data, (rxBuf+DATA_SEG), LEN_EVENT_DATA);
						break;
					
					case ID_supply_projectile_action: 
						memcpy(&judge_info->supply_projectile_action, (rxBuf+DATA_SEG), LEN_SUPPLY_PROJECTILE_ACTION);
						break;
					
					case ID_referee_warning: 
						memcpy(&judge_info->referee_warning, (rxBuf+DATA_SEG), LEN_REFEREE_WARNING);
						break;
					
					case ID_dart_remaining_time: 
						memcpy(&judge_info->dart_remaining_time, (rxBuf+DATA_SEG), LEN_DART_REMAINING_TIME);
						break;
						
					case ID_game_robot_state: 
						memcpy(&judge_info->game_robot_status, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_STATUS);
						Determine_ID();
						break;
					
					case ID_power_heat_data: 
						memcpy(&judge_info->power_heat_data, (rxBuf+DATA_SEG), LEN_POWER_HEAT_DATA);
						break;
					
					case ID_game_robot_pos: 
						memcpy(&judge_info->game_robot_pos, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_POS);
						break;
					
					case ID_buff_musk: 
						memcpy(&judge_info->buff, (rxBuf+DATA_SEG), LEN_BUFF_MASK);
						break;
					
					case ID_aerial_robot_energy: 
						memcpy(&judge_info->aerial_robot_energy, (rxBuf+DATA_SEG), LEN_AERIAL_ROBOT_ENERGY);
						break;
					
					case ID_robot_hurt: 
						memcpy(&judge_info->robot_hurt, (rxBuf+DATA_SEG), LEN_ROBOT_HURT);
						break;
					
					case ID_shoot_data: 
						memcpy(&judge_info->shoot_data, (rxBuf+DATA_SEG), LEN_SHOOT_DATA);
						break;
					
					case ID_bullet_remaining: 
						memcpy(&judge_info->bullet_remaining, (rxBuf+DATA_SEG), LEN_BULLET_REMAINING);
						break;
					
					case ID_rfid_status: 
						memcpy(&judge_info->rfid_status, (rxBuf+DATA_SEG), LEN_RFID_STATUS);
						break;
						
					case ID_dart_client_directive:
						memcpy(&judge_info->dart_client,(rxBuf+DATA_SEG),LEN_DART_CLIENT_DIRECTIVE);
						break;
//					case ID_smallmap:
//						memcpy(&judge_info->dart_client,(rxBuf+DATA_SEG),LEN_DART_CLIENT_DIRECTIVE);
//						break;
					
				}
			}
		}
		
		/* 帧尾CRC16下一字节是否为0xA5 */
		if(rxBuf[frame_length] == JUDGE_FRAME_HEADER)
		{
			/* 如果一个数据包出现了多帧数据就再次读取 */
			judge_sensor_update( judge_sen, &rxBuf[frame_length] );
		}
	}
	
	judge_info->data_valid = res;
	if(judge_info->data_valid != true)
		judge_info->err_cnt++;
	
	// 接收到数据表示在线
	judge_info->offline_cnt = 0;
}
void Determine_ID(void)//判断自己是哪个队伍
{
	if(judge_info.game_robot_status.robot_id < 10)//本机器人的ID，红方
	{ 
		judge_info.ids.teammate_hero 		 	= 1;
		judge_info.ids.teammate_engineer  = 2;
		judge_info.ids.teammate_infantry3 = 3;
		judge_info.ids.teammate_infantry4 = 4;
		judge_info.ids.teammate_infantry5 = 5;
		judge_info.ids.teammate_plane		 	= 6;
		judge_info.ids.teammate_sentry		= 7;
		
		judge_info.ids.client_hero 		 	= 0x0101;
		judge_info.ids.client_engineer  = 0x0102;
		judge_info.ids.client_infantry3 = 0x0103;
		judge_info.ids.client_infantry4 = 0x0104;
		judge_info.ids.client_infantry5 = 0x0105;
		judge_info.ids.client_plane			= 0x0106;
		
		if     (judge_info.game_robot_status.robot_id == hero_red)//不断刷新放置在比赛中更改颜色
			judge_info.self_client = judge_info.ids.client_hero;
		else if(judge_info.game_robot_status.robot_id == engineer_red)
			judge_info.self_client = judge_info.ids.client_engineer;
		else if(judge_info.game_robot_status.robot_id == infantry3_red)
			judge_info.self_client = judge_info.ids.client_infantry3;
		else if(judge_info.game_robot_status.robot_id == infantry4_red)
			judge_info.self_client = judge_info.ids.client_infantry4;
		else if(judge_info.game_robot_status.robot_id == infantry5_red)
			judge_info.self_client = judge_info.ids.client_infantry5;
		else if(judge_info.game_robot_status.robot_id == plane_red)
			judge_info.self_client = judge_info.ids.client_plane;
	}
	else //蓝方
	{
		judge_info.ids.teammate_hero 		 	= 101;
		judge_info.ids.teammate_engineer  = 102;
		judge_info.ids.teammate_infantry3 = 103;
		judge_info.ids.teammate_infantry4 = 104;
		judge_info.ids.teammate_infantry5 = 105;
		judge_info.ids.teammate_plane		 	= 106;
		judge_info.ids.teammate_sentry		= 107;
		
		judge_info.ids.client_hero 		 	= 0x0165;
		judge_info.ids.client_engineer  = 0x0166;
		judge_info.ids.client_infantry3 = 0x0167;
		judge_info.ids.client_infantry4 = 0x0168;
		judge_info.ids.client_infantry5 = 0x0169;
		judge_info.ids.client_plane			= 0x016A;
		
		if     (judge_info.game_robot_status.robot_id == hero_blue)
			judge_info.self_client = judge_info.ids.client_hero;
		else if(judge_info.game_robot_status.robot_id == engineer_blue)
			judge_info.self_client = judge_info.ids.client_engineer;
		else if(judge_info.game_robot_status.robot_id == infantry3_blue)
			judge_info.self_client = judge_info.ids.client_infantry3;
		else if(judge_info.game_robot_status.robot_id == infantry4_blue)
			judge_info.self_client = judge_info.ids.client_infantry4;
		else if(judge_info.game_robot_status.robot_id == infantry5_blue)
			judge_info.self_client = judge_info.ids.client_infantry5;
		else if(judge_info.game_robot_status.robot_id == plane_blue)
			judge_info.self_client = judge_info.ids.client_plane;

	}

}
