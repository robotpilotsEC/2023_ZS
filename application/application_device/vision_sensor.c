/**
 * @file        vision_sensor.c
 * @author      RobotPilots@2022
 * @Version     V1.0
 * @date        
 * @brief       About the Pathway.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "rc.h"
#include "imu.h"
#include "rp_math.h"
#include "vision_sensor.h"
#include "vision_potocal.h"
#include "Gimbal_motor.h"
#include "zs_rp_config.h"
#include "Auto_Aim.h"

extern UART_HandleTypeDef huart3;
extern Gimbal_angle_t Gimbal_angle;

#define VISION_TX_BUF_LEN LEN_VISION_TX_PACKET
uint8_t vision_dma_txbuf[VISION_TX_BUF_LEN];

extern void vision_update(vision_sensor_t *vision, uint8_t *rxBuf);
extern void vision_init(vision_sensor_t *vision);
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void vision_check(vision_sensor_t *vision);
void vision_heart_beat(vision_sensor_t *vision);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 视觉驱动
drv_uart_t	vision_sensor_driver = {
	.type = DRV_UART1,
//	.tx_byte =  UART1_SendData,
};

// 视觉信息
vision_info_t 	vision_sensor_info = {
	.State.offline_max_cnt = 200,
};

// 视觉传感器
vision_sensor_t	vision_sensor = {
	.info = &vision_sensor_info,
	.init = vision_init,
	.update = vision_update,
	.check = vision_check,
	.heart_beat = vision_heart_beat,
	.work_state = DEV_OFFLINE,
	.id = DEV_ID_VISION	,
};
/* Private functions ---------------------------------------------------------*/
extern Vision_Cmd_Id_t AUTO_CMD_MODE;
char Rx_Cmd_id;
static void vision_check(vision_sensor_t *vision)
{

	Vision_Rx_Data_t *Pack = &vision_sensor.info->RxPacket.RxData;

	memcpy(&vision_sensor.info->RxPacket,vision_sensor.info->RxPacket.RxData.data, LEN_VISION_RX_PACKET);
	
	if(Pack->pitch_angle == NULL)Pack->pitch_angle = pitch;  
	if(Pack->yaw_angle   == NULL)Pack->yaw_angle   = yaw;
	
	if(Pack->pitch_angle <= 8191 || Pack->pitch_angle >= 0)Pack->pitch_angle = Pack->pitch_angle;
	else                                                   Pack->pitch_angle = pitch;
	if(Pack->yaw_angle   <= 8191 || Pack->yaw_angle   >= 0)Pack->yaw_angle   = Pack->yaw_angle  ;
	else                                                   Pack->yaw_angle   = yaw;
	
	if(!Pack->is_find_Target && !Pack->is_find_Dafu)
	{
		Pack->pitch_angle = pitch; 
		Pack->yaw_angle   = yaw;
	}
	
	//视觉模式下一直更新陀螺仪模式的云台目标值
	if(vision_enter == ON)
	{
	  Gimbal_angle.angle_target_follow_yaw = yaw ; 
    Gimbal_angle.angle_target_follow_pitch = pitch; 
	}
	
	if(VISION_OFFLINE)
	{
		Pack->pitch_angle = pitch; 
		Pack->yaw_angle   = yaw;
	}
	Rx_Cmd_id = vision_sensor.info->RxPacket.FrameHeader.cmd_id;
}


extern uint16_t change_target;
void Vision_Get(void)
{
	float vision_yaw;
	float vision_pitch;
	vision_yaw = (yaw+180)*22.7555f;
	vision_pitch = (pitch+180)*22.7555f;
	vision_sensor.info->TxPacket.TxData.yaw        = vision_yaw;
	vision_sensor.info->TxPacket.TxData.pitch      = vision_pitch;
	vision_sensor.info->TxPacket.TxData.fric_speed = Get_Speed_Limit();//SHOOT_SPEED_LIMIT_1;
	vision_sensor.info->TxPacket.TxData.my_color   = Get_Color();//0红色 1蓝色 
	vision_sensor.info->TxPacket.TxData.blood_0 = 20; //切换目标
	vision_sensor.info->TxPacket.TxData.blood_1 = 30; //切换目标
	vision_sensor.info->TxPacket.TxData.blood_2 = 40; //切换目标
	vision_sensor.info->TxPacket.TxData.blood_3 = 50; //切换目标
	vision_sensor.info->TxPacket.TxData.blood_4 = 60; //切换目标
	vision_sensor.info->TxPacket.TxData.blood_5 = 70; //切换目标
	vision_sensor.info->TxPacket.TxData.blood_6 = 80; //切换目标
	vision_sensor.info->TxPacket.TxData.blood_7 = 90; //切换目标
	vision_sensor.info->TxPacket.TxData.blood_8 = 100; //切换目标
	vision_sensor.info->TxPacket.TxData.is_change_target= change_target; //切换目标
	vision_dma_txbuf[sof] = VISION_FRAME_HEADER;
	vision_dma_txbuf[Cmd_ID] = Vision_Mode;	//键盘和遥控器部分还没写
	
}
void Vision_TX(void)
{
	//Cmd_Last = vision_sensor.info->TxPacket.FrameHeader.cmd_id;
	vision_sensor.info->TxPacket.FrameHeader.sof    = vision_dma_txbuf[sof];
	vision_sensor.info->TxPacket.FrameHeader.cmd_id = (Vision_Cmd_Id_t)vision_dma_txbuf[Cmd_ID];
	vision_sensor.info->TxPacket.FrameHeader.crc8   = vision_dma_txbuf[Crc8];
	
	Append_CRC8_Check_Sum(vision_dma_txbuf, LEN_FRAME_HEADER);
	
	memcpy(&vision_dma_txbuf[Data], &vision_sensor.info->TxPacket.TxData, LEN_TX_DATA);
	
	Append_CRC16_Check_Sum(vision_dma_txbuf, LEN_VISION_TX_PACKET);
	
	if(!USART_TEST){
		if(( POSITION_N == 1))
		{	
			HAL_UART_Transmit_DMA(&huart3, vision_dma_txbuf, LEN_VISION_TX_PACKET);		
		}		
		
  }
}

void vision_heart_beat(vision_sensor_t *vision_sen)
{
	vision_info_t *vision_info = vision_sen->info;

	vision_info->State.offline_cnt++;
	if(vision_info->State.offline_cnt > vision_info->State.offline_max_cnt) 
	{
		vision_info->State.offline_cnt = vision_info->State.offline_max_cnt;
		vision_sen->work_state = DEV_OFFLINE;
	} 
	else 
	{
		/* 离线->在线 */
		if(vision_sen->work_state == DEV_OFFLINE)
			vision_sen->work_state = DEV_ONLINE;
	}	
	
}
/* Exported functions --------------------------------------------------------*/
