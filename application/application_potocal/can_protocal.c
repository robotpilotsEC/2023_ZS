/**
 * @file        can_potocal.c
 * @author      RobotPilots@2022
 * @Version     V1.0
 * @date        2022
 * @brief       CAN Potocal.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "can_protocal.h"
#include "judge_infantryprotocol.h"

extern judge_sensor_t	judge_sensor;
//void Judge_Feedback(uint32_t ID, judge_sensor_t *judge ,uint8_t *date);

/*************** 信息接收处理 ***************/
/**
 *	@brief	CAN1 接收数据 数据处理
 */

void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{	
	
		MOTOR_CAN1_RX(canId, rxBuf);	
		CAP_RP_2023.updata(canId, rxBuf);	

}

/**
 *	@brief	CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	#if POSITION_N == 1
	
		Up_RX(canId,can2_rx_data);	
		MOTOR_CAN2_RX(canId, rxBuf);	
	#endif
	
	#if POSITION_N == 0
	  Down_RX(canId, rxBuf);
	  CAP_RP_2023.updata(canId, rxBuf);
	#endif	

}

