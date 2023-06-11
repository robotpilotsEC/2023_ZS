#include "CAN_send.h"

CAN_TxFrameTypeDef hcan1TxFrame;
CAN_TxFrameTypeDef hcan2TxFrame;

/*
   上主控Can发送函数（裁判系统数据）
*/

uint8_t Board_Tx_uint8(uint32_t std, uint8_t *data,char hcan,uint32_t DL)
{
	uint8_t i;
	CAN_TxFrameTypeDef TxMes;
	uint32_t CAN_Tx_Mail_Box;
	
	TxMes.header.IDE = CAN_ID_STD;
	TxMes.header.RTR = CAN_RTR_DATA;
	TxMes.header.DLC = DL;    //
	TxMes.header.StdId = std; 	  //
	
	TxMes.data[0] = data[0];
	TxMes.data[1] = data[1];	
	TxMes.data[2] = data[2];	
	TxMes.data[3] = data[3];

	TxMes.data[4] = data[4];
	TxMes.data[5] = data[5];	
	TxMes.data[6] = data[6];	
	TxMes.data[7] = data[7];
	
	if(hcan == 1)i = HAL_CAN_AddTxMessage(&hcan1,&TxMes.header,TxMes.data,  &CAN_Tx_Mail_Box);
  if(hcan == 2)i = HAL_CAN_AddTxMessage(&hcan2,&TxMes.header,TxMes.data,  &CAN_Tx_Mail_Box);
	return i;
}


/**
 *	@brief	通过CAN发送数据
 */
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, int16_t *dat)
{
	uint32_t TxMailBox;
	CAN_TxFrameTypeDef *TxFrame;
	
	if(hcan->Instance == CAN1)
	{
		TxFrame = &hcan1TxFrame;	
	}
	else if(hcan->Instance == CAN2)
	{
		TxFrame = &hcan2TxFrame;	
	}
	else
		return HAL_ERROR;
	
	TxFrame->header.StdId = stdId;
	TxFrame->header.IDE = CAN_ID_STD;
	TxFrame->header.RTR = CAN_RTR_DATA;
	TxFrame->header.DLC = 8;
	
	TxFrame->data[0] = (uint8_t)((int16_t)dat[0] >> 8);
	TxFrame->data[1] = (uint8_t)((int16_t)dat[0]);
	TxFrame->data[2] = (uint8_t)((int16_t)dat[1] >> 8);
	TxFrame->data[3] = (uint8_t)((int16_t)dat[1]);
	TxFrame->data[4] = (uint8_t)((int16_t)dat[2] >> 8);
	TxFrame->data[5] = (uint8_t)((int16_t)dat[2]);
	TxFrame->data[6] = (uint8_t)((int16_t)dat[3] >> 8);
	TxFrame->data[7] = (uint8_t)((int16_t)dat[3]);		
	
	//if中将报文发送出去
	if(HAL_CAN_AddTxMessage(hcan, &TxFrame->header, &TxFrame->data[0], &TxMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
	
}