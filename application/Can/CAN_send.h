#ifndef __CAN_SEND_H_
#define __CAN_SEND_H_

#include "struct_typedef.h"
#include "can_protocal.h"

uint8_t Board_Tx_uint8(uint32_t std, uint8_t *data,char hcan,uint32_t DL);
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, int16_t *dat);



#endif
