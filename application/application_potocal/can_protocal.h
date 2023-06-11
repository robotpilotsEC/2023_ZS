#ifndef __CAN_POTOCAL_H
#define __CAN_POTOCAL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "CAN_receive.h"
#include "Motor.h"
#include "super_cap.h"


/* Exported functions --------------------------------------------------------*/
void	CAN_SendSingleData(drv_can_t *drv, int16_t txData);
void 	CAN_SendDataBuff(drv_type_t drv_type, uint32_t std_id, int16_t *txBuff);

#endif

