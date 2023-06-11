#ifndef __RP_INIT_H
#define __RP_INIT_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

#include "haltick_drv.h"
#include "CAN_receive.h"
#include "tim_drv.h"
#include "uart_drv.h"
#include "io_drv.h"

#include "judge_infantryprotocol.h"
#include "motor.h"
#include "imu.h"
#include "rc.h"

void DRIVER_Init(void);
void DEVICE_Init(void);

#endif
