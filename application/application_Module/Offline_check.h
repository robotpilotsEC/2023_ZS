#ifndef __OFFLINE_CHECK_H
#define __OFFLINE_CHECK_H

#include "POTOCAL.h"
#include "Chassis.h"
#include "CAN_receive.h"
#include "gimbal_motor.h"
#include "classis_motor.h" 
#include "shoot_motor.h"
#include "tim_drv.h"
#include "rc.h"
#include "key.h"
#include "Motor.h"


typedef struct{
	uint32_t time;
}sleep_t;

typedef struct{
	uint8_t KeyZ_ON;
	uint8_t KeyX_ON;
	uint8_t KeyCtrl_ON;
}ALL_Reset_t;


////∑¢ÀÕ” œ‰ID∫≈  std_id
////RM2006 RM3508
//#define RM_F_ID 0x200
//#define RM_B_ID 0x1FF
////GM6020
//#define GM_F_ID 0x1FF
//#define GM_B_ID 0x2FF



extern int16_t Offline_flag;
extern ALL_Reset_t ALL_Reset;

extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void OFFLINE_CHECK_TASK(void);
void RC_Offline_Sleep(void);
void Sleep_Mode(void);
void rc_offline_handle(void);
void motor_offline_handle(void);
void System_Reset(void);
void All_Reset_Judge(ALL_Reset_t *ALL_Reset);
void offline_LED_UP(void);
void offline_LED_DOWN(void);
#endif







