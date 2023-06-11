#ifndef __AUTO_AIM_H
#define __AUTO_AIM_H

#include "zs_rp_config.h"
#include "uart_drv.h"
#include  "vision_potocal.h"
#include  "vision_sensor.h"
#include  "judge_sensor.h"
#include "key.h"
#include "shoot_motor.h"
#include "main.h"
typedef enum {

 RED = 0,
 BLUE,
 AUTO,	
 NULL_ENEMY,	
	
} Enemy_t;

typedef enum { 
 NO  = 0,//未ok
 OK  = 1,//已ok
 ING = 2,//正在进行
} State_t;


extern uint8_t Vision_Mode;
//extern Vision_Cmd_Id_t Vision_Cmd_Mode;
bool Find_Tar(void);
char Get_Color(void);
uint8_t Get_Speed_Limit(void);
State_t ZM_STATE(void);
State_t DF_STATE(void);
bool ARRIVE(void);
bool Auto_Shoot(void);
#endif 
