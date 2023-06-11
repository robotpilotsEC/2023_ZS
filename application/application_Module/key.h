#ifndef __KEY_H
#define __KEY_H


#include "judge_sensor.h"
#include "vision_sensor.h"
#include "slave.h"
#include "imu.h"
#include "rc.h"
#include "gimbal_motor.h"
#include "classis_motor.h"
#include "shoot_motor.h"
#include "S_function.h"

#define  TURN_45_MAC_ANGLE_VALUE 1000

extern char fast_180_flag;
extern char single_shoot_flag;
extern int16_t KEY_Q_Speed_z;
extern int16_t KEY_E_Speed_z;
extern char High_Spd ;
extern char Low_Spd ;
extern char TURN_45_FLAG;
extern uint8_t Enemy;
void ENEMY(void);
void KB_CTRL(void);

void RC_S1_CTRL(void);
void RC_S2_CTRL(void);
#endif

