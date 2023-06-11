#ifndef CALSSIS_MOTOR_H
#define CALSSIS_MOTOR_H


#include "can.h"
#include "rc.h"
#include "math_support.h"
#include "uart_drv.h"
#include "gimbal_motor.h"
#include <math.h>
#include "Power_Limit.h"
#include "key.h"
#include "motor_config.h"
#include "speed_config.h"
#include "Chassis.h"
#include "cap_protocal.h"


typedef struct __chassis_speed_t
{
	float rate_limit;//限制速度
	float shift_enlarge_speed_rate;//按下shift加到最大速度
	float speed_max;
	float Speed_xy[4];
	float Speed_xyz[4];
	float Speed_x;
	float Speed_y;
	float Speed_z;

	float Speed_X_out;
	float Speed_Y_out;

	float Speed_x_45;
	float Speed_y_45;
	float sin_now_45;
	float cos_now_45;

	float x;//小陀螺x坐标
	float y;//小陀螺y坐标
	float sin_now;
	float cos_now;
	float rate;
}_chassis_speed_t;
	
extern int16_t angle_target_gyro;
extern int16_t angle_target_gyro_now;
extern int16_t MODE;
extern int16_t MODE_MAC_OR_TOP;
extern int16_t Small_Top_Mode_Flag;
extern int16_t Small_Top_Mode ;
extern _chassis_speed_t chassis_speed;
extern int16_t current_limit[4];    //祖传功率算法限功率输出数组定义


void chassis_speed_init(_chassis_speed_t *speed_packet);
void Chassis_motor(_chassis_speed_t *chassis_speed);
void Chassis_speed_set(_chassis_speed_t *chassis_speed);
void Gimbal_half_turn_Chassis_speed_set(_chassis_speed_t *chassis_speed);
void Small_top_Chassis_speed_set(_chassis_speed_t *chassis_speed);

#endif


