#ifndef __SPEED_CONFIG_H
#define __SPEED_CONFIG_H

#include "zs_rp_config.h"

//底盘速度放大倍数
#define ENLARGE_SPEED 12.0f

//底盘速度最大值
#define CHASSIS_SPEED_MAX 8000.0f

//Shift 键按下：1.0
//Shift 键谈弹起：0.6
#define SHIFT_HIGH_SPD_RATE 1.0f
#define SHIFT_LOW_SPD_RATE 0.5f

//云台速度放大倍数
#define ENLARGE_GIMBAL_SPEED 5

//拨弹轮速度
#define FLICK_SPEED_HIGH  -6500
#define FLICK_SPEED     -4000
#define FLICK_SPEED_DEC  -2000
#define CHANGE_BARR_SPEED 1000
#define CHANGE_BARR_SPEED_FAST 4000
//摩擦轮速度
#define FRICTION_SPEED -4000

//单发所转动的机械角度
#define SINGLE_SHOOT_ANGLE 36000//36864

//射击摩擦轮转速
#if CAR_MODE == 2
#define Fri_spd_15 -4400
#define Fri_spd_18 -4950
#define Fri_spd_20 -4950
#define Fri_spd_22 -5200
#define Fri_spd_30 -6800

#endif

//射击摩擦轮转速
#if CAR_MODE == 1
#define Fri_spd_15 -4400
#define Fri_spd_18 -4870
#define Fri_spd_20 -4950
#define Fri_spd_22 -5200
#define Fri_spd_30 -6800
#endif

//小陀螺速度
#define SMALL_TOP_SPEED_SLOW 100
#define SMALL_TOP_SPEED_MID  200
#define SMALL_TOP_SPEED_FAST 300

//遥控器控制云台速度的缩小率
#define  ANGLE_LIM_GYRO_YAW_RATE -0.0015
#define  ANGLE_LIM_GYRO_PITCH_RATE -0.0015
#define  ANGLE_LIM_MAC_PITCH_RATE -0.005

#endif

