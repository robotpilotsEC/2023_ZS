#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H

#include "zs_rp_config.h"

#if CAR_MODE == 1

//底盘机械中值角度
#define CHASSIS_MAC_FRONT_MID_VALUE 2060
#define CHASSIS_MAC_BACK_MID_VALUE  6152
//云台机械中值角度+机械角度上下限
#define PITCH_GIMBAL_MAC_MID_VALUE 6177
#define PITCH_GIMBAL_MAC_VALUE_MIN 5530
#define PITCH_GIMBAL_MAC_VALUE_MAX 6600
//头尾判断角度
#define F_B_DECIDE 4500

#define PITCH_GIMBAL_GYRO_VALUE_MIN -25
#define PITCH_GIMBAL_GYRO_VALUE_MAX  38 //双枪38 普麦 19

#endif


#if CAR_MODE == 2
//底盘机械中值角度
#define CHASSIS_MAC_FRONT_MID_VALUE 3368


#define CHASSIS_MAC_BACK_MID_VALUE  7438
//云台机械中值角度+机械角度上下限
#define PITCH_GIMBAL_MAC_MID_VALUE 2760
#define PITCH_GIMBAL_MAC_VALUE_MIN 2000
#define PITCH_GIMBAL_MAC_VALUE_MAX 3000
//头尾判断角度
#define F_B_DECIDE 4500

#define PITCH_GIMBAL_GYRO_VALUE_MIN -25
#define PITCH_GIMBAL_GYRO_VALUE_MAX  19 //双枪38 普麦 19

#endif



//底盘PID参数设置
#define PID_CHASSIS_KP   8.0f
#define PID_CHASSIS_KI   0.6f
#define PID_CHASSIS_KD   0.0f
#define PID_CHASSIS_INTEGRALIMIT     5000
#define PID_CHASSIS_MAXOUTPUT        9000

#define PID_CHASSIS_INTEGRALIMIT     5000
#define PID_CHASSIS_MAXOUTPUT        9000
#define SLOW_PID_CHASSIS_MAXOUTPUT   4000


//底盘跟随PID参数设置
#define PID_CHASSIS_FOLLOW_KP      0.25f
#define PID_CHASSIS_FOLLOW_KI      0.0f
#define PID_CHASSIS_FOLLOW_KD      0.03f
#define PID_CHASSIS_FOLLOW_INTEGRALIMIT   5000
#define PID_CHASSIS_FOLLOW_MAXOUTPUT      9000

//云台PID参数设置
#define SPD_PID_GIMBAL_YAW_GYRO_KP            26.0f
#define SPD_PID_GIMBAL_YAW_GYRO_KI					  1.0f
#define SPD_PID_GIMBAL_YAW_GYRO_KD    				0.0f
#define SPD_PID_GIMBAL_YAW_GYRO_INTEGRALIMIT  2000
#define SPD_PID_GIMBAL_YAW_GYRO_MAXOUTPUT     25000

#define POS_PID_GIMBAL_YAW_GYRO_KP 						230.0f
#define POS_PID_GIMBAL_YAW_GYRO_KI						0.0f
#define POS_PID_GIMBAL_YAW_GYRO_KD            0.0f
#define POS_PID_GIMBAL_YAW_GYRO_INTEGRALIMIT  0.0f
#define POS_PID_GIMBAL_YAW_GYRO_MAXOUTPUT     25000

#define SPD_PID_GIMBAL_YAW_MAC_KP							30.0f
#define SPD_PID_GIMBAL_YAW_MAC_KI  						1.5f
#define SPD_PID_GIMBAL_YAW_MAC_KD             0.0f
#define SPD_PID_GIMBAL_YAW_MAC_INTEGRALIMIT   2000
#define SPD_PID_GIMBAL_YAW_MAC_MAXOUTPUT      20000

#define POS_PID_GIMBAL_YAW_MAC_KP 						14.0f
#define POS_PID_GIMBAL_YAW_MAC_KI 						0.0f
#define POS_PID_GIMBAL_YAW_MAC_KD 						0.0f
#define POS_PID_GIMBAL_YAW_MAC_INTEGRALIMIT   0.0f
#define POS_PID_GIMBAL_YAW_MAC_MAXOUTPUT      20000

#define SPD_PID_GIMBAL_PITCH_GYRO_KP            22.0f//原来为23
#define SPD_PID_GIMBAL_PITCH_GYRO_KI 						0.9f
#define SPD_PID_GIMBAL_PITCH_GYRO_KD    				0.0f
#define SPD_PID_GIMBAL_PITCH_GYRO_INTEGRALIMIT  5000
#define SPD_PID_GIMBAL_PITCH_GYRO_MAXOUTPUT     25000

#define POS_PID_GIMBAL_PITCH_GYRO_KP 						380.0f
#define POS_PID_GIMBAL_PITCH_GYRO_KI						0.0f
#define POS_PID_GIMBAL_PITCH_GYRO_KD            0.03f
#define POS_PID_GIMBAL_PITCH_GYRO_INTEGRALIMIT  0.0f
#define POS_PID_GIMBAL_PITCH_GYRO_MAXOUTPUT     25000

#define SPD_PID_GIMBAL_PITCH_MAC_KP							17.0f
#define SPD_PID_GIMBAL_PITCH_MAC_KI 						1.5f
#define SPD_PID_GIMBAL_PITCH_MAC_KD             0.0f
#define SPD_PID_GIMBAL_PITCH_MAC_INTEGRALIMIT   2000
#define SPD_PID_GIMBAL_PITCH_MAC_MAXOUTPUT      20000

#define POS_PID_GIMBAL_PITCH_MAC_KP 						13.0f
#define POS_PID_GIMBAL_PITCH_MAC_KI 						0.0f
#define POS_PID_GIMBAL_PITCH_MAC_KD 						0.0f
#define POS_PID_GIMBAL_PITCH_MAC_INTEGRALIMIT   0.0f
#define POS_PID_GIMBAL_PITCH_MAC_MAXOUTPUT      20000


#define SPD_PID_SHOOT_FRICTION_KP 						8.0f
#define SPD_PID_SHOOT_FRICTION_KI 						0.5f
#define SPD_PID_SHOOT_FRICTION_KD 						0.4f
#define SPD_PID_SHOOT_FRICTION_INTEGRALIMIT   2000
#define SPD_PID_SHOOT_FRICTION_MAXOUTPUT      8000
#endif









