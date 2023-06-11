#ifndef __ZS_RP_CONFIG_H
#define __ZS_RP_CONFIG_H
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "rp_driver.h"
#include "rp_device.h"

/*------------------
 *1 double_mec
 *2 normal_mec
 ------------------*/
#define CAR_MODE 2
/*------------------
 *1 单枪模式
 *2 双枪模式
 ------------------*/
#define BARR 1
/*------------------
 *0 下主控
 *1 上主控
 ------------------*/
#define POSITION_N 1
//全程序开关位
#define ON 1
#define OFF 0
//模式（遥控/键盘/视觉）
#define MODE_RC 0
#define MODE_KEY 1
#define MODE_VISION 2

//小陀螺模式（开/关）
#define Little_TOP_ON  1
#define Little_TOP_OFF 0

/*------------------
 *2 机械模式
 *3 陀螺仪模式
 ------------------*/
#define MODE_MAC 2
#define MODE_TOP 3
/*------------------
 小陀螺模式
 *0 慢速
 *1 快速
 *2 变速
 ------------------*/
#define SMALL_TOP_MODE_FAST 0
#define SMALL_TOP_MODE_S_F  1


#define CAP_OUTPUT_POWER_LIMIT 150
#define CAP_OUTPUT_POWER_LOW   15

#define VISION_MD  POSITION_N
#define CHASSIS_MD POSITION_N
#define GIMBAL_MD  POSITION_N
#define SHOOT_MD   POSITION_N
	 
#define AIM_X 980
#define AIM_Y 540


//陀螺仪kp
#define IMU_INIT_KP 1.0f
#define IMU_NORM_KP 0.1f

#define MOUSE_X_RATE 15.f//6.46f  //鼠标转换为通道值的比例
#define MOUSE_Y_RATE 10.f//8.49f  //鼠标转换为通道值的比例

#endif
