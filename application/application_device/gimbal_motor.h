#ifndef GIMBAL_MOTOR_H
#define GIMBAL_MOTOR_H


#include "main.h"
#include "classis_motor.h"
#include "can.h"
#include "rc.h"
#include "math_support.h"
#include "uart_drv.h"
#include "bmi.h"
#include "Offline_check.h"
#include "vision_potocal.h"
#include "vision_sensor.h"
#include "motor_config.h"
#include "speed_config.h"
#include "gimbal.h"
#include "Auto_Aim.h"



typedef struct Gimbal_angle_struct
{
  float angle_target_follow_yaw;          //yaw��̨Ŀ��ֵ������ģʽ��
	float angle_lessen_follow_yaw;          //pitch��̨ң��������ֵ��С������ģʽ��
	float angle_target_follow_pitch;        //yaw��̨Ŀ��ֵ������ģʽ��
	float angle_lessen_follow_pitch ;       //pitch��̨ң��������ֵ��С������ģʽ��
	float angle_target_gimbal_pitch_mac ;   //pitch��̨ң��������ֵ��С(��еģʽ)
	float angle_lessen_gimbal_pitch_mac;    //pitch��̨ң��������ֵ��С����еģʽ��
	float gimbal_pitch_follow_mac_angle;    //pitch��̨�ڸ���ģʽʱ�ĵ����е�Ƕ�
	uint16_t angle_mac_follow;              //���̸���YAW�Ļ�е�Ƕ�
	float pitch_mac ;	                     //��еģʽ�µ�gyro pitchֵ
}Gimbal_angle_t;

extern rc_sensor_t  rc;
extern Gimbal_angle_t Gimbal_angle;
extern char vision_enter;
extern char vision_enter_rc;

extern float gimbal_yaw_motor_current;
extern float gimbal_pitch_motor_current;

void Gimbal_follow_motor(void);
void Gimbal_angle_struct_Init(Gimbal_angle_t* gimbal_angle);
void Gimbal_mac_motor(void);
void vision_enter_RC(void);
void Gimbal_vision(void);
void Gimbal_Pos_Init(void);
void Gimbal_Auto_Ctrl(void);
void imu_kp_change(void);
void Change_Target(void);
#endif

