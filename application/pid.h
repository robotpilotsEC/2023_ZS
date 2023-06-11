/******************************************************************************

*******************************************************************************/
#ifndef __pid_H
#define __pid_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"


#define PID_CALC_FOLLOW_LESSEN_VALUE 0.8
enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,   
};

enum{
    Left_front_spd	  		           = 0,
    Right_front_spd  		             = 1,
    Left_back_spd 			             = 2,  
    Right_back_spd				           = 3,
	  Gimbal_follow_motor_yaw_spd      = 4,
	  Gimbal_follow_motor_pitch_spd    = 5,
	  Gimbal_mac_motor_yaw_spd         = 6,
	  Gimbal_mac_motor_pitch_spd       = 7,	
};

enum{
    Gimbal_follow_motor_yaw_pos      = 0,
	  Gimbal_follow_motor_pitch_pos    = 1,
	  Gimbal_mac_motor_yaw_pos         = 2,
	  Gimbal_mac_motor_pitch_pos       = 3,	  
}; 

typedef struct __pid_t
{
    float p;
    float i;
    float d;

    float target;				//目标值
    float measure;			//测量值
    float err[3];				//误差
	   
    float pout;							//p输出
    float iout;							//i输出
    float dout;							//d输出
	  
    float pos_out;						//本次位置式输出
    float last_pos_out;				//上次输出


    float last_delta_out;
    
	  float max_err;
	  float deadband;				//err < deadband return
		uint32_t MaxOutput;				//输出限幅
    uint32_t IntegralLimit;		//积分限幅

		

}pid_t;

void PID_struct_init(
    pid_t* pid,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd);
    

		
extern pid_t pid_spd[8];
extern pid_t pid_spd_follow;

extern pid_t pid_chassis_angle;
extern pid_t pid_pos[4];
extern pid_t pid_shoot_left_friction_wheel_spd;
extern pid_t pid_shoot_right_friction_wheel_spd;
extern pid_t pid_shoot_flick_wheel_follow_spd;
extern pid_t pid_shoot_flick_wheel_follow_pos;
extern pid_t pid_shoot_flick_wheel_mac_spd;
extern pid_t pid_shoot_flick_wheel_mac_pos;
extern pid_t pid_shoot_flick_wheel_jam_spd;
extern pid_t pid_shoot_flick_wheel_jam_pos;
extern pid_t pid_shoot_flick_wheel_stop_pos;
extern pid_t pid_shoot_flick_wheel_stop_spd;
extern pid_t pid_shoot_change_barrel_spd;
extern pid_t pid_shoot_change_barrel_pos;
extern pid_t pid_spd_mac_back;


void pid_calc(pid_t* pid, float fdb, float ref);
void pid_calc_follow(pid_t* pid, float measure, float target);   
void pid_calc_out_mac_yaw(pid_t* pid, float measure, float target);
void pid_calc_out_follow_yaw(pid_t* pid, float measure, float target);
void pid_calc_out_gimbal_pitch(pid_t* pid, float measure, float target);
void pid_calc_shoot_friction(pid_t* pid, float measure, float target);
void pid_calc_shoot_continuous_flick(pid_t* pid, float measure, float target);
void pid_calc_shoot_single_flick(pid_t* pid, float measure, float target);

extern float pitch,roll,yaw;
void cascade_pid_ctrl_gimbal_yaw(pid_t *outpid,pid_t *inpid,float out_measure,float in_measure,float target);
void cascade_pid_ctrl_gimbal_pitch(pid_t *outpid,pid_t *inpid,float out_measure,float in_measure,float target);
void cascade_pid_ctrl(pid_t *outpid,pid_t *inpid,float out_measure,float in_measure, float target);
void cascade_pid_ctrl_shoot(pid_t *outpid,pid_t *inpid,float out_measure,float in_measure,float target);
void cascade_pid_ctrl_mac_yaw(pid_t *outpid,pid_t *inpid,float out_measure,float in_measure,float target);
#endif

