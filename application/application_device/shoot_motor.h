#ifndef SHOOT_MOTOR_H
#define SHOOT_MOTOR_H


#include "can.h"
#include "rc.h"
#include "math_support.h"
#include "uart_drv.h"
#include "tim_drv.h"
#include "slave.h"
#include "classis_motor.h"
#include "motor_config.h"
#include "speed_config.h"
#include "Shoot.h"


typedef struct __shoot_skip_t
{
	int16_t s2_friction_last; //摩擦轮跳变
	int16_t s2_friction_now ;
	int16_t s2_flick_last;
	int16_t s2_flick_now;
	int16_t s2_cover_last;
	int16_t s2_cover_now;
}_shoot_skip_t;

typedef struct __shoot_flag_t
{
	int16_t friction_flag_top ;            //跟随模式摩擦轮开关标志位
	int16_t flick_flag_top ;              //跟随模式拨弹轮开关标志位

	int16_t friction_flag_mac;            //机械模式摩擦轮开关标志位
	int16_t flick_flag_mac;               //机械模式拨弹轮开关标志位

	int16_t cover_flag ;                  //弹仓开关标志位
}_shoot_flag_t;

typedef struct __change_barr_t
{
		int64_t no_barrel_clock_time;
		int64_t barrel_clock_time;
		int16_t barrel_clock;
		int32_t barrel_1_pos;
		int32_t barrel_2_pos;
		float change_barral_pos_target;
		float change_barral_speed_tar;
		float change_barral_state; 
		float change_barral_state_2_to_1;  
		float change_barral_state_1_to_2;  
		float	change_barral_motor_current;                     //换枪管电机电机电流（2006）
		char barrel_ID ;                                      //枪管默认ID号
}_change_barr_t;

typedef  struct
{
	int16_t cnt_11;
	int16_t cnt_12;
	int16_t cnt_13;
	int16_t cnt_14;
	int16_t cnt_15;

	int16_t cnt_16;
	int16_t cnt_17;
	int16_t cnt_18;
	int16_t cnt_19;
	int16_t cnt_20;
	
	int16_t cnt_21;
	int16_t cnt_22;
	int16_t cnt_23;
	int16_t cnt_24;
	int16_t cnt_25;	
	
	int16_t cnt_26;
	int16_t cnt_27;
	int16_t cnt_28;
	int16_t cnt_29;
	int16_t cnt_30;
}shoot_speed_t;



typedef  struct shoot_recode_str
{
	float pj,sp_cnt,max,min,jicha;
	float speed, pre_speed;
	
	int16_t cnt;
	shoot_speed_t shoot_speed;
	
	
}shoot_recode_t;




extern int16_t judge_jam_flag ;
extern int16_t barrel_clock;
extern float flick_motor_current; 
extern _shoot_skip_t shoot_skip;
extern _shoot_flag_t shoot_flag;
extern _change_barr_t change_barr;
extern	float flick_pos_target;
extern float friction_speed_tar_L;
extern float friction_speed_tar_R;
extern float friction_speed_tar;

void Shoot_Speed_Test(void);
void shoot_motor_running_fire(void);
void shoot_motor_single_fire(void);
void shoot_stop(void);
void change_barrel_motor(_change_barr_t *change_barr_t);
void Gam_handle(void);
void Box_Cover_Control_RC(void);
void Box_Cover_Control(void);
void Cover_Close(void);
void Cover_Open(void);
void Shoot_heat_limit(_change_barr_t *change_barr_t);
void shoot_motor_fire_jam_judge(void);
void shoot_skip_init(_shoot_skip_t *shoot_skip_t);
void shoot_flag_init(_shoot_flag_t *shoot_flag_t, _change_barr_t *change_barr_t);
void Shoot_speed_limit(void);
void shoot_motor_running_fire_rc(rc_sensor_t *rc);
void change_clock(_change_barr_t *change_barr_t);
void Fri_Spd_adjust(void);
void R_Fast_Shoot(void);
void Fri_Stop_Judge(void);
void Auto_Shoot_Ctrl(void);
#endif
