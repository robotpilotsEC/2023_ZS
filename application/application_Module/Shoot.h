#ifndef SHOOT_H
#define SHOOT_H
#include "pid.h"
#include "motor_config.h"


void shoot_left_friction_wheel_init(void);
void shoot_right_friction_wheel_init(void);
void shoot_flick_wheel_follow_init(void);
void shoot_flick_wheel_mac_init(void);
void shoot_flick_wheel_stop_init(void);
void shoot_flick_wheel_jam_handle_init(void);
void shoot_change_barrel_init(void);
	

#endif 


