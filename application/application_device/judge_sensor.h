#ifndef __JUDGE_SENSOR_H
#define __JUDGE_SENSOR_H

/* Includes ------------------------------------------------------------------*/

#include "ALGO.h"
#include "judge_infantryprotocol.h"


#define JUDGE_ONLINE   judge_sensor.work_state == DEV_ONLINE
#define JUDGE_OFFLINE  judge_sensor.work_state == DEV_OFFLINE

#define SHOOT_SPEED    judge_info.shoot_data.bullet_speed


#define SHOOT_SPEED_LIMIT_1       judge_info.game_robot_status.shooter_id1_17mm_speed_limit
#define SHOOT_SPEED_LIMIT_2   judge_info.game_robot_status.shooter_id2_17mm_speed_limit

#define BARREL_SPEED   judge_info.shoot_data.bullet_speed


#define SHOOTING_HEAT_1        judge_sensor.info->power_heat_data.shooter_id1_17mm_shooting_heat
#define SHOOTING_HEAT_2    judge_sensor.info->power_heat_data.shooter_id2_17mm_shooting_heat

#define SHOOTING_HEAT_LIMIT_1      judge_sensor.info->game_robot_status.shooter_id1_17mm_shooting_limit
#define SHOOTING_HEAT_LIMIT_2  judge_sensor.info->game_robot_status.shooter_id2_17mm_shooting_limit

#define STOP_SHOOT_1    (SHOOTING_HEAT_LIMIT_1 - SHOOTING_HEAT_1 <= 30)
#define START_SHOOT_1   (SHOOTING_HEAT_LIMIT_1 - SHOOTING_HEAT_1 > 30)
#define STOP_SHOOT_2    (SHOOTING_HEAT_LIMIT_2 - SHOOTING_HEAT_2 <= 30)
#define START_SHOOT_2   (SHOOTING_HEAT_LIMIT_2 - SHOOTING_HEAT_2 > 30)
#define R_SHOOT          SHOOTING_HEAT_LIMIT_1 - SHOOTING_HEAT_1

#define SLOW_SHOOT_1    (SHOOTING_HEAT_LIMIT_1 - SHOOTING_HEAT_1 <= 80)
#define SLOW_SHOOT_2    (SHOOTING_HEAT_LIMIT_2 - SHOOTING_HEAT_2 <= 80)
#define HIGH_SHOOT_1    (SHOOTING_HEAT_LIMIT_1 - SHOOTING_HEAT_1 > 80)
#define HIGH_SHOOT_2    (SHOOTING_HEAT_LIMIT_2 - SHOOTING_HEAT_2 > 80)


#define BUFFER judge_sensor.info->power_heat_data.chassis_power_buffer

extern judge_info_t judge_info;
extern judge_sensor_t judge_sensor;

/* Exported functions --------------------------------------------------------*/

#endif
