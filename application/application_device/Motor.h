#ifndef __MOTOR_H
#define __MOTOR_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "can_protocal.h"
/* CAN send and receive ID */
typedef enum{
    Left_front	  		       = 0,
    Right_front 		         = 1,
    Left_back			           = 2,  
    Right_back				       = 3,
	  Gimbal_yaw               = 4,
	  Gimbal_pitch             = 5,
	
}motor_num;

typedef enum{
    R_friction	  		       = 0,
    L_friction 		           = 1,
    flick			               = 2,     
	  change_barrel            = 3,
}shoot_motor_num;
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

//rm motor data
typedef struct{   
    int16_t   speed_rpm;
	  int16_t   real_current;
    int16_t   given_current;
	  uint8_t  	hall;
	  int16_t 	  angle;				//abs angle range:[0,8191]
	  uint16_t 	last_angle;	  //abs angle range:[0,8191]
	  uint16_t	  offset_angle;
	  int32_t		round_cnt;
	  int32_t		total_angle;
	  uint8_t		buf_idx;
	  uint16_t	  fited_angle;
	  uint32_t  	msg_cnt;
} motor_measure_t;

typedef struct 
{
	uint8_t     offline_cnt_L_F;
	uint8_t     offline_cnt_L_B;
	uint8_t     offline_cnt_R_F;
	uint8_t     offline_cnt_R_B;
	uint8_t     offline_cnt_max;
	uint8_t     status;
}chassis_motor_info_t;

typedef struct 
{
	uint8_t     offline_cnt_Pitch;
	uint8_t     offline_cnt_Yaw;
	uint8_t     offline_cnt_max;
	uint8_t     status;
}gimbal_motor_info_t;

typedef struct 
{
	uint8_t     offline_cnt_L;
	uint8_t     offline_cnt_R;
	uint8_t     offline_cnt_f;
	uint8_t     offline_cnt_max;
	uint8_t     status;
}shoot_motor_info_t;

typedef struct {
	CAN_TxHeaderTypeDef header;
	uint8_t				data[8];
} CAN_TxFrameTypeDef;


typedef struct motor_offline_check_struct {
	
	chassis_motor_info_t *chassis_motor_offline;
	gimbal_motor_info_t	*gimbal_motor_offline;
	shoot_motor_info_t	*shoot_motor_offline;

} motor_offline_check_t;

extern motor_measure_t  motor_chassis_gimbal[6];
extern motor_measure_t  motor_shoot[4];
extern motor_offline_check_t motor_offline_check;
void MOTOR_CAN1_RX(uint32_t canId, uint8_t *rxBuf);
void MOTOR_CAN2_RX(uint32_t canId, uint8_t *rxBuf);
#endif
