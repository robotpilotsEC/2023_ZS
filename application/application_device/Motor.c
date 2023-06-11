/********************************************************************************
  * @file		Motor.c
	* @author    RobotPilots@2022
  * @version		V1.0.0
  * @date			
  * @brief   		
  *******************************************************************************/

#include "Motor.h"

chassis_motor_info_t chassis_motor_info;
gimbal_motor_info_t gimbal_motor_info;
shoot_motor_info_t shoot_motor_info;

motor_offline_check_t motor_offline_check = {
	.chassis_motor_offline = &chassis_motor_info,
	.gimbal_motor_offline = &gimbal_motor_info,
	.shoot_motor_offline = &shoot_motor_info,
};



/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机;*/
motor_measure_t motor_chassis_gimbal[6];
/*	

电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机;*/
motor_measure_t motor_shoot[4];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              shoot_can_send_data[8];


//motor data read 
//底盘和云台的数据处理
void get_motor_measure(motor_measure_t *ptr)                                    
{                                                                   
	ptr->last_angle = ptr->angle;																
	ptr->angle = (uint16_t)(can1_rx_data[0]<<8 | can1_rx_data[1]);           
	ptr->real_current = (int16_t)(can1_rx_data[4]<<8 |can1_rx_data[5]);     
	ptr->speed_rpm = (int16_t)(can1_rx_data[2]<<8 |can1_rx_data[3]);                          
	ptr->hall = can1_rx_data[6];   
	
	if(ptr->angle - ptr->last_angle > 4096)                         
	ptr->round_cnt --;                                            
	else if (ptr->angle - ptr->last_angle < -4096)                   
	ptr->round_cnt ++;                                             
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;                                            
}

void get_shoot_motor_measure(motor_measure_t *ptr)                                    
{                                                                   
	ptr->last_angle = ptr->angle;																
	ptr->angle = (uint16_t)(can2_rx_data[0]<<8 | can2_rx_data[1]);           
	ptr->real_current = (int16_t)(can2_rx_data[4]<<8 |can2_rx_data[5]);     
	ptr->speed_rpm = (int16_t)(can2_rx_data[2]<<8 |can2_rx_data[3]);                          
	ptr->hall = can2_rx_data[6];      
 	
	if(judge_jam_flag < 100)
	{
		if(ptr->angle - ptr->last_angle > 4096)                         
		ptr->round_cnt --;                                            
		else if (ptr->angle - ptr->last_angle < -4096)                   
		ptr->round_cnt ++;  
		ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;	
  }

}

void MOTOR_CAN1_RX(uint32_t canId, uint8_t *rxBuf)
{
		motor_offline_check.chassis_motor_offline->offline_cnt_max = OFFLINE_TIME_MAX;
		motor_offline_check.gimbal_motor_offline->offline_cnt_max = OFFLINE_TIME_MAX;
	switch (canId)
	{
			case CAN_3508_M1_ID:
			{
				get_motor_measure(&motor_chassis_gimbal[Left_front]);
				motor_offline_check.chassis_motor_offline->offline_cnt_L_F = 0;
				break;
			}
			case CAN_3508_M2_ID:	
			{
				get_motor_measure(&motor_chassis_gimbal[Right_front]);
				motor_offline_check.chassis_motor_offline->offline_cnt_R_F = 0;
				break;
			}				
			case CAN_3508_M3_ID:
			{
				get_motor_measure(&motor_chassis_gimbal[Left_back]);
				motor_offline_check.chassis_motor_offline->offline_cnt_L_B = 0;
				break;
			}
			case CAN_3508_M4_ID:		
			{
				get_motor_measure(&motor_chassis_gimbal[Right_back]);
				motor_offline_check.chassis_motor_offline->offline_cnt_R_B = 0;
				break;
			}
			case CAN_YAW_MOTOR_ID:
			{ 
				
				get_motor_measure(&motor_chassis_gimbal[Gimbal_yaw]);
				motor_offline_check.gimbal_motor_offline->offline_cnt_Yaw = 0;
				break;
			}			
			case CAN_PIT_MOTOR_ID:
			{			  
				
			 get_motor_measure(&motor_chassis_gimbal[Gimbal_pitch]);
				motor_offline_check.gimbal_motor_offline->offline_cnt_Pitch = 0;
				break;
			}
			default:break;		
		}
}



void MOTOR_CAN2_RX(uint32_t canId, uint8_t *rxBuf)
{
		motor_offline_check.shoot_motor_offline->offline_cnt_max = OFFLINE_TIME_MAX;
	
	switch (canId)
			{
					case 0x201://3508右摩擦轮
					{
						get_shoot_motor_measure(&motor_shoot[L_friction]);
						motor_offline_check.shoot_motor_offline->offline_cnt_R = 0; 
						break;
					}
					case 0x202://3508左摩擦轮
					{
						get_shoot_motor_measure(&motor_shoot[R_friction]);
						motor_offline_check.shoot_motor_offline->offline_cnt_L = 0;
						break;
					}
					case 0x203://拨弹轮
					{				
						get_shoot_motor_measure(&motor_shoot[flick]);
							motor_offline_check.shoot_motor_offline->offline_cnt_f = 0;
						break;
					}
					case 0x204://换枪管
					{				
						get_shoot_motor_measure(&motor_shoot[change_barrel]);
						
							motor_offline_check.shoot_motor_offline->offline_cnt_f = 0;
						break;
					}
					default:
					{
						break;
					}						
			}
}
/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t CAN_Send_Mail_box;
	
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;

	  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &gimbal_tx_message, gimbal_can_send_data, &CAN_Send_Mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box ;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_shoot(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    shoot_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    shoot_tx_message.IDE = CAN_ID_STD;
    shoot_tx_message.RTR = CAN_RTR_DATA;
    shoot_tx_message.DLC = 0x08;
    shoot_can_send_data[0] = (motor1 >> 8);
    shoot_can_send_data[1] = motor1;
    shoot_can_send_data[2] = (motor2 >> 8);
    shoot_can_send_data[3] = motor2;
    shoot_can_send_data[4] = (motor3 >> 8);
    shoot_can_send_data[5] = motor3;
    shoot_can_send_data[6] = (motor4 >> 8);
    shoot_can_send_data[7] =motor4;

	  HAL_CAN_AddTxMessage(&GIMBAL_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}
