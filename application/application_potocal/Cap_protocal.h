#ifndef __CAP_PROTOCAL_H
#define __CAP_PROTOCAL_H

#include "main.h"
#include"zs_rp_config.h"

typedef struct{
	
	uint16_t Chassis_power_buffer;      //底盘功率缓冲：0-60 J
	uint16_t Chassis_power_limit;       //机器人底盘功率上限 0-120W
	int16_t  Discharge_power_limit;     //电容放电功率限制，-120-300W
	int16_t  Charge_power_limit;        //电容充电功率限制，0-150W
	uint16_t Chassis_output_volt;				//底盘输出电压 单位 毫伏 **
	uint16_t Chassis_output_curr;				//底盘输出电流 单位 毫安 **

  union{
		uint16_t all;
		struct{
						uint16_t Cap_switch : 1;
						uint16_t Cap_record : 1;
						uint16_t Gamegoing : 1;
					}bit;
	}Cap_control;

}Cap_send_data_t ;

typedef struct{
	float Cap_volt;		//电容两端电压，0-30V
	float Cap_Curr;		//电容电流，-20-20A
	union{
		uint16_t State;   //电容状态
		struct{
			uint16_t Warning : 1;					  //报警
			uint16_t Cap_overvolt : 1;		  //电容过压
			uint16_t Cap_overcurr : 1;		  //电容过流
			uint16_t Cap_undervolt : 1;		  //电容欠压
			uint16_t Cap_undercurr : 1;		  //电容欠流
			uint16_t Can_receive_failed :1; //电容未接收到CAN通信数据
		}bit;
	}Cap_state;

}Cap_receive_data_t;

extern int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min);
extern float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min);
extern Cap_send_data_t    Cap_send_data;
extern Cap_receive_data_t Cap_receive_data;
extern int16_t Cap_output_limit;

void can_send_0x2E(char can);
void can_send_0x2F(char can);
void set_message(void);
void CAN_rxDataHandler(uint32_t canId, uint8_t *rxBuf);


#endif
