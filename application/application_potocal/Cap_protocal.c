#include "cap_protocal.h"
#include "judge_sensor.h"
#include "can_receive.h"
#include "super_cap.h"
#include "CAN_send.h"

Cap_send_data_t    Cap_send_data;
Cap_receive_data_t Cap_receive_data;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern CAP_RP_t CAP_RP_2023;
extern judge_sensor_t	judge_sensor;

int16_t can_data_0x2E[4] = {0};
int16_t can_data_0x2F[4] = {0};

int16_t Cap_output_limit = CAP_OUTPUT_POWER_LIMIT;

void set_message()
{
    Cap_send_data.Chassis_power_buffer = judge_sensor.info->power_heat_data.chassis_power_buffer;
    Cap_send_data.Chassis_power_limit = judge_sensor.info->game_robot_status.chassis_power_limit;
    Cap_send_data.Chassis_output_volt = judge_sensor.info->power_heat_data.chassis_volt;	//**
    Cap_send_data.Chassis_output_curr = judge_sensor.info->power_heat_data.chassis_current;	//**
		Cap_send_data.Discharge_power_limit = Cap_output_limit;
    Cap_send_data.Charge_power_limit = 150;
	
    Cap_send_data.Cap_control.bit.Cap_switch = 1; //电容开关
    Cap_send_data.Cap_control.bit.Cap_record = 0; //记录功能开关
	
    if(judge_sensor.info->game_status.game_progress == 4)	//***
    {
    	Cap_send_data.Cap_control.bit.Gamegoing = 1;
		}
		else
		{
			Cap_send_data.Cap_control.bit.Gamegoing = 0;
		}	
}

//裁判系统发送功率缓冲的频率为50Hz，该函数需要每20ms调用一次。
void can_send_0x2E(char can)
{
    can_data_0x2E[0] = Cap_send_data.Chassis_power_buffer;    //底盘功率缓冲，0~60J
    can_data_0x2E[1] = Cap_send_data.Chassis_output_volt;    	//底盘输出电压 单位 毫伏 **
    can_data_0x2E[2] = Cap_send_data.Chassis_output_curr;    //底盘输出电流 单位 毫安 **
	
		if(can == 1)
			CAN_SendData(&hcan1, (uint32_t)0x2E, can_data_0x2E);
		
		else if(can == 2)
			CAN_SendData(&hcan2, (uint32_t)0x2E, can_data_0x2E);
}

void can_send_0x2F(char can)
{

    can_data_0x2F[0] = Cap_send_data.Chassis_power_limit;   //底盘功率限制上限，0~120W
    can_data_0x2F[1] = Cap_send_data.Discharge_power_limit;    //电容放电功率限制，-120~300W
    can_data_0x2F[2] = Cap_send_data.Charge_power_limit;    //电容充电功率限制，0~150W
    can_data_0x2F[3] = Cap_send_data.Cap_control.all;            //电容开关，0（关闭）、1（开启）
	
		if(can == 1)
			CAN_SendData(&hcan1, (uint32_t)0x2F, can_data_0x2F);
		
		else if(can == 2)
			CAN_SendData(&hcan2, (uint32_t)0x2F, can_data_0x2F);
}


void CAN_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
    if(canId == 0x30)
    {
        Cap_receive_data.Cap_volt = int16_to_float(((uint16_t)rxBuf[0] << 8| rxBuf[1]), 32000, -32000, 30, 0);
        Cap_receive_data.Cap_Curr = int16_to_float(((uint16_t)rxBuf[2] << 8| rxBuf[3]), 32000, -32000, 20, -20);
        Cap_receive_data.Cap_state.State = ((uint16_t)rxBuf[4] << 8| rxBuf[5]);    
        CAP_RP_2023.offline_cnt = 0;			
    }
}

int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min)
{
    int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;   //加0.5使向下取整变成四舍五入
    return b;
}

float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
    return b;
}
