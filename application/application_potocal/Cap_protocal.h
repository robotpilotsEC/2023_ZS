#ifndef __CAP_PROTOCAL_H
#define __CAP_PROTOCAL_H

#include "main.h"
#include"zs_rp_config.h"

typedef struct{
	
	uint16_t Chassis_power_buffer;      //���̹��ʻ��壺0-60 J
	uint16_t Chassis_power_limit;       //�����˵��̹������� 0-120W
	int16_t  Discharge_power_limit;     //���ݷŵ繦�����ƣ�-120-300W
	int16_t  Charge_power_limit;        //���ݳ�繦�����ƣ�0-150W
	uint16_t Chassis_output_volt;				//���������ѹ ��λ ���� **
	uint16_t Chassis_output_curr;				//����������� ��λ ���� **

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
	float Cap_volt;		//�������˵�ѹ��0-30V
	float Cap_Curr;		//���ݵ�����-20-20A
	union{
		uint16_t State;   //����״̬
		struct{
			uint16_t Warning : 1;					  //����
			uint16_t Cap_overvolt : 1;		  //���ݹ�ѹ
			uint16_t Cap_overcurr : 1;		  //���ݹ���
			uint16_t Cap_undervolt : 1;		  //����Ƿѹ
			uint16_t Cap_undercurr : 1;		  //����Ƿ��
			uint16_t Can_receive_failed :1; //����δ���յ�CANͨ������
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
