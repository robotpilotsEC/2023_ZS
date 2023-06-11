#include "rp_init.h"
#include "judge_sensor.h"
#include "vision_sensor.h"
#include "vision_potocal.h"

void DRIVER_Init(void)
{

	CAN1_Init();
	CAN2_Init();		

	PWM_Init();	
	USART5_Init();
	USART2_Init();
	USART3_Init();
	

	//USART4_Init();
	//USART1_Init();
	
}
void DEVICE_Init(void)
{	

	Init_Rc();
	Init_vision();
	
	judge_sensor.init(&judge_sensor);
	vision_sensor.init(&vision_sensor);
}

