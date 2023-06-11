/********************************************************************************
  * @file			usart_potocal.c
  * @version		V1.0.0
  * @date			
  * @brief   		
 *******************************************************************************/

/*
*	 usart 通信接口
*/

#include "usart_potocal.h"

#include <string.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

/*
	 更新检查数据
*/
void USART2_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_SendData(rxBuf,huart2);
 	Rc_Data(rxBuf);
}

/*
*	@brief	在串口3中解析遥控数据协议
*/
void USART3_rxDataHandler(uint8_t *rxBuf)
{
	
	if(USART_TEST)UART_SendData(rxBuf,huart3);
  
	vision_sensor.update(&vision_sensor, rxBuf);
	vision_sensor.check(&vision_sensor);
  	
}


/**
 *	@brief	在串口5中解析裁判系统数据协议
 */

void USART5_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_SendData(rxBuf,huart5);	
	
		judge_sensor.update(&judge_sensor, rxBuf);
		judge_sensor.check(&judge_sensor);
	
}
