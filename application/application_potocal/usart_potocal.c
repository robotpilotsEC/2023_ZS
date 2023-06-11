/********************************************************************************
  * @file			usart_potocal.c
  * @version		V1.0.0
  * @date			
  * @brief   		
 *******************************************************************************/

/*
*	 usart ͨ�Žӿ�
*/

#include "usart_potocal.h"

#include <string.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

/*
	 ���¼������
*/
void USART2_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_SendData(rxBuf,huart2);
 	Rc_Data(rxBuf);
}

/*
*	@brief	�ڴ���3�н���ң������Э��
*/
void USART3_rxDataHandler(uint8_t *rxBuf)
{
	
	if(USART_TEST)UART_SendData(rxBuf,huart3);
  
	vision_sensor.update(&vision_sensor, rxBuf);
	vision_sensor.check(&vision_sensor);
  	
}


/**
 *	@brief	�ڴ���5�н�������ϵͳ����Э��
 */

void USART5_rxDataHandler(uint8_t *rxBuf)
{
	if(USART_TEST)UART_SendData(rxBuf,huart5);	
	
		judge_sensor.update(&judge_sensor, rxBuf);
		judge_sensor.check(&judge_sensor);
	
}
