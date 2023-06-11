//#include "can_communication_double_board.h"
//#include "main.h"
//#include "string.h"

//extern CAN_HandleTypeDef hcan1;
//extern CAN_HandleTypeDef hcan2;

//static CAN_TxHeaderTypeDef  tx_message;
//static uint8_t my_info_t[5];
//get_info_t get_info[2];

////receive data read
//#define get_another_measure(ptr, data)                                    \
//    {                                                                   \
//                                                                              \
//        (ptr)->age = (uint8_t)((data)[4]);                          \
//        (ptr)->height = (uint32_t)((data)[0] << 8 | (data)[1] << 8 | (data)[2] << 8 | (data)[3]);      \
//                                                                     \
//    }


///**
//  * @brief          send data to another board (0x101)
//  * @param[in]      
//  
//  * @retval         none
//  */
///**
//  * @brief          给另一块板子发送数据(0x101)
//  
//  * @retval         none
//  */
//void CAN_cmd_double(uint8_t send_data_1, fp32 send_data_2)
//{
//    uint32_t send_mail_box ;
//    tx_message.StdId = 0x101;
//    tx_message.IDE = CAN_ID_STD;
//    tx_message.RTR = CAN_RTR_DATA;
//    tx_message.DLC = 0x05;
//    
////	  memcpy(my_info_t,&send_data_2,4);
//	
//    my_info_t[0] = send_data_1;
//	   memcpy(my_info_t+1,&send_data_2,4);
//	
//    HAL_CAN_AddTxMessage(&hcan2, &tx_message, my_info_t, &send_mail_box);
//}

///**
//  * @brief          hal CAN fifo call back, receive motor data
//  * @param[in]      hcan, the point to CAN handle
//  * @retval         none
//  */
///**
//  * @brief          hal库CAN回调函数,接收数据
//  * @param[in]      hcan:CAN句柄指针
//  * @retval         none
//  */
//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    CAN_RxHeaderTypeDef rx_header_1;
//    uint8_t rx_data[8];

//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header_1, rx_data);

//    switch (rx_header_1.StdId)
//    {
//        case 0x101:
//				
//        {
//            //get id
//            get_another_measure(&get_info[0], rx_data);
//            break;
//        }

//        default:
//        {
//            break;
//        }
//    }
//		
//		
//}


