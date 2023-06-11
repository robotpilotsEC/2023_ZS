#ifndef CAN_COMMUNICATION_DOUBLE_BOARD_H
#define CAN_COMMUNICATION_DOUBLE_BOARD_H

#include "struct_typedef.h"


/**
  * @brief          send data to another board (0x101)
  * @param[in]      
  
  * @retval         none
  */
/**
  * @brief          给另一块板子发送数据(0x101)
  
  * @retval         none
  */
extern void CAN_cmd_double(uint8_t send_data_1, fp32 send_data_2);
//void* my_memcpy(void* dest, const void* src, size_t count);

//rm data
typedef struct
{
    int8_t age;
    int16_t height;
    
} get_info_t;



#endif




