#ifndef __SUPER_CAP_H
#define __SUPER_CAP_H

#include "zs_rp_config.h"
#include "rp_math.h"
#include "S_function.h"
#include "can_receive.h"
#include "cap_protocal.h"

typedef struct CAP_RP_struct {
 Cap_send_data_t      *TX;
 Cap_receive_data_t   *RX;
 uint16_t		       offline_cnt;
 uint16_t		       offline_max_cnt;	
 dev_work_state_t	 work_state; 


 void   (*ctrl)(void);	
 void		(*heart_beat)(void);	
 void		(*updata)(uint32_t canId, uint8_t *rxBuf);
} CAP_RP_t;

extern CAP_RP_t CAP_RP_2023;


void cap_rp_ctrl(void);
void cap_rp_heart_beat(void);
#endif
