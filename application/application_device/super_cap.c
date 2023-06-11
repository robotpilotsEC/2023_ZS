#include "super_cap.h"


CAP_RP_t CAP_RP_2023 = {

 .TX       = &Cap_send_data,
 .RX       = &Cap_receive_data,
 .work_state = DEV_OFFLINE,	
 .offline_max_cnt = 5000,
 .heart_beat = cap_rp_heart_beat,
 .updata     = CAN_rxDataHandler,
 .ctrl       = cap_rp_ctrl,
};


void cap_rp_ctrl(void)
{
	set_message();
		
	can_send_0x2E(1);
	can_send_0x2F(1);
		
}

void cap_rp_heart_beat(void)
{

	CAP_RP_2023.offline_cnt++;
	if(CAP_RP_2023.offline_cnt > CAP_RP_2023.offline_max_cnt) 
	{
		CAP_RP_2023.offline_cnt = CAP_RP_2023.offline_max_cnt;
		CAP_RP_2023.work_state = DEV_OFFLINE;
	}
	else 
	{
		if(CAP_RP_2023.work_state == DEV_OFFLINE)
			 CAP_RP_2023.work_state =  DEV_ONLINE;
	}
}






