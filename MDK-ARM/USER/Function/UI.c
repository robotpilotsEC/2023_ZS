#include "UI.h"
#include "crc.h"
#include "usart.h"
#include "string.h"
#include "stdbool.h"
#include "usart.h"
#include "arm_math.h"
#include "stdio.h"
#include "judge_sensor.h"
#include "Auto_Aim.h"
#include "super_cap.h"
#include "classis_motor.h"

/*
mec
	裁判id 可以直接收取
	cap 可以直接收取
	发送pitch yaw数据
	部分标志位
	
	
	
 ----------------------------------------------------	
|	1920*1080
|		
|		
|		
|		
|		
|		
|		
|		
|		
 ----------------------------------------------------		
	
	
*/

uint8_t CliendTxBuffer[200];//客户端
uint8_t TeammateTxBuffer[200];//队友

Client_Slave_Flag Slaver_flag;//中间标志位

int16_t chassis_speed_rpm_to_v;

uint8_t global_fiction,
				global_clip,
				global_spin,
				global_auto_aim,
				global_twist,
				global_anti_top,
				global_move;

uint32_t global_sight_bead_x = 960,
				 global_sight_bead_y = 720;
         //[0,100]
				 
float global_supercapacitor_volt,//<=24.5
	    global_gimbal_pitch,
      global_gimbal_yaw,
			global_supercapacitor_point;

int   global_bullet_speed,
	    global_bullet_sum;

//ID号，发送给谁
uint8_t  robot_id;
uint16_t client_id;

extern Vision_Cmd_Id_t AUTO_CMD_MODE;
extern char LOCK1,LOCK2;
//////////////////////////////////////

/*获取状态信息*/
void Get_Client_Info(void)//上云台负责更新
{
	if     (shoot_flag.cover_flag == OFF)Slaver_flag.global_clip = true;
	else if(shoot_flag.cover_flag == ON)Slaver_flag.global_clip = false;		

	if     (shoot_flag.friction_flag_top == ON)Slaver_flag.global_fiction = true;
	else if(shoot_flag.friction_flag_top == OFF)Slaver_flag.global_fiction = false;		

	if     (Small_Top_Mode_Flag == Little_TOP_ON)
	{
		if (Small_Top_Mode == SMALL_TOP_MODE_FAST)Slaver_flag.global_spin = 2;
	//	if(Small_Top_Mode == SMALL_TOP_MODE_S_F) Slaver_flag.global_spin = 2;
	}
	else                                  Slaver_flag.global_spin = 0;
	
	if(Vision_Mode != CMD_AIM_OFF)
	{
		if(Auto_Shoot())
		  Slaver_flag.global_auto_aim = 3;
		
		if(ZM_STATE() == ING)
			Slaver_flag.global_auto_aim = 1;
		
		else if(DF_STATE() == ING)
			Slaver_flag.global_auto_aim = 2;
	}
//	else if(Vision_Mode == CMD_AIM_OFF)
//		Slaver_flag.global_auto_aim = 0;
	
	if     (MODE_MAC_OR_TOP == MODE_MAC)Slaver_flag.move_mode = 0;
	else if(MODE_MAC_OR_TOP == MODE_TOP)Slaver_flag.move_mode = 1;

	if(ARRIVE())Slaver_flag.global_auto_arrive = 1;
	else        Slaver_flag.global_auto_arrive = 0;
	
	if(LOCK1 == 1)Slaver_flag.barrel_type = 1;
	else if(LOCK2 == 1)Slaver_flag.barrel_type = 2;
	else 
		Slaver_flag.barrel_type = 0;
}





/***********************************添加图层**********************************************/

/*工具*/

/*-发送工具-*/
void Usart5_Sent_Byte(uint8_t ch)
{
	UART5->DR = ch;
	while((UART5->SR&0x40)==0);
}

void Usart5_Sent_string(uint8_t *string, uint16_t length)
{
	uint16_t i;
	
	for(i = 0; i<length; i++)
	{
		Usart5_Sent_Byte(string[i]);
	}
}

void Client_Sent_String(uint8_t *string, uint16_t length)
{
	Usart5_Sent_string(string, length);
}


/*-绘制工具-*/
char empty_line[30] = {"                            "};
void Char_Graphic(ext_client_string_t* graphic,//最终要发出去的数组中的数据段内容
									const char* name,
									uint32_t operate_tpye,
									
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t length,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
								
									const char *character)//外部放入的数组
{
	graphic_data_struct_t *data_struct = &graphic->grapic_data_struct;
	for(char i=0;i<3;i++)
		data_struct->graphic_name[i] = name[i];	//字符索引
	data_struct->operate_tpye = operate_tpye; //图层操作
	data_struct->graphic_tpye = CHAR;         //Char型
	data_struct->layer = layer;//都在第零层
	data_struct->color = color;//都是白色
	data_struct->start_angle = size;
	data_struct->end_angle = length;	
	data_struct->width = width;
	data_struct->start_x = start_x;
	data_struct->start_y = start_y;	
	
	data_struct->radius = 0;
	data_struct->end_x = 0;
	data_struct->end_y = 0;
	
	memcpy(graphic->data,empty_line,28);
	memcpy(graphic->data,character,length);
}


void Figure_Graphic(graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
									uint32_t layer,
									uint32_t color,
									uint32_t start_angle,
									uint32_t end_angle,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									uint32_t radius,
									uint32_t end_x,
									uint32_t end_y)							
{
	
	for(char i=0;i<3;i++)
	graphic->graphic_name[i] = name[i];	//字符索引
	
	graphic->operate_tpye = operate_tpye; //图层操作
	graphic->graphic_tpye = graphic_tpye;         //Char型
	graphic->layer        = layer;//都在第一层
	graphic->color        = color;//变色
	graphic->start_angle  = start_angle;
	graphic->end_angle    = end_angle;	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->radius       = radius;
	graphic->end_x        = end_x;
	graphic->end_y        = end_y;
}

void Float_Graphic(Float_data_struct_t* graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t decimal,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									int number)							
{
	for(char i=0;i<3;i++)
		graphic->graphic_name[i] = name[i];	//字符索引
	graphic->operate_tpye = operate_tpye; //图层操作
	graphic->graphic_tpye = graphic_tpye;  
	graphic->layer        = layer;//
	graphic->color        = color;//变色
	graphic->start_angle  = size;
	graphic->end_angle    = decimal;//小数有效位	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->number       = number;
}

void Int_Graphic(Int_data_struct_t* graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t zero,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									int number)							
{
	for(char i=0;i<3;i++)
		graphic->graphic_name[i] = name[i];	//字符索引
	graphic->operate_tpye = operate_tpye; //图层操作
	graphic->graphic_tpye = graphic_tpye;        
	graphic->layer        = layer;//都在第一层
	graphic->color        = color;//变色
	graphic->start_angle  = size;
	graphic->end_angle    = zero;	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->number       = number;
}


///*********************************************/



ext_deleteLayer_data_t tx_client_delete;

ext_charstring_data_t tx_client_char;
uint8_t state_first_graphic;//0~7循环

ext_graphic_two_data_t tx_client_graphic_figure;
uint8_t update_figure_flag;

ext_graphic_five_data_t tx_aim_figure;//第三层放准心
uint8_t update_aim_flag;//1-add,3删除

ext_graphic_five_data_t tx_gimbalangle_figure;
uint8_t update_gimbalangle_flag;

ext_float_two_data_t tx_gimbal_angle;
uint8_t update_float_flag;

ext_int_two_data_t tx_bullet_int;
ext_int_two_data_t tx_chassis_speed_int;
ext_int_two_data_t tx_fw_int,tx_bw_int;
uint8_t update_int_flag;



/*设计*/

/*******************************绘字符串******************************/


char first_line[30]  = {"  permit fire:"};//是否可以射击,最多放30个字符串，bool
char second_line[30] = {"small top"};//小陀螺
char third_line[30]  = {"vision:"};//视觉
char fourth_line[30] = {"  MODE: MAC OR TOP"};
char fifth_line[30]  = {"flick"};//发弹量int

char sixth_line[30]  = {"speed_ok"};//射速int
char seventh_line[30]= {"cap"};//超级电容剩余量,float




static void Draw_char()
{
	if(state_first_graphic == 0)/*一直更新*/
	{
		Char_Graphic(&tx_client_char.clientData,"CL2",ADD,0,YELLOW,14,strlen(second_line),2,(AIM_X-130),(1080*9/12),second_line);
		state_first_graphic = 1;
	}
	else if(state_first_graphic == 1)
	{
		Char_Graphic(&tx_client_char.clientData,"CL4",ADD,0,YELLOW,14,strlen(fourth_line),2,(50),(1080*7/12),fourth_line);
		state_first_graphic = 2;
	}
	else if(state_first_graphic == 2)
	{
		Char_Graphic(&tx_client_char.clientData,"CL7",ADD,0,YELLOW,14,strlen(seventh_line),2,(1920-200),(1080*7/12),seventh_line);
		state_first_graphic = 0;
  }
}


/***************************绘制准心********************************/
uint32_t circle_angle = 0;
bool circle_dir = 0;

static void sight_bead_figrue(uint32_t x,uint32_t y)//可移动准心，请强制转换成uint32_t1920*1080有部分地区无法画出
{
	uint32_t circle_roll_x[4];
	uint32_t circle_roll_y[4];	

	
	if(!circle_dir){
	
		circle_angle = circle_angle+10;
		
	}
	else if(circle_dir){
	
		circle_angle = circle_angle-5;
		
	} 
	if(circle_angle == 50 || circle_angle == 0)circle_dir = !circle_dir;
	
	circle_roll_x[0] = circle_angle;
	circle_roll_x[1] = 180 - circle_roll_x[0] - 30;	
	circle_roll_x[2] = circle_roll_x[0] + 180;	
	circle_roll_x[3] = circle_roll_x[1] + 180;	
	
	circle_roll_y[0] = circle_roll_x[0] + 30;
	circle_roll_y[1] = circle_roll_x[1] + 30;	
	circle_roll_y[2] = circle_roll_x[2] + 30;	
	circle_roll_y[3] = circle_roll_x[3] + 30;	
	
	//舵机打开了
	if(Slaver_flag.global_clip == false){
		Figure_Graphic(&tx_aim_figure.clientData[0],"GR1",update_aim_flag,LINE,2,GREEN,0,0,4,  AIM_X-5,AIM_Y-60  ,0,  AIM_X+25,AIM_Y-90);

		Figure_Graphic(&tx_aim_figure.clientData[1],"GR2",update_aim_flag,LINE,2,GREEN,0,0,4,  AIM_X+25,AIM_Y-60  ,0,  AIM_X-5,AIM_Y-90);
		Figure_Graphic(&tx_aim_figure.clientData[2],"GR3",update_aim_flag,RECTANGLE,2,GREEN,0,0,4,  AIM_X,AIM_Y-60  ,0,  AIM_X+30,AIM_Y-90);
		Figure_Graphic(&tx_aim_figure.clientData[3],"GR4",DELETE,CIRCLE,2,GREEN,0,0,4,  AIM_X+15,AIM_Y  ,20,  0,0);		
		Figure_Graphic(&tx_aim_figure.clientData[4],"GR5",DELETE,CIRCLE,2,GREEN,0,0,4,  AIM_X+15,AIM_Y  ,20,  0,0);	
	}
	//摩擦轮没打开	
	if(Slaver_flag.global_fiction == false){
		
		Figure_Graphic(&tx_aim_figure.clientData[0],"GR1",update_aim_flag,LINE,2,YELLOW,0,0,4,  AIM_X,AIM_Y-60  ,0,  AIM_X+25,AIM_Y-90);
		Figure_Graphic(&tx_aim_figure.clientData[1],"GR2",update_aim_flag,LINE,2,YELLOW,0,0,4,  AIM_X+25,AIM_Y-60  ,0,  AIM_X,AIM_Y-90);
		Figure_Graphic(&tx_aim_figure.clientData[2],"GR3",update_aim_flag,RECTANGLE,2,YELLOW,0,0,4,  AIM_X,AIM_Y-60  ,0,  AIM_X+30,AIM_Y-90);
		Figure_Graphic(&tx_aim_figure.clientData[3],"GR4",DELETE,CIRCLE,2,GREEN,0,0,4,  AIM_X+15,AIM_Y  ,20,  0,0);		
		Figure_Graphic(&tx_aim_figure.clientData[4],"GR5",DELETE,CIRCLE,2,GREEN,0,0,4,  AIM_X+15,AIM_Y  ,20,  0,0);	
		
	}
	/*
	  视觉模式准星设置
	*/
	else if(Slaver_flag.global_fiction == true && Slaver_flag.global_clip == true)
	{
	  if(Slaver_flag.global_auto_aim == 3)
		{
				Figure_Graphic(&tx_aim_figure.clientData[0],"GR1",update_aim_flag,ARC,2,CYAN_BLUE,circle_roll_x[0],circle_roll_y[0],10,  AIM_X,AIM_Y  ,60,  100,100);
				Figure_Graphic(&tx_aim_figure.clientData[1],"GR2",update_aim_flag,ARC,2,CYAN_BLUE,circle_roll_x[1],circle_roll_y[1],10,  AIM_X,AIM_Y  ,60,  100,100);
				Figure_Graphic(&tx_aim_figure.clientData[2],"GR3",update_aim_flag,ARC,2,CYAN_BLUE,circle_roll_x[2],circle_roll_y[2],10,  AIM_X,AIM_Y  ,60,  100,100);
				Figure_Graphic(&tx_aim_figure.clientData[3],"GR4",update_aim_flag,ARC,2,CYAN_BLUE,circle_roll_x[3],circle_roll_y[3],10,  AIM_X,AIM_Y  ,60,  100,100);
		}

		else if(Slaver_flag.global_auto_aim != 3)
		{
			if(Slaver_flag.global_auto_aim == 1)
			{
//				Figure_Graphic(&tx_aim_figure.clientData[0],"GR1",update_aim_flag,ARC,2,ORANGE,50, 130, 8,  AIM_X+20,AIM_Y-75  ,30,  50,50);
//				Figure_Graphic(&tx_aim_figure.clientData[1],"GR2",update_aim_flag,ARC,2,ORANGE,230,310, 8,  AIM_X+20,AIM_Y-75  ,30,  50,50);
//				
//				Figure_Graphic(&tx_aim_figure.clientData[2],"GR3",DELETE,ARC,2,ORANGE,220,250,10,  AIM_X+20,AIM_Y-75  ,30,  50,50);
//				Figure_Graphic(&tx_aim_figure.clientData[3],"GR4",DELETE,ARC,2,ORANGE,110,140,10, AIM_X+20,AIM_Y-75  ,30,  50,50);	

	      Figure_Graphic(&tx_aim_figure.clientData[0],"GR1",update_aim_flag,LINE,2,WHITE,0,0,3,  AIM_X+15,AIM_Y-70  ,0,  AIM_X+15,AIM_Y-110);
			  Figure_Graphic(&tx_aim_figure.clientData[1],"GR2",update_aim_flag,LINE,2,WHITE,0,0,3,  AIM_X-5,AIM_Y-90	 ,0,  AIM_X+35,AIM_Y-90);
				Figure_Graphic(&tx_aim_figure.clientData[2],"GR3",DELETE,ARC,2,RED_BLUE,220,250,10,  AIM_X+20,AIM_Y-35  ,30,  50,50);
				Figure_Graphic(&tx_aim_figure.clientData[3],"GR4",DELETE,ARC,2,RED_BLUE,290,320,10,  AIM_X+20,AIM_Y-35  ,30,  50,50);	
				//Figure_Graphic(&tx_aim_figure.clientData[4],"GR5",DELETE,LINE,2,WHITE,0,0,3,  AIM_X-40,AIM_Y-45	 ,0,  AIM_X,AIM_Y-45);				
			}
			else if(Slaver_flag.global_auto_aim == 2)
			{
				Figure_Graphic(&tx_aim_figure.clientData[0],"GR1",update_aim_flag,ARC,2,RED_BLUE,50, 130, 8,  AIM_X,AIM_Y  ,30,  50,50);
				Figure_Graphic(&tx_aim_figure.clientData[1],"GR2",update_aim_flag,ARC,2,RED_BLUE,230,310, 8,  AIM_X,AIM_Y  ,30,  50,50);
				
				Figure_Graphic(&tx_aim_figure.clientData[2],"GR3",DELETE,ARC,2,RED_BLUE,220,250,10,  AIM_X,AIM_Y  ,30,  50,50);
				Figure_Graphic(&tx_aim_figure.clientData[3],"GR4",DELETE,ARC,2,RED_BLUE,110,140,10,  AIM_X,AIM_Y  ,30,  50,50);
			}
			else
			{
				Figure_Graphic(&tx_aim_figure.clientData[0],"GR1",update_aim_flag,LINE,2,WHITE,0,0,3,  AIM_X+15,AIM_Y-70  ,0,  AIM_X+15,AIM_Y-110);
			  Figure_Graphic(&tx_aim_figure.clientData[1],"GR2",update_aim_flag,LINE,2,WHITE,0,0,3,  AIM_X-5,AIM_Y-90	 ,0,  AIM_X+35,AIM_Y-90);
				Figure_Graphic(&tx_aim_figure.clientData[2],"GR3",DELETE,ARC,2,RED_BLUE,220,250,10,  AIM_X+20,AIM_Y-35  ,30,  50,50);
				Figure_Graphic(&tx_aim_figure.clientData[3],"GR4",DELETE,ARC,2,RED_BLUE,290,320,10,  AIM_X+20,AIM_Y-35  ,30,  50,50);							
			}
			
			
			if(Slaver_flag.global_auto_arrive)
			{
				Figure_Graphic(&tx_aim_figure.clientData[4],"GR5",update_aim_flag,CIRCLE,2,GREEN,0,0,4,  AIM_X+20,AIM_Y-75  ,20,  0,0);		
			}
			else
			{
				//Figure_Graphic(&tx_aim_figure.clientData[4],"GR5",update_aim_flag,LINE,2,WHITE,0,0,3,  AIM_X-60,AIM_Y-75	 ,0,  AIM_X-20,AIM_Y-75);
			}		
		}
	}
}
/************************************绘制图形*******************************/

char mode_F[30]  = {" Fast"};
char mode_F_S[30]  = {" Slow and Fast"};
char mode_NO[30]  = {"  No"};
static void spin_second_figure(char spin)//小陀螺打开为true
{
	if     (spin == 2)//匀速小陀螺为黄色          //200 1080*8/12
	{
		Figure_Graphic(&tx_client_graphic_figure.clientData[0],"GL1",update_figure_flag,CIRCLE,1,YELLOW,0,0,5,  AIM_X+40,1080*9/12, 20,0,0);
	  Char_Graphic(&tx_client_char.clientData,"CL2",ADD,0,YELLOW,14,strlen(mode_F),2,(AIM_X+65),(1080*9/12),mode_F);
	}
//	else if(spin == 2)//变速小陀螺为红蓝主色
//	{
//		Figure_Graphic(&tx_client_graphic_figure.clientData[0],"GL1",update_figure_flag,CIRCLE,1,RED_BLUE,0,0,5, AIM_X+40,1080*9/12, 20,0,0);
//	  Char_Graphic(&tx_client_char.clientData,"CL2",ADD,0,YELLOW,14,strlen(mode_F_S),2,(AIM_X+65),(1080*9/12),mode_F_S);
//	}
	else if(spin == 0)//没开小陀螺为白色
	{
		Figure_Graphic(&tx_client_graphic_figure.clientData[0],"GL1",update_figure_flag,CIRCLE,1,WHITE,0,0,5,  AIM_X+40,1080*9/12, 10,0,0);
		Char_Graphic(&tx_client_char.clientData,"CL2",ADD,0,YELLOW,14,strlen(mode_NO),2,(AIM_X+65),(1080*9/12),mode_NO);
	}
}


static void move_fourth_figure(char move)//
{
	if(move == 1)
		Figure_Graphic(&tx_client_graphic_figure.clientData[1],"GL2",update_figure_flag,CIRCLE,1,GREEN,0,0,5,  330,1080*7/12, 20,0,0);
	else if(move == 0)
		Figure_Graphic(&tx_client_graphic_figure.clientData[1],"GL2",update_figure_flag,CIRCLE,1,WHITE,0,0,5,  330,1080*7/12, 10,0,0);	
}





//电容只有一个图层
ext_graphic_one_data_t tx_supercapacitor_figure;
uint8_t update_supercapacitor_flag;
uint8_t volt_proportion;
static void supercapacitor_figure(float volt,float turning_volt)//剩余超级电容（单位百分比），低于某百分比变白色
{
	
	if(volt < CAP_OUTPUT_POWER_LOW)//直线长度为3
	{
		Figure_Graphic(&tx_supercapacitor_figure.clientData,"SR1",update_supercapacitor_flag,
	                  LINE,5,WHITE,   0,0,15,(uint32_t)((1900)-volt*volt*0.5f),585  ,0, 1900,585);
		
	}

	else if(volt >= CAP_OUTPUT_POWER_LOW)
	{
		Figure_Graphic(&tx_supercapacitor_figure.clientData,"SR1",update_supercapacitor_flag,
	                  LINE,5,RED_BLUE,0,0,15,(uint32_t)((1900)-volt*volt*0.5f),585  ,0, 1900,585);
	}
  //  volt_proportion = volt/24.5*100;
	//	Float_Graphic(&tx_gimbal_angle.clientData[0],"FR1",update_float_flag,FLOAT,4,YELLOW,10,2,3,(1920*6/7),500  ,volt_proportion);	
}


/***********************云台角度图形化表示********************************/

static void gimbalangle_figure(float gimbal_yaw)//
{
    float point_x[4] = {0};
    float point_y[4] = {0};
    arm_sin_cos_f32(gimbal_yaw + 45, point_y, point_x);
    arm_sin_cos_f32(gimbal_yaw + 135, point_y + 1, point_x + 1);
    arm_sin_cos_f32(gimbal_yaw - 135, point_y + 2, point_x + 2);
    arm_sin_cos_f32(gimbal_yaw - 45, point_y + 3, point_x + 3);
    for(uint8_t i = 0; i < 4; i++)
    {
        point_x[i] *= 60;
        point_y[i] *= 60;
    }
    
    Figure_Graphic(&tx_gimbalangle_figure.clientData[0],"GA0",update_gimbalangle_flag,LINE,6,CYAN_BLUE,0,0,3,\
            AIM_X+(int32_t)point_x[0],160+(int32_t)point_y[0],0,AIM_X+(int32_t)point_x[1],160+(int32_t)point_y[1]);

    Figure_Graphic(&tx_gimbalangle_figure.clientData[1],"GA1",update_gimbalangle_flag,LINE,6,YELLOW,0,0,3,\
            AIM_X+(int32_t)point_x[1],160+(int32_t)point_y[1],0,AIM_X+(int32_t)point_x[2],160+(int32_t)point_y[2]);

    Figure_Graphic(&tx_gimbalangle_figure.clientData[2],"GA2",update_gimbalangle_flag,LINE,6,YELLOW,0,0,3,\
            AIM_X+(int32_t)point_x[2],160+(int32_t)point_y[2],0,AIM_X+(int32_t)point_x[3],160+(int32_t)point_y[3]);

    Figure_Graphic(&tx_gimbalangle_figure.clientData[3],"GA3",update_gimbalangle_flag,LINE,6,YELLOW,0,0,3,\
            AIM_X+(int32_t)point_x[3],160+(int32_t)point_y[3],0,AIM_X+(int32_t)point_x[0],160+(int32_t)point_y[0]);

    Figure_Graphic(&tx_gimbalangle_figure.clientData[4],"GA4",update_gimbalangle_flag,LINE,6,RED_BLUE,0,0,3,\
            AIM_X,190,0,AIM_X,150);
      
}

/******************绘制浮点数*************************/
//第五层图层

static void gimbal_angle_float(float gimbal_pitch,float gimbal_yaw)//当前云台角度
{
	//青色pitch第一行，黄色yaw第二行
		Float_Graphic(&tx_gimbal_angle.clientData[0],"FR1",update_float_flag,FLOAT,4,CYAN_BLUE,30,2,3,(1920*4/6),810  ,(int)(gimbal_pitch*1000));
		Float_Graphic(&tx_gimbal_angle.clientData[1],"FR2",update_float_flag,FLOAT,4,YELLOW,   30,2,3,(1920*4/6),760  ,(int)(gimbal_yaw*1000));		
}

/**********************绘制int类型***************************/

static void bullet_int(int bullet_sum)//子弹射速和发弹量
{
	//总数量第一行，射速第二行
		Int_Graphic(&tx_bullet_int.clientData[0],"IR1",update_int_flag,INT,5,WHITE,30,0,3,(280),(540)  ,(int)bullet_sum);
		Int_Graphic(&tx_bullet_int.clientData[1],"IR2",update_int_flag,INT,5,WHITE,30,0,3,(280),(540)  ,(int)bullet_sum);		
}

static void chassis_speed_int(int chassis_speed_rpm_to_v)//子弹射速和发弹量
{
	//总数量第一行，射速第二行
		//Int_Graphic(&tx_bullet_int.clientData[0],"IR1",update_int_flag,INT,5,WHITE,30,0,3,(280),(540)  ,(int)bullet_sum);
		//Int_Graphic(&tx_bullet_int.clientData[1],"IR2",update_int_flag,INT,5,WHITE,30,0,3,(280),(540)  ,(int)bullet_sum);	

    Int_Graphic(&tx_chassis_speed_int.clientData[0],"FR1",update_int_flag,INT,5,YELLOW,25,0,3,(280),(540)  ,chassis_speed_rpm_to_v);		
}





/*****************************************************************/

static void Draw_Figure_bool()
{

	spin_second_figure(global_spin);

	move_fourth_figure(global_move);

}

/****************************************************************/




/*发送*/


void Client_graphic_Init()
{

		//帧头
		tx_client_char.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_client_char.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(ext_client_string_t);
		tx_client_char.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_client_char.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验
	
		//命令码
		tx_client_char.CmdID = ID_robot_interactive_header_data;
		
		//数据段头结构
		tx_client_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
		tx_client_char.dataFrameHeader.send_ID     = robot_id;
		tx_client_char.dataFrameHeader.receiver_ID = client_id;
		
		//数据段
		Draw_char();
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_char.CmdID, LEN_CMD_ID+tx_client_char.txFrameHeader.data_length);//加上命令码长度2
		
		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_client_char));
		
		Client_Sent_String(CliendTxBuffer, sizeof(tx_client_char));
}

//删除图层信息
void Client_graphic_delete_update(uint8_t delete_layer)//删除图层信息
{
		//帧头
		tx_client_delete.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_client_delete.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(ext_client_custom_graphic_delete_t);
		tx_client_delete.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_client_delete.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_client_delete.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_client_delete.dataFrameHeader.data_cmd_id = INTERACT_ID_delete_graphic;
		tx_client_delete.dataFrameHeader.send_ID     = robot_id;
		tx_client_delete.dataFrameHeader.receiver_ID = client_id;
		
		//数据段
		tx_client_delete.clientData.operate_type = ALL_delete;
		tx_client_delete.clientData.layer = delete_layer;
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_delete.CmdID, LEN_CMD_ID+tx_client_delete.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_client_delete));
		
		Client_Sent_String(CliendTxBuffer, sizeof(tx_client_delete));
}
/*
  2个图像一起更新
*/
void Client_graphic_Info_update(void)
{
		//帧头
		tx_client_graphic_figure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_client_graphic_figure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*2;
		tx_client_graphic_figure.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_client_graphic_figure.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_client_graphic_figure.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_client_graphic_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
		tx_client_graphic_figure.dataFrameHeader.send_ID     = robot_id;
		tx_client_graphic_figure.dataFrameHeader.receiver_ID = client_id;

		//数据段
		Draw_Figure_bool();
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_graphic_figure.CmdID, LEN_CMD_ID+tx_client_graphic_figure.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_client_graphic_figure));

		Client_Sent_String(CliendTxBuffer, sizeof(tx_client_graphic_figure));

}
/*
  数据更新
*/
int16_t chassis_speed_everage;

void Client_data_Info_update(void)
{
			//帧头
		tx_chassis_speed_int.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_chassis_speed_int.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(Int_data_struct_t);
		tx_chassis_speed_int.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_chassis_speed_int.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_chassis_speed_int.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_chassis_speed_int.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
		tx_chassis_speed_int.dataFrameHeader.send_ID     = robot_id;
		tx_chassis_speed_int.dataFrameHeader.receiver_ID = client_id;

		//数据段
	  chassis_speed_everage = -	(motor_chassis_gimbal[Left_front].speed_rpm
															+ motor_chassis_gimbal[Right_front].speed_rpm
													  	+	motor_chassis_gimbal[Left_back].speed_rpm
														  +	motor_chassis_gimbal[Right_back].speed_rpm) / 4;
		chassis_speed_int(chassis_speed_everage);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint16_t*)&tx_chassis_speed_int.CmdID, LEN_CMD_ID+tx_chassis_speed_int.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_chassis_speed_int));

		Client_Sent_String(CliendTxBuffer, sizeof(tx_chassis_speed_int));

}
/*
  5个图像一起更新
*/
void Client_aim_update()
{
		//帧头
		tx_aim_figure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_aim_figure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*5;
		tx_aim_figure.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_aim_figure.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_aim_figure.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_aim_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_five_graphic;
		tx_aim_figure.dataFrameHeader.send_ID     = robot_id;
		tx_aim_figure.dataFrameHeader.receiver_ID = client_id;
	
		//数据段
		sight_bead_figrue(global_sight_bead_x,global_sight_bead_y);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_aim_figure.CmdID, LEN_CMD_ID+tx_aim_figure.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_aim_figure));
		
		Client_Sent_String(CliendTxBuffer, sizeof(tx_aim_figure));
}
/*
  1个图像一起更新
*/
void Client_supercapacitor_update()
{
		//帧头
		tx_supercapacitor_figure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_supercapacitor_figure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t);
		tx_supercapacitor_figure.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_supercapacitor_figure.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_supercapacitor_figure.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_supercapacitor_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_one_graphic;
		tx_supercapacitor_figure.dataFrameHeader.send_ID     = robot_id;
		tx_supercapacitor_figure.dataFrameHeader.receiver_ID = client_id;
	
		//数据段
		supercapacitor_figure(global_supercapacitor_volt,global_supercapacitor_point);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_supercapacitor_figure.CmdID, LEN_CMD_ID+tx_supercapacitor_figure.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_supercapacitor_figure));
		
		Client_Sent_String(CliendTxBuffer, sizeof(tx_supercapacitor_figure));
}

/*
  5个图像一起更新
*/
void Client_gimbalangle_figure_update()
{
    //帧头
		tx_gimbalangle_figure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_gimbalangle_figure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t) * 5;
		tx_gimbalangle_figure.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_gimbalangle_figure.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_gimbalangle_figure.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_gimbalangle_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_five_graphic;
		tx_gimbalangle_figure.dataFrameHeader.send_ID     = robot_id;
		tx_gimbalangle_figure.dataFrameHeader.receiver_ID = client_id;
	
		//数据段
		gimbalangle_figure(global_gimbal_yaw);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_gimbalangle_figure.CmdID, LEN_CMD_ID+tx_gimbalangle_figure.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_gimbalangle_figure));
		
		Client_Sent_String(CliendTxBuffer, sizeof(tx_gimbalangle_figure));
}

/*
  5个图像一起更新(重复了)
*/
void Client_figure_update(graphic_data_struct_t graphic)
{
    //帧头
		tx_gimbalangle_figure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_gimbalangle_figure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t) * 5;
		tx_gimbalangle_figure.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_gimbalangle_figure.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_gimbalangle_figure.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_gimbalangle_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_five_graphic;
		tx_gimbalangle_figure.dataFrameHeader.send_ID     = robot_id;
		tx_gimbalangle_figure.dataFrameHeader.receiver_ID = client_id;
	
		//数据段
		gimbalangle_figure(global_gimbal_yaw);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_gimbalangle_figure.CmdID, LEN_CMD_ID+tx_gimbalangle_figure.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_gimbalangle_figure));
		
		Client_Sent_String(CliendTxBuffer, sizeof(tx_gimbalangle_figure));
}

void Client_gimbal_angle_update()//两个图像更新
{
		//帧头
		tx_gimbal_angle.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_gimbal_angle.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*2;
		tx_gimbal_angle.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_gimbal_angle.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_gimbal_angle.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_gimbal_angle.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
		tx_gimbal_angle.dataFrameHeader.send_ID     = robot_id;
		tx_gimbal_angle.dataFrameHeader.receiver_ID = client_id;
	
		//数据段
		gimbal_angle_float(global_gimbal_pitch,global_gimbal_yaw);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_gimbal_angle.CmdID, LEN_CMD_ID+tx_gimbal_angle.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_gimbal_angle));
		
		Client_Sent_String(CliendTxBuffer, sizeof(tx_gimbal_angle));
}

void Client_bullet_int_update()//两个图像更新
{
		//帧头
		tx_bullet_int.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		tx_bullet_int.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*2;
		tx_bullet_int.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&tx_bullet_int.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_bullet_int.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		tx_bullet_int.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
		tx_bullet_int.dataFrameHeader.send_ID     = robot_id;
		tx_bullet_int.dataFrameHeader.receiver_ID = client_id;
	
		//数据段
		bullet_int(global_bullet_sum);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_bullet_int.CmdID, LEN_CMD_ID+tx_bullet_int.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_bullet_int));
		
		Client_Sent_String(CliendTxBuffer, sizeof(tx_bullet_int));
}





/*准星*/
ext_graphic_seven_data_t high_aim_figure;//操作手准心之上,不更新
ext_graphic_seven_data_t low_aim_shortfigure_1;//准心下的第一个短线
ext_graphic_two_data_t low_aim_shortfigure_2;
ext_graphic_two_data_t  low_aim_shortfigure_3;//两个纵线
ext_graphic_five_data_t  low_aim_longfigure;//准心下的长横线
//!!!!!!!!!!!!!!!!!!全局变量
uint32_t division_value = 10;
//division_value分度值,line_length短线长度的一半
static void aim_1(uint32_t division_value,uint32_t line_length)//准心上半部分的宽度"AH"--aim_high
{
	Figure_Graphic(&high_aim_figure.clientData[0],"AH1",ADD,LINE,3,YELLOW,0,0,2,  AIM_X+15-line_length, AIM_Y-75+30                ,0,  AIM_X+15+line_length,  AIM_Y-75+30);//graphic_Remove
	Figure_Graphic(&high_aim_figure.clientData[1],"AH2",ADD,LINE,3,YELLOW,0,0,2,  AIM_X+15-line_length, AIM_Y-75+30+division_value  ,0,  AIM_X+15+line_length,  AIM_Y-75+30+division_value  );
	Figure_Graphic(&high_aim_figure.clientData[2],"AH3",ADD,LINE,3,YELLOW,0,0,2,  AIM_X+15-line_length, AIM_Y-75+30+division_value*2,0,  AIM_X+15+line_length,  AIM_Y-75+30+division_value*2);
	Figure_Graphic(&high_aim_figure.clientData[3],"AH4",ADD,LINE,3,YELLOW,0,0,2,  AIM_X+15-line_length, AIM_Y-75+30+division_value*3,0,  AIM_X+15+line_length,  AIM_Y-75+30+division_value*3);
	Figure_Graphic(&high_aim_figure.clientData[4],"AH5",ADD,LINE,3,YELLOW,0,0,2,  AIM_X+15-line_length, AIM_Y-75+30+division_value*4,0,  AIM_X+15+line_length,  AIM_Y-75+30+division_value*4);
	Figure_Graphic(&high_aim_figure.clientData[5],"AH6",ADD,LINE,3,YELLOW,0,0,2,  AIM_X+15-line_length, AIM_Y-75+30+division_value*5,0,  AIM_X+15+line_length,  AIM_Y-75+30+division_value*5);
	Figure_Graphic(&high_aim_figure.clientData[6],"AH7",ADD,LINE,3,YELLOW,0,0,2,  AIM_X+15-line_length, AIM_Y-75+30+division_value*6,0,  AIM_X+15+line_length,  AIM_Y-75+30+division_value*6);
}

void _high_aim_(void)
{
		//帧头
		high_aim_figure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		high_aim_figure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*7;
		high_aim_figure.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&high_aim_figure.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		high_aim_figure.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		high_aim_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
		high_aim_figure.dataFrameHeader.send_ID     = robot_id;
		high_aim_figure.dataFrameHeader.receiver_ID = client_id;
	
		//数据段
		aim_1(division_value,10);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&high_aim_figure.CmdID, LEN_CMD_ID+high_aim_figure.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(high_aim_figure));
		
		Client_Sent_String(CliendTxBuffer, sizeof(high_aim_figure));
}

static void aim_lowshort_2(uint32_t division_value,uint32_t line_length)//准心上半部分的宽度"AL"--aim_low
{
	Figure_Graphic(&low_aim_shortfigure_1.clientData[0],"AL1",ADD,LINE,3,ORANGE		,0,0,1,  AIM_X+15-line_length,	AIM_Y-75-30                 ,0,  AIM_X+15+line_length,		AIM_Y-75-30);//graphic_Remove
	Figure_Graphic(&low_aim_shortfigure_1.clientData[1],"AL2",ADD,LINE,3,ORANGE		,0,0,1,  AIM_X+15-line_length,	AIM_Y-75-30-division_value  ,0,  AIM_X+15+line_length,		AIM_Y-75-30-division_value  );                                                                                                      
	Figure_Graphic(&low_aim_shortfigure_1.clientData[2],"AL3",ADD,LINE,3,ORANGE		,0,0,1,  AIM_X+15-line_length,	AIM_Y-75-30-division_value*2,0,  AIM_X+15+line_length,		AIM_Y-75-30-division_value*2);
	Figure_Graphic(&low_aim_shortfigure_1.clientData[3],"AL4",ADD,LINE,3,ORANGE		,0,0,1,  AIM_X+15-line_length,	AIM_Y-75-30-division_value*4,0,  AIM_X+15+line_length,		AIM_Y-75-30-division_value*4);
	Figure_Graphic(&low_aim_shortfigure_1.clientData[4],"AL5",ADD,LINE,3,ORANGE		,0,0,1,  AIM_X+15-line_length,	AIM_Y-75-30-division_value*5,0,  AIM_X+15+line_length,		AIM_Y-75-30-division_value*5);                              
	Figure_Graphic(&low_aim_shortfigure_1.clientData[5],"AL6",ADD,LINE,3,CYAN_BLUE,0,0,1,  AIM_X+15-line_length,	AIM_Y-75-30-division_value*6,0,  AIM_X+15+line_length,		AIM_Y-75-30-division_value*6);
	Figure_Graphic(&low_aim_shortfigure_1.clientData[6],"AL7",ADD,LINE,3,GREEN		,0,0,1,  AIM_X+15-line_length,	AIM_Y-75-30-division_value*8,0,  AIM_X+15+line_length,		AIM_Y-75-30-division_value*8);
}
void _lowshort_aim_2()
{
		//帧头
		low_aim_shortfigure_1.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		low_aim_shortfigure_1.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*7;
		low_aim_shortfigure_1.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&low_aim_shortfigure_1.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		low_aim_shortfigure_1.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		low_aim_shortfigure_1.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
		low_aim_shortfigure_1.dataFrameHeader.send_ID     = robot_id;
		low_aim_shortfigure_1.dataFrameHeader.receiver_ID = client_id;
	
		//数据段
		aim_lowshort_2(division_value,10);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&low_aim_shortfigure_1.CmdID, LEN_CMD_ID+low_aim_shortfigure_1.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(low_aim_shortfigure_1));
		
		Client_Sent_String(CliendTxBuffer, sizeof(low_aim_shortfigure_1));
}
static void aim_lowshort_3(uint32_t division_value,uint32_t line_length)//准心上半部分的宽度"AM"--aim_low_middle
{
	Figure_Graphic(&low_aim_shortfigure_2.clientData[0],"AM1",ADD,LINE,3,WHITE,0,0,1,  AIM_X+15-line_length,AIM_Y-75-division_value*9 ,0,  AIM_X+15+line_length,	AIM_Y-75-division_value*9 );//graphic_Remove
	Figure_Graphic(&low_aim_shortfigure_2.clientData[1],"AM2",ADD,LINE,3,WHITE,0,0,1,  AIM_X+15-line_length,AIM_Y-75-division_value*10,0,  AIM_X+15+line_length,	AIM_Y-75-division_value*10);
}
void _lowshort_aim_3()
{
		//帧头
		low_aim_shortfigure_2.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		low_aim_shortfigure_2.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*2;
		low_aim_shortfigure_2.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&low_aim_shortfigure_2.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		low_aim_shortfigure_2.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		low_aim_shortfigure_2.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
		low_aim_shortfigure_2.dataFrameHeader.send_ID     = robot_id;
		low_aim_shortfigure_2.dataFrameHeader.receiver_ID = client_id;
	
		//数据段
		aim_lowshort_3(division_value,10);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&low_aim_shortfigure_2.CmdID, LEN_CMD_ID+low_aim_shortfigure_2.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(low_aim_shortfigure_2));
		
		Client_Sent_String(CliendTxBuffer, sizeof(low_aim_shortfigure_2));
}
//stem--茎
static void aim_lowshort_stem(uint32_t division_value,uint32_t line_length)//准心上半部分的宽度"AM"--aim_low_bottom,"AS"--aim_stem
{ 
	Figure_Graphic(&low_aim_shortfigure_3.clientData[0],"AS1",ADD,LINE,3,YELLOW,0,0,1,   AIM_X+15,AIM_Y-75+30+division_value*6 ,0,   AIM_X+15,            	AIM_Y-75+30);
	Figure_Graphic(&low_aim_shortfigure_3.clientData[1],"AS2",ADD,LINE,3,YELLOW,0,0,1,   AIM_X+15,AIM_Y-75-30-division_value*22,0,   AIM_X+15,            	AIM_Y-75-30);

}
void _lowshortstem_aim_4()
{
		//帧头
		low_aim_shortfigure_3.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		low_aim_shortfigure_3.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*2;
		low_aim_shortfigure_3.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&low_aim_shortfigure_3.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		low_aim_shortfigure_3.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		low_aim_shortfigure_3.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
		low_aim_shortfigure_3.dataFrameHeader.send_ID     = robot_id;
		low_aim_shortfigure_3.dataFrameHeader.receiver_ID = client_id;
	
		//数据段
		aim_lowshort_stem(division_value,10);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&low_aim_shortfigure_3.CmdID, LEN_CMD_ID+low_aim_shortfigure_3.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(low_aim_shortfigure_3));
		
		Client_Sent_String(CliendTxBuffer, sizeof(low_aim_shortfigure_3));
}
//图层四


static void aim_lowlong(uint32_t division_value,uint32_t line_length)//准心上半部分的宽度"AM"--aim_low_Long,"AS"--aim_stem
{ 
	Figure_Graphic(&low_aim_longfigure.clientData[0],"AL1",DELETE,LINE,4,YELLOW,0,0,1,AIM_X+15-line_length-30,AIM_Y-75-30-division_value*19,0,AIM_X+15+line_length+30,	AIM_Y-75-30-division_value*19);//graphic_Remove
	Figure_Graphic(&low_aim_longfigure.clientData[1],"AL2",DELETE,LINE,4,YELLOW,0,0,1,AIM_X+15-line_length-40,AIM_Y-75-30-division_value*15,0,AIM_X+15+line_length+40,	AIM_Y-75-30-division_value*15);
	Figure_Graphic(&low_aim_longfigure.clientData[2],"AL3",ADD,LINE,4,WHITE,0,0,2,		AIM_X+15-line_length-50,AIM_Y-75-30-division_value*11,0,AIM_X+15+line_length+50,	AIM_Y-75-30-division_value*11);
	Figure_Graphic(&low_aim_longfigure.clientData[3],"AL4",ADD,LINE,4,GREEN,0,0,2,		AIM_X+15-line_length-60,AIM_Y-75-30-division_value*7 ,0,AIM_X+15+line_length+60,	AIM_Y-75-30-division_value*7 );
	Figure_Graphic(&low_aim_longfigure.clientData[4],"AL5",ADD,LINE,4,ORANGE,0,0,2,		AIM_X+15-line_length-70,AIM_Y-75-30-division_value*3 ,0,AIM_X+15+line_length+70,	AIM_Y-75-30-division_value*3 );
	
}
void _lowlong_aim_()
{
		//帧头
		low_aim_longfigure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
		low_aim_longfigure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*5;
		low_aim_longfigure.txFrameHeader.seq = 0;//包序号
		memcpy(CliendTxBuffer,&low_aim_longfigure.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		low_aim_longfigure.CmdID = ID_robot_interactive_header_data;

		//数据段头结构
		low_aim_longfigure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_five_graphic;
		low_aim_longfigure.dataFrameHeader.send_ID     = robot_id;
		low_aim_longfigure.dataFrameHeader.receiver_ID = client_id;
	
		//数据段
		aim_lowlong(division_value,10);
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&low_aim_longfigure.CmdID, LEN_CMD_ID+low_aim_longfigure.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(low_aim_longfigure));
		
		Client_Sent_String(CliendTxBuffer, sizeof(low_aim_longfigure));
}


void Client_draw_aim(void)
{
	
}



///////////////////////////////////////////发送任务
extern judge_info_t judge_info;
void Client_info_update(void)
{
	//全是上云台
	global_fiction  = Slaver_flag.global_fiction;
	global_clip     = Slaver_flag.global_clip;
	global_spin     = Slaver_flag.global_spin;
	global_auto_aim = Slaver_flag.global_auto_aim;
	global_anti_top = Slaver_flag.global_anti_top;
	global_twist    = Slaver_flag.global_twist;
	global_move     = Slaver_flag.move_mode;
	
	global_gimbal_yaw =  360 - Limit_Target( motor_chassis_gimbal[Gimbal_yaw].angle - CHASSIS_MAC_FRONT_MID_VALUE) * 360 / 8192;

	global_supercapacitor_volt = CAP_RP_2023.RX->Cap_volt;
	global_supercapacitor_point = (uint32_t)(global_supercapacitor_volt / 24.5f * 100);
	
	robot_id  = judge_info.game_robot_status.robot_id;
	client_id = judge_info.self_client;
	
	if(global_supercapacitor_volt > 24.5f)
	{
		global_supercapacitor_volt = 24.5f;
	}

	global_supercapacitor_point = constrain(global_supercapacitor_point,1,100);

}
	

uint8_t Client_circle = 10;
void Up_Data(void)
{
	static uint32_t i;
	uint8_t num;
	
	num = i%Client_circle;

	Client_info_update();
	
	
	switch(num)
	{
		case 0:
		Client_graphic_Init();//不用一直更新，但是无法判断什么时候进入客户端所以需要轮询,可以操作手key控制			
		break;
	
		case 1:
		Client_graphic_Info_update();
		update_figure_flag = MODIFY;			
		break;	
	
		case 2:
		Client_supercapacitor_update();
		update_supercapacitor_flag = MODIFY;		
    	
		break;	
	
		case 3:
		Client_aim_update();
		update_aim_flag = MODIFY;			
		break;	

		case 4:
		Client_gimbalangle_figure_update();
		update_gimbalangle_flag = MODIFY;		
//		case 5:
//    Client_data_Info_update();		
//		update_int_flag = MODIFY;	
//		
		break;		
		
	}
	
 //准星部分
	num = i%200;
	switch(num)
	{
		case 1:
		_lowshortstem_aim_4();	
		break;
		
		case 10:
		_high_aim_();	
		break;
		
		case 20:
		_lowshort_aim_2();	
		break;
		
		case 30:
		_lowshort_aim_3();	
		break;

		case 40:
		_lowlong_aim_();
		break;		
	}	    

	if(i%100 == 0)
	{
		update_int_flag    = ADD;
		update_aim_flag    = ADD;		
		update_figure_flag = ADD;
		update_float_flag  = ADD;
		update_gimbalangle_flag    = ADD;	
		update_supercapacitor_flag = ADD;
	}
	
	i++;
	
}

	
	

void Client_task(void)
{
	if(POSITION_N == 1)
	{	
		Get_Client_Info();	
	}

	else if(POSITION_N == 0)
	{	
			Up_Data();			
	}
}







