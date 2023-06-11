
/**
  ******************************************************************************
  * @file			pid.c
  * @version		V1.0.0
  * @date			
  * @brief   		对于PID， 反馈/测量习惯性叫get/measure/real/fdb,
						  期望输入一般叫set/target/ref
  *******************************************************************************/
  
  
  
/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include <math.h>
#include "math_support.h"
#include "CAN_receive.h"
#include "rc.h"

#define ABS(x)		((x>0)? (x): (-x)) 

void abs_limit(float *a, float ABS_MAX){
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

/*pid总体初始化-----------------------------------------------------------------*/
void PID_struct_init(pid_t* pid, uint32_t maxout, uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd)
{
	  pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;

}


/*
	pitch云台陀螺仪模式PID(位置环)
*/
void pid_calc_out_gimbal_pitch(pid_t* pid, float measure, float target)
{  
	  pid->measure = measure; //measure
    pid->target = target;
    
	  if(pid->target >= 36)
		{
			pid->target = 36;
		}
		else if(pid->target <= -25)
		{
			pid->target = -25;
		}
		
		pid->err[2] = target - measure;	//target- measure
	  pid->pout = pid->p * pid->err[2];  //kp*err
		pid->iout += pid->i * pid->err[2];  //ki*err
		abs_limit(&(pid->iout), pid->IntegralLimit);
		pid->pos_out = pid->pout + pid->iout + pid->dout ;
		abs_limit(&(pid->pos_out), pid->MaxOutput);
		pid->last_pos_out = pid->pos_out;	//update last time
}



/*
	yaw云台机械模式PID（位置环）
*/
void pid_calc_out_mac_yaw(pid_t* pid, float measure, float target){
   
	  pid->measure = measure; //measure
    pid->target = target;
	
		//误差处理要在过零点的前面，要不然过零点处理无效
    pid->err[2] = pid->target - pid->measure;
   //机械角度过零点检测
		if( pid->err[2] < -4096)  
		{
			 pid->err[2] = pid->err[2] + 8192;
	  }
	  else if( pid->err[2] > 4096)  
		{
			pid->err[2] = pid->err[2] - 8192;
		}
		
		 	  
	  pid->pout = pid->p * pid->err[2];  //kp*err
		pid->iout += pid->i * pid->err[2];  //ki*err
		abs_limit(&(pid->iout), pid->IntegralLimit);
		pid->pos_out = pid->pout + pid->iout + pid->dout ;
		abs_limit(&(pid->pos_out), pid->MaxOutput);
		pid->last_pos_out = pid->pos_out;	//update last time 
}


/*
	yaw云台陀螺仪模式PID（位置环）
*/
void pid_calc_out_follow_yaw(pid_t* pid, float measure, float target){
   
	  pid->measure = measure; //measure
    pid->target = target;
 
	
 //陀螺仪过零点检测		 
			
		while(pid->target >= 180 || pid->target < -180) //此语句超级重要，不加就会疯车
		{	
			if(pid->target >= 180) 
				pid->target -=  360;
			
			else if(pid->target < -180)	
				pid->target += 360;
		}
		
		pid->err[2] = pid->target - pid->measure;
		
		while(pid->err[2] >= 180 || pid->err[2] < -180) 
		{	 
			if(pid->target - pid->measure >= 180) 
				pid->err[2] = pid->target - pid->measure - 360;
			
			else if(pid->target - pid->measure < -180) 
				pid->err[2] = pid->target - pid->measure + 360;
			
			else
				pid->err[2] = pid->target - pid->measure;	 
	  }
	 
	  pid->pout = pid->p * pid->err[2];  //kp*err

		pid->pos_out = pid->pout;
		abs_limit(&(pid->pos_out), pid->MaxOutput);
		pid->last_pos_out = pid->pos_out;	//update last time 
   }



/*
	底盘PID（速度环）
*/
void pid_calc(pid_t* pid, float measure, float target){

	  pid->measure = measure; //measure
    pid->target = target;
    pid->err[2] = target - measure;	//target- measure
	
	  pid->pout = pid->p * pid->err[2];  //kp*err
		pid->iout += pid->i * pid->err[2];  //ki*err
		abs_limit(&(pid->iout), pid->IntegralLimit);
		pid->pos_out = pid->pout + pid->iout + pid->dout ;
		abs_limit(&(pid->pos_out), pid->MaxOutput);
		pid->last_pos_out = pid->pos_out;	//update last time 
}
   

/*
	摩擦轮PID(速度环)
*/

void pid_calc_shoot_friction(pid_t* pid, float measure, float target)
{
	  pid->measure = measure; //measure
    pid->target = target;
    pid->err[2] = target - measure;	//target- measure
	
	  pid->pout = pid->p * pid->err[2];  //kp*err
		pid->iout += pid->i * pid->err[2];  //累加ki*err
		abs_limit(&(pid->iout), pid->IntegralLimit);
		pid->pos_out = pid->pout + pid->iout + pid->dout ;
		abs_limit(&(pid->pos_out), pid->MaxOutput);
		pid->last_pos_out = pid->pos_out;	//update last time 
}


/*
	拨弹轮PID（速度环 连发）
*/
void pid_calc_shoot_continuous_flick(pid_t* pid, float measure, float target)
{
	  pid->measure = measure; //measure
    pid->target = target;
    pid->err[2] = target - measure;	//target- measure
	
	  pid->pout = pid->p * pid->err[2];  //kp*err
		pid->iout += pid->i * pid->err[2];  //ki*err
		abs_limit(&(pid->iout), pid->IntegralLimit);
		pid->pos_out = pid->pout + pid->iout + pid->dout ;
		abs_limit(&(pid->pos_out), pid->MaxOutput);
		pid->last_pos_out = pid->pos_out;	//update last time 
}
/*
	拨弹轮PID（速度环 单发）
*/
void pid_calc_shoot_single_flick(pid_t* pid, float measure, float target)
{
	  pid->measure = measure; //measure
    pid->target = target;
    pid->err[2] = target - measure;	//target- measure
	
	  pid->pout = pid->p * pid->err[2];  //kp*err
		pid->iout += pid->i * pid->err[2];  //ki*err
		abs_limit(&(pid->iout), pid->IntegralLimit);
		pid->pos_out = pid->pout + pid->iout + pid->dout ;
		abs_limit(&(pid->pos_out), pid->MaxOutput);
		pid->last_pos_out = pid->pos_out;	//update last time 
}
/*
	拨弹轮/换枪管 串级PID（单发/切换枪管）
*/
void cascade_pid_ctrl_shoot(pid_t *outpid,pid_t *inpid,float out_measure,float in_measure,float target)
{
		/*目标值与测量值需要在外面给出*/
		pid_t *outPid = outpid;
		pid_t *inPid = inpid;
	
		outPid->target = target;
		outPid->measure = out_measure;
	  inPid->measure = in_measure;
	 
		// 外环PID
				
		pid_calc_shoot_single_flick(outpid,outPid->measure,outPid->target);
		
		// 内环期望 = 外环输出
		inPid->target = outPid->pos_out;
		// 内环PID
		inPid->err[2] =  inPid->target - inPid->measure;
		
	  pid_calc_shoot_single_flick(inpid,inPid->measure,inPid->target);
}

/*
	底盘跟随独立PID(位置环)
*/
void pid_calc_follow(pid_t* pid, float measure, float target)
{
   
	//加入下云台电机过零点检测 measure = 测量角度(0-8192) target = 2060
	  pid->measure = measure; //measure
    pid->target = target;
    pid->err[2] = target - measure;	//target- measure
		
    if( pid->err[2] < -4096)  
		{
			pid->err[2] = pid->err[2] + 8192;
		}
		else if( pid->err[2] > 4096)  
		{
			pid->err[2] = pid->err[2] - 8192;
		}
	  
		pid->pout = pid->p * pid->err[2];//kp*err
		abs_limit(&(pid->iout), pid->IntegralLimit);
		pid->pos_out = (pid->pout + pid->iout + pid->dout);//待测试
		abs_limit(&(pid->pos_out), pid->MaxOutput);
		pid->last_pos_out = pid->pos_out;	//update last time 
	
}


/*
	yaw云台串级PID(双环 陀螺仪)
*/
void cascade_pid_ctrl_gimbal_yaw(pid_t *outpid,pid_t *inpid,float out_measure,float in_measure,float target)
{
		/*目标值与测量值需要在外面给出*/
		pid_t *outPid = outpid;
		pid_t *inPid = inpid;
	
		outPid->target = target;
		outPid->measure = out_measure;
	  inPid->measure = in_measure;
	 
		// 外环PID
				
		pid_calc_out_follow_yaw(outpid,outPid->measure,outPid->target);
		
		// 内环期望 = 外环输出
		inPid->target = outPid->pos_out;
		// 内环PID
		inPid->err[2] =  inPid->target - inPid->measure;
		
	  pid_calc(inpid,inPid->measure,inPid->target);
}

/*
	pitch云台陀螺仪模式串级PID（双环）
*/
void cascade_pid_ctrl_gimbal_pitch(pid_t *outpid,pid_t *inpid,float out_measure,float in_measure,float target)
{
		/*目标值与测量值需要在外面给出*/
		pid_t *outPid = outpid;
		pid_t *inPid = inpid;
	
		outPid->target = target;
		outPid->measure = out_measure;
	  inPid->measure = in_measure;
	 
		// 外环PID
				
		pid_calc_out_gimbal_pitch(outpid,outPid->measure,outPid->target);
		
		// 内环期望 = 外环输出
		inPid->target = outPid->pos_out;
		// 内环PID
		inPid->err[2] =  inPid->target - inPid->measure;
		
	  pid_calc(inpid,inPid->measure,inPid->target);
}


/*
	yaw云台机械模式串级PID（双环）
*/
void cascade_pid_ctrl_mac_yaw(pid_t *outpid,pid_t *inpid,float out_measure,float in_measure,float target)
{
		/*目标值与测量值需要在外面给出*/
		pid_t *outPid = outpid;
		pid_t *inPid = inpid;
	
		outPid->target = target;
		outPid->measure = out_measure;
	  inPid->measure = in_measure;
	 
		// 外环PID
				
		pid_calc_out_mac_yaw(outpid,outPid->measure,outPid->target);
		
		// 内环期望 = 外环输出
		inPid->target = outPid->pos_out;
		// 内环PID
		inPid->err[2] =  inPid->target - inPid->measure;
		
	  pid_calc(inpid,inPid->measure,inPid->target);
}
/*
	底盘串级PID/机械模式pitch云台串级PID
*/
void cascade_pid_ctrl(pid_t *outpid,pid_t *inpid,float out_measure,float in_measure,float target)
{
		/*目标值与测量值需要在外面给出*/
		pid_t *outPid = outpid;
		pid_t *inPid = inpid;
	
		outPid->target = target;
		outPid->measure = out_measure;
	  inPid->measure = in_measure;
	 
		// 外环PID
				
		pid_calc(outpid,outPid->measure,outPid->target);
		
		// 内环期望 = 外环输出
		inPid->target = outPid->pos_out;
		// 内环PID
		inPid->err[2] =  inPid->target - inPid->measure;
		
	  pid_calc(inpid,inPid->measure,inPid->target);
}







pid_t pid_pos[4];
pid_t pid_spd[8];
pid_t pid_shoot_left_friction_wheel_spd;
pid_t pid_shoot_right_friction_wheel_spd;
pid_t pid_shoot_flick_wheel_follow_spd;
pid_t pid_shoot_flick_wheel_follow_pos;
pid_t pid_shoot_flick_wheel_mac_spd;
pid_t pid_shoot_flick_wheel_mac_pos;
pid_t pid_shoot_change_barrel_spd;
pid_t pid_shoot_change_barrel_pos;
pid_t pid_shoot_flick_wheel_jam_spd;
pid_t pid_shoot_flick_wheel_jam_pos;
pid_t pid_shoot_flick_wheel_stop_pos;
pid_t pid_shoot_flick_wheel_stop_spd;
pid_t pid_spd_follow;

