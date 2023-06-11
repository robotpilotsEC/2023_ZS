#include "S_function.h"
#include "State.h"


/*
		限制目标值 防止超过8191
*/
float Limit_Target(float tar)
{
	if(tar < 0)         tar += 8191;
	else if(tar >=8191) tar -= 8191;
	return tar;
}




float Half_Turn(float angle,float max)
{
	if(abs(angle) > (max/2))
	{	
		if(angle >= 0)
			angle += -max;		
		else
			angle +=  max;
	}
	return angle;
}








//反馈值为，对6020建立的直角坐标系
//void MID_RESET(motor_t *motor, int16_t MID, uint16_t data)
//{
//	int16_t angle = 0;
//	
//	if(data < MID)
//		 angle = data - MID + 8191;
//	else
//	   angle = data - MID;
//	
//  angle = -angle + 8191 + 2047;
//	
//	if(angle > 8191)angle = angle - 8191;

//	angle = Limit_Target(angle - 2047);
//	
//	motor->info->angle = angle;
//			
//}

//float UP_FX(float x)
//{
//	if(x == 200)return 1;
//  else return (1/(1 + exp(-10 * x/200 + 2)));
//	
//	else return x/200;
//}

//float DO_FX(float x)
//{
//	if(x == 200)return 0;
//	else return (1/(1 + exp(+10 * x/200 - 3)));
//}

//float FRI_FX(float x,int max)
//{
//	if(x > max)return 1;
//  else return (x / max);
//}

float SF(float t,float *slopeFilter,float res)
{
  for(int i = SF_LENGTH-1;i>0;i--)
  {
    slopeFilter[i] = slopeFilter[i-1];
  }slopeFilter[0] = t;
  for(int i = 0;i<SF_LENGTH;i++)
  {
    res += slopeFilter[i];
  }
	return (res/SF_LENGTH);
}

float SF_2(float t,float *slopeFilter,float res,int length)//滑动滤波
{
  for(int i = length-1;i>0;i--)
  {
    slopeFilter[i] = slopeFilter[i-1];
  }slopeFilter[0] = t;
  for(int i = 0;i<length;i++)
  {
    res += slopeFilter[i];
  }return (res/length);
}

int Get_Symbol(float num)
{
	int symbol;
	if     (num > 0)symbol = +1;
	else if(num < 0)symbol = -1;
	else if(num== 0)symbol =  0;
	
	return symbol;
}


float JUDGE_NULL(float last, float now)
{
	if(last != 0)
	{
		if(now == NULL)
		{
			now = last;
		}
	}
	return now;
}
