#ifndef __S_FUNCTION_H
#define __S_FUNCTION_H

#include "math_support.h"
#include "rc.h"


float Limit_Target(float tar);

//float UP_FX(float x);
//float DO_FX(float x);			
//				
//float FRI_FX(float x,int max);
//	
float SF(float t,float *slopeFilter,float res);
float SF_2(float t,float *slopeFilter,float res,int length);//滑动滤波；		

//int Get_Symbol(float num);//获取±性		
//float JUDGE_NULL(float last, float now);			

						
#endif


