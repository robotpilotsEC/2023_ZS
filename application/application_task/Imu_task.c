#include "Imu_task.h"
#include "cmsis_os.h"
#include "main.h"

void Start_imu_task(void const * argument)
{
  /* USER CODE BEGIN Start_imu_task */
  /* Infinite loop */
  for(;;)
  {
	
		  BMI_Get_RawData(&ggx,&ggy,&ggz,&aax,&aay,&aaz);
   	  BMI_Get_EulerAngle(&pitch,&roll,&yaw,&ggx,&ggy,&ggz,&aax,&aay,&aaz);
		
    osDelay(1);
  }
  /* USER CODE END Start_imu_task */
}
