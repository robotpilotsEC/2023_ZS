/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern int16_t ucSec; //秒计时
float usTms; //秒计时
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId MotorHandle;
osThreadId Chassis_TaskHandle;
osThreadId Gimbal_TaskHandle;
osThreadId Shoot_TaskHandle;
osThreadId Vision_TaskHandle;
osThreadId Imu_taskHandle;
osThreadId Rc_TaskHandle;
osThreadId Judge_TaskHandle;
osThreadId normal_taskHandle;
osThreadId supercap_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void motor(void const * argument);
void Start_chassis_task(void const * argument);
void Start_gimbal_Task(void const * argument);
void Start_shoot_task(void const * argument);
void Start_Vision_Task(void const * argument);
void Start_imu_task(void const * argument);
void rc_task(void const * argument);
extern void Judge_task(void const * argument);
void Normal_tasks(void const * argument);
void Supercap_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Motor */
  osThreadDef(Motor, motor, osPriorityNormal, 0, 256);
  MotorHandle = osThreadCreate(osThread(Motor), NULL);

  /* definition and creation of Chassis_Task */
  osThreadDef(Chassis_Task, Start_chassis_task, osPriorityNormal, 0, 128);
  Chassis_TaskHandle = osThreadCreate(osThread(Chassis_Task), NULL);

  /* definition and creation of Gimbal_Task */
  osThreadDef(Gimbal_Task, Start_gimbal_Task, osPriorityNormal, 0, 128);
  Gimbal_TaskHandle = osThreadCreate(osThread(Gimbal_Task), NULL);

  /* definition and creation of Shoot_Task */
  osThreadDef(Shoot_Task, Start_shoot_task, osPriorityNormal, 0, 128);
  Shoot_TaskHandle = osThreadCreate(osThread(Shoot_Task), NULL);

  /* definition and creation of Vision_Task */
  osThreadDef(Vision_Task, Start_Vision_Task, osPriorityNormal, 0, 128);
  Vision_TaskHandle = osThreadCreate(osThread(Vision_Task), NULL);

  /* definition and creation of Imu_task */
  osThreadDef(Imu_task, Start_imu_task, osPriorityNormal, 0, 128);
  Imu_taskHandle = osThreadCreate(osThread(Imu_task), NULL);

  /* definition and creation of Rc_Task */
  osThreadDef(Rc_Task, rc_task, osPriorityNormal, 0, 128);
  Rc_TaskHandle = osThreadCreate(osThread(Rc_Task), NULL);

  /* definition and creation of Judge_Task */
  osThreadDef(Judge_Task, Judge_task, osPriorityNormal, 0, 512);
  Judge_TaskHandle = osThreadCreate(osThread(Judge_Task), NULL);

  /* definition and creation of normal_task */
  osThreadDef(normal_task, Normal_tasks, osPriorityNormal, 0, 128);
  normal_taskHandle = osThreadCreate(osThread(normal_task), NULL);

  /* definition and creation of supercap_task */
  osThreadDef(supercap_task, Supercap_task, osPriorityNormal, 0, 128);
  supercap_taskHandle = osThreadCreate(osThread(supercap_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */

/* USER CODE END Header_StartDefaultTask */
extern char vision_enter;
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		if(vision_enter == 1)
    usTms++;
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_motor */
/**
* @brief Function implementing the Motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor */
void motor(void const * argument)
{
  /* USER CODE BEGIN motor */
	
  		
  /* Infinite loop */
  for(;;)
  {  

      osDelay(1);
  }
  /* USER CODE END motor */
}

/* USER CODE BEGIN Header_Start_chassis_task */
/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_chassis_task */
__weak void Start_chassis_task(void const * argument)
{
  /* USER CODE BEGIN Start_chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_chassis_task */
}

/* USER CODE BEGIN Header_Start_gimbal_Task */
/**
* @brief Function implementing the Gimbal_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_gimbal_Task */
__weak void Start_gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Start_gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_gimbal_Task */
}

/* USER CODE BEGIN Header_Start_shoot_task */
/**
* @brief Function implementing the Shoot_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_shoot_task */
__weak void Start_shoot_task(void const * argument)
{
  /* USER CODE BEGIN Start_shoot_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_shoot_task */
}

/* USER CODE BEGIN Header_Start_Vision_Task */
/**
* @brief Function implementing the Vision_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Vision_Task */
__weak void Start_Vision_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Vision_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Vision_Task */
}

/* USER CODE BEGIN Header_Start_imu_task */
/**
* @brief Function implementing the Imu_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_imu_task */
__weak void Start_imu_task(void const * argument)
{
  /* USER CODE BEGIN Start_imu_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_imu_task */
}

/* USER CODE BEGIN Header_rc_task */
/**
* @brief Function implementing the Rc_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rc_task */
__weak void rc_task(void const * argument)
{
  /* USER CODE BEGIN rc_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END rc_task */
}

/* USER CODE BEGIN Header_Normal_tasks */
/**
* @brief Function implementing the normal_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Normal_tasks */
__weak void Normal_tasks(void const * argument)
{
  /* USER CODE BEGIN Normal_tasks */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Normal_tasks */
}

/* USER CODE BEGIN Header_Supercap_task */
/**
* @brief Function implementing the supercap_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Supercap_task */
__weak void Supercap_task(void const * argument)
{
  /* USER CODE BEGIN Supercap_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Supercap_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
