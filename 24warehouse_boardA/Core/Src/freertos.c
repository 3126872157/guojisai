/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

#include "calibrate_task.h"
#include "Chassis_Task.h"
#include "INS_task.h"
#include "Bodanpan_Task.h"
#include "arm_task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osThreadId calibrate_tast_handle;
osThreadId chassisTaskHandle;
osThreadId imuTaskHandle;
osThreadId BodanpanTaskHandle;
osThreadId armTaskHandle;

/* USER CODE END Variables */
osThreadId Prepare_Task_Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void arm_task(void const * argument);

/* USER CODE END FunctionPrototypes */

void Prepare_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* definition and creation of Prepare_Task_ */
  osThreadDef(Prepare_Task_, Prepare_Task, osPriorityIdle, 0, 128);
  Prepare_Task_Handle = osThreadCreate(osThread(Prepare_Task_), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  
	osThreadDef(cali, calibrate_task, osPriorityNormal, 0, 512);
    calibrate_tast_handle = osThreadCreate(osThread(cali), NULL);
	
	osThreadDef(ChassisTask, chassis_task, osPriorityAboveNormal, 0, 512);
    chassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);
	
	osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 1024);
    imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);
	
	osThreadDef(Bodanpan_Task_,Bodanpan_Task,osPriorityBelowNormal, 0, 512);
    BodanpanTaskHandle = osThreadCreate(osThread(Bodanpan_Task_), NULL);
	
	osThreadDef(armTask, arm_task, osPriorityNormal, 0, 512);
	armTaskHandle = osThreadCreate(osThread(armTask), NULL);
	
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Prepare_Task */
/**
  * @brief  Function implementing the Prepare_Task_ thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Prepare_Task */
__weak void Prepare_Task(void const * argument)
{
  /* USER CODE BEGIN Prepare_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Prepare_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

__weak void arm_task(void const * argument)
{
	while(1)
	{
		osDelay(1);
	}
}

/* USER CODE END Application */
