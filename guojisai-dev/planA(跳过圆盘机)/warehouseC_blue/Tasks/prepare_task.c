#include "prepare_task.h"
#include "task.h"
#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os.h"

//void Prepare_Task(void const * argument)
//{
//    /* init code for USB_DEVICE */
//    MX_USB_DEVICE_Init();
//    /* USER CODE BEGIN Prepare_Task */
//
//    osThreadDef(cali, calibrate_task, osPriorityNormal, 0, 512);
//    calibrateTaskHandle = osThreadCreate(osThread(cali), NULL);
//
//    osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 512);
//    imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);
//
//    /* Infinite loop */
//    for(;;)
//    {
//        if(my_cali_flag == 1)
//        {
//            vTaskDelay(100);
//            vTaskDelete(calibrateTaskHandle);
//            vTaskDelay(100);
//
//            osThreadDef(ChassisTask, chassis_task, osPriorityAboveNormal, 0, 512);
//            chassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);
//
//            osThreadDef(Bodanpan_Task_,Bodanpan_Task,osPriorityBelowNormal, 0, 128);
//            BodanpanTaskHandle = osThreadCreate(osThread(Bodanpan_Task_), NULL);
//
//            osThreadDef(armTask, arm_task, osPriorityNormal, 0, 512);
//            armTaskHandle = osThreadCreate(osThread(armTask), NULL);
//
//            osThreadDef(flowTask, flow_task, osPriorityNormal, 0, 512);
//            flowTaskHandle = osThreadCreate(osThread(flowTask), NULL);
//
//            osThreadDef(armControlTask, arm_control_task, osPriorityNormal, 0, 256);
//            armControlTaskHandle = osThreadCreate(osThread(armControlTask), NULL);
//
//            vTaskDelay(100);
//            vTaskDelete(NULL);
//        }
//
//
//        osDelay(1);
//    }