/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "UI_task.h"
#include "ins_task.h"
#include "referee_usart_task.h"
#include "led_flow_task.h"
#include "gimbal_task.h"
//#include "calibrate_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

osThreadId imuTaskHandle;
osThreadId led_RGB_flow_handle;
osThreadId calibrate_tast_handle;
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
osThreadId PC_sendHandle;
osThreadId Gimbal_taskHandle;
osThreadId REFEREEHandle;
osThreadId Ui_taskHandle;
osThreadId USB_taskHandle;
osThreadId SolveTrajectoryHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void pc_send(void const * argument);
void gimbal_task(void const * argument);
void referee_usart_task(void const * argument);
void UI_task(void const * argument);
void usb_task(void const * argument);
extern void solvetrajectory_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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
  /* definition and creation of PC_send */
  osThreadDef(PC_send, pc_send, osPriorityNormal, 0, 128);
  PC_sendHandle = osThreadCreate(osThread(PC_send), NULL);

  /* definition and creation of Gimbal_task */
  osThreadDef(Gimbal_task, gimbal_task, osPriorityHigh, 0, 512);
  Gimbal_taskHandle = osThreadCreate(osThread(Gimbal_task), NULL);

  /* definition and creation of REFEREE */
  osThreadDef(REFEREE, referee_usart_task, osPriorityAboveNormal, 0, 128);
  REFEREEHandle = osThreadCreate(osThread(REFEREE), NULL);

  /* definition and creation of Ui_task */
  osThreadDef(Ui_task, UI_task, osPriorityNormal, 0, 256);
  Ui_taskHandle = osThreadCreate(osThread(Ui_task), NULL);

  /* definition and creation of USB_task */
  osThreadDef(USB_task, usb_task, osPriorityNormal, 0, 256);
  USB_taskHandle = osThreadCreate(osThread(USB_task), NULL);

  /* definition and creation of SolveTrajectory */
//  osThreadDef(SolveTrajectory, solvetrajectory_task, osPriorityNormal, 0, 256);
//  SolveTrajectoryHandle = osThreadCreate(osThread(SolveTrajectory), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(imuTask, INS_Task, osPriorityRealtime, 0, 1024);
	imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

//	osThreadDef(led, led_RGB_flow_task, osPriorityLow , 0, 256);
//	led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);
	
//	osThreadDef(cali, calibrate_task, osPriorityNormal, 0, 512);
//	calibrate_tast_handle = osThreadCreate(osThread(cali), NULL);//½ÃÕý

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_pc_send */
/**
  * @brief  Function implementing the PC_send thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_pc_send */
__weak void pc_send(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN pc_send */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END pc_send */
}

/* USER CODE BEGIN Header_gimbal_task */
/**
* @brief Function implementing the Gimbal_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_task */
__weak void gimbal_task(void const * argument)
{
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}

/* USER CODE BEGIN Header_referee_usart_task */
/**
* @brief Function implementing the REFEREE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_referee_usart_task */
__weak void referee_usart_task(void const * argument)
{
  /* USER CODE BEGIN referee_usart_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END referee_usart_task */
}

/* USER CODE BEGIN Header_UI_task */
/**
* @brief Function implementing the Ui_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UI_task */
__weak void UI_task(void const * argument)
{
  /* USER CODE BEGIN UI_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UI_task */
}

/* USER CODE BEGIN Header_usb_task */
/**
* @brief Function implementing the USB_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usb_task */
__weak void usb_task(void const * argument)
{
  /* USER CODE BEGIN usb_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usb_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
