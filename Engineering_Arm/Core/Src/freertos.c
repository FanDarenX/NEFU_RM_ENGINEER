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
#include "position_task.h"
#include "yaw_task.h"
#include "custom.h"
#include "keyscan_task.h"
#include "image_task.h"
#include "chassis_task.h"
#include "UI_task.h"
#include "referee_usart_task.h"
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

/* USER CODE END Variables */
osThreadId POSITION_TASKHandle;
osThreadId YAW_6020_TASKHandle;
osThreadId KEYSCAN_TASKHandle;
osThreadId CUSTOM_TASKHandle;
osThreadId IMAGE_TASKHandle;
osThreadId CHASSIS_TASKHandle;
osThreadId REFEREE_USARTHandle;
osThreadId UI_TASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Position_Task(void const * argument);
void Yaw_Task(void const * argument);
void keyscan_task(void const * argument);
void Custom_Task(void const * argument);
void Image_Task(void const * argument);
void Chassis_Task(void const * argument);
void referee_usart_task(void const * argument);
void UI_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
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
  /* definition and creation of POSITION_TASK */
  osThreadDef(POSITION_TASK, Position_Task, osPriorityNormal, 0, 128);
  POSITION_TASKHandle = osThreadCreate(osThread(POSITION_TASK), NULL);

  /* definition and creation of YAW_6020_TASK */
  osThreadDef(YAW_6020_TASK, Yaw_Task, osPriorityNormal, 0, 128);
  YAW_6020_TASKHandle = osThreadCreate(osThread(YAW_6020_TASK), NULL);

  /* definition and creation of KEYSCAN_TASK */
  osThreadDef(KEYSCAN_TASK, keyscan_task, osPriorityNormal, 0, 256);
  KEYSCAN_TASKHandle = osThreadCreate(osThread(KEYSCAN_TASK), NULL);

  /* definition and creation of CUSTOM_TASK */
  osThreadDef(CUSTOM_TASK, Custom_Task, osPriorityNormal, 0, 128);
  CUSTOM_TASKHandle = osThreadCreate(osThread(CUSTOM_TASK), NULL);

  /* definition and creation of IMAGE_TASK */
  osThreadDef(IMAGE_TASK, Image_Task, osPriorityNormal, 0, 128);
  IMAGE_TASKHandle = osThreadCreate(osThread(IMAGE_TASK), NULL);

  /* definition and creation of CHASSIS_TASK */
  osThreadDef(CHASSIS_TASK, Chassis_Task, osPriorityNormal, 0, 512);
  CHASSIS_TASKHandle = osThreadCreate(osThread(CHASSIS_TASK), NULL);

  /* definition and creation of REFEREE_USART */
  osThreadDef(REFEREE_USART, referee_usart_task, osPriorityNormal, 0, 256);
  REFEREE_USARTHandle = osThreadCreate(osThread(REFEREE_USART), NULL);

  /* definition and creation of UI_TASK */
  osThreadDef(UI_TASK, UI_task, osPriorityNormal, 0, 256);
  UI_TASKHandle = osThreadCreate(osThread(UI_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Position_Task */
/**
 * @brief  Function implementing the POSITION_TASK thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Position_Task */
void Position_Task(void const * argument)
{
  /* USER CODE BEGIN Position_Task */
  osDelay(357);
  position_init();
  /* Infinite loop */
  for (;;)
  {
    position_task();
    osDelay(1);
  }
  /* USER CODE END Position_Task */
}

/* USER CODE BEGIN Header_Yaw_Task */
/**
 * @brief Function implementing the YAW_6020_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Yaw_Task */
void Yaw_Task(void const * argument)
{
  /* USER CODE BEGIN Yaw_Task */
  osDelay(357);
  TD_INIT();
  /* Infinite loop */
  for (;;)
  {
    TD_task();
    osDelay(1);
  }
  /* USER CODE END Yaw_Task */
}

/* USER CODE BEGIN Header_keyscan_task */
/**
 * @brief Function implementing the KEYSCAN_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_keyscan_task */
__weak void keyscan_task(void const * argument)
{
  /* USER CODE BEGIN keyscan_task */
  //  /* Infinite loop */
  //  for(;;)
  //  {
  //    osDelay(1);
  //  }
  /* USER CODE END keyscan_task */
}

/* USER CODE BEGIN Header_Custom_Task */
/**
 * @brief Function implementing the CUSTOM_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Custom_Task */
void Custom_Task(void const * argument)
{
  /* USER CODE BEGIN Custom_Task */
  osDelay(357);
	/* 在UART8初始化完成后添加 */
//	__HAL_UART_ENABLE_IT(&huart8, UART_IT_RXNE);  // 接收中断
HAL_UARTEx_ReceiveToIdle_DMA(&huart8, receive_data, receive_size);
	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);

  
  /* Infinite loop */
  for (;;)
  {
    Custom_task();
    osDelay(1);
  }
  /* USER CODE END Custom_Task */
}

/* USER CODE BEGIN Header_Image_Task */
/**
 * @brief Function implementing the IMAGE_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Image_Task */
void Image_Task(void const * argument)
{
  /* USER CODE BEGIN Image_Task */
  osDelay(357);
  image_task_init(&image_task);
  /* Infinite loop */
  for (;;)
  {
    image_mode_task();
    osDelay(1);
  }
  /* USER CODE END Image_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
 * @brief Function implementing the CHASSIS_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Chassis_Task */
void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
  chassis_init();
  /* Infinite loop */
  for (;;)
  {
    Chassis_Task_os();
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_referee_usart_task */
/**
 * @brief Function implementing the REFEREE_USART_T thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_referee_usart_task */
__weak void referee_usart_task(void const * argument)
{
  /* USER CODE BEGIN referee_usart_task */
  /* Infinite loop */
//  for (;;)
//  {
//    osDelay(1);
//  }
  /* USER CODE END referee_usart_task */
}

/* USER CODE BEGIN Header_UI_task */
/**
 * @brief Function implementing the UI_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UI_task */
__weak void UI_task(void const * argument)
{
  /* USER CODE BEGIN UI_task */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END UI_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
