/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.蜂鸣器报警任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "referee.h"
#include "bsp_usart.h"
#include "decet_task.h"
#include "ins_task.h"
#include "shoot.h"
#include "ANO_DT.h"
/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
uint16_t temp_bullet=0;
uint8_t send_flag=0;
uint8_t send_HW[4]={0x80, 0x06, 0x03, 0x77};
void pc_send(void const * argument)
{
	int i=10;
	   vTaskDelay(1000);
		 while(i)
		 {
		  usart_state = HAL_UART_Transmit(&huart1, send_HW, 4,10);
			if(usart_state != HAL_OK) //刚上电会发送失败，重新初始化串口1
	    MX_USART1_UART_Init();
			i--;
		 }
    while(1)
    {
			char char_temp;
      vTaskDelay(5000);
    }
}



