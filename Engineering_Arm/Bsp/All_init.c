/*
 * @Descripttion:
 * @version:
 * @Author: fyh
 * @Date: 2024-4-03 14:38:03
 * @LastEditors: fyh
 * @LastEditTime: 2024-4-03 11:02:19
 */
#include "All_init.h"
#include "can.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "tim.h"
#include "CAN_receive.h"
#include "Video_downlink.h"

void All_init(void)
{
	remote_control_init();
	CAN1_FILTER_CONFIG(&hcan1);
	CAN2_FILTER_CONFIG(&hcan2);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	HAL_Delay(2000);
	bsp_can_init();
	dm_motor_init();
	//	HAL_Delay(1);
	//	write_motor_data(DM_yaw.id, 0, 1, 0, 0, 0);
	//	HAL_Delay(1);
	//	save_motor_data(DM_yaw.id, 0);
	dm_motor_enable(&hcan1, &DM_yaw);
	HAL_Delay(500); //! pitch轴使能延时，测试
	dm_motor_enable(&hcan1, &DM_pitch);
	//	HAL_Delay(1000);
	uart8_init(buff_1,buff_2,NEW_RC_RX_BUF_NUM);
}
