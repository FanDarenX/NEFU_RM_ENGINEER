/*
一些滤波的函数库
*/

#include "user_lib.h"
#include "usart.h"
#include "pid.h"
#include "bsp_can.h"

/**
 * @brief          一阶低通滤波初始化
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      滤波参数
 * @retval         返回空
 */
// 如果你的应用场景需要较强的滤波效果（即减少高频噪声）并且可以接受较慢的响应速度，那么这个配置是合理的。
// 如果你需要更快的响应速度，可以考虑减小 num 的值或增大 frame_period。
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num)
{
  first_order_filter_type->frame_period = frame_period;
  first_order_filter_type->num = num;
  first_order_filter_type->input = 0.0f;
  first_order_filter_type->out = 0.0f;
}

/**
 * @brief          一阶低通滤波计算
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @retval         返回空
 */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
  first_order_filter_type->last_out = first_order_filter_type->out;
  first_order_filter_type->input = input;
  first_order_filter_type->out =
      first_order_filter_type->num / (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->out +
      first_order_filter_type->frame_period / (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->input;
  //					first_order_filter_type->input * first_order_filter_type->num[0]+(1-first_order_filter_type->num[0])*first_order_filter_type->last_out;
}

/**
 * @brief          按键重复调用
 * @param          键盘按键
 * @param          客户端按键反馈数据
 */
static int32_t cnt = 0;
static int16_t i = 0;
int key_count(uint16_t key, uint16_t key_target)
{
  if (key == 0)
  {
    i = 1;
  }
  if (key == key_target && i == 1)
  {
    cnt++;
    i = 0;
  }

  if (cnt % 2 == 1)
    return 1;
  else
    return 0;
}

////==============电机限位函数==============//

// void limit_moto(CAN_HandleTypeDef *hcan ,moto_measure *mea ,pid_typedef *angle, pid_typedef *speed, int32_t set, int32_t angle_offset)
//{
//		angle->target = set - angle_offset;
//		angle->f_calculate_position(angle, mea->total_angle, angle->target);
//		speed->f_calculate(speed, mea->speed, (angle->pid_out)/5);
//
//		if(hcan->Instance == CAN1)              //判断数据传输入口
//		{
//			SET_CAN1Ahead_MOTOR_CURRENT(&hcan1, lift_moto_pid[0].pid_out, lift_moto_pid[1].pid_out, chassis_moto_pid[0].pid_out, chassis_moto_pid[1].pid_out);
//			SET_CAN1Back_MOTOR_CURRENT(&hcan1, chassis_moto_pid[2].pid_out, chassis_moto_pid[3].pid_out , 0, 0);
//		}
//		else if(hcan->Instance == CAN2)
//		{
//			SET_CAN2Ahead_MOTOR_CURRENT(&hcan2, tp_moto_pid[0].pid_out, tp_moto_pid[1].pid_out, spin_moto_pid[0].pid_out, spin_moto_pid[1].pid_out);
//			SET_CAN2Back_MOTOR_CURRENT(&hcan2, help_moto_pid.pid_out, block_moto_pid.pid_out, 0, 0);
//		}
// }
/*******************************************************************************************
 * @Brief  			延时函数
 * @Param				延时时间
 * @Retval
 * @Date
 *******************************************************************************************/

/**
 * @brief    			斜坡函数初始化
 * @param    			斜坡函数结构体
 * @param    			间隔时间
 * @param    			斜坡目标值
 * @param    			斜坡源
 */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
  ramp_source_type->frame_period = frame_period;
  ramp_source_type->max_value = max;
  ramp_source_type->min_value = min;
  ramp_source_type->input = 0.0f;
  ramp_source_type->out = 0.0f;
}

void ramp_calc(ramp_function_source_t *chassis_ramp, float input)
{
  chassis_ramp->input = input;
  if (input != 0)
  {
    chassis_ramp->out += chassis_ramp->input * chassis_ramp->frame_period;
    if (chassis_ramp->out > chassis_ramp->max_value)
      chassis_ramp->out = chassis_ramp->max_value;
    else if (chassis_ramp->out < chassis_ramp->min_value)
      chassis_ramp->out = chassis_ramp->min_value;
  }
  else
  {
    if (chassis_ramp->out > 5)
      chassis_ramp->out -= 10.0f;
    else if (chassis_ramp->out < -5)
      chassis_ramp->out += 10.0f;
    else
      chassis_ramp->out = 0;
  }
}
