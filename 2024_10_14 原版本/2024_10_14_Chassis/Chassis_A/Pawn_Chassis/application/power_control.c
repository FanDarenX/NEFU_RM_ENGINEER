/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             只控制80w功率，主要通过控制电机电流设定值,如果限制功率是40w，减少
  *             JUDGE_TOTAL_CURRENT_LIMIT和POWER_CURRENT_LIMIT的值，还有底盘最大速度
  *             (包括max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "power_control.h"
#include "arm_math.h"
//#define POWER_LIMIT         60.0f
//#define WARNING_POWER       45.0f
//#define WARNING_POWER_BUFF  40.0f

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
//#define BUFFER_TOTAL_CURRENT_LIMIT      5000.0f
//#define POWER_TOTAL_CURRENT_LIMIT       10000.0f

fp32   POWER_LIMIT =          59.0f;
fp32   WARNING_POWER  =       60.0f;
fp32 BUFFER_TOTAL_CURRENT_LIMIT = 10000.0f;
fp32 POWER_TOTAL_CURRENT_LIMIT = 12000.0f;
fp32 WARNING_POWER_BUFF = 40.0f;
fp32 total_current ;
fp32 current_scale ;
fp32 chassis_power ;
fp32 chassis_power_buffer ;
fp32 limit;
float total_current_limit;
/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data
  * @retval         none
  */
/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */
void chassis_power_control(chassis_move_t *chassis_power_control)
{
    get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer,&limit);
    total_current_limit =64000;
    if(chassis_power_buffer<WARNING_POWER)
    {
        current_scale =((chassis_power_buffer)/(WARNING_POWER))*((chassis_power_buffer)/(WARNING_POWER));
        total_current_limit =  current_scale*total_current_limit;
    }
    else
    {
        total_current_limit = total_current_limit;
    }

    total_current = 0.0f;
    //calculate the original motor current set
    //计算原本电机电流设定
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_power_control->motor_speed_pid[i].out);

    }


    if(total_current > total_current_limit)
    {
        chassis_power_control->motor_speed_pid[0].out*=current_scale;
        chassis_power_control->motor_speed_pid[1].out*=current_scale;
        chassis_power_control->motor_speed_pid[2].out*=current_scale;
        chassis_power_control->motor_speed_pid[3].out*=current_scale;
    }
    else
    {
        chassis_power_control->motor_speed_pid[0].out=chassis_power_control->motor_speed_pid[0].out;
        chassis_power_control->motor_speed_pid[0].out=chassis_power_control->motor_speed_pid[0].out;
        chassis_power_control->motor_speed_pid[2].out= chassis_power_control->motor_speed_pid[2].out;
        chassis_power_control->motor_speed_pid[3].out= chassis_power_control->motor_speed_pid[3].out;
    }
}
