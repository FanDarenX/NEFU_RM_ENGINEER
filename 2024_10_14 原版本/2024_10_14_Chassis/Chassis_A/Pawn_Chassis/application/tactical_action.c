#include "tactical_action.h"
#include "gimbal_task.h"
#include "classis_task.h"
#include "math.h"

#define rc_deadband_limit(input, output, dealine)        \
    { if ((input) > (dealine) || (input) < -(dealine)) \
        {  (output)= (input); }                                                \
        else                                             \
        {  (output)= 0; }}
/****************小陀螺***********************/
//速度分解：使车以云台当前方向为前进方向运行
//功率调配：前进速度慢时自旋速度快
/********************************************/
void chassis_rc_to_control_vector_rotate(fp32 *vx_set, fp32 *vy_set,fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    int16_t vx_channel, vy_channel,wz_channel;
    fp32 vx_set_channel, vy_set_channel,wz_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL],  vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL],  vy_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
        vx_channel=chassis_move_rc_to_vector->vx_max_speed;
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
        vx_channel=chassis_move_rc_to_vector->vx_min_speed;

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
        vy_channel=chassis_move_rc_to_vector->vx_max_speed;
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
        vy_channel=chassis_move_rc_to_vector->vx_min_speed;

    vx_set_channel = cos(gimbal_control.gimbal_yaw_motor.relative_angle) *vx_channel   +   sin(gimbal_control.gimbal_yaw_motor.relative_angle)*vy_channel;// * CHASSIS_VX_RC_SEN;
    vy_set_channel = sin(gimbal_control.gimbal_yaw_motor.relative_angle) *vx_channel  -   cos(gimbal_control.gimbal_yaw_motor.relative_angle)*vy_channel;// * -CHASSIS_VY_RC_SEN;
    wz_set_channel = ROTATE_SPEED;
    //阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_wz, wz_set_channel);
    //stop command, need not slow change, set zero derectly
    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }
    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }
    if (wz_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_WZ_RC_SEN && wz_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_WZ_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
    *wz_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out;
}
/******************飞坡加速*********************/
//速度分解：使车以云台当前方向为前进方向运行
//功率调配：前进速度慢时自旋速度快
/********************************************/
void action_fly_slope()
{   //战术动作：飞坡！




}


void action_rushbaron()
{   //战术动作：打大幅！




}


