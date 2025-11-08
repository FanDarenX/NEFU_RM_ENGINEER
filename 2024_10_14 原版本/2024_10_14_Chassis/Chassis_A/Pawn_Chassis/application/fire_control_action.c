#include "fire_control_action.h"
#include "gimbal_task.h"
#include "math.h"
#include "shoot.h"
#include "bsp_filter.h"
#include "bsp_usart.h"
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

PC_get_data_t PC_get_data,PC_get_data_analyz;
extKalman_t K_Yaw,K_Pitch;
/**************************PC数据读取+火控角度解算***********************************/
//名称：步兵火控
//日期：2021年10月27日
//版本：V1.0
//目标：静止瞄准静止目标,动态死区运算
/***********************************************************************************/
void PC_filter_Init()
{   //                      Q      R     Q/R越大，越倾向于测量；越小越倾向于预测
    KalmanCreate(&K_Yaw,  0.2f, 1.0f);
    KalmanCreate(&K_Pitch,0.2f, 1.0f);
}
void PC_data_analyz(PC_get_data_t *PC_analyz,PC_get_data_t *PC)
{
    PC->mode            =   PC_temp[1];
    PC->pitch           =   (int16_t)(PC_temp[5] << 8 | PC_temp[4]);
    PC->yaw             =   (int16_t)(PC_temp[7] << 8 | PC_temp[6]);
    PC->distance        =   (int16_t)(PC_temp[9] << 8 | PC_temp[8]);
//储存上一次的数据
    PC_analyz->pitch_last     = 	PC_analyz->pitch;
    PC_analyz->yaw_last       = 	PC_analyz->yaw;
    PC_analyz->distance_last  = 	PC_analyz->distance;
//更新数据
    PC_analyz->mode     =    PC->mode;
    if(PC_analyz->mode == 0x31)
    {   //若视野中有目标，则处理数据
        PC_analyz->pitch    =    (fp32)PC->pitch    *0.0001f;
        PC_analyz->yaw      =   -(fp32)PC->yaw      *0.0001f;
        PC_analyz->distance =    (fp32)PC->distance *0.01f;
    }
    else if(PC_analyz->mode == 0x30)
    {   //若视野中无目标，则不处理数据并沿用上一次数据，同时稳住云台
        PC_analyz->pitch       =  PC_analyz->pitch_last;
        PC_analyz->yaw         =  PC_analyz->yaw_last;
        PC_analyz->distance    =  PC_analyz->distance_last;
    }
}
/*******************************解算后的角度传输给云台*************************************************/


uint8_t trajectory_analyz(fp32 *add_yaw, fp32 *add_pitch, PC_get_data_t *PC_analyz)
{
    static fp32 yaw_channel = 0, pitch_channel = 0;
    static fp32 K_yaw_channel = 0, K_pitch_channel = 0;
    uint8_t out=0;
    if (gimbal_behaviour == GIMBAL_ABSOLUTE_AUTOMA_ANGLE )//自瞄不用遥控器，使用PC传入数据
    {
        pitch_channel = PC_analyz->pitch;
        yaw_channel   = PC_analyz->yaw;

        K_yaw_channel   = yaw_channel;//KalmanFilter(&K_Yaw,yaw_channel);
        K_pitch_channel = pitch_channel;//KalmanFilter(&K_Pitch,pitch_channel);

        *add_yaw           = K_yaw_channel;
        *add_pitch         = K_pitch_channel;
    }
    else
    {
        *add_yaw           = 0;
        *add_pitch         = 0;
        PC_analyz->pitch   =0;
        PC_analyz->yaw     =0;
        PC_analyz->pitch_last   =0;
        PC_analyz->yaw_last     =0;
        PC_analyz->distance_last=0;
    }

    if(PC_analyz->mode == 0x31)out=0x31;
    else if(PC_analyz->mode == 0x30)out=0x30;
    return out;
}
