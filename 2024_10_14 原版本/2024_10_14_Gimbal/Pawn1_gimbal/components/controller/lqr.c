#include "lqr.h"
#include "stddef.h"
#include "user_lib.h"
#include "arm_math.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
		
void LQR_init(lqr_type_def *lqr, const fp32 LQR_Ak[3], const fp32 LQR_Ik[4], fp32 max_angle_err, fp32 max_out)
{
    if (lqr == NULL || LQR_Ak == NULL || LQR_Ik == NULL)
    {
        return;
    }
    lqr->kp_angle = 	LQR_Ak[0];
    lqr->ki_angle = 	LQR_Ak[1];
    lqr->kp_speed = 	LQR_Ak[2];
	lqr->kp_current = 	LQR_Ak[3];
	for(uint8_t i = 0; i < 4; i++)
	{
		lqr->current_K[i] = LQR_Ik[i];
	}
    lqr->max_angle_err = max_angle_err;
	lqr->angle_err_last = 0;
    lqr->max_out = max_out;
}

fp32 LQR_calc(lqr_type_def *lqr, fp32 angle_get, fp32 angle_set, fp32 speed_get, fp32 disturbance)
{
	fp32 err;
	if(lqr ==NULL)
	{
		return 0.0f;
	}
	// 反馈值，期望值获取
	lqr->angle_get = angle_get;
	lqr->angle_set = angle_set;
	lqr->speed_get = speed_get;
	lqr->speed_set = 0.0f;               	// 速度期望值应赋0
//	lqr->current_get = current_get;
//	lqr->current_set = 0.0f;							// 电流期望值应赋0
//	lqr->current_err = lqr->current_set - lqr->current_get;
	// 计算单个环
	err = (lqr->angle_set - lqr->angle_get);
	lqr->angle_err = rad_format(err);
	lqr->angle_err_cumu += (lqr->angle_err + lqr->angle_err_last)/2.0f;//梯形积分
	LimitMax(lqr->angle_err_cumu, lqr->max_angle_err);              // 角度误差限幅
	
	lqr->angle_out = lqr->kp_angle * lqr->angle_err + lqr->ki_angle * lqr->angle_err_cumu;
	
	lqr->speed_out = lqr->kp_speed * (lqr->speed_set - lqr->speed_get);
	
	lqr->current_out = lqr->current_K[3] * lqr->disturbance*lqr->disturbance*lqr->disturbance + \
						lqr->current_K[2] * lqr->disturbance*lqr->disturbance + \
						lqr->current_K[1] * lqr->disturbance + \
						lqr->current_K[0]; //系统辨识得到电流前馈三次多项式系数
	lqr->current_out = lqr->kp_current * lqr->current_out;
	
	// 计算总的输出
	lqr->out = lqr->angle_out + lqr->speed_out + lqr->current_out;
	LimitMax(lqr->out, lqr->max_out);               // 输出限幅
	
	lqr->angle_err_last = lqr->angle_err;
	
	return lqr->out;
}

void LQR_clear(lqr_type_def *lqr)
{
    if (lqr == NULL)
    {
        return;
    }

    lqr->out = lqr->angle_out = lqr->speed_out = lqr->current_out = 0.0f;
    lqr->angle_get = lqr->angle_set = lqr->angle_err = lqr->angle_err_last = lqr->speed_get = lqr->speed_set =0.0f;// lqr->current_get = lqr->current_set = 0.0f;
}
