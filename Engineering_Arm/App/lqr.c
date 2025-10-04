#include "lqr.h"
// 海哥的lqr，该
void abs_limit(float *num, fp32 Limit)
{
	if (*num > Limit)
	{
		*num = Limit;
	}
	else if (*num < -Limit)
	{
		*num = -Limit;
	}
}

int16_t gimbal_LQR_calc(fp32 K1, fp32 K2, fp32 K0, fp32 angle_get, fp32 speed_get, fp32 angle_set, fp32 speed_set)
{
	fp32 accel = 0.0f;
	int16_t current = 0;
	static fp32 angle_int = 0;
	fp32 K0_angle = 0;

	angle_int += angle_get - angle_set;
	abs_limit(&angle_int, 5.0f);
	K0_angle = angle_int * K0;
	abs_limit(&K0_angle, 5000.0f);

	accel = (K1 * (angle_set - angle_get) - K2 * speed_get - K0_angle); // 通过角度和速度求出理论要求输入的角加速度
	abs_limit(&accel, 25000.0f);
	current = (int16_t)accel; // 理论上说这里不需要加权
	return current;
}
