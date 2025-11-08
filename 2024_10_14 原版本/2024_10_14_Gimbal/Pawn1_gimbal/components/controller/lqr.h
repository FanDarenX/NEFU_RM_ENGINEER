#ifndef LQR_H
#define LQR_H

#include "struct_typedef.h"

typedef struct
{
  // angle_Loop
  fp32 kp_angle;
  fp32 ki_angle;

	fp32 angle_get;
	fp32 angle_set;
	fp32 angle_err;
	fp32 angle_err_last;
	fp32 angle_err_cumu;

	// speed_Loop
	fp32 kp_speed;
	fp32 speed_get;
	fp32 speed_set;				// 速度期望值，应赋0


	// current_FeedForward
	fp32 current_K[4];			//系统辨识得到电流前馈三次多项式系数
	fp32 kp_current;
	fp32 disturbance;			//干扰量，p轴是角度，y轴是底盘角速度

//	fp32 current_get;
//	fp32 current_set;
//	fp32 current_err;

	fp32 angle_out;				// 角度环输出
	fp32 speed_out;				// 速度环输出
	fp32 current_out;			// 电流前馈
	fp32 out;							// 总输出

	fp32 max_angle_err;   // 最大角度累积误差
	fp32 max_out;					// 输出限幅

} lqr_type_def;

void LQR_init(lqr_type_def *lqr, const fp32 LQR_Ak[3], const fp32 LQR_Ik[4], fp32 max_angle_err, fp32 max_out);
extern fp32 LQR_calc(lqr_type_def *lqr, fp32 angle_get, fp32 angle_set, fp32 speed_get, fp32 current_get);
extern void LQR_clear(lqr_type_def *lqr);

#endif
