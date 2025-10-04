#include "TD.h"
#include "math.h"
#include "main.h"
#include "Can_receive.h"

TD Y_td;
TD X_td;


void TD_init(TD *TT, float r, float h1, float h2)
{
	TT->xx1 = 0; // 初始化 xx1
	TT->xx2 = 0; // 初始化 xx2
	TT->fh1 = 0; // 初始化 fh1
	TT->r = r;
	TT->h1 = h1;
	TT->h2 = h2;
}


/*信号函数Sign*/
int16_t Sign(float x)
{
	if (x > 1E-6f) // 1*10的-6次
		return 1;
	else if (x < 1E-6f)
		return -1;
	else
		return 0;
}

/*非线性函数：区间函数Fsg*/
int16_t Fsg(float x, float d)
{
	float output = 0;
	output = (Sign(x + d) - Sign(x - d)) / 2;
	return output;
}

/*快速控制综合最优函数*/
float Fhan(float x1, float x2, float r, float h)
{
	float d, a, a0, a1, a2, y;

	d = r * h * h;
	if (fabs(d) < 1E-6f)
	{
		d = 1E-6f; // 避免除以零
	}

	a0 = h * x2;

	y = x1 + a0;

	if (fabs(y) < 1E-6f)
	{
		y = 1E-6f; // 避免 y 接近零时产生不稳定性
	}

	a1 = sqrt(d * (d + 8.0f * fabs(y))); // 使用 fabs 确保非负

	a2 = a0 + Sign(y) * (a1 - d) * 0.5f;

	a = (a0 + y) * Fsg(y, d) + a2 * (1 - Fsg(y, d));

	if (fabs(a) < 1E-6f)
	{
		a = 1E-6f; // 避免 a 接近零时产生不稳定性
	}

	return (-r) * (a / d) * Fsg(a, d) - r * Sign(a) * (1 - Fsg(a, d));
}

/*
参数 r：该参数决定了跟踪微分器的增益。
较大的 r 值可以使系统更快地响应变化，但过大的值可能导致系统不稳定或产生振荡。
通常情况下，可以选择一个相对较小的值开始测试，例如 r = 5 或者 r = 10，然后根据实际效果调整。

参数 h1：这个参数与内部计算中的非线性项有关，影响了系统的动态特性。
一般而言，h1 应该小于等于 1，并且可以根据具体应用场景进行调整。
对于角度范围在 0 到 180 的情况，可以尝试从 h1 = 0.5 开始，观察其表现并适当调整。

参数 h2：这是时间步长或者说是采样周期的比例因子，它直接影响到状态更新的速度。
如果您的系统是以较高的频率运行（如 1kHz），那么可以选择较小的时间步长比例因子；
反之，则选择较大的值。考虑到角度的变化率不会特别高，可以从 h2 = 0.01 或者 h2 = 0.005 开始尝试。
*/
void Tracking_Differentiator(TD *TT, float target, float r, float h1, float h2)
{
	if (isnan(target))
	{
		// 处理 target 为 NaN 的情况
		target = 0; // 或者设置一个合理的默认值
	}
	// 计算 Fhan 函数时确保 d 不为零
	float d = r * h1 * h1;
	if (fabs(d) < 1E-6f)
	{
		d = 1E-6f;
	}

	TT->fh1 = Fhan(TT->xx1 - target, TT->xx2, r, h1);

	// 更新状态变量
	TT->xx1 = TT->xx1 + h2 * TT->xx2;
	TT->xx2 = TT->xx2 + h2 * TT->fh1;
}

void TD_INIT(void)
{
	/*遥控器*/ // 参数 r 大小和响应速度正相关
	TD_init(&Y_td, 12000.0f, 0.5f, 0.005f);//36000.0f
	TD_init(&X_td, 25000.0f, 0.5f, 0.005f); // 50000对应中间抬最高极限蜷缩
}


void td_task(void)
{
	Tracking_Differentiator(&Y_td, Y_target, Y_td.r, Y_td.h1, Y_td.h2);
    Tracking_Differentiator(&X_td, X_target, X_td.r, X_td.h1, X_td.h2);
}

