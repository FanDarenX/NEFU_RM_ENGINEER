#include "user_lib.h"
#include "stdio.h"
#include "gimbal_task.h"

#define _PI_2  1.57079632679
#define _PI    3.14159265359
#define _3PI_2 4.71238898038
#define _2PI   6.28318530718

#define _round(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x) - 0.5))
//快速三角函数
const int sine_array[200] = {
    0, 79, 158, 237, 316, 395, 473, 552, 631, 710,
    789, 867, 946, 1024, 1103, 1181, 1260, 1338, 1416, 1494,
    1572, 1650, 1728, 1806, 1883, 1961, 2038, 2115, 2192, 2269,
    2346, 2423, 2499, 2575, 2652, 2728, 2804, 2879, 2955, 3030,
    3105, 3180, 3255, 3329, 3404, 3478, 3552, 3625, 3699, 3772,
    3845, 3918, 3990, 4063, 4135, 4206, 4278, 4349, 4420, 4491,
    4561, 4631, 4701, 4770, 4840, 4909, 4977, 5046, 5113, 5181,
    5249, 5316, 5382, 5449, 5515, 5580, 5646, 5711, 5775, 5839,
    5903, 5967, 6030, 6093, 6155, 6217, 6279, 6340, 6401, 6461,
    6521, 6581, 6640, 6699, 6758, 6815, 6873, 6930, 6987, 7043,
    7099, 7154, 7209, 7264, 7318, 7371, 7424, 7477, 7529, 7581,
    7632, 7683, 7733, 7783, 7832, 7881, 7930, 7977, 8025, 8072,
    8118, 8164, 8209, 8254, 8298, 8342, 8385, 8428, 8470, 8512,
    8553, 8594, 8634, 8673, 8712, 8751, 8789, 8826, 8863, 8899,
    8935, 8970, 9005, 9039, 9072, 9105, 9138, 9169, 9201, 9231,
    9261, 9291, 9320, 9348, 9376, 9403, 9429, 9455, 9481, 9506,
    9530, 9554, 9577, 9599, 9621, 9642, 9663, 9683, 9702, 9721,
    9739, 9757, 9774, 9790, 9806, 9821, 9836, 9850, 9863, 9876,
    9888, 9899, 9910, 9920, 9930, 9939, 9947, 9955, 9962, 9969,
    9975, 9980, 9985, 9989, 9992, 9995, 9997, 9999, 10000, 10000
};
/**
  * @brief          斜波函数初始化
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      最大值
  * @param[in]      最小值
  * @retval         返回空
  */
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      输入值
  * @param[in]      滤波参数
  * @retval         返回空
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}
/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
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
void first_order_filter_speed_ramp_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
	input=Speed_ramp(input,chassis_control.chassis_balance.foot_speed);
    first_order_filter_type->input = input;
    first_order_filter_type->out =first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//绝对限制
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

//判断符号位
fp32 sign(fp32 value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

//浮点死区
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//限幅函数
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//循环限幅函数
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//弧度格式化为-PI~PI

//角度格式化为-180~180
fp32 theta_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}

uint8_t GlobalDebugMode = 7;

//快速开方
float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

//快速求平方根倒数
/*
float invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f375a86- (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}*/
//float invSqrt(float x)
//{
//float xhalf = 0.5 * x;
//int i = *(int*)&x; // get bits for floating value
//i = 0x5f3759df - (i >> 1); // gives initial guess
//x = *(float*)&i; // convert bits back to float
//x = x * (1.5 - xhalf * x * x); // Newton step
//return x;
//}
float user_sin ( float a ) {
    if ( a < _PI_2 ) {
        return 0.0001 * sine_array[_round ( 126.6873f * a )];
    } else if ( a < _PI ) {
        return 0.0001 * sine_array[398 - _round ( 126.6873f * a )];
    } else if ( a < _3PI_2 ) {
        return -0.0001f * sine_array[-398 + _round ( 126.6873f * a )];
    } else {
        return -0.0001f * sine_array[796 - _round ( 126.6873f * a )];
    }
}

float user_cos ( float a ) {
    float a_sin = a + _PI_2;
    a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
    return user_sin ( a_sin );
}
float user_tan(float a)
{
	float a_tan = user_sin(a)/user_cos(a);
	return a_tan;
}
//int main () {
//    float test = 0.5 * 3.14;
//    printf ( "_sin is %f\n", _sin ( test ) );
//    printf ( "_cos is %f\n", _cos ( test ) );
//}

//浮点死区
float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}


//限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

int float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

/**
  * @brief          最小二乘法初始化
  * @param[in]      最小二乘法结构体
  * @param[in]      样本数
  * @retval         返回空
  */
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order)
{
    OLS->Order = order;
    OLS->Count = 0;
    OLS->x = (float *)user_malloc(sizeof(float) * order);
    OLS->y = (float *)user_malloc(sizeof(float) * order);
    OLS->k = 0;
    OLS->b = 0;
    memset((void *)OLS->x, 0, sizeof(float) * order);
    memset((void *)OLS->y, 0, sizeof(float) * order);
    memset((void *)OLS->t, 0, sizeof(float) * 4);
}

/**
  * @brief          最小二乘法拟合
  * @param[in]      最小二乘法结构体
  * @param[in]      信号新样本距上一个样本时间间隔
  * @param[in]      信号值
  */
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }
    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;
}

/**
  * @brief          最小二乘法提取信号微分
  * @param[in]      最小二乘法结构体
  * @param[in]      信号新样本距上一个样本时间间隔
  * @param[in]      信号值
  * @retval         返回斜率k
  */
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k;
}

/**
  * @brief          获取最小二乘法提取信号微分
  * @param[in]      最小二乘法结构体
  * @retval         返回斜率k
  */
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k;
}

/**
  * @brief          最小二乘法平滑信号
  * @param[in]      最小二乘法结构体
  * @param[in]      信号新样本距上一个样本时间间隔
  * @param[in]      信号值
  * @retval         返回平滑输出
  */
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

/**
  * @brief          获取最小二乘法平滑信号
  * @param[in]      最小二乘法结构体
  * @retval         返回平滑输出
  */
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

//快速浮点型幂
fp32 fast_pow(double x, int n)  
{  
    if(n==0)  
        return 1.0;  
    if(n<0)  
        return 1.0/fast_pow(x,-n);  
    float half = fast_pow(x,n>>1);  
    if(n%2==0)  
        return half*half;  
    else  
        return half*half*x;  
}  
//快速tan
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define ABS(x) (((x) > 0) ? (x) : (-(x)))
fp32 atan_tl(fp32 dy)
{
	fp32 dx=1,ax,ay,a,r,s;
	ax = ABS(dx);
	ay = ABS(dy);
	a = MIN(ax, ay)/MAX(ax, ay);
	s = a*a;
	r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
	if(ay > ax) r = PI_2 - r;
	if(dx < 0) r = PI - r;
	if(dy<0) r = -r;
	return r;
}

