/*
 * @Descripttion:
 * @version:
 * @Author: lxf
 * @Date: 2022-11-07 18:34:14
 * @LastEditors: lxf
 * @LastEditTime: 2023-03-28 04:03:18
 */
#ifndef __PID_H_
#define __PID_H_

#include "main.h"
#include "CAN_receive.h"
#include "struct_typedef.h"

/*新增PID*/
typedef struct _pid_struct_t
{
    float kp;      // 比例
    float ki;      // 积分
    float kd;      // 微分
    float i_max;   // 积分限幅
    float out_max; // 输出限幅

    float ref;    // target value目标角度
    float fdb;    // feedback value设定角度
    float err[2]; // error and last error差值

    float p_out;  // 比例输出
    float i_out;  // 积分输出
    float d_out;  // 微分输出
    float output; // pid总输出
    //	float aaa;
} pid_struct_t;

void Pid_Init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);

void PID_Init(pid_struct_t *speed_pid, pid_struct_t *angle_speed);
float pid_calc(pid_struct_t *pid, float ref, float fdb);
int LIMIT_MIN_MAX(int value, int min, int max);
/*新增PID*/

typedef enum time
{
    Last,
    Now
} time;

typedef struct _pid_typedef
{
    float kp;
    float ki;
    float kd;
    float target;

    float set[3]; // 设置的值
    float get[3]; // 得到的值
    float err[3]; // 得到的误差

    float pout;    // 比例输出
    float iout;    // 积分输出
    float dout;    // 微分输出
    float pid_out; // pid总输出

    float I_limit;       // 积分限幅
    uint32_t Max_output; // 最大输出限制
    uint32_t Max_err;    // 最大误差
    int16_t deadband;

    void (*f_param_init)(struct _pid_typedef *pid, float target, float I_limit, uint32_t Max_output, uint32_t Max_err, int16_t deadband, float pid_out,
                         float kp, float ki, float kd);                               // 目标值,积分限幅,最大输出,最大误差,死区,pid_out,p,i,d
    float (*f_calculate)(struct _pid_typedef *pid, float get_speed, float set_speed); // 指针函数，初始时= pid
    float (*f_calculate_position)(struct _pid_typedef *pid, float get_speed, float set_speed);

    /*新增PID*/
    float (*f_cal)(pid_struct_t *pid, float ref, float fdb);
    void (*f_init)(pid_struct_t *pid,
                   float kp,
                   float ki,
                   float kd,
                   float i_max,
                   float out_max);

    struct motor // 反馈值
    {
        uint16_t angle;       // 转子角度
        int16_t speed;        // 速度
        int16_t real_current; // 实际电流
        int8_t temperature;   // 温度
    } motor;

} pid_typedef;

void change_pid_par(pid_typedef *set_pid_p, float N_kp, float N_ki, float N_kd);
void pid_init(pid_typedef *pid);
void pid_param_init(pid_typedef *pid, float target, float I_limit, uint32_t Max_output, uint32_t Max_err, int16_t deadband, float pid_out,
                    float kp, float ki, float kd);
float pid_calculate(pid_typedef *pid, float get_speed, float set_speed);
void limit(float *a, float max_value);
void Motor_limit(CAN_HandleTypeDef *hcan, moto_measure *mea1, pid_typedef *angle1, pid_typedef *speed1, int32_t set1, int32_t angle_offset1);

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    // PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  // 最大输出
    fp32 max_iout; // 最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  // 微分项 0最新 1上一次 2上上次
    fp32 error[3]; // 误差项 0最新 1上一次 2上上次

} pid_type_def;

extern void PID_init(pid_type_def *pid, uint8_t mode, fp32 Kp, fp32 Ki, fp32 Kd, fp32 max_out, fp32 max_iout);
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);
extern void PID_clear(pid_type_def *pid);

#endif
