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

/*����PID*/
typedef struct _pid_struct_t
{
    float kp;      // ����
    float ki;      // ����
    float kd;      // ΢��
    float i_max;   // �����޷�
    float out_max; // ����޷�

    float ref;    // target valueĿ��Ƕ�
    float fdb;    // feedback value�趨�Ƕ�
    float err[2]; // error and last error��ֵ

    float p_out;  // �������
    float i_out;  // �������
    float d_out;  // ΢�����
    float output; // pid�����
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
/*����PID*/

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

    float set[3]; // ���õ�ֵ
    float get[3]; // �õ���ֵ
    float err[3]; // �õ������

    float pout;    // �������
    float iout;    // �������
    float dout;    // ΢�����
    float pid_out; // pid�����

    float I_limit;       // �����޷�
    uint32_t Max_output; // ����������
    uint32_t Max_err;    // ������
    int16_t deadband;

    void (*f_param_init)(struct _pid_typedef *pid, float target, float I_limit, uint32_t Max_output, uint32_t Max_err, int16_t deadband, float pid_out,
                         float kp, float ki, float kd);                               // Ŀ��ֵ,�����޷�,������,������,����,pid_out,p,i,d
    float (*f_calculate)(struct _pid_typedef *pid, float get_speed, float set_speed); // ָ�뺯������ʼʱ= pid
    float (*f_calculate_position)(struct _pid_typedef *pid, float get_speed, float set_speed);

    /*����PID*/
    float (*f_cal)(pid_struct_t *pid, float ref, float fdb);
    void (*f_init)(pid_struct_t *pid,
                   float kp,
                   float ki,
                   float kd,
                   float i_max,
                   float out_max);

    struct motor // ����ֵ
    {
        uint16_t angle;       // ת�ӽǶ�
        int16_t speed;        // �ٶ�
        int16_t real_current; // ʵ�ʵ���
        int8_t temperature;   // �¶�
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
    // PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  // ������
    fp32 max_iout; // ���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  // ΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; // ����� 0���� 1��һ�� 2���ϴ�

} pid_type_def;

extern void PID_init(pid_type_def *pid, uint8_t mode, fp32 Kp, fp32 Ki, fp32 Kd, fp32 max_out, fp32 max_iout);
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);
extern void PID_clear(pid_type_def *pid);

#endif
