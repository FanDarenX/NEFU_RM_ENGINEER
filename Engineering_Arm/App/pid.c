/*
 * @Descripttion:
 * @version:
 * @Author: lxf、xqz
 * @Date: 2022-11-07 18:34:02
 * @LastEditors: xqz
 * @LastEditTime:
 */
#include "pid.h"
#include "math.h"

pid_struct_t moto_pid;

void limit(float *a, float max_value) // 限幅函数
{
  if (*a > max_value)
    *a = max_value;
  if (*a < -max_value)
    *a = -max_value;
}

/*
  没有按照标准的PID公式来写，因为考虑到响应问题。从简
*/
float pid_calculate(pid_typedef *pid, float get_speed, float set_speed) // PID计算->输出函数
{
  float index;
  uint8_t deadband = 5;
  pid->get[Now] = get_speed;             // 实际值
  pid->set[Now] = set_speed;             // 目标值
  pid->err[Now] = set_speed - get_speed; // 误差

  if (fabs(pid->err[Now]) > pid->Max_err && pid->Max_err != 0)
    return 0;

  if (fabs(pid->err[Now]) > deadband) // 死区判断
  {
    //			if(fabs(pid->err[Now])>0)
    //			{
    pid->pout = pid->kp * pid->err[Now]; // P
    //			}

    if (fabs(pid->err[Now]) > 1000) // 积分分离标准
    {
      index = 0;
    }
    else
    {
      index = 1;
    }
    pid->iout += (pid->ki * pid->err[Now] * index); // I
    limit(&(pid->iout), pid->I_limit);              // I限幅

    pid->dout = pid->kd * (pid->err[Now] - pid->err[Last]); // D
    pid->pid_out = pid->pout + pid->iout + pid->dout;       // PID输出

    limit(&(pid->pid_out), pid->Max_output); // 输出限幅
    pid->err[Last] = pid->err[Now];
    return pid->pid_out;
  }
  else
  {
    pid->err[Last] = pid->err[Now];
    return 0;
  }
}

void pid_param_init(pid_typedef *pid, float target, float I_limit, uint32_t Max_output, uint32_t Max_err, int16_t deadband, float pid_out,
                    float kp, float ki, float kd) // 目标值,积分限幅,最大输出,最大误差,死区,pid_out,p,i,d
{
  pid->target = target;
  pid->I_limit = I_limit;
  pid->Max_output = Max_output;
  pid->Max_err = Max_err;
  pid->deadband = deadband;
  pid->pid_out = pid_out;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

void change_pid_par(pid_typedef *set_pid_p, float N_kp, float N_ki, float N_kd) // 改变pid参数
{
  set_pid_p->kp = N_kp;
  set_pid_p->ki = N_ki;
  set_pid_p->kd = N_kd;
}

void pid_init(pid_typedef *pid) // 结构体初始化（将函数放入结构体）
{
  pid->f_param_init = pid_param_init;
  pid->f_calculate = pid_calculate;
  pid->f_calculate_position = pid_calculate;
}

// void Motor_limit(moto_measure *mea,pid_typedef *angle,pid_typedef *speed,int32_t set,int32_t angle_offset)
//{
//     angle->target = set - angle_offset;
//     angle->f_calculate_position(angle,mea->total_angle,angle->target);
//     speed->f_calculate(speed,mea->speed,angle->pid_out/5);
////    if(hcan->Instance == CAN1)
////    {
//////        SET_CAN1Ahead_MOTOR_CURRENT(&hcan1,lift_moto_pid[0].pid_out,lift_moto_pid[1].pid_out,
//////														Forward_moto_pid[0].pid_out,Forward_moto_pid[1].pid_out);
////
//////		SET_CAN1Back_MOTOR_CURRENT(&hcan1,slideway_moto_pid[0].pid_out,slideway_moto_pid[1].pid_out,0,0);
////    }else if(hcan->Instance == CAN2)
////	{
//////		SET_CAN2Ahead_MOTOR_CURRENT(&hcan2,Sucker_moto_pid[0].pid_out,Sucker_moto_pid[1].pid_out,Sucker_moto_pid[2].pid_out,Img_tx_moto_pid.pid_out);
//////		SET_CAN2Back_MOTOR_CURRENT(&hcan2,);

////	}

//}

void Motor_limit(CAN_HandleTypeDef *hcan, moto_measure *mea1, pid_typedef *angle1, pid_typedef *speed1, int32_t set1, int32_t angle_offset1)
{
  if (angle1->get[Now] - set1 > 0)
    angle1->target = set1;
  else
    angle1->target = set1 - 00;
  angle1->f_calculate_position(angle1, mea1->total_angle_true, angle1->target);
  speed1->f_calculate(speed1, mea1->speed * 6.28 / 8192, angle1->pid_out);
}

#define LimitMax(input, max) \
  {                          \
    if (input > max)         \
    {                        \
      input = max;           \
    }                        \
    else if (input < -max)   \
    {                        \
      input = -max;          \
    }                        \
  }

/**
 * @brief          pid struct data init
 * @param[out]     pid: PID struct data point
 * @param[in]      mode: PID_POSITION: normal pid
 *                 PID_DELTA: delta pid
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid max out
 * @param[in]      max_iout: pid max iout
 * @retval         none
 */
/**
 * @brief          pid struct data init
 * @param[out]     pid: PID结构数据指针
 * @param[in]      mode: PID_POSITION:普通PID
 *                 PID_DELTA: 差分PID
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid最大输出
 * @param[in]      max_iout: pid最大积分输出
 * @retval         none
 */
void PID_init(pid_type_def *pid, uint8_t mode, fp32 Kp, fp32 Ki, fp32 Kd, fp32 max_out, fp32 max_iout)
{
  if (pid == NULL)
  {
    return;
  }
  pid->mode = mode;
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->max_out = max_out;
  pid->max_iout = max_iout;
  pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
  pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
 * @brief          pid calculate
 * @param[out]     pid: PID struct data point
 * @param[in]      ref: feedback data
 * @param[in]      set: set point
 * @retval         pid out
 */
/**
 * @brief          pid计算
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @retval         pid输出
 */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
  if (pid == NULL)
  {
    return 0.0f;
  }

  pid->error[2] = pid->error[1];
  pid->error[1] = pid->error[0];
  pid->set = set;
  pid->fdb = ref;
  pid->error[0] = set - ref;
  if (pid->mode == PID_POSITION)
  {
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
  }
  else if (pid->mode == PID_DELTA)
  {
    pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
    pid->Iout = pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    pid->out += pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
  }
  return pid->out;
}

/**
 * @brief          pid out clear
 * @param[out]     pid: PID struct data point
 * @retval         none
 */
/**
 * @brief          pid 输出清除
 * @param[out]     pid: PID结构数据指针
 * @retval         none
 */
void PID_clear(pid_type_def *pid)
{
  if (pid == NULL)
  {
    return;
  }

  pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
  pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
  pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
  pid->fdb = pid->set = 0.0f;
}

/*新增PID*/
int LIMIT_MIN_MAX(int value, int min, int max)
{
  if (value < min)
  {
    return min;
  }
  else if (value > max)
  {
    return max;
  }
  else
  {
    return value;
  }
}

float pid_calc(pid_struct_t *pid, float ref, float fdb) // PID运算函数
{

  //	ac = LIMIT_MIN_MAX(ac,20,100);

  pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->ref - pid->fdb;

  pid->p_out = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);

  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}

void Pid_Init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max) // PID初始化函数
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->i_max = i_max;
  pid->out_max = out_max;
}

void PID_Init(pid_struct_t *speed_pid, pid_struct_t *angle_speed) // 角度环和速度环的PID初始化,只是初测出来的数据，具体还需要测试
{
  Pid_Init(speed_pid, 30, 0.15, 0, 30000, 30000); // P=30,I=0,D=0
  Pid_Init(angle_speed, 200, 1, 0, 0, 320);       // P=400,I=0,D=0
}
