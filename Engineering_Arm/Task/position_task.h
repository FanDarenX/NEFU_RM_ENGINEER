#ifndef __POSITION_TASK_H
#define __POSITION_TASK_H

#include "pid.h"
#include "remote_control.h"
#include "yaw_task.h"
#include "main.h"

/*遥控器读值换名*/
/*倍数*/ /*之后测试需要找到合适的倍数*/ // TODO
extern float Multiple_Cup_roll;
extern float Multiple_Cup_pitch;
extern float Multiple_X;
extern float Multiple_Y;
extern float Multiple_Z;
/*拨杆*/
// s左1右0
#define Lift_Up rc_ctrl.rc.s[1] == 1
#define Lift_Down rc_ctrl.rc.s[1] == 2
#define Lift_Middle rc_ctrl.rc.s[1] == 3
#define Right_Up rc_ctrl.rc.s[0] == 1
#define Right_Down rc_ctrl.rc.s[0] == 2
#define Right_Middle rc_ctrl.rc.s[0] == 3
/*摇杆*/
/*机械臂*/
#define Cup_roll rc_ctrl.rc.ch[4] / 660.0f * 8192 * Multiple_Cup_roll   // 拨轮
#define Cup_pitch rc_ctrl.rc.ch[3] / 660.0f * 8192 * Multiple_Cup_pitch // 左y纵
#define Arm_yaw rc_ctrl.rc.ch[0] / 660.0f * PI;                         // 右x横
#define Arm_pitch rc_ctrl.rc.ch[1] / 660.0f * PI;                       // 右y纵
/*龙门架*/
#define X_traverse rc_ctrl.rc.ch[2] / 660.0f * 8192 * Multiple_X // 左x横
#define Y_traverse rc_ctrl.rc.ch[3] / 660.0f * 8192 * Multiple_Y // 左y纵
#define Z_traverse rc_ctrl.rc.ch[4] / 660.0f * 8192 * Multiple_Z // 拨轮
/*MAX*/
#define X_MAX 8192 * Multiple_X
#define Y_MAX 8192 * Multiple_Y

/*目标值*/ // 固定角度使用时需要与遥控器分离开                      //TODO
/*机械臂*/
extern float Cup_target_pitch;
extern float Cup_target_roll;
extern float yaw;
extern float pic;
/*龙门架*/
extern float X_target;
extern float Y_target;
extern float Z_target;

/*键盘目标值*/
extern float Cup_pitch_temp;
extern float Cup_roll_temp;
extern float yaw_temp;
extern float pic_temp;
extern float X_temp;
extern float Y_temp;
extern float Z_temp;

extern float compensate_Y;
extern float compensate_Z;

/*初始安全位置*/
/*安全倍数*/
extern float Y_safe_multiple;
extern float Uplift_safe_multiple;
/*安全距离*/
#define XY_safe 0
#define X_safe 0
#define Y_safe 8192 * Y_safe_multiple
#define Uplift_safe 8192 * Uplift_safe_multiple

/*控制状态*/
extern int remoteControl;
extern int keyboard;

/*各组成部分位置环复用函数*/
void Cup_location(float Cup_target_pitch, float Cup_target_roll); // 两者之一需要为零
void DM_location(float yaw, float pic);
void XY_location(float X_target, float Y_target);
void Uplift_location(float Z_target);
/*顶层构造角度函数*/
void Cup_fixed(float angle_pitch, float angle_roll);
void DM_fixed(float angle, float pic);
void XY_fixed(float angleX, float angleY);
void Uplift_fixed(float angle);

double msp(double x, double in_min, double in_max, double out_min, double out_max); // 数值映射
void position_init();
void position_task();

#endif