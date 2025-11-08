/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       #include "Classis_Control.h"
  * @brief      底盘控制逻辑和参数
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2021\9\6        钟涵海
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef ClASSIS_H
#define ClASSIS_H
#include "struct_typedef.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "pid.h"
#include "string.h"
#include "user_lib.h"
#include "gimbal_task.h"
/*******************************************************************/
#define CHASSIS_TASK_INIT_TIME 350
#define CHASSIS_CONTROL_TIME_MS 2
/*******************************************************************/
#define CHASSIS_Y_CHANNEL 2      //左右的遥控器通道号码
#define CHASSIS_X_CHANNEL 3      //前后的遥控器通道号码
#define CHASSIS_WZ_CHANNEL 0     //在全遥控模式下，可以通过遥控器控制旋转
/*******************************************************************/
#define CHASSIS_MODE_CHANNEL 0   //选择底盘状态 开关通道号
#define CHASSIS_VX_RC_SEN 10.0f //遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VY_RC_SEN -5.0f //遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_WZ_RC_SEN 10.0f
/*******************************************************************/
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
/*******************************************************************/
//chassi forward, back, left, right key
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY     KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY      KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY      KEY_PRESSED_OFFSET_D
#define CHASSIS_RIGHT_KEY     KEY_PRESSED_OFFSET_A
#define SJ                    KEY_PRESSED_OFFSET_C
#define GSXTL                 KEY_PRESSED_OFFSET_E
#define DSXTL                 KEY_PRESSED_OFFSET_Q
#define ZC                    KEY_PRESSED_OFFSET_R
#define TL                    KEY_PRESSED_OFFSET_V
/*******************************************************************/
//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15.0f
#define M3505_MOTOR_SPEED_PID_KI 0.2f
#define M3505_MOTOR_SPEED_PID_KD 10.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT  8000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 7000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 200.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 8000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 1000.0f
/*******************************************************************/
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.1666666667f
#define CHASSIS_CONTROL_TIME 0.002f
#define MOTOR_DISTANCE_TO_CENTER 0.26f
//摇杆死区
#define CHASSIS_RC_DEADLINE 10
#define CHASSIS_OPEN_RC_SCALE 5 //遥控器乘以该比例发送到can上
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
/*****************************转速转真实速度*******************************/
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
/*******************************************************************/
typedef struct
{
    int speed_y_target;
    int speed_x_target;
    int speed_r_target;
    int anglespeed_r_target;
} Speed_target;

typedef struct
{
    int motor_l1;
    int motor_l2;
    int motor_r1;
    int motor_r2;
} Speed_Resolving;
/*******************************************************************/
typedef enum
{
    CHASSIS_ZERO_FORCE,                   //chassis will be like no power,底盘无力, 跟没上电那样
    CHASSIS_NO_MOVE,                      //chassis will be stop,底盘保持不动
    CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,   //chassis will follow gimbal, usually in infantry,正常步兵底盘跟随云台
    CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,  //chassis will follow chassis yaw angle, usually in engineer,
    //because chassis does have gyro sensor, its yaw angle is calculed by gyro in gimbal and gimbal motor angle,
    //if you have a gyro sensor in chassis, please updata yaw, pitch, roll angle in "chassis_feedback_update"  function
    //工程底盘角度控制底盘，由于底盘未有陀螺仪，故而角度是减去云台角度而得到，
    //如果有底盘陀螺仪请更新底盘的yaw，pitch，roll角度 在chassis_feedback_update函数中
    CHASSIS_NO_FOLLOW_YAW,                //chassis does not follow angle, angle is open-loop,but wheels have closed-loop speed
    //底盘不跟随角度，角度是开环的，但轮子是有速度环
    CHASSIS_OPEN                          //the value of remote control will mulitiply a value, get current value that will be sent to can bus
    // 遥控器的值乘以比例成电流值 直接发送到can总线上
} chassis_behaviour_e;
typedef enum
{
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //chassis will follow yaw gimbal motor relative angle.底盘会跟随云台相对角度
    CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //chassis will have yaw angle(chassis_yaw) close-looped control.底盘有底盘角度控制闭环
    CHASSIS_VECTOR_NO_FOLLOW_YAW,       //chassis will have rotation speed control. 底盘有旋转速度控制
    CHASSIS_VECTOR_RAW,                 //control-current will be sent to CAN bus derectly.

} chassis_mode_e;
typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 real_speed;
    fp32 speed_set;
    int16_t give_current;
} chassis_motor_t;

//typedef struct
//{
//	const cap_measure_t *cap_measure;
//} chassis_cap_t;
/*******************************************************************/
typedef struct
{
    const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针, the point to remote control
    const gimbal_motor_t *chassis_yaw_motor;   //will use the relative angle of yaw gimbal motor to calculate the euler angle.底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角.
    const gimbal_motor_t *chassis_pitch_motor; //will use the relative angle of pitch gimbal motor to calculate the euler angle.底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
    const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor.获取陀螺仪解算出的欧拉角指针
    chassis_mode_e chassis_mode;               //state machine. 底盘控制状态机
    chassis_mode_e last_chassis_mode;          //last state machine.底盘上次控制状态机
    chassis_motor_t motor_chassis[4];          //chassis motor data.底盘电机数据
//	chassis_cap_t   chassis_cap;               //超级电容数据

    pid_type_def motor_speed_pid[4];             //motor speed PID.底盘电机速度pid
    pid_type_def chassis_angle_pid;              //follow angle PID.底盘跟随角度pid

    first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
    first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
    first_order_filter_type_t chassis_cmd_slow_set_wz;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值

    fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s. 底盘速度 前进方向 前为正，单位 m/s
    fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.底盘速度 左右方向 左为正  单位 m/s
    fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.底盘旋转角速度，逆时针为正 单位 rad/s
    fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.底盘设定速度 前进方向 前为正，单位 m/s
    fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.底盘设定速度 左右方向 左为正，单位 m/s
    fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 单位 rad/s
    fp32 chassis_relative_angle;      //the relative angle between chassis and gimbal.底盘与云台的相对角度，单位 rad
    fp32 chassis_relative_angle_set;  //the set relative angle.设置相对云台控制角度
    fp32 chassis_yaw_set;

    fp32 vx_max_speed;  //max forward speed, unit m/s.前进方向最大速度 单位m/s
    fp32 vx_min_speed;  //max backward speed, unit m/s.后退方向最大速度 单位m/s
    fp32 vy_max_speed;  //max letf speed, unit m/s.左方向最大速度 单位m/s
    fp32 vy_min_speed;  //max right speed, unit m/s.右方向最大速度 单位m/s
    fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的yaw角度
    fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的pitch角度
    fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的roll角度

    fp32 cap_Value_Bat;   //电池电压
    fp32 cap_Value_Cap;   //电容电压
    fp32 cap_Power_Charge;//输入功率
    fp32 cap_Power_Chassis;//底盘功率


    fp32 current_sum;
} chassis_move_t;
/*******************************************************************/
extern Speed_target    Classis_TarSpd;//运行期望速度
extern Speed_Resolving Classis_ResSpd;//解算后速度
extern chassis_move_t chassis_move;
extern chassis_behaviour_e chassis_behaviour_mode ;
extern  pid_type_def motor_classis_left1_pid;					    //声明PID数据结构体
extern  pid_type_def motor_classis_left2_pid;
extern	pid_type_def motor_classis_right1_pid;
extern	pid_type_def motor_classis_right2_pid;
extern	const motor_measure_t *motor_classis_left1_data;	//声明电机结构体指针
extern	const motor_measure_t *motor_classis_left2_data;
extern	const motor_measure_t *motor_classis_right1_data;
extern	const motor_measure_t *motor_classis_right2_data;
extern	const fp32 motor_left1_PIDdata[3];	    //P,I,D
extern	const fp32 motor_left2_PIDdata[3];
extern	const fp32 motor_right1_PIDdata[3];
extern	const fp32 motor_right2_PIDdata[3];
extern	int motor_left1_speed;                     //速度设定
extern	int motor_left2_speed;
extern	int motor_right1_speed;
extern	int motor_right2_speed;

void classis_task(void const * argument);
void Classis_Int(chassis_move_t *chassis_move_init);
void Resolve_Speed(void);//速度解算函数;
void Classis_PID(chassis_move_t *chassis_move_control_loop);

#endif
