#ifndef GIMBAL_H
#define GIMBAL_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "string.h"
#include "remote_control.h"
#include "user_lib.h"
#include "bsp_filter.h"
#include "motor_control.h"
#include "A1_motor.h"
#include "kalman_filter.h"

/*********************************延时时间***************************************/
#define CHASSIS_TASK_INIT_TIME   350
#define CHASSIS_CONTROL_TIME_MS    1
/**********************************************************************************/
//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL             0
#define PITCH_CHANNEL           1
#define RIGHT_CHANNEL           0
#define CANNON_MODE_CHANNEL     1
#define INS_TASK_INIT_TIME      7       //任务开始初期 delay 一段时间
/*****************************编码器相关宏定义****************************************/
#define HAND_ENCONDE           900
#define PITCH_COM              -2.556f
#define ENCONDE_2_ANGLE        0.000766990394f     // 2*PI      /8192
#define ENCONDE_2_SPEED        0.002652582382f     // 0.15915f 
/**********************************************************************************/
#define GIMBAL_CONTROL_TIME         1//1
//电机反馈码盘值范围
#define HALF_ECD_RANGE              32768
#define ECD_RANGE                   65536
#define HT_HALF                     50
/**********************************************************************************/
#define RC_DEADBAND          10
#define PC_P                  1
#define YAW_RC_SEN           -0.000004f
#define PITCH_RC_SEN          0.000002f
#define YAW_MOUSE_SEN         0.00002f
#define PITCH_MOUSE_SEN       0.000015f
#define YAW_ENCODE_SEN        0.01f
#define PITCH_ENCODE_SEN      0.01f

#define MOTOR_ECD_TO_RAD      0.0000958738f //      2*PI /65536
#define MOTOR_TO_SPEED        0.0000958738f //      2*PI /65536
#define POSITION_SIT          0.0f
#define POSITION_STAND_LOW    0.0f
#define POSITION_STAND_HIGH   0.0f

#define PI2					  6.28318530717959f
#define PI					  3.14159265358979f
#define PI_2					1.57079632679489f
#define PI_4					0.78539816339744f
#define L1   0.15f
#define L2   0.25f
#define L3   0.25f
#define L4   0.15f
#define L5   0.11f
#define WHEEL_DIA      		 0.628318533f
#define LEG_OFFEST     		 0.3364994797f//20度 
#define FRONT_LEG_OFFEST   0.324707389f//20度 
#define BACK_LEG_OFFEST    0.39820371f//20度 
#define ALL_WEIGHT_2   		 0.2325f
#define PITCH_OFFSET   		 0.0000f
/***************************************************************************/
/***************************************************************************/
typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,
	  CHASSIS_PURE,
	  CHASSIS_PURE_SIT,
	  CHASSIS_PURE_HIGH,
    CHASSIS_FOLLOW_GIMBAL,
    CHASSIS_NO_FOLLOW,
    CHASSIS_SIT,
} chassis_behaviour_e;
typedef enum
{
    CHASSIS_MOTOR_ZERO = 0, //电机放松
    CHASSIS_MOTOR_FORCE,    //电机上劲
} chassis_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    fp32 pitch_angle_set,roll_angle_set,yaw_angle_set;   //期望姿态
    fp32 pitch_angle,roll_angle,yaw_angle;       //姿态角度
    fp32 pitch_gyro,roll_gyro,yaw_gyro;       //姿态角度

    fp32 leg_length_L_set,leg_length_R_set;
    fp32 leg_length_L,leg_length_R;//逆运动学结算后的腿长度
    fp32 leg_dlength_L,leg_dlength_R;//腿长度变化率
    fp32 leg_angle_set;
    fp32 leg_angle_L,leg_angle_R;//逆运动学结算后的腿与机身角度
    fp32 leg_gyro_L,leg_gyro_R;//逆运动学结算后的腿与竖直角速度
    fp32 leg_dangle_L,leg_dangle_R;
    fp32 leg_accel_L,leg_accel_R;

    fp32 foot_speed_R_set,foot_speed_L_set;
    fp32 foot_speed_set;
    fp32 foot_speed,foot_speed_K;//解算后的足部线速度
		fp32 foot_speed_Q;
    fp32 foot_distance_set;
    fp32 foot_distance_R_set;
    fp32 foot_distance_L_set;
    fp32 foot_distance,foot_distance_K;//解算后的足部运动距离

    fp32 torque_K_balance_L;//平衡用力矩
    fp32 torque_K_balance_R;
    fp32 torque_K_coordinate;  //协调用力矩
    fp32 torque_K_roll;   //侧倾用力矩
    fp32 torque_K_stand_L;
    fp32 torque_K_stand_R;
    fp32 torque_K_yaw;   //转弯用力矩
    fp32 F_foot_L,F_foot_R;
    fp32 Tp_foot_L,Tp_foot_R;		

    fp32 J1_L,J2_L;
    fp32 J3_L,J4_L;
    fp32 J1_R,J2_R;
    fp32 J3_R,J4_R;
    fp32 invJ1_L,invJ2_L;
    fp32 invJ3_L,invJ4_L;
    fp32 invJ1_R,invJ2_R;
    fp32 invJ3_R,invJ4_R;
		
		fp32 dL0_L,ddL0_L;
		fp32 dseita_L,ddseita_L;
		fp32 dL0_R,ddL0_R;
		fp32 dseita_R,ddseita_R;		

    fp32 foot_roll_angle;
    fp32 High;

    uint8_t balance_ready;//是否正常直立
    uint8_t suspend_flag;//是否正常直立
		uint8_t L_suspend_flag;//左腿是否悬空
		uint8_t R_suspend_flag;//右腿是否悬空
		uint8_t save_mode;
    uint16_t jump_time;
} chassis_balance_t;

typedef struct
{
    //const HT_motor_measure_t *motor_measure;
    //const MOTOR_recv *motor_measure;
    const A1_MOTOR_recv *motor_measure;

    chassis_motor_mode_e motor_mode;
    chassis_motor_mode_e last_motor_mode;

    fp32 position_set;
    fp32 position_offset;
    fp32 position;
    fp32 position0;
    fp32 position_kp;
    fp32 velocity_set;
    fp32 velocity;
    fp32 accel;
    fp32 velocity0;
    fp32 velocity_kp;

    fp32 torque_balance;//平衡用力矩
    fp32 torque_stand;  //站立用力矩（或力矩参数）
    fp32 torque_roll;   //侧倾用力矩
    fp32 torque_yaw;   //转弯用力矩
    fp32 torque_out;		//输出总力矩

    fp32 position_limit_MAX;
    fp32 position_limit_MIN;
} chassis_unterleib_motor_t;

typedef struct
{
    const motor_measure_t *motor_measure;

    chassis_motor_mode_e motor_mode;
    chassis_motor_mode_e last_motor_mode;

    fp32 position_set;
    fp32 position;
    fp32 position_kp;
    fp32 velocity_set;
    fp32 velocity;
    fp32 accel;
    fp32 velocity_kp;
    fp32 speed;
    fp32 speed_K;
    fp32 distance;
    fp32 distance_K;
    fp32 distance_offest;
    fp32 turns;

    fp32 torque_out;		//输出总力矩
    fp32 torque_get;		//负载力矩
    fp32 power_now;
    fp32 power_set;
} chassis_foot_motor_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    pid_type_def gimbal_motor_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
} gimbal_motor_t;
typedef struct
{
    const Gimbal_ctrl_t *chassis_rc_ctrl; 
		const RC_ctrl_t *rc_ctrl;
    const cap_measure_t *chassis_cap_ctrl;
    const fp32 *chassis_INT_angle_point;
    const fp32 *chassis_INT_gyro_point;
    chassis_balance_t chassis_balance;
    chassis_unterleib_motor_t unterleib_motor1;//   |1      3|
    chassis_unterleib_motor_t unterleib_motor2;//  L|        |R
    chassis_unterleib_motor_t unterleib_motor3;//   |        |
    chassis_unterleib_motor_t unterleib_motor4;//   |2      4|
    chassis_foot_motor_t foot_L_motor;
    chassis_foot_motor_t foot_R_motor;
    gimbal_motor_t yaw_motor;

    uint8_t stand_ready,stand_ready_last;
		uint8_t jump_ready,jump_ready_last;
} chassis_control_t;

typedef struct
{
    int8_t x_flag;
    int8_t y_flag;
    int8_t w_flag;
    int8_t sp_flag;
    int8_t tl_flag;
    int8_t mode;
    uint16_t power_limit;
} sup_cup_t;
int16_t gimbal_LQR_calc(fp32 K1,fp32 K2,fp32 K0,fp32 angle_get,fp32 speed_get,fp32 angle_set,fp32 speed_set);
extern float error_angle,error_angle1;
static fp32 pitch_ecd_to_angle_change(uint16_t ecd);
extern float yaw_K;
extern sup_cup_t  sup_cap;
extern uint8_t test_flag;
extern int16_t gimbal_PI_LQR(fp32 KP,fp32 KI,fp32 K1,fp32 K2,fp32 K3,//PI_LQR Serise
                             fp32 angle_get,fp32 speed_get,fp32 current_get,
                             fp32 angle_set);
extern chassis_control_t chassis_control;
extern KalmanFilter_t distance_KF;
extern void Chassis_Int(chassis_control_t *init);

extern fp32 Angle_conversion(fp32 Q1);
extern fp32 Speed_ramp(fp32 speed_set,fp32 foot_speed);

void gh_Zaccel_init(void);
void gh_Zaccel_update(void);
float dynamic_feed_adj(void);

#endif
