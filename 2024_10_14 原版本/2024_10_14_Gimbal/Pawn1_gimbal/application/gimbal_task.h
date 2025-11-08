#ifndef GIMBAL_H
#define GIMBAL_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "string.h"
#include "remote_control.h"
#include "user_lib.h"
#include "bsp_filter.h"
#include "stdbool.h"
#include "lqr.h"

#define GIMBAL_TASK_INIT_TIME 350
#define GIMBAL_CONTROL_TIME_MS 2
/************************************************************************************/
//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL   0
#define PITCH_CHANNEL 1
#define GIMBAL_MODE_CHANNEL 0
#define CANNON_MODE_CHANNEL 1
/*************************************************************************************/
//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP                 300.0f//120
#define PITCH_SPEED_PID_KI                 5.0f//30
#define PITCH_SPEED_PID_KD                 0.0f
#define PITCH_SPEED_PID_MAX_OUT            28000.0f
#define PITCH_SPEED_PID_MAX_IOUT           10000.0f
//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP                   6946.0f//200
#define YAW_SPEED_PID_KI                   20.0f
#define YAW_SPEED_PID_KD                   3000.0f
#define YAW_SPEED_PID_MAX_OUT              30000.0f
#define YAW_SPEED_PID_MAX_IOUT             5000.0f
//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP         7.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI         0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD         70.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT    40.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT   20.0f
//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP           25.4f
#define YAW_GYRO_ABSOLUTE_PID_KI           0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD           200.0f//281
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT      50.0f //wuc
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT     10.0f
//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP       8.0f//20度时候速度拉满
#define PITCH_ENCODE_RELATIVE_PID_KI       0.0f
#define PITCH_ENCODE_RELATIVE_PID_KD       70.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT  80.0f//期望转正速度最大60转/分
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 10.0f
//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP         8.0f//40度时候速度拉满
#define YAW_ENCODE_RELATIVE_PID_KI         0.0f
#define YAW_ENCODE_RELATIVE_PID_KD         0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT    100.0f //期望转正速度最大120转/分
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT   30.0f
//自瞄用pid
#define YAW_AC_ANGLE_PID_KP         10.0f//40度时候速度拉满
#define YAW_AC_ANGLE_PID_KI         0.0f
#define YAW_AC_ANGLE_PID_KD         80.0f
#define YAW_AC_ANGLE_PID_MAX_OUT    150.0f //期望转正速度最大120转/分
#define YAW_AC_ANGLE_PID_MAX_IOUT   20.0f

#define YAW_AC_SPEED_PID_KP         300.0f//200
#define YAW_AC_SPEED_PID_KI         0.0f
#define YAW_AC_SPEED_PID_KD         0.0f
#define YAW_AC_SPEED_PID_MAX_OUT    28000.0f
#define YAW_AC_SPEED_PID_MAX_IOUT   10000.0f

#define PITCH_AC_ANGLE_PID_KP       10.0f//40度时候速度拉满
#define PITCH_AC_ANGLE_PID_KI       0.0f
#define PITCH_AC_ANGLE_PID_KD       50.0f
#define PITCH_AC_ANGLE_PID_MAX_OUT  100.0f //期望转正速度最大120转/分
#define PITCH_AC_ANGLE_PID_MAX_IOUT 10.0f

#define PITCH_AC_SPEED_PID_KP       300.0f//120
#define PITCH_AC_SPEED_PID_KI       0.0f//30
#define PITCH_AC_SPEED_PID_KD       50.0f
#define PITCH_AC_SPEED_PID_MAX_OUT  28000.0f
#define PITCH_AC_SPEED_PID_MAX_IOUT 10000.0f

/*-------------------- LQR系数 --------------------*/
/********************** PC ***********************/
#define PITCH_CURRENT_LQR_K 			{0, 0, 0, 0}
#define YAW_CURRENT_LQR_K					{0, 0, 0, 0}
// pitch 角度环 角度由陀螺仪解算 LQR参数以及LQR最大误差
#define PITCH_GYRO_ABSOLUTE_LQR_PC_KP 			10000.0f  //160000.0f //150000.0f // 120000.0f
#define PITCH_GYRO_ABSOLUTE_LQR_PC_KI 			100.0f	   //700
#define PITCH_GYRO_ABSOLUTE_LQR_PC_MAX_ERR 	0.0f
// pitch 速度环
#define PITCH_SPEED_LQR_PC_KP        		2000.0f  //6000
// pitch电流前馈
#define PITCH_CURRENT_LQR_PC_KP  		 	0.0f//0.4f
#define PITCH_PC_MAX_OUT 					25000.0f

// yaw 角度环 角度由陀螺仪解算 LQR参数以及 LQR最大误差
#define YAW_GYRO_ABSOLUTE_LQR_PC_KP        	500000.0f
#define YAW_GYRO_ABSOLUTE_LQR_PC_KI        	200.0f
#define YAW_GYRO_ABSOLUTE_LQR_PC_MAX_ERR 	100.0f
// yaw 速度环 LQR参数以及
#define YAW_SPEED_LQR_PC_KP        			45000.0f
// yaw电流前馈
#define yaw_CURRENT_LQR_PC_KP 				0.5f
#define YAW_PC_MAX_OUT 						30000.0f


#define INS_TASK_INIT_TIME 7 //任务开始初期 delay 一段时间
/*******************************编码器相关宏定义**********************************************/
#define HAND_ENCONDE 7156

#define PITCH_COM    -0.474f
#define ENCONDE_2_ANGLE 0.000766990394f //      2*  PI  /8192
/*************************************************************************************/
#define GIMBAL_CONTROL_TIME 1//1
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
/**********************************************************************************/
#define RC_DEADBAND   10
#define PC_P          1
#define YAW_RC_SEN   -0.000004f
#define PITCH_RC_SEN  0.000002f
#define YAW_MOUSE_SEN   0.00002f
#define PITCH_MOUSE_SEN 0.000015f
#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*PI /8192

#define PI					3.14159265358979f
/***************************************************************************/
/***************************************************************************/
typedef struct
{
	int lift_L;
	int lift_r;
	int tran;
	int clamp;
}Rotation_AngleSpeed;

typedef struct
{
	int lift_l;
	int lift_r;
	int tran;
	int clamp;
}Rotation_Disatance;

typedef struct
{
	int lift_l;
	int lift_r;
	int tran;
	int clamp;
}Gimabl_Target_Speed;

typedef enum
{
  GIMBAL_ZERO_FORCE = 0, 
  GIMBAL_INIT,           
  GIMBAL_CALI,           
  GIMBAL_ABSOLUTE_ANGLE, 
	GIMBAL_ABSOLUTE_AUTOMA_ANGLE,
  GIMBAL_RELATIVE_ANGLE, 
  GIMBAL_MOTIONLESS,     
	GIMBAL_REMOTE,
} gimbal_behaviour_e;
typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
	  GIMBAL_MOTOR_AUTOMA,
} gimbal_motor_mode_e;

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
    const motor_measure_t *gimbal_motor_measure;
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;
	  gimbal_PID_t gimbal_motor_AC_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_AC_relative_angle_pid;
		lqr_type_def gimbal_motor_PC_lqr;
    pid_type_def gimbal_motor_gyro_pid;
	  pid_type_def gimbal_motor_AC_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    fp32 max_angle; //rad
    fp32 min_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set1; //rad
	  fp32 relative_angle_set2; //rad
	  
    fp32 absolute_angle;     //rad
	  fp32 absolute_angle_offest;
		fp32 absolute_angle_offest_sum;
	  fp32 absolute_angle_last;     //rad
	  fp32 absolute_angle_dealt;     //rad
    fp32 absolute_angle_set1; //rad
   	fp32 absolute_angle_set2; //rad
  	fp32 absolute_current_set; //rad		
		fp32 absolute_current; //rad		
		
    fp32 motor_gyro;         //rad/s
		fp32 motor_gyro_relative;         //rad/s
	  fp32 motor_gyro_last;         //rad/s
		fp32 motor_gyro_dealt;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
		int16_t given_current1;
		
		fp32 gravity_recordtime;
		fp32 calibration_t[5];
		fp32 calibration;
} gimbal_motor_t;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

/* PC传来的数据 */
typedef struct
{
	int8_t mode; // now no use
  uint8_t fire_flag;
	float text;
	
	fp32 pitch;
	fp32 yaw;
	fp32 distance;
	int8_t number;
	  
	//mubiao aim
	fp32 aim_z;
	fp32 aim_x;
	fp32 aim_y;
	fp32 aim_pitch;
	fp32 aim_yaw;
	//new
	uint8_t header;
  bool tracking : 1;
  uint8_t id : 3;         // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;
	fp32 x;
	fp32 y;
	fp32 z;
	//float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
  uint16_t checksum;
	
	// Pose in n pixel
	fp32 Pose_X;
	fp32 Pose_Y;
	fp32 Pose_Z;
	
	fp32 pitch_last;
	fp32 yaw_last;
	fp32 distance_last;
	
} PC_data_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
		uint8_t shoot_mood;
		uint16_t power_limit;
} gimbal_control_t;

typedef struct
{
    int8_t x_flag;
		int8_t y_flag;
	  uint16_t High;
		int8_t w_flag;
		int8_t sp_flag;
		int8_t tl_flag;
	  int8_t sport_flag;
		uint8_t protect_flag;
		int8_t jump_flag;
	  int8_t side_flag,common_flag;
	  uint8_t sit_flag,stand_flag;
		int8_t remake_flag;
		int8_t mode;
		uint16_t power_limit;
		uint8_t reborn_flag;
} sup_cup_t;
int16_t gimbal_LQR_calc(fp32 K1,fp32 K2,fp32 K0,fp32 angle_get,fp32 speed_get,fp32 angle_set,fp32 speed_set);
extern int8_t Pawn_mode;
extern float error_angle,error_angle1;
extern gimbal_control_t gimbal_control;
const gimbal_motor_t *get_yaw_motor_point(void);
const gimbal_motor_t *get_pitch_motor_point(void);
static fp32 pitch_ecd_to_angle_change(uint16_t ecd);
extern float yaw_K;
//extern fp32 LQR_yaw[3],LQR_pitch[3],LQR_ACyaw[3],LQR_ACpitch[3];
extern gimbal_behaviour_e gimbal_behaviour;//行为层模式设置
extern sup_cup_t  sup_cap;
extern int16_t gimbal_PI_LQR(fp32 KP,fp32 KI,fp32 K1,fp32 K2,fp32 K3,//PI_LQR Serise
										  fp32 angle_get,fp32 speed_get,fp32 current_get,
									    fp32 angle_set);
extern uint16_t Guard_shoot;
extern fp32 LQR_PI_pitch[5];
extern fp32 LQR_PI_pitch_AC[5];
extern fp32 LQR_PI_yaw[5];
extern fp32 LQR_PI_yaw_AC[5];
extern kalman2_state tast_K2;
extern uint8_t test_fire_flag;
void red_linght_close();
void red_linght_open();
void BEEP_close();
void BEEP_open();
#endif 
