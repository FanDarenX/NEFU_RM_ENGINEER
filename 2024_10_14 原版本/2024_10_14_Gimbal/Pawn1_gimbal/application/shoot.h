#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"

//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1
//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15

//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18
//拨弹速度
#define TRIGGER_SPEED               -3240.0f

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1
//卡单时间 以及反转时间
#define PI_FOUR                     -0.78539816339744830961566084581988f
#define PI_TEN                      -0.7854f
/***************摩擦轮转速宏定义*******************/
#define SPEED_15 4000
#define SPEED_18 4470
#define SPEED_22 6000
#define SPEED_30 7000
/**************************************************/
//拨弹轮电机PID
#define TRIGGER_SPEED_PID_KP        20.0f
#define TRIGGER_SPEED_PID_KI        3.0f
#define TRIGGER_SPEED_PID_KD        0.0f
#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 8000.0f

#define TRIGGER_ANGLE_PID_KP   150.0f
#define TRIGGER_ANGLE_PID_KI   2.0f
#define TRIGGER_ANGLE_PID_KD   50.0f
#define TRIGGER_ANGLE_PID_MAX_OUT  8000.0f
#define TRIGGER_ANGLE_PID_MAX_IOUT 1000.0f
//摩擦轮电机PID
#define FIRE1_SPEED_PID_KP        50.0f//40
#define FIRE1_SPEED_PID_KI        1.0f
#define FIRE1_SPEED_PID_KD        60.0f//60
#define FIRE2_SPEED_PID_KP        40.0f//40
#define FIRE2_SPEED_PID_KI        1.0f
#define FIRE2_SPEED_PID_KD        80.0f//60
#define FIRE_BULLET_PID_MAX_OUT  15000.0f
#define FIRE_BULLET_PID_MAX_IOUT 2000.0f
//枪管切换
#define CURRENT_LIMIT 5000
#define ANGLE_DELAT   20
#define BARREL_SPEED_PID_KP        10.0f//40
#define BARREL_SPEED_PID_KI        1.0f
#define BARREL_SPEED_PID_KD        0.0f//60
#define BARREL_PID_MAX_OUT  6000.0f
#define BARREL_PID_MAX_IOUT 2000.0f
/****************************************/
#define SHOOT_LOW_SPEED        8500 //8500//4320
#define SHOOT_HIGH_SPEED       8500 // 8500//4320

#define SHOOT_HEAT_REMAIN_VALUE  80

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,
    SHOOT_FIRE,
} shoot_mode_e;

typedef enum
{
    TRIGGER_A ,
    TRIGGER_T,
    TRIGGER_S =0 ,
} trigger_mode_e;

typedef struct
{
    shoot_mode_e   shoot_mode;
	  trigger_mode_e trigger_mode;
	  trigger_mode_e last_trigger_mode;
	
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
	  const motor_measure_t *fire_motor1_measure;
	  const motor_measure_t *fire_motor2_measure;
	  const motor_measure_t *barrel_motor_measure;

    pid_type_def trigger_motor_pid;
	  pid_type_def trigger_angle_motor_pid;
		pid_type_def fire1_motor_pid;
	  pid_type_def fire2_motor_pid;
	  pid_type_def barrel_motor_pid;
	
    fp32 trigger_speed_set;
    fp32 trigger_speed;
		fp32 trigger_current;
    fp32 fire_speed1_set;
	  fp32 fire_speed2_set;
	  fp32 fire_speed1;
	  fp32 fire_speed2;
		fp32 fire1_temperate;
		fp32 fire2_temperate;
	
		fp32 barrel_speed;
		fp32 barrel_speed_set;
	  fp32 barrel_angle;
		fp32 barrel_angle_set;
		fp32 barrel_current;
		fp32 barrel_given_current;
	
    fp32 angle;
    fp32 set_angle;
    fp32 ecd_count;
		fp32 barrel_count;
		fp32 bullet_speed;
		
    bool_t move_flag;

    uint16_t heat_limit;
    uint16_t heat;
		
		//int16_t trigger_current;
	  int16_t fire1_current;
	  int16_t fire2_current;
		
		uint16_t shooter_heat;
		uint16_t shooter_heat_limit;
		
		//陈君代码要用的部分
		uint8_t fire_flag;   //反小陀螺开火标志
} shoot_control_t;

//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern void shoot_init(void);
/************************************射击进程******************************************/
extern void shoot_control_loop(void);
extern shoot_control_t shoot_control;
extern void bullet_speed_get();
extern shoot_control_t shoot_control;
extern uint16_t fire_speed;
extern uint8_t mc_num,mc_flag;
extern int8_t change_flag;
extern uint8_t barrel_choice_flag;//1是1号 2是2号 
extern uint8_t barrel_ready_flag;//到位置以后置1
#endif
