/*
 * @Descripttion:
 * @version:
 * @Author: lxf
 * @Date: 2023-03-23 13:06:38
 * @LastEditors: lxf
 * @LastEditTime: 2023-05-01 10:49:00
 */
#ifndef __KEYSCAN_TASK_H
#define __KEYSCAN_TASK_H

// #include "main.h"
#include "remote_control.h"
#include "user_lib.h"
#include "struct_typedef.h"
#include "cmsis_os.h"
#include "pid.h"
#include "usart.h"
#include "tim.h"
#include "string.h"
#include "custom.h"

/*结构体*/
typedef struct
{
	uint8_t die_last;
	uint8_t die_now;
	uint8_t die_flag;
} die_t;
extern die_t die;

typedef struct
{
	int16_t Vx;
	int16_t Vy;
	int16_t Vz;

	float speed;
} chassis_speed_t;

typedef struct
{
	int16_t go_yaw;
	int16_t go_up;
	int16_t go_little_yaw;
	int16_t pos_roll;
	int16_t pos_pitch;
	int16_t pos_yaw;
	/*float->int16_t*/
	float go_yaw_pos;
	float go_up_pos;
	float go_little_yaw_pos;
	float pos_roll_pos;
	float pos_pitch_pos;
	float pos_yaw_pos;

} motor_cmd_typedef_t;
typedef struct
{
	const RC_ctrl_t *key_rc_ctrl;
	chassis_speed_t key_speed; // 底盘XYZ速度计算
	uint8_t chassis_mode;	   // 0为底盘运动，1为停止键盘控制取矿

	/*新增模式*/
	uint8_t custom_mode; // 1为自定义控制器控制
	int Base;
	int Safe;
	int Cashing;
	int Cashing_yaw;
	int Gold;
	int Silver;

	/*一键存矿*/
	int deposit;
	int extraction;

	uint8_t go_die_flag;
} key_typedef_t;
/*结构体*/

/*GPIO_define*/
/*小臂气泵*/
#define Cup_GPIO_Port GPIOE
#define Cup_Pin GPIO_PIN_6
/*小臂泄压阀*/
#define VALVE_Pin GPIO_PIN_0
#define VALVE_GPIO_Port GPIOB
/*左气泵&&泄压阀*/
#define Left_GPIO_Port GPIOC
/*左气泵&&泄压阀号*/
#define Left_Pin GPIO_PIN_3		  // 左气泵
#define Left_logic_pin GPIO_PIN_4 // 左泄压阀
/*右气泵&&泄压阀*/
#define Right_GPIO_pump GPIOB
#define Right_GPIO_logic GPIOC
/*右气泵&&泄压阀号*/
#define Right_Pin_pump GPIO_PIN_1
#define Right_Pin_logic GPIO_PIN_0
/*继电器*/
#define Relays_Pin GPIO_PIN_2
#define Relays_GPIO_Port GPIOC
/*GPIO_define*/

/*DEFINE*/ /*七个自由度*/
/*safe 初始位置*/
#define Uplift_obtain 0.0f // 先Uplift_safe、Y_safe再这个
#define Y_obtain 0.0f;	   //! 安全位置Y
#define X_obtain 0.0f;	   //! 中间X
#define Yaw_obtain 0.0f;
#define Pitch_obtain 180.0f;
#define Cup_pitch_obtain -10.0f; //! 实际测下角度是否正确
#define Cup_roll_obtain 0.0f;
/*兑矿位置*/ //!(三级矿)
/*小兑矿*/
/*左侧面*/
#define Uplift_Cash_0 24.3f; // 对应800
#define Y_Cash_0 45.0f;
#define X_Cash_0 0.0f;
#define Yaw_Cash_0 0.0f;
#define Pitch_Cash_0 105.0f;
#define Cup_pitch_Cash_0 -30.0f;
#define Cup_roll_Cash_0 45.0f;
/*右侧面*/
#define Uplift_Cash_1 24.3f; // 对应800
#define Y_Cash_1 45.0f;
#define X_Cash_1 0.0f;
#define Yaw_Cash_1 0.0f;
#define Pitch_Cash_1 -105.0f;
#define Cup_pitch_Cash_1 -30.0f;
#define Cup_roll_Cash_1 45.0f;

/*大兑矿*/
/*左侧面*/
#define Uplift_Cash_yaw_0 24.3f; // 对应800
#define Y_Cash_yaw_0 45.0f;
#define X_Cash_yaw_0 0.0f;
#define Yaw_Cash_yaw_0 0.0f;
#define Pitch_Cash_yaw_0 90.0f;
#define Cup_pitch_Cash_yaw_0 -30.0f;
#define Cup_roll_Cash_yaw_0 0.0f;
/*右侧面*/
#define Uplift_Cash_yaw_1 24.3f; // 对应800
#define Y_Cash_yaw_1 45.0f;
#define X_Cash_yaw_1 0.0f;
#define Yaw_Cash_yaw_1 0.0f;
#define Pitch_Cash_yaw_1 -90.0f;
#define Cup_pitch_Cash_yaw_1 -30.0f;
#define Cup_roll_Cash_yaw_1 0.0f;

/*金矿位置*/
// 靠近矿的左侧吸
#define Uplift_GOLD 6.2f
#define Y_gold 10.0f;
#define X_gold 0.0f;
#define Yaw_gold 0.0f;
#define Pitch_gold 90.0f;
#define Cup_pitch_gold 83.0f;
#define Cup_roll_gold 0.0f;
/*取银矿初始位置*/
#define Uplift_silver_1 5.0f;
#define Uplift_silver_2 1.5f;
#define Uplift_silver_3 10.0f;
#define Uplift_silver_4 1.0f;
#define Y_silver_1 34.0f;
#define Y_silver_2 43.0f;
#define Y_silver_3 5.0f;
#define X_silver 0.0f;
#define Yaw_silver 0.0f;
#define Pitch_silver_1 0.0f;
#define Pitch_silver_2 180.0f; // TODO方向
#define Cup_pitch_silver_1 -15.0f;
#define Cup_pitch_silver_2 -50.0f;
#define Cup_roll_silver 0.0f;
/*一键存矿初始位置*/
#define Uplift_deposit 7.0f;
#define Y_deposit_1 44.0f
#define Y_deposit_2 42.0f;
#define X_deposit 0.0f;
#define Yaw_deposit 62.0f;
#define Pitch_deposit 96.0f;
#define Cup_pitch_deposit -49.0f
#define Cup_roll_deposit 15.0f;
/*一键取矿初始位置*/
#define Uplift_extraction 4.0f;
#define Y_extraction_1 49.0f;
#define Y_extraction_2 34.0f;
#define X_extraction 0.0f;
#define Yaw_extraction 60.0f;
#define Pitch_extraction 90.0f;
#define Cup_pitch_extraction -10.0f
#define Cup_roll_extraction 0.0f;

/*函数*/
void key_scan_init(key_typedef_t *key_init);
void keyscan_task(void const *argument); // Freertos
void limit_key(float *value, float max, float min);
void limit_key_int16(int16_t *value, int16_t max, int16_t min);
// void pos_update(key_typedef_t *key_update); // qt数据更新float->int16_t
//!/*死亡复位*/
void renew_mode(void); // 复活杀死之前程序，重新初始化，并且使机械臂能够回到原来的相对值

/*全局变量*/
extern key_typedef_t key_task;

/*死亡复位标志位*/
extern int reposition_flag_Y;
extern int reposition_flag_Cup;
extern int Again_flag;
extern int chassis_move_flag;
extern int Cup_X_reset;
extern int direction;
extern int Cup_SF;
extern int Right_SF;
extern int Left_SF;
extern int Spinning_Top_L;
extern int Spinning_Top_R;

/**
 *
 *
 *
 *
 *
 *
 *
 */
/*借鉴2*/
#define K_key_to_3508 20.64969697f // TODO可能和底盘PID有关
/*待定*/
#define Vcc2_Pin GPIO_PIN_4
#define Vcc2_GPIO_Port GPIOE
#define Gnd2_Pin GPIO_PIN_5
#define Gnd2_GPIO_Port GPIOE
#define QiBeng2_Pin GPIO_PIN_6
#define QiBeng2_GPIO_Port GPIOE
#define Vcc1_Pin GPIO_PIN_0
#define Vcc1_GPIO_Port GPIOF
#define Gnd2F1_Pin GPIO_PIN_1
#define Gnd2F1_GPIO_Port GPIOF
#define NEW_BENG_Pin GPIO_PIN_1
#define NEW_BENG_GPIO_Port GPIOB
#define QiBeng1_Pin GPIO_PIN_12
#define QiBeng1_GPIO_Port GPIOE

#endif