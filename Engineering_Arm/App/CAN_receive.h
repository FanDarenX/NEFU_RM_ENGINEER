/*
 * @Descripttion:
 * @version:
 * @Author: lxf
 * @Date: 2022-12-04 13:40:39
 * @LastEditors: lxf
 * @LastEditTime: 2023-04-10 21:55:00
 */
#include "main.h"
#include "can.h"
#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"
#include "remote_control.h"

#define PI 3.14159265358979323846f

/*电机ID结构体*/
typedef enum
{
	/*CAN1*/
	/*底盘电机*/
	CAN_CHASSIS_TX_ID = 0x200, // 2006、3508电机接收标识符
	CAN_3508M1_ID = 0x201,
	CAN_3508M2_ID = 0x202,
	CAN_3508M3_ID = 0x203,
	CAN_3508M4_ID = 0x204,

	/*DM-机械臂yaw、pitch*/ // Master ID
	CAN_YAW_DM_ID = 0x02,	// 对应CAN ID 0x03
	CAN_PITCH_DM_ID = 0x00, // 对应CAN ID 0x01

	/*2006电机-机械臂吸盘pitch、roll*/
	CAN_2006M1_ID = 0X205, // 2006电机标识符
	CAN_2006M2_ID = 0x206,
	/*CAN1*/

	/*CAN2*/

	/*A_TO_C*/
	CAN2_A_TO_CAN1_C_Chassis = 0x101,
	CAN2_A_TO_CAN1_C_XY = 0x102,

	/*龙门架抬升z轴*/		// 试试可不可以用0x201
	CAN_3508_TX_ID = 0x1FF, // 3508电机5~8接收标识符和6020和2006
	CAN_3508MA_ID = 0x201,
	CAN_3508MB_ID = 0x202,
	CAN_3508MC_ID = 0x203,
	CAN_3508MD_ID = 0x204,

	/*龙门架横向x轴*/
	CAN_2006LX_ID = 0x206, // 后
	/*龙门架纵向y轴*/
	CAN_2006LY_ID = 0x205, // 先
	/*CAN2*/

	/**
	 *
	 *
	 *
	 *
	 *
	 *
	 *
	 */
	/*淘汰变量*/
	CAN_img_3508_ID = 0x204,	  // 图传3508电机
	CAN_POSITION_6020_ID = 0x207, // 207
	CAN_YAW_6020_ID = 0x208,
	CAN_SEND_TO_C_SPEED_ID = 0X207, // 发送速度时的id
	CAN_SEND_TO_QI_O_ID = 0X206,
	CAN_SEND_TO_C_GO_ID = 0X205,
	CAN_SEND_TO_C_END_ID = 0X206,
} CAN_MSG_ID;

/*CAN反馈值结构体*/
typedef struct
{
	uint16_t angle; // 转子角度 (0~8191)
	int16_t speed;	// 转子速度
	int16_t last_speed;
	int16_t real_current; // 电机电流
	int16_t temperature;  // 电机温度
	uint16_t last_angle;
	uint16_t offset_angle; // 补偿角度
	int32_t round_cnt;	   // 转子转动圈数
	int32_t total_angle;   // 转子转动总角度
	int32_t last_total_angle;
	int32_t angle_err;
	int32_t angle_ERR;
	int32_t total_angle_true;
} moto_measure;

extern moto_measure chassis_moto[4]; // 底盘电机
/*机械臂吸盘*/
extern moto_measure Cup_moto[2];
extern motor_t DM_yaw;
extern motor_t DM_pitch; // 由于按照初版机械构图为俯仰角pitch，当前机械构图为roll。方便起见均按照pitch写
/*龙门架抬升z轴电机*/
extern moto_measure Uplift_moto[4]; // 龙门架抬升电机
/*龙门架横向x电机*/
extern moto_measure X_moto;
/*龙门架纵向y电机*/
extern moto_measure Y_moto;
/**/
void get_moto_measure(moto_measure *ptr, uint8_t RXdata[8]); // 电机反馈值
const moto_measure *moto_measure_point(int i);
const moto_measure *moto_ground_measure_point(int i);
// 函数声明

float ABS(float x);			// 绝对值
void DM4310_YAW(float pos); /*角度灯*/

/*电机电流控制函数*/
void CHASSIS_CURRENT(int16_t K1, int16_t K2, int16_t K3, int16_t K4);
void Uplift_CURRENT(int16_t K1, int16_t K2, int16_t K3, int16_t K4);
void Cup_CURRENT(int16_t K1, int16_t K2);
void XY_CURRENT(int16_t K1, int16_t K2);

/*A_TO_C*/
void A_TO_C_Chassis(int16_t x, int16_t y, int16_t r);
void A_TO_C_XY(float X, float Y);

/**
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
/*淘汰函数和变量*/
extern moto_measure position_moto_6020; // 末端6020电机
extern moto_measure yaw_moto_6020;		// yaw轴6020电机
extern moto_measure moto_6020[2];
// extern C_GO_POS_t c_go_pos; // c板发送的go电机位置
typedef struct
{
	int16_t GO_yaw;
	int16_t GO_up;
	int16_t GO_little_yaw;
	uint8_t live_flag;
	uint8_t mode;
} C_GO_POS_t;
void CAN_cmd_to_Qi_BOARD_flag(uint8_t Qi_sucker, uint8_t Qi_ground, uint8_t mes3, uint8_t mes4,
							  uint8_t mes5, uint8_t mes6, uint8_t mes7, uint8_t mes8);
void SET_CAN1Ahead_MOTOR_CURRENT(CAN_HandleTypeDef *hcan, int16_t K1, int16_t K2, int16_t K3, int16_t K4); // 给电机值
void SET_CAN1Back_MOTOR_CURRENT(CAN_HandleTypeDef *hcan, int16_t K1, int16_t K2, int16_t K3, int16_t K4);
void SET_CAN2Ahead_MOTOR_CURRENT(CAN_HandleTypeDef *hcan, int16_t K1, int16_t K2, int16_t K3, int16_t K4);
void SET_CAN2Back_MOTOR_CURRENT(CAN_HandleTypeDef *hcan, int16_t K1, int16_t K2, int16_t K3, int16_t K4);

// 给C板发送底盘信息
void CAN_cmd_TO_C_BOARD_SPEED(int16_t x_speed, int16_t y_speed, int16_t z_speed, uint8_t key,
							  uint8_t message1);
// 给c板发送GO电机的模式和速度
void CAN_cmd_TO_C_BOARD_GO(int16_t GO0_speed, int16_t GO1_speed, int16_t GO2_speed, uint8_t mode0, uint8_t mode1);
void CAN_cmd_TO_C_BOARD_END(int16_t roll_speed, int16_t pitch_speed, int16_t yaw_sucker_speed, uint8_t mode0, uint8_t mode1);
