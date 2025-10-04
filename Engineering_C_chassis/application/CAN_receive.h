/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "chassis_task.h"
#include "XY.h"

#define A_TO_C_CAN hcan1
#define CHASSIS_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
  /*CAN1*/
  CAN2_A_TO_CAN1_C_Chassis = 0x101,
  CAN2_A_TO_CAN1_C_XY = 0x102,
  /*CAN1*/

  /*CAN2*/
  CAN_3508M1_ID = 0x201,
  CAN_3508M2_ID = 0x202,
  CAN_3508M3_ID = 0x203,
  CAN_3508M4_ID = 0x204,

  /*���żܺ���x��*/
  CAN_2006LX_ID = 0x206, // ��
  /*���ż�����y��*/
  CAN_2006LY_ID = 0x205, // ��
  /*CAN2*/

} can_msg_id_e;

// rm motor data
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
} motor_measure_t;

/*CAN����ֵ�ṹ��*/
typedef struct
{
  uint16_t angle; // ת�ӽǶ� (0~8191)
  int16_t speed;  // ת���ٶ�
  int16_t last_speed;
  int16_t real_current; // �������
  int16_t temperature;  // ����¶�
  uint16_t last_angle;
  uint16_t offset_angle; // �����Ƕ�
  int32_t round_cnt;     // ת��ת��Ȧ��
  int32_t total_angle;   // ת��ת���ܽǶ�
  int32_t last_total_angle;
  int32_t angle_err;
  int32_t angle_ERR;
  int32_t total_angle_true;
} moto_measure;

extern moto_measure chassis_moto[4]; // ���̵��
/*���żܺ���x���*/
extern moto_measure X_moto;
/*���ż�����y���*/
extern moto_measure Y_moto;
// extern int flag;

extern void CHASSIS_CURRENT(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4); // 3508������Ƶ���, ��Χ [-16384,16384]
extern void XY_CURRENT(int16_t K1, int16_t K2);

extern void CAN_cmd_chassis_reset_ID(void); // ����IDΪ0x700��CAN��,��������3508��������������ID

#endif
