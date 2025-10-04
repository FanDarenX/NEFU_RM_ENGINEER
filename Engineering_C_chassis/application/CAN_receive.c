#include "CAN_receive.h"
#include "main.h"
#include "math.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

int flag = 0;

static motor_measure_t motor_chassis[7];

static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

/********************************************************* */
/*底盘四电机*/
moto_measure chassis_moto[4] = {0}; // 底盘电机
/*龙门架横向x电机*/
moto_measure X_moto = {0};
/*龙门架纵向y电机*/
moto_measure Y_moto = {0};

CAN_TxHeaderTypeDef TX_message; // ALL
uint8_t TXdata[8];

void get_moto_measure(moto_measure *ptr, uint8_t RXdata[8])
{
  ptr->last_angle = ptr->angle;
  ptr->last_speed = ptr->speed;
  ptr->last_total_angle = ptr->total_angle;

  ptr->angle = (RXdata[0] << 8) | RXdata[1]; // 电机反馈
  ptr->speed = (RXdata[2] << 8) | RXdata[3];
  ptr->real_current = (RXdata[4] << 8) | RXdata[5];
  ptr->temperature = RXdata[6];

  if (ptr->angle - ptr->last_angle > 4096)
  {
    ptr->angle_ERR = ptr->angle - ptr->last_angle;
    ptr->round_cnt -= 1;
  }
  else if (ptr->angle - ptr->last_angle < -4096)
  {
    ptr->angle_ERR = ptr->angle - ptr->last_angle;
    ptr->round_cnt += 1;
  }
  ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle;
  ptr->angle_err = ptr->last_total_angle - ptr->total_angle;
  ptr->total_angle_true = ptr->total_angle * 6.28 / 8192;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  //  flag = 0;
  CAN_RxHeaderTypeDef RX1header;
  CAN_RxHeaderTypeDef RX2header;
  uint8_t RX1data[8];
  uint8_t RX2data[8];

  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RX1header, RX1data) == HAL_OK)
  {
    if (hcan->Instance == CAN1)
    {
      switch (RX1header.StdId)
      {
      case CAN2_A_TO_CAN1_C_Chassis:
      {
        chassis_speed_task.chassis_speed.chassis_speed_x = (int16_t)(RX1data[0] << 8 | RX1data[1]);
        chassis_speed_task.chassis_speed.chassis_speed_y = (int16_t)(RX1data[2] << 8 | RX1data[3]);
        chassis_speed_task.chassis_speed.chassis_speed_r = (int16_t)(RX1data[4] << 8 | RX1data[5]);
        //		flag = 1;
      }
      break;
        //   case CAN2_A_TO_CAN1_C_XY:
        //   {
        //     // 将 RX2data 中的字节复制到 float 变量中
        // memcpy(&X_target, &RX2data[0], 4); // 复制前 4 个字节到 X_target
        // memcpy(&Y_target, &RX2data[4], 4); // 复制后 4 个字节到 Y_target
        //     // XY_CURRENT(X_target, Y_target);
        //   }
        //   break;
      default:
      {
        break;
      }
      }
    }
  }

  if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RX2header, RX2data) == HAL_OK)
  {
    if (hcan->Instance == CAN2)
    {
      switch (RX2header.StdId)
      {
      /*这是底盘四个电机*/
      case CAN_3508M1_ID:
      case CAN_3508M2_ID:
      case CAN_3508M3_ID:
      case CAN_3508M4_ID:
      {
        static int j;
        j = RX2header.StdId - CAN_3508M1_ID;
        get_moto_measure(&chassis_moto[j], RX2data);
        break;
      }
        /*这是龙门架纵向y轴电机*/
      case CAN_2006LY_ID:
        get_moto_measure(&Y_moto, RX2data);
        break;
      /*这是龙门架横向x轴电机*/
      case CAN_2006LX_ID:
        get_moto_measure(&X_moto, RX2data);
        break;

      // case CAN2_A_TO_CAN1_C_Chassis:
      // {
      //   chassis_speed_task.chassis_speed.chassis_speed_x = (int16_t)(RX2data[0] << 8 | RX2data[1]);
      //   chassis_speed_task.chassis_speed.chassis_speed_y = (int16_t)(RX2data[2] << 8 | RX2data[3]);
      //   chassis_speed_task.chassis_speed.chassis_speed_r = (int16_t)(RX2data[4] << 8 | RX2data[5]);
      //   //		flag = 1;
      // }
      // break;
      default:
      {
        break;
      }
      }
    }
  }
}

/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CHASSIS_CURRENT(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x200;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void XY_CURRENT(int16_t K1, int16_t K2) // CAN2
{
  TX_message.StdId = 0x1FF;
  TX_message.IDE = CAN_ID_STD;
  TX_message.RTR = CAN_RTR_DATA;
  TX_message.DLC = 4;
  TX_message.TransmitGlobalTime = DISABLE;

  TXdata[0] = K1 >> 8;
  TXdata[1] = K1;
  TXdata[2] = K2 >> 8;
  TXdata[3] = K2;
  // TXdata[4] = 0;
  // TXdata[5] = 0;
  // TXdata[6] = 0;
  // TXdata[7] = 0;
  HAL_CAN_AddTxMessage(&hcan2, &TX_message, TXdata, CAN_FILTER_FIFO0);
}

void CAN_cmd_chassis_reset_ID(void) // 发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x700;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = 0;
  chassis_can_send_data[3] = 0;
  chassis_can_send_data[4] = 0;
  chassis_can_send_data[5] = 0;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
