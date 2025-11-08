/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
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

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "can.h"
#include "main.h"
#include "decet_task.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
//
#define ABS(x) (((x) > 0) ? (x) : (-(x)))
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
#define get_cap_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->Value_Bat     = (fp32)(((data)[0] << 8 | (data)[1])*0.001f);            \
        (ptr)->Value_Cap     = (fp32)(((data)[2] << 8 | (data)[3])*0.001f);      \
        (ptr)->Power_Charge  = (fp32)(((data)[4] << 8 | (data)[5])*0.002f);  \
        (ptr)->Power_Chassis = (fp32)(((data)[6] << 8 | (data)[7])*0.002f);                                   \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
motor_measure_t motor_chassis[4];
motor_measure_t motor_gimbal[6];
cap_measure_t   super_cap;
		
static CAN_TxHeaderTypeDef  pitch_tx_message;
static uint8_t              pitch_can_send_data[8];
		
static CAN_TxHeaderTypeDef  yaw_tx_message;
static uint8_t              yaw_can_send_data[8];		

static CAN_TxHeaderTypeDef  cannon_tx_message;
static uint8_t              cannon_can_send_data[8];
static CAN_TxHeaderTypeDef  barrel_tx_message;
static uint8_t              barrel_can_send_data[8];		
		
static CAN_TxHeaderTypeDef  cap_tx_message;
static uint8_t              cap_can_send_data[8];

static CAN_TxHeaderTypeDef  cap_INS_tx_message;
 uint8_t              cap_INS_can_send_data[8];
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	if(hcan==&hcan1)
	{
    switch (rx_header.StdId)
    {
				case CAN_TRIGGER_MOTOR_ID:{get_motor_measure(&motor_gimbal[2], rx_data);decet_flag.motor_count[2]++;break;}
				case CAN_FIRE1_MOTOR_ID  :{get_motor_measure(&motor_gimbal[0], rx_data);decet_flag.motor_count[0]++;break;}
				case CAN_FIRE2_MOTOR_ID  :{get_motor_measure(&motor_gimbal[1], rx_data);decet_flag.motor_count[1]++;break;}
				case CAN_PIT_MOTOR_ID    :{get_motor_measure(&motor_gimbal[4], rx_data);decet_flag.motor_count[4]++;break;}
				case CAN_BARR_MOTOR_ID   :{get_motor_measure(&motor_gimbal[5], rx_data);decet_flag.motor_count[5]++;break;}

				default:{break;}
		}
	}
	if(hcan==&hcan2)
	{
		switch (rx_header.StdId)
    {
			case CAN_GET_CAP_ID:
			{
				get_cap_measure(&super_cap, rx_data);
				decet_flag.motor_count[9]++;
			  break;
			}
		  case CAN_YAW_MOTOR_ID:
		  {
					get_motor_measure(&motor_gimbal[3], rx_data);
					decet_flag.motor_count[8]++;
				  break;
      }
			default:
			{
					break;
			}
		}
	}
}

/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw:   (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev:   (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_yaw_gimbal(int16_t yaw)
{
    uint32_t send_mail_box;
    yaw_tx_message.StdId = 0x2FF;
    yaw_tx_message.IDE =   CAN_ID_STD;
    yaw_tx_message.RTR =   CAN_RTR_DATA;
    yaw_tx_message.DLC = 0x08;

	  yaw_can_send_data[0] = (yaw >> 8);
    yaw_can_send_data[1] = yaw;
    yaw_can_send_data[2] = (0 >> 8);
    yaw_can_send_data[3] = 0;
    yaw_can_send_data[4] = (0 >> 8);
    yaw_can_send_data[5] = 0;
    yaw_can_send_data[6] = (0 >> 8);
    yaw_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan2, &yaw_tx_message, yaw_can_send_data, &send_mail_box);
}
void CAN_cmd_pitch_gimbal(int16_t pitch,int16_t barrel)
{
    uint32_t send_mail_box;
    pitch_tx_message.StdId = 0x1FF;
    pitch_tx_message.IDE =   CAN_ID_STD;
    pitch_tx_message.RTR =   CAN_RTR_DATA;
    pitch_tx_message.DLC = 0x08;
	  pitch_can_send_data[0] = (0 >> 8);
    pitch_can_send_data[1] = 0;
    pitch_can_send_data[2] = (pitch >> 8);
    pitch_can_send_data[3] = pitch;
 		pitch_can_send_data[4] = (barrel >> 8);
	  pitch_can_send_data[5] = barrel;
    pitch_can_send_data[6] = (0 >> 8);
    pitch_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &pitch_tx_message, pitch_can_send_data, &send_mail_box);
}
	
/**
  * @brief     
  * @param[in]      主炮控制电流, 范围 [-30000,30000]
  */
void CAN_cmd_cannon(int16_t fire1, int16_t fire2 ,int16_t shoot, int16_t raw)
{
    uint32_t send_mail_box;
    cannon_tx_message.StdId = 0x200;// CAN_GIMBAL_ALL_ID;
    cannon_tx_message.IDE =   CAN_ID_STD;
    cannon_tx_message.RTR =   CAN_RTR_DATA;
    cannon_tx_message.DLC = 0x08;
		cannon_can_send_data[0] = (fire1 >> 8);
	  cannon_can_send_data[1] = fire1;
		cannon_can_send_data[2] = (fire2 >> 8);
	  cannon_can_send_data[3] = fire2;
    cannon_can_send_data[4] = (shoot >> 8);
	  cannon_can_send_data[5] = shoot;
		cannon_can_send_data[6] = 0;
	  cannon_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CANNON_CAN, &cannon_tx_message, cannon_can_send_data, &send_mail_box);
}
void CAN_cmd_barrel(int16_t barrel)
{
    uint32_t send_mail_box;
    barrel_tx_message.StdId = 0x1FF;// CAN_GIMBAL_ALL_ID;
    barrel_tx_message.IDE =   CAN_ID_STD;
    barrel_tx_message.RTR =   CAN_RTR_DATA;
    barrel_tx_message.DLC = 0x08;
		barrel_can_send_data[4] = (barrel >> 8);
	  barrel_can_send_data[5] = barrel;
		barrel_can_send_data[2] = 0;
	  barrel_can_send_data[3] = 0;
    barrel_can_send_data[0] = 0;
	  barrel_can_send_data[1] = 0;
		barrel_can_send_data[6] = 0;
	  barrel_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &barrel_tx_message, barrel_can_send_data, &send_mail_box);
}

void CAN_cmd_cap_flag(int16_t X_speed, //速度
	int8_t Y_speed, //速度
	int8_t remake_flag,
	uint8_t sit_flag,uint8_t stand_flag,    							//机体高度 & 跳跃动作
	int8_t mode_R,                       									//遥控器
	int8_t sport_flag,                   									//运动模式         
	int8_t tl_flag,uint8_t side_flag,uint8_t jump_flag,   //战术动作
	uint16_t power_flag,uint8_t reborn_flag,uint8_t protect_flag)
{
	/*
	fp32 X_speed;//基础速度2m/s，运动模式下3m/s+
	fp32 High;//当前腿长，基础0.18、蹲下0.11、起立0.27
	uint8_t common_flag;//蹲下，小板凳模式
	uint8_t stand_flag;//起身，中门对狙
	uint8_t mode_R;
	uint8_t tl_flag;//侧对敌，你觉得你能-杀-死-我！
	uint8_t side_flag;//侧对敌，叹息之墙
	uint8_t power_flag;//功率上限
	uint16_t energy_buff;//缓冲功率*/
    uint32_t send_mail_box;
	  uint8_t temp4;
	  uint8_t temp7;
    cap_tx_message.StdId = CAN_SEND_TO_CAP_ID;
    cap_tx_message.IDE =   CAN_ID_STD;
    cap_tx_message.RTR =   CAN_RTR_DATA;
    cap_tx_message.DLC =   0x08;
    cap_can_send_data[0] = (X_speed>> 8);
    cap_can_send_data[1] =  X_speed;
    cap_can_send_data[2] =  Y_speed;
    cap_can_send_data[3] =  remake_flag;
	  temp4 = mode_R | sport_flag<<2 | tl_flag<<4 | side_flag<<6 | protect_flag<<7;
    cap_can_send_data[4] = temp4;
		cap_can_send_data[5] = power_flag>>8;
		cap_can_send_data[6] = power_flag;
		temp7 = sit_flag | stand_flag <<2 | reborn_flag<<4 | jump_flag<<6;
		cap_can_send_data[7] = temp7;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &cap_tx_message, cap_can_send_data, &send_mail_box);
}
void CAN_cmd_cap_angle(fp32 INS_pitch,fp32 INS_roll,fp32 INS_yaw,  fp32 motor_pitch,fp32 motor_yaw)
{
    uint32_t send_mail_box;
    fp32 yaw =   INS_yaw   - motor_yaw;
    fp32 pitch = INS_pitch - motor_pitch;
	  fp32 roll =  INS_roll;
	  int16_t yaw_temp=    (int16_t)(yaw*10000.0f);
		int16_t pitch_temp=  (int16_t)(pitch*10000.0f);
		int16_t roll_temp=   (int16_t)(roll*10000.0f);
	
    //cap_INS_tx_message.StdId = CAN_INS_TO_CAP_ID;
    cap_INS_tx_message.IDE = CAN_ID_STD;
    cap_INS_tx_message.RTR = CAN_RTR_DATA;
    cap_INS_tx_message.DLC = 0x08;
    cap_INS_can_send_data[0] = (pitch_temp>>8);
    cap_INS_can_send_data[1] = pitch_temp;
    cap_INS_can_send_data[2] = (roll_temp>>8);
    cap_INS_can_send_data[3] = roll_temp;
    cap_INS_can_send_data[4] = (yaw_temp>>8);
    cap_INS_can_send_data[5] = yaw_temp;
    cap_INS_can_send_data[6] =0;
    cap_INS_can_send_data[7] =0;
    
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &cap_INS_tx_message, cap_INS_can_send_data, &send_mail_box);
}

/**
  * @brief          返回6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_6020motor_measure_point(uint8_t i)
{
    return &motor_gimbal[i];
}
/**
  * @brief          返回2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_2006motor_measure_point(uint8_t i)
{
    return &motor_chassis[i];
}
/**
  * @brief          返回3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_3508motor_measure_point(uint8_t i)
{
    return &motor_chassis[i];
}
/**
  * @brief          返回超级电容数据指针
  * @param[in]      
  * @retval         
  */
//const cap_measure_t *get_cap_measure_point()
//{
//    return &super_cap;
//}
