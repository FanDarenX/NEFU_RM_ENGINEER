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
#define get_9025_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[7] << 8 | (data)[6]);            \
        (ptr)->speed_rpm = (int16_t)((data)[5] << 8 | (data)[4]);      \
        (ptr)->given_current = (int16_t)((data)[3] << 8 | (data)[2]);  \
        (ptr)->temperate = (data)[1];                                   \
    }
#define get_6020_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
#define get_HT_motor_measure(ptr, data)                                    \
    {                        \
	      (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd =       -uint_to_float((uint16_t)((data)[1] << 8 | (data)[2] ),P_MIN, P_MAX, 16);      \
        (ptr)->speed_rpm = -uint_to_float((uint16_t)(data[3]<<4)|(data[4]>>4), V_MIN, V_MAX, 12)/6;\
    }
#define get_gimbal_measure(ptr, data)                                    \
	{                                                                   \
		(ptr)->X_speed        =   (fp32)(((int8_t )(data)[0] << 8 | (data)[1])*0.0045f);            \
		(ptr)->Y_speed        = 	(fp32)(((int8_t )(data)[2] << 8 | (data)[3])*0.001f);      \
		(ptr)->A1_motor_reset = 	(data)[3]; \
		(ptr)->mode_R         = 	(data)[4]&3;  \
		(ptr)->sport_flag     = 	((data)[4]&12); \
		(ptr)->tl_flag        = 	(data)[4]&16 ; \
		(ptr)->side_flag      = 	(data)[4]&96 ; \
		(ptr)->protect_flag   = 	(data)[4]&128 ; \
		(ptr)->power_flag     = 	(uint16_t)((data)[5] << 8 | (data)[6])/100;    \
		(ptr)->energy_buff    = 	(uint16_t)( (data)[5] << 8 | (data)[6] ) - (ptr)->power_flag;    \
		(ptr)->sit_flag       = 	(data)[7]&3;    \
		(ptr)->stand_flag     = 	(data)[7]&12;    \
		(ptr)->reborn_flag    = 	(data)[7]&48 ; \
		(ptr)->jump_flag      = 	(data)[7]&192 ; \
	}
#define get_cap_measure(ptr, data)                                    \
    {                                                                   \
			(ptr)->Value_Bat     = (fp32)(((data)[0] << 8 | (data)[1])*0.001f);            \
			(ptr)->Value_Cap     = (fp32)(((data)[2] << 8 | (data)[3])*0.001f);      \
			(ptr)->Power_Charge  = (fp32)(((data)[4] << 8 | (data)[5])*0.002f);  \
			(ptr)->Power_Chassis = (fp32)(((data)[6] << 8 | (data)[7])*0.002f);   \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
motor_measure_t motor_gimbal[1];
HT_motor_measure_t motor_HT[4];
motor_measure_t motor_9025[2];
cap_measure_t   super_cap;
Gimbal_ctrl_t Gimbal_ctrl;
CAN_TxHeaderTypeDef  chassis_tx_message;

uint16_t reset_temp = 0;
static uint8_t              chassis_can_send_data[8];

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
static float uint_to_float(int x_int, float x_min, float x_max, int bits);
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);

static void  CanTransmit(uint8_t *buf, uint8_t len,uint8_t id);
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
uint8_t can_temp[8];
volatile float Position,Velocity= 0;
CAN_RxHeaderTypeDef rx_header;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    uint8_t rx_data[8];
    uint16_t tmp_value;

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if(hcan==&hcan2)
    {
        switch (rx_header.StdId)
        {
							case CAN_YAW_MOTOR_ID:
							{
									get_6020_motor_measure(&motor_gimbal[0], rx_data);
									decet_flag.motor_count[6]++;
									break;
							}
							case CAN_GIMBAL_ID:
							{
									get_gimbal_measure(&Gimbal_ctrl, rx_data);
									decet_flag.motor_count[7]++;
									break;
							}
							case CAN_SUPERCAP_ID:
							{
									get_cap_measure(&super_cap, rx_data);
									decet_flag.motor_count[8]++;
									break;
							}
							default:
							{
							}
        }
    }

    else if(hcan==&hcan1)
    {
        switch (rx_header.StdId)
        {
							case CAN_9025_M1_ID:
							{
									get_9025_motor_measure(&motor_9025[0], rx_data);
									decet_flag.motor_count[4]++;
									break;
							}
							case CAN_9025_M2_ID:
							{
									get_9025_motor_measure(&motor_9025[1], rx_data);
									decet_flag.motor_count[5]++;
									break;
							}
							default:
							{
									if(rx_data[0] == HT_SLAVE_ID1)
									{
											get_HT_motor_measure(&motor_HT[3], rx_data);
											decet_flag.motor_count[5]++;
									}
									else if(rx_data[0] == HT_SLAVE_ID3)
									{
											get_HT_motor_measure(&motor_HT[2], rx_data);
											decet_flag.motor_count[4]++;
									}
							}
        }
    }
}

void CAN_cmd_cap_flag(int8_t x_flag,int8_t y_flag,int8_t tl_flag,int8_t sp_flag,int8_t mode,uint8_t buffer_energy,uint16_t power_limit)
{
    uint32_t send_mail_box;
    cap_tx_message.StdId = CAN_SEND_TO_CAP_ID;
    cap_tx_message.IDE =   CAN_ID_STD;
    cap_tx_message.RTR =   CAN_RTR_DATA;
    cap_tx_message.DLC =   0x08;
    cap_can_send_data[0] = x_flag;
    cap_can_send_data[1] = y_flag;
    cap_can_send_data[2] = tl_flag;
    cap_can_send_data[3] = sp_flag;
    cap_can_send_data[4] = mode;
    cap_can_send_data[5] = buffer_energy;
    cap_can_send_data[6] =(power_limit>>8);
    cap_can_send_data[7] = power_limit;

    HAL_CAN_AddTxMessage(&hcan2, &cap_tx_message, cap_can_send_data, &send_mail_box);
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

    cap_INS_tx_message.StdId = CAN_INS_TO_CAP_ID;
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
  * @brief  Can总线发送控制参数
  * @param
  * @retval
  */
void CAN_cmd_chassis_read(uint8_t id,int16_t current)
{
    // CAN_TxHeaderTypeDef chassis_tx_message;             /**!< can通信发送协议头 */
    uint32_t canTxMailbox= CAN_TX_MAILBOX0;

    chassis_tx_message.StdId = 0x140+id;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0xA1;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = current;
    chassis_can_send_data[5] = (current>>8);
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    //找到空的发送邮箱 把数据发送出去
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);	// 如果三个发送邮箱都阻塞了就等待直到其中某个邮箱空闲
    if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET)
    {   // 检查发送邮箱0状态 如果邮箱0空闲就将待发送数据放入FIFO0
        canTxMailbox = CAN_TX_MAILBOX0;
    }
    else if ((hcan1.Instance->TSR & CAN_TSR_TME1) != RESET)
    {
        canTxMailbox = CAN_TX_MAILBOX1;
    }
    else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET)
    {
        canTxMailbox = CAN_TX_MAILBOX2;
    }


    if(HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, (uint32_t *)&canTxMailbox)==HAL_OK) {};
}
void CAN_cmd_9025(int16_t current1,int16_t current2)
{
    CAN_TxHeaderTypeDef TxHead;             /**!< can通信发送协议头 */
    uint32_t canTxMailbox;
    uint32_t TxMailboxX = CAN_TX_MAILBOX0;				// CAN发送邮箱

    chassis_tx_message.StdId = 0x280;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = current1;
    chassis_can_send_data[1] = (current1>>8);
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = current2;
    chassis_can_send_data[5] = (current2>>8);
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    //找到空的发送邮箱 把数据发送出去
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);	// 如果三个发送邮箱都阻塞了就等待直到其中某个邮箱空闲
    if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET)
    {   // 检查发送邮箱0状态 如果邮箱0空闲就将待发送数据放入FIFO0
        TxMailboxX = CAN_TX_MAILBOX0;
    }
    else if ((hcan1.Instance->TSR & CAN_TSR_TME1) != RESET)
    {
        TxMailboxX = CAN_TX_MAILBOX1;
    }
    else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET)
    {
        TxMailboxX = CAN_TX_MAILBOX2;
    }

    if(HAL_CAN_AddTxMessage(&hcan1, &TxHead, chassis_can_send_data, (uint32_t *)&canTxMailbox)==HAL_OK) {};
}

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint8_t id)
{
    uint16_t p, v, kp, kd, t;
    uint8_t buf[8];
    f_t=-f_t;
    /* 限制输入的参数在定义的范围内 */
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);

    /* 根据协议，对float参数进行转换 */
    p = float_to_uint(f_p,      P_MIN,  P_MAX,  16);
    v = float_to_uint(f_v,      V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp,    KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd,    KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,      T_MIN,  T_MAX,  12);

    /* 根据传输协议，把数据转换为CAN命令数据字段 */
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;
    /* 通过CAN接口把buf中的内容发送出去 */
    CanTransmit(buf, sizeof(buf),id);
}
/* 把buf中的内容通过CAN接口发送出去 */
static void CanTransmit(uint8_t *buf, uint8_t len,uint8_t id)
{
    CAN_TxHeaderTypeDef TxHead;             /**!< can通信发送协议头 */
    uint32_t canTxMailbox;
    uint32_t TxMailboxX = CAN_TX_MAILBOX1;				// CAN发送邮箱

    if((buf != NULL) && (len != 0))
    {
        if(id==0 || id==3)TxHead.StdId    = HT_SLAVE_ID1;     /* 指定标准标识符，该值在0x00-0x7FF */
        else if (id==1 || id==2)TxHead.StdId    = HT_SLAVE_ID3;     /* 指定标准标识符，该值在0x00-0x7FF */
        TxHead.IDE      = CAN_ID_STD;       /* 指定将要传输消息的标识符类型 */
        TxHead.RTR      = CAN_RTR_DATA;     /* 指定消息传输帧类型 */
        TxHead.DLC      = len;              /* 指定将要传输的帧长度 */


        if(id==0||id==1)
        {
            //找到空的发送邮箱 把数据发送出去
            while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);	// 如果三个发送邮箱都阻塞了就等待直到其中某个邮箱空闲
            if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET)
            {   // 检查发送邮箱0状态 如果邮箱0空闲就将待发送数据放入FIFO0
                TxMailboxX = CAN_TX_MAILBOX0;
            }
            if ((hcan2.Instance->TSR & CAN_TSR_TME1) != RESET)
            {
                TxMailboxX = CAN_TX_MAILBOX1;
            }
            else if ((hcan2.Instance->TSR & CAN_TSR_TME2) != RESET)
            {
                TxMailboxX = CAN_TX_MAILBOX2;
            }
            if(HAL_CAN_AddTxMessage(&hcan2, &TxHead, buf, (uint32_t *)&canTxMailbox)==HAL_OK) {};
        }
        else if(id==2||id==3)
        {
            //找到空的发送邮箱 把数据发送出去
            while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);	// 如果三个发送邮箱都阻塞了就等待直到其中某个邮箱空闲
            if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET)
            {   // 检查发送邮箱0状态 如果邮箱0空闲就将待发送数据放入FIFO0
                TxMailboxX = CAN_TX_MAILBOX0;
            }
            else if ((hcan1.Instance->TSR & CAN_TSR_TME1) != RESET)
            {
                TxMailboxX = CAN_TX_MAILBOX1;
            }
            else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET)
            {
                TxMailboxX = CAN_TX_MAILBOX2;
            }
            if(HAL_CAN_AddTxMessage(&hcan1, &TxHead, buf, (uint32_t *)&canTxMailbox)==HAL_OK) {};
        }
    }
}


void CanComm_ControlCmd(uint8_t cmd,uint8_t id)
{
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    switch(cmd)
    {
    case CMD_MOTOR_MODE:
        buf[7] = 0xFC;
        break;

    case CMD_RESET_MODE:
        buf[7] = 0xFD;
        break;

    case CMD_ZERO_POSITION:
        buf[7] = 0xFE;
        break;

    default:
        return; /* 直接退出函数 */
    }
    CanTransmit(buf, sizeof(buf),id);
}


const motor_measure_t *get_6020motor_measure_point(uint8_t i)
{
    return &motor_gimbal[i];
}
const HT_motor_measure_t *get_HT_motor_measure_point(uint8_t i)
{
    return &motor_HT[i];
}
const motor_measure_t *get_9025motor_measure_point(uint8_t i)
{
    return &motor_9025[i];
}
/**
  * @brief          返回超级电容数据指针
  * @param[in]
  * @retval
  */
const cap_measure_t *get_cap_measure_point()
{
    return &super_cap;
}
const Gimbal_ctrl_t *get_Gimabl_control_point(void)
{
    return &Gimbal_ctrl;
}
const cap_measure_t *get_SuperCap_control_point(void)
{
    return &super_cap;
}
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}