#ifndef VIDEO_DOWNLINK_H__
#define VIDEO_DOWNLINK_H__

// #include "custom.h"

#include "main.h"
#include "remote_control.h"
//#include "custom.h"

/**
 * @brief 串口初始化函数
 * @author RM、Skr
 * @param *rx1_buf、*rx2_buf 缓冲接收数组的指针
 * @param 缓冲数组的大小
 */
extern void uart8_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

// 宏定义
#define NEW_RC_RX_BUF_NUM 42u   // 双缓冲
#define NEW_RC_FRAME_LENGTH 21u // 一帧数据长度

#define Custom_LENGTH 39u

#define receive_size 39

// 结构体
typedef __packed struct
{
   uint8_t sof_1;
   uint8_t sof_2;
   uint64_t ch_0 : 11;
   uint64_t ch_1 : 11;
   uint64_t ch_2 : 11;
   uint64_t ch_3 : 11;
   uint8_t mode_sw : 2;
   uint8_t pause : 1;
   uint8_t fn_1 : 1;
   uint8_t fn_2 : 1;
   uint64_t wheel : 11;
   uint8_t trigger : 1;

   int16_t mouse_x;
   int16_t mouse_y;
   int16_t mouse_z;
   uint8_t mouse_left : 2;
   uint8_t mouse_right : 2;
   uint8_t mouse_middle : 2;
   uint16_t key;
   uint16_t crc16;
} remote_data_t;

/*
自定义控制器数据接收
添加了移动平均滤波
*/
#define FILTER_WINDOW_SIZE 4  // 滤波窗口大小，可根据需要调整
#define MAX_ALLOWED_CHANGE 300    // 允许的最大变化量（根据实际情况调整）

#define Custom_RX 999999


// 定义滤波数据结构
typedef struct {
    uint16_t buffer[FILTER_WINDOW_SIZE];
    uint8_t index;
    uint16_t sum;
    uint16_t last_valid;          // 存储最后一个有效值
} filter_t;


typedef struct
{
	uint16_t angle;
	uint16_t last_angle;
	int total_angle;
	int total_last_angle;
	int round_cnt;
	float total_angle_ture;

	/*弥补编码器硬件误差*/
	int Positive_to_negative; // 正变负临时用变量=当前总角度+补偿值（160）
	// int Negative_to_positive; // 负变正直接使用当前总角度,简化可以不需要

	/*临时变量*/
	int tmp_total_angle; // 用于储存正变负时的总角度
} custom_measure;


// 宏定义函数
// 新拨杆
#define RC_SW_C ((uint8_t)0)
#define RC_SW_N ((uint8_t)1)
#define RC_SW_S ((uint8_t)2)
// 鼠标按键
#define MOUSE_PRESS_L(s) ((s)->mouse_left == 1)
#define MOUSE_PRESS_R(s) ((s)->mouse_right == 1)
#define MOUSE_PRESS_M(s) ((s)->mouse_middle == 1)
// 新按键
#define FN_SET ((uint8_t)1)
#define FN_RESET ((uint8_t)0)

// 函数
/**
 * @brief Get the crc16 checksum
 * @author RM
 * @param p_msg Data to check
 * @param lenData length
 * @param crc16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
extern uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16);

/**
 * @brief crc16 verify function
 * @author RM
 * @param p_msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
extern void solve_dt7_data(volatile const uint8_t *sbus_buf, remote_data_t *rc_ctrl);

/**
 * @brief 串口中断处理函数，双缓冲区接收
 * @author RM、Skr
 * @note  注意把_it文件里的串口中断处理函数给注释掉，防止重定义
 */
extern void USART6_IRQHandler(void);

/**
 * @brief 遥控器数据解码
 * @author Skr
 * @param *sbus_buf 接收到的数据数组的指针
 * @param *rc_ctrl  遥控器数据结构体
 */
extern void solve_new_rc_data(volatile const uint8_t *sbus_buf, remote_data_t *rc_ctrl);

// 缓存数组
extern uint8_t buff_1[NEW_RC_RX_BUF_NUM];
extern uint8_t buff_2[NEW_RC_RX_BUF_NUM];

void solve_custom_data(volatile const uint8_t *sbus_buf, custom_measure *Custom_data);
void solve_custom_data_plus(volatile const uint8_t *sbus_buf, custom_measure *Custom_data);

void Custom_state(void);



//extern uint8_t receive_data[receive_size];
extern custom_measure Custom_data[7];
extern filter_t angle_filters[4];

extern int Custom_rx_threshold;

#endif