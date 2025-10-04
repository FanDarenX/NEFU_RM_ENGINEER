#include "Video_downlink.h"

/**
 * @author  SkrW、RM
 * @brief   新图传串口初始化
 *
 * @details 1. 使用USART6，即c板上3pin的串口
 *          2. 新图传的串口波特率为921600，注意修改，以及确认DMA接收通道是否正确
 */
#include "main.h"
#include "string.h"
#include "cmsis_os.h"
#include "math.h"

extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;

/**
 * @author  SkrW、RM
 * @brief   新图传遥控器解码
 *
 * @details DMA双缓冲接收图传发来的数据，进行CRC验证后进行解码，存放在结构体变量 rc_ctrl_new 中
 *          注意：
 *          1. 新图传的串口波特率为921600，注意修改，以及确认DMA接收通道是否正确
 *          2. 注意把_it文件中的串口中断处理函数给注释掉，防止重定义
 *          3. dt7掉线后，dt7_online_flag会小于0，此时鼠标和键盘数据由新图传链路传输，并将其赋值回老遥控器的结构体变量
 *          4. 已预留使用4pin串口连接新图传情况的相关函数，鉴于封装时队内所使用的统一项目疑似未启用usart1，为防止报错，已将其注释。需要使用时直接解注释即可
 */

#include "main.h"
#include <stdio.h>
#include <stdbool.h>

// 串口变量导入
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

// extern UART_HandleTypeDef huart1;
// extern DMA_HandleTypeDef hdma_usart1_rx;

// 链路切换
extern int dt7_online_flag;

// 缓存数组
uint8_t buff_1[NEW_RC_RX_BUF_NUM];
uint8_t buff_2[NEW_RC_RX_BUF_NUM];

custom_measure Custom_data[7] = {0};

// 结构体
remote_data_t rc_ctrl_new;

/*自定义控制器数据接收阈值*/
int Custom_rx_threshold = 0;

// 为每个通道创建滤波器实例
filter_t angle_filters[4] = {0};

// CRC16查表
static uint16_t crc16_init = 0xffff;
static const uint16_t crc16_tab[256] =
    {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/**
 * @brief Get the crc16 checksum
 * @author RM
 * @param p_msg Data to check
 * @param lenData length
 * @param crc16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if (p_msg == NULL)
    {
        return 0xffff;
    }

    while (len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}

/**
 * @brief crc16 verify function
 * @author RM
 * @param p_msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected = 0;

    if ((p_msg == NULL) || (len <= 2))
    {
        return false;
    }
    w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}

/**
 * @brief 串口中断处理函数，双缓冲区接收
 * @author RM、Skr
 * @note  注意把_it文件里的串口中断处理函数给注释掉，防止重定义
 */
void UART8_IRQHandler(void)
{
    if (huart8.Instance->SR & UART_FLAG_RXNE) // 接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart8);
    }
    else if (UART8->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart8);

        if ((hdma_uart8_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_uart8_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = NEW_RC_RX_BUF_NUM - hdma_uart8_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_uart8_rx.Instance->NDTR = NEW_RC_RX_BUF_NUM;

            // set memory buffer 1
            // 设定缓冲区1
            hdma_uart8_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_uart8_rx);

            if (this_time_rx_len == NEW_RC_FRAME_LENGTH) // 接收到一帧数据后解码
            {
                if (verify_crc16_check_sum(buff_1, NEW_RC_FRAME_LENGTH) == true)
                {
                    solve_new_rc_data(buff_1, &rc_ctrl_new);
                }
            }
            if (this_time_rx_len == Custom_LENGTH)
            {
                if (verify_crc16_check_sum(buff_1, Custom_LENGTH) == true)
                {
                    solve_custom_data_plus(buff_1, Custom_data);
                    Custom_rx_threshold = Custom_RX;
                }
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_uart8_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = NEW_RC_RX_BUF_NUM - hdma_uart8_rx.Instance->NDTR; // 不定长数据接收

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_uart8_rx.Instance->NDTR = NEW_RC_RX_BUF_NUM;

            // set memory buffer 0
            // 设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_uart8_rx);

            if (this_time_rx_len == NEW_RC_FRAME_LENGTH)
            {
                // 处理遥控器数据
                if (verify_crc16_check_sum(buff_2, NEW_RC_FRAME_LENGTH) == true)
                {
                    solve_new_rc_data(buff_2, &rc_ctrl_new);
                }
            }
            if (this_time_rx_len == Custom_LENGTH)
            {
                if (verify_crc16_check_sum(buff_2, Custom_LENGTH) == true)
                {
                    solve_custom_data_plus(buff_2, Custom_data);
                    Custom_rx_threshold = Custom_RX;
                }
            }
        }
    }
}

/**
 * @brief 遥控器数据解码
 * @author Skr
 * @param *sbus_buf 接收到的数据数组的指针
 * @param *rc_ctrl  遥控器数据结构体
 */
static void solve_new_rc_data(volatile const uint8_t *sbus_buf, remote_data_t *ctrl)
{
    if (sbus_buf == NULL || ctrl == NULL)
    {
        return;
    }

    // 摇杆
    ctrl->ch_0 = (sbus_buf[2] | (sbus_buf[3] << 8)) & 0x7FF;
    ctrl->ch_1 = ((sbus_buf[3] >> 3) | (sbus_buf[4] << 5)) & 0x7FF;
    ctrl->ch_2 = ((sbus_buf[4] >> 6) | (sbus_buf[5] << 2) | (sbus_buf[6]) << 10) & 0x7FF;
    ctrl->ch_3 = ((sbus_buf[6] >> 1) | (sbus_buf[7] << 7)) & 0x7FF;
    // 摇杆算数
    ctrl->ch_0 = ctrl->ch_0 - 1024;
    ctrl->ch_1 = ctrl->ch_1 - 1024;
    ctrl->ch_2 = ctrl->ch_2 - 1024;
    ctrl->ch_3 = ctrl->ch_3 - 1024;

    // 挡位切换开关CNS
    ctrl->mode_sw = (sbus_buf[7] >> 4) & 0x03;

    // 暂停按键
    ctrl->pause = (sbus_buf[7] >> 6) & 0x01;

    // 自定义按键
    ctrl->fn_1 = (sbus_buf[7] >> 7) & 0x01;
    ctrl->fn_2 = (sbus_buf[8]) & 0x01;

    // 拨轮
    ctrl->wheel = ((sbus_buf[8] >> 1) | (sbus_buf[9] << 7)) & 0x7FF;
    // 拨轮算数
    ctrl->wheel = ctrl->wheel - 1024;

    // 扳机键
    ctrl->trigger = (sbus_buf[9] >> 4) & 0x01;

    // 鼠标移动以及鼠标滚轮滚动
    ctrl->mouse_x = (sbus_buf[10] | (sbus_buf[11] << 8)) & 0xFFFF;
    ctrl->mouse_y = (sbus_buf[12] | (sbus_buf[13] << 8)) & 0xFFFF;
    ctrl->mouse_z = (sbus_buf[14] | (sbus_buf[15] << 8)) & 0xFFFF;

    // 鼠标按键
    ctrl->mouse_left = (sbus_buf[16]) & 0x03;
    ctrl->mouse_right = (sbus_buf[16] >> 2) & 0x03;
    ctrl->mouse_middle = (sbus_buf[16] >> 4) & 0x03;

    // 键盘按键
    ctrl->key = (sbus_buf[17] | sbus_buf[18] << 8);

    //    // 链路切换
    if (sbus_online_flag < 0)
    {
        rc_ctrl.mouse.press_l = ctrl->mouse_left;
        rc_ctrl.mouse.press_r = ctrl->mouse_right;
        rc_ctrl.key.v = ctrl->key;
        rc_ctrl.mouse.x = ctrl->mouse_x;
        rc_ctrl.mouse.y = ctrl->mouse_y;
        rc_ctrl.mouse.z = ctrl->mouse_z;
    }
}

// 异常值检测函数
static inline bool is_outlier(uint16_t new_value, uint16_t last_valid)
{
    return abs((int16_t)(new_value - last_valid)) > MAX_ALLOWED_CHANGE;
}

/*
自定义控制器数据接收，无滤波
*/
void solve_custom_data(volatile const uint8_t *sbus_buf, custom_measure *Custom_data)
{
    if (sbus_buf[0] == 0xA5 && sbus_buf[6] == 0x03 && sbus_buf[5] == 0x02)
    {
        for (int i = 0; i < 4; i++) // TODO改成单独试试
        {
            Custom_data[i].angle = (uint16_t)(sbus_buf[7 + i * 2] | sbus_buf[8 + i * 2] << 8);
        }
    }
}

void solve_custom_data_plus(volatile const uint8_t *sbus_buf, custom_measure *Custom_data)
{
    if (sbus_buf[0] == 0xA5 && sbus_buf[6] == 0x03 && sbus_buf[5] == 0x02)
    {
        for (int i = 0; i < 4; i++)
        {
            // 获取原始数据
            uint16_t raw_angle = (uint16_t)(sbus_buf[7 + i * 2] | sbus_buf[8 + i * 2] << 8);
            filter_t *filter = &angle_filters[i];

            // 异常值检测
            if (is_outlier(raw_angle, filter->last_valid) && filter->last_valid != 0)
            {
                // 如果是异常值，用最后一个有效值代替
                raw_angle = filter->last_valid;
            }
            else
            {
                // 更新最后一个有效值
                filter->last_valid = raw_angle;
            }

            // 更新滤波器
            filter->sum -= filter->buffer[filter->index]; // 减去旧值
            filter->buffer[filter->index] = raw_angle;    // 存储新值
            filter->sum += raw_angle;                     // 加上新值
            filter->index = (filter->index + 1) % FILTER_WINDOW_SIZE;

            // 计算平均值并存储
            Custom_data[i].angle = filter->sum / FILTER_WINDOW_SIZE;
        }
    }
}

void Custom_state(void)
{
    if (Custom_rx_threshold > 0)
    {
        Custom_rx_threshold = Custom_rx_threshold - 1;
    }
}
