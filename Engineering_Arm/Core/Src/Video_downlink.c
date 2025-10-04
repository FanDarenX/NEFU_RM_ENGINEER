#include "Video_downlink.h"

/**
 * @author  SkrW��RM
 * @brief   ��ͼ�����ڳ�ʼ��
 *
 * @details 1. ʹ��USART6����c����3pin�Ĵ���
 *          2. ��ͼ���Ĵ��ڲ�����Ϊ921600��ע���޸ģ��Լ�ȷ��DMA����ͨ���Ƿ���ȷ
 */
#include "main.h"
#include "string.h"
#include "cmsis_os.h"
#include "math.h"

extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;

/**
 * @author  SkrW��RM
 * @brief   ��ͼ��ң��������
 *
 * @details DMA˫�������ͼ�����������ݣ�����CRC��֤����н��룬����ڽṹ����� rc_ctrl_new ��
 *          ע�⣺
 *          1. ��ͼ���Ĵ��ڲ�����Ϊ921600��ע���޸ģ��Լ�ȷ��DMA����ͨ���Ƿ���ȷ
 *          2. ע���_it�ļ��еĴ����жϴ�������ע�͵�����ֹ�ض���
 *          3. dt7���ߺ�dt7_online_flag��С��0����ʱ���ͼ�����������ͼ����·���䣬�����丳ֵ����ң�����Ľṹ�����
 *          4. ��Ԥ��ʹ��4pin����������ͼ���������غ��������ڷ�װʱ������ʹ�õ�ͳһ��Ŀ����δ����usart1��Ϊ��ֹ�����ѽ���ע�͡���Ҫʹ��ʱֱ�ӽ�ע�ͼ���
 */

#include "main.h"
#include <stdio.h>
#include <stdbool.h>

// ���ڱ�������
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

// extern UART_HandleTypeDef huart1;
// extern DMA_HandleTypeDef hdma_usart1_rx;

// ��·�л�
extern int dt7_online_flag;

// ��������
uint8_t buff_1[NEW_RC_RX_BUF_NUM];
uint8_t buff_2[NEW_RC_RX_BUF_NUM];

custom_measure Custom_data[7] = {0};

// �ṹ��
remote_data_t rc_ctrl_new;

/*�Զ�����������ݽ�����ֵ*/
int Custom_rx_threshold = 0;

// Ϊÿ��ͨ�������˲���ʵ��
filter_t angle_filters[4] = {0};

// CRC16���
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
 * @brief �����жϴ�������˫����������
 * @author RM��Skr
 * @note  ע���_it�ļ���Ĵ����жϴ�������ע�͵�����ֹ�ض���
 */
void UART8_IRQHandler(void)
{
    if (huart8.Instance->SR & UART_FLAG_RXNE) // ���յ�����
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
            // ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_uart8_rx);

            // get receive data length, length = set_data_length - remain_length
            // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = NEW_RC_RX_BUF_NUM - hdma_uart8_rx.Instance->NDTR;

            // reset set_data_lenght
            // �����趨���ݳ���
            hdma_uart8_rx.Instance->NDTR = NEW_RC_RX_BUF_NUM;

            // set memory buffer 1
            // �趨������1
            hdma_uart8_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_uart8_rx);

            if (this_time_rx_len == NEW_RC_FRAME_LENGTH) // ���յ�һ֡���ݺ����
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
            // ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_uart8_rx);

            // get receive data length, length = set_data_length - remain_length
            // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = NEW_RC_RX_BUF_NUM - hdma_uart8_rx.Instance->NDTR; // ���������ݽ���

            // reset set_data_lenght
            // �����趨���ݳ���
            hdma_uart8_rx.Instance->NDTR = NEW_RC_RX_BUF_NUM;

            // set memory buffer 0
            // �趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_uart8_rx);

            if (this_time_rx_len == NEW_RC_FRAME_LENGTH)
            {
                // ����ң��������
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
 * @brief ң�������ݽ���
 * @author Skr
 * @param *sbus_buf ���յ������������ָ��
 * @param *rc_ctrl  ң�������ݽṹ��
 */
static void solve_new_rc_data(volatile const uint8_t *sbus_buf, remote_data_t *ctrl)
{
    if (sbus_buf == NULL || ctrl == NULL)
    {
        return;
    }

    // ҡ��
    ctrl->ch_0 = (sbus_buf[2] | (sbus_buf[3] << 8)) & 0x7FF;
    ctrl->ch_1 = ((sbus_buf[3] >> 3) | (sbus_buf[4] << 5)) & 0x7FF;
    ctrl->ch_2 = ((sbus_buf[4] >> 6) | (sbus_buf[5] << 2) | (sbus_buf[6]) << 10) & 0x7FF;
    ctrl->ch_3 = ((sbus_buf[6] >> 1) | (sbus_buf[7] << 7)) & 0x7FF;
    // ҡ������
    ctrl->ch_0 = ctrl->ch_0 - 1024;
    ctrl->ch_1 = ctrl->ch_1 - 1024;
    ctrl->ch_2 = ctrl->ch_2 - 1024;
    ctrl->ch_3 = ctrl->ch_3 - 1024;

    // ��λ�л�����CNS
    ctrl->mode_sw = (sbus_buf[7] >> 4) & 0x03;

    // ��ͣ����
    ctrl->pause = (sbus_buf[7] >> 6) & 0x01;

    // �Զ��尴��
    ctrl->fn_1 = (sbus_buf[7] >> 7) & 0x01;
    ctrl->fn_2 = (sbus_buf[8]) & 0x01;

    // ����
    ctrl->wheel = ((sbus_buf[8] >> 1) | (sbus_buf[9] << 7)) & 0x7FF;
    // ��������
    ctrl->wheel = ctrl->wheel - 1024;

    // �����
    ctrl->trigger = (sbus_buf[9] >> 4) & 0x01;

    // ����ƶ��Լ������ֹ���
    ctrl->mouse_x = (sbus_buf[10] | (sbus_buf[11] << 8)) & 0xFFFF;
    ctrl->mouse_y = (sbus_buf[12] | (sbus_buf[13] << 8)) & 0xFFFF;
    ctrl->mouse_z = (sbus_buf[14] | (sbus_buf[15] << 8)) & 0xFFFF;

    // ��갴��
    ctrl->mouse_left = (sbus_buf[16]) & 0x03;
    ctrl->mouse_right = (sbus_buf[16] >> 2) & 0x03;
    ctrl->mouse_middle = (sbus_buf[16] >> 4) & 0x03;

    // ���̰���
    ctrl->key = (sbus_buf[17] | sbus_buf[18] << 8);

    //    // ��·�л�
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

// �쳣ֵ��⺯��
static inline bool is_outlier(uint16_t new_value, uint16_t last_valid)
{
    return abs((int16_t)(new_value - last_valid)) > MAX_ALLOWED_CHANGE;
}

/*
�Զ�����������ݽ��գ����˲�
*/
void solve_custom_data(volatile const uint8_t *sbus_buf, custom_measure *Custom_data)
{
    if (sbus_buf[0] == 0xA5 && sbus_buf[6] == 0x03 && sbus_buf[5] == 0x02)
    {
        for (int i = 0; i < 4; i++) // TODO�ĳɵ�������
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
            // ��ȡԭʼ����
            uint16_t raw_angle = (uint16_t)(sbus_buf[7 + i * 2] | sbus_buf[8 + i * 2] << 8);
            filter_t *filter = &angle_filters[i];

            // �쳣ֵ���
            if (is_outlier(raw_angle, filter->last_valid) && filter->last_valid != 0)
            {
                // ������쳣ֵ�������һ����Чֵ����
                raw_angle = filter->last_valid;
            }
            else
            {
                // �������һ����Чֵ
                filter->last_valid = raw_angle;
            }

            // �����˲���
            filter->sum -= filter->buffer[filter->index]; // ��ȥ��ֵ
            filter->buffer[filter->index] = raw_angle;    // �洢��ֵ
            filter->sum += raw_angle;                     // ������ֵ
            filter->index = (filter->index + 1) % FILTER_WINDOW_SIZE;

            // ����ƽ��ֵ���洢
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
