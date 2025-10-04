#ifndef VIDEO_DOWNLINK_H__
#define VIDEO_DOWNLINK_H__

// #include "custom.h"

#include "main.h"
#include "remote_control.h"
//#include "custom.h"

/**
 * @brief ���ڳ�ʼ������
 * @author RM��Skr
 * @param *rx1_buf��*rx2_buf ������������ָ��
 * @param ��������Ĵ�С
 */
extern void uart8_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

// �궨��
#define NEW_RC_RX_BUF_NUM 42u   // ˫����
#define NEW_RC_FRAME_LENGTH 21u // һ֡���ݳ���

#define Custom_LENGTH 39u

#define receive_size 39

// �ṹ��
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
�Զ�����������ݽ���
������ƶ�ƽ���˲�
*/
#define FILTER_WINDOW_SIZE 4  // �˲����ڴ�С���ɸ�����Ҫ����
#define MAX_ALLOWED_CHANGE 300    // ��������仯��������ʵ�����������

#define Custom_RX 999999


// �����˲����ݽṹ
typedef struct {
    uint16_t buffer[FILTER_WINDOW_SIZE];
    uint8_t index;
    uint16_t sum;
    uint16_t last_valid;          // �洢���һ����Чֵ
} filter_t;


typedef struct
{
	uint16_t angle;
	uint16_t last_angle;
	int total_angle;
	int total_last_angle;
	int round_cnt;
	float total_angle_ture;

	/*�ֲ�������Ӳ�����*/
	int Positive_to_negative; // ���为��ʱ�ñ���=��ǰ�ܽǶ�+����ֵ��160��
	// int Negative_to_positive; // ������ֱ��ʹ�õ�ǰ�ܽǶ�,�򻯿��Բ���Ҫ

	/*��ʱ����*/
	int tmp_total_angle; // ���ڴ������为ʱ���ܽǶ�
} custom_measure;


// �궨�庯��
// �²���
#define RC_SW_C ((uint8_t)0)
#define RC_SW_N ((uint8_t)1)
#define RC_SW_S ((uint8_t)2)
// ��갴��
#define MOUSE_PRESS_L(s) ((s)->mouse_left == 1)
#define MOUSE_PRESS_R(s) ((s)->mouse_right == 1)
#define MOUSE_PRESS_M(s) ((s)->mouse_middle == 1)
// �°���
#define FN_SET ((uint8_t)1)
#define FN_RESET ((uint8_t)0)

// ����
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
 * @brief �����жϴ�������˫����������
 * @author RM��Skr
 * @note  ע���_it�ļ���Ĵ����жϴ�������ע�͵�����ֹ�ض���
 */
extern void USART6_IRQHandler(void);

/**
 * @brief ң�������ݽ���
 * @author Skr
 * @param *sbus_buf ���յ������������ָ��
 * @param *rc_ctrl  ң�������ݽṹ��
 */
extern void solve_new_rc_data(volatile const uint8_t *sbus_buf, remote_data_t *rc_ctrl);

// ��������
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