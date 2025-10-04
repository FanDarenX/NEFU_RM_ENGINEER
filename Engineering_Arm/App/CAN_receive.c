#include "CAN_receive.h"

CAN_TxHeaderTypeDef TX1_message; // ALL
CAN_TxHeaderTypeDef TX2_message; // ALL

CAN_RxHeaderTypeDef RX1message; // CAN1
CAN_RxHeaderTypeDef RX2message; // CAN2

/*A_TO_CA_TO_C*/
CAN_TxHeaderTypeDef TX_message;
uint8_t TXdata[8];

uint8_t TX1data[8]; // ALL
uint8_t TX2data[8];
uint8_t RX1data[8]; // CAN1
uint8_t RX2data[8]; // CAN2

/*CAN1*/
/*�����ĵ��*/
moto_measure chassis_moto[4] = {0}; // ���̵��
/*��е������*/
moto_measure Cup_moto[2] = {0}; // ��е������pitch��roll
/*��е��yaw��pitch*/
motor_t DM_yaw = {0};
motor_t DM_pitch = {0};
/*CAN1*/

/*CAN2*/
/*���ż�̧��z���*/
moto_measure Uplift_moto[4] = {0}; // ���ż�̧�����
/*���żܺ���x���*/
moto_measure X_moto = {0};
/*���ż�����y���*/
moto_measure Y_moto = {0};
/*CAN2*/

/**
 *
 *
 *
 *
 *
 *
 *
 *
 */
/*��̭����*/
C_GO_POS_t c_go_pos;
moto_measure position_moto_6020; // ĩ��6020���
moto_measure yaw_moto_6020;		 // yaw��6020���
moto_measure moto_6020[2] = {0};
uint8_t TX_C_Data_KEY[8];
uint8_t TX_C_Data_GO[8];
uint8_t TX_C_Data_END[8];
uint8_t TX_QI_Data_other[8];
CAN_TxHeaderTypeDef TX_C_board_KEY;
CAN_TxHeaderTypeDef TX_Qi_board_other;
// CAN_TxHeaderTypeDef TX_C_board_GO;
CAN_TxHeaderTypeDef TX_C_board_END;
/*��̭����*/
/**
 *
 *
 *
 *
 *
 *
 *
 *
 */
/**************************************************************/
/*
	�������������Ҫ�����ڵõ�������ٶȣ��Ƕȵ�����
	ͬʱ���й����⣬ÿ�ι�������Ȧ��+1��-1��
	�����Ϳ���ͳ�Ƶ��ת���ܵĽǶ����ڽǶ��ٶȴ������ơ�
*/
void get_moto_measure(moto_measure *ptr, uint8_t RXdata[8])
{
	ptr->last_angle = ptr->angle;
	ptr->last_speed = ptr->speed;
	ptr->last_total_angle = ptr->total_angle;

	ptr->angle = (RXdata[0] << 8) | RXdata[1]; // �������
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
/**************************************************************/
float ABS(float x)
{
	return x > 0 ? x : -x;
}
/*ת��ת�ٵ�*/
void Gm6020_Visualization_Led(int16_t Speed)
{
	// �� out ӳ�䵽 0 �� 8 ֮�������
	int level = ABS((float)Speed) / 10;

	// ȷ�� level �� 0 �� 8 ֮��
	if (level < 0)
		level = 0;
	if (level > 8)
		level = 8;

	// ���� level ���� LED �ĵ�����Ϩ��
	for (int i = 1; i <= 8; i++)
	{
		GPIO_PinState state = (i <= level) ? GPIO_PIN_RESET : GPIO_PIN_SET;
		switch (i)
		{
		case 1:
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, state);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, state);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, state);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, state);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, state);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, state);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, state);
			break;
		case 8:
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, state);
			break;
		}
	}
}
/*DM�Ƕȵ�*/
void DM4310_YAW(float pos)
{
	int ledIndex;
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
	if (pos >= 0.2)
	{
		// pos �� 0 �� 3.1415926 ��Ϊ�����ȼ�
		if (pos <= 1.0471976f)
		{ // 3.1415926 / 3
			ledIndex = 5;
		}
		else if (pos <= 2.0943952f)
		{ // 2 * 3.1415926 / 3
			ledIndex = 6;
		}
		else
		{
			ledIndex = 7;
		}
	}
	else if (pos <= -0.2)
	{
		// pos �� 0 �� -3.1415926 ��Ϊ�����ȼ�
		if (pos >= -1.0471976f)
		{ // -3.1415926 / 3
			ledIndex = 3;
		}
		else if (pos >= -2.0943952f)
		{ // -2 * 3.1415926 / 3
			ledIndex = 2;
		}
		else
		{
			ledIndex = 1;
		}
	}

	// ���� ledIndex ���� LED �ĵ�����Ϩ��
	for (int i = 1; i <= 8; i++)
	{
		GPIO_PinState state = (i == ledIndex) ? GPIO_PIN_RESET : GPIO_PIN_SET;

		switch (i)
		{
			//		case 1:
			//			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, state);
			//			break;
		case 7:
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, state);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, state);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, state);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, state);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, state);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, state);
			break;
		}
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RX1message, RX1data) == HAL_OK)
	{
		if (hcan->Instance == CAN1)
		{
			switch (RX1message.StdId)
			{
			/*���ǵ����ĸ����*/
			// case CAN_3508M1_ID:
			// case CAN_3508M2_ID:
			// case CAN_3508M3_ID:
			// case CAN_3508M4_ID:
			// {
			// 	static int j;
			// 	j = RX1message.StdId - CAN_3508M1_ID;
			// 	get_moto_measure(&chassis_moto[j], RX1data);
			// 	break;
			// }
			/*���ǻ�е�������������*/
			case CAN_2006M1_ID:
			case CAN_2006M2_ID:
			{
				static int i;
				i = RX1message.StdId - CAN_2006M1_ID;
				get_moto_measure(&Cup_moto[i], RX1data);
				break;
			}
			case CAN_YAW_DM_ID:
				dm_motor_fbdata(&DM_yaw, RX1data);
				receive_motor_data(&DM_yaw, RX1data);
				break;
			case CAN_PITCH_DM_ID:
				dm_motor_fbdata(&DM_pitch, RX1data);
				receive_motor_data(&DM_pitch, RX1data);
				break;

			default:
			{
				break;
			}
			}
		}
	}

	if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RX2message, RX2data) == HAL_OK)
	{
		if (hcan->Instance == CAN2) // ID����
		{
			switch (RX2message.StdId)
			{
			/*�������ż�̧���ĸ����*/
			case CAN_3508MA_ID:
			case CAN_3508MB_ID:
			case CAN_3508MC_ID:
			case CAN_3508MD_ID:
			{
				static int j;
				j = RX2message.StdId - CAN_3508MA_ID;
				get_moto_measure(&Uplift_moto[j], RX2data);
				break;
			}
			/*�������ż�����y����*/
			case CAN_2006LY_ID:
				get_moto_measure(&Y_moto, RX2data);
				break;
			/*�������żܺ���x����*/
			case CAN_2006LX_ID:
				get_moto_measure(&X_moto, RX2data);
				break;
			default:
			{
				break;
			}
			}
		}
	}
}

/**************************************************************/
/**
  * @brief ��������������
  * @param CAN1 or CAN2

  * @retval None
  */
/***************************************************************/

/*�����ĵ����������*/
void CHASSIS_CURRENT(int16_t K1, int16_t K2, int16_t K3, int16_t K4) // CAN1 ID:1~4
{
	TX1_message.StdId = 0x200; // �����ʶ��
	TX1_message.IDE = CAN_ID_STD;
	TX1_message.RTR = CAN_RTR_DATA;
	TX1_message.DLC = 8; // ���ͳ��� (x�ֽ�)
	TX1_message.TransmitGlobalTime = DISABLE;

	TX1data[0] = K1 >> 8;
	TX1data[1] = K1;
	TX1data[2] = K2 >> 8;
	TX1data[3] = K2;
	TX1data[4] = K3 >> 8;
	TX1data[5] = K3;
	TX1data[6] = K4 >> 8;
	TX1data[7] = K4;
	HAL_CAN_AddTxMessage(&hcan1, &TX1_message, TX1data, CAN_FILTER_FIFO0); // �����ݴ��������FIFOx
}
/*���ż�̧��z���ĵ����������*/
void Uplift_CURRENT(int16_t K1, int16_t K2, int16_t K3, int16_t K4) // CAN2 ID:1~4
{
	TX2_message.StdId = 0x200;
	TX2_message.IDE = CAN_ID_STD;
	TX2_message.RTR = CAN_RTR_DATA;
	TX2_message.DLC = 8;
	TX2_message.TransmitGlobalTime = DISABLE;

	TX2data[0] = K1 >> 8;
	TX2data[1] = K1;
	TX2data[2] = K2 >> 8;
	TX2data[3] = K2;
	TX2data[4] = K3 >> 8;
	TX2data[5] = K3;
	TX2data[6] = K4 >> 8;
	TX2data[7] = K4;
	HAL_CAN_AddTxMessage(&hcan2, &TX2_message, TX2data, CAN_FILTER_FIFO0);
}
/*��е������pitch��row��������*/
void Cup_CURRENT(int16_t K1, int16_t K2) // CAN1	ID:5~8
{
	TX1_message.StdId = 0x1FF;
	TX1_message.IDE = CAN_ID_STD;
	TX1_message.RTR = CAN_RTR_DATA;
	TX1_message.DLC = 4;
	TX1_message.TransmitGlobalTime = DISABLE;

	TX1data[0] = K1 >> 8;
	TX1data[1] = K1;
	TX1data[2] = K2 >> 8;
	TX1data[3] = K2;
	// TXdata[4] = 0;
	// TXdata[5] = 0;
	// TXdata[6] = 0;
	// TXdata[7] = 0;
	HAL_CAN_AddTxMessage(&hcan1, &TX1_message, TX1data, CAN_FILTER_FIFO0);
}

/*���żܺ���x��&&y�����*/
void XY_CURRENT(int16_t K1, int16_t K2) // CAN2
{
	TX2_message.StdId = 0x1FF;
	TX2_message.IDE = CAN_ID_STD;
	TX2_message.RTR = CAN_RTR_DATA;
	TX2_message.DLC = 4;
	TX2_message.TransmitGlobalTime = DISABLE;

	TX2data[0] = K1 >> 8;
	TX2data[1] = K1;
	TX2data[2] = K2 >> 8;
	TX2data[3] = K2;
	// TXdata[4] = 0;
	// TXdata[5] = 0;
	// TXdata[6] = 0;
	// TXdata[7] = 0;
	HAL_CAN_AddTxMessage(&hcan2, &TX2_message, TX2data, CAN_FILTER_FIFO0);
}

void A_TO_C_Chassis(int16_t x, int16_t y, int16_t r)
{
	//	uint32_t send_mail_box;
	TX_message.StdId = CAN2_A_TO_CAN1_C_Chassis;
	TX_message.IDE = CAN_ID_STD;
	TX_message.RTR = CAN_RTR_DATA;
	TX_message.DLC = 0x06;
	TX_message.TransmitGlobalTime = DISABLE;

	TXdata[0] = x >> 8;
	TXdata[1] = x;
	TXdata[2] = y >> 8;
	TXdata[3] = y;
	TXdata[4] = r >> 8;
	TXdata[5] = r;

	HAL_CAN_AddTxMessage(&hcan2, &TX_message, TXdata, CAN_FILTER_FIFO0);
}
void A_TO_C_XY(float X, float Y)
{
	// ����CAN����ͷ
	TX_message.StdId = CAN2_A_TO_CAN1_C_XY;
	TX_message.IDE = CAN_ID_STD;
	TX_message.RTR = CAN_RTR_DATA;
	TX_message.DLC = 0x08; // 8�ֽڣ�������������float
	TX_message.TransmitGlobalTime = DISABLE;

	// ��floatת��Ϊ�ֽ�����
	uint8_t *x_bytes = (uint8_t *)&X;
	uint8_t *y_bytes = (uint8_t *)&Y;

	// ���ݽ��ն��ֽ���������ݣ��˴�ʾ��ΪС����
	// ������ն��Ǵ�����跴תÿ��float���ֽ�˳��
	for (int i = 0; i < 4; i++)
	{
		TXdata[i] = x_bytes[i];		// X���ֽ�
		TXdata[i + 4] = y_bytes[i]; // Y���ֽ�
	}

	// ����CAN����
	HAL_CAN_AddTxMessage(&hcan2, &TX_message, TXdata, CAN_FILTER_FIFO0);
}

/*
void SET_CAN1Back_MOTOR_CURRENT(CAN_HandleTypeDef *hcan, int16_t K1, int16_t K2, int16_t K3, int16_t K4) // CAN1	ID:5~8
{
	TX_message.StdId = 0x1FF;
	TX_message.IDE = CAN_ID_STD;
	TX_message.RTR = CAN_RTR_DATA;
	TX_message.DLC = 8;
	TX_message.TransmitGlobalTime = DISABLE;

	TXdata[0] = K1 >> 8;
	TXdata[1] = K1;
	TXdata[2] = K2 >> 8;
	TXdata[3] = K2;
	TXdata[4] = K3 >> 8;
	TXdata[5] = K3;
	TXdata[6] = K4 >> 8;
	TXdata[7] = K4;
	HAL_CAN_AddTxMessage(&hcan1, &TX_message, TXdata, CAN_FILTER_FIFO0);
}
void SET_CAN2Back_MOTOR_CURRENT(CAN_HandleTypeDef *hcan, int16_t K1, int16_t K2, int16_t K3, int16_t K4) // CAN2 ID:5~8
{
	TX_message.StdId = 0x1FF;
	TX_message.IDE = CAN_ID_STD;
	TX_message.RTR = CAN_RTR_DATA;
	TX_message.DLC = 8;
	TX_message.TransmitGlobalTime = DISABLE;

	TXdata[0] = K1 >> 8;
	TXdata[1] = K1;
	TXdata[2] = K2 >> 8;
	TXdata[3] = K2;
	TXdata[4] = K3 >> 8;
	TXdata[5] = K3;
	TXdata[6] = K4 >> 8;
	TXdata[7] = K4;
	HAL_CAN_AddTxMessage(&hcan2, &TX_message, TXdata, CAN_FILTER_FIFO0);
}

// ����·�崫�����ٶȵ���Ϣ
void CAN_cmd_TO_C_BOARD_SPEED(int16_t x_speed, int16_t y_speed, int16_t z_speed, uint8_t key,
							  uint8_t message1)
// ��������ʱע����������
{
	uint32_t send_mail_box;
	TX_C_board_KEY.StdId = CAN_SEND_TO_C_SPEED_ID;
	TX_C_board_KEY.IDE = CAN_ID_STD;
	TX_C_board_KEY.RTR = CAN_RTR_DATA;
	TX_C_board_KEY.DLC = 0x08;
	TX_C_Data_KEY[0] = (x_speed) >> 8;
	TX_C_Data_KEY[1] = (x_speed);
	TX_C_Data_KEY[2] = (y_speed) >> 8;
	TX_C_Data_KEY[3] = (y_speed);
	TX_C_Data_KEY[4] = (z_speed) >> 8;
	TX_C_Data_KEY[5] = (z_speed);
	TX_C_Data_KEY[6] = (key);
	TX_C_Data_KEY[7] = (message1);

	HAL_CAN_AddTxMessage(&hcan2, &TX_C_board_KEY, TX_C_Data_KEY, &send_mail_box);
}

// ��c�巢��GO�����ģʽ���ٶ�
void CAN_cmd_TO_C_BOARD_GO(int16_t GO0_speed, int16_t GO1_speed, int16_t GO2_speed, uint8_t mode0, uint8_t mode1)
// ��������ʱע����������
{
	uint32_t send_mail_box;
	TX_C_board_GO.StdId = CAN_SEND_TO_C_GO_ID;
	TX_C_board_GO.IDE = CAN_ID_STD;
	TX_C_board_GO.RTR = CAN_RTR_DATA;
	TX_C_board_GO.DLC = 0x08;
	TX_C_Data_GO[0] = (GO0_speed) >> 8;
	TX_C_Data_GO[1] = (GO0_speed);
	TX_C_Data_GO[2] = (GO1_speed) >> 8;
	TX_C_Data_GO[3] = (GO1_speed);
	TX_C_Data_GO[4] = (GO2_speed) >> 8;
	TX_C_Data_GO[5] = (GO2_speed);
	TX_C_Data_GO[6] = (mode0);
	TX_C_Data_GO[7] = (mode1);

	HAL_CAN_AddTxMessage(&hcan2, &TX_C_board_GO, TX_C_Data_GO, &send_mail_box);
}

// ��c�巢��ĩ�˵���ٶȻ�λ��
void CAN_cmd_TO_C_BOARD_END(int16_t roll_speed, int16_t pitch_speed, int16_t yaw_sucker_speed, uint8_t mode0, uint8_t mode1)
// ��������ʱע����������
{
	uint32_t send_mail_box;
	TX_C_board_END.StdId = CAN_SEND_TO_C_END_ID;
	TX_C_board_END.IDE = CAN_ID_STD;
	TX_C_board_END.RTR = CAN_RTR_DATA;
	TX_C_board_END.DLC = 0x08;
	TX_C_Data_END[0] = (roll_speed) >> 8;
	TX_C_Data_END[1] = (roll_speed);
	TX_C_Data_END[2] = (pitch_speed) >> 8;
	TX_C_Data_END[3] = (pitch_speed);
	TX_C_Data_END[4] = (yaw_sucker_speed) >> 8;
	TX_C_Data_END[5] = (yaw_sucker_speed);
	TX_C_Data_END[6] = (mode0);
	TX_C_Data_END[7] = (mode1);

	HAL_CAN_AddTxMessage(&hcan2, &TX_C_board_END, TX_C_Data_END, &send_mail_box);
}
// ����·�巢����صı�־λ
void CAN_cmd_to_Qi_BOARD_flag(uint8_t Qi_sucker, uint8_t Qi_ground, uint8_t mes3, uint8_t mes4,
							  uint8_t mes5, uint8_t mes6, uint8_t mes7, uint8_t mes8)
{
	uint32_t send_mail_box1;
	TX_Qi_board_other.StdId = CAN_SEND_TO_QI_O_ID;
	TX_Qi_board_other.IDE = CAN_ID_STD;
	TX_Qi_board_other.RTR = CAN_RTR_DATA;
	TX_Qi_board_other.DLC = 0X08;
	TX_QI_Data_other[0] = Qi_sucker;
	TX_QI_Data_other[1] = Qi_ground;
	TX_QI_Data_other[2] = mes3;
	TX_QI_Data_other[3] = mes4;
	TX_QI_Data_other[4] = mes5;
	TX_QI_Data_other[5] = mes6;
	TX_QI_Data_other[6] = mes7;
	TX_QI_Data_other[7] = mes8;

	HAL_CAN_AddTxMessage(&hcan2, &TX_Qi_board_other, TX_QI_Data_other, &send_mail_box1);
}

#define get_C_go_pos(ptr, data)                                       \
	{                                                                 \
		(ptr)->GO_yaw = (int16_t)((data)[0] << 8 | (data)[1]);        \
		(ptr)->GO_up = (int16_t)((data)[2] << 8 | (data)[3]);         \
		(ptr)->GO_little_yaw = (int16_t)((data)[4] << 8 | (data)[5]); \
		(ptr)->live_flag = (data)[6];                                 \
		(ptr)->mode = (data)[7];                                      \
	}


*/
/*******************************************************************/
