/*
																 _                                _
	 /\                                                         (_)                              (_)
	/  \     _ __    ___   ___   ______    ___   _ __     __ _   _   _ __     ___    ___   _ __   _   _ __     __ _
   / /\ \   | '__|  / _ \ / __| |______|  / _ \ | '_ \   / _` | | | | '_ \   / _ \  / _ \ | '__| | | | '_ \   / _` |
  / ____ \  | |    |  __/ \__ \          |  __/ | | | | | (_| | | | | | | | |  __/ |  __/ | |    | | | | | | | (_| |
 /_/    \_\ |_|     \___| |___/           \___| |_| |_|  \__, | |_| |_| |_|  \___|  \___| |_|    |_| |_| |_|  \__, |
														  __/ |                                                __/ |
														 |___/                                                |___/
*/

#include "custom.h"
#include "math.h"

//-----TX--PE1-----//
//-----RX--PE0-----//

uint8_t RECEIVE_STA = 0;
uint8_t receive_data[receive_size] = {0}; // 接收数组

CONTROL_DATA control_data = {0}; // 遥控器数据


KEYBOARD_DATE keyboard_date;

NUMBER_CONVERT Number_convert; // 数值转换

int Custom_action = 0;

int Custom_Cup_roll = 0; // 用于相对角度
int initial_angle = 0;

int new_data_flag = 0;

/*储存计算后的目标变量值*/
float yaw_custom = 0.0f;
float pic_custom = 0.0f;
float Cup_pitch_custom = 0.0f;
float Cup_roll_custom = 0.0f;
float X_custom = 0.0f;
float Y_custom = 0.0f;
float Z_custom = 0.0f;



/**
 * @brief HAL库UART接收DMA空闲中断
 *
 * @param huart UART编号
 * @param Size 长
 */
// A5 00 00 00 00 02 03 11 22 33 44 55 00 10 00 10 00 10 00 10 00 10 00 10 00 10 00 00 00 00 00 00 00 00 00 00 00 00 00

// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	if (huart->Instance == UART8)
//	{
//		HAL_UARTEx_ReceiveToIdle_DMA(huart, receive_data, receive_size);
//	}
// }

// void USART3_IRQHandler(void) {
////    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE)) {
////        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
//        get_custom(); // 处理数据
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receive_data, receive_size);
////    }
//    HAL_UART_IRQHandler(&huart3);
//}

// void UART8_IRQHandler(void)
//{
////	if (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_IDLE))
////	{
////		__HAL_UART_CLEAR_IDLEFLAG(&huart8);

////		new_data_flag = 1;
//
//		get_custom();
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart8, receive_data, receive_size);
////	}
//	HAL_UART_IRQHandler(&huart8);
//}

// void UART8_IRQHandler(void)
//{
//     HAL_UART_IRQHandler(&huart8);
//
//     // 检测空闲中断
//     if(__HAL_UART_GET_FLAG(&huart8, UART_FLAG_IDLE) != RESET)
//     {
//         __HAL_UART_CLEAR_IDLEFLAG(&huart8);
//
//         // 停止DMA传输
//         HAL_UART_DMAStop(&huart8);
//
//		get_custom();
//
//		// 重新启动DMA接收
//         __HAL_DMA_SET_COUNTER(&hdma_uart8_rx, receive_size);
//         hdma_uart8_rx.Instance->CR |= DMA_SxCR_EN;
//         __HAL_UART_ENABLE(&huart8);
//     }
// }

// void UART8_IRQHandler(void)
//{
//     /* 处理UART中断 */
//     HAL_UART_IRQHandler(&huart8);
//
////    /* 检测空闲中断 */
////    if(__HAL_UART_GET_FLAG(&huart8, UART_FLAG_IDLE) != RESET)
////    {
////        /* 清除空闲中断标志 */
////        __HAL_UART_CLEAR_IDLEFLAG(&huart8);
////
////        /* 停止DMA传输 */
////        HAL_UART_DMAStop(&huart8);
//
//        /* 计算接收到的数据长度 */
//        uint16_t received_length = receive_size - __HAL_DMA_GET_COUNTER(&hdma_uart8_rx);
//
//        /* 处理接收到的数据 */
//        if(received_length > 0)
//        {
//            get_custom(); // 您的自定义数据处理函数
//        }
//        /* 使用ReceiveToIdle_DMA重新启动接收 */
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart8, receive_data, receive_size);
//
//        /* 或者保持原有方式重启DMA（二选一） */
//        // __HAL_DMA_SET_COUNTER(&hdma_uart8_rx, receive_size);
//        // hdma_uart8_rx.Instance->CR |= DMA_SxCR_EN;
//        // __HAL_UART_ENABLE(&huart8);
////    }
//}

void get_custom(void)
{
	//	int received_size = receive_size - __HAL_DMA_GET_COUNTER(&hdma_uart8_rx);
	//	if (received_size >= 39)
	//	{
	//	HAL_UARTEx_ReceiveToIdle_DMA(&huart8, receive_data, receive_size); // TODO测试读取是否有问题
	if (receive_data[0] == 0xA5 && receive_data[6] == 0x03 && receive_data[5] == 0x02)
	{
		//		Custom_rx_threshold = 10000;
		//		Custom_rx_threshold = 1000;
		// control_data.data[0] = receive_data[7];
		// control_data.data[1] = receive_data[8];
		// control_data.data[2] = receive_data[9];
		// control_data.data[3] = receive_data[10];
		// control_data.data[4] = receive_data[11];
		/*仿写绝对角度*/ // TODO
		/**
		 *用到total_angle:吸盘pitch，DM的yaw和pitch,抬升前伸横移（需要测试增加平均值或高斯滤波减少抖动）
		 *吸盘roll用相对角度
		 *假定理想法：
		 *假设通过滤波后得到的数值为定值，则设置标志位，初始化将首100次读到值的平均值作为初值，之后读取值减去初始平均值，伪从0开始
		 *实际可以略微增添补偿值
		 *有limit保底
		 *注意横移部分是否需要单独添加标志位，防止抬升机械卡位
		 *关注抬升倍数放大后跳变的放大是否有较大影响，
		 *计算编码值对应齿数
		 *还有DM的弧度制转化
		 *微小跳变在TD函数的作用下会消除
		 */
		for (int i = 0; i < 4; i++) // TODO改成单独试试
		{
			// Custom_data[i].last_angle = Custom_data[i].angle;
			Custom_data[i].angle = (uint16_t)(receive_data[7 + i * 2] | receive_data[8 + i * 2] << 8);
			// 	Custom_data[i].total_last_angle = Custom_data[i].total_angle;
			// 	if (Custom_data[i].angle - Custom_data[i].last_angle > 2048)
			// 	{
			// 		Custom_data[i].round_cnt -= 1;
			// 		// 正变负
			// 		Custom_data[i].Positive_to_negative = 1;
			// 	}
			// 	else if (Custom_data[i].angle - Custom_data[i].last_angle < -2048)
			// 	{
			// 		Custom_data[i].round_cnt += 1;
			// 		// 负变正
			// 		Custom_data[i].Positive_to_negative = 0;
			// 	}
			// 	Custom_data[i].total_angle = Custom_data[i].round_cnt * 4096 + Custom_data[i].angle;

			// 	/*软件补足硬件缺陷*/
			// 	if (Custom_data[i].Positive_to_negative)															 // 正变负
			// 		Custom_data[i].tmp_total_angle = Custom_data[i].total_angle - 210.0f * Custom_data[i].round_cnt; // 差值大概为160
			// 	else																								 // 负变正
			// 		Custom_data[i].tmp_total_angle = Custom_data[i].total_angle - 210.0f * Custom_data[i].round_cnt;

			// 	Custom_data[i].total_angle_ture = Custom_data[i].total_angle * PI * 2 / 4096;
			// }
			// if (Custom_data[3].total_angle != 0 && !Custom_Cup_roll) // 记录初始角度（吸盘roll）
			// {
			// 	initial_angle = Custom_data[3].tmp_total_angle;
			// 	Custom_Cup_roll = 1;
			// }
		}
		//	else
		//	{
		//		Synchronize_Frame();
		//	}
		//	}
		//	 else
		//	 {
		//	 	Synchronize_Frame();
		//	 }
	}
	else if (receive_data[0] == 0xA5 && receive_data[6] == 0x03 && receive_data[5] == 0x04)
	{
		Number_convert.date[0] = receive_data[7];
		Number_convert.date[1] = receive_data[8];
		keyboard_date.mouse_x = Number_convert.number; // X轴

		Number_convert.date[0] = receive_data[9];
		Number_convert.date[1] = receive_data[10];
		keyboard_date.mouse_y = Number_convert.number; // Y轴

		Number_convert.date[0] = receive_data[11];
		Number_convert.date[1] = receive_data[12];
		keyboard_date.roller = Number_convert.number; // 滚轮

		keyboard_date.left = receive_data[13]; // 左键

		keyboard_date.right = receive_data[14]; // 右键

		keyboard_date.dates_1 = receive_data[15]; // 键值
		keyboard_date.dates_2 = receive_data[16]; // 键值

		keystroke_dispose();
		// 链路切换
		if (sbus_online_flag < 0)
		{
			rc_ctrl.mouse.press_l = keyboard_date.left;
			rc_ctrl.mouse.press_r = keyboard_date.right;
			rc_ctrl.key.v = receive_data[15] | receive_data[16] << 8;
			rc_ctrl.mouse.x = keyboard_date.mouse_x;
			rc_ctrl.mouse.y = keyboard_date.mouse_y;
			rc_ctrl.mouse.z = keyboard_date.roller;
		}
	}
}

void keystroke_dispose(void)
{
	keyboard_date.keystroke[0] = (keyboard_date.dates_1 & 0x01);		// W
	keyboard_date.keystroke[1] = ((keyboard_date.dates_1 & 0x02) >> 1); // S
	keyboard_date.keystroke[2] = ((keyboard_date.dates_1 & 0x04) >> 2); // A
	keyboard_date.keystroke[3] = ((keyboard_date.dates_1 & 0x08) >> 3); // D
	keyboard_date.keystroke[4] = ((keyboard_date.dates_1 & 0x10) >> 4); // SHIFT
	keyboard_date.keystroke[5] = ((keyboard_date.dates_1 & 0x20) >> 5); // CTRL
	keyboard_date.keystroke[6] = ((keyboard_date.dates_1 & 0x40) >> 6); // Q
	keyboard_date.keystroke[7] = ((keyboard_date.dates_1 & 0x80) >> 7); // E

	keyboard_date.keystroke[8] = (keyboard_date.dates_2 & 0x01);		 // R
	keyboard_date.keystroke[9] = ((keyboard_date.dates_2 & 0x02) >> 1);	 // F
	keyboard_date.keystroke[10] = ((keyboard_date.dates_2 & 0x04) >> 2); // G
	keyboard_date.keystroke[11] = ((keyboard_date.dates_2 & 0x08) >> 3); // Z
	keyboard_date.keystroke[12] = ((keyboard_date.dates_2 & 0x10) >> 4); // X
	keyboard_date.keystroke[13] = ((keyboard_date.dates_2 & 0x20) >> 5); // C
	keyboard_date.keystroke[14] = ((keyboard_date.dates_2 & 0x40) >> 6); // V
	keyboard_date.keystroke[15] = ((keyboard_date.dates_2 & 0x80) >> 7); // B
}

//void Custom_state(void)
//{
//	if (Custom_rx_threshold > 0)
//	{
//		Custom_rx_threshold = Custom_rx_threshold - 1;
//	}
//}

// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//     if (huart->Instance == UART8)
//     {
//        new_data_flag = 1;
//         // 重新启动DMA传输
//         HAL_UARTEx_ReceiveToIdle_DMA(huart, receive_data, receive_size);
//		__HAL_UART_CLEAR_IDLEFLAG(&huart8);
//     }
// }

void Custom_task(void)
{
	sbus_state();
	Custom_state();
	//	Custom_state();
	//	if (new_data_flag)
	//	{
	//		new_data_flag = 0;
	//		get_custom();
	//	}
	//	get_custom();
	// if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_B)
	// {
	// 	for (int i = 0; i < 7; i++)
	// 	{
	// 		Custom_data[i].round_cnt = 0;
	// 	}
	// }
	if (key_task.custom_mode) // 这是在左上右上（debug测试使用）时，并且没进     && Custom_rx_threshold
	{
		yaw_custom = (Custom_data[0].angle - 1500) / 1000.0f * 135.0f;
		pic_custom = (Custom_data[1].angle - 1500) / 1000.0f * 180.0f; // TODO
		Cup_pitch_custom = ((Custom_data[2].angle - 1500) / 1000.0f * 135.0f)*1.5f;
		Cup_roll_custom = -(Custom_data[3].angle - 1500) / 1000.0f * 135.0f;

		// 正变负，临时变量等于总角度+160
		// 负变正，直接使用总角度
		//  2621 ~ 1092 ~ -240（正变负丢失160）958~1002 -> 980:90度
		//   ~ 1555 ~    995~1000     ->997.5:90度
		//	-700 ~ 260 ~ 1305  961~997     979->90度
		// yaw_custom = (-Custom_data[0].tmp_total_angle - 860.0f) / 990.0f * 90.0f;		   // DM_yaw860
		// pic_custom = (-Custom_data[1].tmp_total_angle - 241.0f) / 990.0f * 90.0f + 180.0f; // DM_pitch -241
		// Cup_pitch_custom = -(Custom_data[3].tmp_total_angle - 957.0f) / 990.0f * 90.0f;	   // 中957                            995.5->90°
		// 																				   // Cup_target_roll = control_data.encoder_1;								 // 相对角度
		// 																				   //		/*龙门架*/
		// Cup_roll_custom = (Custom_data[2].tmp_total_angle - initial_angle) / 4096.0f * 45.0f;

		//-2993 ~ -1728 ~ -601   2392
		//-1369 ~ -3724  2355
		//-1526 ~ -3938  2412
		//		X_custom = 0.0f;														  // 1063~3435
		//		Y_custom = (-Custom_data[1].tmp_total_angle - 1200.0f) / 2185.0f * 49.0f; // -1119~-3385
		//		Z_custom = (-Custom_data[2].tmp_total_angle - 1153.0f) / 2347.0f * 33.0f; // -1153~-3500

		/*避免抽风*/
		limit_key(&Cup_pitch_custom, 230, -90);
		limit_key(&Cup_roll_custom, 90, -90);
		limit_key(&yaw_custom, 90, -90);
		limit_key(&pic_custom, 200, -200); // TODO新版工程180
										   //		limit_key(&X_custom, 0.1, -0.1);
										   //		limit_key(&Y_custom, 49.0f, 0.7f);
										   //		limit_key(&Z_custom, 15.0f, 1.0f);
	}
}

/*清空*/
void Synchronize_Frame(void)
{
	for (int i = 0; i < receive_size - 1; i++)
	{
		if (receive_data[i] == 0xA5)
		{
			// 查找帧起始标志
			// 将帧头移动到缓冲区开头
			memmove(receive_data, &receive_data[i], receive_size - i);
			memset(&receive_data[receive_size - i], 0, i); // 清空无效数据
			break;
		}
	}
}

/*发送部分*/
/*********************************************************
 *方法一
 *重定义 fputc 函数
 *
 *********************************************************/
int fputc(int ch, FILE *f)
{
	while (__HAL_UART_GET_FLAG(&USART_Custom, UART_FLAG_TC) == RESET)
		;
	HAL_UART_Transmit(&USART_Custom, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
/*使用示例*/
/*
printf("Hello, Robomaster!\n");
fputc('A', stdout); // 单个字符输出
*/

/********************
 *方法二
 *字符串打印
 *
 ********************/
void UsartPrintf(UART_HandleTypeDef USARTx, char *fmt, ...) // 使用例子：UsartPrintf(USART_DEBUG, "The USART3 is OK!\r\n");
{

	unsigned char UsartPrintfBuf[296];
	va_list ap;
	unsigned char *pStr = UsartPrintfBuf;

	va_start(ap, fmt);
	vsnprintf((char *)UsartPrintfBuf, sizeof(UsartPrintfBuf), fmt, ap); // 格式化
	va_end(ap);

	while (*pStr != NULL)
	{
		HAL_UART_Transmit(&USARTx, (uint8_t *)pStr++, 1, HAL_MAX_DELAY);
	}
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	return;
// }
