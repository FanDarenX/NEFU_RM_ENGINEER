/*
 * @Descripttion: 上位机调参
 * @version: 
 * @Author: lxf
 * @Date: 2023-05-07 19:07:36
 * @LastEditors: lxf（Bear Fly）
 * @LastEditTime: 2023-05-14 17:40:09
 */
/*
//记得开启DMA，循环模式
伏特加上位机调参,网站:https://www.vofa.plus/
*/
//高质量调参
#include "vodka.h"
#include "stdarg.h"
#include "stdlib.h"
#include "usart.h"

#include "CAN_receive.h"
#include "remote_control.h"

struct vodka_just_float_struct_type vodka_float_data;

void date_send(void)
{
//		vodka_float_data.data1 = rc_ctrl.mouse.press_l; //设置值（需要发送的数据1）
//		vodka_float_data.data2 = GM6020_yaw_moto.total_angle;//实际值（需要发送的数据2）
//		vodka_float_data.data3 = GM6020_yaw_moto.angle;//
//		vodka_float_data.data4 = ;
		
//		vodka_float_data.data5 = ;
//		vodka_float_data.data6 = ;
//		vodka_float_data.data7 = ;
//		vodka_float_data.data8 = ;
	
//		vodka_JustFloat_send_struct(&vodka_float_data); //发送数据
}

void vodka_JustFloat_send_struct(struct vodka_just_float_struct_type *data_str)
{
//    uint8_t* psend = NULL;
//    data_str->pend = 0x7f800000; //帧尾
//    
//    psend = (uint8_t *)data_str;
//    
//    /* 这里调用串口字符串将psend发生出去 */
//	HAL_UART_Transmit_DMA(&huart8,psend,sizeof(struct vodka_just_float_struct_type));
}

