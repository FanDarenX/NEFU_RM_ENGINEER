#ifndef __CUSTOM_H__
#define __CUSTOM_H__

#include <stdio.h>
#include "main.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <usart.h>
#include "position_task.h"
#include "keyscan_task.h"
#include "Video_downlink.h"


#define USART_Custom huart8
#define receive_size 39

void UsartPrintf(UART_HandleTypeDef USARTx, char *fmt, ...);
int fputc(int ch, FILE *f);

typedef struct
{
	// 0--255 || 0x00--0xff
	uint8_t data[5];
	// 0--65535 || 0x0000--0xffff
} CONTROL_DATA; // 遥控器数据结构体




typedef struct
{
	uint32_t	mouse_x;
	uint32_t	mouse_y;
	uint32_t	roller;//滚轮
	uint8_t		left;
	uint8_t		right;
	uint32_t	dates_1;
	uint32_t	dates_2;
	uint8_t		keystroke[16];//按键
	
}KEYBOARD_DATE;//键盘数据结构体


typedef union
{
	char	date[2];
	uint32_t	number;     
}NUMBER_CONVERT;//数值转换共用体



void keystroke_dispose(void);


void Synchronize_Frame(void);
void Custom_task(void);
void get_custom(void);

extern uint8_t receive_data[receive_size]; // 接收数组
extern CONTROL_DATA control_data;		   // 自定义控制器编码器数据

extern uint8_t RECEIVE_STA;
extern uint8_t receive_data[receive_size]; // 接收数组

extern int Custom_action;

extern custom_measure Custom_data[7];

extern int initial_angle;

extern uint8_t receive_data[receive_size];

/*储存计算后的目标变量值*/
extern float yaw_custom;
extern float pic_custom;
extern float Cup_pitch_custom;
extern float Cup_roll_custom;
extern float X_custom;
extern float Y_custom;
extern float Z_custom;

#endif