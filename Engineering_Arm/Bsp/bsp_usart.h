#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"
#include "usart.h"
//#include "gimbal_behaviour.h"
#define PC_HUART      huart1
#define PC_BUF_LEN    50
extern	bool_t PC_update_flag;
extern	bool_t PC_flag;

//extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
#define PTZ_HUART      huart1

extern uint8_t PC_Buf[PC_BUF_LEN];
typedef enum
{
	rune_dema	='a',	//大符标定    0x61
	rune_mode 	='d',	//大符模式  0x64
	assist_mode	='c',	//辅助瞄准  0x63
	nomal_mode	=0,	//普通模式
	rune_big_mode='e',//大符
	nomal='z'
}Command;

typedef struct
{
	int16_t pitch;
	int16_t yaw;
	int16_t buffpitch;
	int16_t buffyaw;
	float distance;
	uint8_t shoot;
	uint8_t num_shoot;//大符下打出的子弹数

	int16_t gray;
	char BaoGuang;
	const fp32 *PC_INT_angle_point;
	fp32 Hero_yaw_angle[10];
  fp32 Hero_pitch_angle[10];
	uint32_t  PC_time;
	uint16_t pc_time1;
	uint16_t pc_time2;
	fp32 PC_yaw_angle[10];
  fp32 PC_pitch_angle[10];
	
}_PCDATA;

extern _PCDATA Pc_Data;
#define HEAD_LEN    4
#define ON  1
#define OFF 0

#define PC_HUART      huart1
#define PC_BUF_LEN    50
extern int Shake_hands_flag;

extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void Bsp_UsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size);
void UART_IdleRxCallback(UART_HandleTypeDef *huart);

extern void usart6_tx_dma_init(uint8_t *data,uint16_t len);
extern void usart6_tx_dma_enable(uint8_t *data, uint16_t len);
#endif

