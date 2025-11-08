#ifndef __COM_PRO_H
#define __COM_PRO_H

#include "stm32f4xx_hal.h"
#include "main.h"

#define DEBUG_USART
#ifdef DEBUG_USART
#define printf1(...) HAL_UART_Transmit(&huart1,\
																		   (uint8_t *)u_buf,\
																			 sprintf((char*)u_buf,__VA_ARGS__),0xffff)
#define printf1_IT(...) HAL_UART_Transmit_IT(&huart1,\
																				(uint8_t *)u_buf,\
																				sprintf((char*)u_buf,__VA_ARGS__))
#define printf1_DMA(...) HAL_UART_Transmit_DMA(&huart1,\
																				(uint8_t *)u_buf,\
																				sprintf((char*)u_buf,__VA_ARGS__))
#endif
																				
#ifndef DEBUG_USART
																				
#define printf1(...) 
#define printf1_IT(...) 
#define printf1_DMA(...) 
																				
#endif																				
extern uint16_t rem_temp[4];																				
extern uint8_t sbus_rx_buffer[18]; 		//ÉùÃ÷Ò£¿ØÆ÷»º´æÊý×é
void Com_Pro(void);
void Rem_int(void);
void Rem_Classis_Control(void);
void Rem_Gimbal_Control(void);
#endif
																				
																				