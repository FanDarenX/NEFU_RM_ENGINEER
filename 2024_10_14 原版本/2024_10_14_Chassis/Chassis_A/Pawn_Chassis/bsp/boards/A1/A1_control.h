#ifndef A1_CONTROL_H
#define A1_CONTROL_H

#include "A1_motor.h"
#include "usart.h"
#include "main.h"


extern A1_MOTOR_recv A1_PC_Buf1;
extern A1_MOTOR_recv A1_PC_Buf6;
extern A1_MOTOR_send A1_send_data[4];   
extern A1_MOTOR_recv A1_get_data[4];

extern void A1_modify_data(A1_MOTOR_send* motor_s);
extern uint8_t A1_extract_data(A1_MOTOR_recv* motor_r);
extern const A1_MOTOR_recv *get_A1_motor_measure_point(uint8_t i);
extern HAL_StatusTypeDef A1_SERVO_Send_recv(A1_MOTOR_send *pData,uint8_t usart,float T);

#endif
