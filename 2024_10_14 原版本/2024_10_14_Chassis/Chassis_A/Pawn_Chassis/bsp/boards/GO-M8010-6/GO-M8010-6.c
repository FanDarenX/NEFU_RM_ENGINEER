#include "usart.h"
#include "motor_control.h"
#include "crc_ccitt.h"
#include "stdio.h"

#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 

MOTOR_recv PC_Buf1;
MOTOR_recv PC_Buf6;
MOTOR_send Utree_send_data[4];   //以全局变量声明电机控制结构体和电机数据结构体，方便在故障时通过debug查看变量值
MOTOR_recv Utree_get_data[4];

int modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 17;
    motor_s->motor_send_data.head[0] = 0xFE;
    motor_s->motor_send_data.head[1] = 0xEE;
	
//		SATURATE(motor_s->id,   0,    15);
//		SATURATE(motor_s->mode, 0,    7);
		SATURATE(motor_s->K_P,  0.0f,   24.599f);
		SATURATE(motor_s->K_W,  0.0f,   24.599f);
		SATURATE(motor_s->T,   -22.0f,  22.0f);
		SATURATE(motor_s->W,   -804.00f,  804.00f);
		SATURATE(motor_s->Pos, -65051.0f,  65051.0f);

	
    motor_s->motor_send_data.mode.id   = motor_s->id;
    motor_s->motor_send_data.mode.status  = motor_s->mode;
    motor_s->motor_send_data.comd.k_pos  =  motor_s->K_P/25.6f*1280;
    motor_s->motor_send_data.comd.k_spd  =  motor_s->K_W/25.6f*1280;
    motor_s->motor_send_data.comd.pos_des  = -motor_s->Pos/6.2832f*32768 *6.33f;
    motor_s->motor_send_data.comd.spd_des  = -motor_s->W/6.2832f*256*6.33f;
    motor_s->motor_send_data.comd.tor_des  = -motor_s->T*256/6.33f;
    motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
    return 0;
}

int extract_data(MOTOR_recv *motor_r)
{
    if(motor_r->motor_recv_data.CRC16 !=
        crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14)){
        // printf("[WARNING] Receive data CRC error");
        motor_r->correct = 0;
        return motor_r->correct;
    }
    else
		{
        motor_r->motor_id = motor_r->motor_recv_data.mode.id;
        motor_r->mode = motor_r->motor_recv_data.mode.status;
        motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
        motor_r->MError = motor_r->motor_recv_data.fbk.MError;
        motor_r->W = -((float)motor_r->motor_recv_data.fbk.speed/256)*6.2832f /6.33f;
        motor_r->T = -((float)motor_r->motor_recv_data.fbk.torque) / 256*6.33f;
        motor_r->Pos = -6.2832f*((float)motor_r->motor_recv_data.fbk.pos) / 32768/6.33f;
				motor_r->footForce = motor_r->motor_recv_data.fbk.force;
				motor_r->correct = 1;
        return motor_r->correct;
    }
}

HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData,uint8_t usart,float T)
{
	if(usart==6)
	{
		pData->T = T;
		modify_data(pData);
		HAL_UART_Transmit_DMA(&huart6, (uint8_t *)pData, sizeof(pData->motor_send_data)); 
	}
  else if(usart==1)
	{
	  pData->T = T;
		modify_data(pData);
		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)pData, sizeof(pData->motor_send_data)); 
	}
}

const MOTOR_recv *get_Utree_motor_measure_point(uint8_t i)
{
    return &Utree_get_data[i];
}