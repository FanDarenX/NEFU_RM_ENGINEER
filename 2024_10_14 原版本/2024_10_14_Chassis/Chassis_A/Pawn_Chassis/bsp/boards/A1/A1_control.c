#include "A1_control.h"
#include "crc32.h"      // CRC32校验算法


#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 
/* TIPS: A1 电机的波特率为4.8Mb/s */

A1_MOTOR_recv A1_PC_Buf1;
A1_MOTOR_recv A1_PC_Buf6;
A1_MOTOR_send A1_send_data[4];  
A1_MOTOR_send A1_Stand_send_data[4];  
A1_MOTOR_recv A1_get_data[4];


/**
  * @Name    A1_modify_data 
  * @brief   改动数据内容，发送给电机
  * @param   motor_s: [输入] 数据包
  * @retval
  * @author  
  * @Data    2023-07-06
 **/


 void A1_modify_data(A1_MOTOR_send* motor_s){
    motor_s->hex_len = 34;
    motor_s->motor_send_data.head.start[0] = 0xFE;
    motor_s->motor_send_data.head.start[1] = 0xEE;
	 
	  // 限幅
		SATURATE(motor_s->K_P,  0.0f,   16.00f);
		SATURATE(motor_s->K_W,  0.0f,   32.00f);
		SATURATE(motor_s->T,   -33.0f,  33.0f);
		SATURATE(motor_s->W,   -30.00f,  30.00f);
		SATURATE(motor_s->Pos, -823549.0f,  823549.0f);
	 
	 
    motor_s->motor_send_data.head.motorID = motor_s->id;
    motor_s->motor_send_data.head.reserved = 0x0;
    motor_s->motor_send_data.Mdata.mode = motor_s->mode;
    motor_s->motor_send_data.Mdata.ModifyBit = 0xFF;
    motor_s->motor_send_data.Mdata.ReadBit = 0x0;
    motor_s->motor_send_data.Mdata.reserved = 0x0;
    motor_s->motor_send_data.Mdata.Modify.L = 0;
    motor_s->motor_send_data.Mdata.Tor = -motor_s->T*256  /9.10f;
    motor_s->motor_send_data.Mdata.Spd = -motor_s->W*128  *9.10f ;
    motor_s->motor_send_data.Mdata.Pos = -(int)((motor_s->Pos/6.2832)*16384.0)   *9.10f;
    motor_s->motor_send_data.Mdata.K_P = motor_s->K_P*2048;
    
//    if(motor_s->motorType == MotorType::A1Go1){
        motor_s->motor_send_data.Mdata.K_W = motor_s->K_W*1024;
//    }
//    else if(motor_s->motorType == MotorType::B1){
//        motor_s->motor_send_data.Mdata.K_W = motor_s->K_W*512;       
//    }

    motor_s->motor_send_data.Mdata.A1_LowHzMotorCmdIndex = 0;
    motor_s->motor_send_data.Mdata.A1_LowHzMotorCmdByte = 0;
    motor_s->motor_send_data.Mdata.Res[0] = motor_s->Res;
    motor_s->motor_send_data.CRCdata.u32 = A1_crc32_core((uint32_t*)(&(motor_s->motor_send_data)), 7);
}




/**
  * @Name    A1_extract_data
  * @brief   电机反馈数据 
  * @param   motor_r: [输入] 数据包
  * @retval
  * @author  
  * @Data    2023-07-06
 **/


uint8_t A1_extract_data(A1_MOTOR_recv* motor_r){
    if(motor_r->motor_recv_data.CRCdata.u32 != A1_crc32_core((uint32_t*)(&(motor_r->motor_recv_data)), 18))
		{   // CRC 不通过
        //std::cout << "[WARNING] Receive data CRC error" << std::endl;
        motor_r->correct = false;
        return motor_r->correct;
    }else{
        motor_r->motor_id = motor_r->motor_recv_data.head.motorID;
        motor_r->mode = motor_r->motor_recv_data.Mdata.mode;
        motor_r->Temp = motor_r->motor_recv_data.Mdata.Temp;
        motor_r->MError = motor_r->motor_recv_data.Mdata.MError;
        motor_r->T = -((float)motor_r->motor_recv_data.Mdata.T) / 256   *9.10f;
        motor_r->W = -((float)motor_r->motor_recv_data.Mdata.W) / 128   /9.10f;
        motor_r->LW = motor_r->motor_recv_data.Mdata.LW;

        motor_r->Acc = (int)motor_r->motor_recv_data.Mdata.Acc;
        motor_r->Pos = -6.2832*((float)motor_r->motor_recv_data.Mdata.Pos) / 16384  /9.10f;
        
        motor_r->gyro[0] = ((float)motor_r->motor_recv_data.Mdata.gyro[0]) * 0.00107993176;
        motor_r->gyro[1] = ((float)motor_r->motor_recv_data.Mdata.gyro[1]) * 0.00107993176;
        motor_r->gyro[2] = ((float)motor_r->motor_recv_data.Mdata.gyro[2]) * 0.00107993176;
        
        motor_r->acc[0] = ((float)motor_r->motor_recv_data.Mdata.acc[0]) * 0.0023911132;
        motor_r->acc[1] = ((float)motor_r->motor_recv_data.Mdata.acc[1]) * 0.0023911132;
        motor_r->acc[2] = ((float)motor_r->motor_recv_data.Mdata.acc[2]) * 0.0023911132;

        motor_r->correct = true;
        return motor_r->correct;
    }
}

HAL_StatusTypeDef A1_SERVO_Send_recv(A1_MOTOR_send *pData,uint8_t usart,float T)
{
	if(usart==6)
	{
		pData->T = T;
		A1_modify_data(pData);
		HAL_UART_Transmit_DMA(&huart6, (uint8_t *)pData, sizeof(pData->motor_send_data)); 
	}
  else if(usart==1)
	{
	  pData->T = T;
		A1_modify_data(pData);
		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)pData, sizeof(pData->motor_send_data)); 
	}
}

const A1_MOTOR_recv *get_A1_motor_measure_point(uint8_t i)
{
    return &A1_get_data[i];
}


