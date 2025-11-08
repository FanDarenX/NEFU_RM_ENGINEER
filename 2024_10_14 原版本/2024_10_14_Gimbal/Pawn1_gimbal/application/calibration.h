#ifndef CALIBRATION_H
#define CALIBRATION_H
#include "struct_typedef.h"
#include "main.h"
#include "gimbal_task.h"
typedef struct
{
    uint8_t gravity_flag;
	  uint8_t gyro_flag ;
} cailbration_flag_t;
#define GYRO_DATA_LENGHT 6
extern cailbration_flag_t cailbration_flag;
extern uint8_t gyro_off_read_data[GYRO_DATA_LENGHT];
extern uint8_t gyro_off_write_data[GYRO_DATA_LENGHT];
extern void Calibration_Init(cailbration_flag_t *cailbration_flag);
extern void Gravity_compensation(gimbal_control_t *gimbal);
extern void Dritt_compensation(gimbal_control_t *gimbal);
extern uint8_t Debug_switch(cailbration_flag_t *cailbration_flag);
#endif
