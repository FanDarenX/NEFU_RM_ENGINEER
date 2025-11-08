#ifndef FIRECONTROL_H
#define FIRECONTROL_H
#include "struct_typedef.h"
#include "gimbal_task.h"


typedef struct
{
	int8_t 	mode;
	fp32 		pitch;
	fp32 		yaw;
	fp32 		distance;
	int8_t 	number;

	fp32 pitch_last;
	fp32 yaw_last;
	fp32 distance_last;
}PC_get_data_t;
extern PC_get_data_t PC_get_data,PC_get_data_analyz; 
void PC_data_analyz(PC_get_data_t *PC_analyz,PC_get_data_t *PC);
uint8_t trajectory_analyz(fp32 *add_yaw, fp32 *add_pitch, PC_get_data_t *PC_analyz);
void PC_filter_Init(void);
#endif
