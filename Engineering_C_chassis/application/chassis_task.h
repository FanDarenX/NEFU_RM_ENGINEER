#ifndef __CHASSIS_H_
#define __CHASSIS_H_

#include "main.h"
#include "pid.h"
#include "user_lib.h"

typedef struct
{
	float chassis_speed_x;
	float chassis_speed_y;
	float chassis_speed_r;

} chassis_speed_rel;

typedef struct
{
	chassis_speed_rel chassis_speed;
	ramp_function_source_t chassis_ramp[3]; // 2
	first_order_filter_type_t chassis_first_order_z;
	first_order_filter_type_t chassis_first_order_x;
	first_order_filter_type_t chassis_first_order_y;
} chassis_task_t;

extern chassis_task_t chassis_speed_task;

typedef struct
{
	float chassis_M1;
	float chassis_M2;
	float chassis_M3;
	float chassis_M4;

} Motor_speed;

extern Motor_speed Speed_set;
extern uint8_t speed_k;
extern uint8_t remote_mode;

void Chassis_Task_os(void);
void chassis_init(void);

#endif
