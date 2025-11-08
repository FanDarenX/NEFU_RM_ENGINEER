#ifndef DECET_H
#define DECET_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "user_lib.h"

typedef struct
{
	uint16_t bmi_count;
	uint16_t bmi_frequency;
	uint16_t chassis_count;
	uint16_t chassis_frequency;
	uint16_t gimbal_count;
	uint16_t gimbal_frequency;
	uint16_t remote_count;
	uint16_t remote_frequency;
  uint16_t UI_count;
	uint16_t UI_frequency;
	uint16_t PC_count;
	uint16_t PC_frequency;
	uint16_t PC_send_count;
  uint16_t PC_send_frequency;
  uint16_t usb_send_count;
  uint16_t usb_send_frequency;
	uint16_t motor_count[10];
	uint16_t motor_frequency[10];
	
	uint8_t motor_state[10];
}decet_flag_t;

extern decet_flag_t decet_flag;

void Hz_detection_task(void);

#endif
