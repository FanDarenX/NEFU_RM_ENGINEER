#ifndef IMAGE_TASK_H
#define IMAGE_TASK_H

#include "pid.h"
#include "remote_control.h"
#include "yaw_task.h"
#include "keyscan_task.h"

typedef struct
{
	const key_typedef_t *img_key_p;
	const RC_ctrl_t *image_rc;
	int16_t current_yaw_angle;
	int16_t current_pitch_angle;
}img_task_t;


//extern uint8_t image_mode;

enum image_mode_enum
{
	image_chassis=0,
	image_little,
	image_big,
	image_exchange,
	image_mouse,
	image_store,
};
extern enum image_mode_enum image_mode;
//extern image_mode_enum image_mode;
extern img_task_t image_task;

void image_task_init(img_task_t *image_init);
void yaw_servo_task(int16_t angle);
void pitch_servo_task(int16_t angle);
void image_mode_task(void);
#endif
