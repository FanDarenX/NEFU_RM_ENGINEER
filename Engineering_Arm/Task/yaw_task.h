#ifndef YAW_TASK
#define YAW_TASK

#include "pid.h"
#include "remote_control.h"

#include "position_task.h"


typedef struct
{
	float xx1;
	float xx2;
	float fh1;
	float r;
	float h1;
	float h2;
} TD;

extern TD Cup_pitch_td;
extern TD Cup_roll_td;
extern TD yaw_td;
extern TD pic_td;
extern TD Uplift_td;
extern TD Y_td;
extern TD X_td;

void TD_init(TD *TT, float r, float h1, float h2);
void TD_INIT(void);
void TD_task(void);
void Tracking_Differentiator(TD *TT, float target, float r, float h1, float h2);
#endif
