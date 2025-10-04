#ifndef _XY_H
#define _XY_H

#include "CAN_receive.h"

#define Y_safe 8192 * Y_safe_multiple

extern float X_target;
extern float Y_target;

void XY_pid_init(void);
void XY_task();

#endif