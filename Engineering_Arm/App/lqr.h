#ifndef __LQR_H
#define __LQR_H

#include "struct_typedef.h"

int16_t gimbal_LQR_calc(fp32 K1,fp32 K2,fp32 K0,fp32 angle_get,fp32 speed_get,fp32 angle_set,fp32 speed_set);

#endif

