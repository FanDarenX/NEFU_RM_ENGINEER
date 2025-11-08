#ifndef BSP_MATH_H
#define BSP_MATH_H
#include "struct_typedef.h"
//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
