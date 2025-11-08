#ifndef TACTICAL_H
#define TACTICAL_H
#include "struct_typedef.h"
#include "classis_task.h"


#define ROTATE_SPEED 4000

void chassis_rc_to_control_vector_rotate(fp32 *vx_set, fp32 *vy_set,fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
