#include "calibration.h"
#include "gimbal_task.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "stack_macros.h"
#include "bsp_flash.h"
//#define ABS(x) (((x) > 0) ? (x) : (-(x)))

uint8_t gyro_off_read_data[GYRO_DATA_LENGHT]= {0};
uint8_t gyro_off_write_data[GYRO_DATA_LENGHT]= {0};
cailbration_flag_t cailbration_flag;

void Calibration_Init(cailbration_flag_t *cailbration_flag)
{
    cailbration_flag->gravity_flag=0;
    cailbration_flag->gyro_flag=0;
}

uint8_t Debug_switch(cailbration_flag_t *cailbration_flag)
{
    uint8_t flag;
    if(cailbration_flag->gravity_flag!=0)     return flag=1;
    else if(cailbration_flag->gyro_flag!=0)   return flag=2;

    else                                      return flag=0;
}
