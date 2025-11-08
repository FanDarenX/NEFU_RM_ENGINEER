#include "decet_task.h"
decet_flag_t decet_flag;
void Hz_detection_task()
{
    decet_flag.bmi_frequency=decet_flag.bmi_count;
    decet_flag.bmi_count=0;

    decet_flag.chassis_frequency=decet_flag.chassis_count;
    decet_flag.chassis_count=0;

    decet_flag.gimbal_frequency=decet_flag.gimbal_count;
    decet_flag.gimbal_count=0;

    decet_flag.remote_frequency=decet_flag.remote_count;
    decet_flag.remote_count=0;

    decet_flag.UI_frequency=decet_flag.UI_count;
    decet_flag.UI_count=0;

    decet_flag.PC_frequency=decet_flag.PC_count;
    decet_flag.PC_count=0;

    decet_flag.PC_send_frequency=decet_flag.PC_send_count;
    decet_flag.PC_send_count=0;

    decet_flag.usb_send_frequency=decet_flag.usb_send_count;
    decet_flag.usb_send_count=0;

    for(int i=0; i<10; i++)
    {
        decet_flag.motor_frequency[i]=decet_flag.motor_count[i];
        decet_flag.motor_count[i]=0;

        if(decet_flag.motor_frequency[i]==0)
            decet_flag.motor_state[i]=0;
        else
            decet_flag.motor_state[i]=1;
    }

}