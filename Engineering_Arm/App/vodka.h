#ifndef _VOLKA_H_
#define _VOLKA_H_

#include "stdint.h"

#define kWaveNumMax 20


struct vodka_just_float_struct_type
{
    float data1;
    float data2;
    float data3;
    float data4;
    float data5;
    float data6;
    float data7;
    float data8;
    float data9;
    float data10;
    
    int pend;
};
void date_send(void);
void vodka_JustFloat_send(int len,...);
void vodka_JustFloat_send_struct(struct vodka_just_float_struct_type *data_str);

#endif

