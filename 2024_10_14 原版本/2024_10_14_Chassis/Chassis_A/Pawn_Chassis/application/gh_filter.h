#ifndef GH_FILTER_H
#define GH_FILTER_H

typedef struct
{
//    uint8_t i;
    float x0;     //初始值
    float dx;     //预测增量
    float g;      //g参数
    float h;      //h参数
    float dt;     //步长
    float x_est;  //上次值
    float x_now;
} _GH_Filter_Struct;

void g_h_fliter(float data,_GH_Filter_Struct* gh_param);

#endif
