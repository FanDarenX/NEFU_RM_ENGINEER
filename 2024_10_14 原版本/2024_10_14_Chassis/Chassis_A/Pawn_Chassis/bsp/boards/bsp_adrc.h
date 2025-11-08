#ifndef _BSP_ADRC
#define _BSP_ADRC
typedef struct
{
    float r ;//快速跟踪因子
    float h ;//滤波因子,系统调用步长
    /**************ESO**********/
    float b,      //系统系数
          delta,  //delta为fal（e，alpha，delta）函数的线性区间宽度
          belta01, //扩张状态观测器反馈增益1
          belta02, //扩张状态观测器反馈增益2
          belta03 ;//扩张状态观测器反馈增益3
    /**************NLSEF*******/
    float alpha1, //
          alpha2, //
          belta1, //跟踪输入信号增益
          belta2 ;//跟踪微分信号增益
//中间变量区，不需要用户管理以及赋值
    /****************TD*******************/
    float x1, //跟踪输入
          x2, //跟踪输入的微分
          /****************ESO******************/
          e,  //误差
          z1, //跟踪反馈值
          z2, //跟踪反馈值的而微分
          z3,//跟踪系统的扰动（总扰动）
          /**************NLSEF******************/
          u ;//输出值
} ADRC_t;
void ADRC_Init(ADRC_t *ADRC,
               float r,float h,//TD
               float b,float delta,float belta01,float belta02,float belta03,//ESO
               float alpha1,float alpha2,float belta1,float belta2);//NLSEF
float ADRC(ADRC_t *ADRC,float v,float y);  //  参数：v：输入的目标值；  y：反馈值
#endif
