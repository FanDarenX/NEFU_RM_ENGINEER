/**
  *   使用时先定义一个kalman指针，然后调用kalmanCreate()创建一个滤波器
  *   每次读取到传感器数据后即可调用KalmanFilter()来对数据进行滤波
  *          使用示例
  *          extKalman p;                  //定义一个卡尔曼滤波器结构体
  *          float SersorData;             //需要进行滤波的数据
  *          KalmanCreate(&p,20,200);      //初始化该滤波器的Q=20 R=200参数
  *          while(1)
  *          {
  *             SersorData = sersor();                     //获取数据
  *             SersorData = KalmanFilter(&p,SersorData);  //对数据进行滤波
  *          }
  */
#include "bsp_filter.h"
extKalman_t K_accel0;
extKalman_t K_accel1;
extKalman_t K_accel2;
/********************************************************************************************************************************
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  * @retval none
  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  *		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
①Q和R都不能取0，Q取0时意味着完全不相信测量值的输入，所以系统输出值衡为初值；
R取0时意味着无条件相信测量值，所以卡尔曼滤波的输出与测量值完全重合，这两种情况下卡尔曼滤波毫无意义。
②讨论Q和R的单独取值同样意义不大，因为卡尔曼增益是受R和Q的比值影响的，
在调节参数的时候要同时比较两个协方差的大小。
③卡尔曼增益的值越大，意味着越相信测量值的输出，收敛速度越快，最优估计值震荡越明显；=============================>Q/R越大，越倾向于测量值
卡尔曼增益的值越小，意味着测量值的可信度越低，越相信系统本身的估计值，最优估计值收敛速度越慢，输出越平稳。=========>Q/R越小，越倾向于预测量
********************************************************************************************************************************/
void KalmanCreate(extKalman_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

/**
  * @name   KalmanFilter
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  *         dat:待滤波数据
  * @retval 滤波后的数据
  * @attention Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
  *            A=1 B=0 H=1 I=1  W(K)    V(k)是高斯白噪声,叠加在测量值上了,可以不用管
  *            以下是卡尔曼的5个核心公式
  *            一阶H'即为它本身,否则为转置矩阵
  */
float KalmanFilter(extKalman_t* p,float dat)
{
    p->X_mid =  p->A*p->X_last ;                //对应公式(1) x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid =  p->A*p->P_last+p->Q;            //对应公式(2) p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg =     p->P_mid/(p->P_mid+p->R);       //对应公式(4) kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now =  p->X_mid+p->kg*(dat-p->X_mid);  //对应公式(3) x(k|k) = x(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;              //对应公式(5) p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                       //状态更新
    p->X_last = p->X_now;
    return p->X_now;							  //输出预测结果x(k|k)
}
float KalmanFilter_2(extKalman_t* p,float Motor_speed,float INS_speed)
{
    p->X_mid =  p->A*p->X_last ;                //对应公式(1) x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid =  p->A*p->P_last+p->Q;            //对应公式(2) p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg =     p->P_mid/(p->P_mid+p->R);       //对应公式(4) kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now =  p->X_mid+p->kg*(Motor_speed-p->X_mid);  //对应公式(3) x(k|k) = x(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;              //对应公式(5) p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                       //状态更新
    p->X_last = Motor_speed;
    return p->X_now;							  //输出预测结果x(k|k)
}
/*******************************************************************************************
* @name   超前-滞后调节器
* @param  T1
********************************************************************************************/

void Leda_Lag_Init(Lead_Lag_t *Lead_Lag,float T1,float T2,float delta_t)
{
    Lead_Lag->T1 = T1;
    Lead_Lag->T2 = T2;
    Lead_Lag->dT = delta_t;
    Lead_Lag->x = 0;
    Lead_Lag->y = 0;
}
float Lead_Lag(Lead_Lag_t *Lead_Lag,float error)
{
    float out;
    Lead_Lag->x = (1 - Lead_Lag->dT / Lead_Lag->T2) * Lead_Lag->x + (Lead_Lag->dT / Lead_Lag->T2) * error; //u的值是其他环节给进来的
    Lead_Lag->y = (1 - Lead_Lag->T1 / Lead_Lag->T2) * Lead_Lag->x + (Lead_Lag->T1 / Lead_Lag->T2) * error;
    out = Lead_Lag->y/(Lead_Lag->T1/Lead_Lag->T2);
    return out;
}
/***************************************************************************************
*跟踪微分器TD   Trace differentiator
***************************************************************************************/
//void TD_Init(TD_t *TD,float h,float r)
//{
//	TD->h=h;//0.2
//	TD->r=r;//5
//	TD->OUT_1=0;
//	TD->OUT_2=0;
//};
//void Trace_differentiator(TD_t *TD,float error)
//{
//	  float fh;
//    fh = -TD->r * TD->r *(TD->OUT_1-error) -2 * TD->r * TD->OUT_2;
//    TD->OUT_1+=TD->OUT_2*TD->h;
//    TD->OUT_2+=fh*TD->h;
//}


/***************************************************************************************
*二维卡尔曼
***************************************************************************************/
void kalman2_init(kalman2_state *state)//, float *init_x, float (*init_p)[2]//×?oó??Dèμ÷ê?q[0],q[1];
{
    state->x[0]    = 0;
    state->x[1]    = 0;
    state->p[0][0] = 1;
    state->p[0][1] = 0;
    state->p[1][0] = 0;
    state->p[1][1] = 1;
//	  state->A       = {{1, 0.1}, {0, 1}};
    state->A[0][0] = 1;
    state->A[0][1] = 0.002f;//1;//t?é±?  á?????àíê±?ìμ?2?2ms      PID_PIT.I*0.27;//0.0027
    state->A[1][0] = 0;
    state->A[1][1] = 1;
//	  state->H       = {1,0};
    state->H[0][0] = 1;
    state->H[0][1] = 0;//1
    state->H[1][0] = 0;
    state->H[1][1] = 0;
//    state->q       = {{10e-6,0}, {0,10e-6}};  /* measure noise convariance */
    state->q[0]    = 0.01f;//5;//0.0001;//10e-7;//10e-7;
    state->q[1]    = 0.01f;//10e-6;//0.5;//0.0035;//5*10e-7;
//    state->r/* estimated error convariance */PID_ROL.D*
    state->r[0][0] = 5.0f;//10e-4;//52.4586;//0.1;//10e-3;//10e-7;
    state->r[0][1] = 0;
    state->r[1][0] = 0;
    state->r[1][1] = 1.0f;
//    state->B
    state->B[0]    = state->A[0][1]* state->A[0][1]/2.0;
    state->B[1]    = state->A[0][1];
}

float kalman2_filter(kalman2_state *state, float x_weiyi,float x_speed,float a)
{
    float temp0 = 0.0f;
    float temp1 = 0.0f;
    float temp0_0 = 0.0f;
    float temp0_1 = 0.0f;
    float temp1_0 = 0.0f;
    float temp1_1 = 0.0f;
    float temp00 = 0.0f;
    float temp01 = 0.0f;
    float temp10 = 0.0f;
    float temp11 = 0.0f;

    /* Step1: Predict X(k+1)= A*X(k) +B*U(k)*/
    state->x[0] = state->A[0][0] * state->x[0] + state->A[0][1] * state->x[1]+state->B[0]*a;//×a??íê×?±ê·??éê1ó?
    state->x[1] = state->A[1][0] * state->x[0] + state->A[1][1] * state->x[1]+state->B[1]*a;
    /* Step2: Covariance Predict P(k+1)=A*P(k)*(A^T)+Q;*/
    state->p[0][0] = (state->p[0][0]+state->p[1][0]*state->A[0][1])+(state->p[0][1]+state->p[1][1]*state->A[0][1])*state->A[0][1]+state->q[0];
    state->p[0][1] = state->p[0][1]+state->p[1][1]*state->A[0][1];//+state->q[0];
    state->p[1][0] = state->p[1][0]+state->p[1][1]*state->A[0][1];//+state->q[1];
    state->p[1][1] = state->p[1][1]+state->q[1];
    /* Step3: Gain Measurement : gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose.  μúèy??1?ê?×a??*/
    temp0_0  = (state->p[0][0] +state->r[0][0])*(state->p[1][1] +state->r[1][1])-(state->p[0][1] +state->r[0][1])*(state->p[1][0] +state->r[1][0]);//?yè·//r?????ó
    temp0_1  = (state->p[0][0] +state->r[0][0])*(state->p[1][1] +state->r[1][1])-(state->p[0][1] +state->r[0][1])*(state->p[1][0] +state->r[1][0]);
    temp1_0  = (state->p[0][0] +state->r[0][0])*(state->p[1][1] +state->r[1][1])-(state->p[0][1] +state->r[0][1])*(state->p[1][0] +state->r[1][0]);
    temp1_1  = (state->p[0][0] +state->r[0][0])*(state->p[1][1] +state->r[1][1])-(state->p[0][1] +state->r[0][1])*(state->p[1][0] +state->r[1][0]);
    temp00  =   state->p[1][1] / temp0_0;
    temp01  =  -state->p[0][1] / temp0_1;
    temp10  =  -state->p[1][0] / temp1_0;
    temp11  =   state->p[0][0] / temp1_1;
    state->gain[0][0] = state->p[0][0]* temp00 + state->p[0][1]* temp10;
    state->gain[0][1] = state->p[0][0]* temp01 + state->p[0][1]* temp11;
    state->gain[1][0] = state->p[1][0]* temp00 + state->p[1][1]* temp10;
    state->gain[1][1] = state->p[1][0]* temp01 + state->p[1][1]* temp11;
    /* Step4: Status Update : x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
    state->x[0] = state->x[0] + state->gain[0][0] * (x_weiyi - state->x[0])+ state->gain[0][1] * (x_speed - state->x[1]); //?ao???ó?ò???2?êy
    state->x[1] = state->x[1] + state->gain[1][0] * (x_weiyi - state->x[0])+ state->gain[1][1] * (x_speed - state->x[1]);
    /* Step5: Covariance Update p: p(n|n) = [I - gain * H] * p(n|n-1)  ?üD?p*/
    temp0=state->p[0][0];
    temp1=state->p[0][1];
    state->p[0][0] = (1 - state->gain[0][0] ) * state->p[0][0]-(state->gain[0][1]* state->p[1][0]);
    state->p[0][1] = (1 - state->gain[0][0] ) * state->p[0][1]-(state->gain[0][1]* state->p[1][1]);
    state->p[1][0] = (1 - state->gain[1][1] ) * state->p[1][0]-state->gain[1][0]* temp0;//state->p[0][0]
    state->p[1][1] = (1 - state->gain[1][1] ) * state->p[1][1]-state->gain[1][0]* temp1;//state->p[0][1]

    return state->x[0];
}


////z轴kalman滤波初始化，初始化时用
//kalman2_init(&BaroAlt_klm);
////输入气压计高度，速度和惯性坐标下的加速度---------输出高度和速度
//kalman2_filter(&BaroAlt_klm, BaroAltoo,0,az_c);