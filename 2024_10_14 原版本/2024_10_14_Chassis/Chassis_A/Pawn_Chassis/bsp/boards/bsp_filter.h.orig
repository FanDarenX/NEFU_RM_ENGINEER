/**
  * @author  Liu heng
  * 卡尔曼滤波器来自RoboMaster论坛  
  */
  
#ifndef _BSP_FILTTER
#define _BSP_FILTTER


typedef struct {
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
	  float B;
    float Q;
    float R;
    float H;
}extKalman_t;

typedef struct {
    float T1; //上一时刻的最优结果  X(k-|k-1)
    float T2;  //当前时刻的预测结果  X(k|k-1)
    float dT;  //当前时刻的最优结果  X(k|k)
		float x;
		float y;
}Lead_Lag_t;

//typedef struct {
//    float r;
//    float h;  
//    float OUT_1; 
//		float OUT_2;
//}TD_t;

typedef struct {
      float x[2];     /* state: [0]-angle [1]-diffrence of angle, 2x1 */
      float A[2][2];  /* X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2 */
      float H[2][2];     /* Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2   */
      float q[2];     /* process(predict) noise convariance,2x1 [q0,0; 0,q1] */
      float r[2][2];        /* measure noise convariance */
      float p[2][2];  /* estimated error convariance,2x2 [p0 p1; p2 p3] */
      float gain[2][2];  /* 2x1 */
	  float B[2];
} kalman2_state;   

extern float Attitude_Angel_X;
extern extKalman_t K_accel0;
extern extKalman_t K_accel1;
extern extKalman_t K_accel2;
void KalmanCreate(extKalman_t *p,float T_Q,float T_R);
float KalmanFilter(extKalman_t* p,float dat);
void Leda_Lag_Init(Lead_Lag_t *Lead_Lag,float T1,float T2,float delta_t);
float Lead_Lag(Lead_Lag_t *Lead_Lag,float error);
//void TD_Init(TD_t *TD,float h,float r);
//void Trace_differentiator(TD_t *TD,float error);
void kalman2_init(kalman2_state *state);
float kalman2_filter(kalman2_state *state, float x_weiyi,float x_speed,float a);

float KalmanFilter_2(extKalman_t* p,float Motor_speed,float INS_speed);
#endif
