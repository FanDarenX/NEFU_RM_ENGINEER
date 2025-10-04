/**
 *   ʹ��ʱ�ȶ���һ��kalmanָ�룬Ȼ�����kalmanCreate()����һ���˲���
 *   ÿ�ζ�ȡ�����������ݺ󼴿ɵ���KalmanFilter()�������ݽ����˲�
 *          ʹ��ʾ��
 *          extKalman p;                  //����һ���������˲����ṹ��
 *          float SersorData;             //��Ҫ�����˲�������
 *          KalmanCreate(&p,20,200);      //��ʼ�����˲�����Q=20 R=200����
 *          while(1)
 *          {
 *             SersorData = sersor();                     //��ȡ����
 *             SersorData = KalmanFilter(&p,SersorData);  //�����ݽ����˲�
 *          }
 */
#include "bsp_filter.h"
extKalman_t Kalm_go;
/********************************************************************************************************************************
  * @name   kalmanCreate
  * @brief  ����һ���������˲���
  * @param  p:  �˲���
  *         T_Q:ϵͳ����Э����
  *         T_R:��������Э����
  * @retval none
  * @attention R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
  *		       	��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
��Q��R������ȡ0��Qȡ0ʱ��ζ����ȫ�����Ų���ֵ�����룬����ϵͳ���ֵ��Ϊ��ֵ��
Rȡ0ʱ��ζ�����������Ų���ֵ�����Կ������˲�����������ֵ��ȫ�غϣ�����������¿������˲��������塣
������Q��R�ĵ���ȡֵͬ�����岻����Ϊ��������������R��Q�ı�ֵӰ��ģ�
�ڵ��ڲ�����ʱ��Ҫͬʱ�Ƚ�����Э����Ĵ�С��
�ۿ����������ֵԽ����ζ��Խ���Ų���ֵ������������ٶ�Խ�죬���Ź���ֵ��Խ���ԣ�=============================>Q/RԽ��Խ�����ڲ���ֵ
�����������ֵԽС����ζ�Ų���ֵ�Ŀ��Ŷ�Խ�ͣ�Խ����ϵͳ����Ĺ���ֵ�����Ź���ֵ�����ٶ�Խ�������Խƽ�ȡ�=========>Q/RԽС��Խ������Ԥ����
********************************************************************************************************************************/
void KalmanCreate(extKalman_t *p, float T_Q, float T_R)
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
 * @brief  �������˲���
 * @param  p:  �˲���
 *         dat:���˲�����
 * @retval �˲��������
 * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲����ֵ,���������
 *            A=1 B=0 H=1 I=1  W(K)    V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
 *            �����ǿ�������5�����Ĺ�ʽ
 *            һ��H'��Ϊ������,����Ϊת�þ���
 */
float KalmanFilter(extKalman_t *p, float dat)
{
  p->X_mid = p->A * p->X_last;                    // ��Ӧ��ʽ(1) x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
  p->P_mid = p->A * p->P_last + p->Q;             // ��Ӧ��ʽ(2) p(k|k-1) = A*p(k-1|k-1)*A'+Q
  p->kg = p->P_mid / (p->P_mid + p->R);           // ��Ӧ��ʽ(4) kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  p->X_now = p->X_mid + p->kg * (dat - p->X_mid); // ��Ӧ��ʽ(3) x(k|k) = x(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  p->P_now = (1 - p->kg) * p->P_mid;              // ��Ӧ��ʽ(5) p(k|k) = (I-kg(k)*H)*P(k|k-1)
  p->P_last = p->P_now;                           // ״̬����
  p->X_last = p->X_now;
  return p->X_now; // ���Ԥ����x(k|k)
}
float KalmanFilter_2(extKalman_t *p, float Motor_speed, float INS_speed)
{
  p->X_mid = p->A * p->X_last;                            // ��Ӧ��ʽ(1) x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
  p->P_mid = p->A * p->P_last + p->Q;                     // ��Ӧ��ʽ(2) p(k|k-1) = A*p(k-1|k-1)*A'+Q
  p->kg = p->P_mid / (p->P_mid + p->R);                   // ��Ӧ��ʽ(4) kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  p->X_now = p->X_mid + p->kg * (Motor_speed - p->X_mid); // ��Ӧ��ʽ(3) x(k|k) = x(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  p->P_now = (1 - p->kg) * p->P_mid;                      // ��Ӧ��ʽ(5) p(k|k) = (I-kg(k)*H)*P(k|k-1)
  p->P_last = p->P_now;                                   // ״̬����
  p->X_last = Motor_speed;
  return p->X_now; // ���Ԥ����x(k|k)
}
/*******************************************************************************************
 * @name   ��ǰ-�ͺ������
 * @param  T1
 ********************************************************************************************/

void Leda_Lag_Init(Lead_Lag_t *Lead_Lag, float T1, float T2, float delta_t)
{
  Lead_Lag->T1 = T1;
  Lead_Lag->T2 = T2;
  Lead_Lag->dT = delta_t;
  Lead_Lag->x = 0;
  Lead_Lag->y = 0;
}
float Lead_Lag(Lead_Lag_t *Lead_Lag, float error)
{
  float out;
  Lead_Lag->x = (1 - Lead_Lag->dT / Lead_Lag->T2) * Lead_Lag->x + (Lead_Lag->dT / Lead_Lag->T2) * error; // u��ֵ���������ڸ�������
  Lead_Lag->y = (1 - Lead_Lag->T1 / Lead_Lag->T2) * Lead_Lag->x + (Lead_Lag->T1 / Lead_Lag->T2) * error;
  out = Lead_Lag->y / (Lead_Lag->T1 / Lead_Lag->T2);
  return out;
}
/***************************************************************************************
 *����΢����TD   Trace differentiator
 ***************************************************************************************/
// void TD_Init(TD_t *TD,float h,float r)
//{
//	TD->h=h;//0.2
//	TD->r=r;//5
//	TD->OUT_1=0;
//	TD->OUT_2=0;
// };
// void Trace_differentiator(TD_t *TD,float error)
//{
//	  float fh;
//     fh = -TD->r * TD->r *(TD->OUT_1-error) -2 * TD->r * TD->OUT_2;
//     TD->OUT_1+=TD->OUT_2*TD->h;
//     TD->OUT_2+=fh*TD->h;
// }

/***************************************************************************************
 *��ά������
 ***************************************************************************************/
void kalman2_init(kalman2_state *state) //, float *init_x, float (*init_p)[2]//��?o��??D���̡¨�?q[0],q[1];
{
  state->x[0] = 0;
  state->x[1] = 0;
  state->p[0][0] = 1;
  state->p[0][1] = 0;
  state->p[1][0] = 0;
  state->p[1][1] = 1;
  //	  state->A       = {{1, 0.1}, {0, 1}};
  state->A[0][0] = 1;
  state->A[0][1] = 0.002f; // 1;//t?����?  ��?????��������?����?2?2ms      PID_PIT.I*0.27;//0.0027
  state->A[1][0] = 0;
  state->A[1][1] = 1;
  //	  state->H       = {1,0};
  state->H[0][0] = 1;
  state->H[0][1] = 0; // 1
  state->H[1][0] = 0;
  state->H[1][1] = 0;
  //    state->q       = {{10e-6,0}, {0,10e-6}};  /* measure noise convariance */
  state->q[0] = 0.01f; // 5;//0.0001;//10e-7;//10e-7;
  state->q[1] = 0.01f; // 10e-6;//0.5;//0.0035;//5*10e-7;
  //    state->r/* estimated error convariance */PID_ROL.D*
  state->r[0][0] = 5.0f; // 10e-4;//52.4586;//0.1;//10e-3;//10e-7;
  state->r[0][1] = 0;
  state->r[1][0] = 0;
  state->r[1][1] = 1.0f;
  //    state->B
  state->B[0] = state->A[0][1] * state->A[0][1] / 2.0;
  state->B[1] = state->A[0][1];
}

float kalman2_filter(kalman2_state *state, float x_weiyi, float x_speed, float a)
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
  state->x[0] = state->A[0][0] * state->x[0] + state->A[0][1] * state->x[1] + state->B[0] * a; // ��a??������?������??����1��?
  state->x[1] = state->A[1][0] * state->x[0] + state->A[1][1] * state->x[1] + state->B[1] * a;
  /* Step2: Covariance Predict P(k+1)=A*P(k)*(A^T)+Q;*/
  state->p[0][0] = (state->p[0][0] + state->p[1][0] * state->A[0][1]) + (state->p[0][1] + state->p[1][1] * state->A[0][1]) * state->A[0][1] + state->q[0];
  state->p[0][1] = state->p[0][1] + state->p[1][1] * state->A[0][1]; //+state->q[0];
  state->p[1][0] = state->p[1][0] + state->p[1][1] * state->A[0][1]; //+state->q[1];
  state->p[1][1] = state->p[1][1] + state->q[1];
  /* Step3: Gain Measurement : gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose.  �̨���y??1?��?��a??*/
  temp0_0 = (state->p[0][0] + state->r[0][0]) * (state->p[1][1] + state->r[1][1]) - (state->p[0][1] + state->r[0][1]) * (state->p[1][0] + state->r[1][0]); //?y����//r?????��
  temp0_1 = (state->p[0][0] + state->r[0][0]) * (state->p[1][1] + state->r[1][1]) - (state->p[0][1] + state->r[0][1]) * (state->p[1][0] + state->r[1][0]);
  temp1_0 = (state->p[0][0] + state->r[0][0]) * (state->p[1][1] + state->r[1][1]) - (state->p[0][1] + state->r[0][1]) * (state->p[1][0] + state->r[1][0]);
  temp1_1 = (state->p[0][0] + state->r[0][0]) * (state->p[1][1] + state->r[1][1]) - (state->p[0][1] + state->r[0][1]) * (state->p[1][0] + state->r[1][0]);
  temp00 = state->p[1][1] / temp0_0;
  temp01 = -state->p[0][1] / temp0_1;
  temp10 = -state->p[1][0] / temp1_0;
  temp11 = state->p[0][0] / temp1_1;
  state->gain[0][0] = state->p[0][0] * temp00 + state->p[0][1] * temp10;
  state->gain[0][1] = state->p[0][0] * temp01 + state->p[0][1] * temp11;
  state->gain[1][0] = state->p[1][0] * temp00 + state->p[1][1] * temp10;
  state->gain[1][1] = state->p[1][0] * temp01 + state->p[1][1] * temp11;
  /* Step4: Status Update : x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
  state->x[0] = state->x[0] + state->gain[0][0] * (x_weiyi - state->x[0]) + state->gain[0][1] * (x_speed - state->x[1]); //?ao???��?��???2?��y
  state->x[1] = state->x[1] + state->gain[1][0] * (x_weiyi - state->x[0]) + state->gain[1][1] * (x_speed - state->x[1]);
  /* Step5: Covariance Update p: p(n|n) = [I - gain * H] * p(n|n-1)  ?��D?p*/
  temp0 = state->p[0][0];
  temp1 = state->p[0][1];
  state->p[0][0] = (1 - state->gain[0][0]) * state->p[0][0] - (state->gain[0][1] * state->p[1][0]);
  state->p[0][1] = (1 - state->gain[0][0]) * state->p[0][1] - (state->gain[0][1] * state->p[1][1]);
  state->p[1][0] = (1 - state->gain[1][1]) * state->p[1][0] - state->gain[1][0] * temp0; // state->p[0][0]
  state->p[1][1] = (1 - state->gain[1][1]) * state->p[1][1] - state->gain[1][0] * temp1; // state->p[0][1]

  return state->x[0];
}

////z��kalman�˲���ʼ������ʼ��ʱ��
// kalman2_init(&BaroAlt_klm);
////������ѹ�Ƹ߶ȣ��ٶȺ͹��������µļ��ٶ�---------����߶Ⱥ��ٶ�
// kalman2_filter(&BaroAlt_klm, BaroAltoo,0,az_c);