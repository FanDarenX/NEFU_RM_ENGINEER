#include "gimbal_task.h"
#include "CAN_receive.h"
#include "pid.h"
#include "main.h"
#include "bsp_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "ins_task.h"
#include "shoot.h"
#include "user_lib.h"
#include "decet_task.h"
#include "bsp_filter.h"
#include "arm_math.h"
#include "bsp_usart.h"
#include "bsp_adrc.h"
#include "referee.h"
#include "tim.h"
#include "bsp_flash.h"
#include "VMC.h"
#include "Model_Com.h"
#include "motor_control.h"
#include "stdio.h"
#include "A1_control.h"
#include "kf.h"
#include "gh_filter.h"

/************************************************************************************
		 _/_/_/  _/    _/    _/_/      _/_/_/    _/_/_/  _/_/_/    _/_/_/
  _/        _/    _/  _/    _/  _/        _/          _/    _/
 _/        _/_/_/_/  _/_/_/_/    _/_/      _/_/      _/      _/_/
_/        _/    _/  _/    _/        _/        _/    _/          _/
 _/_/_/  _/    _/  _/    _/  _/_/_/    _/_/_/    _/_/_/  _/_/_/
 ***********************************************************************************/
//电机编码值规整 0—8191
#define ABS(x) (((x) > 0) ? (x) : (-(x)))
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
#define LimitMax(input, max)   \
{                          \
		if (input > max)       \
		{                      \
				input = max;       \
		}                      \
		else if (input < -max) \
		{                      \
				input = -max;      \
		}                      \
}
#define LimitPNMax(input, Pmax, Nmax)   \
{                          \
		if (input > Pmax)       \
		{                      \
				input = Pmax;       \
		}                      \
		else if (input < Nmax) \
		{                      \
				input = Nmax;      \
		}                      \
}
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif
int8_t Pawn_mode;
//float buff1[2000];
//float buff2[2000];
//float buff3[2000];
/***************************************************************************************/
//云台控制所有相关数据
extKalman_t  K_remo,K_yaw_Auto,K_pitch_Auto;
chassis_control_t chassis_control;
sup_cup_t         sup_cap;
uint8_t ce_flag = 0;
/*********************************************************************************/
static chassis_behaviour_e  chassis_behaviour      = CHASSIS_ZERO_FORCE;//行为层模式设置
static chassis_behaviour_e  chassis_behaviour_last = CHASSIS_ZERO_FORCE;//行为层模式设置
/*********************************************************************************/
//任务中四个主要内容
static void Chassis_Behavior(chassis_control_t *chassis_mode_set);
static void Chassis_mode_change_control_transit(chassis_control_t *chassis_mode_change);
static void chassis_feedback_update(chassis_control_t *feedback_update);
static void chassis_banlance_control(chassis_control_t *banlance_control);
/*********************************************************************************/
//电机模式
static void A1_motor_init(void);
static void A1_motor_sport_set(void);
/*********************************************************************************/
//用到的函数,结构体和变量
fp32 Velocity_ramp(float RC_filter,float XMAX);
fp32 power_control(chassis_control_t *banlance_control);
static fp32 yaw_ecd_to_angle_change(uint16_t ecd);
void K_get_absolute_speed(chassis_control_t *banlance_control);
fp32 Angle_conversion(fp32 Q1){return Q1=(Q1>PI?(Q1-2*PI):(Q1<-PI?(2*PI+Q1):Q1));}
fp32 Speed_ramp(fp32 speed_set,fp32 foot_speed);
first_order_filter_type_t rc_filter;
first_order_filter_type_t tl_run_filter;
first_order_filter_type_t tl_filter;
first_order_filter_type_t power_limit_filter;
Ordinary_Least_Squares_t OLS_S0_L;
Ordinary_Least_Squares_t OLS_S0_R;
Ordinary_Least_Squares_t OLS_angle_L;
Ordinary_Least_Squares_t OLS_angle_R;
KalmanFilter_t distance_KF;
static fp32 tl_speed;
short link_scope_variable=0;
/*********************************************************************************/
//LQR参数和PID参数
//fp32 balance_LQR[2][6] = {  42.55f, 4.42f, 42.28f, 25.69f, 47.93f, 11.71f,
//                            4500,	500,  2000,   1800,  35000,	2000//4500,	500,  1800,   1800,  35000,	2000  //4500,	500,  1500,   1500,  35000,	2000
//                          }; 
fp32 balance_LQR[2][6] = {	45.66f,4.89f,36.04f,23.97f,40.23f,10.11f,//48.66f,4.89f,43.04f,   26.97f,   40.23f,  14.11f,//45.21f,    5.81f,   43.11f,   26.56f,   39.17f,  13.78f, //
                            4500,	600,  1800,   1800,  35000,	2000//4500,	500,  1800,   1800,  35000,	2000  //4500,	500,  1500,   1500,  35000,	2000
                          }; //1 48.0964    4.8271   42.9013   26.8132   41.7997  -13.7276
fp32 balance_LQR_sit[2][6] = {	36.57f,    3.43f,   37.52f,   21.04f,   48.83,   -3.12, //27.57f,    2.43f,   37.52f,   21.04f,   78.83,   -3.12,
                            0,	0,  5000,  2200,	  40000,	2200 //35000,	2000,  5000,  2200,	  0,	0 
                          }; //2  
fp32 balance_LQR_stand[2][6] = { 56.60, 4.75,  39.76,   22.48,  30.80, 18.03,
                            5000,	500,  1500,   1500,  35000,	2200//4500,	500,  1600,   1500,  35000,	2000
                          }; 	
fp32 balance_LQR_CE[2][6] = {	45.21f,    4.81f,   43.11f,   26.56f,   39.17f,  13.78f, //41.31f,3.26f,	30.02f,	18.53f,	41.82f,	8.21f,
                            4500,	500,  1600,   1500,  35000,	2000
                          };  
//fp32 balance_LQR_CE[2][6] = {28.01f,4.91f,	22.11f,	17.69f,	120.33f,	3.76f,  //Tk
//                             6000,	1000,  800,   800,  60000,	2000
//                            }; 
fp32 balance_LQR_tl[2][6] = {	45.21f,  4.81f,   43.04f,   26.56f,   39.17f,  13.78f, // 45.21f,  4.81f,   0.0f,   26.56f,   39.17f,  13.78f,//41.31f,3.26f,	30.02f,	18.53f,	41.82f,	8.21f,
                            4500,	500,   0,   1500,  35000,	2000
                          }; 
//fp32 balance_LQR_sit[2][6] = {	39.10 ,3.62, 42.23, 25.21, 48.10, 10.80,
//                                4500,	500,  1600,   1500,  35000,	2000
//                             };//裁判系统之前 
//fp32 balance_LQR_sit[2][6] = {	43.04f,    3.74f,   42.31f,   25.77f,   47.71f,  11.51f,
//                                4500,	500,  1600,   1500,  35000,	2000
//                             }; 													
fp32 roll_PD[2]= {300,16}; //260 10 							//	 500,30															
fp32 stand_PD[2]= {1480,20.18};//1480 20.18						//  1500,100
fp32 Jump_PD[2]= {3600,50};				     //  1500,100
fp32 yaw_PD[2]= {1500,500};							//	1500,500											
fp32 coordinate_PD[2]= {30.0f,10.0f};		// 30.0f,10.0f												

void gimbal_task(void const * argument)
{
    /*****************开始等待一段时间****************/
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    /****************初始化**************************/
		SPORT_KF_INIT();
	  gh_Zaccel_init();
	  Chassis_Int(&chassis_control);
		if(chassis_control.unterleib_motor1.motor_measure->Pos==0 || chassis_control.unterleib_motor2.motor_measure->Pos==0 
			|| chassis_control.unterleib_motor3.motor_measure->Pos==0 || chassis_control.unterleib_motor4.motor_measure->Pos==0){
					Chassis_Int(&chassis_control);
				 }
    /****************************************/
    while (1)
    {//2ms 20s10000次
			
        if(chassis_control.stand_ready==2)
        {
            Chassis_Behavior(&chassis_control);                           //确定底盘模式                    |4      1| 
            Chassis_mode_change_control_transit(&chassis_control);        //控制模式切换 控制数据过渡       |        |
            chassis_feedback_update(&chassis_control);                    //传感器数据采集                  |        | 
            chassis_banlance_control(&chassis_control); 						      //平衡状态计算                    |3      2|

						if(chassis_behaviour==CHASSIS_SIT){
							if(chassis_control.chassis_balance.save_mode==2){
								A1_SERVO_Send_recv(&A1_send_data[0],6,-4);
								A1_SERVO_Send_recv(&A1_send_data[2],1,-4);
								vTaskDelay(1);
								A1_SERVO_Send_recv(&A1_send_data[1],6,4);	//将控制指令发送给电机，同时接收返回值	
								A1_SERVO_Send_recv(&A1_send_data[3],1,4); //将控制指令发送给电机，同时接收返回值	
								CAN_cmd_chassis_read(0x03,0);
								CAN_cmd_chassis_read(0x01,0); 								
							}
							else{
								A1_SERVO_Send_recv(&A1_send_data[0],6,-2);
								A1_SERVO_Send_recv(&A1_send_data[2],1,-2);
								vTaskDelay(1);
								A1_SERVO_Send_recv(&A1_send_data[1],6,2);	//将控制指令发送给电机，同时接收返回值
								A1_SERVO_Send_recv(&A1_send_data[3],1,2); //将控制指令发送给电机，同时接收返回值	
								CAN_cmd_chassis_read(0x03,chassis_control.foot_L_motor.torque_out);
								CAN_cmd_chassis_read(0x01,chassis_control.foot_R_motor.torque_out);  								
							}												
						}
						else{
								A1_SERVO_Send_recv(&A1_send_data[0],6,chassis_control.unterleib_motor1.torque_out);
								A1_SERVO_Send_recv(&A1_send_data[2],1,chassis_control.unterleib_motor3.torque_out);
								vTaskDelay(1);
								A1_SERVO_Send_recv(&A1_send_data[1],6,chassis_control.unterleib_motor2.torque_out);	//将控制指令发送给电机，同时接收返回值
								A1_SERVO_Send_recv(&A1_send_data[3],1,chassis_control.unterleib_motor4.torque_out); //将控制指令发送给电机，同时接收返回值	
								CAN_cmd_chassis_read(0x03,chassis_control.foot_L_motor.torque_out);
								CAN_cmd_chassis_read(0x01,chassis_control.foot_R_motor.torque_out);								
						}
					
//            A1_SERVO_Send_recv(&A1_send_data[0],6,0);//右前，正值起立
//            A1_SERVO_Send_recv(&A1_send_data[2],1,0);//左后，正值起立
//            vTaskDelay(1);
//            A1_SERVO_Send_recv(&A1_send_data[1],6,0);	//右后，负值起立
//            A1_SERVO_Send_recv(&A1_send_data[3],1,0); //左前，负值起立
//            CAN_cmd_chassis_read(0x03,0);						  //左驱动轮，正值向前
//            CAN_cmd_chassis_read(0x01,0);					    //右驱动轮，正值向后
        }
        else if(chassis_control.stand_ready==1)
        {
            Chassis_Behavior(&chassis_control);                           //确定底盘模式                    |4      1| 
            Chassis_mode_change_control_transit(&chassis_control);        //控制模式切换 控制数据过渡       |        |
            chassis_feedback_update(&chassis_control);                    //传感器数据采集                  |        | 
            chassis_banlance_control(&chassis_control); 						      //平衡状态计算                    |3      2|

						A1_SERVO_Send_recv(&A1_send_data[0],6,-2);
						A1_SERVO_Send_recv(&A1_send_data[2],1,-2);
						vTaskDelay(1);
						A1_SERVO_Send_recv(&A1_send_data[1],6,2);	//将控制指令发送给电机，同时接收返回值
						A1_SERVO_Send_recv(&A1_send_data[3],1,2); //将控制指令发送给电机，同时接收返回值						
            CAN_cmd_chassis_read(0x03,chassis_control.foot_L_motor.torque_out);
            CAN_cmd_chassis_read(0x01,chassis_control.foot_R_motor.torque_out);     
        }				
        else
        {
            Chassis_Behavior(&chassis_control);                         //确定底盘模式
            Chassis_mode_change_control_transit(&chassis_control);      //控制模式切换 控制数据过渡
            chassis_feedback_update(&chassis_control);                  //传感器数据采集
            chassis_banlance_control(&chassis_control); 						    //平衡状态计算

            A1_SERVO_Send_recv(&A1_send_data[0],6,0);
            A1_SERVO_Send_recv(&A1_send_data[2],1,0);
            vTaskDelay(1);
            A1_SERVO_Send_recv(&A1_send_data[1],6,0);	//将控制指令发送给电机，同时接收返回值
            A1_SERVO_Send_recv(&A1_send_data[3],1,0); //将控制指令发送给电机，同时接收返回值
            CAN_cmd_chassis_read(0x03,chassis_control.foot_L_motor.torque_out);//chassis_control.foot_L_motor.torque_out
            CAN_cmd_chassis_read(0x01,chassis_control.foot_R_motor.torque_out);  
//					  CAN_cmd_chassis_read(0x03,0);						  //左驱动轮，正值向前
//            CAN_cmd_chassis_read(0x01,0);					    //右驱动轮，正值向后
        }				


#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/****************************************************************************************************/
/*********************************************************************************/
/*                               底盘初始化                                       */
/*********************************************************************************/
void Chassis_Int(chassis_control_t *init)
{
    const static fp32 rc_filt_numb[1] = {0.15f};
    const static fp32 tl_numb[1] = {0.3f};

    //电机数据指针获取
    init->unterleib_motor1.motor_measure   = get_A1_motor_measure_point(0);
    init->unterleib_motor2.motor_measure   = get_A1_motor_measure_point(1);
    init->unterleib_motor3.motor_measure   = get_A1_motor_measure_point(2);
    init->unterleib_motor4.motor_measure   = get_A1_motor_measure_point(3);

    init->foot_L_motor.motor_measure       = get_9025motor_measure_point(1);
    init->foot_R_motor.motor_measure       = get_9025motor_measure_point(0);
		init->yaw_motor.gimbal_motor_measure   = get_6020motor_measure_point(0);
		
    //陀螺仪数据指针获取
    init->chassis_INT_angle_point = get_INS_angle_point();
    init->chassis_INT_gyro_point  = get_gyro_data_point();
    //遥控器数据指针获取
    init->chassis_rc_ctrl = get_Gimabl_control_point();
		init->rc_ctrl = get_remote_control_point();
    init->chassis_cap_ctrl = get_SuperCap_control_point();
		
    //初始化滤波器
    first_order_filter_init(&rc_filter, 0.002f, rc_filt_numb);
    first_order_filter_init(&tl_filter, 0.002f, tl_numb);
    first_order_filter_init(&tl_run_filter, 0.002f, tl_numb);
    OLS_Init(&OLS_S0_L,2);
    OLS_Init(&OLS_S0_R,2);
    OLS_Init(&OLS_angle_L,2);
    OLS_Init(&OLS_angle_R,2);
		
    //初始化电机模式
    init->unterleib_motor1.motor_mode = init->unterleib_motor1.last_motor_mode = CHASSIS_MOTOR_ZERO;
    init->unterleib_motor2.motor_mode = init->unterleib_motor2.last_motor_mode = CHASSIS_MOTOR_ZERO;
    init->unterleib_motor3.motor_mode = init->unterleib_motor3.last_motor_mode = CHASSIS_MOTOR_ZERO;
    init->unterleib_motor4.motor_mode = init->unterleib_motor4.last_motor_mode = CHASSIS_MOTOR_ZERO;
    init->foot_L_motor.motor_mode		  = init->foot_L_motor.last_motor_mode     = CHASSIS_MOTOR_ZERO;
    init->foot_R_motor.motor_mode     = init->foot_R_motor.last_motor_mode     = CHASSIS_MOTOR_ZERO;
    init->yaw_motor.gimbal_motor_mode = init->yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    /*******************宇树电机初始化****************************/
    //1.给电机发送全0数据
    A1_motor_init();
    for(uint8_t i=0; i<5; i++)
    {
        A1_SERVO_Send_recv(&A1_send_data[0],6,0);
        vTaskDelay(1);//将控制指令发送给电机，同时接收返回值
        A1_SERVO_Send_recv(&A1_send_data[2],1,0);
        vTaskDelay(1);//将控制指令发送给电机，同时接收返回值
        A1_SERVO_Send_recv(&A1_send_data[1],6,0);
        vTaskDelay(1);//将控制指令发送给电机，同时接收返回值
        A1_SERVO_Send_recv(&A1_send_data[3],1,0);
        vTaskDelay(1);//将控制指令发送给电机，同时接收返回值
    }
    //2.获取当前位置并记录，初始化关节角度为0
    init->unterleib_motor1.position_offset = init->unterleib_motor1.motor_measure->Pos;
    init->unterleib_motor2.position_offset = init->unterleib_motor2.motor_measure->Pos;
    init->unterleib_motor3.position_offset = init->unterleib_motor3.motor_measure->Pos;
    init->unterleib_motor4.position_offset = init->unterleib_motor4.motor_measure->Pos;
    //3.初始化设置相关参数
    init->unterleib_motor1.position =  init->unterleib_motor1.motor_measure->Pos - init->unterleib_motor1.position_offset - LEG_OFFEST;
    init->unterleib_motor2.position =  init->unterleib_motor2.motor_measure->Pos - init->unterleib_motor2.position_offset + LEG_OFFEST + PI;
    init->unterleib_motor3.position = -init->unterleib_motor3.motor_measure->Pos + init->unterleib_motor3.position_offset + LEG_OFFEST + PI;  //Q1
    init->unterleib_motor4.position = -init->unterleib_motor4.motor_measure->Pos + init->unterleib_motor4.position_offset - LEG_OFFEST;       //Q4

    init->unterleib_motor1.position0=init->unterleib_motor1.position;
    init->unterleib_motor2.position0=init->unterleib_motor2.position;
    init->unterleib_motor3.position0=init->unterleib_motor3.position;
    init->unterleib_motor4.position0=init->unterleib_motor4.position;

    init->unterleib_motor1.position_set = 0.0f;
    init->unterleib_motor2.position_set = 0.0f;
    init->unterleib_motor3.position_set = 0.0f;
    init->unterleib_motor4.position_set = 0.0f;

    A1_motor_sport_set();
    /****************************************************************/
    chassis_feedback_update(init);
    vTaskDelay(5);
    init->foot_L_motor.distance_offest= init->foot_L_motor.distance;//角度
    init->foot_R_motor.distance_offest= init->foot_R_motor.distance;//角度

    init->chassis_balance .High =0.11f;
}



/*************************************************************************/
/*                            底盘行为                                   */
/*************************************************************************/
static void Chassis_Behavior(chassis_control_t *chassis_mode_set)
{
    /******************************行为层*******************************************/
    static uint16_t zero_force_time=0,protect_time=0,limit_protect_time=0;
    if (chassis_mode_set == NULL)
    {
        return;
    }
    chassis_behaviour_last = chassis_behaviour;

		/**********************保护模式************************/
		if(!chassis_control.chassis_rc_ctrl->protect_flag){//protect_flag为0则开启保护
			if(chassis_mode_set->stand_ready==2 && !chassis_mode_set->chassis_balance.suspend_flag){
				if(ABS(chassis_mode_set->chassis_balance.pitch_angle)>=0.45f)
					chassis_mode_set->chassis_balance.save_mode=2,protect_time=0,limit_protect_time++;
				else if(ABS(chassis_mode_set->chassis_balance.pitch_angle)>=0.26f || ABS(chassis_mode_set->chassis_balance.pitch_gyro)>=2.0f)
					chassis_mode_set->chassis_balance.save_mode=1,protect_time=0,limit_protect_time++;
				else if(chassis_mode_set->chassis_balance.save_mode && (protect_time>=300 || limit_protect_time>=500))
					chassis_mode_set->chassis_balance.save_mode=0,protect_time=0,limit_protect_time=0;
				protect_time++;
			}		
		}
		
		if(chassis_mode_set->chassis_balance.save_mode && !chassis_mode_set->chassis_balance.suspend_flag)
		{
				chassis_behaviour = CHASSIS_SIT;
				tl_speed=0;
				zero_force_time=0;
		}
    else if (switch_is_down(chassis_mode_set->chassis_rc_ctrl->mode_R) || Gimbal_ctrl.reborn_flag)
    {
        if(zero_force_time>100)
        {
            chassis_behaviour = CHASSIS_ZERO_FORCE;
            zero_force_time=0;
        }
        else
        {
            zero_force_time++;
        }
    }
    else if (chassis_mode_set->chassis_rc_ctrl->tl_flag /* && !switch_is_up(chassis_mode_set->chassis_rc_ctrl->mode_R) */)
    {
				chassis_behaviour = CHASSIS_NO_FOLLOW;
				tl_speed=-7;
				zero_force_time=0;
    }
		else if (switch_is_mid(chassis_mode_set->chassis_rc_ctrl->mode_R) || switch_is_up(chassis_mode_set->chassis_rc_ctrl->mode_R))
    {
        if(chassis_behaviour == CHASSIS_NO_FOLLOW)
        {
            if(chassis_mode_set->yaw_motor .relative_angle <PI_2 && chassis_mode_set->yaw_motor .relative_angle >0)
            {
                tl_speed=0;
                chassis_behaviour=CHASSIS_FOLLOW_GIMBAL;//CHASSIS_FOLLOW_GIMBAL
            }
        }
        else
        {
            chassis_behaviour=CHASSIS_FOLLOW_GIMBAL;//CHASSIS_FOLLOW_GIMBAL
        }
        zero_force_time=0;
    }
    else
        chassis_behaviour = CHASSIS_ZERO_FORCE;
    /******************************move层*******************************************/
    chassis_mode_set->unterleib_motor1.last_motor_mode = chassis_mode_set->unterleib_motor1.motor_mode;
    chassis_mode_set->unterleib_motor2.last_motor_mode = chassis_mode_set->unterleib_motor2.motor_mode;
    chassis_mode_set->unterleib_motor3.last_motor_mode = chassis_mode_set->unterleib_motor3.motor_mode;
    chassis_mode_set->unterleib_motor4.last_motor_mode = chassis_mode_set->unterleib_motor4.motor_mode;
    chassis_mode_set->foot_L_motor.last_motor_mode = chassis_mode_set->foot_L_motor.motor_mode;
    chassis_mode_set->foot_R_motor.last_motor_mode = chassis_mode_set->foot_R_motor.motor_mode;
    //根据云台行为状态机设置电机状态机
    if (chassis_behaviour == CHASSIS_ZERO_FORCE)
    {
        chassis_mode_set->unterleib_motor1.motor_mode = CHASSIS_MOTOR_ZERO;
        chassis_mode_set->unterleib_motor2.motor_mode = CHASSIS_MOTOR_ZERO;
        chassis_mode_set->unterleib_motor3.motor_mode = CHASSIS_MOTOR_ZERO;
        chassis_mode_set->unterleib_motor4.motor_mode = CHASSIS_MOTOR_ZERO;
        chassis_mode_set->foot_L_motor.motor_mode		  = CHASSIS_MOTOR_ZERO;
        chassis_mode_set->foot_R_motor.motor_mode     = CHASSIS_MOTOR_ZERO;
    }	
    else if (chassis_behaviour == CHASSIS_FOLLOW_GIMBAL)
    {
        chassis_mode_set->unterleib_motor1.motor_mode = CHASSIS_MOTOR_FORCE;
        chassis_mode_set->unterleib_motor2.motor_mode = CHASSIS_MOTOR_FORCE;
        chassis_mode_set->unterleib_motor3.motor_mode = CHASSIS_MOTOR_FORCE;
        chassis_mode_set->unterleib_motor4.motor_mode = CHASSIS_MOTOR_FORCE;
        chassis_mode_set->foot_L_motor.motor_mode		  = CHASSIS_MOTOR_FORCE;
        chassis_mode_set->foot_R_motor.motor_mode     = CHASSIS_MOTOR_FORCE;
    }
    else if (chassis_behaviour == CHASSIS_NO_FOLLOW )
    {
        chassis_mode_set->unterleib_motor1.motor_mode = CHASSIS_MOTOR_FORCE;
        chassis_mode_set->unterleib_motor2.motor_mode = CHASSIS_MOTOR_FORCE;
        chassis_mode_set->unterleib_motor3.motor_mode = CHASSIS_MOTOR_FORCE;
        chassis_mode_set->unterleib_motor4.motor_mode = CHASSIS_MOTOR_FORCE;
        chassis_mode_set->foot_L_motor.motor_mode		  = CHASSIS_MOTOR_FORCE;
        chassis_mode_set->foot_R_motor.motor_mode     = CHASSIS_MOTOR_FORCE;
    }
    else if (chassis_behaviour == CHASSIS_SIT )
    {
        chassis_mode_set->unterleib_motor1.motor_mode = CHASSIS_MOTOR_FORCE;
        chassis_mode_set->unterleib_motor2.motor_mode = CHASSIS_MOTOR_FORCE;
        chassis_mode_set->unterleib_motor3.motor_mode = CHASSIS_MOTOR_FORCE;
        chassis_mode_set->unterleib_motor4.motor_mode = CHASSIS_MOTOR_FORCE;
        chassis_mode_set->foot_L_motor.motor_mode		  = CHASSIS_MOTOR_FORCE;
        chassis_mode_set->foot_R_motor.motor_mode     = CHASSIS_MOTOR_FORCE;
    }		
}

/**************************************************************************/
//                         模式切换时数据过渡
//时间：2023/11/30
//问题：从瘫子状态起立的时候状态直接转换为了起身状态（蹲着），在后续的设计中应该考虑起身动作
/**************************************************************************/
static uint8_t remake_flag=0;
static uint16_t remake_temp=0;
static void Chassis_mode_change_control_transit(chassis_control_t *chassis_mode_change)
{
    static uint16_t stand_time_temp=0;
    if (chassis_mode_change == NULL)
    {
        return;
    }
    chassis_mode_change->stand_ready_last = chassis_mode_change->stand_ready;
    //yaw电机状态机切换保存数据
    if (chassis_mode_change->unterleib_motor1.last_motor_mode == CHASSIS_MOTOR_FORCE && chassis_mode_change->unterleib_motor1.motor_mode == CHASSIS_MOTOR_ZERO)
    {   //之前没摊，现在瘫了，就开摆
        chassis_mode_change->unterleib_motor1.torque_balance = chassis_mode_change->unterleib_motor1.torque_stand
                = chassis_mode_change->unterleib_motor1.torque_roll = chassis_mode_change->unterleib_motor1.torque_yaw= chassis_mode_change->unterleib_motor1.torque_out =0;
        chassis_mode_change->unterleib_motor2.torque_balance = chassis_mode_change->unterleib_motor2.torque_stand
                = chassis_mode_change->unterleib_motor2.torque_roll = chassis_mode_change->unterleib_motor2.torque_yaw= chassis_mode_change->unterleib_motor1.torque_out =0;
        chassis_mode_change->unterleib_motor3.torque_balance = chassis_mode_change->unterleib_motor3.torque_stand
                = chassis_mode_change->unterleib_motor3.torque_roll = chassis_mode_change->unterleib_motor3.torque_yaw= chassis_mode_change->unterleib_motor1.torque_out =0;
        chassis_mode_change->unterleib_motor4.torque_balance = chassis_mode_change->unterleib_motor4.torque_stand
                = chassis_mode_change->unterleib_motor4.torque_roll = chassis_mode_change->unterleib_motor4.torque_yaw= chassis_mode_change->unterleib_motor1.torque_out =0;

        chassis_mode_change->stand_ready=0;
    }
    else if ((chassis_mode_change->unterleib_motor1.last_motor_mode == CHASSIS_MOTOR_ZERO && chassis_mode_change->unterleib_motor1.motor_mode == CHASSIS_MOTOR_FORCE))
    {   //之前摊着，现在没摊
        chassis_mode_change->chassis_balance.foot_distance_L_set=chassis_mode_change->foot_L_motor.distance;
        chassis_mode_change->chassis_balance.foot_distance_R_set=chassis_mode_change->foot_R_motor.distance;
        chassis_mode_change->chassis_balance.foot_distance_set=chassis_mode_change->chassis_balance.foot_distance;
        chassis_mode_change->chassis_balance .yaw_angle_set = chassis_mode_change->chassis_balance .yaw_angle;
        chassis_mode_change->stand_ready=1;
    }
		
		//REMAKE
		if(chassis_mode_change->chassis_rc_ctrl->A1_motor_reset)
			remake_temp=0,remake_flag=1,chassis_mode_change->stand_ready=0;
		if(remake_flag)
		{
			if(remake_temp<=1000)
				remake_temp++,chassis_mode_change->stand_ready=0;
			else
				remake_flag=0,remake_temp=0,chassis_mode_change->stand_ready=1;
		}
		
		if(chassis_mode_change->chassis_balance .suspend_flag) 
		{
        chassis_mode_change->chassis_balance.foot_distance_L_set=chassis_mode_change->foot_L_motor.distance;
        chassis_mode_change->chassis_balance.foot_distance_R_set=chassis_mode_change->foot_R_motor.distance;
        chassis_mode_change->chassis_balance.foot_distance_set=chassis_mode_change->chassis_balance.foot_distance;
        chassis_mode_change->chassis_balance .yaw_angle_set = chassis_mode_change->chassis_balance .yaw_angle;		
		}		

    if((chassis_behaviour == CHASSIS_NO_FOLLOW || chassis_behaviour == CHASSIS_ZERO_FORCE) && chassis_behaviour_last == CHASSIS_FOLLOW_GIMBAL)
    {
        chassis_mode_change->chassis_balance.yaw_angle_set = chassis_mode_change->chassis_balance.yaw_angle;
    }
    if(chassis_mode_change->stand_ready==1)
    {
        if(stand_time_temp<600)stand_time_temp++;//500
        else {
            stand_time_temp=0;
            chassis_mode_change->stand_ready=2;
            chassis_mode_change->chassis_balance.foot_distance_L_set=chassis_mode_change->foot_L_motor.distance;
            chassis_mode_change->chassis_balance.foot_distance_R_set=chassis_mode_change->foot_R_motor.distance;
            chassis_mode_change->chassis_balance.foot_distance_set=chassis_mode_change->chassis_balance.foot_distance;
            chassis_mode_change->chassis_balance .yaw_angle_set = chassis_mode_change->chassis_balance .yaw_angle;
        }
    }
}


/*************************************************************************/
/*                            底盘数据更新                                */
/*************************************************************************/
void VMC_solver(chassis_control_t *feedback_update,fp32 Q1,fp32 S1,fp32 Q4,fp32 S4,uint8_t ce);	//ce 1为L 0为R
fp32 atan_tl(fp32 a);
fp32 foot_place[3]= {PI_2,0.18f,0.18f};
fp32 angle_temp1,angle_temp2;
static void chassis_feedback_update(chassis_control_t *feedback_update)
{
    fp32 temp;
    static fp32 distance_L_temp,distance_R_temp;
    feedback_update->chassis_balance.leg_angle_set=PI_2;
    /**************欧拉角采集***********************/
    feedback_update->chassis_balance.pitch_angle = *(feedback_update->chassis_INT_angle_point  + INS_PITCH_ADDRESS_OFFSET);
    feedback_update->chassis_balance.roll_angle  = *(feedback_update->chassis_INT_angle_point  + INS_ROLL_ADDRESS_OFFSET);
    feedback_update->chassis_balance.yaw_angle   = *(feedback_update->chassis_INT_angle_point  + INS_YAW_ADDRESS_OFFSET)+yaw_count*2*PI;
    feedback_update->chassis_balance.pitch_gyro  = *(feedback_update->chassis_INT_gyro_point  + INS_PITCH_ADDRESS_OFFSET);
    feedback_update->chassis_balance.roll_gyro   = *(feedback_update->chassis_INT_gyro_point  + INS_ROLL_ADDRESS_OFFSET);
    feedback_update->chassis_balance.yaw_gyro    = *(feedback_update->chassis_INT_gyro_point  + INS_YAW_ADDRESS_OFFSET);
    /**************yaw轴电机采集***************/
    feedback_update->yaw_motor.relative_angle = yaw_ecd_to_angle_change(feedback_update->yaw_motor.gimbal_motor_measure->ecd);
    feedback_update->yaw_motor.motor_speed    = 0.1047197f*(fp32)feedback_update->yaw_motor.gimbal_motor_measure->speed_rpm;
    /**********************************关节电机采集*********************************/
    //读取当前数据 
    feedback_update->unterleib_motor1.position =  feedback_update->unterleib_motor1.motor_measure->Pos - feedback_update->unterleib_motor1.position_offset - LEG_OFFEST;
    feedback_update->unterleib_motor2.position =  feedback_update->unterleib_motor2.motor_measure->Pos - feedback_update->unterleib_motor2.position_offset + LEG_OFFEST + PI;
    feedback_update->unterleib_motor3.position = -feedback_update->unterleib_motor3.motor_measure->Pos + feedback_update->unterleib_motor3.position_offset + LEG_OFFEST + PI;  //Q1
    feedback_update->unterleib_motor4.position = -feedback_update->unterleib_motor4.motor_measure->Pos + feedback_update->unterleib_motor4.position_offset - LEG_OFFEST;       //Q4
	  feedback_update->unterleib_motor1.velocity = feedback_update->unterleib_motor1.motor_measure->W;//求转速，单位是弧度/s
    feedback_update->unterleib_motor2.velocity = feedback_update->unterleib_motor2.motor_measure->W;
    feedback_update->unterleib_motor3.velocity = feedback_update->unterleib_motor3.motor_measure->W;
    feedback_update->unterleib_motor4.velocity = feedback_update->unterleib_motor4.motor_measure->W;
    feedback_update->unterleib_motor1.accel    = feedback_update->unterleib_motor1.motor_measure->Acc;
    feedback_update->unterleib_motor2.accel    = feedback_update->unterleib_motor2.motor_measure->Acc;
    feedback_update->unterleib_motor3.accel    = feedback_update->unterleib_motor3.motor_measure->Acc;
    feedback_update->unterleib_motor4.accel    = feedback_update->unterleib_motor4.motor_measure->Acc;
    /***************************驱动轮解算*****************************/
    feedback_update->foot_L_motor.position = feedback_update->foot_L_motor.motor_measure->ecd*MOTOR_ECD_TO_RAD;//角度
    feedback_update->foot_R_motor.position = feedback_update->foot_R_motor.motor_measure->ecd*MOTOR_ECD_TO_RAD;
    feedback_update->foot_L_motor.velocity = feedback_update->foot_L_motor.motor_measure->speed_rpm *ENCONDE_2_SPEED;
    feedback_update->foot_R_motor.velocity = feedback_update->foot_R_motor.motor_measure->speed_rpm *ENCONDE_2_SPEED;
    feedback_update->foot_L_motor.torque_get = feedback_update->foot_L_motor.motor_measure->given_current ;
    feedback_update->foot_R_motor.torque_get = feedback_update->foot_R_motor.motor_measure->given_current ;
    if((feedback_update->foot_L_motor.motor_measure->last_ecd - feedback_update->foot_L_motor.motor_measure->ecd )>HALF_ECD_RANGE)feedback_update->foot_L_motor.turns++;
    else if((feedback_update->foot_L_motor.motor_measure->last_ecd - feedback_update->foot_L_motor.motor_measure->ecd )<-HALF_ECD_RANGE)feedback_update->foot_L_motor.turns--;
    if((feedback_update->foot_R_motor.motor_measure->last_ecd - feedback_update->foot_R_motor.motor_measure->ecd )>HALF_ECD_RANGE)feedback_update->foot_R_motor.turns++;
    else if((feedback_update->foot_R_motor.motor_measure->last_ecd - feedback_update->foot_R_motor.motor_measure->ecd )<-HALF_ECD_RANGE)feedback_update->foot_R_motor.turns--;

    feedback_update->foot_L_motor.distance	=  (feedback_update->foot_L_motor.position/PI*0.5f + feedback_update->foot_L_motor.turns)*WHEEL_DIA - feedback_update->foot_L_motor.distance_offest;//轮子行进距离
    feedback_update->foot_R_motor.distance  = -(feedback_update->foot_R_motor.position/PI*0.5f + feedback_update->foot_R_motor.turns)*WHEEL_DIA - feedback_update->foot_R_motor.distance_offest;
    feedback_update->chassis_balance.foot_distance = (feedback_update->foot_L_motor.distance+feedback_update->foot_R_motor.distance)/2;
    K_get_absolute_speed(feedback_update);

    /**************************运动学正解********************************/
    // 五连杆运动正解算
    Forward_kinematic_solution(feedback_update,feedback_update->unterleib_motor2.position,
															 feedback_update->unterleib_motor2.velocity,
                               feedback_update->unterleib_motor1.position,
															 feedback_update->unterleib_motor1.velocity,
                               feedback_update->unterleib_motor1.accel,
															 feedback_update->unterleib_motor1.accel,0);

    Forward_kinematic_solution(feedback_update,feedback_update->unterleib_motor3.position,
                               feedback_update->unterleib_motor3.velocity,
                               feedback_update->unterleib_motor4.position,
                               feedback_update->unterleib_motor4.velocity,
                               feedback_update->unterleib_motor3.accel,
                               feedback_update->unterleib_motor4.accel,1);
															 
    feedback_update->chassis_balance .leg_dlength_L = OLS_Derivative(&OLS_S0_L,0.002f,feedback_update->chassis_balance.leg_length_L);
    feedback_update->chassis_balance .leg_dlength_R = OLS_Derivative(&OLS_S0_R,0.002f,feedback_update->chassis_balance.leg_length_R);

    feedback_update->chassis_balance .leg_dangle_L = OLS_Derivative(&OLS_angle_L,0.002f,feedback_update->chassis_balance.leg_angle_L);
    feedback_update->chassis_balance .leg_dangle_R = OLS_Derivative(&OLS_angle_R,0.002f,feedback_update->chassis_balance.leg_angle_R);

//		feedback_update->foot_L_motor.speed	=  feedback_update->foot_L_motor.velocity*WHEEL_DIA;//轮子线速度
//    feedback_update->foot_R_motor.speed = -feedback_update->foot_R_motor.velocity*WHEEL_DIA;
		feedback_update->foot_L_motor.speed	=  feedback_update->foot_L_motor.velocity*WHEEL_DIA
					+feedback_update->chassis_balance .leg_length_L*feedback_update->chassis_balance .leg_dangle_L*arm_cos_f32(feedback_update->chassis_balance .leg_angle_L-PI/2)
					+feedback_update->chassis_balance .leg_dlength_L*arm_sin_f32(feedback_update->chassis_balance .leg_angle_L-PI/2);//结算后的机体速度
    feedback_update->foot_R_motor.speed = -feedback_update->foot_R_motor.velocity*WHEEL_DIA
					+feedback_update->chassis_balance .leg_length_R*feedback_update->chassis_balance .leg_dangle_R*arm_cos_f32(feedback_update->chassis_balance .leg_angle_R-PI/2)
					+feedback_update->chassis_balance .leg_dlength_R*arm_sin_f32(feedback_update->chassis_balance .leg_angle_R-PI/2);
    feedback_update->chassis_balance.foot_speed  = (feedback_update->foot_L_motor.speed+feedback_update->foot_R_motor.speed)/2;

    feedback_update->chassis_balance .leg_angle_L  -= feedback_update->chassis_balance.pitch_angle;
    feedback_update->chassis_balance .leg_gyro_L   -= feedback_update->chassis_balance.pitch_gyro;
    feedback_update->chassis_balance .leg_angle_R  -= feedback_update->chassis_balance.pitch_angle;
    feedback_update->chassis_balance .leg_gyro_R   -= feedback_update->chassis_balance.pitch_gyro;
		
    /*******************************腿长主动适应*************************/
		if(feedback_update->chassis_rc_ctrl->sit_flag) feedback_update->chassis_balance .High=0.11f;
		else if(feedback_update->chassis_rc_ctrl->stand_flag || switch_is_up(feedback_update->chassis_rc_ctrl->mode_R))	feedback_update->chassis_balance .High=0.27f;
		else if(feedback_update->chassis_balance.suspend_flag) feedback_update->chassis_balance .High=0.18f;
		else feedback_update->chassis_balance .High=0.18f;
		//跳跃模式腿长优先级更高
//		if(feedback_update->chassis_rc_ctrl->jump_flag==64) feedback_update->chassis_balance .High=0.10f;
//		else if(feedback_update->chassis_rc_ctrl->jump_flag==128) feedback_update->chassis_balance .High=0.35f;
//		else if(feedback_update->chassis_rc_ctrl->jump_flag==192) feedback_update->chassis_balance .High=0.10f;
		if(feedback_update->chassis_rc_ctrl->jump_flag==64) feedback_update->chassis_balance .High=0.35f;
		else if(feedback_update->chassis_rc_ctrl->jump_flag==128) feedback_update->chassis_balance .High=0.10f;
				
    if(feedback_update->chassis_balance.suspend_flag)
    {   
        foot_place[1]=foot_place[2]=feedback_update->chassis_balance.High;
    }   
    else
    {   
        feedback_update->chassis_balance .foot_roll_angle = feedback_update->chassis_balance.roll_angle - atan(2*(feedback_update->chassis_balance .leg_length_L-feedback_update->chassis_balance .leg_length_R));
        foot_place[1]=feedback_update->chassis_balance .High + 0.25f*arm_sin_f32(feedback_update->chassis_balance .foot_roll_angle)/arm_cos_f32(feedback_update->chassis_balance .foot_roll_angle);
        foot_place[2]=feedback_update->chassis_balance. High - 0.25f*arm_sin_f32(feedback_update->chassis_balance .foot_roll_angle)/arm_cos_f32(feedback_update->chassis_balance .foot_roll_angle);    
		}   
		/****************************摔倒保护*************************/
	  feedback_update->chassis_balance .F_foot_L = feedback_update->chassis_balance .invJ1_L*feedback_update->unterleib_motor2 .motor_measure ->T + feedback_update->chassis_balance .invJ3_L*feedback_update->unterleib_motor1 .motor_measure ->T;
	  feedback_update->chassis_balance .F_foot_R = -(feedback_update->chassis_balance .invJ1_R*feedback_update->unterleib_motor3 .motor_measure ->T + feedback_update->chassis_balance .invJ3_R*feedback_update->unterleib_motor4 .motor_measure ->T);
	  feedback_update->chassis_balance .Tp_foot_L = feedback_update->chassis_balance .invJ2_L*feedback_update->unterleib_motor2 .motor_measure ->T + feedback_update->chassis_balance .invJ4_L*feedback_update->unterleib_motor1 .motor_measure ->T;
	  feedback_update->chassis_balance .Tp_foot_R = -(feedback_update->chassis_balance .invJ2_R*feedback_update->unterleib_motor3 .motor_measure ->T + feedback_update->chassis_balance .invJ4_R*feedback_update->unterleib_motor4 .motor_measure ->T);		
		
		if(chassis_behaviour!=CHASSIS_SIT){
			if(feedback_update->chassis_balance .F_foot_L<20 && feedback_update->chassis_balance .F_foot_R<20)
			{feedback_update->chassis_balance .suspend_flag=1;}//判断当前是否悬空 
			else 
			{feedback_update->chassis_balance .suspend_flag=0;}	
		}
		else feedback_update->chassis_balance .suspend_flag=0,feedback_update->chassis_balance .L_suspend_flag=0,feedback_update->chassis_balance .R_suspend_flag=0;
    /****************************遥控器数据滤波**************************************/
		first_order_filter_speed_ramp_cali(&rc_filter, feedback_update->chassis_rc_ctrl->X_speed);
    first_order_filter_cali(&tl_run_filter, feedback_update->chassis_rc_ctrl->X_speed);
    /******************************************************************/
    // X_SPORT KALMAN_FILTER
    SPORT_KF_UPDATE();
    // ROBOT Z ACCEL GH FILTER
    gh_Zaccel_update();
}

/****************************************************************************************/
//机体姿态稳定函数
//日期：2023/12/3
//pitch平衡：LQR
//roll平衡： 对机体角度和角速度做PD
//
//云台跟随： yaw轴电机做PD
//双腿协调： 对两腿角度差做协调
//2023/12/3  关节电机平衡力矩改为分别计算
//2023/12/3  机体高度改为对解算后腿长闭环
//2023/12/3  驱动轮反馈量完全分离
//2023/12/3  roll对地面平衡
/****************************************************************************************/
fp32 power_limit_temp;
uint16_t MAX_banlance=6,FEED_f=80,MAX_foot=6;
fp32 set_pitch;
static void chassis_banlance_control(chassis_control_t *banlance_control)
{   //求set
	
    fp32 temp,r2l_temp;
    fp32 torque_K_balance_temp1_L,torque_K_balance_temp2_L,torque_K_balance_temp1_R,torque_K_balance_temp2_R;
    fp32 torque_K_roll_high_temp1_L,torque_K_roll_high_temp2_L,torque_K_roll_high_temp1_R,torque_K_roll_high_temp2_R;
    fp32 vx_set_channel;
    /*********************外环*****************************/
    /********************角度转期望速度****************************/
    first_order_filter_cali(&tl_filter, tl_speed); 
    power_limit_temp = power_control(banlance_control);
    if(chassis_behaviour == CHASSIS_NO_FOLLOW)
    {		//不跟随云台 或 小板凳
        banlance_control->chassis_balance.torque_K_yaw=yaw_PD[1]*(tl_filter.out-banlance_control->yaw_motor.motor_speed);
        banlance_control->chassis_balance.foot_distance_set+=(power_limit_temp*rc_filter .out)*0.002f;				
    }
		else if(chassis_behaviour == CHASSIS_SIT)
		{
			banlance_control->yaw_motor.relative_angle_set= 0,ce_flag = 0;
			banlance_control->chassis_balance.torque_K_yaw=yaw_PD[0]*(banlance_control->yaw_motor.relative_angle_set-banlance_control->yaw_motor.relative_angle)+
				yaw_PD[1]*(-banlance_control->yaw_motor.motor_speed );
		}
    else if(chassis_behaviour == CHASSIS_FOLLOW_GIMBAL)
    {		//跟随云台（现在不用）
        if(banlance_control->chassis_balance .suspend_flag==0) // 未悬空
        {
            if (banlance_control->chassis_rc_ctrl->side_flag && banlance_control->yaw_motor.motor_speed<0 && banlance_control->yaw_motor.relative_angle_set==0)
            {
                banlance_control->yaw_motor.relative_angle_set= PI_2;
                ce_flag = 1;
            }
            else if (banlance_control->chassis_rc_ctrl->side_flag && banlance_control->yaw_motor.motor_speed>0 && banlance_control->yaw_motor.relative_angle_set==0)
            {
                banlance_control->yaw_motor.relative_angle_set= -PI_2;
                ce_flag = 1;
            }
            else if (banlance_control->chassis_rc_ctrl->side_flag==0)
            {
                banlance_control->yaw_motor.relative_angle_set= 0;
                ce_flag = 0;
            }

            banlance_control->chassis_balance.torque_K_yaw=yaw_PD[0]*(banlance_control->yaw_motor.relative_angle_set-banlance_control->yaw_motor.relative_angle)+
                    yaw_PD[1]*(-banlance_control->yaw_motor.motor_speed );

            if(banlance_control->yaw_motor.relative_angle_set== PI_2)
            {
                banlance_control->chassis_balance.foot_distance_set-=power_limit_temp*(rc_filter.out)*0.002f;
            }
            else
            {
                banlance_control->chassis_balance.foot_distance_set+=power_limit_temp*(rc_filter.out)*0.002f;
						}
        }
        else//跳跃过程中不跟yaw
        {
            banlance_control->chassis_balance.torque_K_yaw=0;
        }
    }
		else if(chassis_behaviour == CHASSIS_PURE_SIT || chassis_behaviour == CHASSIS_PURE || chassis_behaviour == CHASSIS_PURE_HIGH)
    {		//不跟随云台 或 小板凳
				//我他妈真是沙壁，yaw轴写成了pitch
        banlance_control->chassis_balance.torque_K_yaw=-yaw_PD[0]*(banlance_control->chassis_balance.yaw_angle_set-banlance_control->chassis_balance.yaw_angle)-yaw_PD[1]*(0-banlance_control->chassis_balance.yaw_gyro);
        banlance_control->chassis_balance.foot_distance_set+=(power_limit_temp*rc_filter.out)*0.002f;
    }
    /*************************roll+协同************************/
    if(banlance_control->chassis_balance .suspend_flag==0)//未悬空
    {
        /********************roll平衡(F)****************************/
        banlance_control->chassis_balance.torque_K_roll =
          + roll_PD[0] * (banlance_control->chassis_balance.roll_angle_set-banlance_control->chassis_balance.roll_angle)
    			+ roll_PD[1] * (0-banlance_control->chassis_balance.roll_gyro);
        /*******************双腿协同(Tp)*****************************/
        banlance_control->chassis_balance.torque_K_coordinate =
            + coordinate_PD[0] * (banlance_control->chassis_balance.leg_angle_L - banlance_control->chassis_balance.leg_angle_R)
            + coordinate_PD[1] * (banlance_control->chassis_balance.leg_gyro_L  - banlance_control->chassis_balance.leg_gyro_R);
    }
    else//悬空
    {   //跳跃下没有这些
        banlance_control->chassis_balance.torque_K_roll = 0;
        banlance_control->chassis_balance.torque_K_coordinate = 0;
    }

    /***********************腿长****************************/
    /**************小板凳蚌腿*******************/
    FEED_f = dynamic_feed_adj();   // 动态前馈推力分配
    /*******************************************/
		if(chassis_control.chassis_rc_ctrl->jump_flag){
			banlance_control->chassis_balance.torque_K_stand_L =
					FEED_f+Jump_PD[0] * (foot_place[1]-banlance_control->chassis_balance.leg_length_L) + Jump_PD[1] *(-banlance_control->chassis_balance.leg_dlength_L);
			banlance_control->chassis_balance.torque_K_stand_R =
					FEED_f+Jump_PD[0] * (foot_place[2]-banlance_control->chassis_balance.leg_length_R) + Jump_PD[1] *(-banlance_control->chassis_balance.leg_dlength_R);			
		} else {
			banlance_control->chassis_balance.torque_K_stand_L =
					FEED_f+stand_PD[0] * (foot_place[1]-banlance_control->chassis_balance.leg_length_L) + stand_PD[1] *(-banlance_control->chassis_balance.leg_dlength_L);
			banlance_control->chassis_balance.torque_K_stand_R =
					FEED_f+stand_PD[0] * (foot_place[2]-banlance_control->chassis_balance.leg_length_R) + stand_PD[1] *(-banlance_control->chassis_balance.leg_dlength_R);		
		}
    /********************pitch平衡****************************/
		//2023.12.22 突然发现符号取反了，麻喽麻喽
    if(banlance_control->chassis_balance .suspend_flag==1)
    {   //悬空
        banlance_control->chassis_balance.torque_K_balance_L =
            -balance_LQR_sit[0][0]*(PI_2-banlance_control->chassis_balance.leg_angle_L)
       			-balance_LQR_sit[0][1]*(0 - banlance_control->chassis_balance.leg_gyro_L);
        banlance_control->chassis_balance.torque_K_balance_R =
            +balance_LQR_sit[0][0]*(PI_2-banlance_control->chassis_balance.leg_angle_R)
      			+balance_LQR_sit[0][1]*(0 - banlance_control->chassis_balance.leg_gyro_R);
    }			
    else if(chassis_behaviour == CHASSIS_NO_FOLLOW)
    {		//不跟随云台
				if(banlance_control->chassis_rc_ctrl->stand_flag || switch_is_up(banlance_control->chassis_rc_ctrl->mode_R))
				{
					banlance_control->chassis_balance.torque_K_balance_L =
							-balance_LQR_stand[0][0]*(PI_2-banlance_control->chassis_balance .leg_angle_L)
							-balance_LQR_stand[0][1]*(0 - banlance_control->chassis_balance.leg_gyro_L)
							-balance_LQR_stand[0][2]*(banlance_control->chassis_balance.foot_distance_set - banlance_control->chassis_balance.foot_distance  )
							-balance_LQR_stand[0][3]*(0 - banlance_control->chassis_balance.foot_speed)
							-balance_LQR_stand[0][4]*(PITCH_OFFSET-banlance_control->chassis_balance.pitch_angle)
							-balance_LQR_stand[0][5]*(0-banlance_control->chassis_balance.pitch_gyro );

					banlance_control->chassis_balance.torque_K_balance_R =
							+balance_LQR_stand[0][0]*(PI_2-banlance_control->chassis_balance .leg_angle_R)
							+balance_LQR_stand[0][1]*(0 - banlance_control->chassis_balance.leg_gyro_R)
							+balance_LQR_stand[0][2]*(banlance_control->chassis_balance.foot_distance_set -banlance_control->chassis_balance.foot_distance  )
							+balance_LQR_stand[0][3]*(0 - banlance_control->chassis_balance.foot_speed)
							+balance_LQR_stand[0][4]*(PITCH_OFFSET-banlance_control->chassis_balance.pitch_angle)
							+balance_LQR_stand[0][5]*(0-banlance_control->chassis_balance.pitch_gyro );				
				}
				else
				{
					banlance_control->chassis_balance.torque_K_balance_L =
							-balance_LQR_tl[0][0]*(PI_2-banlance_control->chassis_balance .leg_angle_L)
							-balance_LQR_tl[0][1]*(0 - banlance_control->chassis_balance.leg_gyro_L)
							-balance_LQR_tl[0][2]*(banlance_control->chassis_balance.foot_distance_set - banlance_control->chassis_balance.foot_distance  )
							-balance_LQR_tl[0][3]*(0 - banlance_control->chassis_balance.foot_speed)
							-balance_LQR_tl[0][4]*(PITCH_OFFSET-banlance_control->chassis_balance.pitch_angle)
							-balance_LQR_tl[0][5]*(0-banlance_control->chassis_balance.pitch_gyro );

					banlance_control->chassis_balance.torque_K_balance_R =
							+balance_LQR_tl[0][0]*(PI_2-banlance_control->chassis_balance .leg_angle_R)
							+balance_LQR_tl[0][1]*(0 - banlance_control->chassis_balance.leg_gyro_R)
							+balance_LQR_tl[0][2]*(banlance_control->chassis_balance.foot_distance_set -banlance_control->chassis_balance.foot_distance  )
							+balance_LQR_tl[0][3]*(0 - banlance_control->chassis_balance.foot_speed)
							+balance_LQR_tl[0][4]*(PITCH_OFFSET-banlance_control->chassis_balance.pitch_angle)
							+balance_LQR_tl[0][5]*(0-banlance_control->chassis_balance.pitch_gyro );						
				}					
    }
    else
    {   //跟随云台（现在用）
				if(banlance_control->chassis_rc_ctrl->stand_flag || switch_is_up(banlance_control->chassis_rc_ctrl->mode_R))
				{   //站立
						banlance_control->chassis_balance.torque_K_balance_L =
								-balance_LQR_stand[0][0]*(PI_2 - banlance_control->chassis_balance.leg_angle_L)
								-balance_LQR_stand[0][1]*(0 - banlance_control->chassis_balance.leg_gyro_L)
								-balance_LQR_stand[0][2]*(banlance_control->chassis_balance.foot_distance_set - banlance_control->chassis_balance.foot_distance )
								-balance_LQR_stand[0][3]*(0 - banlance_control->chassis_balance.foot_speed)
								-balance_LQR_stand[0][4]*(PITCH_OFFSET - banlance_control->chassis_balance.pitch_angle)
								-balance_LQR_stand[0][5]*(0 - banlance_control->chassis_balance.pitch_gyro );

						banlance_control->chassis_balance.torque_K_balance_R =
								+balance_LQR_stand[0][0]*(PI_2-banlance_control->chassis_balance .leg_angle_R) 
								+balance_LQR_stand[0][1]*(0 - banlance_control->chassis_balance.leg_gyro_R)
								+balance_LQR_stand[0][2]*(banlance_control->chassis_balance.foot_distance_set - banlance_control->chassis_balance.foot_distance)
								+balance_LQR_stand[0][3]*(0 - banlance_control->chassis_balance.foot_speed)
								+balance_LQR_stand[0][4]*(PITCH_OFFSET-banlance_control->chassis_balance.pitch_angle)
								+balance_LQR_stand[0][5]*(0-banlance_control->chassis_balance.pitch_gyro);				
				}
				else if(banlance_control->chassis_rc_ctrl->sit_flag)
				{   //蹲下模式
						banlance_control->chassis_balance.torque_K_balance_L =
								-balance_LQR_sit[0][0]*(PI_2 - banlance_control->chassis_balance.leg_angle_L)
								-balance_LQR_sit[0][1]*(0 - banlance_control->chassis_balance.leg_gyro_L)
								-balance_LQR_sit[0][2]*(banlance_control->chassis_balance.foot_distance_set - banlance_control->chassis_balance.foot_distance )
								-balance_LQR_sit[0][3]*(0 - banlance_control->chassis_balance.foot_speed)
								-balance_LQR_sit[0][4]*(PITCH_OFFSET - banlance_control->chassis_balance.pitch_angle)
								-balance_LQR_sit[0][5]*(0 - banlance_control->chassis_balance.pitch_gyro );

						banlance_control->chassis_balance.torque_K_balance_R =
								+balance_LQR_sit[0][0]*(PI_2-banlance_control->chassis_balance .leg_angle_R) 
								+balance_LQR_sit[0][1]*(0 - banlance_control->chassis_balance.leg_gyro_R)
								+balance_LQR_sit[0][2]*(banlance_control->chassis_balance.foot_distance_set - banlance_control->chassis_balance.foot_distance)
								+balance_LQR_sit[0][3]*(0 - banlance_control->chassis_balance.foot_speed)
								+balance_LQR_sit[0][4]*(PITCH_OFFSET-banlance_control->chassis_balance.pitch_angle)
								+balance_LQR_sit[0][5]*(0-banlance_control->chassis_balance.pitch_gyro);				
				}		
				else
				{   //普通模式
						banlance_control->chassis_balance.torque_K_balance_L =
								-balance_LQR[0][0]*(PI_2 - banlance_control->chassis_balance.leg_angle_L)
								-balance_LQR[0][1]*(0 - banlance_control->chassis_balance.leg_gyro_L)
								-balance_LQR[0][2]*(banlance_control->chassis_balance.foot_distance_set - banlance_control->chassis_balance.foot_distance )
								-balance_LQR[0][3]*(0 - banlance_control->chassis_balance.foot_speed)
								-balance_LQR[0][4]*(PITCH_OFFSET - banlance_control->chassis_balance.pitch_angle)
								-balance_LQR[0][5]*(0 - banlance_control->chassis_balance.pitch_gyro );

						banlance_control->chassis_balance.torque_K_balance_R =
								+balance_LQR[0][0]*(PI_2-banlance_control->chassis_balance .leg_angle_R) 
								+balance_LQR[0][1]*(0 - banlance_control->chassis_balance.leg_gyro_R)
								+balance_LQR[0][2]*(banlance_control->chassis_balance.foot_distance_set - banlance_control->chassis_balance.foot_distance)
								+balance_LQR[0][3]*(0 - banlance_control->chassis_balance.foot_speed)
								+balance_LQR[0][4]*(PITCH_OFFSET-banlance_control->chassis_balance.pitch_angle)
								+balance_LQR[0][5]*(0-banlance_control->chassis_balance.pitch_gyro);				
				}	
    }		
    /***********************************************************/
    if(banlance_control->stand_ready)
    {
        /**************小板凳*******************/
        if(chassis_behaviour==CHASSIS_SIT || banlance_control->chassis_rc_ctrl->sit_flag)
				{
//						set_pitch=0.1f*(banlance_control->chassis_balance.foot_distance -banlance_control->chassis_balance.foot_distance_set) - 0.1*banlance_control->chassis_balance .foot_speed;
//						LimitMax(set_pitch,0.09f);
						banlance_control->foot_L_motor .torque_out =
								-balance_LQR_sit[1][0]*(PI_2-banlance_control->chassis_balance.leg_angle_L)
								-balance_LQR_sit[1][1]*(0 - banlance_control->chassis_balance.leg_gyro_L)		
								+balance_LQR_sit[1][2]*(banlance_control->chassis_balance.foot_distance_set -banlance_control->chassis_balance.foot_distance)
								+balance_LQR_sit[1][3]*(0 - banlance_control->chassis_balance.foot_speed)
								+balance_LQR_sit[1][4]*(0-banlance_control->chassis_balance.pitch_angle)
								+balance_LQR_sit[1][5]*(0-banlance_control->chassis_balance.pitch_gyro );

						banlance_control->foot_R_motor .torque_out =
								+balance_LQR_sit[1][0]*(PI_2-banlance_control->chassis_balance.leg_angle_R)
								+balance_LQR_sit[1][1]*(0 - banlance_control->chassis_balance.leg_gyro_R)
								-balance_LQR_sit[1][2]*(banlance_control->chassis_balance.foot_distance_set -banlance_control->chassis_balance .foot_distance)
								-balance_LQR_sit[1][3]*(0 - banlance_control->chassis_balance.foot_speed)
								-balance_LQR_sit[1][4]*(0-banlance_control->chassis_balance.pitch_angle)
								-balance_LQR_sit[1][5]*(0-banlance_control->chassis_balance.pitch_gyro );				
				}
				else if(switch_is_up(banlance_control->chassis_rc_ctrl->mode_R))
        {
						banlance_control->foot_L_motor .torque_out =
								-balance_LQR_stand[1][0]*(PI_2-banlance_control->chassis_balance.leg_angle_L)
								-balance_LQR_stand[1][1]*(0 - banlance_control->chassis_balance.leg_gyro_L)
								+balance_LQR_stand[1][2]*(banlance_control->chassis_balance.foot_distance_set -banlance_control->chassis_balance.foot_distance)
								+balance_LQR_stand[1][3]*(0 - banlance_control->chassis_balance.foot_speed)
								+balance_LQR_stand[1][4]*(PITCH_OFFSET-banlance_control->chassis_balance.pitch_angle)
								+balance_LQR_stand[1][5]*(0-banlance_control->chassis_balance.pitch_gyro );

						banlance_control->foot_R_motor .torque_out =
								+balance_LQR_stand[1][0]*(PI_2-banlance_control->chassis_balance.leg_angle_R)
								+balance_LQR_stand[1][1]*(0 - banlance_control->chassis_balance.leg_gyro_R)
								-balance_LQR_stand[1][2]*(banlance_control->chassis_balance.foot_distance_set -banlance_control->chassis_balance .foot_distance)
								-balance_LQR_stand[1][3]*(0 - banlance_control->chassis_balance.foot_speed)
								-balance_LQR_stand[1][4]*(PITCH_OFFSET-banlance_control->chassis_balance.pitch_angle)
								-balance_LQR_stand[1][5]*(0-banlance_control->chassis_balance.pitch_gyro );
        }							
        /*********************普通************************/
        else
        {
            if(ce_flag == 0) // 未测对敌（现在主要用这个）
            {
                banlance_control->foot_L_motor .torque_out =
                    -balance_LQR[1][0]*(PI_2-banlance_control->chassis_balance.leg_angle_L)
                    -balance_LQR[1][1]*(0 - banlance_control->chassis_balance.leg_gyro_L)
                    +balance_LQR[1][2]*(banlance_control->chassis_balance.foot_distance_set -banlance_control->chassis_balance.foot_distance)
                    +balance_LQR[1][3]*(0 - banlance_control->chassis_balance.foot_speed)
                    //+balance_LQR_sit[1][3]*(0 -xsport.xhat_data[1])
                    +balance_LQR[1][4]*(set_pitch+PITCH_OFFSET-banlance_control->chassis_balance.pitch_angle)
                    +balance_LQR[1][5]*(0-banlance_control->chassis_balance.pitch_gyro );

                banlance_control->foot_R_motor .torque_out =
                    +balance_LQR[1][0]*(PI_2-banlance_control->chassis_balance.leg_angle_R)
                    +balance_LQR[1][1]*(0-banlance_control->chassis_balance.leg_gyro_R)
                    -balance_LQR[1][2]*(banlance_control->chassis_balance.foot_distance_set -banlance_control->chassis_balance .foot_distance)
                    -balance_LQR[1][3]*(0 - banlance_control->chassis_balance.foot_speed)
                    //-balance_LQR_sit[1][3]*(0 -xsport.xhat_data[1])
                    -balance_LQR[1][4]*(set_pitch+PITCH_OFFSET-banlance_control->chassis_balance.pitch_angle)
                    -balance_LQR[1][5]*(0-banlance_control->chassis_balance.pitch_gyro );
            }
            else if(ce_flag == 1) // 测对敌人 站的稳但是慢 单独一个参量
            { 
                banlance_control->foot_L_motor .torque_out =
                    -balance_LQR_CE[1][0]*(PI_2-banlance_control->chassis_balance.leg_angle_L)
                    -balance_LQR_CE[1][1]*(0 - banlance_control->chassis_balance.leg_gyro_L)
                    +balance_LQR_CE[1][2]*(banlance_control->chassis_balance.foot_distance_set -banlance_control->chassis_balance.foot_distance)
                    +balance_LQR_CE[1][3]*(0 - banlance_control->chassis_balance.foot_speed)
                    +balance_LQR_CE[1][4]*(PITCH_OFFSET-banlance_control->chassis_balance.pitch_angle)
                    +balance_LQR_CE[1][5]*(0-banlance_control->chassis_balance.pitch_gyro );

                banlance_control->foot_R_motor .torque_out =
                    +balance_LQR_CE[1][0]*(PI_2-banlance_control->chassis_balance.leg_angle_R)
                    +balance_LQR_CE[1][1]*(0-banlance_control->chassis_balance.leg_gyro_R)
                    -balance_LQR_CE[1][2]*(banlance_control->chassis_balance.foot_distance_set -banlance_control->chassis_balance .foot_distance)
                    -balance_LQR_CE[1][3]*(0 - banlance_control->chassis_balance.foot_speed)
                    -balance_LQR_CE[1][4]*(PITCH_OFFSET-banlance_control->chassis_balance.pitch_angle)
                    -balance_LQR_CE[1][5]*(0-banlance_control->chassis_balance.pitch_gyro );
            }
        }
    }
    else
    {
        banlance_control->foot_L_motor.torque_out =  balance_LQR[1][3]*(rc_filter.out/2 - banlance_control->chassis_balance.foot_speed);
        banlance_control->foot_R_motor.torque_out = -balance_LQR[1][3]*(rc_filter.out/2 - banlance_control->chassis_balance.foot_speed);  
				banlance_control->chassis_balance  .torque_K_yaw=0;
    }
		
    /*****************************合并********************************/
    torque_K_balance_temp1_L = ( + banlance_control->chassis_balance.torque_K_balance_L + banlance_control->chassis_balance.torque_K_coordinate)*banlance_control->chassis_balance.J3_L;
    torque_K_balance_temp2_L = ( + banlance_control->chassis_balance.torque_K_balance_L + banlance_control->chassis_balance.torque_K_coordinate)*banlance_control->chassis_balance.J4_L;
    torque_K_balance_temp1_R = ( + banlance_control->chassis_balance.torque_K_balance_R + banlance_control->chassis_balance.torque_K_coordinate)*banlance_control->chassis_balance.J3_R;
    torque_K_balance_temp2_R = ( + banlance_control->chassis_balance.torque_K_balance_R + banlance_control->chassis_balance.torque_K_coordinate)*banlance_control->chassis_balance.J4_R;
		
		torque_K_roll_high_temp1_L = ( + banlance_control->chassis_balance.torque_K_stand_L - banlance_control->chassis_balance.torque_K_roll) * banlance_control->chassis_balance.J1_L;
    torque_K_roll_high_temp2_L = ( + banlance_control->chassis_balance.torque_K_stand_L - banlance_control->chassis_balance.torque_K_roll) * banlance_control->chassis_balance.J2_L;
    torque_K_roll_high_temp1_R = ( + banlance_control->chassis_balance.torque_K_stand_R + banlance_control->chassis_balance.torque_K_roll) * banlance_control->chassis_balance.J1_R;
    torque_K_roll_high_temp2_R = ( + banlance_control->chassis_balance.torque_K_stand_R + banlance_control->chassis_balance.torque_K_roll) * banlance_control->chassis_balance.J2_R;
    /******************小板凳模式不前后摆腿***********************/
    if(banlance_control->chassis_rc_ctrl->sit_flag)	    MAX_banlance=2000;
    else                                              	MAX_banlance=2000;    
		/****************************************/
    LimitMax(torque_K_balance_temp1_L,MAX_banlance);
    LimitMax(torque_K_balance_temp2_L,MAX_banlance);
    LimitMax(torque_K_balance_temp1_R,MAX_banlance);
    LimitMax(torque_K_balance_temp2_R,MAX_banlance);
    LimitMax(torque_K_roll_high_temp1_L,15);
    LimitMax(torque_K_roll_high_temp2_L,15);
    LimitMax(torque_K_roll_high_temp1_R,15);
    LimitMax(torque_K_roll_high_temp2_R,15);
//    LimitMax(torque_K_balance_temp1_L,3000);
//    LimitMax(torque_K_balance_temp2_L,3000);
//    LimitMax(torque_K_balance_temp1_R,3000);
//    LimitMax(torque_K_balance_temp2_R,3000);
//    LimitMax(torque_K_roll_high_temp1_L,20);
//    LimitMax(torque_K_roll_high_temp2_L,20);
//    LimitMax(torque_K_roll_high_temp1_R,20);
//    LimitMax(torque_K_roll_high_temp2_R,20);

    banlance_control->unterleib_motor1.torque_out = + torque_K_balance_temp2_R	+ torque_K_roll_high_temp2_R;
    banlance_control->unterleib_motor2.torque_out = + torque_K_balance_temp1_R	+ torque_K_roll_high_temp1_R;
    banlance_control->unterleib_motor3.torque_out = + torque_K_balance_temp1_L	- torque_K_roll_high_temp1_L;
    banlance_control->unterleib_motor4.torque_out = + torque_K_balance_temp2_L	- torque_K_roll_high_temp2_L;
	
		LimitMax(banlance_control->unterleib_motor1.torque_out,15);
    LimitMax(banlance_control->unterleib_motor2.torque_out,15);
    LimitMax(banlance_control->unterleib_motor3.torque_out,15);
    LimitMax(banlance_control->unterleib_motor4.torque_out,15);
		
		/*******************************跳跃模式*****************************************/
//		if(banlance_control->chassis_rc_ctrl->jump_flag==64){
//			banlance_control->unterleib_motor1.torque_out=-5;banlance_control->unterleib_motor2.torque_out=5;
//			banlance_control->unterleib_motor3.torque_out=-5;banlance_control->unterleib_motor4.torque_out=5;
//			LimitMax(banlance_control->unterleib_motor1.torque_out,10);
//			LimitMax(banlance_control->unterleib_motor2.torque_out,10);
//			LimitMax(banlance_control->unterleib_motor3.torque_out,10);
//			LimitMax(banlance_control->unterleib_motor4.torque_out,10);			
//		}
//		if(banlance_control->chassis_rc_ctrl->jump_flag==96){
//			banlance_control->unterleib_motor1.torque_out=20;banlance_control->unterleib_motor2.torque_out=-20;
//			banlance_control->unterleib_motor3.torque_out=20;banlance_control->unterleib_motor4.torque_out=-20;
//			LimitMax(banlance_control->unterleib_motor1.torque_out,20);
//			LimitMax(banlance_control->unterleib_motor2.torque_out,20);
//			LimitMax(banlance_control->unterleib_motor3.torque_out,20);
//			LimitMax(banlance_control->unterleib_motor4.torque_out,20);			
//		}
//		if(banlance_control->chassis_rc_ctrl->jump_flag==192){
//			banlance_control->unterleib_motor1.torque_out=-10;banlance_control->unterleib_motor2.torque_out=10;
//			banlance_control->unterleib_motor3.torque_out=-10;banlance_control->unterleib_motor4.torque_out=10;
//			LimitMax(banlance_control->unterleib_motor1.torque_out,10);
//			LimitMax(banlance_control->unterleib_motor2.torque_out,10);
//			LimitMax(banlance_control->unterleib_motor3.torque_out,10);
//			LimitMax(banlance_control->unterleib_motor4.torque_out,10);			
//		}		

    LimitMax(banlance_control->chassis_balance.torque_K_yaw,1000);
    banlance_control->foot_L_motor.torque_out += banlance_control->chassis_balance.torque_K_yaw;
    banlance_control->foot_R_motor.torque_out += banlance_control->chassis_balance.torque_K_yaw;
		if(banlance_control->chassis_balance.save_mode==2)
			banlance_control->foot_L_motor.torque_out=0,banlance_control->foot_R_motor.torque_out=0;
		if((chassis_control.stand_ready==2 && banlance_control->chassis_balance .suspend_flag))
			banlance_control->foot_R_motor.torque_out=0,banlance_control->foot_L_motor.torque_out=0;
    power_control(banlance_control);
    /******************运动模式轮子劲大***********************/
    if(banlance_control->chassis_rc_ctrl->sport_flag)	  MAX_foot=2000;
    else                                              	MAX_foot=2000;
    LimitMax(banlance_control->foot_L_motor.torque_out,MAX_foot);
    LimitMax(banlance_control->foot_R_motor.torque_out,MAX_foot);
}


/**********************************速度斜坡函数****************************************/
fp32 Velocity_ramp(float RC_filter,float XMAX)
{
	static float last_RC_filter=0,foot_distance_buffer=0;
	if(RC_filter-last_RC_filter>=XMAX) RC_filter+=XMAX;
	else if(RC_filter-last_RC_filter<=-XMAX) RC_filter-=XMAX;
	foot_distance_buffer+=RC_filter;
	last_RC_filter = RC_filter;
	return foot_distance_buffer;
}

/**************功率控制*********************/
fp32 power_control(chassis_control_t *banlance_control)
{
	fp32 limit_coefficient,I_L,I_R;
	/**********************计算当前底盘功率*********************/
	I_L	= ABS(banlance_control->foot_L_motor.torque_get )*(33.0f/2048);
	I_R = ABS(banlance_control->foot_R_motor.torque_get )*(33.0f/2048);
	banlance_control->foot_L_motor.power_now =I_L * ABS(banlance_control->foot_L_motor.motor_measure->speed_rpm)/20.0f ;
	banlance_control->foot_R_motor.power_now =I_R * ABS(banlance_control->foot_R_motor.motor_measure->speed_rpm)/20.0f ;
	limit_coefficient = (banlance_control->chassis_rc_ctrl->energy_buff-20)/(banlance_control->foot_L_motor.power_now + banlance_control->foot_R_motor.power_now);
	first_order_filter_cali(&power_limit_filter,limit_coefficient);
	LimitPNMax(limit_coefficient,1.0f,0.0f);	
  return limit_coefficient;
}

/****************************************************************************************/
//轮绝对速度求解
//日期：2023/11/30
/****************************************************************************************/
fp32 L[2][2]= {15,10,10,15};
void K_get_absolute_speed(chassis_control_t *banlance_control)
{
    static fp32 x_k,x_k_1,dx_k,dx_k_1,ddx_k,z, dz,a;
    z = banlance_control->chassis_balance .foot_distance  ;
    dz = banlance_control->chassis_balance .foot_speed ;
    a = -INS.MotionAccel_n[0];
    //上一次的估计值放到k-1里
    x_k_1=x_k;
    dx_k_1=dx_k;
    //求解当前dx矩阵
    dx_k =dx_k_1+L[0][0]*(z - x_k_1) + L[0][1]*(dz -dx_k_1);
    ddx_k=a+     L[1][0]*(z - x_k_1) + L[0][1]*(dz -dx_k_1);
    //用dx求解x估计矩阵
    x_k = x_k_1 +dx_k*0.002f;
    dx_k =dx_k_1 + ddx_k*0.002f;
    banlance_control->chassis_balance .foot_distance_K = x_k;
    banlance_control->chassis_balance .foot_speed_K= dx_k;
}

/* mode 0 停转 5 开环缓慢转动 10 闭环伺服转动*/
static void A1_motor_init(void)
{
    A1_send_data[0].id = 0;
    A1_send_data[0].mode = 0;
    A1_send_data[0].K_P = 0;
    A1_send_data[0].K_W = 0;
    A1_send_data[0].Pos = 0;
    A1_send_data[0].T = 0;
    A1_send_data[0].W = 0;

    A1_send_data[1].id = 1;
    A1_send_data[1].mode = 0;
    A1_send_data[1].K_P = 0;
    A1_send_data[1].K_W = 0;
    A1_send_data[1].Pos = 0;
    A1_send_data[1].T = 0;
    A1_send_data[1].W = 0;

    A1_send_data[2].id = 1;
    A1_send_data[2].mode = 0;
    A1_send_data[2].K_P = 0;
    A1_send_data[2].K_W = 0;
    A1_send_data[2].Pos = 0;
    A1_send_data[2].T = 0;
    A1_send_data[2].W = 0;

    A1_send_data[3].id = 0;
    A1_send_data[3].mode = 0;
    A1_send_data[3].K_P = 0;
    A1_send_data[3].K_W = 0;
    A1_send_data[3].Pos = 0;
    A1_send_data[3].T = 0;
    A1_send_data[3].W = 0;
}

static void A1_motor_sport_set(void)
{
    A1_send_data[0].id = 0;
    A1_send_data[0].mode = 10;
    A1_send_data[0].K_P = 0;
    A1_send_data[0].K_W = 0.8f;
    A1_send_data[0].Pos = 0;
    A1_send_data[0].T = 0;
    A1_send_data[0].W = 0;

    A1_send_data[1].id = 1;
    A1_send_data[1].mode = 10;
    A1_send_data[1].K_P = 0;
    A1_send_data[1].K_W = 0.8f;
    A1_send_data[1].Pos = 0;
    A1_send_data[1].T = 0;
    A1_send_data[1].W = 0;

    A1_send_data[2].id = 1;
    A1_send_data[2].mode = 10;
    A1_send_data[2].K_P = 0;
    A1_send_data[2].K_W = 0.8f;
    A1_send_data[2].Pos = 0;
    A1_send_data[2].T = 0;
    A1_send_data[2].W = 0;

    A1_send_data[3].id = 0;
    A1_send_data[3].mode = 10;
    A1_send_data[3].K_P = 0;
    A1_send_data[3].K_W = 0.8f;
    A1_send_data[3].Pos = 0;
    A1_send_data[3].T = 0;
    A1_send_data[3].W = 0;
}

void motor_position_Init(uint8_t id)
{
    CanComm_ControlCmd(CMD_MOTOR_MODE,id);
    vTaskDelay(100);
    CanComm_SendControlPara(0,0,0,0,0,id);
    vTaskDelay(100);
    CanComm_ControlCmd(CMD_ZERO_POSITION,id);
}

void A1_motor_reset(void)
{
    A1_send_data[0].id = 0;
    A1_send_data[0].mode = 10;
    A1_send_data[0].K_P = 0;
    A1_send_data[0].K_W = 1.0f;
    A1_send_data[0].Pos = 0;
    A1_send_data[0].T = 0;
    A1_send_data[0].W = 0;

    A1_send_data[1].id = 1;
    A1_send_data[1].mode = 10;
    A1_send_data[1].K_P = 0;
    A1_send_data[1].K_W = 1.0f;
    A1_send_data[1].Pos = 0;
    A1_send_data[1].T = 0;
    A1_send_data[1].W = 0;

    A1_send_data[2].id = 1;
    A1_send_data[2].mode = 10;
    A1_send_data[2].K_P = 0;
    A1_send_data[2].K_W = 1.0f;
    A1_send_data[2].Pos = 0;
    A1_send_data[2].T = 0;
    A1_send_data[2].W = 0;

    A1_send_data[3].id = 0;
    A1_send_data[3].mode = 10;
    A1_send_data[3].K_P = 0;
    A1_send_data[3].K_W = 1.0f;
    A1_send_data[3].Pos = 0;
    A1_send_data[3].T = 0;
    A1_send_data[3].W = 0;
}

static fp32 yaw_ecd_to_angle_change(uint16_t ecd)
{
    int32_t relative_ecd = ecd-HAND_ENCONDE+ECD_RANGE;
    if(relative_ecd>ECD_RANGE)
        relative_ecd-=ECD_RANGE;

    if(relative_ecd<-HALF_ECD_RANGE)
        relative_ecd=relative_ecd+ECD_RANGE;

    if(relative_ecd>HALF_ECD_RANGE)
        relative_ecd=relative_ecd-ECD_RANGE;

    return relative_ecd*ENCONDE_2_ANGLE;
}


_GH_Filter_Struct RB_Zaccel;

void gh_Zaccel_init(void)
{
    memset(&RB_Zaccel,0,sizeof(RB_Zaccel));
    RB_Zaccel.dt = 0.001;
    RB_Zaccel.g  = 0.015;
    RB_Zaccel.h  = 0.0001;
}

float obv_real,obv_gh;

void gh_Zaccel_update(void)
{
    g_h_fliter(INS.Accel[2],&RB_Zaccel);
}

// dynamic feedback fource adjustment
float dynamic_feed_adj(void)
{
    float accel = RB_Zaccel.x_now;
    float F_feed = 0;
    if(chassis_control.stand_ready == 0x02)
    {
        if(switch_is_down(chassis_control.rc_ctrl->rc.s[1]) || chassis_behaviour == CHASSIS_SIT)
        {
            F_feed = 0;
        }
        else if(ABS(accel - 9.8)< 0.1)// 正常状态
        {
            F_feed = 80;
        }
        else // 动态补偿
        {
            F_feed = 80 + (9.8 - accel)*10;
            if(F_feed < 80)
            {
                F_feed = 80;
            }
        }
    }
    else
    {
        F_feed = 0;
    }
		if(chassis_control.chassis_rc_ctrl->jump_flag==128) F_feed = 0; 
    return F_feed;
}


fp32 Speed_ramp(fp32 speed_set,fp32 foot_speed)
{
	fp32 speed_temp = speed_set;
	if(speed_set>0){
		 if(speed_set-foot_speed>=1.2f)
			 speed_set=foot_speed+1.2f;
		 else
			 speed_set=speed_temp;
	}
	else if(speed_set<0){
		if(speed_set-foot_speed<-1.2f)
			speed_set=foot_speed-1.2f;
		else
			speed_set=speed_temp;
	}
	else speed_set=speed_temp;
	
	LimitPNMax(speed_set,2.7f,-1.8f);
//	LimitPNMax(speed_set,2.5f,-1.8f);
	if(!chassis_control.chassis_rc_ctrl->sport_flag) LimitMax(speed_set,1.2f);
	
	return speed_set;
}


