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
#include "fire_control_action.h"
#include "bsp_filter.h"
#include "arm_math.h"
#include "bsp_usart.h"
#include "bsp_adrc.h"
#include "referee.h"
#include "tim.h"
#include "calibration.h"
#include "bsp_flash.h"
#include "lqr.h"
#include "SolveTrajectory.h"
/************************************************************************************/
//电机编码值规整 0—8191
#define ABS(x) (((x) > 0) ? (x) : (-(x)))
#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                               \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
    }
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
#define gimbal_total_lqr_clear(gimbal_clear)                                                \
    {                                                                                       \
				LQR_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_PC_lqr);										\
				LQR_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_PC_lqr);									\
    }		
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif
int8_t Pawn_mode;
/***************************************************************************************/
//云台控制所有相关数据
gimbal_control_t gimbal_control;
extKalman_t  K_remo,K_yaw_Auto,K_pitch_Auto;
sup_cup_t    sup_cap;
gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;//行为层模式设置
static gimbal_behaviour_e gimbal_behaviour_last = GIMBAL_ZERO_FORCE;//行为层模式设置
/*********************************************************************************/
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear);
static float Matlab_Add_Pitch(int distance);
static fp32 pitch_ecd_to_angle_change(uint16_t ecd);
static fp32 yaw_ecd_to_angle_change(uint16_t ecd);
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set,uint8_t type);
/****************************************************************************/
static void Gimbal_Int(gimbal_control_t *init);
static void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set);

static void Gimbal_Behavior(gimbal_control_t *gimbal_mode_set);
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change);
static void gimbal_set_control(gimbal_control_t *set_control);
int16_t gimbal_LQR_calc(fp32 K1,fp32 K2,fp32 angle_get,fp32 K0,fp32 speed_get,fp32 angle_set,fp32 speed_set);
static void Gimbal_PID(gimbal_control_t *control_loop);
int16_t gimbal_PI_LQR(fp32 KP,fp32 KI,fp32 K1,fp32 K2,fp32 K3,//PI_LQR Serise
										  fp32 angle_get,fp32 speed_get,fp32 current_get,
									    fp32 angle_set);
/*************************************对于从遥控器传过来的数据进行采集*************************************************************/
void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
int16_t servo_buty=1500;
/******************************************************************************************************/

//TD_t TD_yaw;
//TD_t TD_pitch;
uint16_t Guard_shoot=0x100;
//                       K1   K2   K3 KP    KI
fp32 LQR_PI_pitch[5]=   { 0, 10000,70,150000,1500};// 0, 10000,60,200000,2000};
fp32 LQR_PI_pitch_AC[5]={ 0, 10000,70,200000,1500};
fp32 LQR_PI_yaw[5]=     { 0, 20000,0,252060,0};
fp32 LQR_PI_yaw_AC[5]=  { 0, 20000,0,252060,0};
fp32 LQR_ACyaw[3]={152060,15000,0};
fp32 LQR_ACpitch[3]={160000,20000,0};
kalman2_state tast_K2;
int16_t chassis_X_speed_temp,chassis_Y_speed_temp;
uint16_t power_flag;
uint8_t test_fire_flag=0;
int jump_flag=0;//目前当作跳跃的标志位 test使用
int jump_delay_flag=1;//jump等待
int jump_delay=80;
void gimbal_task(void const * argument)
{
	/*****************开始等待一段时间****************/
	vTaskDelay(GIMBAL_TASK_INIT_TIME);
  /****************初始化**************************/
	Gimbal_Int(&gimbal_control);
  //射击初始化
  shoot_init();
	Calibration_Init(&cailbration_flag);
	/****************************************/	
  while (1)
  {
			Gimbal_Behavior(&gimbal_control);
			gimbal_mode_change_control_transit(&gimbal_control); //控制模式切换 控制数据过渡
			gimbal_feedback_update(&gimbal_control);
		
			gimbal_set_control(&gimbal_control);                 //设置云台控制量
			Gimbal_PID(&gimbal_control);                         //云台控制PID计算
			shoot_control_loop();        										     //射击任务控制循环
		
			CAN_cmd_yaw_gimbal(gimbal_control.gimbal_yaw_motor.given_current);
		  CAN_cmd_pitch_gimbal(gimbal_control.gimbal_pitch_motor.given_current,shoot_control.barrel_given_current);

			if(!Debug_switch(&cailbration_flag))
			  CAN_cmd_cannon(shoot_control.fire1_current, shoot_control.fire2_current, shoot_control.trigger_current, 0);//+ - - 0
			else 
				CAN_cmd_cannon(0,0,0,0);
			
				/***********************电容板通讯************************/	
				//gimbal_control.power_limit=RM_Referee.robot_state.chassis_power_limit;
				gimbal_control.power_limit=80; //之前调试使用
       		    power_flag=gimbal_control.power_limit*100+100;//RM_Referee.power_heat_data_t.chassis_power_buffer;
//				power_flag=gimbal_control.power_limit*100;	
				CAN_cmd_cap_flag(chassis_X_speed_temp,
												 chassis_Y_speed_temp,
												 sup_cap.remake_flag,
												 sup_cap.sit_flag,
												 sup_cap.stand_flag,
													sup_cap.mode,
													sup_cap.sport_flag,
													sup_cap.tl_flag,
													sup_cap.side_flag,
													sup_cap.jump_flag,
													power_flag,sup_cap.reborn_flag,sup_cap.protect_flag);
	  		/*********************************************************/
			  decet_flag.gimbal_count++;
			  vTaskDelay(GIMBAL_CONTROL_TIME);
					
#if INCLUDE_uxTaskGetStackHighWaterMark
	gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
  }
}
/****************************************************************************************************/
/*********************************************************************************/
/*                               云台初始化                                       */
/*********************************************************************************/
static void Gimbal_Int(gimbal_control_t *init)
{
	  static const fp32 Pitch_speed_pid[3]    = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3]      = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
		
		static const fp32 Pitch_AC_speed_pid[3] = {PITCH_AC_SPEED_PID_KP, PITCH_AC_SPEED_PID_KI, PITCH_AC_SPEED_PID_KD};
    static const fp32 Yaw_AC_speed_pid[3]   = {YAW_AC_SPEED_PID_KP,   YAW_AC_SPEED_PID_KI, YAW_AC_SPEED_PID_KD};

		static const fp32 Pitch_lqr_CurrentK[4] = PITCH_CURRENT_LQR_K;
		static const fp32 Yaw_lqr_CurrentK[4] = YAW_CURRENT_LQR_K;		
		static const fp32 Pitch_PC_lqr[4] = {PITCH_GYRO_ABSOLUTE_LQR_PC_KP, PITCH_GYRO_ABSOLUTE_LQR_PC_KI, PITCH_SPEED_LQR_PC_KP, PITCH_CURRENT_LQR_PC_KP};
		static const fp32 Yaw_PC_lqr[4] = {YAW_GYRO_ABSOLUTE_LQR_PC_KP, YAW_GYRO_ABSOLUTE_LQR_PC_KI, YAW_SPEED_LQR_PC_KP, yaw_CURRENT_LQR_PC_KP};
		
		//遥控器输入滤波
		//KalmanCreate(&K_remo,0.002f,5.0f);
		//KalmanCreate(&K_remo,0.002f,5.0f);
		KalmanCreate(&K_yaw_Auto,  5.0f, 1.0f);
        KalmanCreate(&K_pitch_Auto,5.0f, 1.0f);
		//TD_Init(&TD_yaw,0.2f ,2.0f);
		//TD_Init(&TD_pitch,0.2f ,2.0f);
    //电机数据指针获取
    init->gimbal_yaw_motor.gimbal_motor_measure   = get_6020motor_measure_point(3);
    init->gimbal_pitch_motor.gimbal_motor_measure = get_6020motor_measure_point(4);
    //陀螺仪数据指针获取
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point  = get_gyro_data_point();
		
    //遥控器数据指针获取
    init->gimbal_rc_ctrl         = get_remote_control_point();
    //初始化电机模式
    init->gimbal_yaw_motor.gimbal_motor_mode   = init->gimbal_yaw_motor.last_gimbal_motor_mode   =     GIMBAL_MOTOR_ENCONDE;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode =     GIMBAL_MOTOR_ENCONDE;
		/**************************************************遥控模式*************************************************************/
	  // 初始化yaw/pitch电机PC_lqr
	  LQR_init(&init->gimbal_pitch_motor.gimbal_motor_PC_lqr, Pitch_PC_lqr, Pitch_lqr_CurrentK, PITCH_GYRO_ABSOLUTE_LQR_PC_MAX_ERR, PITCH_PC_MAX_OUT);
	  LQR_init(&init->gimbal_yaw_motor.gimbal_motor_PC_lqr, Yaw_PC_lqr, Yaw_lqr_CurrentK, YAW_GYRO_ABSOLUTE_LQR_PC_MAX_ERR, YAW_PC_MAX_OUT);
   /**************************************************自瞄模式*************************************************************/
	 //初始化滤波器
	  PC_filter_Init();
		//初始化二维卡尔曼
		kalman2_init(&tast_K2);
	 //初始化yaw电机pid
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_AC_absolute_angle_pid, YAW_AC_ANGLE_PID_MAX_OUT,   YAW_AC_ANGLE_PID_MAX_IOUT,   YAW_AC_ANGLE_PID_KP,   YAW_AC_ANGLE_PID_KI,  YAW_AC_ANGLE_PID_KD);
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_AC_gyro_pid,   PID_POSITION, Yaw_AC_speed_pid,   YAW_AC_SPEED_PID_MAX_OUT, YAW_AC_SPEED_PID_MAX_IOUT);
    //初始化pitch电机pid
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_AC_relative_angle_pid, PITCH_AC_ANGLE_PID_MAX_OUT, PITCH_AC_ANGLE_PID_MAX_IOUT, PITCH_AC_ANGLE_PID_KP, PITCH_AC_ANGLE_PID_KI, PITCH_AC_ANGLE_PID_KD);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_AC_gyro_pid, PID_POSITION, Pitch_AC_speed_pid, PITCH_AC_SPEED_PID_MAX_OUT, PITCH_AC_SPEED_PID_MAX_IOUT);
		/*********************************************************************************************************************/
		
		//清除所有PID
    gimbal_total_pid_clear(init);
    gimbal_feedback_update(init);	

    init->gimbal_yaw_motor.max_angle=   65535.0f;
		init->gimbal_yaw_motor.min_angle=  -65535.0f;
 				
	  init->gimbal_pitch_motor.max_angle= 0.34f;//待测
		init->gimbal_pitch_motor.min_angle=-0.454f;
	
    init->gimbal_yaw_motor.absolute_angle_set1 =   0;
    init->gimbal_yaw_motor.relative_angle_set1 =   0;
		
    init->gimbal_pitch_motor.absolute_angle_set1 = 0;
    init->gimbal_pitch_motor.relative_angle_set1 = 0;
		
		sup_cap.High=180;
		sup_cap.protect_flag=1;
}
/*************************************************************************/
/*                            云台行为                                   */
/*************************************************************************/
static void Gimbal_Behavior(gimbal_control_t *gimbal_mode_set)
{
/******************************行为层*******************************************/
    if (gimbal_mode_set == NULL)
    {
        return;
    }
	jump_flag=0;

		gimbal_behaviour_last = gimbal_behaviour;
      //开关控制 云台状态
    if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
			if(cailbration_flag.gyro_flag||switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CANNON_MODE_CHANNEL])||switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CANNON_MODE_CHANNEL]))
				gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
			else
        gimbal_behaviour = GIMBAL_ZERO_FORCE;// GIMBAL_RELATIVE_ANGLE;
    }
    else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
		
			if(switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CANNON_MODE_CHANNEL]))
			{   test_fire_flag = 0; //暂时关闭
			    if(jump_delay_flag==1)
				jump_flag=1;
			}
			else test_fire_flag = 0;
			
			if(rc_ctrl.mouse.press_r==1)
				gimbal_behaviour = GIMBAL_ABSOLUTE_AUTOMA_ANGLE;
			else
        gimbal_behaviour =  GIMBAL_ABSOLUTE_ANGLE;
    }
    else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
		
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }
		else
			gimbal_behaviour = GIMBAL_ZERO_FORCE;
		/************************整车模式层*********************************/
		
		/******************************move层*******************************************/
    //根据云台行为状态机设置电机状态机
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode   = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    { 
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode   = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;//GIMBAL_MOTOR_ENCONDE;
    }
		else if (gimbal_behaviour == GIMBAL_ABSOLUTE_AUTOMA_ANGLE )
		{
		    gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode   = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
		}
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode   = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
}
/*************************************************************************/
/*                            云台数据更新                                */
/*************************************************************************/
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
	static uint16_t num=0;
	static uint16_t gyro_num=0;
	static uint16_t shoot_temp=0;
	static uint16_t protect_temp=0,side_temp=0,sit_temp=0,common_temp=0,stand_temp=0,remake_temp=0,mode_temp=0,jump_temp=0; 
	fp32 gyro_temp=0;
    feedback_update->gimbal_pitch_motor.motor_gyro 				= 	*(feedback_update->gimbal_INT_gyro_point + 0);
	  feedback_update->gimbal_pitch_motor.absolute_angle 		= 	*(feedback_update->gimbal_INT_angle_point + 0);
    feedback_update->gimbal_pitch_motor.relative_angle		= 	pitch_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd);
	  feedback_update->gimbal_pitch_motor.absolute_current 	= 	feedback_update->gimbal_pitch_motor.gimbal_motor_measure->given_current*0.01f;
	
		feedback_update->gimbal_yaw_motor.absolute_angle_last = 	feedback_update->gimbal_yaw_motor.absolute_angle;
	  //feedback_update->gimbal_yaw_motor.absolute_angle_offest_sum +=feedback_update->gimbal_yaw_motor.absolute_angle_offest;
    feedback_update->gimbal_yaw_motor.absolute_angle 			= 	*(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET)+yaw_count*2*PI -feedback_update->gimbal_yaw_motor.absolute_angle_offest_sum;
		feedback_update->gimbal_yaw_motor.absolute_current 		= 	feedback_update->gimbal_yaw_motor.gimbal_motor_measure->given_current*0.01f;
		
    feedback_update->gimbal_yaw_motor.relative_angle 			= 	yaw_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd);
	  feedback_update->gimbal_yaw_motor.motor_gyro_relative = 	cos(feedback_update->gimbal_pitch_motor.relative_angle) *0.1047197f*(fp32)feedback_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;
		feedback_update->gimbal_yaw_motor.motor_gyro 					=		cos(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + 2));
	  
//		PC_data_analyz(&PC_get_data_analyz,&PC_get_data);//小电脑数据分析+
//		gyro_temp = feedback_update->gimbal_yaw_motor.motor_gyro;
//	  if(gyro_temp>2)gyro_temp=2;
//  	else if(gyro_temp<-2)gyro_temp=-2;
	
//		if(gimbal_behaviour == GIMBAL_ZERO_FORCE && feedback_update->gimbal_rc_ctrl->rc.ch[0]==-660&&feedback_update->gimbal_rc_ctrl->rc.ch[1]==660&&
//				feedback_update->gimbal_rc_ctrl->rc.ch[2]==660&&feedback_update->gimbal_rc_ctrl->rc.ch[3]==660)//进入重力补偿校准
//		{
//			if(gyro_num<2000)gyro_num++;
//			else { gyro_num=0;	cailbration_flag.gyro_flag=1;}
//		}
			/*************速度*****************/
			if(sup_cap.side_flag==0)
			{//正着跑是ws
				chassis_X_speed_temp=rc_ctrl.rc.ch[3];
				if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)chassis_X_speed_temp=550;//500
				else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)chassis_X_speed_temp=-550;	
			}
			else
			{//侧着跑是ad
				chassis_X_speed_temp=rc_ctrl.rc.ch[2];
				if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)chassis_X_speed_temp=500;
				else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)chassis_X_speed_temp=-500;
			}
		  /*************小陀螺***************/
			if((rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL) || (rc_ctrl.rc.ch[4]<=-300))
				sup_cap.tl_flag = 1;
			else 
				sup_cap.tl_flag = 0;
			/*************运动模式***************/
			if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT )
				sup_cap.sport_flag = 1;
			else
				sup_cap.sport_flag = 0;
			/*************死亡复活模式***************/
//			if((!RM_Referee.robot_state.mains_power_chassis_output) && ((RM_Referee.game_state.game_progress>>4)==4))
//				sup_cap.reborn_flag=1;
//			else
				sup_cap.reborn_flag=0;
			/*************跳跃模式***************/
			//两种模式
			if((jump_flag==1) && sup_cap.jump_flag==0 && jump_temp>100)
			{sup_cap.jump_flag=1;jump_temp=0;jump_delay_flag=0;}//缩腿
			else if(sup_cap.jump_flag==1 &&	jump_temp>120)
			{sup_cap.jump_flag=2;jump_temp=0;jump_delay_flag=0;}//伸腿
			else if(sup_cap.jump_flag==2 &&	jump_temp>275)
			{sup_cap.jump_flag=3;jump_temp=0;jump_delay_flag=0;}//收腿
			else if(sup_cap.jump_flag==3 &&	jump_temp>100)
			{sup_cap.jump_flag=0;jump_temp=0;jump_delay_flag=1;}//恢复正常
  		if(jump_temp<500)jump_temp++;	
			else jump_temp=0;	
			
//			if((rc_ctrl.key.v & KEY_PRESSED_OFFSET_E) && sup_cap.jump_flag==0 && jump_temp>50)
//			{sup_cap.jump_flag=1;jump_temp=0;}//伸腿
//			else if(sup_cap.jump_flag==1 &&	jump_temp>250)
//			{sup_cap.jump_flag=2;jump_temp=0;}//收腿
//			else if(sup_cap.jump_flag==2 &&	jump_temp>150)
//			{sup_cap.jump_flag=0;jump_temp=0;}//恢复正常				
//  		if(jump_temp<500)jump_temp++;	
//			else jump_temp=0;	
			/*************侧面对敌***************/
			if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q && sup_cap.side_flag==0 && side_temp>300)
			{sup_cap.side_flag=1;side_temp=0;}
			else if((rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q || rc_ctrl.key.v & KEY_PRESSED_OFFSET_W || rc_ctrl.key.v & KEY_PRESSED_OFFSET_S) && sup_cap.side_flag==1 &&	side_temp>300)
			{sup_cap.side_flag=0;side_temp=0;}
  		if(side_temp<1000)side_temp++;
			/*************切换射速***************/
			if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_F && gimbal_control.shoot_mood==0 && shoot_temp>300)
			{gimbal_control.shoot_mood=1;shoot_temp=0;}
			else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_F && gimbal_control.shoot_mood==1 &&	shoot_temp>300)
			{gimbal_control.shoot_mood=0;shoot_temp=0;}
  		if(shoot_temp<1000)shoot_temp++;			
//			/*************保护模式***************/
			if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_E && sup_cap.protect_flag==0 && protect_temp>300)
			{sup_cap.protect_flag=1;protect_temp=0;}
			else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_E && sup_cap.protect_flag==1 &&	protect_temp>300)
			{sup_cap.protect_flag=0;protect_temp=0;}
  		if(protect_temp<1000)protect_temp++;							
			/**************蹲下模式****************/
			if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_X && sup_cap.sit_flag==0 && sit_temp>300 && sup_cap.stand_flag==0)
			{sit_temp=0;sup_cap.sit_flag=1;}
			else if((rc_ctrl.key.v & KEY_PRESSED_OFFSET_X || rc_ctrl.key.v & KEY_PRESSED_OFFSET_W || rc_ctrl.key.v & KEY_PRESSED_OFFSET_S ) && sup_cap.sit_flag==1 && sit_temp>300)
			{sit_temp=0;sup_cap.sit_flag=0;}
  		if(sit_temp<1000)sit_temp++;
			/**************起立模式****************/
			if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_C && sup_cap.stand_flag==0 && stand_temp>300 && sup_cap.sit_flag==0)
			{sup_cap.stand_flag=1;stand_temp=0;}
			else if((rc_ctrl.key.v & KEY_PRESSED_OFFSET_C || rc_ctrl.key.v & KEY_PRESSED_OFFSET_W || rc_ctrl.key.v & KEY_PRESSED_OFFSET_S ) && sup_cap.stand_flag==1 && stand_temp>300)
			{sup_cap.stand_flag=0;stand_temp=0;}//起立过程中，只要移动就退出
			if(stand_temp<1000)stand_temp++;
			/**************底盘重置****************/
			if((rc_ctrl.key.v & KEY_PRESSED_OFFSET_B || (rc_ctrl.rc.ch[4]>=300)) && sup_cap.remake_flag==0 && remake_temp>1000)
			{sup_cap.remake_flag=1;remake_temp=0;}
			else if(sup_cap.remake_flag==1 && remake_temp>1000)
			{sup_cap.remake_flag=0;remake_temp=0;}
			if(remake_temp<2000)remake_temp++;
			/**************杀死线程**************************/
			if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_V )
			{Pawn_mode = 1;mode_temp=0;}//kill pc线程 1s
			else if(Pawn_mode == 1 && mode_temp>100)
			{Pawn_mode = 0;mode_temp=0;}		
			/****************大幅模式*******************/
		  if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_G && mode_temp>1000 && Pawn_mode==0)
			{Pawn_mode = 2;mode_temp=0;}//开启小幅
			else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_G && mode_temp>1000 && Pawn_mode==2)
			{Pawn_mode = 3;mode_temp=0;}//开启大幅
			else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_G && mode_temp>1000 && Pawn_mode==3)
			{Pawn_mode = 0;mode_temp=0;}//关闭大幅
			if(mode_temp<3000)mode_temp++;			
		/****************************************/
			sup_cap.mode = feedback_update->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL];
}
/**************************************************************************/
/*                         模式切换时数据过渡                              */
/**************************************************************************/
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set1 = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set1 = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    
		gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;
    //pitch电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set1 = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set1 = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
   				
		if(gimbal_behaviour_last == GIMBAL_ABSOLUTE_AUTOMA_ANGLE && gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
		{
			gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set1 =		gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
			gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set1   =		gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
			red_linght_open();
		}
		if(gimbal_behaviour_last == GIMBAL_ABSOLUTE_ANGLE && gimbal_behaviour == GIMBAL_ABSOLUTE_AUTOMA_ANGLE)
		{
			gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set2 =		gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
			gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set2   =		gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
			red_linght_close();
		}
		
		gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = 	gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}
/****************************************************************************************/
/*                                              move set                                */
/****************************************************************************************/
float yaw_K;
static void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{
//遥控器传入
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {//开环模式下直接获取遥控器值
			float yaw_channel = 0, pitch_channel = 0;
			rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
			rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
			*add_yaw   = yaw_channel*10.0f;
			*add_pitch = pitch_channel*10.0f;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE )
    {
			float yaw_channel = 0, pitch_channel = 0;
			rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
			rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);      
			//每次输入yaw最大为1°，pitch为0.5°
			*add_yaw   =  yaw_channel   * YAW_RC_SEN   - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
			*add_pitch =  pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
			;
    }
}

fp32 add_yaw_angle2 = 0.0f;
fp32 add_pitch_angle2 = 0.0f;
static void gimbal_set_control(gimbal_control_t *set_control)
{//求set
    if (set_control == NULL)
    {
        return;
    }
    fp32 add_yaw_angle1 = 0.0f;
    fp32 add_pitch_angle1 = 0.0f;

		uint8_t PC_enable_flag=0;
		gimbal_behaviour_control_set(&add_yaw_angle1, &add_pitch_angle1, set_control);
		
//		if (gimbal_behaviour == GIMBAL_ABSOLUTE_AUTOMA_ANGLE )//自瞄不用遥控器，使用PC传入数据			
		if(trajectory_analyz(&add_yaw_angle2, &add_pitch_angle2, &PC_get_data_analyz)==0x30)
		{
				PC_enable_flag=0;
		}
		else if(trajectory_analyz(&add_yaw_angle2, &add_pitch_angle2, &PC_get_data_analyz)==0x31)
		{
				PC_enable_flag=1;
		}
/*******************************************YAW**************************************************/
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {//raw模式下，直接发送控制值
        set_control->gimbal_yaw_motor.raw_cmd_current = 0;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {//gyro模式下，陀螺仪角度控制
//				if(RM_Referee.robot_state.remain_HP==0){
//						set_control->gimbal_yaw_motor.absolute_angle_set1 = set_control->gimbal_yaw_motor.absolute_angle;
//				} else {
//						set_control->gimbal_yaw_motor.absolute_angle_set1 += add_yaw_angle1;//积分
//						if(PC_enable_flag)
//						{
//							set_control->gimbal_yaw_motor.absolute_angle_set2 = set_control->gimbal_yaw_motor.absolute_angle + add_yaw_angle2;
//							set_control->gimbal_yaw_motor.absolute_angle_set2 = KalmanFilter(&K_yaw_Auto,set_control->gimbal_yaw_motor.absolute_angle_set2);
//						}				
//				}
					set_control->gimbal_yaw_motor.absolute_angle_set1 += add_yaw_angle1;//积分
					if(PC_enable_flag)
					{
						set_control->gimbal_yaw_motor.absolute_angle_set2 = set_control->gimbal_yaw_motor.absolute_angle + add_yaw_angle2;
						set_control->gimbal_yaw_motor.absolute_angle_set2 = KalmanFilter(&K_yaw_Auto,set_control->gimbal_yaw_motor.absolute_angle_set2);
					}						
    }
/***************************************PITCH*********************************************/
    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {//raw模式下，直接发送控制值
        set_control->gimbal_pitch_motor.raw_cmd_current = 0;
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {//gyro模式下，陀螺仪角度控制
			set_control->gimbal_pitch_motor.absolute_angle_set1 += add_pitch_angle1;//积分
			if(PC_enable_flag)
			{
			 set_control->gimbal_pitch_motor.absolute_angle_set2 = set_control->gimbal_pitch_motor.absolute_angle + add_pitch_angle2;
		   set_control->gimbal_pitch_motor.absolute_angle_set2 = KalmanFilter(&K_pitch_Auto,set_control->gimbal_pitch_motor.absolute_angle_set2);
			}
			if(set_control->gimbal_pitch_motor.absolute_angle_set1>=set_control->gimbal_pitch_motor.max_angle)      set_control->gimbal_pitch_motor.absolute_angle_set1=set_control->gimbal_pitch_motor.max_angle;
			else if(set_control->gimbal_pitch_motor.absolute_angle_set1<=set_control->gimbal_pitch_motor.min_angle) set_control->gimbal_pitch_motor.absolute_angle_set1=set_control->gimbal_pitch_motor.min_angle;
			if(set_control->gimbal_pitch_motor.absolute_angle_set2>=set_control->gimbal_pitch_motor.max_angle)      set_control->gimbal_pitch_motor.absolute_angle_set2=set_control->gimbal_pitch_motor.max_angle;
			else if(set_control->gimbal_pitch_motor.absolute_angle_set2<=set_control->gimbal_pitch_motor.min_angle) set_control->gimbal_pitch_motor.absolute_angle_set2=set_control->gimbal_pitch_motor.min_angle;
		 }
		
}
/****************************************************************************************************/
/*                                              PID运算                                             */
/****************************************************************************************************/
static void Gimbal_PID(gimbal_control_t *control_loop)
{//电机运算和控制
	 if (control_loop == NULL)
    {
        return;
    }
    if(gimbal_behaviour == GIMBAL_ABSOLUTE_AUTOMA_ANGLE)
		{
				control_loop->gimbal_yaw_motor.given_current=	gimbal_LQR_calc(LQR_ACyaw[0],LQR_ACyaw[1],LQR_ACyaw[2],
																																							 control_loop->gimbal_yaw_motor.absolute_angle,
																																							 control_loop->gimbal_yaw_motor.motor_gyro,
																																							 control_loop->gimbal_yaw_motor.absolute_angle_set2,0);

				control_loop->gimbal_pitch_motor.given_current= gimbal_PI_LQR(LQR_PI_pitch_AC[3],LQR_PI_pitch_AC[4],LQR_PI_pitch_AC[0],LQR_PI_pitch_AC[1],LQR_PI_pitch_AC[2],//PI_LQR Serise
																																			(fp32) control_loop->gimbal_pitch_motor.absolute_angle,
																																			(fp32) control_loop->gimbal_pitch_motor.motor_gyro,
																																			(fp32) control_loop->gimbal_pitch_motor.absolute_current,
  																																		(fp32) control_loop->gimbal_pitch_motor.absolute_angle_set2);
		}
		else
		{
//			if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
//				control_loop->gimbal_yaw_motor.given_current = (int16_t)control_loop->gimbal_yaw_motor.raw_cmd_current;
//			

//			else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
//			{		
//				control_loop->gimbal_yaw_motor.given_current= gimbal_PI_LQR(LQR_PI_yaw[3],LQR_PI_yaw[4],LQR_PI_yaw[0],LQR_PI_yaw[1],LQR_PI_yaw[2],//PI_LQR Serise
//																																			(fp32) control_loop->gimbal_yaw_motor.absolute_angle,
//																																			(fp32) control_loop->gimbal_yaw_motor.motor_gyro,
//																																			(fp32) control_loop->gimbal_yaw_motor.absolute_current,
//  																																		(fp32) control_loop->gimbal_yaw_motor.absolute_angle_set1);
//			}
//			else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
//			{				control_loop->gimbal_yaw_motor.motor_gyro_set =  gimbal_PID_calc(&control_loop->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, control_loop->gimbal_yaw_motor.relative_angle, control_loop->gimbal_yaw_motor.relative_angle_set1,1);
//							control_loop->gimbal_yaw_motor.current_set    =  PID_calc(&control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid,control_loop->gimbal_yaw_motor.motor_gyro,control_loop->gimbal_yaw_motor.motor_gyro_set);
//							control_loop->gimbal_yaw_motor.given_current  =  (int16_t)(control_loop->gimbal_yaw_motor.current_set);
//			}
//			
//			if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
//			{
//				control_loop->gimbal_pitch_motor.given_current = control_loop->gimbal_pitch_motor.raw_cmd_current+(int16_t)(-control_loop->gimbal_pitch_motor.absolute_angle*2000.2f-2000.0f);
//			 
//			}
//			else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
//			{
//				control_loop->gimbal_pitch_motor.given_current = gimbal_PI_LQR(LQR_PI_pitch[3],LQR_PI_pitch[4],LQR_PI_pitch[0],LQR_PI_pitch[1],LQR_PI_pitch[2],//PI_LQR Serise
//																																			(fp32) control_loop->gimbal_pitch_motor.absolute_angle,(fp32) control_loop->gimbal_pitch_motor.motor_gyro,(fp32) control_loop->gimbal_pitch_motor.absolute_current,
//  																																		(fp32) control_loop->gimbal_pitch_motor.absolute_angle_set1);
//			}
//					
//			else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
//			{
//				control_loop->gimbal_pitch_motor.motor_gyro_set = gimbal_PID_calc(&control_loop->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, control_loop->gimbal_pitch_motor.relative_angle, control_loop->gimbal_pitch_motor.relative_angle_set1,0);
//				control_loop->gimbal_pitch_motor.current_set =    PID_calc(&control_loop->gimbal_pitch_motor.gimbal_motor_gyro_pid,control_loop->gimbal_pitch_motor.motor_gyro,control_loop->gimbal_pitch_motor.motor_gyro_set);
//				control_loop->gimbal_pitch_motor.given_current = (int16_t)(control_loop->gimbal_pitch_motor.current_set);
//			}

				control_loop->gimbal_yaw_motor.given_current= gimbal_PI_LQR(LQR_PI_yaw[3],LQR_PI_yaw[4],LQR_PI_yaw[0],LQR_PI_yaw[1],LQR_PI_yaw[2],//PI_LQR Serise
																																			(fp32) control_loop->gimbal_yaw_motor.absolute_angle,
																																			(fp32) control_loop->gimbal_yaw_motor.motor_gyro,
																																			(fp32) control_loop->gimbal_yaw_motor.absolute_current,
  																																		(fp32) control_loop->gimbal_yaw_motor.absolute_angle_set1);
				control_loop->gimbal_pitch_motor.given_current = gimbal_PI_LQR(LQR_PI_pitch[3],LQR_PI_pitch[4],LQR_PI_pitch[0],LQR_PI_pitch[1],LQR_PI_pitch[2],//PI_LQR Serise
																																			(fp32) control_loop->gimbal_pitch_motor.absolute_angle,(fp32) control_loop->gimbal_pitch_motor.motor_gyro,(fp32) control_loop->gimbal_pitch_motor.absolute_current,
  																																		(fp32) control_loop->gimbal_pitch_motor.absolute_angle_set1);
		}
}
/****************************************************************************************/
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set,uint8_t type)
{
    fp32 err,error_delta;
	static fp32 err_sum;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;
		err = set - get;

    error_delta = err - pid->err;
    err_sum += err;

    pid->Pout = pid->kp * err;
    pid->Iout = pid->ki * err_sum;
    pid->Dout = pid->kd * error_delta;
		
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
		pid->err = err;
    return pid->out;		
}


int16_t gimbal_LQR_calc(fp32 K1,fp32 K2,fp32 K0,fp32 angle_get,fp32 speed_get,fp32 angle_set,fp32 speed_set)
{
	fp32 accel=0.0f;
	int16_t current=0;
	static fp32 angle_int=0;
	fp32 K0_angle=0;
	
	angle_int += angle_get-angle_set;
	abs_limit(&angle_int,5.0f);
	K0_angle  = angle_int*K0;
	abs_limit(&K0_angle,5000.0f);
	
	accel=(K1*(angle_set-angle_get)-K2*speed_get-K0_angle);//通过角度和速度求出理论要求输入的角加速度
	abs_limit(&accel, 25000.0f);
	current=(int16_t)accel;//理论上说这里不需要加权
	return current;
}
int16_t gimbal_PI_LQR(fp32 KP,fp32 KI,fp32 K1,fp32 K2,fp32 K3,//PI_LQR Serise
										  fp32 angle_get,fp32 speed_get,fp32 current_get,
									    fp32 angle_set)
{
	fp32 current=0,first_out=0;
	fp32 KP_angle;
	static fp32 KI_angle=0;
	KP_angle=(angle_set-angle_get);
	KI_angle+=(angle_set-angle_get);
	abs_limit(&KI_angle,5.0f);
	
	first_out = KP*KP_angle+KI*KI_angle;
	
	current=(first_out - (K2*speed_get + K3*(-current_get)));//通过角度和速度求出理论要求输入的角加速度
	abs_limit(&current,30000.0f);
	return (int16_t)current;
}

static fp32 yaw_ecd_to_angle_change(uint16_t ecd)
{
    int32_t relative_ecd =ecd- HAND_ENCONDE+ECD_RANGE;
		if(relative_ecd>ECD_RANGE)	
			relative_ecd-=ECD_RANGE;
	  
	  if(relative_ecd<-HALF_ECD_RANGE)
		relative_ecd=relative_ecd+ECD_RANGE;
		
		if(relative_ecd>HALF_ECD_RANGE)
		relative_ecd=relative_ecd-ECD_RANGE;
		
    return relative_ecd*ENCONDE_2_ANGLE;
}


static fp32 pitch_ecd_to_angle_change(uint16_t ecd)
{
     int32_t relative_ecd =-ecd;
			
	  if(relative_ecd<0)
		relative_ecd=relative_ecd+ECD_RANGE;
		
		if(relative_ecd>HALF_ECD_RANGE)
		relative_ecd=relative_ecd-ECD_RANGE;
		
    return -relative_ecd*ENCONDE_2_ANGLE+PITCH_COM;
}


static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

static float Matlab_Add_Pitch(int distance)
{
	float temp = 0.00000000004602f*distance*distance*distance - 0.00000002017f*distance*distance + 0.0001028f*distance + 0.08787f + 0.02f;
	if(temp>=0.14f) temp=0.14f + 0.02f;
	else if(temp<=0.098f) temp=0.098f + 0.02f;
	return temp;
}

const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

void red_linght_close()
{
	HAL_GPIO_WritePin(RED_LIGHT_GPIO_Port,RED_LIGHT_Pin,0);
}
void red_linght_open()
{
	HAL_GPIO_WritePin(RED_LIGHT_GPIO_Port,RED_LIGHT_Pin,1);
}
void BEEP_close()
{
	TIM4->CCR3 = 0;
}
void BEEP_open()
{
	TIM4->CCR3 = 10000;
}
