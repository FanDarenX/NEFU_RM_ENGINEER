#include "classis_task.h"
#include "FreeRTOS.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "pid.h"
#include "main.h"
#include "Com_Pro.h"
#include "task.h"
#include "bsp_math.h"
#include "math.h"
#include "ANO_DT.h"
#include "INS_task.h"
#include "user_lib.h"
#include "decet_task.h"
#include "bsp_usart.h"
#include "referee.h"
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
		#define ABS(x) (((x) > 0) ? (x) : (-(x)))
/*****************************底盘结构体**********************************/
extern gimbal_control_t gimbal_control;
chassis_move_t chassis_move;
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif
/******************************功率控制**************************************/
fp32 POWER_LIMIT =          59.0f;
fp32 WARNING_POWER  =       60.0f;
fp32 BUFFER_TOTAL_CURRENT_LIMIT = 10000.0f;
fp32 POWER_TOTAL_CURRENT_LIMIT  = 12000.0f;
fp32 WARNING_POWER_BUFF =   40.0f;

uint8_t cf_flag=0; 
fp32 chassis_power ;
fp32 chassis_power_buffer ;
fp32 limit;
float total_current_limit;
			 fp32 current_scale;//超功率使用缓冲能量时的减功率比
/***********************************************************************/
void Classis_Int(chassis_move_t *chassis_move_init)
{
	int8_t i=0;
	//底盘速度环pid值
	const static fp32 motor_speed_pid[3] =     {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	//底盘角度pid值
	const static fp32 chassis_yaw_pid[3] =     {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
	const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
	const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
	const static fp32 chassis_w_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
	/*******************************底盘电机PID初始化**************************************/	
	//底盘开机状态为空挡
	chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
	//获取遥控器指针
	chassis_move_init->chassis_RC = get_remote_control_point();
  //获取陀螺仪姿态角指针
	chassis_move_init->chassis_INS_angle = get_INS_angle_point();
	//获取云台电机数据指针
	chassis_move_init->chassis_yaw_motor   = get_yaw_motor_point();
	chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
//	chassis_move_init->chassis_cap.cap_measure  = get_cap_measure_point();
	//底盘电机PID装填
  for (i = 0; i < 4; i++)
	{
		PID_init(&chassis_move_init->motor_speed_pid[i], PID_DELTA, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_3508motor_measure_point(i); 		//获取ID为0号的电机数据指针
	}
//	chassis_move_init->vx_max_speed=3000;
//	chassis_move_init->vy_max_speed=2000;
//		chassis_move_init->vx_min_speed=-3000;
//	chassis_move_init->vy_min_speed=-2000;
	//初始化滤波器
	 first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
   first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
		 first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_wz, CHASSIS_CONTROL_TIME, chassis_w_order_filter);
	//初始化角度PID
	PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	//无电流输出
  for (i = 0; i < 4; i++)
	{chassis_move.motor_chassis[i].give_current=0;}
	/*************************************************************************************/
}

/********************************************************************************************************/
/**********************************************行动选择**************************************************/
/********************************************************************************************************/
void Classis_Behavior(chassis_move_t *chassis_move_mode)
{//行为模式 
	if (chassis_move_mode == NULL)
    {
        return;
    }
/**********************************行为层******************************/
    if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
			chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;//遥控器控制
    else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
		{
			if(chassis_move_mode->chassis_RC->key.v & TL )
			chassis_behaviour_mode =CHASSIS_NO_FOLLOW_YAW;
			else 
			chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;//跟云台
		
		}
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
      chassis_behaviour_mode = CHASSIS_NO_MOVE;//
		else
			chassis_behaviour_mode = CHASSIS_NO_MOVE;//蚌住
/**********************************移动层******************************/
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
    }
}	
/********************************************************************************************************/
/*************************************模式切换过程中的传递*************************************************/
/********************************************************************************************************/
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
 if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //change to follow gimbal angle mode
    //切入跟随云台模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0;
    }
    //切入跟随底盘角度模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //切入不跟随云台模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}
/********************************************************************************************************/
/******************************************电机车体速度更新***********************************************/
/********************************************************************************************************/
void Classis_feedback_update(chassis_move_t *chassis_move_update)
{//传感器采集信息更新
    if (chassis_move_update == NULL)
    {return; }
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
		//更新电机速度，加速度是速度的PID微分
		chassis_move_update->motor_chassis[i].speed = chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
		chassis_move_update->motor_chassis[i].real_speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN*chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
    }
    //更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) *0.25f;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) *0.25f;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) *0.25f / MOTOR_DISTANCE_TO_CENTER;

//		chassis_move_update->current_sum=(ABS(chassis_move_update->motor_chassis[0].chassis_motor_measure->given_current)
//																		+ABS(chassis_move_update->motor_chassis[1].chassis_motor_measure->given_current)
//																		+ABS(chassis_move_update->motor_chassis[2].chassis_motor_measure->given_current)
//																		+ABS(chassis_move_update->motor_chassis[3].chassis_motor_measure->given_current))*0.00042f;
		
    //计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) -     chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
		//超级电容信息更新
		chassis_move_update->cap_Value_Bat =     chassis_move_update->chassis_cap.cap_measure->Value_Bat;
		chassis_move_update->cap_Value_Cap =     chassis_move_update->chassis_cap.cap_measure->Value_Cap;
		chassis_move_update->cap_Power_Charge =  chassis_move_update->chassis_cap.cap_measure->Power_Charge;
 		chassis_move_update->cap_Power_Chassis = chassis_move_update->chassis_cap.cap_measure->Power_Chassis;
		
			if(cf_flag==1)
		{
		chassis_move_update->vx_max_speed=5000;
		chassis_move_update->vy_max_speed=3000;
		chassis_move_update->vx_min_speed=-5000;
		chassis_move_update->vy_min_speed=-3000;
		}
		else
		{
		chassis_move_update->vx_max_speed=3000;
		chassis_move_update->vy_max_speed=2000;
		chassis_move_update->vx_min_speed=-3000;
		chassis_move_update->vy_min_speed=-2000;

		}
}
/********************************************************************************************************/
/**********************************************move层选择*************************************************/
/********************************************************************************************************/
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
		if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)//小陀螺
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}
/********************************************************************************************************/
/*****************************************按照move层设置车体运行******************************************/
/********************************************************************************************************/
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    //获取三个控制设置值
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
    //跟随云台模式
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
			  chassis_move_control->wz_set = PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {//“angle_set” 是旋转速度控制
			  chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {//在原始模式，设置值是发送到CAN总线
			  chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
    }
}

/********************************************************************************************************/
/**********************************************速度解算和PID求解******************************************/
/********************************************************************************************************/
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    wheel_speed[0] = -vx_set - vy_set + /*(CHASSIS_WZ_SET_SCALE - 1.0f)*  MOTOR_DISTANCE_TO_CENTER */wz_set;
    wheel_speed[1] = vx_set  - vy_set  + /*(CHASSIS_WZ_SET_SCALE - 1.0f)*  MOTOR_DISTANCE_TO_CENTER */ wz_set;
    wheel_speed[2] = vx_set  + vy_set  + /*(-CHASSIS_WZ_SET_SCALE - 1.0f)*  MOTOR_DISTANCE_TO_CENTER */wz_set;
    wheel_speed[3] = -vx_set + vy_set + /*(-CHASSIS_WZ_SET_SCALE - 1.0f)*  MOTOR_DISTANCE_TO_CENTER*/wz_set;
}
void Classis_PID(chassis_move_t *chassis_move_control_loop)//底盘pid计算与输出
{
	  fp32 max_vector = 0.0f, vector_rate = 0.0f;
	//	fp32 temp = 0.0f;
		fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
		uint8_t i = 0;
		
		chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);
    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {  //raw控制直接返回
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        return;
    }
    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    { chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];}
    //计算pid
    for (i = 0; i < 4; i++)
    { PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);}
    //赋值电流值
    for (i = 0; i < 4; i++)
    {chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);}
}
 fp32 cap_temp=0.0f;

void chassis_power_control(chassis_move_t *chassis_power_control)//功率限制
{
	static int8_t num=0;
	 fp32 total_current =0.0f;

	 total_current_limit =M3505_MOTOR_SPEED_PID_MAX_OUT*4;//总电流的限制
   get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer, &limit);
	//从裁判系统读出当前 功率 缓冲能量 功率限制 缓冲能量在飞坡后为250，平时是60
	//**************************************************************************
	//当cap电压等于电池电压时，为效率最高
	//**************************************************************************
//	if(chassis_power_control->cap_Value_Cap<=16)
//	{
//	  cap_temp=1/(1-(fp32)limit*0.9f+chassis_power_control->cap_Power_Chassis);
//		if(cap_temp>1||cap_temp<0) cap_temp=1;
//		total_current_limit = cap_temp*total_current_limit;
//		 //计算原本电机电流设定
//    for(uint8_t i = 0; i < 4; i++)
//    {total_current += fabs(chassis_power_control->motor_speed_pid[i].out);}
//		
//		if(total_current > total_current_limit)
//    {		chassis_power_control->motor_speed_pid[0].out*=cap_temp;
//        chassis_power_control->motor_speed_pid[1].out*=cap_temp;
//        chassis_power_control->motor_speed_pid[2].out*=cap_temp;
//        chassis_power_control->motor_speed_pid[3].out*=cap_temp;  }
//		else
//		{   chassis_power_control->motor_speed_pid[0].out= chassis_power_control->motor_speed_pid[0].out;
//        chassis_power_control->motor_speed_pid[1].out= chassis_power_control->motor_speed_pid[1].out;
//        chassis_power_control->motor_speed_pid[2].out= chassis_power_control->motor_speed_pid[2].out;
//        chassis_power_control->motor_speed_pid[3].out= chassis_power_control->motor_speed_pid[3].out; }
//	}
//	else 
	//cap电压高于电池电压，此时暂时不对功率进行超级电容方面的功率限制
	{
	  cap_temp=	((fp32)limit/(fp32)chassis_power_control->cap_Power_Chassis);
		
		if(cap_temp>1) cap_temp=1;
		
		if(chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_Q)
		{
			chassis_power_control->motor_speed_pid[0].out= chassis_power_control->motor_speed_pid[0].out;
			chassis_power_control->motor_speed_pid[1].out= chassis_power_control->motor_speed_pid[1].out;
			chassis_power_control->motor_speed_pid[2].out= chassis_power_control->motor_speed_pid[2].out;
			chassis_power_control->motor_speed_pid[3].out= chassis_power_control->motor_speed_pid[3].out;

			cf_flag=1;
		}
		else
		{
			chassis_power_control->motor_speed_pid[0].out*=cap_temp;
			chassis_power_control->motor_speed_pid[1].out*=cap_temp;
			chassis_power_control->motor_speed_pid[2].out*=cap_temp;
			chassis_power_control->motor_speed_pid[3].out*=cap_temp;
			cf_flag=0;			
		}
	} 
			
}

void classis_task(void const * argument)
{
	/*****************开始等待一段时间****************/
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
  /****************初始化**************************/
	Classis_Int(&chassis_move);
	/****************************************/
while (1)
    {
        //设置底盘控制模式
        Classis_Behavior(&chassis_move);
			  //模式切换处理
				chassis_mode_change_control_transit(&chassis_move);
        //底盘数据更新
        Classis_feedback_update(&chassis_move);
        //底盘速度解算
			  chassis_set_contorl(&chassis_move);
        //底盘控制PID计算
        Classis_PID(&chassis_move);
			  //功率控制
			 // chassis_power_control(&chassis_move);
				//发送控制电流
			  CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                        chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
//			 CAN_cmd_chassis(0,0,0,0);

			  decet_flag.chassis_count++;
//   			Waveform_Write();
        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
		/***********************************************************************************/
		//keyboard set speed set-point
    //键盘控制
		vx_set_channel = 5*vx_channel;// * CHASSIS_VX_RC_SEN;
    vy_set_channel = 3*vy_channel;// * -CHASSIS_VY_RC_SEN;
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }
		/***********************************************************************************/
    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = -chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}
void chassis_rc_to_control_vector_xyz(fp32 *vx_set, fp32 *vy_set,fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel,wz_channel;
    fp32 vx_set_channel, vy_set_channel,wz_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL],  vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL],  vy_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);
		
    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
    wz_set_channel = wz_channel * CHASSIS_WZ_RC_SEN;

    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
		first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_wz, wz_set_channel);
    //stop command, need not slow change, set zero derectly
    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }
    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }
		if (wz_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_WZ_RC_SEN && wz_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_WZ_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
		*wz_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out;
}



static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    //遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *angle_set = - gimbal_control.gimbal_yaw_motor.relative_angle;
}

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
		chassis_rc_to_control_vector_rotate(vx_set, vy_set,wz_set, chassis_move_rc_to_vector);
    //chassis_rc_to_control_vector_xyz(vx_set, vy_set, wz_set , chassis_move_rc_to_vector);
}
