/*
 * @Descripttion:
 * @version:
 * @Author: fyh/xqz
 * @Date: 2024
 * @LastEditors: xqz
 * @LastEditTime: 2025
 */

/*
																_                                _
	/\                                                         (_)                              (_)
   /  \     _ __    ___   ___   ______    ___   _ __     __ _   _   _ __     ___    ___   _ __   _   _ __     __ _
  / /\ \   | '__|  / _ \ / __| |______|  / _ \ | '_ \   / _` | | | | '_ \   / _ \  / _ \ | '__| | | | '_ \   / _` |
 / ____ \  | |    |  __/ \__ \          |  __/ | | | | | (_| | | | | | | | |  __/ |  __/ | |    | | | | | | | (_| |
/_/    \_\ |_|     \___| |___/           \___| |_| |_|  \__, | |_| |_| |_|  \___|  \___| |_|    |_| |_| |_|  \__, |
														 __/ |                                                __/ |
														|___/                                                |___/
*/

#include "chassis_task.h"
#include "pid.h"

uint8_t rc_to_chassis = 1; // 遥控器控制速度放大倍数，配合局部增速

uint8_t remote_mode = 0;						   // 0: 键盘控制 1: 遥控器控制
chassis_task_t chassis_speed_task;				   // 滤波器+float xyr
chassis_speed_rel chassis_speed_get;			   // float xyr
Motor_speed Speed_set;							   // float 1~4
ramp_function_source_t chassis_slope[4];		   // 斜坡函数
first_order_filter_type_t first_chassis_mouse;	   // 一阶低通滤波——让键鼠控制下，数据更加平滑
static float chassis_mouse_filter = 1.6666666667f; // 一阶低通滤波参数——越大越稳，但响应速度降低

pid_typedef Chassis_speed_pid[4];

float Set[4] = {0}; // 滤波计算求和电流结果->PID目标值

void chassis_remote_data(chassis_task_t *chassis_task_filter)
{
	first_order_filter_cali(&chassis_task_filter->chassis_first_order_z, (float)chassis_task_filter->chassis_speed.chassis_speed_r); // 一阶低通滤波处理鼠标反馈值
	first_order_filter_cali(&chassis_task_filter->chassis_first_order_x, (float)chassis_task_filter->chassis_speed.chassis_speed_x); // 一阶低通滤波处理鼠标反馈值
	first_order_filter_cali(&chassis_task_filter->chassis_first_order_y, (float)chassis_task_filter->chassis_speed.chassis_speed_y); // 一阶低通滤波处理鼠标反馈值
}

void chassis_init(void)
{
	for (int i = 0; i < 4; i++)
	{
		pid_init(&Chassis_speed_pid[i]);
		Chassis_speed_pid[i].f_param_init(&Chassis_speed_pid[i], 0, 8000, 15900, 5000, 5, 0, 20, 0.01, 15);
		//		first_order_filter_init(&chassis_cmd_vx, CHASSIS_ACCEL_X_NUM, CHASSIS_CONTROL_TIME);//遥控器数据滤波
		//		first_order_filter_init(&chassis_cmd_vy, CHASSIS_ACCEL_Y_NUM, CHASSIS_CONTROL_TIME);
	}
	ramp_init(&chassis_speed_task.chassis_ramp[0], 0.01, 660 * 7, -660 * 7);
	ramp_init(&chassis_speed_task.chassis_ramp[1], 0.01, 660 * 7, -660 * 7);
	ramp_init(&chassis_speed_task.chassis_ramp[2], 0.01, 660 * 7, -660 * 7);						//?
	first_order_filter_init(&chassis_speed_task.chassis_first_order_z, 0.02, chassis_mouse_filter); // 间隔时间->卡顿度||丝滑度，间隔越短越丝滑
	first_order_filter_init(&chassis_speed_task.chassis_first_order_x, 0.02, chassis_mouse_filter);
	first_order_filter_init(&chassis_speed_task.chassis_first_order_y, 0.02, chassis_mouse_filter);
}

void Chassis_Task_os(void)
{
	//	if(!flag)
	//	{
	//		chassis_speed_task.chassis_speed.chassis_speed_x = 0;
	//        chassis_speed_task.chassis_speed.chassis_speed_y = 0;
	//        chassis_speed_task.chassis_speed.chassis_speed_r = 0;
	//	}
	chassis_remote_data(&chassis_speed_task);
	//	flag = 0;
	// if (!key_task.big_chassis_mode)
	// {

	//! 整体乘倍数可以整体加速（下方pid传入），也可局部加速，如Z
	Set[0] = -chassis_speed_task.chassis_first_order_x.out + chassis_speed_task.chassis_first_order_y.out - (chassis_speed_task.chassis_first_order_z.out*4); //*40
	Set[1] = -chassis_speed_task.chassis_first_order_x.out - chassis_speed_task.chassis_first_order_y.out - (chassis_speed_task.chassis_first_order_z.out*4);
	Set[2] = chassis_speed_task.chassis_first_order_x.out - chassis_speed_task.chassis_first_order_y.out - (chassis_speed_task.chassis_first_order_z.out*4);
	Set[3] = chassis_speed_task.chassis_first_order_x.out + chassis_speed_task.chassis_first_order_y.out - (chassis_speed_task.chassis_first_order_z.out*4);

	for (int i = 0; i < 4; i++)
	{
		// 底盘解算及PID算法闭环控制
		Chassis_speed_pid[i].f_calculate(&Chassis_speed_pid[i], chassis_moto[i].speed, Set[i] * 20); // 整体加速
	}

	CHASSIS_CURRENT(Chassis_speed_pid[0].pid_out, Chassis_speed_pid[1].pid_out,
					Chassis_speed_pid[2].pid_out, Chassis_speed_pid[3].pid_out);
//	CHASSIS_CURRENT(3000, 3000, 3000, 3000);
}
