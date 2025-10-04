/*
 * @Descripttion:
 * @version:
 * @Author: fyh
 * @Date: 2024
 * @LastEditors: fyh
 * @LastEditTime: 2024
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
#include "remote_control.h"
#include "pid.h"
#include "keyscan_task.h"
#include "position_task.h"

uint8_t rc_to_chassis = 1; // ң���������ٶȷŴ�������Ͼֲ�����

uint8_t remote_mode = 0;						   // 0: ���̿��� 1: ң��������
chassis_task_t chassis_speed_task;				   // �˲���+float xyr
chassis_speed_rel chassis_speed_get;			   // float xyr
Motor_speed Speed_set;							   // float 1~4
ramp_function_source_t chassis_slope[4];		   // б�º���
first_order_filter_type_t first_chassis_mouse;	   // һ�׵�ͨ�˲������ü�������£����ݸ���ƽ��
static float chassis_mouse_filter = 1.6666666667f; // һ�׵�ͨ�˲���������Խ��Խ�ȣ�����Ӧ�ٶȽ���

pid_typedef Chassis_speed_pid[4];

float Set[4] = {0}; // �˲�������͵������->PIDĿ��ֵ

void chassis_remote_data(chassis_task_t *chassis_task_filter)
{
	//	speed->chassis_speed_y=first_order_filter_cali(&chassis_cmd_vy,-A_Data[0].x_speed*5);
	//	speed->chassis_speed_x=first_order_filter_cali(&chassis_cmd_vx,A_Data[0].y_speed*5);
	if (Lift_Middle)
		remote_mode = 1;
	else if (Lift_Down || Lift_Up) // Lift_UP��������
		remote_mode = 0;
	if (remote_mode) // ң����
	{
		if (Right_Up)
		{
			chassis_task_filter->chassis_speed.chassis_speed_x = -rc_ctrl.rc.ch[3] / 3.0;
			chassis_task_filter->chassis_speed.chassis_speed_y = rc_ctrl.rc.ch[2] / 3.0;
			chassis_task_filter->chassis_speed.chassis_speed_r = -rc_ctrl.rc.ch[0] / 3.0;
		}
		else if (Right_Middle)
		{
			chassis_task_filter->chassis_speed.chassis_speed_x = -rc_ctrl.rc.ch[3] / 2.0;
			chassis_task_filter->chassis_speed.chassis_speed_y = rc_ctrl.rc.ch[2] / 2.0;
			chassis_task_filter->chassis_speed.chassis_speed_r = -rc_ctrl.rc.ch[0] / 2.0;
		}
		else if (Right_Down)
		{
			chassis_task_filter->chassis_speed.chassis_speed_x = -rc_ctrl.rc.ch[3];
			chassis_task_filter->chassis_speed.chassis_speed_y = rc_ctrl.rc.ch[2];
			chassis_task_filter->chassis_speed.chassis_speed_r = -rc_ctrl.rc.ch[0];
		}
	}
	else // ����
	{
		chassis_task_filter->chassis_speed.chassis_speed_x = -key_task.key_speed.Vx;
		chassis_task_filter->chassis_speed.chassis_speed_y = key_task.key_speed.Vy;
		if (Right_Up)
		{
			chassis_task_filter->chassis_speed.chassis_speed_r = -key_task.key_speed.Vz / 3.0; // 10.0//?
			if (Spinning_Top_L)
			{
				chassis_task_filter->chassis_speed.chassis_speed_r = 5.0f;
			}
			if (Spinning_Top_R)
			{
				chassis_task_filter->chassis_speed.chassis_speed_r = -5.0f; // TODO��������Բ���
			}
		}
		else if (Right_Middle)
		{
			chassis_task_filter->chassis_speed.chassis_speed_r = -key_task.key_speed.Vz / 2.0;
			if (Spinning_Top_L)
			{
				chassis_task_filter->chassis_speed.chassis_speed_r = 100.0f;
			}
			if (Spinning_Top_R)
			{
				chassis_task_filter->chassis_speed.chassis_speed_r = -100.0f;
			}
		}
		else if (Right_Down)
		{
			chassis_task_filter->chassis_speed.chassis_speed_r = -key_task.key_speed.Vz;
			if (Spinning_Top_L)
			{
				chassis_task_filter->chassis_speed.chassis_speed_r = 100.0f;
			}
			if (Spinning_Top_R)
			{
				chassis_task_filter->chassis_speed.chassis_speed_r = -100.0f;
			}
		}
	}
	// first_order_filter_cali(&chassis_task_filter->chassis_first_order_z, chassis_task_filter->chassis_speed.chassis_speed_r); // һ�׵�ͨ�˲�������귴��ֵ
	// first_order_filter_cali(&chassis_task_filter->chassis_first_order_x, chassis_task_filter->chassis_speed.chassis_speed_x); // һ�׵�ͨ�˲�������귴��ֵ
	// first_order_filter_cali(&chassis_task_filter->chassis_first_order_y, chassis_task_filter->chassis_speed.chassis_speed_y); // һ�׵�ͨ�˲�������귴��ֵ
}

void chassis_init(void)
{
	for (int i = 0; i < 4; i++)
	{
		pid_init(&Chassis_speed_pid[i]);
		Chassis_speed_pid[i].f_param_init(&Chassis_speed_pid[i], 0, 8000, 15900, 5000, 5, 0, 20, 0.01, 15);
		//		first_order_filter_init(&chassis_cmd_vx, CHASSIS_ACCEL_X_NUM, CHASSIS_CONTROL_TIME);//ң���������˲�
		//		first_order_filter_init(&chassis_cmd_vy, CHASSIS_ACCEL_Y_NUM, CHASSIS_CONTROL_TIME);
	}
	ramp_init(&chassis_speed_task.chassis_ramp[0], 0.01, 660 * 7, -660 * 7);
	ramp_init(&chassis_speed_task.chassis_ramp[1], 0.01, 660 * 7, -660 * 7);
	//	ramp_init(&chassis_speed_task.chassis_ramp[2], 0.01, 660 * 7, -660 * 7);
	first_order_filter_init(&chassis_speed_task.chassis_first_order_z, 0.02, chassis_mouse_filter); // ���ʱ��->���ٶ�||˿���ȣ����Խ��Խ˿��
	first_order_filter_init(&chassis_speed_task.chassis_first_order_x, 0.02, chassis_mouse_filter); //? ���ָ���ٶȿ��������������û�й�ϵ
	first_order_filter_init(&chassis_speed_task.chassis_first_order_y, 0.02, chassis_mouse_filter);
}

void Chassis_Task_os(void)
{
	chassis_remote_data(&chassis_speed_task);
	if (Lift_Up)
	{
		chassis_speed_task.chassis_speed.chassis_speed_x = 0.0;
		chassis_speed_task.chassis_speed.chassis_speed_y = 0.0;
		chassis_speed_task.chassis_speed.chassis_speed_r = 0.0;
	}
	A_TO_C_Chassis(chassis_speed_task.chassis_speed.chassis_speed_x, chassis_speed_task.chassis_speed.chassis_speed_y, chassis_speed_task.chassis_speed.chassis_speed_r);
	/*TODO
	//! ����˱�������������٣��·�pid���룩��Ҳ�ɾֲ����٣���Z
	Set[0] = -(chassis_speed_task.chassis_first_order_x.out + chassis_speed_task.chassis_first_order_y.out + (chassis_speed_task.chassis_first_order_z.out*4)); //*40
	Set[1] = (chassis_speed_task.chassis_first_order_x.out - chassis_speed_task.chassis_first_order_y.out - (chassis_speed_task.chassis_first_order_z.out*4));
	Set[2] = chassis_speed_task.chassis_first_order_x.out + chassis_speed_task.chassis_first_order_y.out - (chassis_speed_task.chassis_first_order_z.out*4);
	Set[3] = -chassis_speed_task.chassis_first_order_x.out + chassis_speed_task.chassis_first_order_y.out - (chassis_speed_task.chassis_first_order_z.out*4);

	// 	Set[1] = -(chassis_speed_task.chassis_first_order_x.out + chassis_speed_task.chassis_first_order_y.out + (chassis_speed_task.chassis_first_order_z.out * 4));
	// 	Set[2] = (chassis_speed_task.chassis_first_order_x.out - chassis_speed_task.chassis_first_order_y.out - (chassis_speed_task.chassis_first_order_z.out * 4));
	// 	Set[3] = chassis_speed_task.chassis_first_order_x.out + chassis_speed_task.chassis_first_order_y.out - (chassis_speed_task.chassis_first_order_z.out * 4);
	// 	Set[0] = -chassis_speed_task.chassis_first_order_x.out + chassis_speed_task.chassis_first_order_y.out - (chassis_speed_task.chassis_first_order_z.out * 4);
//---
	//+--
	//++-
	-+-
#ifdef debug

	Speed_set.chassis_M1 = Set[0];
	Speed_set.chassis_M2 = Set[1];
	Speed_set.chassis_M3 = Set[2];
	Speed_set.chassis_M4 = Set[3];

	for (int i = 0; i < 4; i++)
	{
		Chassis_speed_pid[i].pid_out = 0;

	} // ���Ե�������

#endif
	if (Lift_Up)
	{
		Set[0] = 0;
		Set[1] = 0;
		Set[2] = 0;
		Set[3] = 0;
	}

	for (int i = 0; i < 4; i++)
	{
		// ���̽��㼰PID�㷨�ջ�����
		Chassis_speed_pid[i].f_calculate(&Chassis_speed_pid[i], chassis_moto[i].speed, Set[i]); // �������
	}

	CHASSIS_CURRENT(Chassis_speed_pid[0].pid_out, Chassis_speed_pid[1].pid_out,
					Chassis_speed_pid[2].pid_out, Chassis_speed_pid[3].pid_out);
	//*/
}
