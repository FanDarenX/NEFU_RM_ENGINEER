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

#include "image_task.h"

enum image_mode_enum image_mode;

img_task_t image_task;
first_order_filter_type_t image_yaw_filter;
first_order_filter_type_t image_pitch_filter;
static float image_mouse_filter = 1.6666666667f;

void image_task_init(img_task_t *image_init)
{
	first_order_filter_init(&image_yaw_filter, 0.02, image_mouse_filter);
	first_order_filter_init(&image_pitch_filter, 0.02, image_mouse_filter);
	image_init->image_rc = get_remote_control_point();
}

void yaw_servo_task(int16_t angle)
{
	//	float angle_step=(2752-396)*1000/270;
	float angle_step = 7.14258258;
	int32_t compare = angle * angle_step;
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 1500 + compare);
}
void pitch_servo_task(int16_t angle)
{
	//	float angle_step=(2705-301)/270;
	//	float angle_step=8.55185185185;
	//	int32_t compare = (angle+135)*angle_step;
	float angle_step = 6.8965517;
	int32_t compare = -angle * angle_step;

	if (!chassis_move_flag && Again_flag)
	{
		compare = -50;
	}
	if (compare < -450)
	{
		compare = -450;
	}
	if (compare > 50)
	{
		compare = 50;
	}
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1500 + compare);
}

void image_mode_task(void)
{
	//	if (key_task.chassis_mode && key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_G)
	image_mode = image_mouse;
	switch (image_mode)
	{
	case image_chassis:
	{
		image_task.current_yaw_angle = 20;
		image_task.current_pitch_angle -= image_task.image_rc->mouse.y * 0.5;
		break;
	}
	case image_little:
	{
		image_task.current_yaw_angle = 90;
		image_task.current_pitch_angle = -10;
		break;
	}
	case image_big:
	{
		image_task.current_yaw_angle = 90;
		image_task.current_pitch_angle = -10;
		break;
	}
	case image_exchange:
	{
		image_task.current_yaw_angle = 90;
		image_task.current_pitch_angle = -10;
		break;
	}
	case image_mouse:
	{
		image_task.current_yaw_angle += image_task.image_rc->mouse.x * 0.5;
		image_task.current_pitch_angle -= image_task.image_rc->mouse.y * 0.5;
		break;
	}
	case image_store:
	{
		image_task.current_yaw_angle = 90;
		image_task.current_pitch_angle = -30;
		break;
	}
	default:
		break;
	}
	first_order_filter_cali(&image_yaw_filter, image_task.current_yaw_angle);	  // 一阶低通滤波处理鼠标反馈值		//yaw轴
	first_order_filter_cali(&image_pitch_filter, image_task.current_pitch_angle); // 一阶低通滤波处理鼠标反馈值		//pitch
	image_task.current_yaw_angle = image_yaw_filter.out;
	image_task.current_pitch_angle = image_pitch_filter.out;
	//	limit_key_int16(&image_task.current_yaw_angle,500.0,2500.0);
	//	limit_key_int16(&image_task.current_pitch_angle,500.0,2500.0);
	yaw_servo_task(-image_task.current_yaw_angle);
	pitch_servo_task(image_task.current_pitch_angle);
}
