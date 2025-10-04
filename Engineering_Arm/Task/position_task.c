/*
 * @Descripttion:
 * @version:1.0
 * @Author: xqz
 * @Date: 2024-11
 * @LastEditors: xqz
 * @LastEditTime:
 */

/*
	 /\                                                         (_)                              (_)
	/  \     _ __    ___   ___   ______    ___   _ __     __ _   _   _ __     ___    ___   _ __   _   _ __     __ _
   / /\ \   | '__|  / _ \ / __| |______|  / _ \ | '_ \   / _` | | | | '_ \   / _ \  / _ \ | '__| | | | '_ \   / _` |
  / ____ \  | |    |  __/ \__ \          |  __/ | | | | | (_| | | | | | | | |  __/ |  __/ | |    | | | | | | | (_| |
 /_/    \_\ |_|     \___| |___/           \___| |_| |_|  \__, | |_| |_| |_|  \___|  \___| |_|    |_| |_| |_|  \__, |
														  __/ |                                                __/ |
														 |___/                                                |___/
*/

#include "position_task.h"
#include "user_lib.h"
#include "keyscan_task.h"
#include "math.h"

/*机械臂吸盘串级PID控制器*/
pid_struct_t Cup_speed_pid[2] = {0};
pid_struct_t Cup_angle_pid[2] = {0}; // 为保持同步，选择使用内侧电机角度环控两电机速度环
/*龙门架串级PID控制器*/
pid_struct_t Uplift_speed_pid[4] = {0};
pid_struct_t Uplift_angle_pid[4] = {0}; // 为保持同步，选择使用受力最多电机角度环控四电机速度环
pid_struct_t X_speed_pid = {0};
pid_struct_t X_angle_pid = {0};
pid_struct_t Y_speed_pid = {0};
pid_struct_t Y_angle_pid = {0};

// int a;

/*低阶一通滤波*/
first_order_filter_type_t roll_filter;
/*斜波*/
static float roll_filter_k = 1.6666666667f;

/*机械臂吸盘目标角度*/
// 待定

/*机械臂目标位置*/
/*遥控器ch转换*/
/*机械臂*/
float yaw = 0.0f; // DM_yaw
float pic = 0.0f; // DM_pitch
/*DM_delay*/
float Cup_target_pitch = 0.0f; // 2006
float Cup_target_roll = 0.0f;  // 2006
/*龙门架*/
float X_target = 0;
float Y_target = 0;
float Z_target = 0.0f;
/*键盘控制计算目标位置*/
float Cup_pitch_temp;
float Cup_roll_temp;
float yaw_temp;
float pic_temp;
float X_temp;
float Y_temp;
float Z_temp;

/*倍数*/ /*之后测试需要找到合适的倍数*/

float compensate_Y = 0.0f;
float compensate_Z = 5.0f;

float Multiple_Cup_pitch = 15.0f; // 36:1//6:5    /*7.50f对应通道值660->90度*/
float Multiple_Cup_roll = 7.50f;  // 7.50f对应pitch和roll轴90度
float Multiple_X = 36.0 / 18.0 * (36.0f - 34.0);
float Multiple_Y = 36.0 / 14.0 * (50.0f - 1.0f); // 此处是总位置减去安全位置//相当于在安全位置下的相对角度了//50
float Multiple_Z = 19.0 / 18.0 * (28.0f);		 // 19:1

/*龙门架补偿值*/
// 拉高取同号，拉低取异号
float cp1 = 100.0f;	  // total负-4096.0f
float cp2 = -1620.0f; // total正-4096.0f
float cp3 = 700.0f;	  // total负3400.0f
float cp4 = 400.0f;	  // total正-2900.0f

float compensate1 = 0.0f / 19.0 * 18.0f; // 4096.0 / 19.0 * 18.0f
float compensate2 = 0.0f / 19.0 * 18.0f; // 1024.0 / 19.0 * 18.0f	// 		  // 要降低
float compensate3 = 0.0f / 19.0 * 18.0f; //(512.0) / 19.0 * 18.0f 补偿少受力 // 3比2多低：4096.0 / 19.0 * 18.0f
float compensate4 = 0.0f / 19.0 * 18.0f; // 4096.0 / 19.0 * 18.0f

/*安全：1.先Uplift_safe_flag，再Y_safe_flag*/
/*角度控制标志*/
int Uplift_safe_flag = 0; // 抬升龙门架竖直面投影安全距离//?暂时不用
int Y_safe_flag = 0;	  // 前伸龙门架水平面投影安全距离

/*初始安全位置倍数*/
float Uplift_safe_multiple = 0.0f; //?暂时不用
float Y_safe_multiple = 36.0 / 14.0 * 34.0f;

/*控制*/
int remoteControl = 1;
int keyboard = 1;

/*机械臂电机反馈指针*/
// 暂时没用，可以看position用到哪些电机了
const moto_measure *get_Cup_moto(int i)
{
	return &Cup_moto[(i)];
}
const motor_t *get_position_moto_DM_yaw()
{
	return &DM_yaw;
}
const motor_t *get_position_moto_DM_pitch()
{
	return &DM_pitch;
}
/*机械臂电机反馈指针*/

/*龙门架电机反馈指针*/
const moto_measure *get_X_moto_2006_point()
{
	return &X_moto;
}
const moto_measure *get_Y_moto_2006_point()
{
	return &Y_moto;
}
const moto_measure *get_Z_moto_3508_point(int i)
{
	return &Uplift_moto[(i)];
}
/*龙门架电机反馈指针*/

void position_init()
{
	/*简版PID*/ //! 测试效果不错，并且用一个参数即可，妙~
	int Init_flag = 0;
	while (!Init_flag) // 方便折叠
	{
		// 2006电机控制电流, 范围 [-10000,10000]
		// 3508电机控制电流, 范围 [-16384,16384]
		// 6020电机控制电流, 范围 [-30000,30000]
		/*CAN1*/
		/*机械臂吸盘*/
		//  0x205
		Pid_Init(&Cup_speed_pid[0], // PID需要细调，“回弹”
				 10,				// Kp
				 0.05,				// Ki
				 1,					// Kd
				 200,				// Imaxout
				 8000				// Outmax
		);
		Pid_Init(&Cup_angle_pid[0],
				 0.1, // Kp
				 0,	  // Ki
				 0,	  // Kd
				 200, // Imaxout
				 8192 // Outmax
		);
		// 0x206
		Pid_Init(&Cup_speed_pid[1],
				 10,   // Kp
				 0.05, // Ki
				 1,	   // Kd
				 200,  // Imaxout
				 8000  // Outmax
		);
		Pid_Init(&Cup_angle_pid[1],
				 0.1, // Kp
				 0,	  // Ki
				 0,	  // Kd
				 200, // Imaxout
				 8192 // Outmax
		);
		/*CAN1*/

		/**************************************/

		/*CAN2*/
		/*龙门架*/
		/*X*/
		/*0x205*/
		Pid_Init(&X_speed_pid,
				 10,   // Kp
				 0.05, // Ki
				 1,	   // Kd
				 200,  // Imaxout
				 8000  // Outmax
		);
		Pid_Init(&X_angle_pid,
				 0.1, // Kp
				 0,	  // Ki
				 0,	  // Kd
				 200, // Imaxout
				 8192 // Outmax
		);
		/*Y*/
		/*0x206*/
		Pid_Init(&Y_speed_pid,
				 10,   // Kp
				 0.05, // Ki
				 1,	   // Kd
				 200,  // Imaxout
				 8000  // Outmax
		);
		Pid_Init(&Y_angle_pid,
				 0.1, // Kp
				 0,	  // Ki
				 0,	  // Kd
				 200, // Imaxout
				 8192 // Outmax
		);
		/*Z*/
		/*0x201*/
		Pid_Init(&Uplift_speed_pid[0],
				 10,   // Kp
				 0.05, // Ki
				 1,	   // Kd
				 200,  // Imaxout
				 8000  // Outmax
		);
		Pid_Init(&Uplift_angle_pid[0],
				 0.1, // Kp
				 0,	  // Ki
				 0,	  // Kd
				 200, // Imaxout
				 8192 // Outmax
		);
		/*0x202*/
		Pid_Init(&Uplift_speed_pid[1],
				 10,   // Kp
				 0.05, // Ki
				 1,	   // Kd
				 200,  // Imaxout
				 8000  // Outmax
		);
		Pid_Init(&Uplift_angle_pid[1],
				 0.1, // Kp
				 0,	  // Ki
				 0,	  // Kd
				 200, // Imaxout
				 8192 // Outmax
		);
		/*0x203*/
		Pid_Init(&Uplift_speed_pid[2],
				 10,   // Kp
				 0.05, // Ki
				 1,	   // Kd
				 200,  // Imaxout
				 8000  // Outmax
		);
		Pid_Init(&Uplift_angle_pid[2],
				 0.1, // Kp
				 0,	  // Ki
				 0,	  // Kd
				 200, // Imaxout
				 8192 // Outmax
		);
		/*0x204*/
		Pid_Init(&Uplift_speed_pid[3],
				 10,   // Kp
				 0.05, // Ki
				 1,	   // Kd
				 200,  // Imaxout
				 8000  // Outmax
		);
		Pid_Init(&Uplift_angle_pid[3],
				 0.1, // Kp
				 0,	  // Ki
				 0,	  // Kd
				 200, // Imaxout
				 8192 // Outmax
		);
		/*CAN2*/
		Init_flag = 1;
	}
}

double msp(double x, double in_min, double in_max, double out_min, double out_max) // 映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi）
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//	a = (int)HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0);
//	return;
void position_task(void)
{
	/*前伸复位*/
	// 这里应该写入总体上电情况里，不然其他电机没进阻尼
	// if (!reposition_flag_Cup)
	// {
	// 	Cup_pitch_temp = -(float)8192 * Multiple_Cup_pitch / 180 * Cup_target_pitch; // 50.0f
	// 	Cup_location(Cup_pitch_td.xx1, 0.0f);
	// 	//		XY_location(0.0f, 0.0f);
	// 	return;
	// }

	if (!reposition_flag_Y)
	{
		Z_temp = -(float)8192 * 19.0 / 18.0 * 3.0f;
		Y_temp = -(float)8192 * 36.0 / 14.0 * Y_target;
		XY_location(0.0f, Y_td.xx1);
		Uplift_location(Uplift_td.xx1);
		return;
	}

	if (Lift_Up)
	{
		remoteControl = 1;
		keyboard = 0;
	}
	else if (Lift_Middle || Lift_Down)
	{
		remoteControl = 0;
		keyboard = 1;
	}

	// remoteControl = 0;
	// keyboard = 1;

	if (remoteControl) // 底盘运动模式下//可以算得上遥控器Debug的条件下
	{
		if (Right_Down)
		{
			// float a = sin(3.14);
			/*机械臂吸盘*/
			Cup_target_pitch = -ABS((float)Cup_pitch);
			Cup_target_roll = (float)Cup_roll;
			/*机械臂YAW&&PITCH*/
			yaw = (float)Arm_yaw;
			yaw = (yaw);
			DM4310_YAW(yaw);
			pic = (float)Arm_pitch;
		}
		else if (Right_Middle)
		{
			/*龙门架X&&Y*/
			X_target = -((float)X_traverse);
			Y_target = -ABS((float)Y_traverse);
			/*龙门架Z*/
			Z_target = -ABS((float)Z_traverse);
		}
	}

	else if (keyboard || Custom_action) // 全局变量传参//!实际测试找到问题，由于程序不断循环计算，导致参数不断缩小导致归零；解决办法：用一个缓存变量存储临时值
	{									//! 貌似意外解决编码值跳变问题，先进行除操作把跳变降下来，此时基本无跳变，此时在进行乘应该会进一步精确
		// 不知道为啥这里识别不了Custom_action等于1
		//! 可尝试设置标志位，初次按下X直接赋为中间值，再次按下跟随自定义控制器，再按则回到X中间值……
		/*整体限制*/
		// limit_key(&Cup_target_pitch, 230 + 100, -90 - 100); //! 总限//可在键鼠和自定义控制器加子限    //?所以是同时运行还是单独运行，应该是单独运行，//TODO测试
		// limit_key(&Cup_target_roll, 360, -360);
		limit_key(&yaw, 90, -90); // TODO新版工程180
		limit_key(&pic, 200, -200);
		// /*龙门架*/
		limit_key(&X_target, 10.0f, -10.0f);
		limit_key(&Y_target, 49.0f, 0.7f);
		limit_key(&Z_target, 29.0f, 1.0f);
		/* 机械臂*/

		/*机械臂吸盘*/
		Cup_pitch_temp = -(float)8192 * Multiple_Cup_pitch / 180 * Cup_target_pitch; // 0~180
		Cup_roll_temp = (float)8192 * Multiple_Cup_roll / 45 * Cup_target_roll;		 //-45~45
		/*机械臂YAW&&PITCH*/
		yaw_temp = PI / 180.0f * yaw; // 0~270
		pic_temp = PI / 180.0f * pic; //-180~180
		/*机械臂*/
		/*******************/

		/*龙门架*/
		/*龙门架X&&Y*/									// TODO看需要分成几份
		X_temp = -(float)8192 * 36.0 / 19.0 * X_target; // X_target:1对应横移1齿//1：36；18：1.max:36齿
		Y_temp = -(float)8192 * 36.0 / 14.0 * Y_target; // TODO- compensate_Y // Y_target:1对应前伸1齿//1：36；14：1.max:52齿
		/*龙门架Z*/
		Z_temp = -(float)8192 * 19.0 / 18.0 * Z_target; // Z_target:1对应上移1齿//1：19；18：1.max:28齿
	}

	/*电机驱动*/ // 能够满足切换模式时电机保持原位姿  //经过测试如果没有循环运行电机驱动函数，会导致电机不工作
				 // 同时实现keyscan计算，position、chasiss驱动
				 // 由于循环计算的问题存在，所以遥控器控制与键盘控制分开

	if (!Cup_X_reset)
	{
		Cup_location(Cup_pitch_td.xx1, Cup_roll_td.xx1);
	}
	DM_location(yaw_td.xx1, pic_td.xx1); // DM电机初始雀食[旺柴]不会自行回到零点，//!所以突破口在这里
	XY_location(X_td.xx1, Y_td.xx1);
	Uplift_location(Uplift_td.xx1);
	//	Uplift_location(Z_target);
	//		Z_temp = -(float)8192 * 19.0 / 18.0 * Z_target;
	//		Uplift_location(Z_temp);
}

int Uplift_is_safe()
{
	return Uplift_safe_flag ? 1 : 0;
}

int Y_is_safe()
{
	return Y_safe_flag ? 1 : 0;
}

/*各组成部分位置环复用函数*/
void Cup_location(float Cup_target_pitch, float Cup_target_roll) // 两者之一需要为零,参数是编码值8192倍数
{
	// if (!Y_is_safe())
	// {
	// 	Cup_target_pitch = 0.0f;
	// 	Cup_target_roll = 0.0f;
	// }

	/*为保持同步，取内侧0x205角度环控制两电机速度环*/ // 实际试过后不精准
	/*机械臂吸盘Pitch&&Roll*/
	pid_calc(&Cup_angle_pid[0], Cup_target_roll + Cup_target_pitch, Cup_moto[0].total_angle); // 0x205里
	pid_calc(&Cup_speed_pid[0], Cup_angle_pid[0].output, Cup_moto[0].speed);
	// 一顺一逆控制pitch，同顺同逆控制roll
	pid_calc(&Cup_angle_pid[1], Cup_target_roll - Cup_target_pitch, Cup_moto[1].total_angle);
	pid_calc(&Cup_speed_pid[1], Cup_angle_pid[1].output, Cup_moto[1].speed);

	Cup_CURRENT(Cup_speed_pid[0].output, Cup_speed_pid[1].output);
}

void DM_location(float yaw, float pic) // 参数是弧度制-PI~PI//内部应包含判定先yaw后pitch的角度顺序
{
	// if (!Y_is_safe())
	// {
	// 	yaw = 0.0f;
	// 	pic = 0.0f;
	// }
	/*伸出先yaw后pitch*/
	mit_ctrl(&hcan1, &DM_yaw, DM_yaw.id, yaw, DM_yaw.ctrl.vel_set, DM_yaw.ctrl.kp_set, DM_yaw.ctrl.kd_set, DM_yaw.ctrl.tor_set);
	// if (DM_yaw.para.pos > yaw - 0.1 && DM_yaw.para.pos < yaw + 0.1)
	mit_ctrl(&hcan1, &DM_pitch, DM_pitch.id, pic, DM_pitch.ctrl.vel_set, DM_pitch.ctrl.kp_set, DM_pitch.ctrl.kd_set, DM_pitch.ctrl.tor_set);
}

void XY_location(float X_target, float Y_target) // 参数是编码值8192倍数// 内部应包含判定先x后y的角度顺序,看Y_target（内含绝对值）与Y_save的，决定X_target是否为零
{
	/*前伸到机械臂水平面投影安全距离*/
	if (Y_moto.total_angle <= Y_safe) // Y_moto.total_angle > Y_safe + Y_target - 10000 &&
		Y_safe_flag = 0;
	else
		Y_safe_flag = 1;

	// /*抬升到机械臂竖直面投影安全距离*/
	// if (!Uplift_is_safe() && !Y_is_safe())
	// {s
	// 	Y_target = XY_safe;
	// }

	// if (Uplift_is_safe())
	// {
	// 	Y_target = ABS(Y_target); // TODO是否需要加负号
	// }

	// if (!Uplift_is_safe() && Y_is_safe())
	// {
	// 	limit_key(&Y_target, Y_MAX, Y_safe + 3000);
	// }
	float compensate_Y = -4096.0 * 36.0 / 14.0;			  // 半个齿
	pid_calc(&Y_angle_pid, Y_target, Y_moto.total_angle); // + compensate_Y
	pid_calc(&Y_speed_pid, Y_angle_pid.output, Y_moto.speed);
	// if (!Y_safe_flag)
	// {
	// pid_calc(&X_angle_pid, X_safe, X_moto.total_angle);
	// }
	// /*这时ALL_safe_flag为真*/
	// else
	// {
	// }
	pid_calc(&X_angle_pid, X_target, X_moto.total_angle);
	pid_calc(&X_speed_pid, X_angle_pid.output, X_moto.speed);
	XY_CURRENT(Y_speed_pid.output, X_speed_pid.output);
}

void Uplift_location(float Z_target) // 参数是编码值8192倍数
{
	//! 注意这里的补偿对控制的影响
	// 1==3;2==4;
	//-10593
	// 10557
	//-9613
	// 9557
	// 1,4升；2，3降
	compensate1 = cp1 / 19.0 * 18.0f;
	compensate2 = cp2 / 19.0 * 18.0f;
	compensate3 = cp3 / 19.0 * 18.0f;
	compensate4 = cp4 / 19.0 * 18.0f;

	// if (Uplift_moto[0].real_current > 10000)
	// {
	// 	Uplift_moto[0].round_cnt -= 1;
	// }

	// pid_calc(&Uplift_angle_pid[0], Z_target + compensate1, Uplift_moto[0].angle); //! 注意方向两顺两逆
	// pid_calc(&Uplift_angle_pid[1], -Z_target + compensate2, Uplift_moto[1].angle);
	// pid_calc(&Uplift_angle_pid[2], Z_target + compensate3, Uplift_moto[2].angle); // TODO[2]待定 //实际实验，除了参与角度计算的该电机，其余电机1.无力2.堵转会速度环一直转
	// pid_calc(&Uplift_angle_pid[3], -Z_target + compensate4, Uplift_moto[3].angle);

	//! 1,3逆；2，4顺；
	pid_calc(&Uplift_angle_pid[0], Z_target + compensate1, Uplift_moto[0].total_angle); //! 注意方向两顺两逆
	pid_calc(&Uplift_angle_pid[1], -Z_target + compensate2, Uplift_moto[1].total_angle);
	pid_calc(&Uplift_angle_pid[2], Z_target + compensate3, Uplift_moto[2].total_angle); // TODO[2]待定 //实际实验，除了参与角度计算的该电机，其余电机1.无力2.堵转会速度环一直转
	pid_calc(&Uplift_angle_pid[3], -Z_target + compensate4, Uplift_moto[3].total_angle);

	pid_calc(&Uplift_speed_pid[0], Uplift_angle_pid[0].output, Uplift_moto[0].speed);
	pid_calc(&Uplift_speed_pid[1], Uplift_angle_pid[1].output, Uplift_moto[1].speed);
	pid_calc(&Uplift_speed_pid[2], Uplift_angle_pid[2].output, Uplift_moto[2].speed);
	pid_calc(&Uplift_speed_pid[3], Uplift_angle_pid[3].output, Uplift_moto[3].speed);

	Uplift_CURRENT(Uplift_speed_pid[0].output, Uplift_speed_pid[1].output, Uplift_speed_pid[2].output, Uplift_speed_pid[3].output);

	// if ( <= Uplift_safe) // TODO[2]待定
	// 	Uplift_safe_flag = 0;
	// else
	// 	Uplift_safe_flag = 1;
}
/*各组成部分位置环复用函数*/

/**暂时不用
 *
 *
 *
 *
 *
 */
/*顶层构造角度函数*/ // 圈数+角度
/*相对操作固定值+相对值即可*/
void Cup_fixed(float angle_pitch, float angle_roll) // 角度pitch:0~180\roll:-45~45，同时上电或复位前把转盘放到朝平面碳板方向向上
{
	angle_pitch = -(float)8192 * Multiple_Cup_pitch / 180 * angle_pitch;
	angle_roll = (float)8192 * Multiple_Cup_roll / 45 * angle_roll;
	Cup_location(angle_pitch, angle_roll);
}

void DM_fixed(float yaw, float pic) // yaw角度-PI~0,pitch角度,有顺序的  // pitch待定  //TODO
{
	yaw = -PI / 180.0f * yaw;
	DM_location(yaw, 0);
}
void XY_fixed(float X_target, float Y_target) // 参数是编码值8192倍数  // 待定  //TODO
{
	X_target = (float)8192 * Multiple_X / 360 * X_target; // 360是把左右分各为360份，X_target:-360~360
	Y_target = (float)8192 * Multiple_Y / 360 * Y_target;
	XY_location(X_target, Y_target);
}
void Uplift_fixed(float Z_target) // 参数是编码值8192倍数  // 待定  //TODO
{
	Z_target = (float)8192 * Multiple_Z / 360 * Z_target;
	Uplift_location(Z_target);
}
/*顶层构造角度函数*/