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

/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 *
 *
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *            佛祖保佑     永不宕机     永无BUG
 *
 *        佛曰:
 *                写字楼里写字间，写字间里程序员；
 *                程序人员写程序，又拿程序换酒钱。
 *                酒醒只在网上坐，酒醉还来网下眠；
 *                酒醉酒醒日复日，网上网下年复年。
 *                但愿老死电脑间，不愿鞠躬老板前；
 *                奔驰宝马贵者趣，公交自行程序员。
 *                别人笑我忒疯癫，我笑自己命太贱；
 *                不见满街漂亮妹，哪个归得程序员？
 */

/*作用：
1.切换各种取矿、底盘运动等模式;
2.计算·得到目标值传给chassis、position等；
3.死亡复位各种init；
*/

#include "keyscan_task.h"
#include "image_task.h"
#include "yaw_task.h"
#include "position_task.h"
#include "image_task.h"
#include "chassis_task.h"
#include "UI_Task.h"
#include "referee_usart_task.h"
#include "chassis_task.h"
#include "UI_task.h"
#include "position_task.h"
#include "math.h"
#include "referee_usart_task.h"
#include "referee.h"
#include "ALL_init.h"

#define Fast 150.0f
#define Slow 3.5f

/*结构体、变量*/
key_typedef_t key_task;

int Cup_SF = 0;
int Right_SF = 0;
int Left_SF = 0;

/*机械臂吸盘PE6->K1*/
/*电磁阀PB0->L2*/
//! 右存矿机构吸盘
void Left_Suck()
{
	HAL_GPIO_WritePin(Cup_GPIO_Port, Cup_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(VALVE_GPIO_Port, VALVE_Pin, GPIO_PIN_RESET);
	Left_SF = 1;
}
void Left_Fart()
{
	HAL_GPIO_WritePin(Cup_GPIO_Port, Cup_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(VALVE_GPIO_Port, VALVE_Pin, GPIO_PIN_SET);
	Left_SF = 0;
}

//! 小臂吸盘控制
// left-deposit
// M1,N1
// C3,C4
void Cup_Fart()
{
	HAL_GPIO_WritePin(Left_GPIO_Port, Left_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Left_GPIO_Port, Left_logic_pin, GPIO_PIN_RESET);
	Cup_SF = 0;
}
void Cup_Suck()
{
	HAL_GPIO_WritePin(Left_GPIO_Port, Left_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Left_GPIO_Port, Left_logic_pin, GPIO_PIN_SET);
	Cup_SF = 1;
}
//! 左存矿机构吸盘
void Right_Fart()
{
	HAL_GPIO_WritePin(Right_GPIO_pump, Right_Pin_pump, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Right_GPIO_logic, Right_Pin_logic, GPIO_PIN_RESET);
	Right_SF = 0;
}
void Right_Suck()
{
	HAL_GPIO_WritePin(Right_GPIO_pump, Right_Pin_pump, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Right_GPIO_logic, Right_Pin_logic, GPIO_PIN_SET);
	Right_SF = 1;
}
/*继电器PC2->L1*/
void Relays_OFF() // 继电器掉电
{
	HAL_GPIO_WritePin(Relays_GPIO_Port, Relays_Pin, GPIO_PIN_RESET);
}
void Relays_ON() // 继电器得电
{
	HAL_GPIO_WritePin(Relays_GPIO_Port, Relays_Pin, GPIO_PIN_SET);
}
/**
 *修改方法
 *1.底盘部分保留
 *2.遥控器部分保留
 *3.修改取矿部分
 *4.增加龙门架safe
 */
// TODO为了加入存矿机构，需要至少三对GPIO口
/*位置变量*/
/*FLAG*/
/*
前提：切换任意取矿模式->均将key_init->chassis_mode置为1// 0为底盘运动，1为取矿
1.初始化为1，因为key_init初始化为底盘运动模式；
2.推导mouse_l内应给chasiss_move_flag置1；
3.初次切换其他任意取矿模式，按任意模式按键第一下各个目标值为0.0f,然后再按想要切换的模式，会到该模式的初始位置；
4.由于目标值变量存储在内存中未进行其他变量赋值，所以切换到底盘运动模式后位姿不变，之后想要再切换为目标模式位姿，
先按下任意模式按键，此时是：变量=变量（相当于自我赋值），延续先前位姿；接着再次按下目标模式按键会切换到该模式初始位置；// 变量自我赋值相当于不对变量进行赋值操作
在进入其他取矿模式目标值计算的函数中进行判断赋0
*/
//! 5.至于按键+-微调，读取的始终是当前目标值，在当前值上进行加减操作。
int chassis_move_flag = 1; // 在模式切换位姿保持中起到中至关重要的作用
int Again_flag = 1;		   // 配合chassis_move_flag切换模式位姿不变
int compute_flag = 1;	   // 按键加减计算 //!暂时不用

/*自定义控制器标志位*/
int Gantry_flag = 0; // Shift + G纯小臂0 | Shift + F整体1

/*小陀螺标志位*/
int Spinning_Top_L = 0;
int Spinning_Top_R = 0;

/*取存矿方向*/
int direction = 1; //! 包含右侧（Ctrl+E左|R右）0左|1右  ；；初始为1配合新右存矿

/*金矿一键标志位*/
int Gold_flag = 0;
/*银矿一键标志位*/
int Silver_flag = 0;
/*一键存矿*/
int deposit_flag = 0;
/*一键取存的矿*/
int extraction_flag = 0;
/*一键兑矿*/
int cashing_flag = 0;

/*死亡||复位标志位*/
int reposition_flag_Y = 0;
int reposition_flag_Cup = 0;

/*Cup_pitch复位*/
int Cup_X_reset = 0;

/*根据剩余血量判断死亡*/
die_t die;

void key_scan_init(key_typedef_t *key_init) // key的init底盘+取矿
{
	/*遥控器数据*/
	key_init->key_rc_ctrl = get_remote_control_point();
	/*底盘||取矿*/
	key_init->chassis_mode = 0; // mouse.l   // 0为底盘运动，1为取矿
	/*启用自定义控制器*/
	key_init->custom_mode = 0; // Shift + B // 初始化不用自定义控制器控制

	/*新增模式*/
	/*基础蜷缩模式*/
	key_init->Base = 0; // Shift + C
	/*安全取矿初始化*/
	key_init->Safe = 0; // mouse.r         //!作用重叠，考虑更改为其他更实用的模式，例如：大一键||死亡复位
	/*一键兑矿*/
	key_init->Cashing = 0; //! 一键兑矿
	/*一键兑矿+Yaw*/
	key_init->Cashing_yaw = 0;
	/*取金矿*/
	key_init->Gold = 0; // Shift+R
	/*取地银矿*/
	key_init->Silver = 0; // Shift+E

	/*一键存矿*/
	key_init->deposit = 0; // Shift + Z
	/*一键取矿*/
	key_init->extraction = 0; // Shift + X

	/*主动死亡标志位*/
	key_init->go_die_flag = 0; // Ctrl + Z + X
}
void chassis_speed(void) // 底盘速度键盘控制
{
	// WD上右为正，SA下左为负
	/*右拨杆控制底盘速度*/
	if (key_task.chassis_mode == 0)
	{
		/*remote == 0 键盘操作*/
		if (remote_mode == 0 && key_task.key_rc_ctrl->rc.s[0] == 1) // Right_Up
			key_task.key_speed.speed = 100.0;
		else if (remote_mode == 0 && key_task.key_rc_ctrl->rc.s[0] == 3) // Right_Middle
			key_task.key_speed.speed = 250.0;
		else if (remote_mode == 0 && key_task.key_rc_ctrl->rc.s[0] == 2) // Right_Down
			key_task.key_speed.speed = 420.0;
		limit_key(&key_task.key_speed.speed, 770, 0);
		if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_D)
		{
			key_task.key_speed.Vy = key_task.key_speed.speed;
			key_task.key_speed.Vx = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_A)
		{
			key_task.key_speed.Vy = -key_task.key_speed.speed;
			key_task.key_speed.Vx = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_S)
		{
			key_task.key_speed.Vx = -key_task.key_speed.speed;
			key_task.key_speed.Vy = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_W)
		{
			key_task.key_speed.Vx = key_task.key_speed.speed;
			key_task.key_speed.Vy = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_W + KEY_PRESSED_OFFSET_A))
		{
			key_task.key_speed.Vx = key_task.key_speed.speed;
			key_task.key_speed.Vy = -key_task.key_speed.speed;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_W + KEY_PRESSED_OFFSET_D))
		{
			key_task.key_speed.Vx = key_task.key_speed.speed;
			key_task.key_speed.Vy = key_task.key_speed.speed;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_S + KEY_PRESSED_OFFSET_A))
		{
			key_task.key_speed.Vx = -key_task.key_speed.speed;
			key_task.key_speed.Vy = -key_task.key_speed.speed;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_S + KEY_PRESSED_OFFSET_D))
		{
			key_task.key_speed.Vx = -key_task.key_speed.speed;
			key_task.key_speed.Vy = key_task.key_speed.speed;
		}

		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_D + KEY_PRESSED_OFFSET_SHIFT))
		{
			key_task.key_speed.Vy = key_task.key_speed.speed / Slow;
			key_task.key_speed.Vx = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_A + KEY_PRESSED_OFFSET_SHIFT))
		{
			key_task.key_speed.Vy = -key_task.key_speed.speed / Slow;
			key_task.key_speed.Vx = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_S + KEY_PRESSED_OFFSET_SHIFT))
		{
			key_task.key_speed.Vx = -key_task.key_speed.speed / Slow;
			key_task.key_speed.Vy = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_W + KEY_PRESSED_OFFSET_SHIFT))
		{
			key_task.key_speed.Vx = key_task.key_speed.speed / Slow;
			key_task.key_speed.Vy = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_W + KEY_PRESSED_OFFSET_A + KEY_PRESSED_OFFSET_SHIFT))
		{
			key_task.key_speed.Vx = key_task.key_speed.speed / Slow;
			key_task.key_speed.Vy = -key_task.key_speed.speed / Slow;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_W + KEY_PRESSED_OFFSET_D + KEY_PRESSED_OFFSET_SHIFT))
		{
			key_task.key_speed.Vx = key_task.key_speed.speed / Slow;
			key_task.key_speed.Vy = key_task.key_speed.speed / Slow;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_S + KEY_PRESSED_OFFSET_A + KEY_PRESSED_OFFSET_SHIFT))
		{
			key_task.key_speed.Vx = -key_task.key_speed.speed / Slow;
			key_task.key_speed.Vy = -key_task.key_speed.speed / Slow;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_S + KEY_PRESSED_OFFSET_D + KEY_PRESSED_OFFSET_SHIFT))
		{
			key_task.key_speed.Vx = -key_task.key_speed.speed / Slow;
			key_task.key_speed.Vy = key_task.key_speed.speed / Slow;
		}

		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_D + KEY_PRESSED_OFFSET_CTRL))
		{
			key_task.key_speed.Vy = key_task.key_speed.speed + Fast;
			key_task.key_speed.Vx = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_A + KEY_PRESSED_OFFSET_CTRL))
		{
			key_task.key_speed.Vy = -(key_task.key_speed.speed + Fast);
			key_task.key_speed.Vx = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_S + KEY_PRESSED_OFFSET_CTRL))
		{
			key_task.key_speed.Vx = -(key_task.key_speed.speed + Fast);
			key_task.key_speed.Vy = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_W + KEY_PRESSED_OFFSET_CTRL))
		{
			key_task.key_speed.Vx = key_task.key_speed.speed + Fast;
			key_task.key_speed.Vy = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_W + KEY_PRESSED_OFFSET_A + KEY_PRESSED_OFFSET_CTRL))
		{
			key_task.key_speed.Vx = key_task.key_speed.speed + Fast;
			key_task.key_speed.Vy = -(key_task.key_speed.speed + Fast);
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_W + KEY_PRESSED_OFFSET_D + KEY_PRESSED_OFFSET_CTRL))
		{
			key_task.key_speed.Vx = key_task.key_speed.speed + Fast;
			key_task.key_speed.Vy = key_task.key_speed.speed + Fast;
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_S + KEY_PRESSED_OFFSET_A + KEY_PRESSED_OFFSET_CTRL))
		{
			key_task.key_speed.Vx = -(key_task.key_speed.speed + Fast);
			key_task.key_speed.Vy = -(key_task.key_speed.speed + Fast);
		}
		else if (key_task.key_rc_ctrl->key.v == (KEY_PRESSED_OFFSET_S + KEY_PRESSED_OFFSET_D + KEY_PRESSED_OFFSET_CTRL))
		{
			key_task.key_speed.Vx = -(key_task.key_speed.speed + Fast);
			key_task.key_speed.Vy = key_task.key_speed.speed + Fast;
		}

		else
		{
			key_task.key_speed.Vx = 0;
			key_task.key_speed.Vy = 0;
		}
		/*切方向，鼠标横移可切换底盘的方向*/
		// 实测发现卡顿，调整滤波间隔有改善但是依旧不比遥控拨杆，且牵扯多。
		///*TODO
		if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_W || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_A || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_S || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_D)
		{
			key_task.key_speed.Vz = key_task.key_rc_ctrl->mouse.x * 1.0F;
		}
		else
			key_task.key_speed.Vz = key_task.key_rc_ctrl->mouse.x * 2.0F; //(此处有限制速度大小，所以给大点就好)
		//*/

		//		if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_W || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_A || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_S || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_D)
		//		{
		//			key_task.key_speed.Vz = key_task.key_rc_ctrl->rc.ch[0] / 2.0;//
		//		}
		//		else
		//			key_task.key_speed.Vz = key_task.key_rc_ctrl->rc.ch[0]; //
		//*/
		limit_key_int16(&key_task.key_speed.Vz, 660, -660);
	}

	else
	{
		key_task.key_speed.Vx = 0;
		key_task.key_speed.Vy = 0;
		key_task.key_speed.Vz = 0;
	}
}
void custom_chassis_mode(void) // 切换各种模式
{
	// 左键底盘运动，右键键盘取矿，双键自定义控制器
	if (key_task.key_rc_ctrl->mouse.press_r && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_G && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_Z && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_G + KEY_PRESSED_OFFSET_Z) // 取矿safe模式//!v2.0模式下可作为键鼠控制中转
	{
		/*取矿模式*/
		key_task.chassis_mode = 1;
		/*自定义控制器关闭*/
		key_task.custom_mode = 0;

		/*新增mode*/
		key_task.Base = 0;
		key_task.Safe = 1;
		key_task.Cashing = 0;
		/*一键兑矿+Yaw*/
		key_task.Cashing_yaw = 0;
		// key_task.Gold_slanting_near = 0;
		key_task.Gold = 0;
		key_task.Silver = 0;
		key_task.deposit = 0;
		key_task.extraction = 0;

		/*配合chassis_move_flag*/

		// 一键取金矿标志位
		//		Gold_flag = 0;

		// 一键取银矿标志位
		//		Silver_flag = 0;

		// 一键存矿标志位
		//		deposit_flag = 0;
	}
	else if (key_task.key_rc_ctrl->mouse.press_l) // 底盘运动模式//取矿模式切换为底盘模式时位姿也不发生改变
	{
		/*底盘运动*/
		key_task.chassis_mode = 0;
		/*自定义控制器关闭*/
		key_task.custom_mode = 0;

		/*新增*/
		key_task.Base = 0;
		key_task.Safe = 0;
		key_task.Cashing = 0;
		key_task.Cashing_yaw = 0;
		// key_task.Gold_slanting_near = 0;
		key_task.Gold = 0;
		key_task.Silver = 0;

		key_task.deposit = 0;
		key_task.extraction = 0;

		//!/*底盘运动模式->任意取矿模式*/
		chassis_move_flag = 1;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_Q && key_task.Cashing == 0) // 一键小兑矿
	{
		/*取矿模式*/
		key_task.chassis_mode = 1;
		/*自定义控制器关闭*/
		key_task.custom_mode = 0;

		/*新增mode*/
		key_task.Base = 0;
		key_task.Safe = 0;
		key_task.Cashing = 1;
		key_task.Cashing_yaw = 0;
		// key_task.Gold_slanting_near = 0;
		key_task.Gold = 0;
		key_task.Silver = 0;

		key_task.deposit = 0;
		key_task.extraction = 0;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_Q && key_task.Cashing_yaw == 0) // 一键大兑矿
	{
		/*取矿模式*/
		key_task.chassis_mode = 1;
		/*自定义控制器关闭*/
		key_task.custom_mode = 0;

		/*新增mode*/
		key_task.Base = 0;
		key_task.Safe = 0;
		key_task.Cashing = 0;
		key_task.Cashing_yaw = 1;
		// key_task.Gold_slanting_near = 0;
		key_task.Gold = 0;
		key_task.Silver = 0;

		key_task.deposit = 0;
		key_task.extraction = 0;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_R && key_task.Gold == 0) // 一键取金矿
	{
		/*取矿模式*/
		key_task.chassis_mode = 1;
		/*自定义控制器关闭*/
		key_task.custom_mode = 0;

		/*新增mode*/
		key_task.Base = 0;
		key_task.Safe = 0;
		key_task.Cashing = 0;
		key_task.Cashing_yaw = 0;
		//		key_task.Gold_slanting_near = 0;
		key_task.Gold = 1;
		key_task.Silver = 0;

		key_task.deposit = 0;
		key_task.extraction = 0;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_E && key_task.Silver == 0) // 取地面银矿模式
	{
		/*取矿模式*/
		key_task.chassis_mode = 1;
		/*自定义控制器关闭*/
		key_task.custom_mode = 0;

		/*新增mode*/
		key_task.Base = 0;
		key_task.Safe = 0;
		key_task.Cashing = 0;
		key_task.Cashing_yaw = 0;
		// key_task.Gold_slanting_near = 0;
		key_task.Gold = 0;
		key_task.Silver = 1;

		key_task.deposit = 0;
		key_task.extraction = 0;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_C && key_task.Base == 0) // BASE模式
	{
		/*取矿模式*/
		key_task.chassis_mode = 0;
		/*自定义控制器关闭*/
		key_task.custom_mode = 0;

		//		dm_motor_enable(&hcan1, &DM_yaw);
		////	HAL_Delay(500); //! pitch轴使能延时，测试
		//	    dm_motor_enable(&hcan1, &DM_pitch);

		/*新增mode*/
		key_task.Base = 1;
		key_task.Safe = 0;
		key_task.Cashing = 0;
		key_task.Cashing_yaw = 0;
		// key_task.Gold_slanting_near = 0;
		key_task.Gold = 0;
		key_task.Silver = 0;

		key_task.deposit = 0;
		key_task.extraction = 0;

		// 一键取金矿标志位
		Gold_flag = 0;

		// 一键取银矿标志位
		Silver_flag = 0;

		// 一键存矿标志位
		deposit_flag = 0;

		// 一键取存的矿标志位
		extraction_flag = 0;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_B)
	{
		HAL_Delay(500);
		dm_motor_enable(&hcan1, &DM_yaw);
		HAL_Delay(500);
		dm_motor_enable(&hcan1, &DM_pitch);
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_B && key_task.custom_mode == 0) // 底盘+自定义小臂取矿模式
	{
		/*底盘运动+自定义取矿模式*/
		key_task.chassis_mode = 0;

		/*保存当前角度值作为初始值*/
		for (int i = 0; i < 4; i++)
		{
			// 重置滤波器状态
			angle_filters[i].sum = 0;
			angle_filters[i].index = 0;
			angle_filters[i].last_valid = Custom_data[i].angle; // 保存当前值

			// 用当前值填充整个滤波器窗口
			for (int j = 0; j < FILTER_WINDOW_SIZE; j++)
			{
				angle_filters[i].buffer[j] = Custom_data[i].angle;
				angle_filters[i].sum += Custom_data[i].angle;
			}
		}

		yaw_custom = 0.0f;
		pic_custom = 0.0f;
		Cup_pitch_custom = 0.0f;
		Cup_roll_custom = 0.0f;

		/*自定义控制器启动*/
		key_task.custom_mode = 1;

		/*新增mode*/
		key_task.Base = 0;
		key_task.Safe = 0;
		key_task.Cashing = 0;
		key_task.Cashing_yaw = 0;
		// key_task.Gold_slanting_near = 0;
		key_task.Gold = 0;
		key_task.Silver = 0;

		key_task.deposit = 0;
		key_task.extraction = 0;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_G || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_G + KEY_PRESSED_OFFSET_W || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_G + KEY_PRESSED_OFFSET_A || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_G + KEY_PRESSED_OFFSET_S || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_G + KEY_PRESSED_OFFSET_D) // 底盘+自定义取矿模式
	{
		Spinning_Top_R = 1;
		Spinning_Top_L = 0;
		// /*用于给自定义控制器是否使用X横向移动使用*/
		// Gantry_flag = 0;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_F || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_F + KEY_PRESSED_OFFSET_W || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_F + KEY_PRESSED_OFFSET_A || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_F + KEY_PRESSED_OFFSET_S || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_F + KEY_PRESSED_OFFSET_D) // 底盘+自定义取矿模式
	{
		Spinning_Top_R = 0;
		Spinning_Top_L = 1;
		// /*用于给自定义控制器是否使用X横向移动使用*/
		// Gantry_flag = 1;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_Z && key_task.deposit == 0) // 底盘+存矿(此时需要调节前伸)
	{
		/*底盘运动+自定义取矿模式*/
		key_task.chassis_mode = 0;
		/*自定义控制器启动*/
		key_task.custom_mode = 0;

		/*新增mode*/
		key_task.Base = 0;
		key_task.Safe = 0;
		key_task.Cashing = 0;
		key_task.Cashing_yaw = 0;
		// key_task.Gold_slanting_near = 0;
		key_task.Gold = 0;
		key_task.Silver = 0;

		key_task.deposit = 1;
		key_task.extraction = 0;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_X && key_task.deposit == 0) // 底盘+取矿(此时需要调节前伸)
	{
		/*底盘运动+自定义取矿模式*/
		key_task.chassis_mode = 0;
		/*自定义控制器启动*/
		key_task.custom_mode = 0;

		/*新增mode*/
		key_task.Base = 0;
		key_task.Safe = 0;
		key_task.Cashing = 0;
		key_task.Cashing_yaw = 0;
		// key_task.Gold_slanting_near = 0;
		key_task.Gold = 0;
		key_task.Silver = 0;

		key_task.deposit = 0;
		key_task.extraction = 1;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_E)
	{
		direction = 0;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_R)
	{
		direction = 1;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_B)
	{
		Cup_moto[0].round_cnt = 0;
		Cup_moto[1].round_cnt = 0;
		X_moto.round_cnt = 0;
		Cup_X_reset = 1;
		osDelay(1000);
	}
	else if (key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_B)
	{
		Cup_X_reset = 0;
	}

	/*松开才置为1，然后再次按下相应模式才会进行对应模式复位*/ //! 核心思路：判断按键抬起后再进行操作
	if (!chassis_move_flag && !key_task.key_rc_ctrl->mouse.press_r && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_R && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_Q && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_Q && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_Z && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_X && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_E && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_C && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_B)
		Again_flag = 1;

	if (key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_G && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_F)
	{
		Spinning_Top_R = 0;
		Spinning_Top_L = 0;
	}
}
/*用于模式切换和按键微调位置*/
void button_gantry(void)
{
	if (!key_task.chassis_mode && !key_task.Base && !Gantry_flag && Lift_Down)
	{
		/*龙门架*/
		if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_E) //&& compute_flag
		{
			//! 不能这样用，因为在循环中v的值没法更改,解决办法：应仿照Again_flag解决 // while (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_Q)
			Z_target += 0.02;
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_R) //&& compute_flag
		{
			Z_target -= 0.02;
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_F) //&& compute_flag
		{
			Y_target += 0.05;
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_G) //&& compute_flag
		{
			Y_target -= 0.05;
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_V) //&& compute_flag
		{
			X_target += 0.1;
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_B) //&& compute_flag
		{
			X_target -= 0.1;
			// compute_flag = 0;
		}
		/*龙门架*/
	}
}
void button_position(void) // 绝对位置+相对位置传入参数//************改为长按了
{
	/*条件判断里可以添加compute_flag来实现按一次控制一次，现在是持续按下控制*/
	// 考虑是否需要在对应的模式下才能使用一个或几个
	if (key_task.chassis_mode == 1 && !key_task.Base && Lift_Down) // 鼠标右键//!按键控制位置加减
	{
		/*龙门架*/
		if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_Q) //&& compute_flag
		{
			//! 不能while这样用，因为在循环中v的值没法更改,解决办法：应仿照Again_flag解决
			if (!direction)
			{
				Z_target -= 0.02;
			}
			else
			{
				Z_target += 0.02;
			}

			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_W) //&& compute_flag
		{
			if (!direction)
			{
				Z_target += 0.02;
			}
			else
			{
				Z_target -= 0.02;
			}
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_A) //&& compute_flag
		{
			if (!direction)
			{
				Y_target -= 0.05;
			}
			else
			{
				Y_target += 0.05;
			}
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_S) //&& compute_flag
		{
			if (!direction)
			{
				Y_target += 0.05;
			}
			else
			{
				Y_target -= 0.05;
			}
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_Z) //&& compute_flag
		{
			X_target += 0.1;
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_X) //&& compute_flag
		{
			X_target -= 0.1;
			// compute_flag = 0;
		}
		/*龙门架*/

		/*机械臂*/
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_E) //&& compute_flag
		{
			//			if(!direction)
			//			{
			yaw += 0.05;
			//			}
			//			else
			//			{
			//			   yaw -= 0.05;
			//			}
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_R) //&& compute_flag
		{
			//			if(!direction)
			//			{
			yaw -= 0.05;
			//			}
			//			else
			//			{
			//			   yaw += 0.05;
			//			}
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_D) //&& compute_flag
		{
			//			if(!direction)
			//			{
			pic -= 0.1;
			//			}
			//			else
			//			{
			//			   pic += 0.1;
			//			}
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_F) //&& compute_flag
		{
			//			if(!direction)
			//			{
			pic += 0.1;
			//			}
			//			else
			//			{
			//			   pic -= 0.1;
			//			}
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_C) //&& compute_flag
		{
			if (!direction)
			{
				Cup_target_pitch -= 0.1;
			}
			else
			{
				Cup_target_pitch += 0.1;
			}
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_V) //&& compute_flag
		{
			if (!direction)
			{
				Cup_target_pitch += 0.1;
			}
			else
			{
				Cup_target_pitch -= 0.1;
			}
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_G) //&& compute_flag
		{
			if (!direction)
			{
				Cup_target_roll -= 0.1;
			}
			else
			{
				Cup_target_roll += 0.1;
			}
			// compute_flag = 0;
		}
		else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_B) //&& compute_flag
		{
			if (!direction)
			{
				Cup_target_roll += 0.1;
			}
			else
			{
				Cup_target_roll -= 0.1;
			}
			// compute_flag = 0;
		}
		/*按键按一次作用一次*/ //! 测试使用成功，效果良好
		if (key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_Q && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_W && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_A && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_S && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_Z && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_X && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_E && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_R && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_D && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_F && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_C && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_V && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_G && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_B)
		{
			compute_flag = 1;
		}
	}
}
void Custom_position(void)
{
	/*先判断是否由底盘运动模式而来*/
	if (chassis_move_flag)
	{
		// 变量自我赋值相当于不对变量进行赋值操作
		chassis_move_flag = 0;
		Again_flag = 0;
		// HAL_Delay(200);
		return;
	}

	if (!Again_flag)
	{
		return;
	}
	// /*初始进入未按下Shift+G时（即：Shift+B），横移固定在中间，*/
	// if (!Gantry_flag)
	// {
	yaw = yaw_custom;					 // DM_yaw   //贴抬升1973~-144                            //1050对应90度
	pic = pic_custom;					 // DM_pitch//-1420~2580//下中-2425，上中-420             //1050对应90度
	Cup_target_pitch = Cup_pitch_custom; // 1220~-1000//平行274//-763							 //991.5对应90度
	Cup_target_roll = Cup_roll_custom;
	// }
	// /*可以在Shift+G和Shift+B之间来回切换，控制横移是否固定*/
	// else
	// {
	// 	yaw = yaw_custom;					 // DM_yaw   //贴抬升1973~-144                            //1050对应90度
	// 	pic = pic_custom;					 // DM_pitch//-1420~2580//下中-2425，上中-420             //1050对应90度
	// 	Cup_target_pitch = Cup_pitch_custom; // 1220~-1000//平行274//-763							 //991.5对应90度
	// 	Cup_target_roll = Cup_roll_custom;
	// 	/*龙门架*/
	// 	X_target = X_custom;
	// 	Y_target = Y_custom;
	// 	Z_target = Z_custom; /// 4096.0  * 8192.0//		Uplift_location(Z_target);
	// }
}
/*各个取矿模式对应初始位置*/
void Base_position(void) // 蜷缩往回收  //!加入条件判断后，需要点击一次及以上按键才会完全归位 //试试加一下归位后将编码器总角度也归为原状（直接圈数归零，custom计算当前总角度）
{
	// Cup_Fart();
	/*机械臂吸盘*/
	Cup_target_pitch = 0.0f;
	Cup_target_roll = 0.0f;
	/*机械臂pitch*/
	if (!direction)
	{
		pic = Pitch_obtain;
	}
	else
	{
		pic = -Pitch_obtain;
	}
	/*机械臂Yaw*/
	yaw = 0.0f;
	/*Uplift*/
	Z_target = 0.0f;
	// /*X*/
	X_target = 0.0f;
	/*Y*/
	Y_target = 0.0f;
}
void Safe_position(void) // 一键取矿初始化，向外伸
{
	/*先判断是否由底盘运动模式而来*/
	if (chassis_move_flag)
	{
		// 变量自我赋值相当于不对变量进行赋值操作
		chassis_move_flag = 0;
		Again_flag = 0;
		// HAL_Delay(200);
		return;
	}

	if (!Again_flag)
	{
		return;
	}
	/*Uplift*/
	Z_target = Uplift_obtain;
	/*Y*/
	Y_target = Y_obtain;
	// /*X*/
	// X_target = X_obtain;
	/*机械臂吸盘*/
	Cup_target_pitch = Cup_pitch_obtain;
	Cup_target_roll = Cup_roll_obtain;
	/*机械臂Yaw*/
	yaw = Yaw_obtain;
	/*机械臂pitch*/
	if (!direction)
	{
		pic = Pitch_obtain;
	}

	else
	{
		pic = -Pitch_obtain;
	}
}
void Cashing_position(void) // 一键兑矿
{
	/*先判断是否由底盘运动模式而来*/
	if (chassis_move_flag)
	{
		// 变量自我赋值相当于不对变量进行赋值操作
		chassis_move_flag = 0;
		return;
	}
	if (!Again_flag)
	{
		return;
	}

	if (!direction)
	{
		/*Uplift*/
		Z_target = 29.0f;
		/*Y*/
		Y_target = Y_Cash_0;
		// /*X*/
		// X_target = X_Cash_0;
		/*机械臂吸盘*/
		Cup_target_pitch = Cup_pitch_Cash_0;
		Cup_target_roll = Cup_roll_Cash_0;
		/*机械臂Yaw*/
		yaw = Yaw_Cash_0;
		if (DM_yaw.para.pos > yaw_temp - 0.1 && DM_yaw.para.pos < yaw_temp + 0.1)
		{
			/*机械臂pitch*/
			pic = Pitch_Cash_0;
		}
	}
	else
	{
		/*Uplift*/
		Z_target = 29.0f; // 27.3
		/*Y*/
		Y_target = Y_Cash_1;
		// /*X*/
		// X_target = X_Cash_1;
		/*机械臂吸盘*/
		Cup_target_pitch = Cup_pitch_Cash_1;
		Cup_target_roll = Cup_roll_Cash_1;
		/*机械臂Yaw*/
		yaw = Yaw_Cash_1;
		if (DM_yaw.para.pos > yaw_temp - 0.1 && DM_yaw.para.pos < yaw_temp + 0.1)
		{
			/*机械臂pitch*/
			pic = Pitch_Cash_1;
		}
	}
	Cup_Suck();
}

void Cashing_yaw_position(void)
{
	/*先判断是否由底盘运动模式而来*/
	if (chassis_move_flag)
	{
		// 变量自我赋值相当于不对变量进行赋值操作
		chassis_move_flag = 0;
		return;
	}
	if (!Again_flag)
	{
		return;
	}

	if (!direction)
	{
		/*Uplift*/
		Z_target = 25.5;
		/*Y*/
		Y_target = Y_Cash_yaw_0;
		// /*X*/
		// X_target = X_Cash_0;
		/*机械臂吸盘*/
		Cup_target_pitch = Cup_pitch_Cash_yaw_0;
		Cup_target_roll = Cup_roll_Cash_yaw_0;
		/*机械臂Yaw*/
		yaw = Yaw_Cash_yaw_0;
		if (DM_yaw.para.pos > yaw_temp - 0.1 && DM_yaw.para.pos < yaw_temp + 0.1)
		{
			/*机械臂pitch*/
			pic = Pitch_Cash_yaw_0;
		}
	}
	else
	{
		/*Uplift*/
		Z_target = 25.5; // 27.3
		/*Y*/
		Y_target = Y_Cash_yaw_1;
		// /*X*/
		// X_target = X_Cash_1;
		/*机械臂吸盘*/
		Cup_target_pitch = Cup_pitch_Cash_yaw_1;
		Cup_target_roll = Cup_roll_Cash_yaw_1;
		/*机械臂Yaw*/
		yaw = Yaw_Cash_yaw_1;
		if (DM_yaw.para.pos > yaw_temp - 0.1 && DM_yaw.para.pos < yaw_temp + 0.1)
		{
			/*机械臂pitch*/
			pic = Pitch_Cash_yaw_1;
		}
	}
	Cup_Suck();
}

void Gold_position(void) // 一键取金矿初始化
{
	/*先判断是否由底盘运动模式而来*/
	if (chassis_move_flag)
	{
		// 变量自我赋值相当于不对变量进行赋值操作
		chassis_move_flag = 0;
		Again_flag = 0;
		return;
	}

	if (!Again_flag)
	{
		return;
	}
	if (!Gold_flag)
	{
		/*Uplift*/
		Z_target = Uplift_GOLD;
		/*Y*/
		Y_target = Y_gold;
		// /*X*/
		// X_target = X_gold;
		/*机械臂吸盘*/
		Cup_target_pitch = Cup_pitch_gold;
		Cup_target_roll = Cup_roll_gold;
		/*机械臂Yaw*/
		yaw = Yaw_gold;
		/*机械臂pitch*/
		if (!direction)
		{
			pic = Pitch_gold;
		}
		else
		{
			pic = -Pitch_gold;
		}
		Cup_Suck();
		Gold_flag = 1;
	}
	else if ((Uplift_moto[0].total_angle > -(float)8192 * 19.0 / 18.0 * Z_target - 2048.0f && Uplift_moto[0].total_angle < -(float)8192 * 19.0 / 18.0 * Z_target + 2048.0f) && Gold_flag == 1)
	{
		Y_target = 30.0f;
		Gold_flag = 2;
	}
	else if ((Y_moto.total_angle > -(float)8192 * 36.0 / 14.0 * Y_target - 2048.0f && Y_moto.total_angle < -(float)8192 * 36.0 / 14.0 * Y_target + 2048.0f) && Gold_flag == 2)
	{
		Z_target = 7.8f;
		Gold_flag = 3;
	}
	else if ((Uplift_moto[0].total_angle > -(float)8192 * 19.0 / 18.0 * Z_target - 2048.0f && Uplift_moto[0].total_angle < -(float)8192 * 19.0 / 18.0 * Z_target + 2048.0f) && Gold_flag == 3)
	{
		Y_target = 5.0f;
		Gold_flag = 4;
	}
}
void Silver_position(void) // 一键取地面银矿初始化
{
	/*先判断是否由底盘运动模式而来*/
	if (chassis_move_flag)
	{
		// 变量自我赋值相当于不对变量进行赋值操作
		chassis_move_flag = 0;
		Again_flag = 0;
		return;
	}

	if (!Again_flag)
	{
		return;
	}

	yaw = Yaw_silver;
	// 七步一键
	// 第一步：抬升；
	if (!Silver_flag)
	{
		/*Uplift*/
		Z_target = Uplift_silver_1;
		pic = Pitch_silver_1;
		// 新添
		Y_target = 10.0f;
		Cup_target_pitch = Cup_pitch_silver_1;
		Cup_Suck();
		Silver_flag = 1;
	}
	// 第二步：前伸+吸盘垂直+开气泵
	// else if ((Uplift_moto[0].total_angle > -(float)8192 * 19.0 / 18.0 * Z_target - 2048.0f && Uplift_moto[0].total_angle < -(float)8192 * 19.0 / 18.0 * Z_target + 2048.0f) && Silver_flag == 1)
	// {
	// 	Y_target = Y_silver_1;
	// 	Cup_target_pitch = Cup_pitch_silver_1;
	// 	Cup_Suck();
	// 	Silver_flag = 2;
	// }
	// 第三步：抬升下压
	else if ((Y_moto.total_angle > -(float)8192 * 36.0 / 14.0 * Y_target - 2048.0f && Y_moto.total_angle < -(float)8192 * 36.0 / 14.0 * Y_target + 2048.0f) && Silver_flag == 1)
	{
		/*Uplift*/
		Z_target = Uplift_silver_2;
		Silver_flag = 2;
	}

	// // 第四步：吸上矿之后抬升出来
	else if ((Uplift_moto[0].total_angle > -(float)8192 * 19.0 / 18.0 * Z_target - 2048.0f && Uplift_moto[0].total_angle < -(float)8192 * 19.0 / 18.0 * Z_target + 2048.0f) && Silver_flag == 2)
	{
		/*Uplift*/
		Z_target = 15.0f; // Uplift_silver_3;
		Silver_flag = 3;
	}
	// // 需补充：前伸放矿；回缩小臂吸盘抬升；前伸吸;抬升
	// //! 为了矿石条形码对应到矿石兑换区
	// // 补充一：前伸放矿
	else if ((Uplift_moto[0].total_angle > -(float)8192 * 19.0 / 18.0 * Z_target - 2048.0f && Uplift_moto[0].total_angle < -(float)8192 * 19.0 / 18.0 * Z_target + 2048.0f) && Silver_flag == 3)
	{
		Y_target = 40.0f; // Y_silver_1 + 10.0f;
		Cup_target_pitch = 0.0f;
		Silver_flag = 4;
	}
	// // 补充二：回缩小臂吸盘抬升
	else if ((Y_moto.total_angle > -(float)8192 * 36.0 / 14.0 * Y_target - 2048.0f && Y_moto.total_angle < -(float)8192 * 36.0 / 14.0 * Y_target + 2048.0f) && Silver_flag == 4)
	{
		Y_target = 41.0f;
		Cup_Fart();
		Silver_flag = 5;
	}
	else if ((Y_moto.total_angle > -(float)8192 * 36.0 / 14.0 * Y_target - 2048.0f && Y_moto.total_angle < -(float)8192 * 36.0 / 14.0 * Y_target + 2048.0f) && Silver_flag == 5)
	{
		Y_target = 10.0f;		  // Y_silver_1 - 10.0f;
								  // Uplift_silver_2 + 0.15f;
		Cup_target_pitch = 83.0f; // Cup_pitch_silver_1 + 90.0f;
		Cup_Suck();
		Silver_flag = 6;
	}
	// // 补充三：前伸吸
	// else if ((Uplift_moto[0].total_angle > -(float)8192 * 19.0 / 18.0 * Z_target - 2048.0f && Uplift_moto[0].total_angle < -(float)8192 * 19.0 / 18.0 * Z_target + 2048.0f) && Silver_flag == 7)
	// {
	// 	Y_target = 39.0f; // Y_silver_1 + 5.0f;
	// 	Cup_Suck();
	// 	Silver_flag = 8;
	// }
	// // 补充四：抬升
	else if ((Y_moto.total_angle > -(float)8192 * 36.0 / 14.0 * Y_target - 2048.0f && Y_moto.total_angle < -(float)8192 * 36.0 / 14.0 * Y_target + 2048.0f) && Silver_flag == 6)
	{
		Z_target = 3.0f;
		Silver_flag = 7;
	}
	// // 第五步：小臂朝上
	// else if ((Uplift_moto[0].total_angle > -(float)8192 * 19.0 / 18.0 * Z_target - 2048.0f && Uplift_moto[0].total_angle < -(float)8192 * 19.0 / 18.0 * Z_target + 2048.0f) && Silver_flag == 9)
	// {
	// 	Y_target = 24.0f; // Y_silver_1 - 10.0f;
	// 	if (!direction)
	// 	{
	// 		pic = Pitch_silver_2;
	// 	}
	// 	else
	// 	{
	// 		pic = -Pitch_silver_2;
	// 	}
	// 	Silver_flag = 10;
	// }
	// // 第六步：回伸
	// else if ((DM_pitch.para.pos > PI / 180.0f * pic - 0.1 && DM_pitch.para.pos < PI / 180.0f * pic + 0.1) && Silver_flag == 10)
	// {
	// 	Y_target = Y_silver_3;
	// 	Z_target = Uplift_silver_4;
	// 	Cup_target_pitch = Cup_pitch_silver_2;
	// 	Silver_flag = 11;
	// }
}
/*用于模式切换和按键微调*/
/*一键存矿*/
void Deposit(void) // TODO可能需要两次乃至三次按键才可到达
{
	/*//分区赛后更新存矿仅右存矿，更改：1.存矿前方向判断是否为右存矿；
	2.开始存矿（参数调整）（小臂yaw归零）
	3.存矿结束后切换至方向左*/

	// 更新：存矿前判断；
	if (!direction && deposit_flag != 6)
	{
		direction = 1;
		return;
	}

	// 第一步：前伸+吸盘
	if (!deposit_flag)
	{
		/*Z*/
		Z_target = 7.0f; //! 机械位置调整后随之提高，同时避免了过低导致的矿石卡地面掉落的问题
		/*Y*/
		Y_target = Y_deposit_1;
		Cup_target_pitch = -62.5f;
		deposit_flag = 1;
	}
	// 第二步：小臂pitch翻转90度（左0右1）
	else if ((Y_moto.total_angle > -(float)8192 * 36.0 / 14.0 * Y_target - 2048.0f && Y_moto.total_angle < -(float)8192 * 36.0 / 14.0 * Y_target + 2048.0f) && deposit_flag == 1)
	{
		/*Z*/
		// Z_target = 5.0f;
		if (!direction)
		{
			pic = Pitch_deposit;
		}
		else
		{
			pic = -Pitch_deposit;
		}
		deposit_flag = 2;
	}
	// 第三步：小臂yaw翻转60度（左0右1）存矿泵开
	else if ((DM_pitch.para.pos > PI / 180.0f * pic - 0.1 && DM_pitch.para.pos < PI / 180.0f * pic + 0.1) && deposit_flag == 2)
	{
		if (!direction)
		{
			yaw = Yaw_deposit;
			Left_Suck();
		}
		else
		{
			yaw = -Yaw_deposit;
			Right_Suck();
		}
		deposit_flag = 3;
	}
	// 吸盘泵关
	else if ((DM_yaw.para.pos > PI / 180.0f * yaw - 0.1 && DM_yaw.para.pos < PI / 180.0f * yaw + 0.1) && deposit_flag == 3)
	{
		Cup_Fart();
		deposit_flag = 4;
	}
	else if (deposit_flag == 4)
	{
		Y_target = Y_deposit_1 + 3.0f;
		yaw = 0.0f;
		deposit_flag = 5;
	}
	else if ((DM_yaw.para.pos > PI / 180.0f * yaw - 0.1 && DM_yaw.para.pos < PI / 180.0f * yaw + 0.1) && deposit_flag == 5)
	{
		pic = 0.0f;
		Cup_target_pitch = -15.0f;
		direction = 0;
		deposit_flag = 6;
	}
}
/*一键取矿*/
void Extraction(void)
{
	// 更新：存矿前判断；
	if (!direction && extraction_flag != 6)
	{
		direction = 1; // 右取矿
		return;
	}

	/*Z*/
	Z_target = Uplift_deposit; // TODO:和上面存矿高度一样

	// 第一步：前伸+吸盘
	if (!extraction_flag)
	{
		/*Y*/
		Y_target = Y_deposit_1;
		Cup_target_pitch = -62.5f;
		extraction_flag = 1;
	}
	// 第二步：小臂pitch翻转90度（左0右1）
	else if ((Y_moto.total_angle > -(float)8192 * 36.0 / 14.0 * Y_target - 2048.0f && Y_moto.total_angle < -(float)8192 * 36.0 / 14.0 * Y_target + 2048.0f) && extraction_flag == 1)
	{
		if (!direction)
		{
			pic = Pitch_deposit;
		}
		else
		{
			pic = -Pitch_deposit;
		}
		extraction_flag = 2;
	}
	// 第三步：小臂yaw翻转60度（左0右1）取矿泵开
	else if ((DM_pitch.para.pos > PI / 180.0f * pic - 0.1 && DM_pitch.para.pos < PI / 180.0f * pic + 0.1) && extraction_flag == 2)
	{
		Cup_Suck();
		if (!direction)
		{
			yaw = Yaw_deposit;
		}
		else
		{
			yaw = -Yaw_deposit;
		}
		extraction_flag = 3;
	}
	// 吸盘泵关
	else if ((DM_yaw.para.pos > PI / 180.0f * yaw - 0.1 && DM_yaw.para.pos < PI / 180.0f * yaw + 0.1) && extraction_flag == 3)
	{
		if (!direction)
		{
			Left_Fart();
		}
		else
		{
			Right_Fart();
		}
		extraction_flag = 4;
	}
	else if (extraction_flag == 4)
	{
		Y_target = Y_deposit_1 + 3.0f;
		yaw = 0.0f;
		extraction_flag = 5;
	}
	else if ((DM_yaw.para.pos > PI / 180.0f * yaw - 0.1 && DM_yaw.para.pos < PI / 180.0f * yaw + 0.1) && extraction_flag == 5)
	{
		pic = 0.0f;
		Cup_target_pitch = -15.0f;
		extraction_flag = 6;
	}
}
void motor_speed(void) // 切换完模式后计算目标值
{
	button_gantry();
	button_position();
	if (key_task.chassis_mode && key_task.Safe)
	{
		Safe_position();
		key_task.Safe = 0;
	}
	if (key_task.chassis_mode && key_task.Cashing)
	{
		Cashing_position();
		key_task.Cashing = 0;
	}
	if (key_task.chassis_mode && key_task.Cashing_yaw)
	{
		Cashing_yaw_position();
		key_task.Cashing_yaw = 0;
	}
	if (key_task.chassis_mode && key_task.Gold)
	{
		Gold_position();
		key_task.Gold = 0;
	}
	if (key_task.chassis_mode && key_task.Silver)
	{
		Silver_position();
		key_task.Silver = 0;
	}
	if (key_task.Base)
	{
		Base_position();
		key_task.Base = 0;
	}
	if (!key_task.chassis_mode && key_task.custom_mode)
	{
		Custom_position();
	}
	if (key_task.deposit)
	{
		Deposit();
		key_task.deposit = 0;
	}
	if (key_task.extraction)
	{
		Extraction();
		key_task.extraction = 0;
	}
}
void limit_key(float *value, float max, float min)
{
	if (*value > max)
		*value = max;
	else if (*value < min)
		*value = min;
}
void limit_key_int16(int16_t *value, int16_t max, int16_t min)
{
	if (*value > max)
		*value = max;
	else if (*value < min)
		*value = min;
}

/*
目标：
前伸2006复，基本在堵转电流值位置即为初始位置，亦步亦趋
小臂吸盘，堵转后再总角度第一次复位，到既定位置后在进行第二次总角度复位
*/
void resetValue(void)
{
	if (!reposition_flag_Y)
	{
		// TODO注意抬升电机是否抬升到目标位置，达到后再进行前伸复位
		if (Y_moto.real_current < 5000) // 10000电流值也需要实际测试
		{
			Y_target -= 0.01f; // limit限制用标志位条件判断加上或去掉
		}
		// 总角度清零
		else if (Y_moto.real_current >= 5000)
		{
			Y_moto.round_cnt = 0;
			Y_target = 0.5f;
			// 前伸此时为初始位置
			reposition_flag_Y = 1;
		}
	}
	// if (!reposition_flag_Cup)
	// {
	// 	if (Cup_moto[0].real_current < 3200) // 10000电流值也需要实际测试
	// 	{
	// 		Cup_target_pitch -= 0.05f; // limit限制用标志位条件判断加上或去掉
	// 	}
	// 	// 总角度清零
	// 	else if (Cup_moto[0].real_current >= 3200)
	// 	{
	// 		Cup_moto[0].round_cnt = 0;
	// 		Cup_moto[1].round_cnt = 0;
	// 		Cup_target_pitch = 60.0f;
	// 		// 前伸此时为初始位置
	// 		reposition_flag_Cup = 1;
	// 	}
	// }
	// if ((Cup_moto[0].total_angle > -(float)8192 * Multiple_Cup_pitch / 180 * Cup_target_pitch - 2048.0f && Cup_moto[0].total_angle < -(float)8192 * Multiple_Cup_pitch / 180 * Cup_target_pitch + 2048.0f) && reposition_flag_Cup == 1)
	// {
	// 	Cup_moto[0].round_cnt = 0;
	// 	Cup_moto[1].round_cnt = 0;
	// 	Cup_target_pitch = 0.0f;
	// 	reposition_flag_Cup = 2;
	// }
	// 这里后续添加小臂吸盘复位条件判断
	// if (0)
	// {
	// }
}

// 死亡重启，很重要，否则复活后电机会跳而且位置不对
// 目标：复位的时候要把抬升电机也进行复位(条件判断此时总角度编码值是否差距过大)掉电：直接cmd电流传送函数给零
void renew_mode(void)
{
	resetValue();
	if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_Z + KEY_PRESSED_OFFSET_X)
	{
		key_task.go_die_flag = 1;
		reposition_flag_Y = 0;
		// reposition_flag_Cup = 0;
	}
	else
		key_task.go_die_flag = 0;
	if (key_task.go_die_flag == 1) //|| die.die_flag == 1
	{
		osDelay(800); // 防止连按

		key_scan_init(&key_task);
		Base_position();
		//		image_task_init(&image_task);
		chassis_init();
		//		UI_init();
	}
}
void keyscan_task(void const *argument)
{
	osDelay(357);
	key_scan_init(&key_task);
	All_init();
	for (;;)
	{
		/*自动判断死亡*/
		/*//TODO好像死亡的时候直接断电了，不如重新上电后进行复位
		die.die_last = die.die_now;
		if (RM_Referee.robot_state.remain_HP == 0)
			die.die_now = 1;
		else
			die.die_now = 0;
		if (die.die_last == 0 && die.die_now == 1)
		{
			die.die_flag = 1;
			osDelay(20);
		}
		else
			die.die_flag = 0;
		*/

		// 小臂吸气放气控制
		if (rc_ctrl.key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_F || rc_ctrl.rc.ch[4] < -20) //
			Cup_Suck();
		else if (rc_ctrl.key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_G || rc_ctrl.rc.ch[4] > 20) //
			Cup_Fart();

		// 左存矿机构吸气放气
		if (rc_ctrl.key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_Z || rc_ctrl.rc.ch[4] < -20) //
			Left_Suck();
		else if (rc_ctrl.key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_X || rc_ctrl.rc.ch[4] > 20) //
			Left_Fart();

		if (rc_ctrl.key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_C || rc_ctrl.rc.ch[4] < -20) //
			Right_Suck();
		else if (rc_ctrl.key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_V || rc_ctrl.rc.ch[4] > 20) //
			Right_Fart();

		renew_mode();
		custom_chassis_mode();
		chassis_speed();
		motor_speed();
		// pos_update(&key_task);
		osDelay(1);
	}
}
