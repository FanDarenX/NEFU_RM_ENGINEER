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
 *            ���汣��     ����崻�     ����BUG
 *
 *        ��Ի:
 *                д��¥��д�ּ䣬д�ּ������Ա��
 *                ������Աд�������ó��򻻾�Ǯ��
 *                ����ֻ���������������������ߣ�
 *                ��������ո��գ����������긴�ꡣ
 *                ��Ը�������Լ䣬��Ը�Ϲ��ϰ�ǰ��
 *                ���۱������Ȥ���������г���Ա��
 *                ����Ц��߯��񲣬��Ц�Լ���̫����
 *                ��������Ư���ã��ĸ���ó���Ա��
 */

/*���ã�
1.�л�����ȡ�󡢵����˶���ģʽ;
2.���㡤�õ�Ŀ��ֵ����chassis��position�ȣ�
3.������λ����init��
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

/*�ṹ�塢����*/
key_typedef_t key_task;

int Cup_SF = 0;
int Right_SF = 0;
int Left_SF = 0;

/*��е������PE6->K1*/
/*��ŷ�PB0->L2*/
//! �Ҵ���������
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

//! С�����̿���
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
//! �����������
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
/*�̵���PC2->L1*/
void Relays_OFF() // �̵�������
{
	HAL_GPIO_WritePin(Relays_GPIO_Port, Relays_Pin, GPIO_PIN_RESET);
}
void Relays_ON() // �̵����õ�
{
	HAL_GPIO_WritePin(Relays_GPIO_Port, Relays_Pin, GPIO_PIN_SET);
}
/**
 *�޸ķ���
 *1.���̲��ֱ���
 *2.ң�������ֱ���
 *3.�޸�ȡ�󲿷�
 *4.�������ż�safe
 */
// TODOΪ�˼������������Ҫ��������GPIO��
/*λ�ñ���*/
/*FLAG*/
/*
ǰ�᣺�л�����ȡ��ģʽ->����key_init->chassis_mode��Ϊ1// 0Ϊ�����˶���1Ϊȡ��
1.��ʼ��Ϊ1����Ϊkey_init��ʼ��Ϊ�����˶�ģʽ��
2.�Ƶ�mouse_l��Ӧ��chasiss_move_flag��1��
3.�����л���������ȡ��ģʽ��������ģʽ������һ�¸���Ŀ��ֵΪ0.0f,Ȼ���ٰ���Ҫ�л���ģʽ���ᵽ��ģʽ�ĳ�ʼλ�ã�
4.����Ŀ��ֵ�����洢���ڴ���δ��������������ֵ�������л��������˶�ģʽ��λ�˲��䣬֮����Ҫ���л�ΪĿ��ģʽλ�ˣ�
�Ȱ�������ģʽ��������ʱ�ǣ�����=�������൱�����Ҹ�ֵ����������ǰλ�ˣ������ٴΰ���Ŀ��ģʽ�������л�����ģʽ��ʼλ�ã�// �������Ҹ�ֵ�൱�ڲ��Ա������и�ֵ����
�ڽ�������ȡ��ģʽĿ��ֵ����ĺ����н����жϸ�0
*/
//! 5.���ڰ���+-΢������ȡ��ʼ���ǵ�ǰĿ��ֵ���ڵ�ǰֵ�Ͻ��мӼ�������
int chassis_move_flag = 1; // ��ģʽ�л�λ�˱���������������Ҫ������
int Again_flag = 1;		   // ���chassis_move_flag�л�ģʽλ�˲���
int compute_flag = 1;	   // �����Ӽ����� //!��ʱ����

/*�Զ����������־λ*/
int Gantry_flag = 0; // Shift + G��С��0 | Shift + F����1

/*С���ݱ�־λ*/
int Spinning_Top_L = 0;
int Spinning_Top_R = 0;

/*ȡ�����*/
int direction = 1; //! �����ҲࣨCtrl+E��|R�ң�0��|1��  ������ʼΪ1������Ҵ��

/*���һ����־λ*/
int Gold_flag = 0;
/*����һ����־λ*/
int Silver_flag = 0;
/*һ�����*/
int deposit_flag = 0;
/*һ��ȡ��Ŀ�*/
int extraction_flag = 0;
/*һ���ҿ�*/
int cashing_flag = 0;

/*����||��λ��־λ*/
int reposition_flag_Y = 0;
int reposition_flag_Cup = 0;

/*Cup_pitch��λ*/
int Cup_X_reset = 0;

/*����ʣ��Ѫ���ж�����*/
die_t die;

void key_scan_init(key_typedef_t *key_init) // key��init����+ȡ��
{
	/*ң��������*/
	key_init->key_rc_ctrl = get_remote_control_point();
	/*����||ȡ��*/
	key_init->chassis_mode = 0; // mouse.l   // 0Ϊ�����˶���1Ϊȡ��
	/*�����Զ��������*/
	key_init->custom_mode = 0; // Shift + B // ��ʼ�������Զ������������

	/*����ģʽ*/
	/*��������ģʽ*/
	key_init->Base = 0; // Shift + C
	/*��ȫȡ���ʼ��*/
	key_init->Safe = 0; // mouse.r         //!�����ص������Ǹ���Ϊ������ʵ�õ�ģʽ�����磺��һ��||������λ
	/*һ���ҿ�*/
	key_init->Cashing = 0; //! һ���ҿ�
	/*һ���ҿ�+Yaw*/
	key_init->Cashing_yaw = 0;
	/*ȡ���*/
	key_init->Gold = 0; // Shift+R
	/*ȡ������*/
	key_init->Silver = 0; // Shift+E

	/*һ�����*/
	key_init->deposit = 0; // Shift + Z
	/*һ��ȡ��*/
	key_init->extraction = 0; // Shift + X

	/*����������־λ*/
	key_init->go_die_flag = 0; // Ctrl + Z + X
}
void chassis_speed(void) // �����ٶȼ��̿���
{
	// WD����Ϊ����SA����Ϊ��
	/*�Ҳ��˿��Ƶ����ٶ�*/
	if (key_task.chassis_mode == 0)
	{
		/*remote == 0 ���̲���*/
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
		/*�з��������ƿ��л����̵ķ���*/
		// ʵ�ⷢ�ֿ��٣������˲�����и��Ƶ������ɲ���ң�ز��ˣ���ǣ���ࡣ
		///*TODO
		if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_W || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_A || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_S || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_D)
		{
			key_task.key_speed.Vz = key_task.key_rc_ctrl->mouse.x * 1.0F;
		}
		else
			key_task.key_speed.Vz = key_task.key_rc_ctrl->mouse.x * 2.0F; //(�˴��������ٶȴ�С�����Ը����ͺ�)
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
void custom_chassis_mode(void) // �л�����ģʽ
{
	// ��������˶����Ҽ�����ȡ��˫���Զ��������
	if (key_task.key_rc_ctrl->mouse.press_r && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_G && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_Z && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_G + KEY_PRESSED_OFFSET_Z) // ȡ��safeģʽ//!v2.0ģʽ�¿���Ϊ���������ת
	{
		/*ȡ��ģʽ*/
		key_task.chassis_mode = 1;
		/*�Զ���������ر�*/
		key_task.custom_mode = 0;

		/*����mode*/
		key_task.Base = 0;
		key_task.Safe = 1;
		key_task.Cashing = 0;
		/*һ���ҿ�+Yaw*/
		key_task.Cashing_yaw = 0;
		// key_task.Gold_slanting_near = 0;
		key_task.Gold = 0;
		key_task.Silver = 0;
		key_task.deposit = 0;
		key_task.extraction = 0;

		/*���chassis_move_flag*/

		// һ��ȡ����־λ
		//		Gold_flag = 0;

		// һ��ȡ�����־λ
		//		Silver_flag = 0;

		// һ������־λ
		//		deposit_flag = 0;
	}
	else if (key_task.key_rc_ctrl->mouse.press_l) // �����˶�ģʽ//ȡ��ģʽ�л�Ϊ����ģʽʱλ��Ҳ�������ı�
	{
		/*�����˶�*/
		key_task.chassis_mode = 0;
		/*�Զ���������ر�*/
		key_task.custom_mode = 0;

		/*����*/
		key_task.Base = 0;
		key_task.Safe = 0;
		key_task.Cashing = 0;
		key_task.Cashing_yaw = 0;
		// key_task.Gold_slanting_near = 0;
		key_task.Gold = 0;
		key_task.Silver = 0;

		key_task.deposit = 0;
		key_task.extraction = 0;

		//!/*�����˶�ģʽ->����ȡ��ģʽ*/
		chassis_move_flag = 1;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_Q && key_task.Cashing == 0) // һ��С�ҿ�
	{
		/*ȡ��ģʽ*/
		key_task.chassis_mode = 1;
		/*�Զ���������ر�*/
		key_task.custom_mode = 0;

		/*����mode*/
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
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_Q && key_task.Cashing_yaw == 0) // һ����ҿ�
	{
		/*ȡ��ģʽ*/
		key_task.chassis_mode = 1;
		/*�Զ���������ر�*/
		key_task.custom_mode = 0;

		/*����mode*/
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
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_R && key_task.Gold == 0) // һ��ȡ���
	{
		/*ȡ��ģʽ*/
		key_task.chassis_mode = 1;
		/*�Զ���������ر�*/
		key_task.custom_mode = 0;

		/*����mode*/
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
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_E && key_task.Silver == 0) // ȡ��������ģʽ
	{
		/*ȡ��ģʽ*/
		key_task.chassis_mode = 1;
		/*�Զ���������ر�*/
		key_task.custom_mode = 0;

		/*����mode*/
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
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_C && key_task.Base == 0) // BASEģʽ
	{
		/*ȡ��ģʽ*/
		key_task.chassis_mode = 0;
		/*�Զ���������ر�*/
		key_task.custom_mode = 0;

		//		dm_motor_enable(&hcan1, &DM_yaw);
		////	HAL_Delay(500); //! pitch��ʹ����ʱ������
		//	    dm_motor_enable(&hcan1, &DM_pitch);

		/*����mode*/
		key_task.Base = 1;
		key_task.Safe = 0;
		key_task.Cashing = 0;
		key_task.Cashing_yaw = 0;
		// key_task.Gold_slanting_near = 0;
		key_task.Gold = 0;
		key_task.Silver = 0;

		key_task.deposit = 0;
		key_task.extraction = 0;

		// һ��ȡ����־λ
		Gold_flag = 0;

		// һ��ȡ�����־λ
		Silver_flag = 0;

		// һ������־λ
		deposit_flag = 0;

		// һ��ȡ��Ŀ��־λ
		extraction_flag = 0;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_B)
	{
		HAL_Delay(500);
		dm_motor_enable(&hcan1, &DM_yaw);
		HAL_Delay(500);
		dm_motor_enable(&hcan1, &DM_pitch);
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_B && key_task.custom_mode == 0) // ����+�Զ���С��ȡ��ģʽ
	{
		/*�����˶�+�Զ���ȡ��ģʽ*/
		key_task.chassis_mode = 0;

		/*���浱ǰ�Ƕ�ֵ��Ϊ��ʼֵ*/
		for (int i = 0; i < 4; i++)
		{
			// �����˲���״̬
			angle_filters[i].sum = 0;
			angle_filters[i].index = 0;
			angle_filters[i].last_valid = Custom_data[i].angle; // ���浱ǰֵ

			// �õ�ǰֵ��������˲�������
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

		/*�Զ������������*/
		key_task.custom_mode = 1;

		/*����mode*/
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
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_G || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_G + KEY_PRESSED_OFFSET_W || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_G + KEY_PRESSED_OFFSET_A || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_G + KEY_PRESSED_OFFSET_S || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_G + KEY_PRESSED_OFFSET_D) // ����+�Զ���ȡ��ģʽ
	{
		Spinning_Top_R = 1;
		Spinning_Top_L = 0;
		// /*���ڸ��Զ���������Ƿ�ʹ��X�����ƶ�ʹ��*/
		// Gantry_flag = 0;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_F || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_F + KEY_PRESSED_OFFSET_W || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_F + KEY_PRESSED_OFFSET_A || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_F + KEY_PRESSED_OFFSET_S || key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_F + KEY_PRESSED_OFFSET_D) // ����+�Զ���ȡ��ģʽ
	{
		Spinning_Top_R = 0;
		Spinning_Top_L = 1;
		// /*���ڸ��Զ���������Ƿ�ʹ��X�����ƶ�ʹ��*/
		// Gantry_flag = 1;
	}
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_Z && key_task.deposit == 0) // ����+���(��ʱ��Ҫ����ǰ��)
	{
		/*�����˶�+�Զ���ȡ��ģʽ*/
		key_task.chassis_mode = 0;
		/*�Զ������������*/
		key_task.custom_mode = 0;

		/*����mode*/
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
	else if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_X && key_task.deposit == 0) // ����+ȡ��(��ʱ��Ҫ����ǰ��)
	{
		/*�����˶�+�Զ���ȡ��ģʽ*/
		key_task.chassis_mode = 0;
		/*�Զ������������*/
		key_task.custom_mode = 0;

		/*����mode*/
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

	/*�ɿ�����Ϊ1��Ȼ���ٴΰ�����Ӧģʽ�Ż���ж�Ӧģʽ��λ*/ //! ����˼·���жϰ���̧����ٽ��в���
	if (!chassis_move_flag && !key_task.key_rc_ctrl->mouse.press_r && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_R && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_Q && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_Q && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_Z && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_X && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_E && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_C && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_B)
		Again_flag = 1;

	if (key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_G && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_SHIFT + KEY_PRESSED_OFFSET_F)
	{
		Spinning_Top_R = 0;
		Spinning_Top_L = 0;
	}
}
/*����ģʽ�л��Ͱ���΢��λ��*/
void button_gantry(void)
{
	if (!key_task.chassis_mode && !key_task.Base && !Gantry_flag && Lift_Down)
	{
		/*���ż�*/
		if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_E) //&& compute_flag
		{
			//! ���������ã���Ϊ��ѭ����v��ֵû������,����취��Ӧ����Again_flag��� // while (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_Q)
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
		/*���ż�*/
	}
}
void button_position(void) // ����λ��+���λ�ô������//************��Ϊ������
{
	/*�����ж���������compute_flag��ʵ�ְ�һ�ο���һ�Σ������ǳ������¿���*/
	// �����Ƿ���Ҫ�ڶ�Ӧ��ģʽ�²���ʹ��һ���򼸸�
	if (key_task.chassis_mode == 1 && !key_task.Base && Lift_Down) // ����Ҽ�//!��������λ�üӼ�
	{
		/*���ż�*/
		if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_Q) //&& compute_flag
		{
			//! ����while�����ã���Ϊ��ѭ����v��ֵû������,����취��Ӧ����Again_flag���
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
		/*���ż�*/

		/*��е��*/
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
		/*������һ������һ��*/ //! ����ʹ�óɹ���Ч������
		if (key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_Q && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_W && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_A && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_S && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_Z && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_X && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_E && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_R && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_D && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_F && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_C && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_V && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_G && key_task.key_rc_ctrl->key.v != KEY_PRESSED_OFFSET_B)
		{
			compute_flag = 1;
		}
	}
}
void Custom_position(void)
{
	/*���ж��Ƿ��ɵ����˶�ģʽ����*/
	if (chassis_move_flag)
	{
		// �������Ҹ�ֵ�൱�ڲ��Ա������и�ֵ����
		chassis_move_flag = 0;
		Again_flag = 0;
		// HAL_Delay(200);
		return;
	}

	if (!Again_flag)
	{
		return;
	}
	// /*��ʼ����δ����Shift+Gʱ������Shift+B�������ƹ̶����м䣬*/
	// if (!Gantry_flag)
	// {
	yaw = yaw_custom;					 // DM_yaw   //��̧��1973~-144                            //1050��Ӧ90��
	pic = pic_custom;					 // DM_pitch//-1420~2580//����-2425������-420             //1050��Ӧ90��
	Cup_target_pitch = Cup_pitch_custom; // 1220~-1000//ƽ��274//-763							 //991.5��Ӧ90��
	Cup_target_roll = Cup_roll_custom;
	// }
	// /*������Shift+G��Shift+B֮�������л������ƺ����Ƿ�̶�*/
	// else
	// {
	// 	yaw = yaw_custom;					 // DM_yaw   //��̧��1973~-144                            //1050��Ӧ90��
	// 	pic = pic_custom;					 // DM_pitch//-1420~2580//����-2425������-420             //1050��Ӧ90��
	// 	Cup_target_pitch = Cup_pitch_custom; // 1220~-1000//ƽ��274//-763							 //991.5��Ӧ90��
	// 	Cup_target_roll = Cup_roll_custom;
	// 	/*���ż�*/
	// 	X_target = X_custom;
	// 	Y_target = Y_custom;
	// 	Z_target = Z_custom; /// 4096.0  * 8192.0//		Uplift_location(Z_target);
	// }
}
/*����ȡ��ģʽ��Ӧ��ʼλ��*/
void Base_position(void) // ����������  //!���������жϺ���Ҫ���һ�μ����ϰ����Ż���ȫ��λ //���Լ�һ�¹�λ�󽫱������ܽǶ�Ҳ��Ϊԭ״��ֱ��Ȧ�����㣬custom���㵱ǰ�ܽǶȣ�
{
	// Cup_Fart();
	/*��е������*/
	Cup_target_pitch = 0.0f;
	Cup_target_roll = 0.0f;
	/*��е��pitch*/
	if (!direction)
	{
		pic = Pitch_obtain;
	}
	else
	{
		pic = -Pitch_obtain;
	}
	/*��е��Yaw*/
	yaw = 0.0f;
	/*Uplift*/
	Z_target = 0.0f;
	// /*X*/
	X_target = 0.0f;
	/*Y*/
	Y_target = 0.0f;
}
void Safe_position(void) // һ��ȡ���ʼ����������
{
	/*���ж��Ƿ��ɵ����˶�ģʽ����*/
	if (chassis_move_flag)
	{
		// �������Ҹ�ֵ�൱�ڲ��Ա������и�ֵ����
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
	/*��е������*/
	Cup_target_pitch = Cup_pitch_obtain;
	Cup_target_roll = Cup_roll_obtain;
	/*��е��Yaw*/
	yaw = Yaw_obtain;
	/*��е��pitch*/
	if (!direction)
	{
		pic = Pitch_obtain;
	}

	else
	{
		pic = -Pitch_obtain;
	}
}
void Cashing_position(void) // һ���ҿ�
{
	/*���ж��Ƿ��ɵ����˶�ģʽ����*/
	if (chassis_move_flag)
	{
		// �������Ҹ�ֵ�൱�ڲ��Ա������и�ֵ����
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
		/*��е������*/
		Cup_target_pitch = Cup_pitch_Cash_0;
		Cup_target_roll = Cup_roll_Cash_0;
		/*��е��Yaw*/
		yaw = Yaw_Cash_0;
		if (DM_yaw.para.pos > yaw_temp - 0.1 && DM_yaw.para.pos < yaw_temp + 0.1)
		{
			/*��е��pitch*/
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
		/*��е������*/
		Cup_target_pitch = Cup_pitch_Cash_1;
		Cup_target_roll = Cup_roll_Cash_1;
		/*��е��Yaw*/
		yaw = Yaw_Cash_1;
		if (DM_yaw.para.pos > yaw_temp - 0.1 && DM_yaw.para.pos < yaw_temp + 0.1)
		{
			/*��е��pitch*/
			pic = Pitch_Cash_1;
		}
	}
	Cup_Suck();
}

void Cashing_yaw_position(void)
{
	/*���ж��Ƿ��ɵ����˶�ģʽ����*/
	if (chassis_move_flag)
	{
		// �������Ҹ�ֵ�൱�ڲ��Ա������и�ֵ����
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
		/*��е������*/
		Cup_target_pitch = Cup_pitch_Cash_yaw_0;
		Cup_target_roll = Cup_roll_Cash_yaw_0;
		/*��е��Yaw*/
		yaw = Yaw_Cash_yaw_0;
		if (DM_yaw.para.pos > yaw_temp - 0.1 && DM_yaw.para.pos < yaw_temp + 0.1)
		{
			/*��е��pitch*/
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
		/*��е������*/
		Cup_target_pitch = Cup_pitch_Cash_yaw_1;
		Cup_target_roll = Cup_roll_Cash_yaw_1;
		/*��е��Yaw*/
		yaw = Yaw_Cash_yaw_1;
		if (DM_yaw.para.pos > yaw_temp - 0.1 && DM_yaw.para.pos < yaw_temp + 0.1)
		{
			/*��е��pitch*/
			pic = Pitch_Cash_yaw_1;
		}
	}
	Cup_Suck();
}

void Gold_position(void) // һ��ȡ����ʼ��
{
	/*���ж��Ƿ��ɵ����˶�ģʽ����*/
	if (chassis_move_flag)
	{
		// �������Ҹ�ֵ�൱�ڲ��Ա������и�ֵ����
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
		/*��е������*/
		Cup_target_pitch = Cup_pitch_gold;
		Cup_target_roll = Cup_roll_gold;
		/*��е��Yaw*/
		yaw = Yaw_gold;
		/*��е��pitch*/
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
void Silver_position(void) // һ��ȡ���������ʼ��
{
	/*���ж��Ƿ��ɵ����˶�ģʽ����*/
	if (chassis_move_flag)
	{
		// �������Ҹ�ֵ�൱�ڲ��Ա������и�ֵ����
		chassis_move_flag = 0;
		Again_flag = 0;
		return;
	}

	if (!Again_flag)
	{
		return;
	}

	yaw = Yaw_silver;
	// �߲�һ��
	// ��һ����̧����
	if (!Silver_flag)
	{
		/*Uplift*/
		Z_target = Uplift_silver_1;
		pic = Pitch_silver_1;
		// ����
		Y_target = 10.0f;
		Cup_target_pitch = Cup_pitch_silver_1;
		Cup_Suck();
		Silver_flag = 1;
	}
	// �ڶ�����ǰ��+���̴�ֱ+������
	// else if ((Uplift_moto[0].total_angle > -(float)8192 * 19.0 / 18.0 * Z_target - 2048.0f && Uplift_moto[0].total_angle < -(float)8192 * 19.0 / 18.0 * Z_target + 2048.0f) && Silver_flag == 1)
	// {
	// 	Y_target = Y_silver_1;
	// 	Cup_target_pitch = Cup_pitch_silver_1;
	// 	Cup_Suck();
	// 	Silver_flag = 2;
	// }
	// ��������̧����ѹ
	else if ((Y_moto.total_angle > -(float)8192 * 36.0 / 14.0 * Y_target - 2048.0f && Y_moto.total_angle < -(float)8192 * 36.0 / 14.0 * Y_target + 2048.0f) && Silver_flag == 1)
	{
		/*Uplift*/
		Z_target = Uplift_silver_2;
		Silver_flag = 2;
	}

	// // ���Ĳ������Ͽ�֮��̧������
	else if ((Uplift_moto[0].total_angle > -(float)8192 * 19.0 / 18.0 * Z_target - 2048.0f && Uplift_moto[0].total_angle < -(float)8192 * 19.0 / 18.0 * Z_target + 2048.0f) && Silver_flag == 2)
	{
		/*Uplift*/
		Z_target = 15.0f; // Uplift_silver_3;
		Silver_flag = 3;
	}
	// // �貹�䣺ǰ��ſ󣻻���С������̧����ǰ����;̧��
	// //! Ϊ�˿�ʯ�������Ӧ����ʯ�һ���
	// // ����һ��ǰ��ſ�
	else if ((Uplift_moto[0].total_angle > -(float)8192 * 19.0 / 18.0 * Z_target - 2048.0f && Uplift_moto[0].total_angle < -(float)8192 * 19.0 / 18.0 * Z_target + 2048.0f) && Silver_flag == 3)
	{
		Y_target = 40.0f; // Y_silver_1 + 10.0f;
		Cup_target_pitch = 0.0f;
		Silver_flag = 4;
	}
	// // �����������С������̧��
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
	// // ��������ǰ����
	// else if ((Uplift_moto[0].total_angle > -(float)8192 * 19.0 / 18.0 * Z_target - 2048.0f && Uplift_moto[0].total_angle < -(float)8192 * 19.0 / 18.0 * Z_target + 2048.0f) && Silver_flag == 7)
	// {
	// 	Y_target = 39.0f; // Y_silver_1 + 5.0f;
	// 	Cup_Suck();
	// 	Silver_flag = 8;
	// }
	// // �����ģ�̧��
	else if ((Y_moto.total_angle > -(float)8192 * 36.0 / 14.0 * Y_target - 2048.0f && Y_moto.total_angle < -(float)8192 * 36.0 / 14.0 * Y_target + 2048.0f) && Silver_flag == 6)
	{
		Z_target = 3.0f;
		Silver_flag = 7;
	}
	// // ���岽��С�۳���
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
	// // ������������
	// else if ((DM_pitch.para.pos > PI / 180.0f * pic - 0.1 && DM_pitch.para.pos < PI / 180.0f * pic + 0.1) && Silver_flag == 10)
	// {
	// 	Y_target = Y_silver_3;
	// 	Z_target = Uplift_silver_4;
	// 	Cup_target_pitch = Cup_pitch_silver_2;
	// 	Silver_flag = 11;
	// }
}
/*����ģʽ�л��Ͱ���΢��*/
/*һ�����*/
void Deposit(void) // TODO������Ҫ�����������ΰ����ſɵ���
{
	/*//����������´����Ҵ�󣬸��ģ�1.���ǰ�����ж��Ƿ�Ϊ�Ҵ��
	2.��ʼ��󣨲�����������С��yaw���㣩
	3.���������л���������*/

	// ���£����ǰ�жϣ�
	if (!direction && deposit_flag != 6)
	{
		direction = 1;
		return;
	}

	// ��һ����ǰ��+����
	if (!deposit_flag)
	{
		/*Z*/
		Z_target = 7.0f; //! ��еλ�õ�������֮��ߣ�ͬʱ�����˹��͵��µĿ�ʯ��������������
		/*Y*/
		Y_target = Y_deposit_1;
		Cup_target_pitch = -62.5f;
		deposit_flag = 1;
	}
	// �ڶ�����С��pitch��ת90�ȣ���0��1��
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
	// ��������С��yaw��ת60�ȣ���0��1�����ÿ�
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
	// ���̱ù�
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
/*һ��ȡ��*/
void Extraction(void)
{
	// ���£����ǰ�жϣ�
	if (!direction && extraction_flag != 6)
	{
		direction = 1; // ��ȡ��
		return;
	}

	/*Z*/
	Z_target = Uplift_deposit; // TODO:��������߶�һ��

	// ��һ����ǰ��+����
	if (!extraction_flag)
	{
		/*Y*/
		Y_target = Y_deposit_1;
		Cup_target_pitch = -62.5f;
		extraction_flag = 1;
	}
	// �ڶ�����С��pitch��ת90�ȣ���0��1��
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
	// ��������С��yaw��ת60�ȣ���0��1��ȡ��ÿ�
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
	// ���̱ù�
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
void motor_speed(void) // �л���ģʽ�����Ŀ��ֵ
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
Ŀ�꣺
ǰ��2006���������ڶ�ת����ֵλ�ü�Ϊ��ʼλ�ã��ಽ����
С�����̣���ת�����ܽǶȵ�һ�θ�λ�����ȶ�λ�ú��ڽ��еڶ����ܽǶȸ�λ
*/
void resetValue(void)
{
	if (!reposition_flag_Y)
	{
		// TODOע��̧������Ƿ�̧����Ŀ��λ�ã��ﵽ���ٽ���ǰ�츴λ
		if (Y_moto.real_current < 5000) // 10000����ֵҲ��Ҫʵ�ʲ���
		{
			Y_target -= 0.01f; // limit�����ñ�־λ�����жϼ��ϻ�ȥ��
		}
		// �ܽǶ�����
		else if (Y_moto.real_current >= 5000)
		{
			Y_moto.round_cnt = 0;
			Y_target = 0.5f;
			// ǰ���ʱΪ��ʼλ��
			reposition_flag_Y = 1;
		}
	}
	// if (!reposition_flag_Cup)
	// {
	// 	if (Cup_moto[0].real_current < 3200) // 10000����ֵҲ��Ҫʵ�ʲ���
	// 	{
	// 		Cup_target_pitch -= 0.05f; // limit�����ñ�־λ�����жϼ��ϻ�ȥ��
	// 	}
	// 	// �ܽǶ�����
	// 	else if (Cup_moto[0].real_current >= 3200)
	// 	{
	// 		Cup_moto[0].round_cnt = 0;
	// 		Cup_moto[1].round_cnt = 0;
	// 		Cup_target_pitch = 60.0f;
	// 		// ǰ���ʱΪ��ʼλ��
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
	// ����������С�����̸�λ�����ж�
	// if (0)
	// {
	// }
}

// ��������������Ҫ�����򸴻������������λ�ò���
// Ŀ�꣺��λ��ʱ��Ҫ��̧�����Ҳ���и�λ(�����жϴ�ʱ�ܽǶȱ���ֵ�Ƿ������)���磺ֱ��cmd�������ͺ�������
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
		osDelay(800); // ��ֹ����

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
		/*�Զ��ж�����*/
		/*//TODO����������ʱ��ֱ�Ӷϵ��ˣ����������ϵ����и�λ
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

		// С��������������
		if (rc_ctrl.key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_F || rc_ctrl.rc.ch[4] < -20) //
			Cup_Suck();
		else if (rc_ctrl.key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_G || rc_ctrl.rc.ch[4] > 20) //
			Cup_Fart();

		// ���������������
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
