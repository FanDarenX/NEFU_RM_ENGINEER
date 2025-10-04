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

/*��е�����̴���PID������*/
pid_struct_t Cup_speed_pid[2] = {0};
pid_struct_t Cup_angle_pid[2] = {0}; // Ϊ����ͬ����ѡ��ʹ���ڲ����ǶȻ���������ٶȻ�
/*���żܴ���PID������*/
pid_struct_t Uplift_speed_pid[4] = {0};
pid_struct_t Uplift_angle_pid[4] = {0}; // Ϊ����ͬ����ѡ��ʹ������������ǶȻ����ĵ���ٶȻ�
pid_struct_t X_speed_pid = {0};
pid_struct_t X_angle_pid = {0};
pid_struct_t Y_speed_pid = {0};
pid_struct_t Y_angle_pid = {0};

// int a;

/*�ͽ�һͨ�˲�*/
first_order_filter_type_t roll_filter;
/*б��*/
static float roll_filter_k = 1.6666666667f;

/*��е������Ŀ��Ƕ�*/
// ����

/*��е��Ŀ��λ��*/
/*ң����chת��*/
/*��е��*/
float yaw = 0.0f; // DM_yaw
float pic = 0.0f; // DM_pitch
/*DM_delay*/
float Cup_target_pitch = 0.0f; // 2006
float Cup_target_roll = 0.0f;  // 2006
/*���ż�*/
float X_target = 0;
float Y_target = 0;
float Z_target = 0.0f;
/*���̿��Ƽ���Ŀ��λ��*/
float Cup_pitch_temp;
float Cup_roll_temp;
float yaw_temp;
float pic_temp;
float X_temp;
float Y_temp;
float Z_temp;

/*����*/ /*֮�������Ҫ�ҵ����ʵı���*/

float compensate_Y = 0.0f;
float compensate_Z = 5.0f;

float Multiple_Cup_pitch = 15.0f; // 36:1//6:5    /*7.50f��Ӧͨ��ֵ660->90��*/
float Multiple_Cup_roll = 7.50f;  // 7.50f��Ӧpitch��roll��90��
float Multiple_X = 36.0 / 18.0 * (36.0f - 34.0);
float Multiple_Y = 36.0 / 14.0 * (50.0f - 1.0f); // �˴�����λ�ü�ȥ��ȫλ��//�൱���ڰ�ȫλ���µ���ԽǶ���//50
float Multiple_Z = 19.0 / 18.0 * (28.0f);		 // 19:1

/*���żܲ���ֵ*/
// ����ȡͬ�ţ�����ȡ���
float cp1 = 100.0f;	  // total��-4096.0f
float cp2 = -1620.0f; // total��-4096.0f
float cp3 = 700.0f;	  // total��3400.0f
float cp4 = 400.0f;	  // total��-2900.0f

float compensate1 = 0.0f / 19.0 * 18.0f; // 4096.0 / 19.0 * 18.0f
float compensate2 = 0.0f / 19.0 * 18.0f; // 1024.0 / 19.0 * 18.0f	// 		  // Ҫ����
float compensate3 = 0.0f / 19.0 * 18.0f; //(512.0) / 19.0 * 18.0f ���������� // 3��2��ͣ�4096.0 / 19.0 * 18.0f
float compensate4 = 0.0f / 19.0 * 18.0f; // 4096.0 / 19.0 * 18.0f

/*��ȫ��1.��Uplift_safe_flag����Y_safe_flag*/
/*�Ƕȿ��Ʊ�־*/
int Uplift_safe_flag = 0; // ̧�����ż���ֱ��ͶӰ��ȫ����//?��ʱ����
int Y_safe_flag = 0;	  // ǰ�����ż�ˮƽ��ͶӰ��ȫ����

/*��ʼ��ȫλ�ñ���*/
float Uplift_safe_multiple = 0.0f; //?��ʱ����
float Y_safe_multiple = 36.0 / 14.0 * 34.0f;

/*����*/
int remoteControl = 1;
int keyboard = 1;

/*��е�۵������ָ��*/
// ��ʱû�ã����Կ�position�õ���Щ�����
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
/*��е�۵������ָ��*/

/*���żܵ������ָ��*/
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
/*���żܵ������ָ��*/

void position_init()
{
	/*���PID*/ //! ����Ч������������һ���������ɣ���~
	int Init_flag = 0;
	while (!Init_flag) // �����۵�
	{
		// 2006������Ƶ���, ��Χ [-10000,10000]
		// 3508������Ƶ���, ��Χ [-16384,16384]
		// 6020������Ƶ���, ��Χ [-30000,30000]
		/*CAN1*/
		/*��е������*/
		//  0x205
		Pid_Init(&Cup_speed_pid[0], // PID��Ҫϸ�������ص���
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
		/*���ż�*/
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

double msp(double x, double in_min, double in_max, double out_min, double out_max) // ӳ�亯��������������ֵ��0~8191��ת��Ϊ�����ƵĽǶ�ֵ��-pi~pi��
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//	a = (int)HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0);
//	return;
void position_task(void)
{
	/*ǰ�츴λ*/
	// ����Ӧ��д�������ϵ�������Ȼ�������û������
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

	if (remoteControl) // �����˶�ģʽ��//���������ң����Debug��������
	{
		if (Right_Down)
		{
			// float a = sin(3.14);
			/*��е������*/
			Cup_target_pitch = -ABS((float)Cup_pitch);
			Cup_target_roll = (float)Cup_roll;
			/*��е��YAW&&PITCH*/
			yaw = (float)Arm_yaw;
			yaw = (yaw);
			DM4310_YAW(yaw);
			pic = (float)Arm_pitch;
		}
		else if (Right_Middle)
		{
			/*���ż�X&&Y*/
			X_target = -((float)X_traverse);
			Y_target = -ABS((float)Y_traverse);
			/*���ż�Z*/
			Z_target = -ABS((float)Z_traverse);
		}
	}

	else if (keyboard || Custom_action) // ȫ�ֱ�������//!ʵ�ʲ����ҵ����⣬���ڳ��򲻶�ѭ�����㣬���²���������С���¹��㣻����취����һ����������洢��ʱֵ
	{									//! ò������������ֵ�������⣬�Ƚ��г����������併��������ʱ���������䣬��ʱ�ڽ��г�Ӧ�û��һ����ȷ
		// ��֪��Ϊɶ����ʶ����Custom_action����1
		//! �ɳ������ñ�־λ�����ΰ���Xֱ�Ӹ�Ϊ�м�ֵ���ٴΰ��¸����Զ�����������ٰ���ص�X�м�ֵ����
		/*��������*/
		// limit_key(&Cup_target_pitch, 230 + 100, -90 - 100); //! ����//���ڼ�����Զ��������������    //?������ͬʱ���л��ǵ������У�Ӧ���ǵ������У�//TODO����
		// limit_key(&Cup_target_roll, 360, -360);
		limit_key(&yaw, 90, -90); // TODO�°湤��180
		limit_key(&pic, 200, -200);
		// /*���ż�*/
		limit_key(&X_target, 10.0f, -10.0f);
		limit_key(&Y_target, 49.0f, 0.7f);
		limit_key(&Z_target, 29.0f, 1.0f);
		/* ��е��*/

		/*��е������*/
		Cup_pitch_temp = -(float)8192 * Multiple_Cup_pitch / 180 * Cup_target_pitch; // 0~180
		Cup_roll_temp = (float)8192 * Multiple_Cup_roll / 45 * Cup_target_roll;		 //-45~45
		/*��е��YAW&&PITCH*/
		yaw_temp = PI / 180.0f * yaw; // 0~270
		pic_temp = PI / 180.0f * pic; //-180~180
		/*��е��*/
		/*******************/

		/*���ż�*/
		/*���ż�X&&Y*/									// TODO����Ҫ�ֳɼ���
		X_temp = -(float)8192 * 36.0 / 19.0 * X_target; // X_target:1��Ӧ����1��//1��36��18��1.max:36��
		Y_temp = -(float)8192 * 36.0 / 14.0 * Y_target; // TODO- compensate_Y // Y_target:1��Ӧǰ��1��//1��36��14��1.max:52��
		/*���ż�Z*/
		Z_temp = -(float)8192 * 19.0 / 18.0 * Z_target; // Z_target:1��Ӧ����1��//1��19��18��1.max:28��
	}

	/*�������*/ // �ܹ������л�ģʽʱ�������ԭλ��  //�����������û��ѭ�����е�������������ᵼ�µ��������
				 // ͬʱʵ��keyscan���㣬position��chasiss����
				 // ����ѭ�������������ڣ�����ң������������̿��Ʒֿ�

	if (!Cup_X_reset)
	{
		Cup_location(Cup_pitch_td.xx1, Cup_roll_td.xx1);
	}
	DM_location(yaw_td.xx1, pic_td.xx1); // DM�����ʼȸʳ[����]�������лص���㣬//!����ͻ�ƿ�������
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

/*����ɲ���λ�û����ú���*/
void Cup_location(float Cup_target_pitch, float Cup_target_roll) // ����֮һ��ҪΪ��,�����Ǳ���ֵ8192����
{
	// if (!Y_is_safe())
	// {
	// 	Cup_target_pitch = 0.0f;
	// 	Cup_target_roll = 0.0f;
	// }

	/*Ϊ����ͬ����ȡ�ڲ�0x205�ǶȻ�����������ٶȻ�*/ // ʵ���Թ��󲻾�׼
	/*��е������Pitch&&Roll*/
	pid_calc(&Cup_angle_pid[0], Cup_target_roll + Cup_target_pitch, Cup_moto[0].total_angle); // 0x205��
	pid_calc(&Cup_speed_pid[0], Cup_angle_pid[0].output, Cup_moto[0].speed);
	// һ˳һ�����pitch��ͬ˳ͬ�����roll
	pid_calc(&Cup_angle_pid[1], Cup_target_roll - Cup_target_pitch, Cup_moto[1].total_angle);
	pid_calc(&Cup_speed_pid[1], Cup_angle_pid[1].output, Cup_moto[1].speed);

	Cup_CURRENT(Cup_speed_pid[0].output, Cup_speed_pid[1].output);
}

void DM_location(float yaw, float pic) // �����ǻ�����-PI~PI//�ڲ�Ӧ�����ж���yaw��pitch�ĽǶ�˳��
{
	// if (!Y_is_safe())
	// {
	// 	yaw = 0.0f;
	// 	pic = 0.0f;
	// }
	/*�����yaw��pitch*/
	mit_ctrl(&hcan1, &DM_yaw, DM_yaw.id, yaw, DM_yaw.ctrl.vel_set, DM_yaw.ctrl.kp_set, DM_yaw.ctrl.kd_set, DM_yaw.ctrl.tor_set);
	// if (DM_yaw.para.pos > yaw - 0.1 && DM_yaw.para.pos < yaw + 0.1)
	mit_ctrl(&hcan1, &DM_pitch, DM_pitch.id, pic, DM_pitch.ctrl.vel_set, DM_pitch.ctrl.kp_set, DM_pitch.ctrl.kd_set, DM_pitch.ctrl.tor_set);
}

void XY_location(float X_target, float Y_target) // �����Ǳ���ֵ8192����// �ڲ�Ӧ�����ж���x��y�ĽǶ�˳��,��Y_target���ں�����ֵ����Y_save�ģ�����X_target�Ƿ�Ϊ��
{
	/*ǰ�쵽��е��ˮƽ��ͶӰ��ȫ����*/
	if (Y_moto.total_angle <= Y_safe) // Y_moto.total_angle > Y_safe + Y_target - 10000 &&
		Y_safe_flag = 0;
	else
		Y_safe_flag = 1;

	// /*̧������е����ֱ��ͶӰ��ȫ����*/
	// if (!Uplift_is_safe() && !Y_is_safe())
	// {s
	// 	Y_target = XY_safe;
	// }

	// if (Uplift_is_safe())
	// {
	// 	Y_target = ABS(Y_target); // TODO�Ƿ���Ҫ�Ӹ���
	// }

	// if (!Uplift_is_safe() && Y_is_safe())
	// {
	// 	limit_key(&Y_target, Y_MAX, Y_safe + 3000);
	// }
	float compensate_Y = -4096.0 * 36.0 / 14.0;			  // �����
	pid_calc(&Y_angle_pid, Y_target, Y_moto.total_angle); // + compensate_Y
	pid_calc(&Y_speed_pid, Y_angle_pid.output, Y_moto.speed);
	// if (!Y_safe_flag)
	// {
	// pid_calc(&X_angle_pid, X_safe, X_moto.total_angle);
	// }
	// /*��ʱALL_safe_flagΪ��*/
	// else
	// {
	// }
	pid_calc(&X_angle_pid, X_target, X_moto.total_angle);
	pid_calc(&X_speed_pid, X_angle_pid.output, X_moto.speed);
	XY_CURRENT(Y_speed_pid.output, X_speed_pid.output);
}

void Uplift_location(float Z_target) // �����Ǳ���ֵ8192����
{
	//! ע������Ĳ����Կ��Ƶ�Ӱ��
	// 1==3;2==4;
	//-10593
	// 10557
	//-9613
	// 9557
	// 1,4����2��3��
	compensate1 = cp1 / 19.0 * 18.0f;
	compensate2 = cp2 / 19.0 * 18.0f;
	compensate3 = cp3 / 19.0 * 18.0f;
	compensate4 = cp4 / 19.0 * 18.0f;

	// if (Uplift_moto[0].real_current > 10000)
	// {
	// 	Uplift_moto[0].round_cnt -= 1;
	// }

	// pid_calc(&Uplift_angle_pid[0], Z_target + compensate1, Uplift_moto[0].angle); //! ע�ⷽ����˳����
	// pid_calc(&Uplift_angle_pid[1], -Z_target + compensate2, Uplift_moto[1].angle);
	// pid_calc(&Uplift_angle_pid[2], Z_target + compensate3, Uplift_moto[2].angle); // TODO[2]���� //ʵ��ʵ�飬���˲���Ƕȼ���ĸõ����������1.����2.��ת���ٶȻ�һֱת
	// pid_calc(&Uplift_angle_pid[3], -Z_target + compensate4, Uplift_moto[3].angle);

	//! 1,3�棻2��4˳��
	pid_calc(&Uplift_angle_pid[0], Z_target + compensate1, Uplift_moto[0].total_angle); //! ע�ⷽ����˳����
	pid_calc(&Uplift_angle_pid[1], -Z_target + compensate2, Uplift_moto[1].total_angle);
	pid_calc(&Uplift_angle_pid[2], Z_target + compensate3, Uplift_moto[2].total_angle); // TODO[2]���� //ʵ��ʵ�飬���˲���Ƕȼ���ĸõ����������1.����2.��ת���ٶȻ�һֱת
	pid_calc(&Uplift_angle_pid[3], -Z_target + compensate4, Uplift_moto[3].total_angle);

	pid_calc(&Uplift_speed_pid[0], Uplift_angle_pid[0].output, Uplift_moto[0].speed);
	pid_calc(&Uplift_speed_pid[1], Uplift_angle_pid[1].output, Uplift_moto[1].speed);
	pid_calc(&Uplift_speed_pid[2], Uplift_angle_pid[2].output, Uplift_moto[2].speed);
	pid_calc(&Uplift_speed_pid[3], Uplift_angle_pid[3].output, Uplift_moto[3].speed);

	Uplift_CURRENT(Uplift_speed_pid[0].output, Uplift_speed_pid[1].output, Uplift_speed_pid[2].output, Uplift_speed_pid[3].output);

	// if ( <= Uplift_safe) // TODO[2]����
	// 	Uplift_safe_flag = 0;
	// else
	// 	Uplift_safe_flag = 1;
}
/*����ɲ���λ�û����ú���*/

/**��ʱ����
 *
 *
 *
 *
 *
 */
/*���㹹��ǶȺ���*/ // Ȧ��+�Ƕ�
/*��Բ����̶�ֵ+���ֵ����*/
void Cup_fixed(float angle_pitch, float angle_roll) // �Ƕ�pitch:0~180\roll:-45~45��ͬʱ�ϵ��λǰ��ת�̷ŵ���ƽ��̼�巽������
{
	angle_pitch = -(float)8192 * Multiple_Cup_pitch / 180 * angle_pitch;
	angle_roll = (float)8192 * Multiple_Cup_roll / 45 * angle_roll;
	Cup_location(angle_pitch, angle_roll);
}

void DM_fixed(float yaw, float pic) // yaw�Ƕ�-PI~0,pitch�Ƕ�,��˳���  // pitch����  //TODO
{
	yaw = -PI / 180.0f * yaw;
	DM_location(yaw, 0);
}
void XY_fixed(float X_target, float Y_target) // �����Ǳ���ֵ8192����  // ����  //TODO
{
	X_target = (float)8192 * Multiple_X / 360 * X_target; // 360�ǰ����ҷָ�Ϊ360�ݣ�X_target:-360~360
	Y_target = (float)8192 * Multiple_Y / 360 * Y_target;
	XY_location(X_target, Y_target);
}
void Uplift_fixed(float Z_target) // �����Ǳ���ֵ8192����  // ����  //TODO
{
	Z_target = (float)8192 * Multiple_Z / 360 * Z_target;
	Uplift_location(Z_target);
}
/*���㹹��ǶȺ���*/