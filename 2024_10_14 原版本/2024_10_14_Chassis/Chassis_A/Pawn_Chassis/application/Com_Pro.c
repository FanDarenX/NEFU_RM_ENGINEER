#include "Com_Pro.h"
#include "remote_control.h"
#include "classis_task.h"
#include "gimbal_task.h"
#include "main.h"
/********************************通讯协议相关声明**************************************/
//帧头  || 1高8     1低8 ||  2高8   2低8  || 3高8   3低8 ||4高8          4低8 || 校验位   帧尾
//0xa7   运动模式  路径误差    期望速度1        期望速度2         特殊行动               0x24
//         0停车     0                                   鹰眼舵机90°
//         1直行    巡线       直速度           横速度
//         2横行    巡线
//         3搬运   寻标志                                夹具舵机收紧 收货一套动作
//
uint8_t sbus_rx_buffer[18]; 		//声明遥控器缓存数组
uint8_t u_buf[256],rx_buf[11];//串口通信缓冲
uint16_t rem_temp[4]= {0,0,0,0};

void Com_Pro()
{
//		if(rx_buf[1]==0x00)
//		{
//			Classis_TarSpd.speed_y_target=Classis_TarSpd.speed_y_target=
//		  Classis_TarSpd.speed_y_target=Classis_TarSpd.speed_y_target=0;
//			Classis_TarSpd.speed_x_target=Classis_TarSpd.speed_x_target=
//		  Classis_TarSpd.speed_x_target=Classis_TarSpd.speed_x_target=0;
//			Classis_TarSpd.speed_r_target=Classis_TarSpd.speed_r_target=
//		  Classis_TarSpd.speed_r_target=Classis_TarSpd.speed_r_target=0;
//		}
//		else if(rx_buf[1]==0x01)
//		{
//			Classis_TarSpd.speed_y_target=Classis_TarSpd.speed_y_target=
//		  Classis_TarSpd.speed_y_target=Classis_TarSpd.speed_y_target=(uint16_t)(rx_buf[3] << 8 | rx_buf[4]);
//			Classis_TarSpd.speed_x_target=Classis_TarSpd.speed_x_target=
//		  Classis_TarSpd.speed_x_target=Classis_TarSpd.speed_x_target=(uint16_t)(rx_buf[5] << 8 | rx_buf[6]);
//		}
//		if(rx_buf[7]==0x01)
//		{
//			tarn_flag=1;
//			lift_flag=1;
//		}
}

//void Rem_int()
//{
//	int i;
//	uint16_t rem_temp0[4];
//
//	rem_temp0[0]=rc_ctrl.rc.ch[0];//右前
//	rem_temp0[1]=rc_ctrl.rc.ch[1];//右前
//	rem_temp0[2]=rc_ctrl.rc.ch[2];//右前
//	rem_temp0[3]=rc_ctrl.rc.ch[3];//右前

//	rem_temp[0]=rem_temp0[0];
//	rem_temp[1]=rem_temp0[1];
//	rem_temp[2]=rem_temp0[2];
//	rem_temp[3]=rem_temp0[3];
//}

//void Rem_Classis_Control()
//{
//	if(rc_ctrl.rc.s[0]==1)
//	{
//		if(ABS(rc_ctrl.rc.ch[0])>=10)
//		Classis_TarSpd.speed_r_target=5*rc_ctrl.rc.ch[0];
//		else if(ABS(rc_ctrl.rc.ch[0])<10)
//		Classis_TarSpd.speed_r_target=0;
//
//		if(ABS(rc_ctrl.rc.ch[2])>=10)
//		Classis_TarSpd.speed_y_target=5*rc_ctrl.rc.ch[2];
//		else if(ABS(rc_ctrl.rc.ch[2])<10)
//		Classis_TarSpd.speed_y_target=0;

//		if(ABS(rc_ctrl.rc.ch[3])>=10)
//		Classis_TarSpd.speed_x_target=-5*rc_ctrl.rc.ch[3];
//		else if(ABS(rc_ctrl.rc.ch[3])<10)
//		Classis_TarSpd.speed_x_target=0;
//	}
//	else
//	{
//	Classis_TarSpd.speed_x_target=Classis_TarSpd.speed_y_target=Classis_TarSpd.speed_r_target=0;
//	}
//}
void Rem_Gimbal_Control()
{
    //if(rc_ctrl.rc.s[0]==2)
//	{
//		if(ABS(rc_ctrl.rc.ch[0])>=10)
//		Gimbal_Speed.tran=3*rc_ctrl.rc.ch[0];
//		else if(ABS(rc_ctrl.rc.ch[0])<10)
//		Gimbal_Speed.tran=0;
//

//		if(ABS(rc_ctrl.rc.ch[3])>=10)
//		{
//			Gimbal_Speed.lift_l=-5*rc_ctrl.rc.ch[3];
//			Gimbal_Speed.lift_r=-5*rc_ctrl.rc.ch[3];
//		}
//		else if(ABS(rc_ctrl.rc.ch[3])<10)
//		{
//			Gimbal_Speed.lift_l=0;
//			Gimbal_Speed.lift_r=0;
//		}
//	}
//	else
//	{
//		Gimbal_Speed.lift_l=Gimbal_Speed.lift_r=Gimbal_Speed.tran=0;
//	}
//
}