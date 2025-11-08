#ifndef A1_motor_H
#define A1_motor_H

#include "motor_msg.h"  // 电机通信协议
#include <stdint.h>
#include <stdbool.h>

typedef struct{
	// 定义 发送格式化数据
    A1_MasterComdDataV3  motor_send_data;  //电机控制数据结构体，详见motor_msg.h
	  int hex_len;                    //发送的16进制命令数组长度, 34
    // long long send_time;            //发送该命令的时间, 微秒(us)
    // 待发送的各项数据
    unsigned short id;              //电机ID，0xBB代表全部电机
    unsigned short mode;            //0:空闲, 5:开环转动, 10:闭环FOC控制
    //实际给FOC的指令力矩为： 
    //K_P*delta_Pos + K_W*delta_W + T
    float T;                        //期望关节的输出力矩（电机本身的力矩）（Nm）
    float W;                        //期望关节速度（电机本身的速度）(rad/s)
    float Pos;                      //期望关节位置（rad）
    float K_P;                      //关节刚度系数
    float K_W;                      //关节速度系数
    A1_COMData32 Res;                  // 通讯 保留字节  用于实现别的一些通讯内容
}A1_MOTOR_send;

typedef struct {
    // 定义 接收数据
    A1_ServoComdDataV3 motor_recv_data;   //电机接收数据结构体，详见motor_msg.h
    int hex_len ;                   			//接收的16进制命令数组长度, 78
    // long long resv_time;         			//接收该命令的时间, 微秒(us)
    bool correct ;                  			//接收数据是否完整（true完整，false不完整）  
    //解读得出的电机数据
    unsigned char motor_id;         			//电机ID
    unsigned char mode;             			//0:空闲, 5:开环转动, 10:闭环FOC控制
    int Temp;                       			//温度
    unsigned char MError;           			//错误码

    float T;                        			// 当前实际电机输出力矩
    float W;                        			// 当前实际电机速度（高速）
    float LW;                       			// 当前实际电机速度（低速）
    int   Acc;                        		// 电机转子加速度
    float Pos;                     				// 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）

    float gyro[3];                  			// 电机驱动板6轴传感器数据
    float acc[3];
}A1_MOTOR_recv;

/* EXTERN */


#endif  // UNITREEMOTOR_H
