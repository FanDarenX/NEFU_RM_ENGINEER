#ifndef __CRC8_CRC16_H__
#define __CRC8_CRC16_H__

#include "main.h"

/*
	命令码 ID 说明
	命令码 功能介绍
	0×0001 比赛进程信息
	0×0002 实时血量变化数据
	0×0003 实时射击数据
	0×0005 学生上传自定义数据
*/

//位置状态结构体
typedef __packed struct
{
	uint8_t flag;	//0无效	1有效
	uint32_t x;
	uint32_t y;
	uint32_t z;
	uint32_t compass;
	
}tLocData;//说明：定位数据单位为米，小数点后两位为有效数据

//比赛进程信息结构体
typedef __packed struct
{
	uint32_t remainTime;		//比赛剩余时间（从倒计时三分钟开始计算，单位 s）
	uint16_t remainLifeValue;	//机器人剩余血量
	float realChassisOutV;		//实时底盘输出电压（单位 V）
	float realChassisOutA;		//实时底盘输出电流（单位 A）
	tLocData locData;			//LOC 状态，见 tLocData 结构体定义
	float remainPower;			//remainPower: 剩余能量（满值 60J）
	
}tGameInfo;

//实时血量变化信息结构体
typedef __packed struct
{
	uint8_t weakId:4;
	uint8_t way:4;
	uint16_t value;
	
}tRealBloodChangedData;

//实时射击信息结构体
typedef __packed struct
{
	float realBulletShootSpeed;		//子弹实时射速（m/s）
	float realBulletShootFreq;		//子弹实时射频（发 /s）
	float realGolfShootSpeed;		//高尔夫实时射速 (m/s 英雄机器人 )
	float realGolfShootFreq;		//高尔夫实时射频 ( 发 /s?英雄机器人 )
	
}tRealShootData;

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

#endif
