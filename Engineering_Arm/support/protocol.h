#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
    GAME_STATE_CMD_ID                 = 0x0001,//比赛状态数据，1Hz 周期发送
    GAME_RESULT_CMD_ID                = 0x0002,//比赛结果数据，比赛结束后发送
    GAME_ROBOT_HP_CMD_ID              = 0x0003,//比赛机器人血量数据，1Hz 周期发送
	  Dart_Launch_Status                = 0x0004,//飞镖发射状态，飞镖发射后发送
	  AI_Challenge_Bonus_penalty_state  = 0x0005,//人工智能挑战赛加成与惩罚状态，1Hz 周期发送
    FIELD_EVENTS_CMD_ID               = 0x0101,//场地事件数据，1Hz 周期发送
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,//场地补给站动作标识数据，动作改变后发送
    SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,//请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）
    REFEREE_WARNING_CMD_ID            = 0x0104,//裁判警告数据，警告发生后发送
	  Countdown_To_The_DART_Launcher    = 0x0105,//飞镖发射口倒计时，1Hz 周期发送
    ROBOT_STATE_CMD_ID                = 0x0201,//机器人状态数据，10Hz 周期发送
    POWER_HEAT_DATA_CMD_ID            = 0x0202,//实时功率热量数据，50Hz 周期发送
    ROBOT_POS_CMD_ID                  = 0x0203,//机器人位置数据，10Hz 发送
    BUFF_MUSK_CMD_ID                  = 0x0204,//机器人增益数据，增益状态改变后发送
    AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,//空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送
    ROBOT_HURT_CMD_ID                 = 0x0206,//伤害状态数据，伤害发生后发送
    SHOOT_DATA_CMD_ID                 = 0x0207,//实时射击数据，子弹发射后发送
    BULLET_REMAINING_CMD_ID           = 0x0208,//子弹剩余发送数，空中机器人以及哨兵机器人发送，1Hz 周期发送
	  ROBOT_RFID_status                 = 0x0209,//机器人 RFID 状态，1Hz 周期发送
	  Instruction_Book_For_Dart         = 0x020A,//飞镖机器人客户端指令书，10Hz 周期发送
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,//机器人间交互数据，发送方触发发送，上限 10Hz 
		Custom_Controller_Interaction_Data= 0x0302,//自定义控制器交互数据接口，通过客户端触发发送，上限 30Hz
		Client_Side_Mini_Map              = 0x0303,//客户端小地图交互数据，触发发送
		Keyboard_And_Mouse_Information    = 0x0304,//键盘、鼠标信息，通过图传串口发送
		Mini_Map_Receive_Date             = 0x0305,//客户端小地图接收信息
    IDCustomData,
}referee_cmd_id_t;
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
