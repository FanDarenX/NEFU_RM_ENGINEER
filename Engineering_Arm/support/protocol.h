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
    GAME_STATE_CMD_ID                 = 0x0001,//����״̬���ݣ�1Hz ���ڷ���
    GAME_RESULT_CMD_ID                = 0x0002,//����������ݣ�������������
    GAME_ROBOT_HP_CMD_ID              = 0x0003,//����������Ѫ�����ݣ�1Hz ���ڷ���
	  Dart_Launch_Status                = 0x0004,//���ڷ���״̬�����ڷ������
	  AI_Challenge_Bonus_penalty_state  = 0x0005,//�˹�������ս���ӳ���ͷ�״̬��1Hz ���ڷ���
    FIELD_EVENTS_CMD_ID               = 0x0101,//�����¼����ݣ�1Hz ���ڷ���
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,//���ز���վ������ʶ���ݣ������ı����
    SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,//���󲹸�վ�������ݣ��ɲ����ӷ��ͣ����� 10Hz����RM �Կ�����δ���ţ�
    REFEREE_WARNING_CMD_ID            = 0x0104,//���о������ݣ����淢������
	  Countdown_To_The_DART_Launcher    = 0x0105,//���ڷ���ڵ���ʱ��1Hz ���ڷ���
    ROBOT_STATE_CMD_ID                = 0x0201,//������״̬���ݣ�10Hz ���ڷ���
    POWER_HEAT_DATA_CMD_ID            = 0x0202,//ʵʱ�����������ݣ�50Hz ���ڷ���
    ROBOT_POS_CMD_ID                  = 0x0203,//������λ�����ݣ�10Hz ����
    BUFF_MUSK_CMD_ID                  = 0x0204,//�������������ݣ�����״̬�ı����
    AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,//���л���������״̬���ݣ�10Hz ���ڷ��ͣ�ֻ�п��л��������ط���
    ROBOT_HURT_CMD_ID                 = 0x0206,//�˺�״̬���ݣ��˺���������
    SHOOT_DATA_CMD_ID                 = 0x0207,//ʵʱ������ݣ��ӵ��������
    BULLET_REMAINING_CMD_ID           = 0x0208,//�ӵ�ʣ�෢���������л������Լ��ڱ������˷��ͣ�1Hz ���ڷ���
	  ROBOT_RFID_status                 = 0x0209,//������ RFID ״̬��1Hz ���ڷ���
	  Instruction_Book_For_Dart         = 0x020A,//���ڻ����˿ͻ���ָ���飬10Hz ���ڷ���
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,//�����˼佻�����ݣ����ͷ��������ͣ����� 10Hz 
		Custom_Controller_Interaction_Data= 0x0302,//�Զ���������������ݽӿڣ�ͨ���ͻ��˴������ͣ����� 30Hz
		Client_Side_Mini_Map              = 0x0303,//�ͻ���С��ͼ�������ݣ���������
		Keyboard_And_Mouse_Information    = 0x0304,//���̡������Ϣ��ͨ��ͼ�����ڷ���
		Mini_Map_Receive_Date             = 0x0305,//�ͻ���С��ͼ������Ϣ
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
