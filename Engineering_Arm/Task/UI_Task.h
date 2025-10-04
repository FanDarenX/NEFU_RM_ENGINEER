#ifndef __UI_TASK_H_
#define __UI_TASK_H_

#define JUDGE_FRAME_HEADER 0xa5

#include "struct_typedef.h"
#include "ui.h"
#include "position_task.h"


#define UI_USART6 ((USART_TypeDef *)USART6_BASE)
#define AIM_X 950
#define AIM_Y 540
#define ID_robot_interactive_header_data 0x0301

extern uint16_t Recieve_id;
extern uint8_t update_figure_flag;

void My_Car_State_UI(void);
void Heaven_Pick_Up(void);
void Heaven_Add_UI(void);

typedef struct
{
	uint8_t Sucker_Qi;
	uint8_t Ground_Qi;
	uint8_t Help_Qi;

} Qi_Ui_t;
extern Qi_Ui_t Qi_Ui;

enum judge_data_length_t
{
	/* Std */
	LEN_FRAME_HEAD = 5, // ֡ͷ����
	LEN_CMD_ID = 2,		// �����볤��
	LEN_FRAME_TAIL = 2, // ֡βCRC16
	/* Ext */
	// 0x000x
	LEN_GAME_STATUS = 11,
	LEN_GAME_RESULT = 1,
	LEN_GAME_ROBOT_HP = 28,
	LEN_DART_STATUS = 3,
	LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS = 11, // 0x0005
	// 0x010x
	LEN_EVENT_DATA = 4,
	LEN_SUPPLY_PROJECTILE_ACTION = 4,  // ����������������������������������
	LEN_SUPPLY_PROJECTILE_BOOKING = 3, // �Կ���δ����
	LEN_REFEREE_WARNING = 2,
	LEN_DART_REMAINING_TIME = 1, // 0x0105
	// 0x020x
	LEN_GAME_ROBOT_STATUS = 27, // 15!!!!!!!!!!!!!!!!!!!!!!!!!!!
	LEN_POWER_HEAT_DATA = 16,	// ��������������������
	LEN_GAME_ROBOT_POS = 16,
	LEN_BUFF_MASK = 1,
	LEN_AERIAL_ROBOT_ENERGY = 1, // ����������
	LEN_ROBOT_HURT = 1,
	LEN_SHOOT_DATA = 7,		  // ��������
	LEN_BULLET_REMAINING = 6, // ��������
	LEN_RFID_STATUS = 4,
	LEN_DART_CLIENT_DIRECTIVE = 12, // 0x020A
	// 0x030x
	// LEN_robot_interactive_header_data      = n,
	// LEN_controller_interactive_header_data = n,
	LEN_MAP_INTERACTIVE_HEADERDATA = 15,
	LEN_KEYBOARD_INFORMATION = 12, // 0x0304
}; // ��2-4

/* �Զ���֡ͷ */
typedef __packed struct
{
	uint8_t SOF;
	uint16_t DataLength;
	uint8_t Seq;
	uint8_t CRC8;

} FrameHeader;

typedef __packed struct
{
	uint8_t sof;
	uint16_t data_length;
	uint8_t seq;
	uint8_t crc8;
} std_frame_header_t; // LEN_FRAME_HEAD
/*******************************************************************************/
/*
	������ ID��
	1��Ӣ��(��)��
	2������(��)��
	3/4/5������(��)��
	6������(��)��
	7���ڱ�(��)��
	9���״�죩
	101��Ӣ��(��)��
	102������(��)��
	103/104/105������(��)��
	106������(��)��
	107���ڱ�(��)��
	109���״����

	�ͻ��� ID��
	0x0101 ΪӢ�۲����ֿͻ���(��) ��
	0x0102 Ϊ���̲����ֿͻ���( �� )��
	0x0103/0x0104/0x0105 Ϊ���������ֿͻ���(��)��
	0x0106 Ϊ���в����ֿͻ���((��)��

	0x0165��Ӣ�۲����ֿͻ���(��)��
	0x0166�����̲����ֿͻ���(��)��
	0x0167/0x0168/0x0169�����������ֿͻ���(��)��
	0x016A�����в����ֿͻ���(��)��
*/
enum judge_robot_ID
{
	hero_red = 1,
	engineer_red = 2,
	infantry3_red = 3,
	infantry4_red = 4,
	infantry5_red = 5,
	plane_red = 6,

	hero_blue = 101,
	engineer_blue = 102,
	infantry3_blue = 103,
	infantry4_blue = 104,
	infantry5_blue = 105,
	plane_blue = 106,
};
typedef __packed struct
{
	uint16_t teammate_hero;
	uint16_t teammate_engineer;
	uint16_t teammate_infantry3;
	uint16_t teammate_infantry4;
	uint16_t teammate_infantry5;
	uint16_t teammate_plane;
	uint16_t teammate_sentry;

	uint16_t client_hero;
	uint16_t client_engineer;
	uint16_t client_infantry3;
	uint16_t client_infantry4;
	uint16_t client_infantry5;
	uint16_t client_plane;
} ext_interact_id_t;

/*
	ѧ�������˼�ͨ�� cmd_id 0x0301������ data_ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����������кϼƴ������� 5000 Byte�� �����з���Ƶ�ʷֱ𲻳���30Hz��
 * +------+------+-------------+------------------------------------+
 * | byte | size |    breif    |            note                    |
 * |offset|      |             |                                    |
 * +------+------+-------------+------------------------------------+
 * |  0   |  2   | 	 data_ID   | 0x0200~0x02FF,��������Щ ID ��ѡȡ |
 * |      |      |             | ����ID�����ɲ������Զ���           |
 * +------|------|-------------|------------------------------------|
 * |  2   |  2   | 	sender_ID  | ��ҪУ�鷢���ߵ� ID ��ȷ��					|
 * +------|------|-------------|------------------------------------|
 * |  4   |  2   | receiver_ID | ��ҪУ������ߵ� ID ��ȷ��					|
 * |      |      |             | ���粻�ܷ��͵��жԻ����˵�ID				|
 * +------|------|-------------|------------------------------------|
 * |  6   |  n   | 		Data     | n ��ҪС�� 113 										|
 * +------+------+-------------+------------------------------------+
*/
/******************************�ͻ��˽�������**************************************/
#define INTERACT_DATA_LEN 113
typedef __packed struct // ���ݶ����ݸ�ʽ
{
	uint16_t data_cmd_id;
	uint16_t send_ID;
	uint16_t receiver_ID;
} ext_client_data_header_t;
enum
{
	// 0x200-0x02ff 	�����Զ������� ��ʽ  INTERACT_ID_XXXX
	INTERACT_ID_delete_graphic = 0x0100,	 /*�ͻ���ɾ��ͼ��*/
	INTERACT_ID_draw_one_graphic = 0x0101,	 /*�ͻ��˻���һ��ͼ��*/
	INTERACT_ID_draw_two_graphic = 0x0102,	 /*�ͻ��˻���2��ͼ��*/
	INTERACT_ID_draw_five_graphic = 0x0103,	 /*�ͻ��˻���5��ͼ��*/
	INTERACT_ID_draw_seven_graphic = 0x0104, /*�ͻ��˻���7��ͼ��*/
	INTERACT_ID_draw_char_graphic = 0x0110,	 /*�ͻ��˻����ַ�ͼ��*/
	INTERACT_ID_bigbome_num = 0x02ff
};
typedef __packed struct
{
	uint8_t data[INTERACT_DATA_LEN]; // ���ݶ�,n��ҪС��113
} robot_interactive_data_t;
// ��λ���ֽڣ�
enum
{
	LEN_INTERACT_delete_graphic = 8,	   // ɾ��ͼ�� 2(��������ID)+2(������ID)+2��������ID��+2���������ݣ�
	LEN_INTERACT_draw_one_graphic = 21,	   // ����2+2+2+15
	LEN_INTERACT_draw_two_graphic = 36,	   // 6+15*2
	LEN_INTERACT_draw_five_graphic = 81,   // 6+15*5
	LEN_INTERACT_draw_seven_graphic = 111, // 6+15*7
	LEN_INTERACT_draw_char_graphic = 51,   // 6+15+30���ַ������ݣ�
};
//****************************��ͼ�����ݶ�����****************************/
typedef __packed struct // ͼ��
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye : 3;
	uint32_t graphic_tpye : 3; // ֱ��  ����  ��Բ  ��Բ  Բ��  ����  ����  �ַ�
	uint32_t layer : 4;
	uint32_t color : 4;
	uint32_t start_angle : 9; // ��    ��    ��    ��    �Ƕ�  ��С  ��С  ��С
	uint32_t end_angle : 9;	  // ��    ��    ��    ��          λ��  ��    ����
	uint32_t width : 10;
	uint32_t start_x : 11; // ���  ���  Բ��  Բ��  Բ��  ���  ���  ���
	uint32_t start_y : 11; //
	uint32_t radius : 10;  // ��    ��    �뾶  ��    ��    ��    ��    ��
	uint32_t end_x : 11;   // �յ�  �Զ�  ��    ����  ����  ��    ��    ��
	uint32_t end_y : 11;   //                              ��    ��    ��
} graphic_data_struct_t;

typedef __packed struct // ������
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye : 3;
	uint32_t graphic_tpye : 3;
	uint32_t layer : 4;
	uint32_t color : 4;
	uint32_t start_angle : 9;
	uint32_t end_angle : 9;
	uint32_t width : 10;
	uint32_t start_x : 11;
	uint32_t start_y : 11;
	float number;
} Float_data_struct_t;

typedef __packed struct // ������
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye : 3;
	uint32_t graphic_tpye : 3;
	uint32_t layer : 4;
	uint32_t color : 4;
	uint32_t start_angle : 9;
	uint32_t end_angle : 9;
	uint32_t width : 10;
	uint32_t start_x : 11;
	uint32_t start_y : 11;
	int number;
} Int_data_struct_t;
/* data_ID: 0X0100  Byte:  2	    �ͻ���ɾ��ͼ��*/
typedef __packed struct
{
	uint8_t operate_type;
	uint8_t layer; // ͼ������0~9
} ext_client_custom_graphic_delete_t;
typedef enum
{
	NONE_delete = 0,
	GRAPHIC_delete = 1,
	ALL_delete = 2
} delete_Graphic_Operate; // ext_client_custom_graphic_delete_t��uint8_t operate_type
/*ͼ��ɾ������*/

// bit 0-2
typedef enum
{
	NONE = 0,	   /*�ղ���*/
	ADD = 1,	   /*����ͼ��*/
	MODIFY = 2,	   /*�޸�ͼ��*/
	DELETE = 3,	   /*ɾ��ͼ��*/
} Graphic_Operate; // graphic_data_struct_t��uint32_t operate_tpye

/*ͼ�����*/
// bit3-5
typedef enum
{
	LINE = 0,	   // ֱ��
	RECTANGLE = 1, // ����
	CIRCLE = 2,	   // ��Բ
	OVAL = 3,	   // ��Բ
	ARC = 4,	   // Բ��
	FLOAT = 5,	   // ������		*100
	INT = 6,	   // ������
	CHAR = 7	   // �ַ�
} Graphic_Type;
/*ͼ������*/
// bit 6-9ͼ���� ���Ϊ9����С0
// bit 10-13��ɫ
typedef enum
{
	RED_BLUE = 0, // ������ɫ
	YELLOW = 1,
	GREEN = 2,
	ORANGE = 3,
	FUCHSIA = 4, /*�Ϻ�ɫ*/
	PINK = 5,
	CYAN_BLUE = 6, /*��ɫ*/
	BLACK = 7,
	WHITE = 8
} Graphic_Color;
/*ͼ����ɫ����*/
// bit 14-31 �Ƕ� [0,360]
/**********************************�ͻ��˻�ͼ************************************************/
// ɾ��ͼ��
typedef __packed struct
{
	std_frame_header_t txFrameHeader;
	uint16_t CmdID;
	ext_client_data_header_t dataFrameHeader;
	ext_client_custom_graphic_delete_t clientData;
	uint16_t FrameTail;
} ext_deleteLayer_data_t;

// ���ַ���
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_string_t;

typedef __packed struct
{
	std_frame_header_t txFrameHeader;		  // ֡ͷ
	uint16_t CmdID;							  // ������
	ext_client_data_header_t dataFrameHeader; // ���ݶ�ͷ�ṹ
	ext_client_string_t clientData;			  // ���ݶ�
	uint16_t FrameTail;						  // ֡β
} ext_charstring_data_t;
// ������ͼ
typedef __packed struct
{
	std_frame_header_t txFrameHeader;		  // ֡ͷ
	uint16_t CmdID;							  // ������
	ext_client_data_header_t dataFrameHeader; // ���ݶ�ͷ�ṹ
	graphic_data_struct_t clientData;		  // ���ݶ�
	uint16_t FrameTail;						  // ֡β
} ext_graphic_one_data_t;

typedef __packed struct
{
	std_frame_header_t txFrameHeader;
	uint16_t CmdID;
	ext_client_data_header_t dataFrameHeader;
	graphic_data_struct_t clientData[2];
	uint16_t FrameTail;

} ext_graphic_two_data_t;

typedef __packed struct
{
	std_frame_header_t txFrameHeader;
	uint16_t CmdID;
	ext_client_data_header_t dataFrameHeader;
	graphic_data_struct_t clientData[5];
	uint16_t FrameTail;
} ext_graphic_five_data_t;

typedef __packed struct
{
	std_frame_header_t txFrameHeader;
	uint16_t CmdID;
	ext_client_data_header_t dataFrameHeader;
	graphic_data_struct_t clientData[7];
	uint16_t FrameTail;
} ext_graphic_seven_data_t;

// ���Ƹ�����
typedef __packed struct
{
	std_frame_header_t txFrameHeader;
	uint16_t CmdID;
	ext_client_data_header_t dataFrameHeader;
	Float_data_struct_t clientData[2];
	uint16_t FrameTail;
} ext_float_two_data_t;

typedef __packed struct
{
	std_frame_header_t txFrameHeader;
	uint16_t CmdID;
	ext_client_data_header_t dataFrameHeader;
	Float_data_struct_t clientData[7];
	uint16_t FrameTail;
} ext_float_seven_data_t;
// ��������
typedef __packed struct
{
	std_frame_header_t txFrameHeader;
	uint16_t CmdID;
	ext_client_data_header_t dataFrameHeader;
	Int_data_struct_t clientData[2];
	uint16_t FrameTail;
} ext_int_two_data_t;
typedef __packed struct
{
	std_frame_header_t txFrameHeader;
	uint16_t CmdID;
	ext_client_data_header_t dataFrameHeader;
	Int_data_struct_t clientData[7];
	uint16_t FrameTail;
} ext_int_seven_data_t;

typedef __packed struct Client_Slave_Flag
{
	uint8_t global_fiction;
	uint8_t global_clip;
	uint8_t global_spin;
	uint8_t global_auto_aim;
	uint8_t global_twist;
	uint8_t global_anti_top;
	uint8_t shift_rush;
	uint8_t user1;
} Client_Slave_Flag;
extern void UI_task(void const *pvParameters);
#define Power_supply HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_SET) // ����

#define CHE_JIA_ZI_X 1200
#define CHE_JIA_ZI_Y 100
#define LEFT_EXCHANGE_LINE_X 750
#define LEFT_EXCHANGE_LINE_Y 100

struct Level_1
{

	uint16_t Left;
	uint16_t Right;

	uint16_t Left_Lift;
	uint16_t Right_Lift;
};

struct Level_2
{

	uint16_t Height;
};

typedef struct
{

	struct Level_1 Level_1;
	struct Level_2 Level_2;

} UI_Draw;

extern void UI_init(void);

#endif
