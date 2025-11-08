#define JUDGE_FRAME_HEADER 0xa5
#include "struct_typedef.h"
#define UI_USART6              ((USART_TypeDef *) USART6_BASE)
#define AIM_X 950
#define AIM_Y 540
#define ID_robot_interactive_header_data 0x0301
enum judge_data_length_t {
    /* Std */
    LEN_FRAME_HEAD 	                 = 5,	// 帧头长度
    LEN_CMD_ID 		                   = 2,	// 命令码长度
    LEN_FRAME_TAIL 	                 = 2,	// 帧尾CRC16
    /* Ext */
    // 0x000x
    LEN_GAME_STATUS 				         = 11,
    LEN_GAME_RESULT 				         = 1,
    LEN_GAME_ROBOT_HP 			         = 28,
    LEN_DART_STATUS					         = 3,
    LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS = 11,//0x0005
    // 0x010x
    LEN_EVENT_DATA					         = 4,
    LEN_SUPPLY_PROJECTILE_ACTION	   = 4,//！！！！！！！！！！！！！！！！！
    LEN_SUPPLY_PROJECTILE_BOOKING	   = 3,//对抗赛未开启
    LEN_REFEREE_WARNING				       = 2,
    LEN_DART_REMAINING_TIME		     	 = 1,//0x0105
    // 0x020x
    LEN_GAME_ROBOT_STATUS			       = 27,//15!!!!!!!!!!!!!!!!!!!!!!!!!!!
    LEN_POWER_HEAT_DATA 			       = 16,//！！！！！！！！！！
    LEN_GAME_ROBOT_POS				       = 16,
    LEN_BUFF_MASK		 				         = 1,
    LEN_AERIAL_ROBOT_ENERGY 	     	 = 1,//！！！！！
    LEN_ROBOT_HURT				         	 = 1,
    LEN_SHOOT_DATA					         = 7,//！！！！
    LEN_BULLET_REMAINING	 		       = 6,//！！！！
    LEN_RFID_STATUS					         = 4,
    LEN_DART_CLIENT_DIRECTIVE        = 12,//0x020A
    // 0x030x
    //LEN_robot_interactive_header_data      = n,
    //LEN_controller_interactive_header_data = n,
    LEN_MAP_INTERACTIVE_HEADERDATA           = 15,
    LEN_KEYBOARD_INFORMATION                 = 12,//0x0304
};//表2-4
/* 自定义帧头 */
typedef __packed struct
{
    uint8_t  SOF;
    uint16_t DataLength;
    uint8_t  Seq;
    uint8_t  CRC8;

} FrameHeader;
typedef __packed struct
{
    uint8_t  sof;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  crc8;
} std_frame_header_t;//LEN_FRAME_HEAD
/*******************************************************************************/
/*
	机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	9，雷达（红）
	101，英雄(蓝)；
	102，工程(蓝)；
	103/104/105，步兵(蓝)；
	106，空中(蓝)；
	107，哨兵(蓝)；
	109，雷达（蓝）

	客户端 ID：
	0x0101 为英雄操作手客户端(红) ；
	0x0102 为工程操作手客户端( 红 )；
	0x0103/0x0104/0x0105 为步兵操作手客户端(红)；
	0x0106 为空中操作手客户端((红)；

	0x0165，英雄操作手客户端(蓝)；
	0x0166，工程操作手客户端(蓝)；
	0x0167/0x0168/0x0169，步兵操作手客户端(蓝)；
	0x016A，空中操作手客户端(蓝)。
*/
enum judge_robot_ID {
    hero_red       = 1,
    engineer_red   = 2,
    infantry3_red  = 3,
    infantry4_red  = 4,
    infantry5_red  = 5,
    plane_red      = 6,

    hero_blue      = 101,
    engineer_blue  = 102,
    infantry3_blue = 103,
    infantry4_blue = 104,
    infantry5_blue = 105,
    plane_blue     = 106,
};
typedef __packed struct {
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
	学生机器人间通信 cmd_id 0x0301，内容 data_ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：数据上下行合计带宽不超过 5000 Byte。 上下行发送频率分别不超过30Hz。
 * +------+------+-------------+------------------------------------+
 * | byte | size |    breif    |            note                    |
 * |offset|      |             |                                    |
 * +------+------+-------------+------------------------------------+
 * |  0   |  2   | 	 data_ID   | 0x0200~0x02FF,可以在这些 ID 段选取 |
 * |      |      |             | 具体ID含义由参赛队自定义           |
 * +------|------|-------------|------------------------------------|
 * |  2   |  2   | 	sender_ID  | 需要校验发送者的 ID 正确性					|
 * +------|------|-------------|------------------------------------|
 * |  4   |  2   | receiver_ID | 需要校验接收者的 ID 正确性					|
 * |      |      |             | 例如不能发送到敌对机器人的ID				|
 * +------|------|-------------|------------------------------------|
 * |  6   |  n   | 		Data     | n 需要小于 113 										|
 * +------+------+-------------+------------------------------------+
*/
/******************************客户端交互数据**************************************/
#define INTERACT_DATA_LEN	113
typedef __packed struct //数据段内容格式
{
    uint16_t data_cmd_id;
    uint16_t send_ID;
    uint16_t receiver_ID;
} ext_client_data_header_t;
enum
{
    //0x200-0x02ff 	队伍自定义命令 格式  INTERACT_ID_XXXX
    INTERACT_ID_delete_graphic 			= 0x0100,	/*客户端删除图形*/
    INTERACT_ID_draw_one_graphic 		= 0x0101,	/*客户端绘制一个图形*/
    INTERACT_ID_draw_two_graphic 		= 0x0102,	/*客户端绘制2个图形*/
    INTERACT_ID_draw_five_graphic 	= 0x0103,	/*客户端绘制5个图形*/
    INTERACT_ID_draw_seven_graphic 	= 0x0104,	/*客户端绘制7个图形*/
    INTERACT_ID_draw_char_graphic 	= 0x0110,	/*客户端绘制字符图形*/
    INTERACT_ID_bigbome_num					= 0x02ff
};
typedef __packed struct
{
    uint8_t data[INTERACT_DATA_LEN]; //数据段,n需要小于113
} robot_interactive_data_t;
//单位（字节）
enum
{
    LEN_INTERACT_delete_graphic     = 8,  //删除图层 2(数据内容ID)+2(发送者ID)+2（接收者ID）+2（数据内容）
    LEN_INTERACT_draw_one_graphic   = 21, // 以上2+2+2+15
    LEN_INTERACT_draw_two_graphic   = 36, //6+15*2
    LEN_INTERACT_draw_five_graphic  = 81, //6+15*5
    LEN_INTERACT_draw_seven_graphic = 111,//6+15*7
    LEN_INTERACT_draw_char_graphic  = 51, //6+15+30（字符串内容）
};
//****************************绘图的数据段内容****************************/
typedef __packed struct//图形
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3; //直线  矩形  正圆  椭圆  圆弧  浮点  整型  字符
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;  //空    空    空    空    角度  大小  大小  大小
    uint32_t end_angle:9;    //空    空    空    空          位数  空    长度
    uint32_t width:10;
    uint32_t start_x:11;     //起点  起点  圆心  圆心  圆心  起点  起点  起点
    uint32_t start_y:11;     //
    uint32_t radius:10;      //空    空    半径  空    空    、    、    空
    uint32_t end_x:11;       //终点  对顶  空    半轴  半轴  、    、    空
    uint32_t end_y:11;       //                              数    数    空
} graphic_data_struct_t;
typedef __packed struct//浮点数
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    float number;
} Float_data_struct_t;
typedef __packed struct//整型数
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    int number;
} Int_data_struct_t;
/* data_ID: 0X0100  Byte:  2	    客户端删除图形*/
typedef __packed struct
{
    uint8_t operate_type;
    uint8_t layer;//图层数：0~9
} ext_client_custom_graphic_delete_t;
typedef enum
{
    NONE_delete    = 0,
    GRAPHIC_delete = 1,
    ALL_delete     = 2
} delete_Graphic_Operate; //ext_client_custom_graphic_delete_t：uint8_t operate_type
/*图层删除操作*/

//bit 0-2
typedef enum
{
    NONE   = 0,/*空操作*/
    ADD    = 1,/*增加图层*/
    MODIFY = 2,/*修改图层*/
    DELETE = 3,/*删除图层*/
} Graphic_Operate; //graphic_data_struct_t：uint32_t operate_tpye
/*图层操作*/
//bit3-5
typedef enum
{
    LINE      = 0,//直线
    RECTANGLE = 1,//矩形
    CIRCLE    = 2,//正圆
    OVAL      = 3,//椭圆
    ARC       = 4,//圆弧
    FLOAT     = 5,//浮点数
    INT       = 6,//整型数
    CHAR      = 7 //字符
} Graphic_Type;
/*图层类型*/
//bit 6-9图层数 最大为9，最小0
//bit 10-13颜色
typedef enum
{
    RED_BLUE  = 0,//红蓝主色
    YELLOW    = 1,
    GREEN     = 2,
    ORANGE    = 3,
    FUCHSIA   = 4,	/*紫红色*/
    PINK      = 5,
    CYAN_BLUE = 6,	/*青色*/
    BLACK     = 7,
    WHITE     = 8
} Graphic_Color;
/*图层颜色类型*/
//bit 14-31 角度 [0,360]
/**********************************客户端绘图************************************************/
//删除图层
typedef __packed struct
{
    std_frame_header_t txFrameHeader;
    uint16_t  CmdID;
    ext_client_data_header_t   dataFrameHeader;
    ext_client_custom_graphic_delete_t clientData;
    uint16_t	FrameTail;
} ext_deleteLayer_data_t;

//绘字符串
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
    uint8_t data[30];
} ext_client_string_t;

typedef __packed struct
{
    std_frame_header_t txFrameHeader;			//帧头
    uint16_t  CmdID;										//命令码
    ext_client_data_header_t   dataFrameHeader;//数据段头结构
    ext_client_string_t clientData;//数据段
    uint16_t	FrameTail;								//帧尾
} ext_charstring_data_t;
//绘象形图
typedef __packed struct
{
    std_frame_header_t txFrameHeader;			//帧头
    uint16_t  CmdID;										//命令码
    ext_client_data_header_t   dataFrameHeader;//数据段头结构
    graphic_data_struct_t clientData;		//数据段
    uint16_t	FrameTail;								//帧尾
} ext_graphic_one_data_t;

typedef __packed struct
{
    std_frame_header_t txFrameHeader;			//帧头
    uint16_t  CmdID;										//命令码
    ext_client_data_header_t   dataFrameHeader;//数据段头结构
    int8_t clientData[5];		//数据段
    uint16_t	FrameTail;								//帧尾
} ext_graphic_guard_data_t;

typedef __packed struct
{
    std_frame_header_t txFrameHeader;
    uint16_t  CmdID;
    ext_client_data_header_t   dataFrameHeader;
    graphic_data_struct_t clientData[2];
    uint16_t	FrameTail;

} ext_graphic_two_data_t;
typedef __packed struct
{
    std_frame_header_t txFrameHeader;
    uint16_t  CmdID;
    ext_client_data_header_t   dataFrameHeader;
    graphic_data_struct_t clientData[5];
    uint16_t	FrameTail;
} ext_graphic_five_data_t;
typedef __packed struct
{
    std_frame_header_t txFrameHeader;
    uint16_t  CmdID;
    ext_client_data_header_t   dataFrameHeader;
    graphic_data_struct_t clientData[7];
    uint16_t	FrameTail;
} ext_graphic_seven_data_t;
//绘制浮点型
typedef __packed struct
{
    std_frame_header_t txFrameHeader;
    uint16_t  CmdID;
    ext_client_data_header_t   dataFrameHeader;
    Float_data_struct_t clientData[2];
    uint16_t	FrameTail;
} ext_float_two_data_t;

typedef __packed struct
{
    std_frame_header_t txFrameHeader;
    uint16_t  CmdID;
    ext_client_data_header_t   dataFrameHeader;
    Float_data_struct_t clientData[7];
    uint16_t	FrameTail;
} ext_float_seven_data_t;
//绘制整型
typedef __packed struct
{
    std_frame_header_t txFrameHeader;
    uint16_t  CmdID;
    ext_client_data_header_t   dataFrameHeader;
    Int_data_struct_t clientData[2];
    uint16_t	FrameTail;
} ext_int_two_data_t;
typedef __packed struct
{
    std_frame_header_t txFrameHeader;
    uint16_t  CmdID;
    ext_client_data_header_t   dataFrameHeader;
    Int_data_struct_t clientData[7];
    uint16_t	FrameTail;
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
#define  Power_supply HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,GPIO_PIN_SET)  //供电

