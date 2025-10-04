#include "UI_task.h"
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "referee.h"
#include "stm32f4xx.h"
#include "bsp_usart.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "keyscan_task.h"

/*相关函数声明区BEGIN*/
void Client_Send_String(uint8_t *string, uint16_t length); // 串口发送函数
void Heaven_Pick_Up(void);								   // 更新标志图形
void Heaven_Add_UI(void);
void My_Car_State_UI(void);
void Claw_Pos(void);

void sucker_UI(void);
void arm_up_UI(void);
void arm_yaw_UI(void);
void go_state_UI(void);

void Char_Graphic(ext_client_string_t *graphic, // 发送char型字符，具体形参含义在定义中查看
				  const char *name,
				  uint32_t operate_tpye,
				  uint32_t layer,
				  uint32_t color,
				  uint32_t size,
				  uint32_t length,
				  uint32_t width,
				  uint32_t start_x,
				  uint32_t start_y,
				  const char *character); // 外部放入的数组

void Figure_Graphic(graphic_data_struct_t *graphic, // 最终要发出去的数组的数据段内容
					const char *name,
					uint32_t operate_tpye,
					uint32_t graphic_tpye, // 绘制什么图像
					uint32_t layer,
					uint32_t color,
					uint32_t start_angle,
					uint32_t end_angle,
					uint32_t width,
					uint32_t start_x,
					uint32_t start_y,
					uint32_t radius,
					uint32_t end_x,
					uint32_t end_y);

/*结构体实例化BEGIN*/
ext_graphic_five_data_t Heaven_pickup; // 绘制五个图形
ext_graphic_seven_data_t Heaven_addui;
ext_graphic_seven_data_t My_Car_state;
ext_client_string_t char_Marking;
ext_graphic_seven_data_t chassis_state;
ext_graphic_seven_data_t go_state_char;
ext_graphic_seven_data_t suck_state;
ext_graphic_five_data_t arm_up_state;  // 机械臂位置
ext_graphic_five_data_t arm_yaw_state; // 机械臂位置
ext_graphic_five_data_t arm_state;	   // 机械臂位置
ext_int_seven_data_t go_state_data;
// Float_data_struct_t go_state_data;
/*结构体实例化END*/

/*定义相关数组与变量BEGIN*/

uint8_t update_figure_flag;
uint8_t update_sucker_flag;
uint8_t update_arm_up_flag;
uint8_t update_go_float_flag;

uint8_t CliendTxBuffer[200]; // 真正发送数组
char empty_line[30] = {"                            "};
const char Sucker_buff1[] = {" Sucker1 "};
const char Sucker_buff2[] = {" Sucker2 "};
// const char Ground_buff[] = {" Ground "};
const char Help_buff[] = {" Help "};

uint16_t Recieve_id;

uint32_t center_line_x = 700;	  // 中心线x坐标
uint32_t center_line_below = 200; // 中心线下方
uint32_t center_line_upper = 300; // 中心线上方

uint32_t yaw_state_left;  // yaw轴左边位置
uint32_t yaw_state_right; // yaw轴右边位置

uint32_t up_state_x;			 // 抬升x坐标
uint32_t up_state_below;		 // 抬升下面位置
uint32_t up_state_upper;		 // 抬升下面位置
uint32_t little_yaw_state_left;	 // little_yaw轴左边位置
uint32_t little_yaw_state_right; // little_yaw轴右边位置

/*定义相关数组与变量END*/
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t UI_high_water; // 获取栈用量用来检测，正式比赛时关闭检测任务
#endif

/*UI终极句柄*/
UI_Draw Ui_Draw;

uint32_t UI_count;
void UI_init(void) // UI初始化
{
	//		vTaskDelay(1000);

	//	for(int i=0;i<5;i++)
	{
		if (RM_Referee.robot_state.robot_id == 2)
			Recieve_id = 0x102;
		else
			Recieve_id = 0x166;

		update_go_float_flag = ADD;
		sucker_UI();
		vTaskDelay(50);
		update_sucker_flag = ADD;
		go_state_UI();
		vTaskDelay(50);
		//		vTaskDelay(50);
		//		arm_up_UI();
		//		update_figure_flag = ADD;
		//		update_sucker_flag = ADD;
		//		update_arm_up_flag = ADD;
	}
	// vTaskDelay(50);
	// Heaven_Add_UI();
}
void UI_task(void const *argument) // 任务主循环
{
	//	UI_init();
	vTaskDelay(3000);
	ui_self_id = RM_Referee.robot_state.robot_id;
	//	ui_init_g_Ungroup();
	ui_init_g();
	while (1)
	{
		if (key_task.key_rc_ctrl->key.v == KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_SHIFT)
		{
			ui_init_g();
		}

		//		if (key_task.Base == 1)
		//		{
		//		ui_init_g_Ungroup();
		//		ui_init_g();
		////		}
		//		ui_self_id = RM_Referee.robot_state.robot_id;
		//  sucker_UI();·
		//  arm_up_UI();
		//
		//		if (RM_Referee.robot_state.robot_id == 2)
		//			Recieve_id = 0x102;
		//		else
		//			Recieve_id = 0x166;

		//		update_go_float_flag = MODIFY;
		//		update_sucker_flag = MODIFY;
		//		// update_arm_up_flag = MODIFY;
		//		if (UI_count % 2 == 0)
		//		{
		//			sucker_UI();
		//		}
		//		else if (UI_count % 3 == 1)
		//		{
		//			go_state_UI();
		//		}

		//		else if (UI_count % 5 == 0)
		//		{
		//			update_sucker_flag = ADD;
		//			sucker_UI();
		//		}
		//		else if (UI_count % 7 == 0)
		//		{
		//			update_go_float_flag = ADD;
		//			go_state_UI();
		//		}
		//		UI_count++;

		// vTaskDelay(1);

		/*模式*/
		if (!key_task.chassis_mode)
		{
			strcpy(ui_g_Ungroup_Keys_can_mode->string, "Chassis_mode");
		}
		if (key_task.custom_mode)
		{
			strcpy(ui_g_Ungroup_Keys_can_mode->string, "Custom_mode");
		}
		if (key_task.Base)
		{
			strcpy(ui_g_Ungroup_Keys_can_mode->string, "Base");
		}
		if (key_task.Safe)
		{
			strcpy(ui_g_Ungroup_Keys_can_mode->string, "Safe");
		}
		if (key_task.Cashing)
		{
			strcpy(ui_g_Ungroup_Keys_can_mode->string, "Cashing");
		}
		if (key_task.Gold)
		{
			strcpy(ui_g_Ungroup_Keys_can_mode->string, "Gold");
		}
		if (key_task.Silver)
		{
			strcpy(ui_g_Ungroup_Keys_can_mode->string, "Silver");
		}
		if (key_task.deposit)
		{
			strcpy(ui_g_Ungroup_Keys_can_mode->string, "deposit");
		}
		if (key_task.extraction)
		{
			strcpy(ui_g_Ungroup_Keys_can_mode->string, "extraction");
		}
		if (key_task.go_die_flag)
		{
			strcpy(ui_g_Ungroup_Keys_can_mode->string, "Y_reset");
		}
		if (Cup_X_reset)
		{
			strcpy(ui_g_Ungroup_Keys_can_mode->string, "Cup_X_reset");
		}

		/*数字*/
		ui_g_Ungroup_Cup_pitch_angle->number = (int32_t)Cup_target_pitch;
		ui_g_Ungroup_Yaw_num->number = (int32_t)yaw;
		ui_g_Ungroup_Pitch_num->number = (int32_t)pic;
		ui_g_Ungroup_Roll_num->number = (int32_t)Cup_target_roll;
		ui_g_Ungroup_Z_num->number = (int32_t)Z_target;
		ui_g_Ungroup_Y_num->number = (int32_t)Y_target;
		ui_g_Ungroup_X_num->number = (int32_t)X_target;

		/*方向*/
		if (!direction)
		{
			ui_g_now_strings[0].color = 1;
		}
		else
		{
			ui_g_now_strings[0].color = 6;
		}
		/*气泵*/
		if (!Cup_SF)
		{
			ui_g_Ungroup_save_M->color = 1;
		}
		else
		{
			ui_g_Ungroup_save_M->color = 2;
		}

		if (!Right_SF)
		{
			ui_g_Ungroup_save_R->color = 1;
		}
		else
		{
			ui_g_Ungroup_save_R->color = 2;
		}

		if (!Left_SF)
		{
			ui_g_Ungroup_save_L->color = 1;
		}
		else
		{
			ui_g_Ungroup_save_L->color = 2;
		}

		if (Custom_rx_threshold) // 自定义控制器可用
		{
			ui_g_Ungroup_Mode->color = 2;
		}
		else // 未接收到数据
		{
			ui_g_Ungroup_Mode->color = 1;
		}

		ui_update_g();
		if (UI_count % 100 == 0)
		{
			ui_init_g();
		}

		UI_count++;

		vTaskDelay(1);
	}
}

void Client_Send_String(uint8_t *string, uint16_t length)
{
	HAL_UART_Transmit(&huart6, string, length, 10);
}

uint16_t Car_begin_x = 1100;
uint16_t Car_begin_y = 100;

// void Heaven_Pick_Up(void)
//{
//	//帧头
//	Heaven_pickup.txFrameHeader.sof = JUDGE_FRAME_HEADER;
//	Heaven_pickup.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*5;
//	Heaven_pickup.txFrameHeader.seq = 0;//包序号
//	memcpy((void *)CliendTxBuffer, &Heaven_pickup.txFrameHeader, sizeof(std_frame_header_t));
//	append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验
//	//命令码
//	Heaven_pickup.CmdID = ID_robot_interactive_header_data;
//	//数据段头结构
//	Heaven_pickup.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
//	Heaven_pickup.dataFrameHeader.send_ID     = RM_Referee.robot_state.robot_id;
//	Heaven_pickup.dataFrameHeader.receiver_ID = Recieve_id;
//	//数据段
//	/*开关显示*/
//	if(Qi_Ui.Help_Qi) //救援气路开关
//	{
//        /*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
//		Figure_Graphic(&Heaven_pickup.clientData[0],"GL1",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,5,250,600,30,250,200);
//	}else
//	{
//        /*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
//		Figure_Graphic(&Heaven_pickup.clientData[0],"GL1",update_figure_flag,CIRCLE,1,GREEN,0,0,5,250,600,30,250,200);
//	}
//	if(Qi_Ui.Sucker_Qi) //取矿吸盘开关
//	{
//        /*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
//		Figure_Graphic(&Heaven_pickup.clientData[1],"GL2",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,5,250,700,30,250,200);
//	}else
//	{
//        /*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
//		Figure_Graphic(&Heaven_pickup.clientData[1],"GL2",update_figure_flag,CIRCLE,1,GREEN,0,0,5,250,700,30,250,200);
//	}
//    /*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
//	Figure_Graphic(&Heaven_pickup.clientData[2],"GL3",update_figure_flag,LINE,1,PINK,0,0,5, 1900, 397, 10, 1490, 630); //右侧边缘限制
//	Figure_Graphic(&Heaven_pickup.clientData[3],"GL4",update_figure_flag,RECTANGLE,2,GREEN,0,0,7, 1900, 100,0, 1200, 400); //区域限制
//	Figure_Graphic(&Heaven_pickup.clientData[4],"GL5",update_figure_flag,RECTANGLE,2,GREEN,0,0,5, 1050, 100,0, 850, 800); //空接辅助
//	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&Heaven_pickup.CmdID, LEN_CMD_ID+Heaven_pickup.txFrameHeader.data_length); //加上命令码长度2
//
//	//帧尾
//	append_CRC16_check_sum(CliendTxBuffer,sizeof(Heaven_pickup));
//	Client_Send_String(CliendTxBuffer, sizeof(Heaven_pickup));
//}

uint8_t CliendTxBuffer_State[400];

void Heaven_Add_UI(void)
{
	// 帧头
	Heaven_addui.txFrameHeader.sof = JUDGE_FRAME_HEADER;
	Heaven_addui.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t) * 7;
	Heaven_addui.txFrameHeader.seq = 1; // 包序号
	memcpy((void *)CliendTxBuffer, &Heaven_addui.txFrameHeader, sizeof(std_frame_header_t));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t)); // 头校验

	// 命令码
	Heaven_addui.CmdID = ID_robot_interactive_header_data;

	// 数据段头结构
	Heaven_addui.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
	Heaven_addui.dataFrameHeader.send_ID = RM_Referee.robot_state.robot_id;
	Heaven_addui.dataFrameHeader.receiver_ID = Recieve_id;

	// 数据段

	/*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
	Figure_Graphic(&Heaven_addui.clientData[6], "GL7", update_figure_flag, LINE, 2, PINK, 0, 0, 5, 207, 173, 10, 852, 544);		  // 右侧边缘限制
	memcpy(CliendTxBuffer + LEN_FRAME_HEAD, (uint8_t *)&Heaven_addui.CmdID, LEN_CMD_ID + Heaven_addui.txFrameHeader.data_length); // 加上命令码长度2

	// 帧尾
	append_CRC16_check_sum(CliendTxBuffer, sizeof(Heaven_addui));
	Client_Send_String(CliendTxBuffer, sizeof(Heaven_addui));
}

uint8_t CliendTxBuffer_State[400];

void My_Car_State_UI(void) // 当前车辆状态
{

	// 帧头
	My_Car_state.txFrameHeader.sof = JUDGE_FRAME_HEADER;
	My_Car_state.txFrameHeader.data_length = 51;
	My_Car_state.txFrameHeader.seq = 2; // 包序号
	memcpy((void *)CliendTxBuffer_State, &My_Car_state.txFrameHeader, sizeof(std_frame_header_t));
	append_CRC8_check_sum(CliendTxBuffer_State, sizeof(std_frame_header_t)); // 头校验

	// 命令码
	My_Car_state.CmdID = ID_robot_interactive_header_data;

	// 数据段头结构
	My_Car_state.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
	My_Car_state.dataFrameHeader.send_ID = RM_Referee.robot_state.robot_id;
	My_Car_state.dataFrameHeader.receiver_ID = Recieve_id;

	// 数据段
	Claw_Pos();
	memcpy(CliendTxBuffer_State + LEN_FRAME_HEAD, (uint8_t *)&My_Car_state.CmdID, LEN_CMD_ID + My_Car_state.txFrameHeader.data_length); // 加上命令码长度2

	// 帧尾
	append_CRC16_check_sum(CliendTxBuffer_State, sizeof(My_Car_state));
	Client_Send_String(CliendTxBuffer_State, sizeof(My_Car_state));
}

void Claw_Pos(void)
{

	//		Ui_Draw.Level_2.Height = (uint16_t)Extend_Lift_Motor.Motor_Info.Position_Now * 50 / 23.5 + 150;
	//		Figure_Graphic(&My_Car_state.clientData[0],"STATE1",update_figure_flag,LINE,2,WHITE,0,0,5, CHE_JIA_ZI_X, Ui_Draw.Level_2.Height + 100, 20, CHE_JIA_ZI_X + 200, Ui_Draw.Level_2.Height + 150);
	//
	//
	//		Figure_Graphic(&My_Car_state.clientData[1],"STATE2",update_figure_flag,LINE,3,ORANGE,0,0,5,  \
//										LEFT_EXCHANGE_LINE_X, LEFT_EXCHANGE_LINE_Y, 20, LEFT_EXCHANGE_LINE_X, LEFT_EXCHANGE_LINE_Y + 400);
	//		Figure_Graphic(&My_Car_state.clientData[4],"STATE5",update_figure_flag,LINE,6,ORANGE, 0,0,5,  \
//										LEFT_EXCHANGE_LINE_X + 350, LEFT_EXCHANGE_LINE_Y, 20, LEFT_EXCHANGE_LINE_X + 350, LEFT_EXCHANGE_LINE_Y + 400);
	//		Figure_Graphic(&My_Car_state.clientData[5],"STATE6",update_figure_flag,LINE,7,GREEN, 0,0,3, Car_begin_x+150, Car_begin_y+175,  0,  Car_begin_x+350, Car_begin_y);
	//		Figure_Graphic(&My_Car_state.clientData[2],"STATE3",update_figure_flag,LINE,4,GREEN, 0,0,3, Car_begin_x+50, Car_begin_y-25,  0,  Car_begin_x+50, Car_begin_y+200);
	//	  	Figure_Graphic(&My_Car_state.clientData[3],"STATE4",update_figure_flag,LINE,5,GREEN, 0,0,3, Car_begin_x+250, Car_begin_y-25,  0, Car_begin_x+250, Car_begin_y+200);
	//  	Figure_Graphic(&My_Car_state.clientData[6],"STATE7",update_figure_flag,LINE,8,GREEN, 0,0,3, Car_begin_x+50, Car_begin_y+85,  0,  Car_begin_x+250, Car_begin_y+85);

	/*函数名，”图像名称“图层操作，图层，颜色，字体大小，字符长度，字宽，起始位置x坐标，始位置y坐标，显示的内容*/
	Char_Graphic(&char_Marking, "char1", update_figure_flag, 1, YELLOW, 12, 10, 20, 850, 200, Sucker_buff1);
	// Char_Graphic(&char_Marking,"char2",update_figure_flag,1,YELLOW,12,10,20,600,200,Ground_buff);
	Char_Graphic(&char_Marking, "char3", update_figure_flag, 1, YELLOW, 12, 10, 20, 500, 200, Help_buff);
}

void Char_Graphic				   //! 绘制文本字符
	(ext_client_string_t *graphic, // 最终要发出去的数组中的数据段内容
	 const char *name,			   // 图像名称
	 uint32_t operate_tpye,		   // 图层操作
	 uint32_t layer,			   // 图层
	 uint32_t color,			   // 颜色
	 uint32_t size,				   // 字体大小
	 uint32_t length,			   // 字符长度
	 uint32_t width,			   // 字宽
	 uint32_t start_x,			   // 起始位置x坐标
	 uint32_t start_y,			   // 起始位置y坐标
	 const char *character)		   // 外部放入的数组，内容是要显示的内容
{
	graphic_data_struct_t *data_struct = &graphic->grapic_data_struct;
	for (char i = 0; i < 3; i++)
		data_struct->graphic_name[i] = name[i]; // 字符索引
	data_struct->operate_tpye = operate_tpye;	// 图层操作
	data_struct->graphic_tpye = CHAR;			// Char型
	data_struct->layer = layer;					// 都在第零层
	data_struct->color = color;					// 都是白色
	data_struct->start_angle = size;
	data_struct->end_angle = length;
	data_struct->width = width;
	data_struct->start_x = start_x;
	data_struct->start_y = start_y;

	data_struct->radius = 0;
	data_struct->end_x = 0;
	data_struct->end_y = 0;

	memcpy((void *)graphic->data, empty_line, 28);
	memcpy((void *)graphic->data, character, length);
}
void Figure_Graphic					 //! 绘制基本图形（线、圆、矩形）
	(graphic_data_struct_t *graphic, // 最终要发出去的数组的数据段内容
	 const char *name,				 // 图像名称
	 uint32_t operate_tpye,			 // 图层操作
	 uint32_t graphic_tpye,			 // 绘制什么图像
	 uint32_t layer,				 // 图层选择
	 uint32_t color,				 // 颜色选择
	 uint32_t start_angle,			 // 开始角度
	 uint32_t end_angle,			 // 结束角度
	 uint32_t width,				 // 宽度
	 uint32_t start_x,				 // 起始位置x坐标
	 uint32_t start_y,				 // 起始位置y坐标
	 uint32_t radius,				 // 半径
	 uint32_t end_x,				 // 结束位置x坐标
	 uint32_t end_y)				 // 结束位置y坐标
{

	for (char i = 0; i < 3; i++)
		graphic->graphic_name[i] = name[i]; // 字符索引

	graphic->operate_tpye = operate_tpye; // 图层操作
	graphic->graphic_tpye = graphic_tpye; // Char型
	graphic->layer = layer;				  // 都在第一层
	graphic->color = color;				  // 变色
	graphic->start_angle = start_angle;
	graphic->end_angle = end_angle;
	graphic->width = width;
	graphic->start_x = start_x;
	graphic->start_y = start_y;
	graphic->radius = radius;
	graphic->end_x = end_x;
	graphic->end_y = end_y;
}

void Int_Graphic				 //! 显示整型数据
	(Int_data_struct_t *graphic, // 最终要发出去的数组的数据段内容
	 const char *name,
	 uint32_t operate_tpye,
	 uint32_t graphic_tpye, // 绘制什么图像
	 uint32_t layer,
	 uint32_t color,
	 uint32_t size,
	 uint32_t zero,
	 uint32_t width,
	 uint32_t start_x,
	 uint32_t start_y,
	 int number)
{
	for (char i = 0; i < 3; i++)
		graphic->graphic_name[i] = name[i]; // 字符索引
	graphic->operate_tpye = operate_tpye;	// 图层操作
	graphic->graphic_tpye = graphic_tpye;
	graphic->layer = layer; // 都在第一层
	graphic->color = color; // 变色
	graphic->start_angle = size;
	graphic->end_angle = zero;
	graphic->width = width;
	graphic->start_x = start_x;
	graphic->start_y = start_y;
	graphic->number = number;
}

void Float_Graphic				   //! 显示浮点数据
	(Float_data_struct_t *graphic, // 最终要发出去的数组的数据段内容
	 const char *name,
	 uint32_t operate_tpye,
	 uint32_t graphic_tpye, // 绘制什么图像
	 uint32_t layer,
	 uint32_t color,
	 uint32_t size,
	 uint32_t decimal,
	 uint32_t width,
	 uint32_t start_x,
	 uint32_t start_y,
	 int32_t number)
{
	for (char i = 0; i < 3; i++)
		graphic->graphic_name[i] = name[i]; // 字符索引
	graphic->operate_tpye = operate_tpye;	// 图层操作
	graphic->graphic_tpye = graphic_tpye;
	graphic->layer = layer; //
	graphic->color = color; // 变色
	graphic->start_angle = size;
	graphic->end_angle = decimal; // 小数有效位
	graphic->width = width;
	graphic->start_x = start_x;
	graphic->start_y = start_y;
	graphic->number = number;
}

void chassis_UI(void)
{
	//	/*函数名，”图像名称“图层操作，图层，颜色，字体大小，字符长度，字宽，起始位置x坐标，始位置y坐标，显示的内容*/
	//	Figure_Graphic(&chassis_state,"GL2",update_figure_flag,1,YELLOW,12,10,20,850,200,Sucker_buff);
	//	//Char_Graphic(&char_Marking,"char2",update_figure_flag,1,YELLOW,12,10,20,600,200,Ground_buff);
	//	Char_Graphic(&char_Marking,"char3",update_figure_flag,1,YELLOW,12,10,20,500,200,Help_buff);
}

void sucker_UI(void)
/*吸盘状态显示(sucker_UI)
使用粉色/绿色圆形表示吸盘开关状态
绘制机械臂位置指示线*/
{
	// 帧头
	suck_state.txFrameHeader.sof = JUDGE_FRAME_HEADER;
	suck_state.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t) * 7;
	suck_state.txFrameHeader.seq = 0; // 包序号
	memcpy((void *)CliendTxBuffer, &suck_state.txFrameHeader, sizeof(std_frame_header_t));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t)); // 头校验

	// 命令码
	suck_state.CmdID = ID_robot_interactive_header_data;

	// 数据段头结构
	suck_state.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
	suck_state.dataFrameHeader.send_ID = RM_Referee.robot_state.robot_id;
	suck_state.dataFrameHeader.receiver_ID = Recieve_id;

	//	if (suck_get_flag)
	//		/*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
	//		Figure_Graphic(&suck_state.clientData[0], "GL1", update_sucker_flag, CIRCLE, 1, GREEN, 0, 0, 5, 250, 600, 30, 250, 200);
	//	else
	//		/*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
	//		Figure_Graphic(&suck_state.clientData[0], "GL1", update_sucker_flag, CIRCLE, 1, PINK, 0, 0, 5, 250, 600, 30, 250, 200);
	//	if (suck_store_flag)
	//		/*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
	//		Figure_Graphic(&suck_state.clientData[1], "GL2", update_sucker_flag, CIRCLE, 1, GREEN, 0, 0, 5, 250, 700, 30, 250, 200);
	//	else
	//		/*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
	//		Figure_Graphic(&suck_state.clientData[1], "GL2", update_sucker_flag, CIRCLE, 1, PINK, 0, 0, 5, 250, 700, 30, 250, 200);

	/*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
	Figure_Graphic(&suck_state.clientData[2], "GL3", update_sucker_flag, LINE, 1, PINK, 0, 0, 5, 1900, 397, 10, 1490, 630); // 右侧小臂位置
	Figure_Graphic(&suck_state.clientData[3], "GL4", update_sucker_flag, LINE, 1, PINK, 0, 0, 5, 1500, 397, 10, 1090, 630); // 右侧小臂位置
	//	if (c_go_pos.live_flag)
	//		/*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
	//		Figure_Graphic(&suck_state.clientData[4], "GL5", update_sucker_flag, CIRCLE, 1, GREEN, 0, 0, 5, 150, 600, 30, 150, 200);
	//	else
	//		/*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
	//		Figure_Graphic(&suck_state.clientData[4], "GL5", update_sucker_flag, CIRCLE, 1, PINK, 0, 0, 5, 150, 600, 30, 150, 200);

	//	if (low_vol_flag)
	//		/*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
	//		Figure_Graphic(&suck_state.clientData[5], "GL6", update_sucker_flag, RECTANGLE, 1, PINK, 0, 0, 5, 500, 800, 30, 600, 700);
	//	else
	//		/*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
	//		Figure_Graphic(&suck_state.clientData[5], "GL6", update_sucker_flag, RECTANGLE, 1, GREEN, 0, 0, 5, 500, 800, 30, 550, 750);
	//	memcpy(CliendTxBuffer + LEN_FRAME_HEAD, (uint8_t *)&suck_state.CmdID, LEN_CMD_ID + suck_state.txFrameHeader.data_length); // 加上命令码长度2

	// 帧尾
	append_CRC16_check_sum(CliendTxBuffer, sizeof(suck_state));
	Client_Send_String(CliendTxBuffer, sizeof(suck_state));
}

void arm_up_UI(void)
/*
机械臂位置显示(arm_up_UI)
垂直线条表示升降范围
粉色圆点指示当前高度位置
*/
{
	// 帧头
	arm_up_state.txFrameHeader.sof = JUDGE_FRAME_HEADER;
	arm_up_state.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t) * 5;
	arm_up_state.txFrameHeader.seq = 0; // 包序号
	memcpy((void *)CliendTxBuffer, &arm_up_state.txFrameHeader, sizeof(std_frame_header_t));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t)); // 头校验

	// 命令码
	arm_up_state.CmdID = ID_robot_interactive_header_data;

	// 数据段头结构
	arm_up_state.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
	arm_up_state.dataFrameHeader.send_ID = RM_Referee.robot_state.robot_id;
	arm_up_state.dataFrameHeader.receiver_ID = Recieve_id;

	//	up_state_x = center_line_x;
	//	up_state_below = center_line_below;
	//	up_state_upper = key_task.motor_message.go_up_pos * (center_line_upper - center_line_below) / (up_upper_limit - 0) + center_line_below;

	// 中心线
	/*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
	Figure_Graphic(&arm_up_state.clientData[0], "UP1", update_arm_up_flag, LINE, 1, GREEN, 0, 0, 5, center_line_x, center_line_below, 10, center_line_x, center_line_upper);
	Figure_Graphic(&arm_up_state.clientData[1], "UP2", update_arm_up_flag, LINE, 1, BLACK, 0, 0, 5, center_line_x - 30, center_line_below, 10, center_line_x + 30, center_line_below);
	Figure_Graphic(&arm_up_state.clientData[2], "UP3", update_arm_up_flag, LINE, 1, BLACK, 0, 0, 5, center_line_x - 30, center_line_upper, 10, center_line_x + 30, center_line_upper);

	/*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
	Figure_Graphic(&arm_up_state.clientData[3], "UP4", update_arm_up_flag, CIRCLE, 1, PINK, 0, 0, 5, up_state_x, up_state_below, 5, up_state_x, up_state_upper);

	memcpy(CliendTxBuffer + LEN_FRAME_HEAD, (uint8_t *)&arm_up_state.CmdID, LEN_CMD_ID + arm_up_state.txFrameHeader.data_length); // 加上命令码长度2

	// 帧尾
	append_CRC16_check_sum(CliendTxBuffer, sizeof(arm_up_state));
	Client_Send_String(CliendTxBuffer, sizeof(arm_up_state));
}

void arm_yaw_UI(void)
{
	// 帧头
	arm_yaw_state.txFrameHeader.sof = JUDGE_FRAME_HEADER;
	arm_yaw_state.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t) * 5;
	arm_yaw_state.txFrameHeader.seq = 0; // 包序号
	memcpy((void *)CliendTxBuffer, &arm_state.txFrameHeader, sizeof(std_frame_header_t));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t)); // 头校验

	// 命令码
	arm_state.CmdID = ID_robot_interactive_header_data;

	// 数据段头结构
	arm_yaw_state.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
	arm_yaw_state.dataFrameHeader.send_ID = RM_Referee.robot_state.robot_id;
	arm_yaw_state.dataFrameHeader.receiver_ID = Recieve_id;

	//	up_state_x = center_line_x;
	//	up_state_below = center_line_below;
	//	up_state_upper = key_task.motor_message.go_up_pos * (center_line_upper - center_line_below) / (up_upper_limit - 0) + center_line_below;

	// 中心线
	/*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
	Figure_Graphic(&arm_yaw_state.clientData[0], "UP1", update_figure_flag, LINE, 1, BLACK, 0, 0, 5, center_line_x, center_line_below, 10, center_line_x, center_line_upper);
	Figure_Graphic(&arm_yaw_state.clientData[1], "UP2", update_figure_flag, LINE, 1, GREEN, 0, 0, 5, center_line_x - 30, center_line_below, 10, center_line_x + 30, center_line_below);
	Figure_Graphic(&arm_yaw_state.clientData[2], "UP3", update_figure_flag, LINE, 1, GREEN, 0, 0, 5, center_line_x - 30, center_line_upper, 10, center_line_x + 30, center_line_upper);

	/*函数名.数据序号【】，”图像名称“，图层操作，绘制什么图像，图层选择，颜色选择，开始角度，结束角度，宽度，起始位置x坐标，起始位置y坐标，半径，结束位置x坐标，结束位置y坐标 */
	Figure_Graphic(&arm_yaw_state.clientData[3], "UP4", update_figure_flag, CIRCLE, 1, PINK, 0, 0, 5, up_state_x, up_state_below, 5, up_state_x, up_state_upper);

	memcpy(CliendTxBuffer + LEN_FRAME_HEAD, (uint8_t *)&arm_yaw_state.CmdID, LEN_CMD_ID + arm_yaw_state.txFrameHeader.data_length); // 加上命令码长度2

	// 帧尾
	append_CRC16_check_sum(CliendTxBuffer, sizeof(arm_yaw_state));
	Client_Send_String(CliendTxBuffer, sizeof(arm_yaw_state));
}

// void go_UI(void)
//{
//	/*函数名，”图像名称“图层操作，图层，颜色，字体大小，字符长度，字宽，起始位置x坐标，始位置y坐标，显示的内容*/
//	Figure_Graphic(&go_state_char,"char3",update_figure_flag,1,YELLOW,12,10,20,850,200,Sucker_buff);
//	//Char_Graphic(&char_Marking,"char2",update_figure_flag,1,YELLOW,12,10,20,600,200,Ground_buff);
//	Char_Graphic(&char_Marking,"char3",update_figure_flag,1,YELLOW,12,10,20,500,200,Help_buff);
// }

static void go_state_float() // 视觉发来的距离
{
	/*函数名，”图像名称“图层操作，图层，颜色，字体大小，有效位，字宽，起始位置x坐标，始位置y坐标，显示的内容*/
	//	Int_Graphic(&go_state_data.clientData[0], "FR1", update_go_float_flag, INT, 1, CYAN_BLUE, 30, 2, 3, 350, 600, -(c_go_pos.GO_yaw + go_offset_angle[0]) * 100 * 90 / 950 / 100);
	//	if ((-(c_go_pos.GO_up + go_offset_angle[1]) * 100 / 4500) >= 90)
	//		Int_Graphic(&go_state_data.clientData[1], "FR2", update_go_float_flag, INT, 1, PINK, 45, 2, 3, 350, 670, -(c_go_pos.GO_up + go_offset_angle[1]) * 100 / 4500);
	//	else
	//		Int_Graphic(&go_state_data.clientData[1], "FR2", update_go_float_flag, INT, 1, CYAN_BLUE, 30, 2, 3, 350, 670, -(c_go_pos.GO_up + go_offset_angle[1]) * 100 / 4500);
	//	Int_Graphic(&go_state_data.clientData[2], "FR3", update_go_float_flag, INT, 1, CYAN_BLUE, 30, 2, 3, 350, 740, -(c_go_pos.GO_little_yaw + go_offset_angle[2]) * 100 * 90 / 950 / 100);
	//	Int_Graphic(&go_state_data.clientData[3], "FR4", update_go_float_flag, INT, 1, CYAN_BLUE, 30, 2, 3, 350, 810, position_moto_2006[0].total_angle_true * 90 / 2200);
}
void go_state_UI() /// 一个图像更新
/*
运动状态显示(go_state_UI)
显示视觉定位数据
用不同颜色标识正常/异常状态
*/
{
	// 帧头
	go_state_data.txFrameHeader.sof = JUDGE_FRAME_HEADER;
	go_state_data.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(Int_data_struct_t) * 7;
	go_state_data.txFrameHeader.seq = 0; // 包序号
	memcpy(CliendTxBuffer, &go_state_data.txFrameHeader, sizeof(std_frame_header_t));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t)); // 头校验

	// 命令码
	go_state_data.CmdID = ID_robot_interactive_header_data;

	// 数据段头结构
	go_state_data.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
	go_state_data.dataFrameHeader.send_ID = RM_Referee.robot_state.robot_id;
	go_state_data.dataFrameHeader.receiver_ID = Recieve_id;

	// 数据段
	go_state_float();
	memcpy(CliendTxBuffer + LEN_FRAME_HEAD, (uint8_t *)&go_state_data.CmdID, LEN_CMD_ID + go_state_data.txFrameHeader.data_length); // 加上命令码长度2

	// 帧尾
	append_CRC16_check_sum(CliendTxBuffer, sizeof(go_state_data));

	Client_Send_String(CliendTxBuffer, sizeof(go_state_data));
}