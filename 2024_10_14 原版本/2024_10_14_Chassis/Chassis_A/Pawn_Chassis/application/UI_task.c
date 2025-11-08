/**
  * @author  Zhang zhaobo
  * 自定义用户界面UI任务
  *  2022.2.13
  *  使用时需包含   UI_task.h
  * 1.给自定义UI单独创建一个任务，方便精准把握发送频率，可以将优先级放低，推荐频率10HZ
  * 2.const RM_Referee_system_t*RM_Referee;这个结构体是外部引用的，包含的是全部裁判系统传输信息
  * 3.整体流程为先进行必要的判断，如判断客户端和自己ID，各个机构状态，再进行初始化，最后开始刷新UI
  *
  */
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
#include "gimbal_task.h"
#include "bsp_usart.h"
#include "decet_task.h"
#include "stm32f4xx_hal.h"
#include "shoot.h"
#include "gimbal_task.h"
#include "CAN_receive.h"
#include "arm_math.h"
/*相关函数声明区BEGIN*/
void Client_Sent_String(uint8_t *string, uint16_t length);//串口发送函数
void Client_graphic_Init(void);//初始化UI。初始化字符串
void Client_graphic_Info_update(void);//更新标志图形
void Client_supercapacitor_update(void);//更新电容状态
void Client_aim_update(void);//更新十字准心
void autoshoot_allow(void);//更新瞄准线
void _high_aim_(void);
void _lowlong_aim_(void);
void _lowshort_aim_2(void);
void _lowshort_aim_3(void);
void Char_Graphic(ext_client_string_t* graphic,//发送char型字符，具体形参含义在定义中查看
                  const char* name,
                  uint32_t operate_tpye,
                  uint32_t layer,
                  uint32_t color,
                  uint32_t size,
                  uint32_t length,
                  uint32_t width,
                  uint32_t start_x,
                  uint32_t start_y,
                  const char *character);//外部放入的数组
void Figure_Graphic(graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
                    const char* name,
                    uint32_t operate_tpye,
                    uint32_t graphic_tpye,//绘制什么图像
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
static void Draw_char(void);//绘制字符
static void Draw_Figure_bool(void);//绘制标志图形
static void supercapacitor_figure(fp32 remain_energy,uint32_t turning_point);//剩余超级电容（单位百分比），低于某百分比变红色
static void auto_fire(void);//左上第一个自瞄打击标志判别
static void chassis(void);//左上第二个底盘状态标志判别
static void firction(void);//左上第三个发射机构状态判别
static void Switch(void);//左上第四个发射机构状态判别
static void Mode(void);
static void Mode(void);
static void sight_bead_figrue(uint32_t x,uint32_t y);//可移动准心，请强制转换成uint32_t1920*1080有部分地区无法画出
static void aim_lowshort_3(uint32_t division_value,uint32_t line_length);//准心上半部分的宽度"AM"--aim_low_middle
static void aim_1(uint32_t division_value,uint32_t line_length);//准心上半部分的宽度"AH"--aim_high
static void aim_lowshort_2(uint32_t division_value,uint32_t line_length);//准心上半部分的宽度"AL"--aim_low
static void aim_lowshort_stem(uint32_t division_value,uint32_t line_length);
void guard_control();//哨兵遥控
void Indicator_light(void);//指示灯函数
void flag_fresh();//flag刷新
void UI_Init();
void Figure_Guard();//结束位置y坐标
/*相关函数声明区END*/


/*结构体实例化BEGIN*/
ext_charstring_data_t tx_client_char;//绘字符串
ext_graphic_five_data_t tx_client_graphic_figure;//绘制五个图形
ext_graphic_one_data_t tx_supercapacitor_figure;//绘制一个图形
ext_graphic_two_data_t tx_aim_figure;//绘制两个图形
ext_graphic_seven_data_t high_aim_figure;//绘制七个图形
ext_graphic_seven_data_t low_aim_shortfigure_1;//绘制七个图形
ext_graphic_two_data_t low_aim_shortfigure_2;//绘制两个图形
ext_graphic_two_data_t  low_aim_shortfigure_3;//绘制两个图形
ext_graphic_five_data_t  low_aim_longfigure;//绘制五个图形
ext_graphic_guard_data_t tx_guard_figure;//哨兵标志位
/*结构体实例化END*/

/*定义相关数组与变量BEGIN*/
uint8_t Client_circle = 10;//发送循环
uint8_t state_first_graphic;//0~7循环
uint8_t CliendTxBuffer[200];//真正发送数组
uint8_t update_figure_flag;
uint8_t update_supercapacitor_flag;
fp32 global_supercapacitor_remain = 0;//[0,100]
uint32_t division_value = 10;
uint8_t update_aim_flag,char_flag;//1-add,3删除
uint8_t auto_sope_flag,lowlong_flag;
uint8_t guard_flag;
int gimbal_auto=0;//判断云台是否处在自瞄模式标志位
int target_status=0;//判断视野内有无目标标志位
int best_fire=0;//判断目标是否在最佳打击状态标志位
int chassis_judge=0;//判断底盘状态
int firction_judge=0;//判断发射机构摩擦轮状态
int switch_judge=0;//判断限位状态
int mode_judge=0;//当前模式 常规 小幅 大幅 反哨兵 飞坡
uint32_t global_sight_bead_x = 960,global_sight_bead_y = 720,global_supercapacitor_point = 31;//[0,100]
char first_line[30]  = {"Cover"};//自瞄状态和是否可以射击,最多放30个字符串，bool
char second_line[30] = {"Chais"};//底盘状态以及小陀螺
char third_line[30]  = {"Fire"};//发射机构状态
char fourth_line[30] = {"Mode"};//弹仓盖
char fifth_line[30]  = {"SPEED_HIGH"};
char sixth_line[30]  = {"SPEED_LOW"};
char seventh_line[30]  = {"FRE_HIGH"};//进攻战术建议
char eighth_line[30]  = {"FRE_LOW"};//离线模块
char empty_line[30] = {"                            "};
uint16_t Recieve_id,Guard_id;
const RM_Referee_system_t *get_Referee_point()
{
    return &RM_Referee;
}

/*定义相关数组与变量END*/
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t UI_high_water;//获取栈用量用来检测，正式比赛时关闭检测任务
#endif
void UI_task(void const *pvParameters)
{
    vTaskDelay(1000);
    UI_Init();
    while (1)
    {
        static uint32_t i;
        if     (RM_Referee.robot_state.robot_id==3)Recieve_id =   0x103;
        else if(RM_Referee.robot_state.robot_id==4)Recieve_id =   0x104;
        else if(RM_Referee.robot_state.robot_id==5)Recieve_id =   0x105;
        else if(RM_Referee.robot_state.robot_id==103)Recieve_id = 0x167;
        else if(RM_Referee.robot_state.robot_id==104)Recieve_id = 0x168;
        else if(RM_Referee.robot_state.robot_id==105)Recieve_id = 0x169;

        Indicator_light();//更新指示灯
        flag_fresh();
        if(i%Client_circle == 0)
        {   //更新第零层图层，放置字符
            Client_graphic_Init();//写字什么的
            char_flag = MODIFY;
        }
        else if(i%Client_circle == 1)
        {   //更新第一层图层，放置各种状态标志位
            Client_graphic_Info_update();
            update_figure_flag = MODIFY;
        }
        else if(i%Client_circle == 2)
        {   //更新第二层图层，放置十字准心
            _lowlong_aim_();//瞄准刻度
            lowlong_flag=MODIFY;
//			Client_aim_update();
//			update_aim_flag = MODIFY;
        }
        else if(i%Client_circle == 3)
        {   //更新第三层图层，放置电容状态
            Client_supercapacitor_update();
            update_supercapacitor_flag = MODIFY;
        }
        else if(i%Client_circle == 4)
        {   //更新第四层图层，自瞄框
            autoshoot_allow();
            auto_sope_flag = MODIFY;
        }
//		else if(i%Client_circle == 5)
//		{//更新第四层图层，放置战略建议，待完善
//			guard_control();
//			guard_flag = MODIFY;
//		}
        /********************不太需要频繁刷新的*************************/
//			else if(i%Client_circle == 5)
//		{
//
//		}
        /*****************隔长时间进行一次添加************************/
        if(i%100 == 5)
        {
            Client_graphic_Info_update();
            update_figure_flag = ADD;
        }
        else if(i%100 == 10)
        {
            Client_supercapacitor_update();
            update_supercapacitor_flag = ADD;
        }
        else if(i%100 == 15)
        {
            autoshoot_allow();
            auto_sope_flag = ADD;
        }
        else if(i%100 == 20)
        {
            Client_graphic_Init();
            char_flag = ADD;
        }
        else if(i%100 == 25)
        {
//			Client_aim_update();
//			update_aim_flag = ADD;
        }
        else if(i%100 == 30)
        {
            _lowlong_aim_();
            lowlong_flag=ADD;
        }
        else if(i%100 == 35)
        {
            _lowshort_aim_3();
        }
        /************************************************/
//		else if(i%100 == 30)
//		{
//			_lowshort_aim_3();
//		}
//		else if(i%100 == 40)
//		{
//			_lowlong_aim_();
//		}
        i++;
        vTaskDelay(10);
        decet_flag.UI_count++;

#if INCLUDE_uxTaskGetStackHighWaterMark
        UI_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
void Client_Sent_String(uint8_t *string, uint16_t length)
{
    HAL_UART_Transmit(&huart6,string,length,100);
}
void UI_Init()
{
    static uint32_t i;
    flag_fresh();
    if(RM_Referee.robot_state.robot_id==3)			 {
        Recieve_id = 0x103;
        Guard_id = 7;
    }
    else if(RM_Referee.robot_state.robot_id==4)  {
        Recieve_id = 0x104;
        Guard_id = 7;
    }
    else if(RM_Referee.robot_state.robot_id==5)  {
        Recieve_id = 0x105;
        Guard_id = 7;
    }
    else if(RM_Referee.robot_state.robot_id==103) {
        Recieve_id = 0x167;
        Guard_id = 107;
    }
    else if(RM_Referee.robot_state.robot_id==104) {
        Recieve_id = 0x168;
        Guard_id = 107;
    }
    else if(RM_Referee.robot_state.robot_id==105) {
        Recieve_id = 0x169;
        Guard_id = 107;
    }

    Client_graphic_Init();//不用一直更新，但是无法判断什么时候进入客户端所以需要轮询,可以操作手key控制
    //update_figure_flag = ADD;
    vTaskDelay(50);
    Client_graphic_Info_update();
    update_figure_flag = ADD;
    vTaskDelay(50);
    Client_aim_update();
    update_aim_flag = ADD;
    vTaskDelay(50);
    Client_supercapacitor_update();
    update_supercapacitor_flag = ADD;
    vTaskDelay(50);
    autoshoot_allow();
    auto_sope_flag = ADD;
    vTaskDelay(50);
}
void flag_fresh()
{
    if(gimbal_behaviour == GIMBAL_ZERO_FORCE)//底盘运动状况显示
    {
        chassis_judge=0;   //红
    }
    else
    {
        if     ( sup_cap.tl_flag ==1)chassis_judge=2;//绿
        else if( sup_cap.tl_flag ==2)chassis_judge=3;//橙
        else if( sup_cap.tl_flag ==0)chassis_judge=1;//粉
    }

    if(gimbal_behaviour == GIMBAL_ABSOLUTE_AUTOMA_ANGLE)//自瞄状态显示
        gimbal_auto=1;
    else gimbal_auto=0;

    if(shoot_control.fire_speed1<50&&shoot_control.fire_speed2>-50)
        firction_judge=0;//不开是红色
    else if(shoot_control.fire_speed1>3500&&shoot_control.fire_speed2<-3500)
        firction_judge=2;//正常
    else
        firction_judge=1;//其他问题

    global_supercapacitor_remain=(super_cap.Value_Cap-12)/13;
}
void Client_graphic_Init(void)
{
//	if(state_first_graphic>=5)
//	{
//		state_first_graphic = 0;
//	}
    //帧头
    tx_client_char.txFrameHeader.sof = JUDGE_FRAME_HEADER;
    tx_client_char.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(ext_client_string_t);
    tx_client_char.txFrameHeader.seq = 0;//包序号
    memcpy(CliendTxBuffer,&tx_client_char.txFrameHeader,sizeof(std_frame_header_t));
    append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

    //命令码
    tx_client_char.CmdID = ID_robot_interactive_header_data;

    //数据段头结构
    tx_client_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;//绘制几个图像
    tx_client_char.dataFrameHeader.send_ID     = RM_Referee.robot_state.robot_id;//当前ID
    tx_client_char.dataFrameHeader.receiver_ID = Recieve_id;//接受者ID

    //数据段
    Draw_char();
    memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_char.CmdID, LEN_CMD_ID+tx_client_char.txFrameHeader.data_length);//加上命令码长度2

    //帧尾
    append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_char));
    Client_Sent_String(CliendTxBuffer, sizeof(tx_client_char));
}
void Client_graphic_Info_update()//绘制标志圆圈
{
    //帧头
    tx_client_graphic_figure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
    tx_client_graphic_figure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*5;
    tx_client_graphic_figure.txFrameHeader.seq = 0;//包序号
    memcpy(CliendTxBuffer,&tx_client_graphic_figure.txFrameHeader,sizeof(std_frame_header_t));
    append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

    //命令码
    tx_client_graphic_figure.CmdID = ID_robot_interactive_header_data;

    //数据段头结构
    tx_client_graphic_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_five_graphic;
    tx_client_graphic_figure.dataFrameHeader.send_ID     = RM_Referee.robot_state.robot_id;
    tx_client_graphic_figure.dataFrameHeader.receiver_ID = Recieve_id;

    //数据段
    Draw_Figure_bool();
    memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_graphic_figure.CmdID, LEN_CMD_ID+tx_client_graphic_figure.txFrameHeader.data_length);//加上命令码长度2
    //帧尾
    append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_client_graphic_figure));
    Client_Sent_String(CliendTxBuffer, sizeof(tx_client_graphic_figure));
}
void Client_supercapacitor_update()//超级电容条绘制
{
    //帧头
    tx_supercapacitor_figure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
    tx_supercapacitor_figure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t);
    tx_supercapacitor_figure.txFrameHeader.seq = 0;//包序号
    memcpy(CliendTxBuffer,&tx_supercapacitor_figure.txFrameHeader,sizeof(std_frame_header_t));
    append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验
    //命令码
    tx_supercapacitor_figure.CmdID = ID_robot_interactive_header_data;
    //数据段头结构
    tx_supercapacitor_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_one_graphic;
    tx_supercapacitor_figure.dataFrameHeader.send_ID     =  RM_Referee.robot_state.robot_id;
    tx_supercapacitor_figure.dataFrameHeader.receiver_ID = Recieve_id;
    //数据段
    supercapacitor_figure(global_supercapacitor_remain,global_supercapacitor_point);
    memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_supercapacitor_figure.CmdID, LEN_CMD_ID+tx_supercapacitor_figure.txFrameHeader.data_length);//加上命令码长度2

    //帧尾
    append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_supercapacitor_figure));

    Client_Sent_String(CliendTxBuffer, sizeof(tx_supercapacitor_figure));
}
void guard_data()
{
    //	Figure_Guard();
    tx_guard_figure.clientData[0]=0x01;
    tx_guard_figure.clientData[1]=0x02;
    tx_guard_figure.clientData[2]=0x03;
    tx_guard_figure.clientData[3]=0x04;
    tx_guard_figure.clientData[4]=0x05;
//		Figure_Graphic(&tx_guard_figure.clientData[1],"HG2",guard_flag,CIRCLE,1,GREEN,0,0,5,   200,1080*7/12, 20,0,0);
//  	Figure_Graphic(&tx_guard_figure.clientData[2],"HG3",guard_flag,CIRCLE,1,PINK,0,0,5,    200,1080*7/12, 20,0,0);
//		Figure_Graphic(&tx_guard_figure.clientData[3],"HG4",guard_flag,CIRCLE,1,ORANGE,0,0,5,  200,1080*7/12, 20,0,0);
//  	Figure_Graphic(&tx_guard_figure.clientData[4],"HG5",guard_flag,CIRCLE,1,ORANGE,0,0,5,  200,1080*7/12, 20,0,0);
}
void guard_control()//哨兵遥控
{
    //帧头
    int8_t a[5];
    tx_guard_figure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
    tx_guard_figure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) +sizeof(a);
    tx_guard_figure.txFrameHeader.seq = 0;//包序号
    memcpy(CliendTxBuffer,&tx_guard_figure.txFrameHeader,sizeof(std_frame_header_t));
    append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

    //命令码
    tx_guard_figure.CmdID = ID_robot_interactive_header_data;

    //数据段头结构
    tx_guard_figure.dataFrameHeader.data_cmd_id = Guard_shoot;
    tx_guard_figure.dataFrameHeader.send_ID     = RM_Referee.robot_state.robot_id;
    tx_guard_figure.dataFrameHeader.receiver_ID = Guard_id;

    //数据段
    guard_data();
    memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_guard_figure.CmdID, LEN_CMD_ID+tx_guard_figure.txFrameHeader.data_length);//加上命令码长度2
    //帧尾
    append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_guard_figure));
    Client_Sent_String(CliendTxBuffer, sizeof(tx_guard_figure));
}
void Client_aim_update()//准星
{
    //帧头
    tx_aim_figure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
    tx_aim_figure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*2;
    tx_aim_figure.txFrameHeader.seq = 0;//包序号
    memcpy(CliendTxBuffer,&tx_aim_figure.txFrameHeader,sizeof(std_frame_header_t));
    append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

    //命令码
    tx_aim_figure.CmdID = ID_robot_interactive_header_data;

    //数据段头结构
    tx_aim_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
    tx_aim_figure.dataFrameHeader.send_ID     = RM_Referee.robot_state.robot_id;
    tx_aim_figure.dataFrameHeader.receiver_ID = Recieve_id;

    //数据段
    sight_bead_figrue(global_sight_bead_x,global_sight_bead_y);
    memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_aim_figure.CmdID, LEN_CMD_ID+tx_aim_figure.txFrameHeader.data_length);//加上命令码长度2

    //帧尾
    append_CRC16_check_sum(CliendTxBuffer,sizeof(tx_aim_figure));

    Client_Sent_String(CliendTxBuffer, sizeof(tx_aim_figure));
}
static void aim_lowshort_stem(uint32_t division_value,uint32_t line_length)//自瞄框
{
    if(Pawn_mode==2||Pawn_mode==3)
    {
        if(PC_temp[1]==0x31&&gimbal_auto==1)
            Figure_Graphic(&low_aim_shortfigure_3.clientData[0],"AS1",auto_sope_flag,CIRCLE,3,GREEN,0,0,5, AIM_X,  AIM_Y,250,  0, 0);
        else if (PC_temp[1]==0x30&&gimbal_auto==1)
            Figure_Graphic(&low_aim_shortfigure_3.clientData[0],"AS1",auto_sope_flag,CIRCLE,3,ORANGE,0,0,5, AIM_X,  AIM_Y,250,  0, 0);
        else if (PC_temp[1]==0x31&&gimbal_auto==0)
            Figure_Graphic(&low_aim_shortfigure_3.clientData[0],"AS1",auto_sope_flag,CIRCLE,3,CYAN_BLUE,0,0,5, AIM_X,  AIM_Y,250,  0, 0);
        else
            Figure_Graphic(&low_aim_shortfigure_3.clientData[0],"AS1",auto_sope_flag,CIRCLE,3,BLACK,0,0,5, AIM_X,  AIM_Y,250,  0, 0);

    }
    else
    {
        if(PC_temp[1]==0x31&&gimbal_auto==1)
            Figure_Graphic(&low_aim_shortfigure_3.clientData[0],"AS1",auto_sope_flag,RECTANGLE,3,GREEN,0,0,5, AIM_X+230,  AIM_Y+230,0,  AIM_X-230, AIM_Y-230);
        else if (PC_temp[1]==0x30&&gimbal_auto==1)
            Figure_Graphic(&low_aim_shortfigure_3.clientData[0],"AS1",auto_sope_flag,RECTANGLE,3,ORANGE,0,0,5, AIM_X+230,  AIM_Y+230,0,  AIM_X-230, AIM_Y-230);
        else if (PC_temp[1]==0x31&&gimbal_auto==0)
            Figure_Graphic(&low_aim_shortfigure_3.clientData[0],"AS1",auto_sope_flag,RECTANGLE,3,CYAN_BLUE,0,0,5, AIM_X+230,  AIM_Y+230,0,  AIM_X-230, AIM_Y-230);
        else
            Figure_Graphic(&low_aim_shortfigure_3.clientData[0],"AS1",auto_sope_flag,RECTANGLE,3,BLACK,0,0,5, AIM_X+230,  AIM_Y+230,0,  AIM_X-230, AIM_Y-230);
    }
}
void autoshoot_allow()
{
    //帧头
    low_aim_shortfigure_3.txFrameHeader.sof = JUDGE_FRAME_HEADER;
    low_aim_shortfigure_3.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*2;
    low_aim_shortfigure_3.txFrameHeader.seq = 0;//包序号
    memcpy(CliendTxBuffer,&low_aim_shortfigure_3.txFrameHeader,sizeof(std_frame_header_t));
    append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

    //命令码
    low_aim_shortfigure_3.CmdID = ID_robot_interactive_header_data;

    //数据段头结构
    low_aim_shortfigure_3.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
    low_aim_shortfigure_3.dataFrameHeader.send_ID     = RM_Referee.robot_state.robot_id;
    low_aim_shortfigure_3.dataFrameHeader.receiver_ID =Recieve_id;
    //数据段
//	aim_lowshort_stem(division_value,10);
    aim_lowshort_stem(10,10);
    memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&low_aim_shortfigure_3.CmdID, LEN_CMD_ID+low_aim_shortfigure_3.txFrameHeader.data_length);//加上命令码长度2

    //帧尾
    append_CRC16_check_sum(CliendTxBuffer,sizeof(low_aim_shortfigure_3));

    Client_Sent_String(CliendTxBuffer, sizeof(low_aim_shortfigure_3));
}
void _high_aim_(void)
{
    //帧头
    high_aim_figure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
    high_aim_figure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*7;
    high_aim_figure.txFrameHeader.seq = 0;//包序号
    memcpy(CliendTxBuffer,&high_aim_figure.txFrameHeader,sizeof(std_frame_header_t));
    append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

    //命令码
    high_aim_figure.CmdID = ID_robot_interactive_header_data;

    //数据段头结构
    high_aim_figure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
    high_aim_figure.dataFrameHeader.send_ID     = RM_Referee.robot_state.robot_id;;
    high_aim_figure.dataFrameHeader.receiver_ID = Recieve_id;

    //数据段
//		aim_1(division_value,10);
    aim_1(10,10);
    memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&high_aim_figure.CmdID, LEN_CMD_ID+high_aim_figure.txFrameHeader.data_length);//加上命令码长度2

    //帧尾
    append_CRC16_check_sum(CliendTxBuffer,sizeof(high_aim_figure));

    Client_Sent_String(CliendTxBuffer, sizeof(high_aim_figure));
}
static void aim_1(uint32_t division_value,uint32_t line_length)//准心上半部分的宽度"AH"--aim_high
{
    Figure_Graphic(&high_aim_figure.clientData[0],"AH1",ADD,LINE,3,YELLOW,0,0,3,  AIM_X-line_length, AIM_Y+30,0,  AIM_X+line_length, AIM_Y+30);                 //graphic_Remove
    Figure_Graphic(&high_aim_figure.clientData[1],"AH2",ADD,LINE,3,YELLOW,0,0,3,  AIM_X-line_length, AIM_Y+30+division_value,0,  AIM_X+line_length, AIM_Y+30+division_value  );
    Figure_Graphic(&high_aim_figure.clientData[2],"AH3",ADD,LINE,3,YELLOW,0,0,3,  AIM_X-line_length, AIM_Y+30+division_value*2,0,  AIM_X+line_length, AIM_Y+30+division_value*2);
    Figure_Graphic(&high_aim_figure.clientData[3],"AH4",ADD,LINE,3,YELLOW,0,0,3,  AIM_X-line_length, AIM_Y+30+division_value*3,0,  AIM_X+line_length, AIM_Y+30+division_value*3);
    Figure_Graphic(&high_aim_figure.clientData[4],"AH5",ADD,LINE,3,YELLOW,0,0,3,  AIM_X-line_length, AIM_Y+30+division_value*4,0,  AIM_X+line_length, AIM_Y+30+division_value*4);
    Figure_Graphic(&high_aim_figure.clientData[5],"AH6",ADD,LINE,3,YELLOW,0,0,3,  AIM_X-line_length, AIM_Y+30+division_value*5,0,  AIM_X+line_length, AIM_Y+30+division_value*5);
    Figure_Graphic(&high_aim_figure.clientData[6],"AH7",ADD,LINE,3,YELLOW,0,0,3,  AIM_X-line_length, AIM_Y+30+division_value*6,0,  AIM_X+line_length, AIM_Y+30+division_value*6);
}
//图层四
static void aim_lowlong(uint32_t division_value,uint32_t line_length)//准心上半部分的宽度"AM"--aim_low_Long,"AS"--aim_stem
{
    int8_t X=35,Y=95;
    uint32_t lightline_position_x=0,lightline_position_y=0;
    if(barrel_choice_flag==1) {
        X=35;
        Y=70;
    }
    else if(barrel_choice_flag==2) {
        X=35;
        Y=80;
    }
    Figure_Graphic(&low_aim_longfigure.clientData[0],"AL0",lowlong_flag,LINE,4,PINK,0,0,1,		AIM_X+X, AIM_Y-Y+30,0,AIM_X+X, AIM_Y-Y-30 );
    Figure_Graphic(&low_aim_longfigure.clientData[1],"AL1",lowlong_flag,LINE,4,PINK,0,0,1,		AIM_X-30+X, AIM_Y-Y,0,AIM_X+30+X, AIM_Y-Y );
    Figure_Graphic(&low_aim_longfigure.clientData[2],"AL2",lowlong_flag,CIRCLE,4,WHITE,0,0,2,		AIM_X+X, AIM_Y-Y,30,0, 0 );

    lightline_position_x=AIM_X+500+(int32_t)(arm_cos_f32(-gimbal_control.gimbal_yaw_motor.relative_angle+PI*0.5f)*70);
    lightline_position_y=AIM_Y+250+(int32_t)(arm_sin_f32(-gimbal_control.gimbal_yaw_motor.relative_angle+PI*0.5f)*70);
    Figure_Graphic(&low_aim_longfigure.clientData[3],"AL3",lowlong_flag,CIRCLE,4,YELLOW,0,0,4,lightline_position_x, lightline_position_y,30,0, 0 );
    Figure_Graphic(&low_aim_longfigure.clientData[4],"AL4",lowlong_flag,RECTANGLE,4,WHITE,0,0,5,AIM_X+520, AIM_Y+350,0,AIM_X+480, AIM_Y+250 );
}
void _lowlong_aim_()
{
    //帧头
    low_aim_longfigure.txFrameHeader.sof = JUDGE_FRAME_HEADER;
    low_aim_longfigure.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*5;
    low_aim_longfigure.txFrameHeader.seq = 0;//包序号
    memcpy(CliendTxBuffer,&low_aim_longfigure.txFrameHeader,sizeof(std_frame_header_t));
    append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

    //命令码
    low_aim_longfigure.CmdID = ID_robot_interactive_header_data;

    //数据段头结构
    low_aim_longfigure.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_five_graphic;
    low_aim_longfigure.dataFrameHeader.send_ID     = RM_Referee.robot_state.robot_id;
    low_aim_longfigure.dataFrameHeader.receiver_ID =Recieve_id;

    //数据段
    //aim_lowlong(division_value,10);
    aim_lowlong(10,10);
    memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&low_aim_longfigure.CmdID, LEN_CMD_ID+low_aim_longfigure.txFrameHeader.data_length);//加上命令码长度2

    //帧尾
    append_CRC16_check_sum(CliendTxBuffer,sizeof(low_aim_longfigure));
    Client_Sent_String(CliendTxBuffer, sizeof(low_aim_longfigure));
}
void _lowshort_aim_2()
{
    //帧头
    low_aim_shortfigure_1.txFrameHeader.sof = JUDGE_FRAME_HEADER;
    low_aim_shortfigure_1.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*7;
    low_aim_shortfigure_1.txFrameHeader.seq = 0;//包序号
    memcpy(CliendTxBuffer,&low_aim_shortfigure_1.txFrameHeader,sizeof(std_frame_header_t));
    append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

    //命令码
    low_aim_shortfigure_1.CmdID = ID_robot_interactive_header_data;

    //数据段头结构
    low_aim_shortfigure_1.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
    low_aim_shortfigure_1.dataFrameHeader.send_ID     = RM_Referee.robot_state.robot_id;
    low_aim_shortfigure_1.dataFrameHeader.receiver_ID = Recieve_id;

    //数据段
    //aim_lowshort_2(division_value,10);
    aim_lowshort_2(10,10);
    memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&low_aim_shortfigure_1.CmdID, LEN_CMD_ID+low_aim_shortfigure_1.txFrameHeader.data_length);//加上命令码长度2

    //帧尾
    append_CRC16_check_sum(CliendTxBuffer,sizeof(low_aim_shortfigure_1));

    Client_Sent_String(CliendTxBuffer, sizeof(low_aim_shortfigure_1));
}
void _lowshort_aim_3()
{
    //帧头
    low_aim_shortfigure_2.txFrameHeader.sof = JUDGE_FRAME_HEADER;
    low_aim_shortfigure_2.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t)*2;
    low_aim_shortfigure_2.txFrameHeader.seq = 0;//包序号
    memcpy(CliendTxBuffer,&low_aim_shortfigure_2.txFrameHeader,sizeof(std_frame_header_t));
    append_CRC8_check_sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

    //命令码
    low_aim_shortfigure_2.CmdID = ID_robot_interactive_header_data;

    //数据段头结构
    low_aim_shortfigure_2.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
    low_aim_shortfigure_2.dataFrameHeader.send_ID     = RM_Referee.robot_state.robot_id;
    low_aim_shortfigure_2.dataFrameHeader.receiver_ID = Recieve_id;

    //数据段
    aim_lowshort_3(division_value,10);

    memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&low_aim_shortfigure_2.CmdID, LEN_CMD_ID+low_aim_shortfigure_2.txFrameHeader.data_length);//加上命令码长度2

    //帧尾
    append_CRC16_check_sum(CliendTxBuffer,sizeof(low_aim_shortfigure_2));

    Client_Sent_String(CliendTxBuffer, sizeof(low_aim_shortfigure_2));
}
static void aim_lowshort_3(uint32_t division_value,uint32_t line_length)//准心上半部分的宽度"AM"--aim_low_middle
{
    {   Figure_Graphic(&low_aim_shortfigure_2.clientData[0],"AM1",ADD,LINE,3,WHITE,0,0,2,  AIM_X-line_length-300,AIM_Y-200,0, AIM_X-100,AIM_Y+100 );//graphic_Remove
        Figure_Graphic(&low_aim_shortfigure_2.clientData[1],"AM2",ADD,LINE,3,WHITE,0,0,2,  AIM_X-line_length+300,AIM_Y-200,0, AIM_X+100,AIM_Y+100 );
    }
}

static void aim_lowshort_2(uint32_t division_value,uint32_t line_length)//准心上半部分的宽度"AL"--aim_low
{
    Figure_Graphic(&low_aim_shortfigure_1.clientData[0],"AL1",ADD,LINE,3,ORANGE		,0,0,2,  AIM_X-line_length,	AIM_Y-30,0,  AIM_X+line_length,	AIM_Y-30);                 //graphic_Remove
    Figure_Graphic(&low_aim_shortfigure_1.clientData[1],"AL2",ADD,LINE,3,ORANGE		,0,0,2,  AIM_X-line_length,	AIM_Y-30-division_value,0,  AIM_X+line_length,	AIM_Y-30-division_value  );
    Figure_Graphic(&low_aim_shortfigure_1.clientData[2],"AL3",ADD,LINE,3,ORANGE		,0,0,2,  AIM_X-line_length,	AIM_Y-30-division_value*2,0,  AIM_X+line_length,	AIM_Y-30-division_value*2);
    Figure_Graphic(&low_aim_shortfigure_1.clientData[3],"AL4",ADD,LINE,3,ORANGE		,0,0,2,  AIM_X-line_length,	AIM_Y-30-division_value*4,0,  AIM_X+line_length,	AIM_Y-30-division_value*4);
    Figure_Graphic(&low_aim_shortfigure_1.clientData[4],"AL5",ADD,LINE,3,ORANGE		,0,0,2,  AIM_X-line_length,	AIM_Y-30-division_value*5,0,  AIM_X+line_length,	AIM_Y-30-division_value*5);
    Figure_Graphic(&low_aim_shortfigure_1.clientData[5],"AL6",ADD,LINE,3,CYAN_BLUE,0,0,2,  AIM_X-line_length,	AIM_Y-30-division_value*6,0,  AIM_X+line_length,	AIM_Y-30-division_value*6);
    Figure_Graphic(&low_aim_shortfigure_1.clientData[6],"AL7",ADD,LINE,3,GREEN		,0,0,2,  AIM_X-line_length,	AIM_Y-30-division_value*8,0,  AIM_X+line_length,	AIM_Y-30-division_value*8);
}
void Char_Graphic(ext_client_string_t* graphic,//最终要发出去的数组中的数据段内容
                  const char* name,//图像名称
                  uint32_t operate_tpye,//图层操作
                  uint32_t layer,//图层
                  uint32_t color,//颜色
                  uint32_t size,//字体大小
                  uint32_t length,//字符长度
                  uint32_t width,//字宽
                  uint32_t start_x,//起始位置x坐标
                  uint32_t start_y,//起始位置y坐标
                  const char *character)//外部放入的数组，内容是要显示的内容
{
    graphic_data_struct_t *data_struct = &graphic->grapic_data_struct;
    for(char i=0; i<3; i++)
        data_struct->graphic_name[i] = name[i];	//字符索引
    data_struct->operate_tpye = operate_tpye; //图层操作
    data_struct->graphic_tpye = CHAR;         //Char型
    data_struct->layer = layer;//都在第零层
    data_struct->color = color;//都是白色
    data_struct->start_angle = size;
    data_struct->end_angle = length;
    data_struct->width = width;
    data_struct->start_x = start_x;
    data_struct->start_y = start_y;

    data_struct->radius = 0;
    data_struct->end_x = 0;
    data_struct->end_y = 0;

    memcpy(graphic->data,empty_line,28);
    memcpy(graphic->data,character,length);
}


void Figure_Graphic(graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
                    const char* name,//图像名称
                    uint32_t operate_tpye,//图层操作
                    uint32_t graphic_tpye,//绘制什么图像
                    uint32_t layer,//图层选择
                    uint32_t color,//颜色选择
                    uint32_t start_angle,//开始角度
                    uint32_t end_angle,//结束角度
                    uint32_t width,//宽度
                    uint32_t start_x,//起始位置x坐标
                    uint32_t start_y,//起始位置y坐标
                    uint32_t radius,//半径
                    uint32_t end_x,//结束位置x坐标
                    uint32_t end_y)//结束位置y坐标
{
    for(char i=0; i<3; i++)
        graphic->graphic_name[i] = name[i];	//字符索引
    graphic->operate_tpye = operate_tpye; //图层操作
    graphic->graphic_tpye = graphic_tpye;         //Char型
    graphic->layer        = layer;//都在第一层
    graphic->color        = color;//变色
    graphic->start_angle  = start_angle;
    graphic->end_angle    = end_angle;
    graphic->width        = width;
    graphic->start_x      = start_x;
    graphic->start_y      = start_y;
    graphic->radius = radius;
    graphic->end_x  = end_x;
    graphic->end_y  = end_y;
}
void Figure_Guard()//结束位置y坐标
{

}
static void Draw_Figure_bool()
{
    auto_fire();
    chassis();
    firction();
    //Switch();
    Mode();
}
static void Draw_char()
{
    state_first_graphic=state_first_graphic+1;
    if(state_first_graphic == 3)//不知道什么时候进入客户端所以要不断更新
    {   //autofire自瞄状态和是否可以射击
        Char_Graphic(&tx_client_char.clientData,"CL1",char_flag,0,WHITE,15,strlen(first_line),1,(500),(1080*9/12),first_line);//x1920/18
        //	state_first_graphic = 1;
    }
    else if(state_first_graphic == 7)
    {   //chassis底盘状态和小陀螺状态
        Char_Graphic(&tx_client_char.clientData,"CL2",char_flag,0,WHITE,15,strlen(second_line),1,(500),(1080*8/12),second_line);
        //	state_first_graphic = 2;
    }
    else if(state_first_graphic == 10)
    {   //firction发射机构状态
        Char_Graphic(&tx_client_char.clientData,"CL3",char_flag,0,WHITE,15,strlen(third_line),1,(500),(1080*7/12),third_line);
        //	state_first_graphic = 3;
    }
    else if(state_first_graphic == 13)
    {   //弹仓盖
        Char_Graphic(&tx_client_char.clientData,"CL4",char_flag,0,WHITE,15,strlen(fourth_line),1,(600),(1080*5.5f/12),fourth_line);
        //state_first_graphic = 4;
    }
    else
    {   //当前射速
        if(change_flag==0)
            Char_Graphic(&tx_client_char.clientData,"CL5",char_flag,0,GREEN,20,strlen(fifth_line),2,(800),(1080*9/12),fifth_line);
        else Char_Graphic(&tx_client_char.clientData,"CL5",char_flag,0,GREEN,20,strlen(sixth_line),2,(800),(1080*9/12),sixth_line);
        //state_first_graphic = 5;
    }
    if(state_first_graphic>15)state_first_graphic=0;
//	else if(state_first_graphic == 5)
//	{//Defense防守战术建议
//		//Char_Graphic(&tx_client_char.clientData,"CL6",char_flag,0,WHITE,10,strlen(sixth_line),1,(1920-150),(1080*7/12),sixth_line);
//		state_first_graphic = 6;
//	}
//	else if(state_first_graphic == 6)
//	{//Attack进攻战术建议
//		Char_Graphic(&tx_client_char.clientData,"CL7",char_flag,0,WHITE,10,strlen(seventh_line),1,(1920-150),(1080*5/12),seventh_line);
//		state_first_graphic = 7;
//	}
//	else if(state_first_graphic == 7)
//	{//offline module离线模块信息
//		Char_Graphic(&tx_client_char.clientData,"CL8",char_flag,0,WHITE,10,strlen(eighth_line),1,(1920-150),(1080*3/12),eighth_line);
//		state_first_graphic = 8;
    //}

}
static void supercapacitor_figure(fp32 remain_energy,uint32_t turning_point)//剩余超级电容（单位百分比），低于某百分比变红色
{
    uint32_t remaining = 0,remaining1=0;//强制转换
    remaining = (uint32_t)(remain_energy*100);
    remaining1 = (uint32_t)(remain_energy*640);
    if(remaining >= turning_point)//直线长度为3
        Figure_Graphic(&tx_supercapacitor_figure.clientData,"SR1",update_supercapacitor_flag,LINE,3,GREEN,  0,0,30,(1920-640-remaining1),30,0,  (1920-640),30);
    else if(remaining < turning_point)
        Figure_Graphic(&tx_supercapacitor_figure.clientData,"SR1",update_supercapacitor_flag,LINE,3,FUCHSIA,0,0,30,(1920-640-remaining1),30,0,  (1920-640),30);
}
static void sight_bead_figrue(uint32_t x,uint32_t y)//可移动准心，请强制转换成uint32_t1920*1080有部分地区无法画出
{
    Figure_Graphic(&tx_aim_figure.clientData[0],"GR1",update_aim_flag,LINE,2,WHITE,0,0,3,  AIM_X,AIM_Y+40,0,  AIM_X,AIM_Y-200);  //graphic_Remove
    Figure_Graphic(&tx_aim_figure.clientData[1],"GR2",update_aim_flag,LINE,2,WHITE,0,0,3,  AIM_X-50,AIM_Y	,0,  AIM_X+50,AIM_Y);
}
static void auto_fire(void)
{   //不开启自瞄且视野内无目标为红色
    if(mc_flag==0/*&&target_status==0*/)
    {
        Figure_Graphic(&tx_client_graphic_figure.clientData[0],"GL1",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,10,  600,1080*9/12, 25,0,0);
    }
    else
    {
        Figure_Graphic(&tx_client_graphic_figure.clientData[0],"GL1",update_figure_flag,CIRCLE,1,GREEN,0,0,10,  600,1080*9/12, 25,0,0);
    }
}
static void chassis(void)
{   //底盘失能为红色
    if(chassis_judge==1)
    {   //底盘处于正常跟随模式为绿色
        Figure_Graphic(&tx_client_graphic_figure.clientData[1],"GL2",update_figure_flag,CIRCLE,1,GREEN,0,0,10, 600,1080*8/12, 25,0,0);
    }
    else if(chassis_judge==2)
    {   //底盘处于小陀螺模式为橙色
        Figure_Graphic(&tx_client_graphic_figure.clientData[1],"GL2",update_figure_flag,CIRCLE,1,ORANGE,0,0,10,600,1080*8/12, 25,0,0);
    }
    else if(chassis_judge==3)
    {   //底盘处于于扭腰为blue
        Figure_Graphic(&tx_client_graphic_figure.clientData[1],"GL2",update_figure_flag,CIRCLE,1,CYAN_BLUE,0,0,10,600,1080*8/12, 25,0,0);
    }
    else
    {
        Figure_Graphic(&tx_client_graphic_figure.clientData[1],"GL2",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,10,600,1080*8/12, 25,0,0);
    }
}
static void firction(void)
{   //摩擦轮两个都不开为红色
    if(firction_judge==0)
    {
        Figure_Graphic(&tx_client_graphic_figure.clientData[2],"GL3",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,10,600,1080*7/12, 25,0,0);
    }
//左开右不开为橙色
    else if(firction_judge==1)
    {
        Figure_Graphic(&tx_client_graphic_figure.clientData[2],"GL3",update_figure_flag,CIRCLE,1,ORANGE,0,0,10,  600,1080*7/12, 25,0,0);
    }
//左右都开为绿色
    else
    {
        Figure_Graphic(&tx_client_graphic_figure.clientData[2],"GL3",update_figure_flag,CIRCLE,1,GREEN,0,0,10,  600,1080*7/12, 25,0,0);
    }
}
static void Switch(void)
{   //限位没有被触发为红色（子弹没有上膛）
    if(switch_judge==0)
    {
        Figure_Graphic(&tx_client_graphic_figure.clientData[3],"GL4",update_figure_flag,CIRCLE,1,FUCHSIA,0,0,5,  600,1080*6/12, 20,0,0);
    }
    else if(switch_judge==1)
    {   //限位触发为绿色（子弹上膛完成）
        Figure_Graphic(&tx_client_graphic_figure.clientData[3],"GL4",update_figure_flag,CIRCLE,1,GREEN,0,0,5,  600,1080*6/12, 20,0,0);
    }

}
static void Mode(void)
{
    if(Pawn_mode==1)
    {
        Figure_Graphic(&tx_client_graphic_figure.clientData[3],"GL4",update_figure_flag,CIRCLE,1,CYAN_BLUE,0,0,10,  600,1080*6/12, 25,0,0);
    }
    else if(Pawn_mode==2)
    {
        Figure_Graphic(&tx_client_graphic_figure.clientData[3],"GL4",update_figure_flag,CIRCLE,1,PINK,0,0,10,  600,1080*6/12, 25,0,0);
    }
    else if(Pawn_mode==3)
    {
        Figure_Graphic(&tx_client_graphic_figure.clientData[3],"GL4",update_figure_flag,CIRCLE,1,WHITE,0,0,10,  600,1080*6/12, 25,0,0);
    }
    else if(Pawn_mode==4)
    {
        Figure_Graphic(&tx_client_graphic_figure.clientData[3],"GL4",update_figure_flag,CIRCLE,1,YELLOW,0,0,10,  600,1080*6/12, 25,0,0);
    }
    else
    {
        Figure_Graphic(&tx_client_graphic_figure.clientData[3],"GL4",update_figure_flag,CIRCLE,1,GREEN,0,0,10,  600,1080*6/12, 25,0,0);
    }
}
void Indicator_light(void)
{
}
