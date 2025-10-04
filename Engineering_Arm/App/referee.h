#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;
typedef __packed struct //0001
{
    uint8_t game_progress;
    uint16_t stage_remain_time; //剩余时间（单位：s）
		uint64_t SyncTimeStamp;
} ext_game_state_t;

typedef __packed struct //0002
{
    uint8_t winner;
} ext_game_result_t;
typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
	  uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
	  uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;
typedef __packed struct //0101
{
    uint32_t event_type;		//各种增益
} ext_event_data_t;

typedef __packed struct //0x0102
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


typedef __packed struct //0x0103
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct //0x104
{
    uint8_t level;
    uint8_t foul_robot_id;
	  uint8_t count;
} ext_referee_warning_t;
typedef __packed struct //0x0201
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_id1_17mm_cooling_rate;
	uint16_t shooter_id1_17mm_cooling_limit;
//	uint16_t shooter_id1_17mm_speed_limit;
//	uint16_t shooter_id2_17mm_cooling_rate;
//	uint16_t shooter_id2_17mm_cooling_limit;
//	uint16_t shooter_id2_17mm_speed_limit;
//	uint16_t shooter_id1_42mm_cooling_rate;
//	uint16_t shooter_id1_42mm_cooling_limit;
//	uint16_t shooter_id1_42mm_speed_limit;
	uint16_t chassis_power_limit;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

typedef __packed struct //0x0202
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_heat0;
    uint16_t shooter_heat1;
	  uint16_t shooter_42mm_barrel_heat;
} ext_power_heat_data_t;

typedef __packed struct //0x0203
{
    float x;
    float y;
//    float z;
    float yaw;
} ext_game_robot_pos_t;

typedef __packed struct //0x0204
{
    uint8_t recovery_buff;
		uint8_t cooling_buff;
		uint8_t defence_buff;
		uint8_t vulnerability_buff;
		uint16_t attack_buff;
} ext_buff_musk_t;

typedef __packed struct //0x0205
{
    uint8_t energy_point;
    uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct //0x0206
{
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct //0x0207
{  
	uint8_t bullet_type;
  uint8_t shooter_id;
  uint8_t bullet_freq;
  float bullet_speed;
} ext_shoot_data_t;
typedef __packed struct //0x208
{
  uint16_t bullet_remaining_num_17mm;
  uint16_t bullet_remaining_num_42mm;	
	uint16_t coin_remaining_num;
} ext_bullet_remaining_t;
typedef __packed struct //0x0301
{
    uint16_t send_ID;
    uint16_t receiver_ID;
    uint16_t data_cmd_id;
    uint16_t data_len;
    uint8_t *data;
} ext_student_interactive_data_t;

typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;


typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;

typedef struct
{
	frame_header_struct_t referee_receive_header;
	frame_header_struct_t referee_send_header;

	ext_game_state_t    game_state;
	ext_game_result_t   game_result;
	ext_game_robot_HP_t game_robot_HP_t;

	ext_event_data_t                field_event;               //
	ext_supply_projectile_action_t  supply_projectile_action_t;
	ext_supply_projectile_booking_t supply_projectile_booking_t;
	ext_referee_warning_t           referee_warning_t;         //裁判系统警告

	ext_game_robot_state_t          robot_state;
	ext_power_heat_data_t           power_heat_data_t;
	ext_game_robot_pos_t            game_robot_pos_t;
	ext_buff_musk_t                 buff_musk_t;               //buff效果
	aerial_robot_energy_t           robot_energy_t;            //机器人功率
	ext_robot_hurt_t                robot_hurt_t;              //机器人受伤状况
	ext_shoot_data_t                shoot_data_t;              //射击数据
	ext_bullet_remaining_t          bullet_remaining_t;        //剩余子弹
	ext_student_interactive_data_t  student_interactive_data_t;//学生交互
}RM_Referee_system_t;

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer,fp32 *limit);

extern uint8_t get_robot_id(void);

extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0);
extern void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1);
extern void get_shoot_speed_limit(uint16_t *speedlimit);
extern void get_shoot_heat0_limit_and_heat_cooling_rate(uint16_t *heat0_limit, uint16_t *rate);

//extern ext_game_state_t    game_state;
//extern ext_game_result_t   game_result;
//extern ext_game_robot_HP_t game_robot_HP_t;

//extern ext_event_data_t                field_event;               //
//extern ext_supply_projectile_action_t  supply_projectile_action_t;
//extern ext_supply_projectile_booking_t supply_projectile_booking_t;
//extern ext_referee_warning_t           referee_warning_t;         //裁判系统警告
extern RM_Referee_system_t RM_Referee;

//extern ext_game_robot_state_t          robot_state;
//extern ext_power_heat_data_t           power_heat_data_t;
//extern ext_game_robot_pos_t            game_robot_pos_t;
//extern ext_buff_musk_t                 buff_musk_t;               //buff效果
//extern aerial_robot_energy_t           robot_energy_t;            //机器人功率
//extern ext_robot_hurt_t                robot_hurt_t;              //机器人受伤状况
//extern ext_shoot_data_t                shoot_data_t;              //射击数据
//extern ext_bullet_remaining_t          bullet_remaining_t;        //剩余子弹
//extern ext_student_interactive_data_t  student_interactive_data_t;//学生交互
#endif
