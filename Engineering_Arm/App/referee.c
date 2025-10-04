#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"


//frame_header_struct_t referee_receive_header;
//frame_header_struct_t referee_send_header;

//ext_game_state_t    game_state;
//ext_game_result_t   game_result;
//ext_game_robot_HP_t game_robot_HP_t;

//ext_event_data_t                field_event;               //
//ext_supply_projectile_action_t  supply_projectile_action_t;
//ext_supply_projectile_booking_t supply_projectile_booking_t;
//ext_referee_warning_t           referee_warning_t;         //裁判系统警告


//ext_game_robot_state_t          robot_state;
//ext_power_heat_data_t           power_heat_data_t;
//ext_game_robot_pos_t            game_robot_pos_t;
//ext_buff_musk_t                 buff_musk_t;               //buff效果
//aerial_robot_energy_t           robot_energy_t;            //机器人功率
//ext_robot_hurt_t                robot_hurt_t;              //机器人受伤状况
//ext_shoot_data_t                shoot_data_t;              //射击数据
//ext_bullet_remaining_t          bullet_remaining_t;        //剩余子弹
//ext_student_interactive_data_t  student_interactive_data_t;//学生交互
RM_Referee_system_t RM_Referee;

void init_referee_struct_data(void)
{
    memset(&RM_Referee.referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&RM_Referee.referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&RM_Referee.game_state, 0, sizeof(ext_game_state_t));
    memset(&RM_Referee.game_result, 0, sizeof(ext_game_result_t));
    memset(&RM_Referee.game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));


    memset(&RM_Referee.field_event, 0, sizeof(ext_event_data_t));
    memset(&RM_Referee.supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&RM_Referee.supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&RM_Referee.referee_warning_t, 0, sizeof(ext_referee_warning_t));


    memset(&RM_Referee.robot_state, 0, sizeof(ext_game_robot_state_t));
    memset(&RM_Referee.power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&RM_Referee.game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&RM_Referee.buff_musk_t, 0, sizeof(ext_buff_musk_t));
    memset(&RM_Referee.robot_energy_t, 0, sizeof(aerial_robot_energy_t));
    memset(&RM_Referee.robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&RM_Referee.shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&RM_Referee.bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));

    memset(&RM_Referee.student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));
}
  uint16_t cmd_id = 0;
void referee_data_solve(uint8_t *frame)
{
  

    uint8_t index = 0;

    memcpy(&RM_Referee.referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&RM_Referee.game_state, frame + index, sizeof(ext_game_state_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&RM_Referee.game_result, frame + index, sizeof(ext_game_result_t));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&RM_Referee.game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;


        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&RM_Referee.field_event, frame + index, sizeof(ext_event_data_t));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&RM_Referee.supply_projectile_action_t, frame + index, sizeof(ext_supply_projectile_action_t));
        }
        break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
        {
            memcpy(&RM_Referee.supply_projectile_booking_t, frame + index, sizeof(RM_Referee.supply_projectile_booking_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&RM_Referee.referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&RM_Referee.robot_state, frame + index, sizeof(ext_game_robot_state_t));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&RM_Referee.power_heat_data_t, frame + index, sizeof(ext_power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&RM_Referee.game_robot_pos_t, frame + index, sizeof(ext_game_robot_pos_t));
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&RM_Referee.buff_musk_t, frame + index, sizeof(ext_buff_musk_t));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&RM_Referee.robot_energy_t, frame + index, sizeof(aerial_robot_energy_t));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&RM_Referee.robot_hurt_t, frame + index, sizeof(ext_robot_hurt_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&RM_Referee.shoot_data_t, frame + index, sizeof(ext_shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&RM_Referee.bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
        }
        break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&RM_Referee.student_interactive_data_t, frame + index, sizeof(ext_student_interactive_data_t));
        }
        break;
        default:
        {
            break;
        }
    }
}

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer,fp32 *limit)
{
    *power = RM_Referee.power_heat_data_t.chassis_power;
    *buffer =RM_Referee.power_heat_data_t.chassis_power_buffer;
    *limit = RM_Referee.robot_state.      chassis_power_limit;
}


uint8_t get_robot_id(void)
{
    return RM_Referee.robot_state.robot_id;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    *heat0_limit = RM_Referee.robot_state.shooter_id1_17mm_cooling_limit;
    *heat0 = RM_Referee.power_heat_data_t.shooter_heat0;
}

void get_shoot_heat0_limit_and_heat_cooling_rate(uint16_t *heat0_limit, uint16_t *rate)
{
    *heat0_limit =RM_Referee. robot_state.shooter_id1_17mm_cooling_limit;
    *rate =RM_Referee.robot_state.shooter_id1_17mm_cooling_rate;
}

//void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
//{
//    *heat1_limit = RM_Referee.robot_state.shooter_id2_17mm_cooling_limit;
//    *heat1 = RM_Referee.power_heat_data_t.shooter_heat1;
//}
//void get_shoot_speed_limit(uint16_t *speedlimit)
//{
//	*speedlimit = RM_Referee.robot_state.shooter_id1_17mm_speed_limit;
//}

