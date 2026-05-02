#include "UC_Referee.h"
#include "string.h"
#include "stdio.h"
#include "UC_Protocol.h"

/*
 * 裁判系统数据缓存与解析实现。
 *
 * 工作方式很简单:
 * 1. 上层拆包模块先完成 SOF/长度/CRC 校验
 * 2. 本文件按 cmd_id 解析 data 段
 * 3. 每种数据段都覆盖写入一个全局镜像结构体
 * 4. 其他业务模块通过 getter 或 extern 访问最近一次有效数据
 */

/* 最近一次接收/发送协议帧的帧头镜像。 */
frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

/* 0x0001~0x0003: 基础比赛状态 */
ext_game_status_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

/* 0x0101~0x0105: 场地事件、判罚和飞镖状态 */
ext_event_data_t field_event;
ext_referee_warning_t referee_warning_t;
ext_dart_remaining_time_t dart_remaining_time_t;

/* 0x0201~0x020E: 机器人本体状态与协同状态 */
ext_game_robot_status_t robot_state;
ext_power_heat_data_t power_heat_data_t;
ext_game_robot_pos_t game_robot_pos_t;
ext_buff_t buff_musk_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_bullet_remaining_t bullet_remaining_t;
ext_rfid_status_t rfid_status_t;
ext_dart_client_cmd_t dart_client_cmd_t;
ext_ground_robot_position_t ground_robot_position_t;
ext_radar_mark_data_t radar_mark_data_t;
ext_sentry_info_t sentry_info_t;
ext_radar_info_t radar_info_t;

/* 0x0301: 机器人交互数据头。
 * 当前工程只缓存头部, 具体 payload 不在这里解析。 */
ext_student_interactive_header_data_t student_interactive_data_t;

/* 启动时清零全部裁判数据缓存, 确保未收到数据前状态确定。 */
void init_referee_struct_data(void)
{
    /* 帧头缓存 */
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    /* 基础比赛状态 */
    memset(&game_state, 0, sizeof(ext_game_status_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));

    /* 事件和判罚 */
    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));
    memset(&dart_remaining_time_t, 0, sizeof(ext_dart_remaining_time_t));

    /* 机器人本体与协同状态 */
    memset(&robot_state, 0, sizeof(ext_game_robot_status_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(ext_buff_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));
    memset(&rfid_status_t, 0, sizeof(ext_rfid_status_t));
    memset(&dart_client_cmd_t, 0, sizeof(ext_dart_client_cmd_t));
    memset(&ground_robot_position_t, 0, sizeof(ext_ground_robot_position_t));
    memset(&radar_mark_data_t, 0, sizeof(ext_radar_mark_data_t));
    memset(&sentry_info_t, 0, sizeof(ext_sentry_info_t));
    memset(&radar_info_t, 0, sizeof(ext_radar_info_t));

    /* 交互数据头 */
    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_header_data_t));
}

/* 解析一帧已经通过 CRC 校验的完整裁判协议包。 */
void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;
    uint8_t index = 0;

    /* 先拷贝 5 byte 帧头。 */
    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);

    /* 帧头后紧跟 2 byte cmd_id。 */
    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    /* 对于已经识别的命令字, 直接把 data 区覆盖写入对应缓存。
     * 结构体大小由协议契约文件中的 _Static_assert 保证。 */
    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(ext_game_status_t));
        }
            break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result));
        }
            break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
        }
            break;
        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, frame + index, sizeof(field_event));
        }
            break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
        }
            break;
        case DART_REMAINING_TIME_CMD_ID:
        {
            memcpy(&dart_remaining_time_t, frame + index, sizeof(ext_dart_remaining_time_t));
        }
            break;
        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index, sizeof(robot_state));
        }
            break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
        }
            break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
        }
            break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
        }
            break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
        }
            break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
        }
            break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
        }
            break;
        case RFID_STATUS_CMD_ID:
        {
            memcpy(&rfid_status_t, frame + index, sizeof(ext_rfid_status_t));
        }
            break;
        case DART_CLIENT_CMD_ID:
        {
            memcpy(&dart_client_cmd_t, frame + index, sizeof(ext_dart_client_cmd_t));
        }
            break;
        case GROUND_ROBOT_POSITION_CMD_ID:
        {
            memcpy(&ground_robot_position_t, frame + index, sizeof(ext_ground_robot_position_t));
        }
            break;
        case RADAR_MARK_DATA_CMD_ID:
        {
            memcpy(&radar_mark_data_t, frame + index, sizeof(ext_radar_mark_data_t));
        }
            break;
        case SENTRY_INFO_CMD_ID:
        {
            memcpy(&sentry_info_t, frame + index, sizeof(ext_sentry_info_t));
        }
            break;
        case RADAR_INFO_CMD_ID:
        {
            memcpy(&radar_info_t, frame + index, sizeof(ext_radar_info_t));
        }
            break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
        }
            break;
        default:
        {
            /* 当前工程未消费的命令字直接忽略。 */
            break;
        }
    }
}

/* 历史兼容接口:
 * 2026 协议已无实时底盘功率字段, 因此 power 返回 0;
 * buffer 返回真实缓冲能量。 */
void get_chassis_power_and_buffer(float *power, uint16_t *buffer)
{
    *power = 0.0f;
    *buffer = power_heat_data_t.buffer_energy;
}

/* 获取本机器人 ID 和默认客户端 ID。
 * 裁判系统客户端 ID 可由 robot_id + 256 得到。 */
void get_robot_id(uint16_t *robot_id, uint16_t *client_id)
{
    *robot_id = robot_state.robot_id;
    *client_id = robot_state.robot_id + 256;
}

/* 仅获取本机器人 ID。 */
void robot_id(uint16_t *robot_id)
{
    *robot_id = robot_state.robot_id;
}

/* 获取 17mm 发射机构热量上限和当前热量。 */
void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    *heat1_limit = robot_state.shooter_barrel_heat_limit;
    *heat1 = power_heat_data_t.shooter_17mm_barrel_heat;
}

/* 获取每秒热量冷却值。 */
void get_shoot_cooling_value1(uint16_t *cooling_value)
{
    *cooling_value = robot_state.shooter_barrel_cooling_value;
}

/* 获取 42mm 发射机构热量上限和当前热量。
 * 当前工程仍复用统一的 shooter_barrel_heat_limit。 */
void get_shoot_heat2_limit_and_heat2(uint16_t *heat2_limit, uint16_t *heat2)
{
    *heat2_limit = robot_state.shooter_barrel_heat_limit;
    *heat2 = power_heat_data_t.shooter_42mm_barrel_heat;
}

/* 获取裁判系统给出的底盘功率上限。 */
void get_chassis_power_limit(uint16_t *power_limit)
{
    *power_limit = robot_state.chassis_power_limit;
}

/* 获取最近一次伤害事件中的装甲 ID 和扣血原因。 */
void get_robot_hurt(uint8_t *armor_id, uint8_t *hurt_type)
{
    *armor_id = robot_hurt_t.armor_type;
    *hurt_type = robot_hurt_t.hurt_type;
}

/* 获取机器人等级。 */
void get_robot_level(uint8_t *level)
{
    *level = robot_state.robot_level;
}

/* 获取 17mm 允许发弹量。
 * 名字虽然叫 ammo, 但这里返回的是 allowance。 */
void get_ammo(uint16_t *bullet_remaining)
{
    *bullet_remaining = bullet_remaining_t.projectile_allowance_17mm;
}

/* 获取己方哨兵血量。 */
void get_sentry_HP(uint16_t *sentry_HP)
{
    *sentry_HP = game_robot_HP_t.ally_7_robot_HP;
}
