#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "main.h"

/*
 * RoboMaster 2026 裁判系统串口协议基础定义。
 *
 * 常规链路完整帧格式:
 *   frame_header(5 byte) + cmd_id(2 byte) + data(n byte) + crc16(2 byte)
 *
 * 其中 frame_header 的布局为:
 *   SOF(1) + data_length(2) + seq(1) + crc8(1)
 *
 * 本头文件负责三件事:
 * 1. 定义协议通用长度常量
 * 2. 定义本工程识别的命令字 ID
 * 3. 提供逐字节拆包所需的基础类型
 */

/* 帧起始字节, 协议固定为 0xA5。 */
#define HEADER_SOF 0xA5

/* 当前工程本地单包缓存上限。
 * 2026 常规链路单包总长不超过 127 byte, 因此 128 byte 足够容纳完整包。 */
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

/* 下列宏主要服务于 App_Referee_Task.c 的逐字节拆包逻辑。 */
#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

/* 当前工程识别到的裁判系统命令字。
 * 注释均对应 RoboMaster 2026 V1.3.0 协议文档中的功能说明。 */
typedef enum
{
    GAME_STATE_CMD_ID                   = 0x0001, /* 比赛状态数据 */
    GAME_RESULT_CMD_ID                  = 0x0002, /* 比赛结果数据 */
    GAME_ROBOT_HP_CMD_ID                = 0x0003, /* 己方机器人/前哨站/基地血量 */
    FIELD_EVENTS_CMD_ID                 = 0x0101, /* 场地事件数据 */
    REFEREE_WARNING_CMD_ID              = 0x0104, /* 裁判警告数据 */
    DART_REMAINING_TIME_CMD_ID          = 0x0105, /* 飞镖发射相关数据 */
    ROBOT_STATE_CMD_ID                  = 0x0201, /* 机器人状态数据 */
    POWER_HEAT_DATA_CMD_ID              = 0x0202, /* 缓冲能量和射击热量 */
    ROBOT_POS_CMD_ID                    = 0x0203, /* 本机器人位置 */
    BUFF_MUSK_CMD_ID                    = 0x0204, /* 机器人增益和能量反馈 */
    ROBOT_HURT_CMD_ID                   = 0x0206, /* 伤害状态 */
    SHOOT_DATA_CMD_ID                   = 0x0207, /* 实时射击数据 */
    BULLET_REMAINING_CMD_ID             = 0x0208, /* 允许发弹量和金币 */
    RFID_STATUS_CMD_ID                  = 0x0209, /* RFID 状态 */
    DART_CLIENT_CMD_ID                  = 0x020A, /* 飞镖选手端指令 */
    GROUND_ROBOT_POSITION_CMD_ID        = 0x020B, /* 地面机器人位置 */
    RADAR_MARK_DATA_CMD_ID              = 0x020C, /* 雷达标记进度 */
    SENTRY_INFO_CMD_ID                  = 0x020D, /* 哨兵自主决策信息 */
    RADAR_INFO_CMD_ID                   = 0x020E, /* 雷达自主决策信息 */
    STUDENT_INTERACTIVE_DATA_CMD_ID     = 0x0301, /* 机器人交互数据 */
    CUSTOM_CONTROLLER_DATA_CMD_ID       = 0x0302, /* 自定义控制器与机器人交互 */
    CLIENT_SIDE_MINIMAP_DATA_CMD_ID     = 0x0303, /* 小地图交互数据 */
    CUSTOMER_MINIMAP_INFORMATION_CMD_ID = 0x0305, /* 小地图接收雷达数据 */
    CUSTOM_CONTROLLER_TO_CLIENT_CMD_ID  = 0x0306, /* 自定义控制器与选手端交互 */
    CLIENT_MAP_PATH_CMD_ID              = 0x0307, /* 小地图接收路径 */
    CLIENT_MAP_ROBOT_DATA_CMD_ID        = 0x0308, /* 小地图接收机器人数据 */
    ROBOT_TO_CUSTOM_CONTROLLER_CMD_ID   = 0x0309, /* 机器人发给自定义控制器 */
    ROBOT_TO_CUSTOM_CLIENT_CMD_ID       = 0x0310, /* 机器人发给自定义客户端 */
    CUSTOM_CLIENT_TO_ROBOT_CMD_ID       = 0x0311, /* 自定义客户端发给机器人 */
    IDCustomData,                              /* 旧代码保留枚举值, 当前未使用 */
} referee_cmd_id_t;

/* 裁判协议帧头, 与协议文档中的 frame_header 一一对应。 */
typedef struct
{
    uint8_t SOF;          /* 固定 0xA5, 一帧起始标识 */
    uint16_t data_length; /* data 段长度, 不含 cmd_id 与 crc16 */
    uint8_t seq;          /* 包序号 */
    uint8_t CRC8;         /* 帧头 CRC8 */
} frame_header_struct_t;

/* 逐字节拆包状态机阶段。 */
typedef enum
{
    STEP_HEADER_SOF  = 0, /* 等待 SOF */
    STEP_LENGTH_LOW  = 1, /* 读取长度低字节 */
    STEP_LENGTH_HIGH = 2, /* 读取长度高字节 */
    STEP_FRAME_SEQ   = 3, /* 读取 seq */
    STEP_HEADER_CRC8 = 4, /* 读取并校验帧头 CRC8 */
    STEP_DATA_CRC16  = 5, /* 收集余下内容并校验整包 CRC16 */
} unpack_step_e;

/* 拆包上下文, 用于跨字节保存当前包的恢复状态。 */
typedef struct
{
    frame_header_struct_t *p_header;                  /* 预留帧头指针, 当前逻辑未显式使用 */
    uint16_t data_len;                                /* 当前帧 data 段长度 */
    uint8_t protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE]; /* 原始完整包缓冲区 */
    unpack_step_e unpack_step;                        /* 当前拆包阶段 */
    uint16_t index;                                   /* protocol_packet 当前写入下标 */
} unpack_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
